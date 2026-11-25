// SPDX-License-Identifier: GPL-2.0
/*
 * PSVR2 Linux driver — control character device and headset housekeeping.
 *
 * Sony's driver runs these from its status (IF7) reader loop; the reference
 * is docs/sony-re/headset.md part A sections 3.3, 4.1 and 5.1:
 *
 *   - Get 0xde clock sync every 500 ms, fitted into a device-us to
 *     CLOCK_MONOTONIC map that the SLAM, IMU and LED paths use.
 *   - Get 0x97 (see-through state) once a second, never within a second of
 *     a Set 0x17.
 *   - See-through follows the function button: a press while the headset is
 *     worn toggles it, taking the headset off forces it off.
 *   - Brightness follow-up: when status byte 10 differs from the configured
 *     brightness, Set 0x12 again.
 *   - Camera mode 0x10 (the SLAM tracking mode) re-sent once a second until
 *     the camera (type 10) and IMU (type 12) calibration have arrived on IF10.
 *
 * /dev/psvr2-ctrl lets userspace send the remaining reports (LED detector
 * config 0x14/0x09, relocaliser 0x0a, quit 0x27, restart 0x05, ...) and read
 * the clock map. Only reports Sony's driver uses are accepted.
 *
 * Copyright (C) 2026 PSVR2 Linux project
 */
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>

#include "psvr2.h"
#include "psvr2_protocol.h"
#include "psvr2_uapi.h"

static bool seethrough_button = true;
module_param(seethrough_button, bool, 0644);
MODULE_PARM_DESC(seethrough_button,
		 "Toggle see-through with the headset function button, as Sony's driver does (default: on)");

static bool brightness_follow = true;
module_param(brightness_follow, bool, 0644);
MODULE_PARM_DESC(brightness_follow,
		 "Re-send the brightness when the headset reports a different one (default: on)");

#define PSVR2_TICK_MS			500
#define PSVR2_CLOCK_RTT_MAX_US		1000	/* Sony +0x25c */
#define PSVR2_CLOCK_RESET_NS		2000000	/* 2000 us outlier: restart */
#define PSVR2_CLOCK_DRIFT_MAX_PPB	50000	/* Sony +0x248 */
#define PSVR2_CLOCK_SANITY_NS		(100 * NSEC_PER_MSEC)
#define PSVR2_TIME_REPLY_LEN		0x18
#define PSVR2_TIME_REPLY_DEVICE_US	4

struct psvr2_ctrl {
	struct kref		kref;
	struct mutex		lock;		/* guards psvr2 */
	struct psvr2_device	*psvr2;		/* NULL once IF7 is gone */
	struct miscdevice	miscdev;
};

/* ------------------------------------------------------------------ clock */

static void psvr2_clock_fit(struct psvr2_clock *ck)
{
	u64 sum_dev = 0, mean_dev;
	s64 sum_off = 0, mean_off, sxx = 0, sxy = 0, drift = 0;
	unsigned int i;

	for (i = 0; i < ck->count; i++) {
		sum_dev += ck->ring_dev[i];
		sum_off += ck->ring_off[i];
	}
	mean_dev = div_u64(sum_dev, ck->count);
	mean_off = div_s64(sum_off, ck->count);

	for (i = 0; i < ck->count; i++) {
		s64 dx = (s64)(ck->ring_dev[i] - mean_dev);
		s64 dy = ck->ring_off[i] - mean_off;

		sxx += dx * dx;
		sxy += dx * dy;
	}
	/* ns per us * 1e6 = ppb; scale sxx down rather than sxy up. */
	if (ck->count >= 2 && sxx >= 1000000)
		drift = div64_s64(sxy, div64_s64(sxx, 1000000));

	ck->ref_dev = mean_dev;
	ck->offset_ns = mean_off;
	ck->drift_ppb = clamp_t(s64, drift, -PSVR2_CLOCK_DRIFT_MAX_PPB,
				PSVR2_CLOCK_DRIFT_MAX_PPB);
	ck->valid = true;
}

/* Offset the current map predicts at extended device time ext. */
static s64 psvr2_clock_predict(const struct psvr2_clock *ck, u64 ext)
{
	return ck->offset_ns +
	       div_s64((s64)(ext - ck->ref_dev) * ck->drift_ppb, 1000000);
}

static int psvr2_clock_read(struct psvr2_device *psvr2, u32 *dev_us,
			    s64 *t0, s64 *t1)
{
	u8 reply[PSVR2_TIME_REPLY_LEN];
	int ret;

	*t0 = ktime_get_ns();
	ret = psvr2_control_get(psvr2, PSVR2_REPORT_GET_DEVICE_TIME, 1, reply,
				sizeof(reply));
	*t1 = ktime_get_ns();
	if (ret)
		return ret;
	*dev_us = get_unaligned_le32(reply + PSVR2_TIME_REPLY_DEVICE_US);
	return 0;
}

static void psvr2_clock_sample(struct psvr2_device *psvr2)
{
	struct psvr2_clock *ck = &psvr2->clock;
	unsigned long flags;
	s64 t0, t1, off;
	u32 dev, rtt;
	u64 ext;

	if (psvr2_clock_read(psvr2, &dev, &t0, &t1) || !dev)
		return;
	rtt = div_u64(t1 - t0, NSEC_PER_USEC);
	if (rtt >= PSVR2_CLOCK_RTT_MAX_US)
		return;

	spin_lock_irqsave(&ck->lock, flags);
	ext = ck->last_ext ? ck->last_ext + (s32)(dev - (u32)ck->last_ext) : dev;
	ck->last_ext = ext;
	off = t0 + (t1 - t0) / 2 - (s64)ext * NSEC_PER_USEC;

	if (ck->valid && abs(off - psvr2_clock_predict(ck, ext)) >
				 PSVR2_CLOCK_RESET_NS)
		ck->count = ck->head = 0;	/* jump: start a new window */

	ck->ring_dev[ck->head] = ext;
	ck->ring_off[ck->head] = off;
	ck->head = (ck->head + 1) % PSVR2_CLOCK_RING;
	if (ck->count < PSVR2_CLOCK_RING)
		ck->count++;
	psvr2_clock_fit(ck);
	ck->rtt_us = rtt;
	spin_unlock_irqrestore(&ck->lock, flags);
}

/*
 * Sony asks for the time three times at the first status read and treats a
 * firmware that answers 0 each time as not supporting PC use.
 */
static void psvr2_clock_probe(struct psvr2_device *psvr2)
{
	s64 t0, t1;
	u32 dev;
	int i;

	for (i = 0; i < 3; i++) {
		if (!psvr2_clock_read(psvr2, &dev, &t0, &t1) && dev) {
			psvr2->clock.supported = true;
			return;
		}
	}
	dev_warn(&psvr2->udev->dev,
		 "headset firmware does not report its clock (report 0xde); timestamps stay host receive times\n");
}

bool psvr2_clock_to_host(struct psvr2_device *psvr2, u32 dev_us, s64 *host_ns)
{
	struct psvr2_clock *ck = &psvr2->clock;
	unsigned long flags;
	bool ok = false;
	u64 ext;
	s64 t;

	spin_lock_irqsave(&ck->lock, flags);
	if (ck->valid) {
		ext = ck->ref_dev + (s32)(dev_us - (u32)ck->ref_dev);
		t = (s64)ext * NSEC_PER_USEC + psvr2_clock_predict(ck, ext);
		/* A capture time is in the recent past, never far off. */
		ok = abs(ktime_get_ns() - t) < PSVR2_CLOCK_SANITY_NS;
		if (ok)
			*host_ns = t;
	}
	spin_unlock_irqrestore(&ck->lock, flags);
	return ok;
}

static void psvr2_clock_get_map(struct psvr2_device *psvr2,
				struct psvr2_clock_map *map)
{
	struct psvr2_clock *ck = &psvr2->clock;
	unsigned long flags;

	memset(map, 0, sizeof(*map));
	spin_lock_irqsave(&ck->lock, flags);
	map->ref_device_us = ck->ref_dev;
	map->offset_ns = ck->offset_ns;
	map->drift_ppb = ck->drift_ppb;
	map->valid = ck->valid;
	map->samples = ck->count;
	map->rtt_us = ck->rtt_us;
	spin_unlock_irqrestore(&ck->lock, flags);
}

static void psvr2_clock_reset(struct psvr2_device *psvr2)
{
	unsigned long flags;

	spin_lock_irqsave(&psvr2->clock.lock, flags);
	psvr2->clock.valid = false;
	psvr2->clock.supported = false;
	psvr2->clock.count = psvr2->clock.head = 0;
	psvr2->clock.last_ext = 0;
	spin_unlock_irqrestore(&psvr2->clock.lock, flags);
}

/* -------------------------------------------------------- headset reports */

int psvr2_set_see_through(struct psvr2_device *psvr2, bool on)
{
	u8 cmd[2] = { on, PSVR2_SEE_THROUGH_ARG };
	int ret;

	ret = psvr2_control_set(psvr2, PSVR2_REPORT_SET_SEE_THROUGH, 1, cmd,
				sizeof(cmd));
	if (!ret)
		psvr2->see_through = on;
	psvr2->see_through_at = jiffies;
	return ret;
}

int psvr2_set_camera_mode(struct psvr2_device *psvr2, u32 mode)
{
	__le32 cmd[2] = { cpu_to_le32(1), cpu_to_le32(mode) };

	psvr2->camera_mode_at = jiffies;
	return psvr2_control_set(psvr2, PSVR2_REPORT_SET_CAMERA_MODE, 1, cmd,
				 sizeof(cmd));
}

static void psvr2_send_brightness(struct psvr2_device *psvr2)
{
	psvr2->brightness_at = jiffies;
	psvr2_control_set(psvr2, PSVR2_REPORT_SET_BRIGHTNESS, 1,
			  &psvr2->brightness, sizeof(psvr2->brightness));
}

static void psvr2_event_work(struct work_struct *work)
{
	struct psvr2_device *psvr2 = container_of(work, struct psvr2_device,
						  event_work);

	if (test_and_clear_bit(PSVR2_EV_SEE_THROUGH_TOGGLE, &psvr2->events))
		psvr2_set_see_through(psvr2, !psvr2->see_through);
	if (test_and_clear_bit(PSVR2_EV_SEE_THROUGH_OFF, &psvr2->events) &&
	    psvr2->see_through)
		psvr2_set_see_through(psvr2, false);
	if (test_and_clear_bit(PSVR2_EV_BRIGHTNESS, &psvr2->events) &&
	    psvr2->panel_brightness != psvr2->brightness &&
	    time_after(jiffies, psvr2->brightness_at + HZ))
		psvr2_send_brightness(psvr2);
}

/* Called from the status URB completion with the 32-byte header. */
void psvr2_ctrl_status(struct psvr2_device *psvr2, const u8 *hdr)
{
	bool worn = hdr[1] == 1;
	bool press = !psvr2->prev_button && hdr[2] == 1;
	bool kick = false;

	psvr2->prev_button = hdr[2];
	psvr2->worn = worn;
	psvr2->panel_brightness = hdr[10];
	if (!READ_ONCE(psvr2->ready))
		return;

	if (seethrough_button) {
		if (worn && press) {
			set_bit(PSVR2_EV_SEE_THROUGH_TOGGLE, &psvr2->events);
			kick = true;
		} else if (!worn && psvr2->see_through) {
			set_bit(PSVR2_EV_SEE_THROUGH_OFF, &psvr2->events);
			kick = true;
		}
	}
	if (brightness_follow && hdr[10] != psvr2->brightness &&
	    time_after(jiffies, psvr2->brightness_at + HZ)) {
		set_bit(PSVR2_EV_BRIGHTNESS, &psvr2->events);
		kick = true;
	}
	if (kick)
		schedule_work(&psvr2->event_work);
}

static void psvr2_tick_work(struct work_struct *work)
{
	struct psvr2_device *psvr2 = container_of(to_delayed_work(work),
						  struct psvr2_device,
						  tick_work);
	const unsigned long need = BIT(PSVR2_CALIB_CAMERA0) | BIT(PSVR2_CALIB_IMU);
	u8 state;

	if (psvr2->clock.supported)
		psvr2_clock_sample(psvr2);

	if (!psvr2->camera_v4l2 && psvr2->aux[2] &&
	    (READ_ONCE(psvr2->calib_seen) & need) != need &&
	    time_after(jiffies, psvr2->camera_mode_at + HZ))
		psvr2_set_camera_mode(psvr2, psvr2->camera_mode);

	if (time_after(jiffies, psvr2->see_through_at + HZ)) {
		psvr2->see_through_at = jiffies;
		if (!psvr2_control_get(psvr2, PSVR2_REPORT_GET_SEE_THROUGH, 1,
				       &state, sizeof(state)))
			psvr2->see_through = state != 0;
	}

	schedule_delayed_work(&psvr2->tick_work,
			      msecs_to_jiffies(PSVR2_TICK_MS));
}

/* ----------------------------------------------------------- char device */

/* Reports Sony's driver sends (docs/sony-re/headset.md §A.2.3). */
static bool psvr2_ctrl_allowed(bool get, u16 id)
{
	static const u8 sets[] = { 0x05, 0x08, 0x09, 0x0a, 0x0b, 0x12, 0x14,
				   0x17, 0x1f, 0x22, 0x25, 0x27, 0x28, 0x29 };
	static const u8 gets[] = { 0x81, 0x8a, 0x8f, 0x97, 0xa1, 0xa4, 0xa5,
				   0xa6, 0xa9, 0xde };
	const u8 *list = get ? gets : sets;
	size_t i, n = get ? sizeof(gets) : sizeof(sets);

	for (i = 0; i < n; i++)
		if (list[i] == id)
			return true;
	return false;
}

static void psvr2_ctrl_free(struct kref *kref)
{
	kfree(container_of(kref, struct psvr2_ctrl, kref));
}

static int psvr2_ctrl_open(struct inode *inode, struct file *file)
{
	struct psvr2_ctrl *ctrl = container_of(file->private_data,
					       struct psvr2_ctrl, miscdev);

	kref_get(&ctrl->kref);
	file->private_data = ctrl;
	return nonseekable_open(inode, file);
}

static int psvr2_ctrl_release(struct inode *inode, struct file *file)
{
	struct psvr2_ctrl *ctrl = file->private_data;

	kref_put(&ctrl->kref, psvr2_ctrl_free);
	return 0;
}

/* Keep the driver's cached state in step with what userspace sends. */
static void psvr2_ctrl_track(struct psvr2_device *psvr2,
			     const struct psvr2_ctrl_report *r)
{
	switch (r->report_id) {
	case PSVR2_REPORT_SET_SEE_THROUGH:
		if (r->len >= 1)
			psvr2->see_through = r->data[0] != 0;
		psvr2->see_through_at = jiffies;
		break;
	case PSVR2_REPORT_SET_BRIGHTNESS:
		if (r->len >= 1)
			psvr2->brightness = min_t(u8, r->data[0], 31);
		psvr2->brightness_at = jiffies;
		break;
	case PSVR2_REPORT_SET_CAMERA_MODE:
		if (r->len >= 8 && !psvr2->camera_v4l2)
			psvr2->camera_mode = get_unaligned_le32(r->data + 4);
		psvr2->camera_mode_at = jiffies;
		break;
	}
}

static long psvr2_ctrl_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	struct psvr2_ctrl *ctrl = file->private_data;
	void __user *uarg = (void __user *)arg;
	struct psvr2_ctrl_report *r = NULL;
	struct psvr2_clock_map map;
	bool get = cmd == PSVR2_IOC_GET;
	long ret;

	if (cmd != PSVR2_IOC_SET && cmd != PSVR2_IOC_GET &&
	    cmd != PSVR2_IOC_CLOCK)
		return -ENOTTY;

	if (cmd != PSVR2_IOC_CLOCK) {
		r = memdup_user(uarg, sizeof(*r));
		if (IS_ERR(r))
			return PTR_ERR(r);
		if (r->len > PSVR2_CTRL_MAX_LEN) {
			ret = -EINVAL;
			goto out_free;
		}
		if (!psvr2_ctrl_allowed(get, r->report_id)) {
			ret = -EPERM;
			goto out_free;
		}
	}

	mutex_lock(&ctrl->lock);
	if (!ctrl->psvr2) {
		ret = -ENODEV;
	} else if (cmd == PSVR2_IOC_CLOCK) {
		psvr2_clock_get_map(ctrl->psvr2, &map);
		ret = copy_to_user(uarg, &map, sizeof(map)) ? -EFAULT : 0;
	} else if (get) {
		ret = psvr2_control_get(ctrl->psvr2, r->report_id, r->subcmd,
					r->data, r->len);
		if (!ret && copy_to_user(uarg, r, sizeof(*r)))
			ret = -EFAULT;
	} else {
		ret = psvr2_control_set(ctrl->psvr2, r->report_id, r->subcmd,
					r->data, r->len);
		if (!ret)
			psvr2_ctrl_track(ctrl->psvr2, r);
	}
	mutex_unlock(&ctrl->lock);

out_free:
	kfree(r);
	return ret;
}

static const struct file_operations psvr2_ctrl_fops = {
	.owner		= THIS_MODULE,
	.open		= psvr2_ctrl_open,
	.release	= psvr2_ctrl_release,
	.unlocked_ioctl	= psvr2_ctrl_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
};

/* ------------------------------------------------------------- lifecycle */

/* Once per shared context, from psvr2_device_get(). */
void psvr2_ctrl_init(struct psvr2_device *psvr2)
{
	INIT_DELAYED_WORK(&psvr2->tick_work, psvr2_tick_work);
	INIT_WORK(&psvr2->event_work, psvr2_event_work);
	spin_lock_init(&psvr2->clock.lock);
	spin_lock_init(&psvr2->calib_lock);
	psvr2->camera_mode = PSVR2_CAMERA_MODE_TRACKING;
}

/* IF7 bind: create /dev/psvr2-ctrl. */
int psvr2_ctrl_start(struct psvr2_device *psvr2, struct device *parent)
{
	struct psvr2_ctrl *ctrl;
	int ret;

	ctrl = kzalloc(sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;
	kref_init(&ctrl->kref);
	mutex_init(&ctrl->lock);
	ctrl->psvr2 = psvr2;
	ctrl->miscdev.minor = MISC_DYNAMIC_MINOR;
	ctrl->miscdev.name = "psvr2-ctrl";
	ctrl->miscdev.fops = &psvr2_ctrl_fops;
	ret = misc_register(&ctrl->miscdev);
	if (ret) {
		dev_err(parent, "failed to register /dev/psvr2-ctrl: %d\n", ret);
		kfree(ctrl);
		return ret;
	}
	psvr2->ctrl = ctrl;
	return 0;
}

/* End of the init sequence on SuperSpeed: start the steady-state loop. */
void psvr2_ctrl_ready(struct psvr2_device *psvr2)
{
	psvr2_clock_probe(psvr2);
	psvr2->see_through_at = jiffies;
	WRITE_ONCE(psvr2->ready, true);
	schedule_delayed_work(&psvr2->tick_work, 0);
}

/* IF7 unbind. */
void psvr2_ctrl_stop(struct psvr2_device *psvr2)
{
	struct psvr2_ctrl *ctrl = psvr2->ctrl;

	WRITE_ONCE(psvr2->ready, false);
	cancel_delayed_work_sync(&psvr2->tick_work);
	cancel_work_sync(&psvr2->event_work);
	psvr2_clock_reset(psvr2);

	if (!ctrl)
		return;
	psvr2->ctrl = NULL;
	misc_deregister(&ctrl->miscdev);
	mutex_lock(&ctrl->lock);
	ctrl->psvr2 = NULL;
	mutex_unlock(&ctrl->lock);
	kref_put(&ctrl->kref, psvr2_ctrl_free);
}
