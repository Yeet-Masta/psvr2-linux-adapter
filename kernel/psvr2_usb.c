// SPDX-License-Identifier: GPL-2.0
/*
 * PSVR2 Linux driver — USB core, shared device context and ep0 control.
 *
 * Binds the vendor interfaces of the Sony PSVR2 PC adapter (USB 054c:0cde).
 * Milestone 1 binds only the status/IMU interface (IF7); the per-device
 * registry and reference counting are in place so the remaining stream
 * interfaces can be added without restructuring.
 *
 * Copyright (C) 2026 PSVR2 Linux project
 */
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/usb.h>
#include <linux/unaligned.h>

#include "psvr2.h"
#include "psvr2_protocol.h"

/* Registry of live per-headset contexts, keyed by struct usb_device. */
static LIST_HEAD(psvr2_devices);
static DEFINE_MUTEX(psvr2_registry_lock);

static void psvr2_init_work(struct work_struct *work);

static void psvr2_device_release(struct kref *kref)
	__releases(&psvr2_registry_lock)
{
	struct psvr2_device *psvr2 = container_of(kref, struct psvr2_device, kref);
	int i;

	list_del(&psvr2->node);
	mutex_unlock(&psvr2_registry_lock);

	debugfs_remove_recursive(psvr2->debugfs_dir);
	usb_put_dev(psvr2->udev);
	mutex_destroy(&psvr2->ctrl_lock);
	for (i = 0; i < PSVR2_CALIB_COUNT; i++)
		kfree(psvr2->calib[i]);
	kfree(psvr2);
}

struct psvr2_device *psvr2_device_get(struct usb_device *udev)
{
	struct psvr2_device *psvr2;

	mutex_lock(&psvr2_registry_lock);
	list_for_each_entry(psvr2, &psvr2_devices, node) {
		if (psvr2->udev == udev && kref_get_unless_zero(&psvr2->kref)) {
			mutex_unlock(&psvr2_registry_lock);
			return psvr2;
		}
	}

	psvr2 = kzalloc(sizeof(*psvr2), GFP_KERNEL);
	if (!psvr2) {
		mutex_unlock(&psvr2_registry_lock);
		return NULL;
	}

	kref_init(&psvr2->kref);
	mutex_init(&psvr2->ctrl_lock);
	psvr2->udev = usb_get_dev(udev);
	psvr2->brightness = 31;
	INIT_DELAYED_WORK(&psvr2->init_work, psvr2_init_work);
	psvr2_ctrl_init(psvr2);
	psvr2->debugfs_dir = debugfs_create_dir("psvr2", NULL);
	list_add(&psvr2->node, &psvr2_devices);
	mutex_unlock(&psvr2_registry_lock);

	return psvr2;
}

void psvr2_device_put(struct psvr2_device *psvr2)
{
	if (psvr2)
		kref_put_mutex(&psvr2->kref, psvr2_device_release,
			       &psvr2_registry_lock);
}

/*
 * ep0 vendor control. The headset expects a sie_ctrl_pkt (8-byte header +
 * payload). No host authentication is required. Sony's driver uses WinUSB's
 * default (long) timeout; some reports, such as 0x81 right after plug-in or
 * the EDID switch, can take well over 100 ms.
 */
#define PSVR2_CTRL_TIMEOUT_MS	1000

static int psvr2_control(struct psvr2_device *psvr2, bool in, u16 report_id,
			 u16 subcmd, void *data, u32 len)
{
	struct sie_ctrl_pkt *pkt;
	int ret;

	if (len > PSVR2_CTRL_DATA_MAX)
		return -EINVAL;

	pkt = kzalloc(sizeof(*pkt), GFP_KERNEL);
	if (!pkt)
		return -ENOMEM;

	pkt->report_id = cpu_to_le16(report_id);
	pkt->subcmd = cpu_to_le16(subcmd);
	pkt->len = cpu_to_le32(len);

	mutex_lock(&psvr2->ctrl_lock);
	if (in) {
		/* Replies may be shorter than asked; the rest stays zero. */
		ret = usb_control_msg(psvr2->udev,
				      usb_rcvctrlpipe(psvr2->udev, 0), 0x01,
				      USB_DIR_IN | USB_TYPE_VENDOR |
					      USB_RECIP_ENDPOINT,
				      report_id, 0, pkt, len + 8,
				      PSVR2_CTRL_TIMEOUT_MS);
		if (ret >= 0) {
			if (len)
				memcpy(data, pkt->data, len);
			ret = 0;
		}
	} else {
		if (len)
			memcpy(pkt->data, data, len);
		ret = usb_control_msg_send(psvr2->udev, 0, 0x09,
					   USB_DIR_OUT | USB_TYPE_VENDOR |
						   USB_RECIP_ENDPOINT,
					   0, 0, pkt, len + 8,
					   PSVR2_CTRL_TIMEOUT_MS, GFP_KERNEL);
	}
	mutex_unlock(&psvr2->ctrl_lock);

	kfree(pkt);
	return ret;
}

int psvr2_control_set(struct psvr2_device *psvr2, u16 report_id, u16 subcmd,
		      const void *data, u32 len)
{
	return psvr2_control(psvr2, false, report_id, subcmd, (void *)data, len);
}

int psvr2_control_get(struct psvr2_device *psvr2, u16 report_id, u16 subcmd,
		      void *data, u32 len)
{
	return psvr2_control(psvr2, true, report_id, subcmd, data, len);
}

/* sysfs: panel brightness, a single byte 0..31 (report 0x12, subcmd 1). */
static ssize_t brightness_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct psvr2_device *psvr2 = usb_get_intfdata(to_usb_interface(dev));

	return sysfs_emit(buf, "%u\n", psvr2->brightness);
}

static ssize_t brightness_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct psvr2_device *psvr2 = usb_get_intfdata(to_usb_interface(dev));
	u8 value;
	int ret;

	ret = kstrtou8(buf, 0, &value);
	if (ret)
		return ret;
	value = min_t(u8, value, 31);

	ret = psvr2_control_set(psvr2, PSVR2_REPORT_SET_BRIGHTNESS, 1, &value,
				sizeof(value));
	if (ret)
		return ret;

	psvr2->brightness = value;
	return count;
}
static DEVICE_ATTR_RW(brightness);

/*
 * sysfs: headset rumble motor, a single byte 0..25 (vibration frequency in Hz,
 * 0 = off) on report 0x08 subcmd 1, as PSVR2 Toolkit sends it. Stock firmware
 * ignores it on PC; the headset must be jailbroken (vr2jb) for the motor to run.
 */
static ssize_t rumble_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct psvr2_device *psvr2 = usb_get_intfdata(to_usb_interface(dev));

	return sysfs_emit(buf, "%u\n", psvr2->rumble);
}

static ssize_t rumble_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct psvr2_device *psvr2 = usb_get_intfdata(to_usb_interface(dev));
	u8 value;
	int ret;

	ret = kstrtou8(buf, 0, &value);
	if (ret)
		return ret;
	value = min_t(u8, value, PSVR2_RUMBLE_MAX_HZ);

	ret = psvr2_control_set(psvr2, PSVR2_REPORT_SET_PERIPHERAL,
				PSVR2_SET_PERIPHERAL_SUBCMD_MOTOR, &value,
				sizeof(value));
	if (ret)
		return ret;

	psvr2->rumble = value;
	return count;
}
static DEVICE_ATTR_RW(rumble);

/*
 * sysfs: the per-unit lens calibration block (report 0x8f subcmd 1) as hex.
 * Layout (Monado psvr2_setup_distortion_and_fovs): u8 version, 7 bytes
 * unknown, then 32 little-endian float32 parameters. Userspace (libpsvr2)
 * turns it into the distortion model's per-eye offsets and rotation.
 */
static ssize_t lens_calibration_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct psvr2_device *psvr2 = usb_get_intfdata(to_usb_interface(dev));
	int i, ret;
	u8 *cal;

	cal = kzalloc(PSVR2_LENS_CAL_LEN, GFP_KERNEL);
	if (!cal)
		return -ENOMEM;
	ret = psvr2_control_get(psvr2, PSVR2_REPORT_GET_LENS_CAL, 1, cal,
				PSVR2_LENS_CAL_LEN);
	if (!ret) {
		/* %*ph prints at most 64 bytes per conversion. */
		for (i = 0; i < PSVR2_LENS_CAL_LEN; i += 64)
			ret += sysfs_emit_at(buf, ret, "%*phN",
					     min(64, PSVR2_LENS_CAL_LEN - i),
					     cal + i);
		ret += sysfs_emit_at(buf, ret, "\n");
	}
	kfree(cal);
	return ret;
}
static DEVICE_ATTR_RO(lens_calibration);

/* sysfs: values read by the init sequence (hex, 0 until it has run). */
static ssize_t firmware_version_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct psvr2_device *psvr2 = usb_get_intfdata(to_usb_interface(dev));

	return sysfs_emit(buf, "%08x\n", psvr2->fw_version);
}
static DEVICE_ATTR_RO(firmware_version);

static ssize_t board_stage_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct psvr2_device *psvr2 = usb_get_intfdata(to_usb_interface(dev));

	return sysfs_emit(buf, "%u\n", psvr2->board_stage);
}
static DEVICE_ATTR_RO(board_stage);

static ssize_t min_driver_version_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct psvr2_device *psvr2 = usb_get_intfdata(to_usb_interface(dev));

	return sysfs_emit(buf, "%08x\n", psvr2->min_driver);
}
static DEVICE_ATTR_RO(min_driver_version);

/* sysfs: headset see-through (Get 0x97 / Set 0x17), 0 or 1. */
static ssize_t see_through_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct psvr2_device *psvr2 = usb_get_intfdata(to_usb_interface(dev));
	u8 state = 0;
	int ret;

	/* The housekeeping loop reads 0x97 once a second; use that when it runs. */
	if (READ_ONCE(psvr2->ready))
		return sysfs_emit(buf, "%u\n", psvr2->see_through ? 1 : 0);
	ret = psvr2_control_get(psvr2, PSVR2_REPORT_GET_SEE_THROUGH, 1, &state,
				sizeof(state));
	if (ret)
		return ret;
	return sysfs_emit(buf, "%u\n", state ? 1 : 0);
}

static ssize_t see_through_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct psvr2_device *psvr2 = usb_get_intfdata(to_usb_interface(dev));
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;
	ret = psvr2_set_see_through(psvr2, on);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(see_through);

/*
 * sysfs: EDID type (Get 0xa9 / Set 0x29). 0 = 90/120 Hz EDID, 1 = 90 Hz only.
 * After a change the headset re-enumerates its DisplayPort sink; wait about
 * a second before using the display (Sony sleeps 1000 ms).
 */
static ssize_t edid_type_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct psvr2_device *psvr2 = usb_get_intfdata(to_usb_interface(dev));
	u8 type = 0;
	int ret;

	ret = psvr2_control_get(psvr2, PSVR2_REPORT_GET_EDID_TYPE, 1, &type,
				sizeof(type));
	if (ret)
		return ret;
	return sysfs_emit(buf, "%u\n", type);
}

static ssize_t edid_type_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct psvr2_device *psvr2 = usb_get_intfdata(to_usb_interface(dev));
	u8 type;
	int ret;

	ret = kstrtou8(buf, 0, &type);
	if (ret)
		return ret;
	if (type != PSVR2_EDID_90_120HZ && type != PSVR2_EDID_90HZ)
		return -EINVAL;
	ret = psvr2_control_set(psvr2, PSVR2_REPORT_SET_EDID_TYPE, 1, &type,
				sizeof(type));
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(edid_type);

/* sysfs: 1 once the init sequence has finished on a SuperSpeed link. */
static ssize_t ready_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct psvr2_device *psvr2 = usb_get_intfdata(to_usb_interface(dev));

	return sysfs_emit(buf, "%u\n", READ_ONCE(psvr2->ready) &&
			  psvr2->board_stage && psvr2->board_stage != ~0u);
}
static DEVICE_ATTR_RO(ready);

/*
 * sysfs: factory calibration blobs from the IF10 "VD" stream, raw. Empty
 * until the headset has sent them (it does once camera mode 0x10 is on).
 */
static ssize_t psvr2_calib_read(struct psvr2_device *psvr2, int slot,
				char *buf, loff_t off, size_t count)
{
	unsigned long flags;
	ssize_t ret = 0;

	spin_lock_irqsave(&psvr2->calib_lock, flags);
	if (psvr2->calib[slot] && off < psvr2->calib_len[slot]) {
		ret = min_t(size_t, count, psvr2->calib_len[slot] - off);
		memcpy(buf, psvr2->calib[slot] + off, ret);
	}
	spin_unlock_irqrestore(&psvr2->calib_lock, flags);
	return ret;
}

#define PSVR2_CALIB_ATTR(name, slot)					\
static ssize_t name##_read(struct file *f, struct kobject *kobj,	\
			   const struct bin_attribute *attr, char *buf,	\
			   loff_t off, size_t count)			\
{									\
	struct device *dev = kobj_to_dev(kobj);				\
									\
	return psvr2_calib_read(usb_get_intfdata(to_usb_interface(dev)),\
				slot, buf, off, count);			\
}									\
static const BIN_ATTR_RO(name, PSVR2_CALIB_MAX_LEN)

PSVR2_CALIB_ATTR(camera_calib0, PSVR2_CALIB_CAMERA0);
PSVR2_CALIB_ATTR(camera_calib1, PSVR2_CALIB_CAMERA1);
PSVR2_CALIB_ATTR(imu_calib, PSVR2_CALIB_IMU);
PSVR2_CALIB_ATTR(imu_set_calib, PSVR2_CALIB_IMU_SET);

static const struct bin_attribute *const psvr2_bin_attrs[] = {
	&bin_attr_camera_calib0,
	&bin_attr_camera_calib1,
	&bin_attr_imu_calib,
	&bin_attr_imu_set_calib,
	NULL,
};

static struct attribute *psvr2_attrs[] = {
	&dev_attr_ready.attr,
	&dev_attr_brightness.attr,
	&dev_attr_rumble.attr,
	&dev_attr_lens_calibration.attr,
	&dev_attr_firmware_version.attr,
	&dev_attr_board_stage.attr,
	&dev_attr_min_driver_version.attr,
	&dev_attr_see_through.attr,
	&dev_attr_edid_type.attr,
	NULL,
};
static const struct attribute_group psvr2_group = {
	.attrs		= psvr2_attrs,
	.bin_attrs	= psvr2_bin_attrs,
};

static const struct attribute_group *psvr2_groups[] = {
	&psvr2_group,
	NULL,
};

/*
 * Headset init, as Sony's driver does it when IF7 connects
 * (docs/sony-re/headset.md §A.3.2). It runs from a work item one second
 * after the bind so probe does not block:
 * Sony uses subcmd 1 for every report here.
 *   Get 0x81 firmware info (once more a second later if the board stage is
 *   still 0), Get 0xa1, then on SuperSpeed only: Set 0x17 see-through off,
 *   Set 0x22 0, Set 0x12 brightness.
 * Sony also selects IF7 alt 1 between 0x22 and 0x12; this driver does that in
 * psvr2_status_start() because the status URBs need it. Failures are logged
 * and init carries on, so a headset that rejects one report still streams.
 */
static void psvr2_init_work(struct work_struct *work)
{
	struct psvr2_device *psvr2 = container_of(to_delayed_work(work),
						  struct psvr2_device,
						  init_work);
	struct device *dev = &psvr2->udev->dev;
	u8 *info;
	__le32 min_driver = 0;
	u8 zero = 0;
	int ret, attempt;

	info = kzalloc(PSVR2_FW_INFO_LEN, GFP_KERNEL);
	if (!info)
		return;
	for (attempt = 0; attempt < 2; attempt++) {
		if (attempt)
			msleep(1000);
		ret = psvr2_control_get(psvr2, PSVR2_REPORT_GET_FW_INFO, 1, info,
					PSVR2_FW_INFO_LEN);
		if (ret)
			continue;
		psvr2->fw_version = get_unaligned_le32(info + PSVR2_FW_INFO_VERSION);
		psvr2->board_stage =
			get_unaligned_le32(info + PSVR2_FW_INFO_BOARD_STAGE);
		if (psvr2->board_stage)
			break;
	}
	kfree(info);
	if (ret || !psvr2->board_stage)
		dev_warn(dev, "failed to read PS VR2 firmware info (%d)\n", ret);
	else
		dev_info(dev, "firmware %08x, board stage %u\n",
			 psvr2->fw_version, psvr2->board_stage);

	if (!psvr2_control_get(psvr2, PSVR2_REPORT_GET_MIN_DRIVER, 1,
			       &min_driver, sizeof(min_driver)))
		psvr2->min_driver = le32_to_cpu(min_driver);

	if (psvr2->udev->speed < USB_SPEED_SUPER)
		return;

	ret = psvr2_set_see_through(psvr2, false);
	if (ret)
		dev_warn(dev, "see-through off failed: %d\n", ret);
	ret = psvr2_control_set(psvr2, PSVR2_REPORT_SET_HOST_MODE, 1, &zero,
				sizeof(zero));
	if (ret)
		dev_warn(dev, "report 0x22 failed: %d\n", ret);
	ret = psvr2_control_set(psvr2, PSVR2_REPORT_SET_BRIGHTNESS, 1,
				&psvr2->brightness, sizeof(psvr2->brightness));
	if (ret)
		dev_warn(dev, "brightness failed: %d\n", ret);
	psvr2->brightness_at = jiffies;

	psvr2_ctrl_ready(psvr2);
}

/*
 * Set 0x1f, 256 bytes, sent by Sony's driver each time the SLAM interface
 * (IF3) connects, before poses are read. Values are the raw constants from
 * Sony's driver (docs/sony-re/headset.md §A.2.6); float32 fields are
 * kept as IEEE-754 bit patterns because the kernel does not use the FPU.
 * Their meaning is not known; the radians and metre-sized values suggest
 * SLAM thresholds.
 */
static void psvr2_send_slam_config(struct psvr2_device *psvr2,
				   struct usb_interface *intf)
{
	static const u32 head[4] = {
		0x40800000,	/* 4.0 */
		0x3d567750,	/* 0.05236 (3 degrees) */
		0xbeb33333,	/* -0.35 */
		0x3eb33333,	/* 0.35 */
	};
	static const u32 tail[4] = {
		0x3f800000,	/* 1.0 */
		0x3fc00000,	/* 1.5 */
		0x3d8efa35,	/* 0.06981 (4 degrees) */
		0x3d2b92a6,	/* 0.04189 (2.4 degrees) */
	};
	u8 *cfg;
	int i, ret;

	cfg = kzalloc(PSVR2_SLAM_CONFIG_LEN, GFP_KERNEL);
	if (!cfg)
		return;
	put_unaligned_le16(40, cfg);
	for (i = 0; i < 4; i++)
		put_unaligned_le32(head[i], cfg + 0x02 + 4 * i);
	put_unaligned_le32(10, cfg + 0x12);
	put_unaligned_le32(5, cfg + 0x16);
	for (i = 0; i < 4; i++)
		put_unaligned_le32(tail[i], cfg + 0x1a + 4 * i);

	ret = psvr2_control_set(psvr2, PSVR2_REPORT_SET_SLAM_CONFIG, 1, cfg,
				PSVR2_SLAM_CONFIG_LEN);
	if (ret)
		dev_warn(&intf->dev, "SLAM config (report 0x1f) failed: %d\n", ret);
	kfree(cfg);
}

/*
 * Bring up the status/IMU interface (IF7): the IIO and input devices are
 * devm-managed against &intf->dev, so they unwind automatically on probe
 * failure or disconnect; only the status URBs need explicit teardown.
 */
static int psvr2_probe_status(struct psvr2_device *psvr2,
			      struct usb_interface *intf)
{
	int ret;

	ret = psvr2_imu_register(psvr2, &intf->dev);
	if (ret)
		return ret;

	ret = psvr2_input_register(psvr2, &intf->dev);
	if (ret)
		return ret;

	ret = psvr2_ctrl_start(psvr2, &intf->dev);
	if (ret)
		return ret;

	ret = psvr2_status_start(psvr2, intf);
	if (ret) {
		psvr2_ctrl_stop(psvr2);
		return ret;
	}

	schedule_delayed_work(&psvr2->init_work, msecs_to_jiffies(1000));
	return 0;
}

static int psvr2_probe(struct usb_interface *intf,
		       const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	u8 ifnum = intf->cur_altsetting->desc.bInterfaceNumber;
	struct psvr2_device *psvr2;
	int ret;

	psvr2 = psvr2_device_get(udev);
	if (!psvr2)
		return -ENOMEM;

	usb_set_intfdata(intf, psvr2);

	/*
	 * A bad link makes the headset fall back to full/high speed, where the
	 * status interface has no endpoints (no IMU or controls) and the
	 * cameras cannot stream. Say so once instead of failing obscurely.
	 */
	if (ifnum == PSVR2_IF_STATUS && udev->speed < USB_SPEED_SUPER)
		dev_warn(&intf->dev,
			 "headset enumerated at %s, not SuperSpeed: replug the PC adapter's USB cable into a USB 3 port\n",
			 usb_speed_string(udev->speed));

	switch (ifnum) {
	case PSVR2_IF_STATUS:
		ret = psvr2_probe_status(psvr2, intf);
		break;
	case PSVR2_IF_SLAM:
		psvr2_send_slam_config(psvr2, intf);
		ret = psvr2_slam_start(psvr2, intf);
		break;
	case PSVR2_IF_CAMERA:
		ret = psvr2_camera_start(psvr2, intf);
		break;
	case PSVR2_IF_GAZE:
		ret = psvr2_gaze_start(psvr2, intf);
		break;
	case PSVR2_IF_LD:
	case PSVR2_IF_RP:
	case PSVR2_IF_VD:
	case PSVR2_IF_LOG:
		ret = psvr2_aux_start(psvr2, intf);
		break;
	default:
		ret = -ENODEV;
		break;
	}
	if (ret)
		goto err_free;

	dev_info(&intf->dev, "PSVR2 interface %u ready\n", ifnum);
	return 0;

err_free:
	usb_set_intfdata(intf, NULL);
	psvr2_device_put(psvr2);
	return ret;
}

static void psvr2_disconnect(struct usb_interface *intf)
{
	struct psvr2_device *psvr2 = usb_get_intfdata(intf);
	u8 ifnum = intf->cur_altsetting->desc.bInterfaceNumber;

	if (!psvr2)
		return;

	switch (ifnum) {
	case PSVR2_IF_STATUS:
		cancel_delayed_work_sync(&psvr2->init_work);
		psvr2_ctrl_stop(psvr2);
		psvr2_status_stop(psvr2);
		break;
	case PSVR2_IF_SLAM:
		psvr2_slam_stop(psvr2);
		break;
	case PSVR2_IF_CAMERA:
		psvr2_camera_stop(psvr2);
		break;
	case PSVR2_IF_GAZE:
		psvr2_gaze_stop(psvr2);
		break;
	case PSVR2_IF_LD:
	case PSVR2_IF_RP:
	case PSVR2_IF_VD:
	case PSVR2_IF_LOG:
		psvr2_aux_stop(psvr2, intf);
		break;
	}

	usb_set_intfdata(intf, NULL);
	psvr2_device_put(psvr2);

	dev_info(&intf->dev, "PSVR2 interface %u removed\n", ifnum);
}

/*
 * System resume. Sony's driver sends nothing on suspend and, after resume,
 * Set 0x05 subcmd 2 (headset restart; its default "sleep action" 2,
 * docs/sony-re/headset.md §A.3.6). This driver has no suspend
 * callbacks, so usbcore rebinds it across a system sleep; the notifier only
 * adds Sony's restart once the system is back.
 */
static void psvr2_restart_work(struct work_struct *work)
{
	struct psvr2_device *psvr2;
	int ret;

	mutex_lock(&psvr2_registry_lock);
	list_for_each_entry(psvr2, &psvr2_devices, node) {
		if (!READ_ONCE(psvr2->ready))
			continue;
		ret = psvr2_control_set(psvr2, PSVR2_REPORT_SET_RESTART, 2,
					NULL, 0);
		dev_info(&psvr2->udev->dev, "resume: headset restart (0x05) %d\n",
			 ret);
	}
	mutex_unlock(&psvr2_registry_lock);
}
static DECLARE_DELAYED_WORK(psvr2_restart, psvr2_restart_work);

static int psvr2_pm_notify(struct notifier_block *nb, unsigned long action,
			   void *data)
{
	/* Give usbcore time to rebind and the init sequence to run. */
	if (action == PM_POST_SUSPEND)
		schedule_delayed_work(&psvr2_restart, msecs_to_jiffies(3000));
	return NOTIFY_DONE;
}

static struct notifier_block psvr2_pm_nb = {
	.notifier_call = psvr2_pm_notify,
};

static const struct usb_device_id psvr2_id_table[] = {
	{ USB_DEVICE_INTERFACE_NUMBER(PSVR2_VENDOR_ID, PSVR2_PRODUCT_ID,
				      PSVR2_IF_STATUS) },
	{ USB_DEVICE_INTERFACE_NUMBER(PSVR2_VENDOR_ID, PSVR2_PRODUCT_ID,
				      PSVR2_IF_SLAM) },
	{ USB_DEVICE_INTERFACE_NUMBER(PSVR2_VENDOR_ID, PSVR2_PRODUCT_ID,
				      PSVR2_IF_CAMERA) },
	{ USB_DEVICE_INTERFACE_NUMBER(PSVR2_VENDOR_ID, PSVR2_PRODUCT_ID,
				      PSVR2_IF_GAZE) },
	{ USB_DEVICE_INTERFACE_NUMBER(PSVR2_VENDOR_ID, PSVR2_PRODUCT_ID,
				      PSVR2_IF_LD) },
	{ USB_DEVICE_INTERFACE_NUMBER(PSVR2_VENDOR_ID, PSVR2_PRODUCT_ID,
				      PSVR2_IF_RP) },
	{ USB_DEVICE_INTERFACE_NUMBER(PSVR2_VENDOR_ID, PSVR2_PRODUCT_ID,
				      PSVR2_IF_VD) },
	{ USB_DEVICE_INTERFACE_NUMBER(PSVR2_VENDOR_ID, PSVR2_PRODUCT_ID,
				      PSVR2_IF_LOG) },
	{ }
};
MODULE_DEVICE_TABLE(usb, psvr2_id_table);

static struct usb_driver psvr2_driver = {
	.name		= "psvr2",
	.id_table	= psvr2_id_table,
	.probe		= psvr2_probe,
	.disconnect	= psvr2_disconnect,
	.dev_groups	= psvr2_groups,
};
static int __init psvr2_init(void)
{
	int ret = usb_register(&psvr2_driver);

	if (!ret)
		register_pm_notifier(&psvr2_pm_nb);
	return ret;
}

static void __exit psvr2_exit(void)
{
	unregister_pm_notifier(&psvr2_pm_nb);
	cancel_delayed_work_sync(&psvr2_restart);
	usb_deregister(&psvr2_driver);
}

module_init(psvr2_init);
module_exit(psvr2_exit);

MODULE_AUTHOR("PSVR2 Linux project");
MODULE_DESCRIPTION("Sony PlayStation VR2 headset driver (IMU, buttons, control)");
MODULE_LICENSE("GPL");
