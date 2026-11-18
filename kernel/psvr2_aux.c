// SPDX-License-Identifier: GPL-2.0
/*
 * PSVR2 Linux driver — auxiliary tracking interfaces (IF8/9/10/12).
 *
 * The headset's tracking pipeline needs these bulk IN endpoints read all the
 * time, as Sony's driver does with one reader thread per interface
 * (docs/sony-re/headset.md section 2.1):
 *
 *   IF8  EP 0x89  "LD" LED-detector blobs     -> /dev/psvr2-ld
 *   IF9  EP 0x8a  "RP" relocaliser keyframes  -> /dev/psvr2-rp
 *   IF10 EP 0x8b  "VD" factory calibration    -> sysfs camera_calib0, ...
 *   IF12 EP 0x8d  firmware console log        -> dynamic debug ("fw: ")
 *
 * Connect hooks follow Sony: IF8 pushes the default LED-detection window
 * (Set 0x14), IF9 reads report 0x8a, IF10 selects camera mode 0x10 (the
 * keepalive in psvr2_ctrl.c re-sends it until the calibration arrived).
 *
 * A stream error does not stop an interface: like Sony's reader, which
 * reopens the interface 500 ms after a failure, the URB is resubmitted after
 * 500 ms, clearing a halted endpoint first.
 *
 * Firmware log: echo 'module psvr2 +p' > /sys/kernel/debug/dynamic_debug/control
 *
 * Copyright (C) 2026 PSVR2 Linux project
 */
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/usb.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>

#include "psvr2.h"
#include "psvr2_protocol.h"
#include "psvr2_uapi.h"

#define PSVR2_AUX_RETRY_MS	500
#define PSVR2_LD_SLOTS		8
#define PSVR2_RP_SLOTS		2

/*
 * Queue of whole packets for one stream char device. Slots are filled from
 * URB completion and drained by read(); a full queue drops the oldest.
 */
struct psvr2_pktq {
	struct kref		kref;
	struct miscdevice	miscdev;
	char			name[16];
	spinlock_t		lock;
	wait_queue_head_t	readq;
	bool			dead;
	unsigned int		slots, slot_size, head, count;
	u32			sequence, dropped;
	u8			*mem;		/* slots * (hdr + slot_size) */
};

struct psvr2_aux {
	struct psvr2_device	*psvr2;
	struct usb_device	*udev;
	struct urb		*urb;
	u8			*buf;
	size_t			buf_size;
	u8			ifnum;
	unsigned int		pipe;
	bool			stopping;
	struct delayed_work	retry;
	struct psvr2_pktq	*q;		/* LD and RP only */
};

static int psvr2_aux_index(u8 ifnum)
{
	switch (ifnum) {
	case PSVR2_IF_LD:
		return 0;
	case PSVR2_IF_RP:
		return 1;
	case PSVR2_IF_VD:
		return 2;
	case PSVR2_IF_LOG:
		return 3;
	default:
		return -1;
	}
}

/* ---------------------------------------------------------- packet queue */

static struct psvr2_packet_hdr *psvr2_pktq_slot(struct psvr2_pktq *q,
						unsigned int i)
{
	return (void *)(q->mem + (size_t)i * (sizeof(struct psvr2_packet_hdr) +
					      q->slot_size));
}

static void psvr2_pktq_free(struct kref *kref)
{
	struct psvr2_pktq *q = container_of(kref, struct psvr2_pktq, kref);

	vfree(q->mem);
	kfree(q);
}

static void psvr2_pktq_push(struct psvr2_pktq *q, const u8 *data, u32 len,
			    s64 now_ns)
{
	struct psvr2_packet_hdr *h;
	unsigned long flags;

	len = min(len, q->slot_size);
	spin_lock_irqsave(&q->lock, flags);
	if (q->count == q->slots) {		/* drop the oldest */
		q->head = (q->head + 1) % q->slots;
		q->count--;
		q->dropped++;
	}
	h = psvr2_pktq_slot(q, (q->head + q->count) % q->slots);
	h->timestamp_ns = now_ns;
	h->length = len;
	h->sequence = q->sequence++;
	h->dropped = 0;
	h->reserved = 0;
	memcpy(h + 1, data, len);
	q->count++;
	spin_unlock_irqrestore(&q->lock, flags);
	wake_up_interruptible(&q->readq);
}

static int psvr2_pktq_open(struct inode *inode, struct file *file)
{
	struct psvr2_pktq *q = container_of(file->private_data,
					    struct psvr2_pktq, miscdev);

	kref_get(&q->kref);
	file->private_data = q;
	return stream_open(inode, file);
}

static int psvr2_pktq_release(struct inode *inode, struct file *file)
{
	struct psvr2_pktq *q = file->private_data;

	kref_put(&q->kref, psvr2_pktq_free);
	return 0;
}

static ssize_t psvr2_pktq_read(struct file *file, char __user *ubuf,
			       size_t count, loff_t *ppos)
{
	struct psvr2_pktq *q = file->private_data;
	struct psvr2_packet_hdr *h;
	unsigned long flags;
	u8 *tmp;
	size_t len;
	int ret;

	for (;;) {
		spin_lock_irqsave(&q->lock, flags);
		if (q->count)
			break;
		spin_unlock_irqrestore(&q->lock, flags);
		if (q->dead)
			return 0;
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(q->readq, q->count || q->dead);
		if (ret)
			return ret;
	}

	h = psvr2_pktq_slot(q, q->head);
	len = sizeof(*h) + h->length;
	if (count < len) {
		spin_unlock_irqrestore(&q->lock, flags);
		return -EMSGSIZE;
	}
	spin_unlock_irqrestore(&q->lock, flags);

	/* Copy out without the lock: the slot may be reused meanwhile. */
	tmp = kvmalloc(len, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;
	spin_lock_irqsave(&q->lock, flags);
	if (!q->count) {
		spin_unlock_irqrestore(&q->lock, flags);
		kvfree(tmp);
		return -EAGAIN;
	}
	h = psvr2_pktq_slot(q, q->head);
	len = min_t(size_t, sizeof(*h) + h->length, count);
	memcpy(tmp, h, len);
	((struct psvr2_packet_hdr *)tmp)->dropped = q->dropped;
	q->dropped = 0;
	q->head = (q->head + 1) % q->slots;
	q->count--;
	spin_unlock_irqrestore(&q->lock, flags);

	ret = copy_to_user(ubuf, tmp, len) ? -EFAULT : len;
	kvfree(tmp);
	return ret;
}

static __poll_t psvr2_pktq_poll(struct file *file, poll_table *wait)
{
	struct psvr2_pktq *q = file->private_data;
	__poll_t mask = 0;

	poll_wait(file, &q->readq, wait);
	if (READ_ONCE(q->count))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (q->dead)
		mask |= EPOLLHUP;
	return mask;
}

static const struct file_operations psvr2_pktq_fops = {
	.owner		= THIS_MODULE,
	.open		= psvr2_pktq_open,
	.release	= psvr2_pktq_release,
	.read		= psvr2_pktq_read,
	.poll		= psvr2_pktq_poll,
};

static struct psvr2_pktq *psvr2_pktq_create(const char *name,
					    unsigned int slots,
					    unsigned int slot_size)
{
	struct psvr2_pktq *q;

	q = kzalloc(sizeof(*q), GFP_KERNEL);
	if (!q)
		return NULL;
	q->mem = vzalloc((size_t)slots *
			 (sizeof(struct psvr2_packet_hdr) + slot_size));
	if (!q->mem) {
		kfree(q);
		return NULL;
	}
	kref_init(&q->kref);
	spin_lock_init(&q->lock);
	init_waitqueue_head(&q->readq);
	q->slots = slots;
	q->slot_size = slot_size;
	strscpy(q->name, name, sizeof(q->name));
	q->miscdev.minor = MISC_DYNAMIC_MINOR;
	q->miscdev.name = q->name;
	q->miscdev.fops = &psvr2_pktq_fops;
	if (misc_register(&q->miscdev)) {
		vfree(q->mem);
		kfree(q);
		return NULL;
	}
	return q;
}

static void psvr2_pktq_destroy(struct psvr2_pktq *q)
{
	if (!q)
		return;
	misc_deregister(&q->miscdev);
	q->dead = true;
	wake_up_interruptible(&q->readq);
	kref_put(&q->kref, psvr2_pktq_free);
}

/* ------------------------------------------------------- stream decoding */

/* Print one firmware log transfer line by line, without ANSI escapes. */
static void psvr2_aux_log(struct psvr2_aux *aux, const u8 *p, unsigned int len)
{
	char line[256];
	unsigned int i, n = 0;

	for (i = 0; i < len; i++) {
		u8 c = p[i];

		if (c == 0x1b) {	/* ESC [ ... final byte 0x40..0x7e */
			if (i + 1 < len && p[i + 1] == '[') {
				for (i += 2; i < len && (p[i] < 0x40 || p[i] > 0x7e); i++)
					;
			}
			continue;
		}
		if (n && (c == '\n' || c == '\r' || c == 0 || n == sizeof(line) - 1)) {
			line[n] = 0;
			dev_dbg(&aux->udev->dev, "fw: %s\n", line);
			n = 0;
		}
		if (c >= 0x20 && c < 0x7f)
			line[n++] = c;
	}
	if (n) {
		line[n] = 0;
		dev_dbg(&aux->udev->dev, "fw: %s\n", line);
	}
}

/*
 * IF10 "VD" records (docs/sony-driver-re.md, calibration stream): "VD", u16
 * version 0x0200, u32 record size, u16 entry count at +8, then from +0x20
 * entries of {u32 type, u32 size, u32 offset from record start}. Types
 * 10..13 are kept for userspace.
 */
static void psvr2_aux_vd(struct psvr2_aux *aux, const u8 *p, unsigned int len)
{
	struct psvr2_device *psvr2 = aux->psvr2;
	unsigned int pos = 0;

	while (pos + 0x20 <= len) {
		const u8 *rec = p + pos;
		u32 size = get_unaligned_le32(rec + 4);
		u16 n = get_unaligned_le16(rec + 8);
		unsigned int i;

		if (rec[0] != 'V' || rec[1] != 'D' ||
		    get_unaligned_le16(rec + 2) != 0x0200 || size < 0x20 ||
		    size > len - pos)
			return;

		for (i = 0; i < n && 0x20 + 12 * (i + 1) <= size; i++) {
			const u8 *e = rec + 0x20 + 12 * i;
			u32 type = get_unaligned_le32(e);
			u32 esize = get_unaligned_le32(e + 4);
			u32 off = get_unaligned_le32(e + 8);
			unsigned long flags;
			int slot;

			if (type < 10 || type > 13)
				continue;
			slot = type - 10;
			if (!esize || esize > PSVR2_CALIB_MAX_LEN || off > size ||
			    esize > size - off || !psvr2->calib[slot])
				continue;
			spin_lock_irqsave(&psvr2->calib_lock, flags);
			memcpy(psvr2->calib[slot], rec + off, esize);
			psvr2->calib_len[slot] = esize;
			spin_unlock_irqrestore(&psvr2->calib_lock, flags);
			if (!test_and_set_bit(slot, &psvr2->calib_seen))
				dev_info(&aux->udev->dev,
					 "calibration type %u received (%u bytes)\n",
					 type, esize);
		}
		pos += size;
	}
}

static void psvr2_aux_data(struct psvr2_aux *aux, unsigned int len)
{
	const u8 *p = aux->buf;
	s64 now = ktime_get_ns();

	switch (aux->ifnum) {
	case PSVR2_IF_LOG:
		psvr2_aux_log(aux, p, len);
		break;
	case PSVR2_IF_VD:
		psvr2_aux_vd(aux, p, len);
		break;
	case PSVR2_IF_LD:
		if (len >= 2 && p[0] == 'L' && p[1] == 'D')
			psvr2_pktq_push(aux->q, p, len, now);
		break;
	case PSVR2_IF_RP:
		/* Sony accepts only "RP" with the full size at +4. */
		if (len >= 8 && p[0] == 'R' && p[1] == 'P' &&
		    get_unaligned_le32(p + 4) == PSVR2_RP_LEN)
			psvr2_pktq_push(aux->q, p, len, now);
		break;
	}
}

/* ------------------------------------------------------------------ URBs */

static void psvr2_aux_retry(struct work_struct *work)
{
	struct psvr2_aux *aux = container_of(to_delayed_work(work),
					     struct psvr2_aux, retry);
	int ret;

	if (READ_ONCE(aux->stopping))
		return;
	usb_clear_halt(aux->udev, aux->pipe);
	ret = usb_submit_urb(aux->urb, GFP_KERNEL);
	if (ret && ret != -EPERM && ret != -ENODEV)
		dev_err(&aux->udev->dev, "IF%u: resubmit failed: %d\n",
			aux->ifnum, ret);
}

static void psvr2_aux_complete(struct urb *urb)
{
	struct psvr2_aux *aux = urb->context;
	int ret;

	switch (urb->status) {
	case 0:
		if (urb->actual_length)
			psvr2_aux_data(aux, urb->actual_length);
		break;
	case -EOVERFLOW:	/* short/large packet — keep draining */
		break;
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
	case -ENODEV:
		return; /* unlinked / device gone */
	default:
		dev_dbg(&aux->udev->dev, "aux IF%u URB error %d, retry in %d ms\n",
			aux->ifnum, urb->status, PSVR2_AUX_RETRY_MS);
		if (!READ_ONCE(aux->stopping))
			schedule_delayed_work(&aux->retry,
					      msecs_to_jiffies(PSVR2_AUX_RETRY_MS));
		return;
	}

	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret && ret != -EPERM)
		dev_err(&aux->udev->dev, "failed to resubmit aux IF%u URB: %d\n",
			aux->ifnum, ret);
}

/* Sony's per-interface connect hooks (docs/sony-re/headset.md 2.1, 3.2). */
static void psvr2_aux_connect(struct psvr2_aux *aux, struct usb_interface *intf)
{
	struct psvr2_device *psvr2 = aux->psvr2;
	int ret;

	switch (aux->ifnum) {
	case PSVR2_IF_LD: {
		/* Default LED-detection window; userspace tunes it per preset. */
		u8 win[16] = { 0 };

		put_unaligned_le32(0x55, win + 0x0);
		put_unaligned_le32(500, win + 0x4);
		put_unaligned_le32(0x24800000 | 1, win + 0x8);
		ret = psvr2_control_set(psvr2, PSVR2_REPORT_SET_LD_WINDOW, 1,
					win, sizeof(win));
		if (ret)
			dev_warn(&intf->dev, "LED window (0x14) failed: %d\n", ret);
		break;
	}
	case PSVR2_IF_RP: {
		u8 info[8];

		/* Sony reads it and never uses the value. */
		psvr2_control_get(psvr2, PSVR2_REPORT_GET_RP_INFO, 1, info,
				  sizeof(info));
		break;
	}
	case PSVR2_IF_VD:
		if (!psvr2->camera_v4l2) {
			ret = psvr2_set_camera_mode(psvr2, psvr2->camera_mode);
			if (ret)
				dev_warn(&intf->dev, "camera mode 0x%x failed: %d\n",
					 psvr2->camera_mode, ret);
		}
		break;
	}
}

int psvr2_aux_start(struct psvr2_device *psvr2, struct usb_interface *intf)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	u8 ifnum = intf->cur_altsetting->desc.bInterfaceNumber;
	struct usb_endpoint_descriptor *ep;
	struct psvr2_aux *aux;
	int idx, i, ret;

	idx = psvr2_aux_index(ifnum);
	if (idx < 0)
		return -ENODEV;

	aux = kzalloc(sizeof(*aux), GFP_KERNEL);
	if (!aux)
		return -ENOMEM;

	aux->psvr2 = psvr2;
	aux->udev = udev;
	aux->ifnum = ifnum;
	aux->buf_size = PSVR2_AUX_XFER_SIZE;
	INIT_DELAYED_WORK(&aux->retry, psvr2_aux_retry);

	/*
	 * Issue the explicit SET_INTERFACE the DMJC fork sends. There is only one
	 * alt setting, so usbcore leaves it selected by default without putting a
	 * request on the wire; the fork sends one anyway, and that may be the
	 * trigger that arms the tracking pipeline. Non-fatal if it fails.
	 */
	ret = usb_set_interface(udev, ifnum, PSVR2_AUX_ALT);
	if (ret)
		dev_warn(&intf->dev, "failed to select IF%u alt %d: %d\n",
			 ifnum, PSVR2_AUX_ALT, ret);

	ret = usb_find_bulk_in_endpoint(intf->cur_altsetting, &ep);
	if (ret) {
		dev_err(&intf->dev, "no bulk IN endpoint on IF%u\n", ifnum);
		goto err_free;
	}
	aux->pipe = usb_rcvbulkpipe(udev, ep->bEndpointAddress);

	if (ifnum == PSVR2_IF_VD) {
		for (i = 0; i < PSVR2_CALIB_COUNT; i++) {
			if (!psvr2->calib[i])
				psvr2->calib[i] = kzalloc(PSVR2_CALIB_MAX_LEN,
							  GFP_KERNEL);
			if (!psvr2->calib[i]) {
				ret = -ENOMEM;
				goto err_free;
			}
		}
		psvr2->calib_seen = 0;
	} else if (ifnum == PSVR2_IF_LD || ifnum == PSVR2_IF_RP) {
		bool ld = ifnum == PSVR2_IF_LD;

		aux->q = psvr2_pktq_create(ld ? "psvr2-ld" : "psvr2-rp",
					   ld ? PSVR2_LD_SLOTS : PSVR2_RP_SLOTS,
					   ld ? PSVR2_LD_MAX_LEN : PSVR2_RP_LEN);
		if (!aux->q) {
			ret = -ENOMEM;
			goto err_free;
		}
	}

	aux->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!aux->urb) {
		ret = -ENOMEM;
		goto err_q;
	}

	aux->buf = usb_alloc_coherent(udev, aux->buf_size, GFP_KERNEL,
				      &aux->urb->transfer_dma);
	if (!aux->buf) {
		ret = -ENOMEM;
		goto err_urb;
	}

	usb_fill_bulk_urb(aux->urb, udev, aux->pipe, aux->buf, aux->buf_size,
			  psvr2_aux_complete, aux);
	aux->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	psvr2->aux[idx] = aux;
	psvr2_aux_connect(aux, intf);

	ret = usb_submit_urb(aux->urb, GFP_KERNEL);
	if (ret) {
		dev_err(&intf->dev, "failed to submit aux IF%u URB: %d\n",
			ifnum, ret);
		psvr2->aux[idx] = NULL;
		goto err_buf;
	}

	dev_dbg(&intf->dev, "reading aux tracking interface %u\n", ifnum);
	return 0;

err_buf:
	usb_free_coherent(udev, aux->buf_size, aux->buf, aux->urb->transfer_dma);
err_urb:
	usb_free_urb(aux->urb);
err_q:
	psvr2_pktq_destroy(aux->q);
err_free:
	kfree(aux);
	return ret;
}

void psvr2_aux_stop(struct psvr2_device *psvr2, struct usb_interface *intf)
{
	u8 ifnum = intf->cur_altsetting->desc.bInterfaceNumber;
	int idx = psvr2_aux_index(ifnum);
	struct psvr2_aux *aux;

	if (idx < 0)
		return;
	aux = psvr2->aux[idx];
	if (!aux)
		return;
	psvr2->aux[idx] = NULL;

	WRITE_ONCE(aux->stopping, true);
	cancel_delayed_work_sync(&aux->retry);
	usb_kill_urb(aux->urb);
	cancel_delayed_work_sync(&aux->retry);
	usb_free_coherent(aux->udev, aux->buf_size, aux->buf,
			  aux->urb->transfer_dma);
	usb_free_urb(aux->urb);
	psvr2_pktq_destroy(aux->q);
	kfree(aux);
}
