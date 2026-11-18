// SPDX-License-Identifier: GPL-2.0
/* Sense controller output-report fields (docs/sony-re/libpad.md, "Sense output report"). */
#include <string.h>
#include "psvr2_libpad.h"

static void put32(uint8_t *b, uint32_t v) { memcpy(b, &v, 4); }

void psvr2_ledsync_resync(const psvr2_sync_state *st, psvr2_ledsync *ls, uint64_t now)
{
	if (ls->phase != 1)
		return;
	/* Unsigned distance; a "future" stamp is added rather than subtracted. */
	const uint64_t d = ls->last_resync <= now ? now - ls->last_resync : now + ls->last_resync;
	if (d < 1000000 || !psvr2_sync_headset_time)
		return;
	const uint64_t t = psvr2_sync_headset_time();
	int32_t ctrl = 0;
	if (!t || psvr2_sync_host_to_ctrl(st, (int64_t)t, &ctrl))
		return;
	ls->last_resync = now;
	ls->seq++;
	ls->base_time = -((int32_t)((uint32_t)ls->period_id * (uint32_t)ls->step) / 2) -
			ls->width / 2 + ctrl;
}

void psvr2_sense_output_fill(const psvr2_sense_out_ctx *c, const psvr2_sync_state *st,
			     psvr2_ledsync *ls, uint8_t *buf, uint64_t *last_out,
			     uint64_t now, uint32_t stamp, uint64_t resync_now)
{
	put32(buf + 0x10, stamp);
	if (c->conn == 2 || c->conn == 3 || c->g_rumble_a || c->g_rumble_b) {
		buf[1] |= 0x02;
	} else if (c->conn == 4 || c->conn == 5) {
		if (c->fw > 0x220)
			buf[1] |= 0x80;
		else if (c->link == 1)
			buf[1] |= 0x02;
	}
	if (!(buf[1] & 0x10)) {
		buf[1] |= 0x10;
		uint8_t lo = (uint8_t)(8 - c->vol_haptic);
		uint8_t hi = (uint8_t)(11 - c->vol_trigger);
		if (hi == 11)
			hi = 10;
		if (lo == 8)
			lo = 7;
		buf[0x23] = (uint8_t)(hi << 4 | (lo & 7));
	}
	if (ls->sync_mode == 0) {
		if (c->dev_fw < 0x1006d || ls->probes > 3) {
			psvr2_ledsync_set_mode(ls, 2);
		} else {
			const uint8_t m = c->probe ? c->probe(c->probe_ctx) : 0;
			if (ls->sync_mode != m)
				psvr2_ledsync_set_mode(ls, m);
			ls->probes++;
		}
	}
	const int first = ls->phase && !ls->last_resync;
	/* (Sony sends the polling-rate feature here when enabled; see doc.) */
	if (first)
		ls->last_resync = now;
	else
		psvr2_ledsync_resync(st, ls, resync_now);
	if (ls->phase == 0 || ls->phase > 4 || st->fit_off)
		buf[0x14] = ls->phase;
	buf[0x15] = ls->seq;
	buf[0x16] = ls->period_id;
	put32(buf + 0x17, (uint32_t)ls->base_time * 3);
	put32(buf + 0x1b, ls->cycle_ns * 3);
	if (ls->phase == 2 || ls->phase == 3)
		memcpy(buf + 0x1f, ls->led, 4);
	else
		memset(buf + 0x1f, 0xff, 4);
	*last_out = now;
	if (c->fw > 0x253) {
		if (c->g_flag2_02)
			buf[2] |= 0x02;
		if (c->g_flag2_10)
			buf[2] |= 0x10;
	}
}

void psvr2_sense_set_vibration(uint8_t *buf, int fresh, uint8_t conn,
			       uint8_t g_rumble_a, uint8_t g_rumble_b, uint8_t amplitude)
{
	buf[0] = 0x02;
	buf[1] |= 0x40;
	if (conn == 2 || conn == 4)
		buf[1] |= 0x01;
	else if (conn == 3 || conn == 5 || g_rumble_a || g_rumble_b)
		buf[1] |= 0x20;
	/* A fresh buffer always gets bit 0; a queued one only on kinds 2/4. */
	if (fresh)
		buf[1] |= 0x01;
	buf[3] = amplitude;
}

void psvr2_sense_set_trigger_level(uint8_t *buf, int fresh, uint8_t old, uint8_t level,
				   uint8_t vol_haptic, const uint8_t *effect)
{
	if (!level) {
		if (!old)
			return;
		/* Strength 0: replace the effect by mode 5 (off). */
		buf[0] = 0x02;
		buf[1] |= 0x04;
		buf[5] = 0x05;
		return;
	}
	buf[0] = 0x02;
	if (!old) {
		/* Re-enable: resend the stored effect. */
		buf[1] |= fresh ? 0x04 : 0x0c;
		if (effect)
			memcpy(buf + 5, effect, 11);
	}
	buf[1] |= 0x10;
	uint8_t hi = (uint8_t)(11 - level), lo = (uint8_t)(8 - vol_haptic);
	if (hi == 11)
		hi = 10;
	if (lo == 8)
		lo = 7;
	buf[0x23] |= (uint8_t)(hi << 4 | (lo & 7));
}

void psvr2_sense_set_trigger_effect(uint8_t *buf, int is_left, int is_right,
				    const uint8_t wire[23])
{
	buf[0] = 0x02;
	/* One trigger per controller; a pad matching both takes the R2 block. */
	if (is_left && (wire[0] & 1)) {
		buf[1] |= 0x04;
		memcpy(buf + 5, wire + 1, 11);
	}
	if (is_right && (wire[0] & 2)) {
		buf[1] |= 0x04;
		memcpy(buf + 5, wire + 12, 11);
	}
}

void psvr2_sense_fill_state(uint8_t *buf, uint8_t trigger_level, uint8_t vib_level,
			    const uint8_t effect[11], uint8_t g_enable, uint8_t g_value)
{
	buf[1] |= 0x04;
	buf[0] = 0x02;
	if (!trigger_level)
		buf[5] = 0x05;
	else
		memcpy(buf + 5, effect, 11);
	buf[2] |= 0x04;
	buf[1] |= 0x08;
	buf[0x24] = 1;
	if (!vib_level)
		buf[4] |= 0x80;
	if (g_enable)
		buf[4] |= (uint8_t)(g_value * 2);
}

void psvr2_sense_output_clamp(uint8_t *buf, psvr2_sync_state *st)
{
	if (!buf || !buf[0x14])
		return;
	if (buf[0x16] > 0x3c) {
		st->n_period_clamp1++;
		if (st->max_period1 < buf[0x16])
			st->max_period1 = buf[0x16];
		buf[0x16] = 0x3c;
	}
	int32_t raw;
	memcpy(&raw, buf + 0x1b, 4);
	const uint32_t ns = (uint32_t)(raw / 3);
	uint32_t fix;
	if (ns < 0x10dd397) {
		if (ns > 0xef4f15)
			return;
		st->n_cycle_lo1++;
		if (ns < st->min_cycle1)
			st->min_cycle1 = ns;
		fix = 0x2cded42;
	} else {
		st->n_cycle_hi1++;
		if (st->max_cycle1 < ns)
			st->max_cycle1 = ns;
		fix = 0x3297ac2;
	}
	memcpy(buf + 0x1b, &fix, 4);
}

int32_t psvr2_pad_frame_output(int kind, int edge, uint16_t report_len, const uint8_t *buf,
			       uint16_t len, uint8_t *dst, uint32_t *towrite)
{
	if (report_len < len)
		return (int32_t)0x80010028;
	const uint32_t limit = edge ? 0x40 : 0x30;
	const int usb = report_len <= limit;
	*towrite = usb ? report_len : 0x4e;
	memset(dst, 0, report_len);
	if (usb) {
		memcpy(dst, buf, len);
		dst[0] = kind == PSVR2_DEV_DS4 ? 5 : 2;
		return len;
	}
	if (report_len < *towrite)
		return (int32_t)0x8001002b;
	uint32_t n = len;
	switch (kind) {
	case PSVR2_DEV_DUALSENSE:	/* 31 00 10, payload */
		dst[0] = 0x31;
		dst[2] = 0x10;
		n = len ? len : 1;
		memcpy(dst + 3, buf + 1, n - 1);
		break;
	case PSVR2_DEV_SENSE:		/* 31 a2, payload (one byte past len is copied too) */
		dst[0] = 0x31;
		dst[1] = 0xa2;
		n = len ? len : 1;
		memcpy(dst + 2, buf + 1, n);
		break;
	default:			/* 11 c0 00, payload */
		dst[0] = 0x11;
		dst[1] = 0xc0;
		memcpy(dst + 3, buf + 1, (size_t)len - 1);
		break;
	}
	const uint32_t crc = psvr2_pad_crc32(0xa2, dst, (int)*towrite);
	memcpy(dst + *towrite - 4, &crc, 4);
	return (int32_t)n;
}
