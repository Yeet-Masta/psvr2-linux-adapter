// SPDX-License-Identifier: GPL-2.0
/* DualSense / DS4 output report fields (docs/sony-re/libpad.md,
 * "DualSense and DS4 output state"). */
#include <string.h>
#include "psvr2_libpad.h"

#define SONY_VID 0x054c

/* c * pct / 100, rounded half up. The first pass divides signed (a
 * negative brightness wraps), the boost pass unsigned. */
static uint8_t pct_of_signed(uint32_t c, int32_t pct)
{
	const int32_t v = (int32_t)(c * (uint32_t)pct * 100);
	return (uint8_t)(v / 10000 + (v % 10000 > 4999));
}

static uint8_t pct_of(uint32_t c, uint32_t pct)
{
	const uint32_t v = c * pct * 100;
	return (uint8_t)(v / 10000 + (v % 10000 > 4999));
}

void psvr2_pad_lightbar_scale(uint8_t *r, uint8_t *g, uint8_t *b, int bright, int no_floor)
{
	if (bright >= 0xff)
		return;
	const uint32_t floor = no_floor ? 0 : 13;
	const int32_t pct = (int32_t)((uint32_t)bright * 100) / 0xff;
	uint8_t c[3] = { pct_of_signed(*r, pct), pct_of_signed(*g, pct), pct_of_signed(*b, pct) };
	if (c[0] < floor && c[1] < floor && c[2] < floor) {
		/* Too dim to see: scale up so the brightest channel reaches the floor. */
		uint8_t max = c[0];
		if (c[1] > max)
			max = c[1];
		if (c[2] > max)
			max = c[2];
		if (!max)
			return;
		const uint32_t up = floor * 100 / max;
		for (int i = 0; i < 3; i++)
			c[i] = pct_of(c[i], up);
	}
	*r = c[0];
	*g = c[1];
	*b = c[2];
}

/* Decode both effect slots and flag those the older DualSense firmware
 * cannot run (multi-position feedback, slope, multi-position vibration). */
int32_t psvr2_pad_effect_unsupported(const uint8_t *blk, uint8_t *slot0, uint8_t *slot1)
{
	*slot0 = *slot1 = 0;
	if (!blk)
		return (int32_t)0x80920001;
	uint8_t param[0x88] = { 0 };
	int32_t ret = 0;
	for (int s = 0; s < 2; s++) {
		if (!(blk[0] & (1u << s)))
			continue;
		ret = psvr2_pad_trigger_effect_decode(blk[1 + 0xb * s], blk + 2 + 0xb * s, param, (uint8_t)s);
		if (ret < 0)
			return ret;
		uint32_t mode;
		memcpy(&mode, param + 8 + 0x38 * s, 4);
		if (mode - 4 < 3)
			*(s ? slot1 : slot0) = 1;
	}
	return ret;
}

static uint8_t dim(uint8_t mode, uint8_t half, uint8_t low)
{
	return mode == 0 ? 0xff : mode == 1 ? half : low;
}

/* One trigger: the effect from the pad, or "off" (wire mode 5) when the
 * firmware cannot run it; then the host estimates the trigger state. */
static void trigger(uint8_t *dst, const uint8_t *eff, int replace, uint8_t *est)
{
	if (replace) {
		dst[0] = 5;
		memset(dst + 1, 0, 10);
	} else {
		memcpy(dst, eff, 11);
	}
	*est = (uint8_t)replace;
}

void psvr2_ds_output_fill(psvr2_pad *p, uint8_t *buf)
{
	static const uint8_t player_leds[4] = { 0x04, 0x0a, 0x15, 0x1b };
	const uint8_t f2 = buf[2];

	buf[0] = 2;
	buf[2] = f2 | 0x10;
	if (p->player >= 1 && p->player <= 4)
		buf[0x2c] = player_leds[p->player - 1];
	uint8_t r = p->ds_rgb[0], g = p->ds_rgb[1], b = p->ds_rgb[2];
	psvr2_pad_lightbar_scale(&r, &g, &b, dim(p->light_dim, 0x80, 0x40), 1);
	buf[0x2d] = r;
	buf[0x2e] = g;
	buf[0x2f] = b;
	buf[2] = f2 | 0x15;
	if (p->mic_led < 3) {
		static const uint16_t mic[3] = { 0x0000, 0x1001, 0x1002 };
		memcpy(buf + 9, &mic[p->mic_led], 2);
	}
	buf[1] |= 0x0c;
	if (!p->trig_level) {
		buf[0x16] = 5;
		buf[0x0b] = 5;
		return;
	}
	uint8_t un0, un1;
	psvr2_pad_effect_unsupported(&p->eff_mask, &un0, &un1);
	const int old_fw = psvr2_pad_is_dualsense_std(p->vid, p->pid) && p->fw < 0x297;
	trigger(buf + 0x16, &p->eff_mode0, old_fw && un0, &p->est_trig[0]);
	trigger(buf + 0x0b, &p->eff_mode1, old_fw && un1, &p->est_trig[1]);
}

void psvr2_ds4_output_fill(const psvr2_pad *p, uint8_t *buf)
{
	if (!p->player)
		return;
	uint8_t r = p->ds4_rgb[0], g = p->ds4_rgb[1], b = p->ds4_rgb[2];
	buf[1] |= 2;
	buf[0] = 5;
	psvr2_pad_lightbar_scale(&r, &g, &b, dim(p->light_dim, 0x80, 0x0d), 0);
	buf[6] = r;
	buf[7] = g;
	buf[8] = b;
}

static int open_dualsense(psvr2_pad *p, const psvr2_pad_switches *sw, uint8_t *r)
{
	static const uint8_t player_leds[4] = { 0x04, 0x0a, 0x15, 0x1b };
	r[0] = 5;
	r[1] = 0x0d;
	r[2] = 0x17;
	if (p->link == 1 && p->player) {
		r[0x27] = 2;
		r[0x2a] = 2;
	}
	r[0x2c] = p->player >= 1 && p->player <= 4 ? player_leds[p->player - 1] : 0;
	if (sw->haptics_b) {
		r[1] |= 2;
		if (!psvr2_pad_is_dualsense_std(p->vid, p->pid) || p->fw > 0x21f)
			r[0x27] |= 4;
	}
	memcpy(r + 0x2d, p->ds_rgb, 3);
	r[9] = p->mic_led == 1;
	uint8_t un0, un1;
	psvr2_pad_effect_unsupported(&p->eff_mask, &un0, &un1);
	const int old_fw = psvr2_pad_is_dualsense_std(p->vid, p->pid) && p->fw < 0x297;
	trigger(r + 0x16, &p->eff_mode0, old_fw && un0, &p->est_trig[0]);
	trigger(r + 0x0b, &p->eff_mode1, old_fw && un1, &p->est_trig[1]);
	return 0x30;
}

static int open_sense(const psvr2_pad *p, const psvr2_pad_switches *sw, uint8_t *r)
{
	r[0] = 2;
	r[1] = 0x1c;
	r[2] = 4;
	if (p->fw > 0x253) {
		if (sw->flag02)
			r[2] |= 2;
		if (sw->flag10)
			r[2] |= 0x10;
	}
	if (sw->state_en)
		r[4] = (uint8_t)(sw->state_val * 2);
	const int left = p->hand == 1 || p->pid == 0x0e45;
	memcpy(r + 5, left ? &p->eff_mode0 : &p->eff_mode1, 11);
	r[0x24] = 1;
	return 0x30;
}

int psvr2_pad_open_reports(psvr2_pad *p, const psvr2_pad_switches *sw,
			   uint8_t rep[2][0x30], uint8_t len[2])
{
	memset(rep, 0, 2 * 0x30);
	len[0] = len[1] = 0;
	if (p->vid == SONY_VID && (p->pid == 0x0ce6 || p->pid == 0x0df2)) {
		if (p->in_use != 1)
			return 0;
		len[0] = (uint8_t)open_dualsense(p, sw, rep[0]);
		return 1;
	}
	if (p->vid == SONY_VID && (p->pid == 0x0ce7 || p->pid == 0x0e45 || p->pid == 0x0e46)) {
		if (p->in_use != 1)
			return 0;
		len[0] = (uint8_t)open_sense(p, sw, rep[0]);
		return 1;
	}
	/* DS4 and anything else: reset, then the lightbar colour. */
	int n = 0;
	if (p->in_use == 1) {
		rep[n][0] = 5;
		rep[n][1] = 1;
		len[n++] = 0x20;
		if (p->player) {
			rep[n][0] = 5;
			rep[n][1] = 2;
			memcpy(rep[n] + 6, p->ds4_rgb, 3);
			len[n++] = 0x20;
		}
	}
	if (psvr2_pad_is_ds4_new(p->vid, p->pid, p->hid_ver))
		p->est_trig[0] = p->est_trig[1] = 1;
	return n;
}
