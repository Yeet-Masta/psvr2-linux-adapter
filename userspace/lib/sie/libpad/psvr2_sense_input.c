// SPDX-License-Identifier: GPL-2.0
/* Sense controller input report decode (docs/sony-re/controllers.md part A, "Sense input report"). */
#include <string.h>
#include "psvr2_libpad.h"

static uint32_t rd32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static int16_t rd16(const uint8_t *p) { int16_t v; memcpy(&v, p, 2); return v; }

/* Report byte 8 and 9 bits, in order, to libpad button bits. */
static const uint32_t btn8[8] = { 0x8000, 0x4000, 0x2000, 0x1000, 0x0400, 0x0800, 0x0100, 0x0200 };
static const uint32_t btn9[5] = { 0x0001, 0x0008, 0x0002, 0x0004, 0x10000 };

void psvr2_sense_input_decode(uint8_t hand, const uint8_t *report, uint8_t bt,
			      psvr2_sense_input *o)
{
	const uint8_t *r = report + bt;	/* r[0] is the report id on USB */
	const int left = hand == 1;

	for (int i = 0; i < 8; i++)
		if (r[8] & (1u << i))
			o->buttons |= btn8[i];
	for (int i = 0; i < 5; i++)
		if (r[9] & (1u << i))
			o->buttons |= btn9[i];
	if (left) {
		o->lx = r[1];
		o->ly = r[2];
		o->rx = o->ry = 0x80;
		o->l2 = r[3];
	} else {
		o->lx = o->ly = 0x80;
		o->rx = r[1];
		o->ry = r[2];
		o->r2 = r[3];
	}
	o->counter = r[7];
	o->battery = r[0x2a];
	o->imu_ticks = rd32(r + 0x1c);
	for (int i = 0; i < 3; i++) {
		o->gyro[i] = rd16(r + 0x10 + 2 * i);
		o->accel[i] = rd16(r + 0x16 + 2 * i);
	}
	o->temperature = r[0x20];
	o->touch = (uint8_t)(r[10] << 4 | (r[9] >> 4 & 8));
	o->trig_prox = r[4];
	o->grip_prox = r[5];
	o->trig_state_l = left ? 0 : r[0x28];
	o->trig_state_r = left ? r[0x28] : 0;
	o->trig_mode = r[0x29];
	o->left = left;
	o->ctrl_ticks = rd32(r + 0x30) ? rd32(r + 0x30) : o->imu_ticks;
	o->echo = rd32(r + 0x2c);
	o->flag0 = r[0x34] & 1;
	o->flag3 = r[0x34] & 8;
}

/* Unpack the 3-bit level fields of a 6-byte block: bit n of the two mask
 * bytes enables level n (stored as value + 1; 0 = absent). */
void psvr2_pad_trigger_levels(const uint8_t *in, uint8_t out[10])
{
	const uint32_t bits = in[2] | in[3] << 8 | in[4] << 16 | (uint32_t)in[5] << 24;
	const uint16_t mask = in[0] | in[1] << 8;
	for (int n = 0; n < 10; n++)
		out[n] = (mask >> n & 1) ? (uint8_t)((bits >> (3 * n) & 7) + 1) : 0;
}

static void set_mode(uint8_t *cmd, uint32_t m) { memcpy(cmd, &m, 4); }

/* Feedback/vibration share one shape: a single non-zero level from some
 * position on, or a per-position profile. Returns 1 when uniform. */
static int uniform_from(const uint8_t lv[10], uint8_t *pos)
{
	uint8_t first = 10;
	int uni = 1;
	for (int n = 0; n < 10; n++) {
		if (first == 10) {
			if (lv[n])
				first = (uint8_t)n;
		} else if (lv[first] != lv[n]) {
			uni = 0;
		}
	}
	*pos = first;
	return uni;
}

int32_t psvr2_pad_trigger_effect_decode(uint8_t wire_mode, const uint8_t *w,
					uint8_t *param, uint8_t slot)
{
	uint8_t *cmd = param + 8 + slot * 0x38, *d = cmd + 8;
	uint8_t lv[10] = { 0 }, pos;
	const uint16_t bits = w[0] | w[1] << 8;

	switch (wire_mode) {
	case 0x21:	/* feedback: resistance from a position on */
		psvr2_pad_trigger_levels(w, lv);
		if (uniform_from(lv, &pos)) {
			set_mode(cmd, PSVR2_TRIG_FEEDBACK);
			d[0] = pos;
			d[1] = pos < 10 ? lv[pos] : 0; /* all-zero mask: Sony reads past the array */
		} else {
			set_mode(cmd, PSVR2_TRIG_MULTI_FEEDBACK);
			memcpy(d, lv, 10);
		}
		break;
	case 0x22: {	/* slope: first and last marked position, two strengths */
		uint8_t first = 10, last = 10;
		for (int n = 0; n < 10; n++)
			if (bits >> n & 1) {
				if (first == 10)
					first = (uint8_t)n;
				else
					last = (uint8_t)n;
			}
		set_mode(cmd, PSVR2_TRIG_SLOPE_FEEDBACK);
		d[0] = first;
		d[1] = last;
		d[2] = (w[2] & 7) + 1;
		d[3] = (w[2] >> 3 & 7) + 1;
		break;
	}
	case 0x25: {	/* weapon: start and release position among 2..8 */
		uint8_t start = 9, end = 9;
		for (int n = 2; n < 9; n++)
			if (bits >> n & 1) {
				if (start != 9) {
					end = (uint8_t)n;
					break;
				}
				start = (uint8_t)n;
			}
		if (start > 7 || end > 8 || end <= start)
			return (int32_t)0x80920001;
		set_mode(cmd, PSVR2_TRIG_WEAPON);
		d[0] = start;
		d[1] = end;
		d[2] = (w[2] & 7) + 1;
		break;
	}
	case 0x26:	/* vibration: amplitude from a position on, frequency w[8] */
		psvr2_pad_trigger_levels(w, lv);
		if (uniform_from(lv, &pos)) {
			set_mode(cmd, PSVR2_TRIG_VIBRATION);
			d[0] = pos;
			d[2] = w[8];
			d[1] = pos < 10 ? lv[pos] : 0;
		} else {
			set_mode(cmd, PSVR2_TRIG_MULTI_VIBRATION);
			memcpy(d + 1, lv, 10);
			d[0] = w[8];
		}
		break;
	default:	/* 0x05 and anything unknown: off */
		set_mode(cmd, PSVR2_TRIG_OFF);
		break;
	}
	param[0] |= (uint8_t)((slot != 0) + 1);
	return 0;
}

/* Trigger travel (0..255) at the ten effect positions. */
static const uint8_t pos_feedback[10] = { 1, 1, 32, 64, 96, 128, 160, 192, 224, 255 };
static const uint8_t pos_vibration[10] = { 8, 8, 39, 71, 103, 135, 167, 199, 231, 255 };
static const uint8_t weapon_start[8] = { 0, 0, 54, 86, 118, 150, 182, 214 };
static const uint8_t weapon_end[9] = { 0, 0, 0, 96, 128, 160, 192, 224, 255 };

static uint8_t zone_state(const uint8_t *w, uint8_t v, const uint8_t pos[10], uint8_t base)
{
	int zone = -1;
	for (int n = 0; n < 10 && v >= pos[n]; n++)
		zone = n;
	if (zone < 0)
		return base;
	return (uint8_t)(base + (w[zone >> 3] >> (zone & 7) & 1));
}

/* First two set bits of the position mask in [lo, hi). */
static void two_marks(const uint8_t *w, int lo, int hi, uint8_t none, uint8_t *a, uint8_t *b)
{
	*a = *b = none;
	for (int n = lo; n < hi; n++)
		if (w[n >> 3] >> (n & 7) & 1) {
			if (*a != none) {
				*b = (uint8_t)n;
				return;
			}
			*a = (uint8_t)n;
		}
}

uint8_t psvr2_pad_trigger_state(uint8_t m, const uint8_t *w, uint8_t v, uint8_t prev)
{
	uint8_t a, b;
	switch (m) {
	case 0x21:	/* 1 outside the zone, 2 inside */
		return zone_state(w, v, pos_feedback, 1);
	case 0x26:	/* 6 / 7 */
		return zone_state(w, v, pos_vibration, 6);
	case 0x25:	/* 3 before start, 4 pulling, 5 released */
		two_marks(w, 2, 9, 9, &a, &b);
		if (a >= 8 || b >= 9 || a >= b)
			return 0;
		if (prev == 5)
			return v >= weapon_start[a] ? 5 : 3;
		if (v < weapon_start[a])
			return 3;
		return v >= weapon_end[b] ? 5 : 4;
	case 0x22:	/* 2 between the two marks, else 1 */
		two_marks(w, 0, 10, 10, &a, &b);
		if (a >= 9 || b >= 10 || a >= b)
			return 0;
		return (v < pos_feedback[a] || v > pos_feedback[b]) ? 1 : 2;
	}
	return 0;
}

/* n (<= 32) bits starting at bit offset bit >= 0, LSB first; sign extends
 * from the last bit read when sign is set. */
uint32_t psvr2_pad_read_bits(const uint8_t *p, int bit, int n, int sign)
{
	uint32_t v = 0, b = 0;
	for (int i = 0; i < n; i++, bit++) {
		b = p[bit >> 3] >> (bit & 7) & 1;
		v |= b << i;
	}
	if (sign && b && n < 32)
		v |= ~0u << n;
	return v;
}

/* D-pad nibble (8 = released): up 0x10, right 0x20, down 0x40, left 0x80.
 * Down-left, left and up-left replace the field instead of adding to it. */
static void dpad(uint32_t *f, uint8_t h)
{
	static const uint8_t bits[8] = { 0x10, 0x30, 0x20, 0x60, 0x40, 0xc0, 0x80, 0x90 };
	if (h < 5)
		*f |= bits[h];
	else if (h < 8)
		*f = bits[h];
}

static void set_bits(uint32_t *f, uint8_t byte, const uint32_t map[8])
{
	for (int i = 0; i < 8; i++)
		if (byte >> i & 1)
			*f |= map[i];
}

/* DualSense input report 0x01 (off 0) or 0x31 (off 1). Adds to buttons
 * and edge_buttons; other fields are overwritten. */
void psvr2_ds_input_decode(const uint8_t *report, uint8_t off, int edge, psvr2_sense_input *o)
{
	static const uint32_t b8[8] = { 0, 0, 0, 0, 0x8000, 0x4000, 0x2000, 0x1000 };
	static const uint32_t b9[8] = { 0x400, 0x800, 0x100, 0x200, 0x1, 0x8, 0x2, 0x4 };
	static const uint32_t b10[8] = { 0x10000, 0x100000, 0x200000, 0, 0, 0, 0, 0 };
	static const uint32_t e34[8] = { 0, 0, 0, 0, 0, 0x10000, 0x1, 0x8 };
	const uint8_t *r = report + off;

	dpad(&o->buttons, r[8] & 0xf);
	set_bits(&o->buttons, r[8], b8);
	set_bits(&o->buttons, r[9], b9);
	set_bits(&o->buttons, r[10], b10);
	o->lx = r[1];
	o->ly = r[2];
	o->rx = r[3];
	o->ry = r[4];
	o->l2 = r[5];
	o->r2 = r[6];
	o->counter = r[7];
	o->battery = r[0x35];
	o->status[0] = r[0x36];
	o->status[1] = r[0x37];
	o->imu_ticks = rd32(r + 0x1c);
	for (int i = 0; i < 3; i++) {
		o->gyro[i] = rd16(r + 0x10 + 2 * i);
		o->accel[i] = rd16(r + 0x16 + 2 * i);
	}
	o->temperature = r[0x20];
	o->touch_counter = 0;
	for (int i = 0; i < 2; i++) {
		o->touch_id[i] = r[0x21 + 4 * i];
		o->touch_xy[i][0] = (uint16_t)psvr2_pad_read_bits(r + 0x22 + 4 * i, 0, 12, 0);
		o->touch_xy[i][1] = (uint16_t)psvr2_pad_read_bits(r + 0x23 + 4 * i, 4, 12, 0);
	}
	o->trig_state_l = r[0x2a];
	o->trig_state_r = r[0x2b];
	o->trig_mode = r[0x30];
	if (!edge)
		return;
	dpad(&o->edge_buttons, r[0x33] & 0xf);
	set_bits(&o->edge_buttons, r[0x33], b8);
	set_bits(&o->edge_buttons, r[0x34], e34);
	const uint8_t prof = r[0x31] & 3;
	o->edge_b31_7 = r[0x31] >> 7;
	o->edge_b31_4 = r[0x31] >> 4 & 7;
	o->edge_b31_3 = r[0x31] >> 3 & 1;
	o->edge_b31_2 = r[0x31] >> 2 & 1;
	o->edge_profile = prof;
	o->edge_b34_1 = r[0x34] >> 1 & 3;
	memset(o->edge_axes, 0x80, 8);
	if (prof) {
		if (prof != 3 && !(r[0x32] & 0xf))
			memcpy(o->edge_axes, r + 0x10, 8);
		memcpy(o->edge_axes + 8, r + 0x18, 4);
	}
}
