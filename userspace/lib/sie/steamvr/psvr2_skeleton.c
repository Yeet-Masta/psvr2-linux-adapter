// SPDX-License-Identifier: GPL-2.0
/*
 * Hand skeleton bind pose, see docs/sony-re/steamvr-and-display.md part A ("Hand skeleton").
 */
#include <math.h>
#include <string.h>
#include "psvr2_steamvr.h"
#include "psvr2_quat.h"

/* Rotation of rad radians about a unit axis (half angle in float). */
static void axis_rad(float rad, float ax, float ay, float az, float q[4])
{
	const float h = rad * 0.5f;
	const float s = sinf(h);

	q[0] = cosf(h);
	q[1] = s * ax;
	q[2] = s * ay;
	q[3] = s * az;
}

/* Same, in degrees; the conversion goes through double like the reference. */
static void axis_angle(float deg, float ax, float ay, float az, float q[4])
{
	axis_rad((float)((double)(deg / 180.0f) * 3.141592653589793), ax, ay, az, q);
}

static float f32(uint32_t b)
{
	float f;

	memcpy(&f, &b, sizeof f);
	return f;
}

/* Rotation from three angles in radians (a, b, c), evaluated in double. */
static void euler(float a, float b, float c, float q[4])
{
	const double hc = c * 0.5f, ha = a * 0.5f, hb = b * 0.5f;
	const double cc = cos(hc), sc = sin(hc);
	const double ca = cos(ha), sa = sin(ha);
	const double cb = cos(hb), sb = sin(hb);

	q[0] = (float)(ca * cc * cb + sa * sc * sb);
	q[1] = (float)(sa * cc * cb - ca * sc * sb);
	q[2] = (float)(ca * cc * sb + sa * sc * cb);
	q[3] = (float)(ca * sc * cb - sa * cc * sb);
}

/* Spherical interpolation from a towards b by t (shortest arc; linear when
 * the two are within about 0.8 degrees). */
static void slerp(const float a[4], float t, const float b_in[4], float o[4])
{
	float b[4] = { b_in[0], b_in[1], b_in[2], b_in[3] };
	float d = ((a[1] * b[1] + a[2] * b[2]) + a[3] * b[3]) + a[0] * b[0];
	float k0 = 1.0f - t, k1 = t;

	if (d < 0.0f) {
		for (int i = 0; i < 4; i++)
			b[i] = -b[i];
		d = -d;
	}
	if (d <= f32(0x3f7ff972)) {
		const float s = sqrtf(1.0f - d * d);
		const float ang = atan2f(s, d);

		k0 = sinf(k0 * ang) * (1.0f / s);
		k1 = sinf(ang * t) * (1.0f / s);
	}
	o[0] = a[0] * k0 + b[0] * k1;
	o[1] = b[1] * k1 + a[1] * k0;
	o[2] = b[2] * k1 + a[2] * k0;
	o[3] = b[3] * k1 + a[3] * k0;
}

static void identity(psvr2_bone *b)
{
	memset(b, 0, sizeof *b);
	b->q[0] = 1.0f;
	b->scale[0] = b->scale[1] = b->scale[2] = 1.0f;
}

void psvr2_steamvr_skel_bind(int side, const float *table, psvr2_bone *out)
{
	const float *row = table + side * (PSVR2_SKEL_BONES * 6);

	for (int i = 0; i < PSVR2_SKEL_BONES; i++, row += 6) {
		float qx[4], qy[4], qz[4], t[4], q[4];

		out[i].pos[0] = row[0] * -1.0f;
		out[i].pos[1] = row[1];
		out[i].pos[2] = row[2];
		axis_angle(row[3], 1.0f, 0.0f, 0.0f, qx);
		axis_angle(row[4], 0.0f, 1.0f, 0.0f, qy);
		axis_angle(row[5], 0.0f, 0.0f, 1.0f, qz);
		psvr2_qmul(qy, qx, t);
		psvr2_qmul(t, qz, q);
		out[i].q[0] = q[0];
		out[i].q[1] = q[1];
		out[i].q[2] = q[2] * -1.0f;
		out[i].q[3] = q[3] * -1.0f;
	}
}

/* Open-hand ("without controller") finger curl. Writes the rotation of the
 * index, middle, ring and little finger bones in the offset pose. */
void psvr2_steamvr_skel_open_curl(int side, const psvr2_skel_input *in, psvr2_bone *off)
{
	static const float max_deg[4] = { 74.0f, 72.0f, 70.0f, 70.0f };
	static const float splay_deg[3] = { -10.0f, -7.0f, -5.0f };
	const int s = side == 0 ? 0 : 1;
	const float index = ((float)in->trig_prox[s] / 255.0f) * 0.7f +
			    ((float)in->trigger[s] / 255.0f) * 0.3f;
	const unsigned l1 = (in->buttons >> (s ? 11 : 10)) & 1;
	const float grip = (float)l1 * 0.2f + ((float)in->grip_prox[s] / 255.0f) * 0.8f;
	const float k[3] = { 0.6f, 0.4f, 0.2f };
	float curl[4];

	curl[0] = index;
	for (int f = 0; f < 3; f++)
		curl[f + 1] = (1.0f - index * k[f]) * grip + index * k[f];

	for (int f = 0; f < 4; f++) {
		const int first = 7 + f * 5;
		float q[4];

		axis_angle(curl[f] * max_deg[f], 0.0f, 0.0f, 1.0f, q);
		for (int b = first; b < first + 4; b++) {
			memcpy(off[b].q, q, sizeof q);
			if (b == first && f < 3) {
				const float deg = curl[f] * splay_deg[f];
				float a[4];

				axis_angle(deg, 0.0f, 1.0f, 0.0f, a);
				psvr2_qmul(a, off[b].q, off[b].q);
				axis_angle(deg, 1.0f, 0.0f, 0.0f, a);
				psvr2_qmul(a, off[b].q, off[b].q);
			}
		}
	}
}

/* Open-hand thumb pose. Resets the whole offset pose to identity first.
 * Thumb bones: 2 = metacarpal, 3 = proximal, 4 = middle, 5 = distal. */
void psvr2_steamvr_skel_open_thumb(int side, const psvr2_skel_input *in, int stick_centred, psvr2_bone *off)
{
	const float sx = ((float)in->stick[0] / 255.0f + (float)in->stick[0] / 255.0f) - 1.0f;
	const float sy = 1.0f - ((float)in->stick[1] / 255.0f + (float)in->stick[1] / 255.0f);
	const int sh_a = side == 0 ? 15 : 14;	/* bit A */
	const int sh_b = side == 0 ? 12 : 13;	/* bit B */
	const int sh_c = side == 0 ? 1 : 2;	/* bit C */
	const float yaw0 = side == 0 ? -5.0f : 5.0f;
	float q[4];

	for (int i = 0; i < PSVR2_SKEL_BONES; i++)
		identity(&off[i]);

	if ((in->touch >> sh_b) & 1) {
		axis_rad(f32(0x3ea0d97c), 1.0f, 0.0f, 0.0f, off[2].q);	/* 18 deg */
		axis_rad(f32(0x3eb2b8c2), 0.0f, 0.0f, 1.0f, off[4].q);	/* 20 deg */
		memcpy(off[5].q, off[4].q, sizeof q);
	} else if ((in->touch >> sh_a) & 1) {
		axis_rad(f32(0x3db2b8c2), 1.0f, 0.0f, 0.0f, off[2].q);	/* 5 deg */
		axis_rad(f32(0x3f060a92), 0.0f, 0.0f, 1.0f, off[3].q);	/* 30 deg */
		axis_rad(f32(0x3f685697), 0.0f, 0.0f, 1.0f, off[4].q);	/* 52 deg */
		memcpy(off[5].q, off[4].q, sizeof q);
	} else if (((in->touch >> sh_c) & 1) || !stick_centred) {
		axis_angle(-sx * 8.0f - yaw0, 0.0f, 1.0f, 0.0f, off[2].q);
		axis_angle(sy * 6.0f + 10.0f, 1.0f, 0.0f, 0.0f, q);
		psvr2_qmul(q, off[2].q, off[2].q);
		axis_angle(66.0f - sy * 20.0f, 0.0f, 0.0f, 1.0f, q);
		psvr2_qmul(q, off[4].q, off[4].q);
		psvr2_qmul(q, off[5].q, off[5].q);
	}
	if (((in->buttons >> sh_b) | (in->buttons >> sh_a) | (in->buttons >> sh_c)) & 1) {
		axis_rad(f32(0x3d567750), 1.0f, 0.0f, 0.0f, q);	/* 3 deg */
		psvr2_qmul(q, off[2].q, off[2].q);
	}
}

/* With-controller index and middle finger pose. Returns a mask of the bones
 * whose blend restarts (the trigger-proximity crossed its threshold). */
uint32_t psvr2_steamvr_skel_ctrl_fingers(int side, const psvr2_skel_input *in, const psvr2_skel_input *prev,
					 psvr2_bone *off)
{
	const int s = side == 0 ? 0 : 1;
	const float on = f32(0x3ca0a0a1);	/* 5/255 */
	const float prox = (float)in->trig_prox[s] / 255.0f;
	const float prev_prox = (float)prev->trig_prox[s] / 255.0f;
	const float trig = (float)in->trigger[s] / 255.0f;
	uint32_t restart = 0;

	if ((prev_prox <= on) == (prox > on))
		restart = 7u << 7;
	if (prox > on) {
		euler(0.0f, f32(0x3d8efa35), f32(0xbdfa35dd), off[7].q);	/*  4, -7 deg */
		euler(0.0f, 0.0f, f32(0x3f9e9d93), off[8].q);		/* 71 deg */
		euler(0.0f, 0.0f, f32(0x3edf66f4), off[9].q);		/* 25 deg */
		if (trig > f32(0x3b808081)) {	/* 1/255 */
			static const uint32_t pull[3][2] = {
				{ 0xbd567750, 0xbdfa35dd },	/* -3, -7 deg */
				{ 0, 0x3fb2b8c2 },		/* 80 deg */
				{ 0, 0x3ef1463a },		/* 27 deg */
			};
			for (int i = 0; i < 3; i++) {
				float t[4];

				euler(0.0f, f32(pull[i][0]), f32(pull[i][1]), t);
				slerp(off[7 + i].q, trig, t, off[7 + i].q);
			}
		}
	}

	const unsigned l1 = (in->buttons >> (s ? 11 : 10)) & 1;
	if ((float)in->grip_prox[s] / 255.0f > on && (float)l1 == 1.0f) {
		float q[4];

		axis_rad(f32(0x3c8efa35), 0.0f, 0.0f, 1.0f, q);	/* 1 deg */
		for (int b = 12; b < 16; b++)
			psvr2_qmul(q, off[b].q, off[b].q);
	}
	return restart;
}

static float rad(float deg)
{
	return (float)((double)(deg / 180.0f) * 3.141592653589793);
}

static void set_pos(psvr2_bone *b, float x, float y, float z)
{
	b->pos[0] = x;
	b->pos[1] = y;
	b->pos[2] = z;
}

/* One fixed thumb shape: bone 2/3/4 rotations from angle triples. */
static void thumb_shape(psvr2_bone *off, float a2, float b2, float c2, uint32_t a3, uint32_t b3, uint32_t c3,
			uint32_t b4, uint32_t c4)
{
	euler(a2, b2, c2, off[2].q);
	euler(f32(a3), f32(b3), f32(c3), off[3].q);
	euler(0.0f, f32(b4), f32(c4), off[4].q);
}

/* With-controller thumb pose. Resets the whole offset pose to identity
 * first, then picks one shape from the touch and button bits. */
void psvr2_steamvr_skel_ctrl_thumb(int side, const psvr2_skel_input *in, int stick_centred, psvr2_bone *off)
{
	const int sg = side == 0 ? -1 : 1;
	const float fs = (float)sg;
	const float sx = ((float)in->stick[0] / 255.0f + (float)in->stick[0] / 255.0f) - 1.0f;
	const float sy = 1.0f - ((float)in->stick[1] / 255.0f + (float)in->stick[1] / 255.0f);
	const int sh_a = side == 0 ? 15 : 14, sh_b = side == 0 ? 12 : 13, sh_c = side == 0 ? 1 : 2;
	const int sh_d = side == 0 ? 0 : 3;
	const float p3x = (float)((double)sg * -0.001), p3y = (float)((double)sg * 0.003);
	float t[4];

	for (int i = 0; i < PSVR2_SKEL_BONES; i++)
		identity(&off[i]);

	if ((in->touch >> sh_b) & 1) {
		set_pos(&off[2], 0.0f, f32(0xbb449ba6), f32(0xbc1374bc));
		euler(f32(0xbe8efa35), rad((float)(sg * -21)), rad((float)(sg * 21)), off[2].q);
		euler(f32(0xbf137207), 0.0f, f32(0xbe8efa35), off[3].q);
		euler(0.0f, 0.0f, f32(0x3e567750), off[4].q);
	} else if ((in->touch >> sh_a) & 1) {
		set_pos(&off[2], 0.0f, f32(0xbc03126f), f32(0xbc343958));
		set_pos(&off[3], p3x, p3y, p3y);
		thumb_shape(off, f32(0xbf0192c0), rad((float)(sg * -22)), rad((float)(sg * 29)),
			    0xbf20d97c, 0x3dfa35dd, 0xbc8efa35, 0x3dfa35dd, 0x3f060a92);
	} else if (((in->touch >> sh_c) & 1) || !stick_centred) {
		set_pos(&off[2], 0.0f, f32(0xbc1374bc), f32(0xbbe56042));
		set_pos(&off[3], p3x, p3y, p3y);
		thumb_shape(off, f32(0xbec49809), rad((float)(sg * -29)), rad((float)(sg * 36)),
			    0xbf060a92, 0x3d567750, 0xbd567750, 0x3e0efa35, 0xbf0192c0);
		euler(rad(fabsf(sx)), rad(sx), rad(sx * 14.0f), t);
		psvr2_qmul(t, off[2].q, off[2].q);
		euler(rad(sx * -23.0f * fs - 10.0f), rad(sx * -8.0f * fs - 8.0f), rad(sx * 3.0f * fs), t);
		psvr2_qmul(t, off[3].q, off[3].q);
		euler(rad(sy * 4.0f), 0.0f, 0.0f, t);
		psvr2_qmul(t, off[2].q, off[2].q);
		euler(0.0f, 0.0f, rad(sy * -30.0f), t);
		psvr2_qmul(t, off[4].q, off[4].q);
		euler(0.0f, 0.0f, rad(sy * -19.0f), t);
		psvr2_qmul(t, off[5].q, off[5].q);
	} else if ((in->buttons >> sh_d) & 1) {
		set_pos(&off[2], (float)((double)sg * -0.002), f32(0xbc1374bc), f32(0xbc03126f));
		set_pos(&off[3], (float)((double)sg * 0.001), 0.0f, 0.0f);
		thumb_shape(off, f32(0xbebba866), rad((float)(sg * -32)), rad((float)(sg * 55)),
			    0xbf0efa35, 0x3db2b8c2, 0xbd8efa35, 0x3d567750, 0xbe32b8c2);
	} else if ((in->buttons >> 16) & 1) {
		const float p = (float)((double)sg * 0.001);

		set_pos(&off[2], 0.0f, f32(0xbc656042), f32(0xbba3d70a));
		set_pos(&off[3], 0.0f, p, p);
		thumb_shape(off, f32(0xbf5aef21), rad((float)(sg * -26)), rad((float)(sg * 55)),
			    0xbf9c61aa, 0xbe0efa35, 0x3edf66f4, 0xbdb2b8c2, 0x3f32b8c2);
	}
	if (((in->buttons >> sh_b) | (in->buttons >> sh_a) | (in->buttons >> sh_c)) & 1) {
		axis_rad(f32(0x3c8efa35), 1.0f, 0.0f, 0.0f, t);	/* 1 deg */
		psvr2_qmul(t, off[2].q, off[2].q);
	}
}
