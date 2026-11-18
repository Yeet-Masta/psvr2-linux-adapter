// SPDX-License-Identifier: GPL-2.0
/*
 * Lens geometry and hidden-area mesh, see docs/sony-re/steamvr.md
 * ("Lens geometry", "Hidden-area mesh").
 */
#include <math.h>
#include <string.h>
#include "psvr2_steamvr.h"

static float f32(uint32_t bits)
{
	float f;

	memcpy(&f, &bits, 4);
	return f;
}

static float calib_f(const uint8_t *c, int off)
{
	float f;

	memcpy(&f, c + off, 4);
	return f;
}

/* Calibration offsets are panel pixels; one pixel is 29.9 um, measured from
 * the pixel centre. The panel centre sits 3.22 mm from the lens axis and the
 * normalised half-width is 32.462 mm. */
#define PITCH_UM	29.9f
#define HALF_PITCH_UM	14.95f
#define LENS_SHIFT_MM	3.22f
#define HALF_WIDTH_MM	f32(0x4201d914)	/* 32.462 */

/* Sign flip that the compiler may not fold into a later multiply: the
 * reference flips the sign bit first, which shows in the sign of NaNs. */
static float negf(float x)
{
	x = -x;
	__asm__("" : "+x"(x));
	return x;
}

static float lens_coord(float px, float shift_mm)
{
	return ((px * PITCH_UM + HALF_PITCH_UM) / 1000.0f + shift_mm) / HALF_WIDTH_MM;
}

int psvr2_steamvr_lens_update(psvr2_lens_state *st, const uint8_t calib[256])
{
	psvr2_lens_geom *g = &st->geom;

	if (!memcmp(st->calib, calib, 256))
		return 0;
	memcpy(st->calib, calib, 256);
	if (calib[0] < 4) {
		memset(g, 0, sizeof(*g));
		g->centre_l[0] = f32(0xbdcb25aa);
		g->centre_r[0] = f32(0x3dcb25aa);
		return 1;
	}
	const float lx = calib_f(calib, 0x08), ly = calib_f(calib, 0x0c);
	const float rx = calib_f(calib, 0x10), ry = calib_f(calib, 0x14);
	const float lrot = calib_f(calib, 0x18), rrot = calib_f(calib, 0x1c);
	const float sep = calib_f(calib, 0x20);

	g->centre_l[0] = lens_coord(negf(lx) - sep, -LENS_SHIFT_MM);
	g->centre_l[1] = lens_coord(negf(ly), 0.0f);
	g->centre_r[0] = lens_coord(negf(rx) + sep, LENS_SHIFT_MM);
	g->centre_r[1] = lens_coord(negf(ry), 0.0f);
	const float al = (float)((double)negf(lrot) * 3.141592653589793 / 180.0);
	const float ar = (float)((double)negf(rrot) * 3.141592653589793 / 180.0);
	g->rot_l[0] = cosf(al);
	g->rot_l[1] = sinf(al);
	g->rot_r[0] = cosf(ar);
	g->rot_r[1] = sinf(ar);
	return 1;
}

/* Hidden-area constants (normalised lens space and texture space). */
#define SEG_STEP	f32(0x3e490fdb)	/* 2*pi/32 */
#define R_START		f32(0x3f666666)	/* 0.9 */
#define R_STEP		f32(0x3b83126f)	/* 0.004 */
#define R2_MAX		f32(0x3f87ae14)	/* 1.06 */
#define X_ASPECT	f32(0x3f7afafb)	/* 0.98039 = 50/51 */
#define R_SCALE		f32(0x3f7082fd)
#define Y_BIAS		f32(0x39717472)
#define LUT_SLOPE	f32(0x3b9f5600)	/* past the table end */
#define LUT_LAST	f32(0x3fd8b389)
#define U_SCALE		f32(0x3eb794b5)
#define U_BIAS		f32(0x3f290e95)
#define V_SCALE		f32(0x3ec0a0f8)
#define CORNER		f32(0x3f8ccccd)	/* 1.1 */

static float lut_scale(const float *lut, float r2)
{
	const float t = r2 * 1023.0f;
	int i = (int)floorf(t);

	if (i < 0)
		i = 0;
	if (i + 1 < PSVR2_LENS_LUT_SIZE) {
		const float fr = t - (float)i;

		return (1.0f - fr) * lut[i] + fr * lut[i + 1];
	}
	return (t - 1023.0f) * LUT_SLOPE + LUT_LAST;
}

/* March outward along one direction until the distorted point leaves the
 * panel; the stored point is the first one outside (in -1..1 clip space). */
static void march(const psvr2_lens_geom *g, const float *lut, float a, float out[2])
{
	const float c = cosf(a), s = sinf(a);

	for (float r = R_START;; r += R_STEP) {
		const float y = s * r * R_SCALE + g->centre_l[1];
		const float x = c * r * X_ASPECT * R_SCALE + g->centre_l[0];
		const float r2 = y * y + x * x;

		if (r2 > R2_MAX)
			return;
		const float k = lut_scale(lut, r2);
		const float dy = (y - Y_BIAS) * k, dx = (x + 0.0f) * k;
		const float u = (g->rot_l[0] * dx - dy * g->rot_l[1]) * U_SCALE + U_BIAS;
		const float v = (dy * g->rot_l[0] + g->rot_l[1] * dx) * V_SCALE + 0.5f;

		out[0] = (u + u) - 1.0f;
		out[1] = (v + v) - 1.0f;
		if (v > 1.0f || v < 0.0f || u > 1.0f || u < 0.0f)
			return;
	}
}

static void put(float v[][2], int i, float x, float y)
{
	v[i][0] = x;
	v[i][1] = y;
}

int psvr2_steamvr_hidden_area(const psvr2_lens_geom *g, const float lut[PSVR2_LENS_LUT_SIZE],
			      float v[PSVR2_HIDDEN_AREA_MAX][2])
{
	/* Clip-space corner of each quadrant, counter-clockwise from +x+y. */
	static const signed char quad[5][2] = { { 1, 1 }, { -1, 1 }, { -1, -1 }, { 1, -1 }, { 1, 1 } };
	int n = 0;

	for (int seg = 0; seg < 32; seg++) {
		const int q = seg >> 3;

		march(g, lut, (float)seg * SEG_STEP, v[n]);
		march(g, lut, (float)(seg + 1) * SEG_STEP, v[n + 1]);
		put(v, n + 2, quad[q][0] * CORNER, quad[q][1] * CORNER);
		n += 3;
		/* First segment of a quadrant: also fill the wedge between the
		 * previous quadrant's corner and this one. */
		if ((seg & 7) == 0) {
			const int p = (q + 3) & 3;

			put(v, n, v[n - 3][0], v[n - 3][1]);
			put(v, n + 1, quad[p][0] * CORNER, quad[p][1] * CORNER);
			put(v, n + 2, quad[q][0] * CORNER, quad[q][1] * CORNER);
			n += 3;
		}
	}
	for (int i = 0; i < n; i++) {
		v[i][0] = (v[i][0] + 1.0f) * 0.5f;
		v[i][1] = (v[i][1] + 1.0f) * 0.5f;
	}
	return n;
}

void psvr2_steamvr_hidden_area_mirror(float v[][2], int n)
{
	for (int i = 0; i < n; i++)
		v[i][0] = 1.0f - v[i][0];
}
