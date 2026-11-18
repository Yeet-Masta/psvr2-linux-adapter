// SPDX-License-Identifier: GPL-2.0
/*
 * PSVR2 lens model: distortion and hidden-area mesh, written from the
 * behaviour of Sony's PC driver (docs/sony-re/display.md 5.2 and 5.3,
 * docs/sony-re/steamvr.md "Hidden-area mesh"). The radial tables are Sony's
 * measured lens data (psvr2_lens_lut.h, scripts/sony-re/extract_lens_lut.py);
 * the constants are Sony's float values, given as bit patterns.
 *
 * params[8]: left centre x, y; right centre x, y; left rotation cos, sin;
 * right rotation cos, sin (see psvr2_get_lens_distortion()).
 */
#include <math.h>
#include <string.h>

#include "libpsvr2.h"
#include "psvr2_lens_lut.h"

static float bits(uint32_t b)
{
	float f;

	memcpy(&f, &b, sizeof(f));
	return f;
}

#define LENS_S		bits(0x3f7082fd)	/* 0.93950, lens radius scale */
#define LENS_A		bits(0x3f7afafb)	/* 50/51, 2000 x 2040 aspect */
#define LENS_Y_OFF	0.0002302693f		/* red/green vertical offset */
#define LENS_FX		bits(0x3eb794b5)	/* 0.35856 */
#define LENS_FY		bits(0x3ec0a0f8)	/* 0.37623 */
#define LENS_U_LEFT	bits(0x3f290e95)	/* 0.66038 */
#define LENS_U_RIGHT	bits(0x3eade2d7)	/* 0.33962 */

static const uint32_t lens_slope_bits[3] = { 0x3b6f1e00, 0x3b9f5600, 0x3bf35000 };
static const uint32_t lens_end_bits[3] = { 0x3fcbd9db, 0x3fd8b389, 0x3febbf26 };

/* Radial scale for colour ch at t = r^2 * 1023. */
static float lens_scale(int ch, float t)
{
	int i = (int)floorf(t);

	if (i < 0)
		i = 0;
	if (i + 1 < 1024) {
		const float f = t - (float)i;

		return f * bits(psvr2_lens_lut_bits[ch][i + 1]) +
		       (1.0f - f) * bits(psvr2_lens_lut_bits[ch][i]);
	}
	return (t - 1023.0f) * bits(lens_slope_bits[ch]) + bits(lens_end_bits[ch]);
}

void psvr2_distort(const float params[8], int eye, float u, float v,
		   float out[6])
{
	const float cx = params[eye ? 2 : 0], cy = params[eye ? 3 : 1];
	const float rc = params[eye ? 6 : 4], rs = params[eye ? 7 : 5];
	const float ub = eye ? LENS_U_RIGHT : LENS_U_LEFT;
	const float x = ((u + u) - 1.0f) * LENS_A * LENS_S + cx;
	const float y = cy + ((v + v) - 1.0f) * LENS_S;
	const float t = (y * y + x * x) * 1023.0f;
	int ch;

	for (ch = 0; ch < 3; ch++) {
		const float k = lens_scale(ch, t);
		const float dy = (ch == 2 ? y + 0.0f : y - LENS_Y_OFF) * k;
		const float dx = (x + 0.0f) * k;

		out[2 * ch] = (rc * dx - dy * rs) * LENS_FX + ub;
		out[2 * ch + 1] = (dx * rs + rc * dy) * LENS_FY + 0.5f;
	}
}

/*
 * Hidden-area mesh: 32 rays from the left lens centre, each marched out to
 * the first point that lands off the panel, joined to the corners of the
 * clip square into 36 triangles. Right eye = left mirrored.
 */
int psvr2_hidden_area(const float params[8], int eye,
		      float verts[PSVR2_HIDDEN_AREA_VERTS][2])
{
	static const float corner[4][2] = {
		{ 1.1f, 1.1f }, { -1.1f, 1.1f }, { -1.1f, -1.1f }, { 1.1f, -1.1f },
	};
	const float cx = params[0], cy = params[1], rc = params[4], rs = params[5];
	const float step = bits(0x3e490fdb);	/* 2 pi / 32 */
	float ray[33][2];
	int k, n = 0, i;

	for (k = 0; k < 32; k++) {
		const float a = (float)k * step, ca = cosf(a), sa = sinf(a);
		float r;

		ray[k][0] = ray[k][1] = 0.0f;
		for (r = 0.9f; r < 4.0f; r += 0.004f) {
			const float y = sa * r * LENS_S + cy;
			const float x = ca * r * LENS_A * LENS_S + cx;
			float kk, dx, dy, uu, vv;

			if (!(y * y + x * x <= 1.06f))
				break;
			kk = lens_scale(1, (y * y + x * x) * 1023.0f);
			dy = (y - LENS_Y_OFF) * kk;
			dx = (x + 0.0f) * kk;
			uu = (rc * dx - dy * rs) * LENS_FX + LENS_U_LEFT;
			vv = (dy * rc + rs * dx) * LENS_FY + 0.5f;
			ray[k][0] = 2.0f * uu - 1.0f;
			ray[k][1] = 2.0f * vv - 1.0f;
			if (uu < 0.0f || uu > 1.0f || vv < 0.0f || vv > 1.0f)
				break;
		}
	}
	ray[32][0] = ray[0][0];
	ray[32][1] = ray[0][1];

	for (k = 0; k < 32; k++) {
		const int q = k / 8;

		memcpy(verts[n++], ray[k], sizeof(ray[k]));
		memcpy(verts[n++], ray[k + 1], sizeof(ray[k]));
		memcpy(verts[n++], corner[q], sizeof(corner[q]));
		if (k % 8 == 0) {
			memcpy(verts[n++], ray[k], sizeof(ray[k]));
			memcpy(verts[n++], corner[(q + 3) % 4], sizeof(corner[q]));
			memcpy(verts[n++], corner[q], sizeof(corner[q]));
		}
	}
	for (i = 0; i < n; i++) {
		verts[i][0] = (verts[i][0] + 1.0f) * 0.5f;
		verts[i][1] = (verts[i][1] + 1.0f) * 0.5f;
		if (eye)
			verts[i][0] = 1.0f - verts[i][0];
	}
	return n;
}
