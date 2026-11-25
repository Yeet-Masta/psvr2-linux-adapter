// SPDX-License-Identifier: GPL-2.0
/*
 * IVRDisplayComponent behaviour, see docs/sony-re/steamvr-and-display.md part A ("Display").
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

psvr2_rect psvr2_steamvr_window_bounds(uint32_t panel_w, uint32_t panel_h)
{
	return (psvr2_rect){ 0, 0, panel_w, panel_h };
}

/* 1.7x supersampling of one eye, integer arithmetic. */
void psvr2_steamvr_render_target(uint32_t panel_w, uint32_t panel_h, uint32_t *w, uint32_t *h)
{
	*w = (panel_w / 2) * 17 / 10;
	*h = panel_h * 17 / 10;
}

psvr2_rect psvr2_steamvr_eye_viewport(uint32_t panel_w, uint32_t panel_h, int eye)
{
	return (psvr2_rect){ eye ? panel_w / 2 : 0, 0, panel_w / 2, panel_h };
}

void psvr2_steamvr_projection_raw(int eye, float *l, float *r, float *t, float *b)
{
	/* Asymmetric: the outer side is wider (1.84177 vs 0.94719). */
	*l = eye ? f32(0xbf727b16) : f32(0xbfebbf26);
	*r = eye ? f32(0x3febbf26) : f32(0x3f727b16);
	*t = f32(0xbfaa1c0d);
	*b = f32(0x3faa1c0d);
}

#define X_ASPECT	f32(0x3f7afafb)
#define R_SCALE		f32(0x3f7082fd)
#define Y_BIAS		f32(0x39717472)
#define U_SCALE		f32(0x3eb794b5)
#define V_SCALE		f32(0x3ec0a0f8)
#define U_BIAS_L	f32(0x3f290e95)
#define U_BIAS_R	f32(0x3eade2d7)

psvr2_distortion psvr2_steamvr_distort(const psvr2_lens_geom *g, const float *lut_r,
				       const float *lut_g, const float *lut_b,
				       int eye, float u, float v)
{
	/* Past the table end each channel continues linearly. */
	static const uint32_t slope[3] = { 0x3b6f1e00, 0x3b9f5600, 0x3bf35000 };
	static const uint32_t last[3] = { 0x3fcbd9db, 0x3fd8b389, 0x3febbf26 };
	const float *cen = eye ? g->centre_r : g->centre_l;
	const float *rot = eye ? g->rot_r : g->rot_l;
	const float x = ((u + u) - 1.0f) * X_ASPECT * R_SCALE + cen[0];
	const float y = cen[1] + ((v + v) - 1.0f) * R_SCALE;
	const float t = (y * y + x * x) * 1023.0f;
	int i = (int)floorf(t);
	float k[3];

	if (i < 0)
		i = 0;
	if (i + 1 < PSVR2_LENS_LUT_SIZE) {
		const float fr = t - (float)i, w = 1.0f - fr;

		k[0] = fr * lut_r[i + 1] + w * lut_r[i];
		k[1] = fr * lut_g[i + 1] + w * lut_g[i];
		k[2] = fr * lut_b[i + 1] + w * lut_b[i];
	} else {
		const float fr = t - 1023.0f;

		for (int c = 0; c < 3; c++)
			k[c] = fr * f32(slope[c]) + f32(last[c]);
	}
	const float xx = x + 0.0f;
	const float c = rot[0], s = rot[1];
	const float ub = eye ? U_BIAS_R : U_BIAS_L;
	/* Red and green are offset vertically by Y_BIAS, blue is not. */
	const float yr = (y - Y_BIAS) * k[0], yg = (y - Y_BIAS) * k[1], yb = (y + 0.0f) * k[2];
	psvr2_distortion d;

	d.rgb[0][0] = (c * (xx * k[0]) - yr * s) * U_SCALE + ub;
	d.rgb[0][1] = (xx * k[0] * s + c * yr) * V_SCALE + 0.5f;
	d.rgb[1][0] = (c * (xx * k[1]) - yg * s) * U_SCALE + ub;
	d.rgb[1][1] = (xx * k[1] * s + c * yg) * V_SCALE + 0.5f;
	d.rgb[2][0] = (c * (k[2] * xx) - yb * s) * U_SCALE + ub;
	d.rgb[2][1] = (k[2] * xx * s + c * yb) * V_SCALE + 0.5f;
	return d;
}
