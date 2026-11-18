// SPDX-License-Identifier: GPL-2.0
#include "psvr2_cb_cplx.h"
#include <math.h>
#include <xmmintrin.h>

void psvr2_cb_c_div(float out[2], const float a[2], const float b[2])
{
	const float den = fmaf(b[1], b[1], b[0] * b[0]);

	out[0] = (a[0] * b[0] + a[1] * b[1]) / den;
	out[1] = (a[1] * b[0] - a[0] * b[1]) / den;
}

void psvr2_cb_c_cbrt(float out[2], const float z[2])
{
	const float ang = atan2f(z[1], z[0]) / 3.0f;
	const float r = (float)sqrt((double)z[1] * (double)z[1] + (double)z[0] * (double)z[0]);
	const float m = cbrtf(r);

	out[0] = cosf(ang) * m;
	out[1] = sinf(ang) * m;
}

void psvr2_cb_c_sqrt(float out[2], const float z[2])
{
	const double x = z[0], y = z[1];
	const double mod = sqrt(fma(y, y, x * x));
	double im = sqrt((mod - x) * 0.5);

	if (y != 0.0 && signbit(y))
		im = -im;
	out[0] = (float)sqrt((mod + x) * 0.5);
	out[1] = (float)im;
}

void psvr2_cb_c_polar(float out[2], float r, float angle)
{
	out[0] = cosf(angle) * r;
	out[1] = sinf(angle) * r;
}

static float rcp_est(float x)
{
	return _mm_cvtss_f32(_mm_rcp_ss(_mm_set_ss(x)));
}

static float rsqrt_est(float x)
{
	return _mm_cvtss_f32(_mm_rsqrt_ss(_mm_set_ss(x)));
}

void psvr2_cb_m2_inverse(float out[4], const float m[4])
{
	const float det = fmaf(-m[1], m[2], m[3] * m[0]);
	const float adj[4] = { m[3], -m[1], -m[2], m[0] };
	const float r0 = rcp_est(det);
	/* One Newton step on the reciprocal, then one more folded into the scale. */
	const float r1 = fmaf(-(r0 * det), r0, r0 + r0);
	const float e = r1 * det;

	for (int k = 0; k < 4; k++) {
		const float y = adj[k] * r1;

		out[k] = fmaf(-e, y, y + y);
	}
}

void psvr2_cb_m2_mul_v2(float out[2], const float m[4], const float v[2])
{
	out[0] = v[0] * m[0] + v[1] * m[2];
	out[1] = v[0] * m[1] + v[1] * m[3];
}

void psvr2_cb_v2_normalize_fast(float out[2], const float v[2])
{
	const float s = v[1] * v[1] + v[0] * v[0];
	const float r0 = rsqrt_est(s);
	const float r1 = fmaf(-(r0 * s), r0, 3.0f) * (r0 * 0.5f);
	const float r2 = fmaf(-(r1 * s), r1, 3.0f) * (r1 * 0.5f);

	out[0] = r2 * v[0];
	out[1] = r2 * v[1];
}

void psvr2_cb_v2_outer(float out[4], const float a[2], const float b[2])
{
	out[0] = b[0] * a[0];
	out[1] = b[0] * a[1];
	out[2] = b[1] * a[0];
	out[3] = b[1] * a[1];
}

void psvr2_cb_v4_outer2(float out[12], const float v[4], const float b[2])
{
	for (int k = 0; k < 4; k++) {
		out[k] = b[0] * v[k];
		out[4 + k] = b[1] * v[k];
		out[8 + k] = 0.0f;
	}
}

void psvr2_cb_add12(float a[12], const float b[12])
{
	for (int k = 0; k < 12; k++)
		a[k] = b[k] + a[k];
}

void psvr2_cb_scale12(float a[12], float s)
{
	for (int k = 0; k < 12; k++)
		a[k] = s * a[k];
}

void psvr2_cb_flip_pose(float out[16], const float r[9], const float t[3])
{
	const float c[16] = {
		r[0], r[3], -r[6], 0.0f,
		r[1], r[4], -r[7], 0.0f,
		-r[2], -r[5], r[8], 0.0f,
		-1000.0f * t[0], -1000.0f * t[1], t[2] * 1000.0f, 0.0f,
	};

	for (int k = 0; k < 16; k++)
		out[k] = c[k];
}
