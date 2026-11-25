// SPDX-License-Identifier: GPL-2.0
/* See psvr2_ct_math.h and docs/sony-re/controller-tracking.md part H. */
#include "psvr2_ct_math.h"

#include <math.h>

void psvr2_ct_quat_rotate(float out[3], const float q[4], const float v[3])
{
	const float x = q[0], y = q[1], z = q[2], w = q[3];
	const float px = (v[0] * w + v[2] * y) - v[1] * z;
	const float py = (v[0] * z + v[1] * w) - v[2] * x;
	const float pz = (v[1] * x + v[2] * w) - v[0] * y;
	const float d = (v[0] * x + v[1] * y) + v[2] * z;

	out[0] = ((d * x + px * w) - py * z) + pz * y;
	out[1] = ((d * y + py * w) - pz * x) + px * z;
	out[2] = ((d * z + pz * w) - px * y) + py * x;
}

float psvr2_ct_quat_length(const float q[4])
{
	return sqrtf(((q[0] * q[0] + q[1] * q[1]) + q[2] * q[2]) + q[3] * q[3]);
}

void psvr2_ct_quat_normalize(float out[4], const float q[4])
{
	const float s = 1.0f / psvr2_ct_quat_length(q);

	for (int i = 0; i < 4; i++)
		out[i] = s * q[i];
}

void psvr2_ct_quat_to_mat3(float out[3][4], const float q[4])
{
	const float x = q[0], y = q[1], z = q[2], w = q[3];
	const float X = x + x, Y = y + y, Z = z + z;

	out[0][0] = (1.0f - Y * y) - Z * z;
	out[0][1] = Z * w + Y * x;
	out[0][2] = Z * x - Y * w;
	out[1][0] = Y * x - Z * w;
	out[1][1] = (1.0f - X * x) - Z * z;
	out[1][2] = Z * y + X * w;
	out[2][0] = Y * w + Z * x;
	out[2][1] = Z * y - X * w;
	out[2][2] = (1.0f - X * x) - Y * y;
}

void psvr2_ct_mat4_mul_vec4(float out[4], const float m[4][4], const float v[4])
{
	float r[4];

	for (int i = 0; i < 4; i++)
		r[i] = ((v[1] * m[1][i] + v[0] * m[0][i]) + v[2] * m[2][i]) + v[3] * m[3][i];
	for (int i = 0; i < 4; i++)
		out[i] = r[i];
}

void psvr2_ct_mat43_mul_vec3(float out[4], const float m[3][4], const float v[3])
{
	float r[4];

	for (int i = 0; i < 4; i++)
		r[i] = (v[1] * m[1][i] + v[0] * m[0][i]) + v[2] * m[2][i];
	for (int i = 0; i < 4; i++)
		out[i] = r[i];
}

void psvr2_ct_mat4_inverse(float out[4][4], const float m[4][4])
{
	/* a(r, c): row r, column c */
#define a(r, c) m[c][r]
	const float m17 = a(2, 2) * a(3, 0) - a(3, 2) * a(2, 0);
	const float m18 = a(2, 3) * a(3, 1) - a(3, 3) * a(2, 1);
	const float m19 = a(2, 2) * a(1, 0) - a(1, 2) * a(2, 0);
	const float m20 = a(2, 3) * a(1, 1) - a(1, 3) * a(2, 1);
	const float m21 = a(1, 3) * a(3, 1) - a(3, 3) * a(1, 1);
	const float m22 = a(1, 2) * a(3, 0) - a(3, 2) * a(1, 0);
	const float c0 = (a(1, 2) * m18 - a(3, 2) * m20) - a(2, 2) * m21;
	const float c1 = (a(1, 3) * m17 - a(3, 3) * m19) - a(2, 3) * m22;
	const float c2 = (a(2, 0) * m21 + a(3, 0) * m20) - a(1, 0) * m18;
	const float c3 = (a(2, 1) * m22 + a(3, 1) * m19) - a(1, 1) * m17;
	const float s = 1.0f / (((c1 * a(0, 1) + c0 * a(0, 0)) + c2 * a(0, 2)) + c3 * a(0, 3));
	const float n27 = a(0, 2) * a(1, 0) - a(0, 0) * a(1, 2);
	const float n28 = a(0, 3) * a(1, 1) - a(0, 1) * a(1, 3);
	const float n29 = a(0, 2) * a(3, 0) - a(0, 0) * a(3, 2);
	const float n30 = a(0, 3) * a(3, 1) - a(0, 1) * a(3, 3);
	const float n31 = a(0, 2) * a(2, 0) - a(0, 0) * a(2, 2);
	const float n32 = a(0, 3) * a(2, 1) - a(0, 1) * a(2, 3);
	const float r[4][4] = {
		{ c0, c1, c2, c3 },
		{ (a(2, 2) * n30 - a(3, 2) * n32) - a(0, 2) * m18,
		  (a(2, 3) * n29 - a(3, 3) * n31) - a(0, 3) * m17,
		  (a(3, 0) * n32 - a(2, 0) * n30) + a(0, 0) * m18,
		  (a(3, 1) * n31 - a(2, 1) * n29) + a(0, 1) * m17 },
		{ (a(3, 2) * n28 - a(1, 2) * n30) + a(0, 2) * m21,
		  (a(3, 3) * n27 - a(1, 3) * n29) + a(0, 3) * m22,
		  (a(1, 0) * n30 - a(3, 0) * n28) - a(0, 0) * m21,
		  (a(1, 1) * n29 - a(3, 1) * n27) - a(0, 1) * m22 },
		{ (a(1, 2) * n32 - a(2, 2) * n28) + a(0, 2) * m20,
		  (a(1, 3) * n31 - a(2, 3) * n27) + a(0, 3) * m19,
		  (a(2, 0) * n28 - a(1, 0) * n32) - a(0, 0) * m20,
		  (a(2, 1) * n27 - a(1, 1) * n31) - a(0, 1) * m19 },
	};
#undef a
	for (int c = 0; c < 4; c++)
		for (int i = 0; i < 4; i++)
			out[c][i] = r[c][i] * s;
}

void psvr2_ct_vec3_add_inplace(float a[3], const float b[3])
{
	for (int i = 0; i < 3; i++)
		a[i] = b[i] + a[i];
}

float psvr2_ct_vec3_length(const float v[3])
{
	return sqrtf((v[0] * v[0] + v[1] * v[1]) + v[2] * v[2]);
}

void psvr2_ct_vec3_cross(float out[3], const float a[3], const float b[3])
{
	const float r0 = a[1] * b[2] - a[2] * b[1];
	const float r1 = a[2] * b[0] - a[0] * b[2];
	const float r2 = a[0] * b[1] - a[1] * b[0];

	out[0] = r0;
	out[1] = r1;
	out[2] = r2;
}

void psvr2_ct_quat_mul(float out[4], const float p[4], const float q[4])
{
	const float r0 = ((p[0] * q[3] + p[3] * q[0]) + p[1] * q[2]) - p[2] * q[1];
	const float r1 = ((p[1] * q[3] + p[3] * q[1]) + p[2] * q[0]) - p[0] * q[2];
	const float r2 = ((p[2] * q[3] + p[3] * q[2]) + p[0] * q[1]) - p[1] * q[0];
	const float r3 = ((p[3] * q[3] - p[0] * q[0]) - p[1] * q[1]) - p[2] * q[2];

	out[0] = r0;
	out[1] = r1;
	out[2] = r2;
	out[3] = r3;
}

void psvr2_ct_quat_yaw(float out[4], float angle)
{
	const float h = angle * 0.5f;

	out[0] = 0.0f;
	out[1] = sinf(h);
	out[2] = 0.0f;
	out[3] = cosf(h);
}

void psvr2_ct_mat3_mul(float out[3][4], const float a[3][4], const float b[3][4])
{
	float r[3][3];

	for (int i = 0; i < 3; i++)
		r[0][i] = (b[0][1] * a[1][i] + b[0][0] * a[0][i]) + b[0][2] * a[2][i];
	for (int c = 1; c < 3; c++)
		for (int i = 0; i < 3; i++)
			r[c][i] = (b[c][0] * a[0][i] + b[c][1] * a[1][i]) + b[c][2] * a[2][i];
	for (int c = 0; c < 3; c++)
		for (int i = 0; i < 3; i++)
			out[c][i] = r[c][i];
}

void psvr2_ct_mat3_rot_y(float out[3][4], float angle)
{
	const float s = sinf(angle), c = cosf(angle);

	out[0][0] = c;    out[0][1] = 0.0f; out[0][2] = -s;
	out[1][0] = 0.0f; out[1][1] = 1.0f; out[1][2] = 0.0f;
	out[2][0] = s;    out[2][1] = 0.0f; out[2][2] = c;
}

float psvr2_ct_azimuth_diff(float *az_a, float *az_b, const float a[3], const float b[3])
{
	const float pi = 3.14159274f, two_pi = 6.28318548f;
	const float ta = atan2f(a[2], a[0]);
	const float tb = atan2f(b[2], b[0]);
	float d = ta - tb;

	if (d > pi)
		d = d + -two_pi;
	else if (d < -pi)
		d = d + two_pi;
	*az_a = ta;
	*az_b = tb;
	return d;
}

float psvr2_ct_mwc_uniform(uint32_t state[2], float lo, float hi)
{
	state[0] = (state[0] & 0xffff) * 36969u + (state[0] >> 16);
	state[1] = (state[1] & 0xffff) * 18000u + (state[1] >> 16);
	const uint32_t u = state[1] + (state[0] << 16);
	/* (u + 1) / (2^32 + 2): strictly inside (0, 1). */
	const float t = (float)(((double)u + 1.0) * 0x1.fffffffcp-33);

	return t * (hi - lo) + lo;
}

void psvr2_ct_vec3_sub_inplace(float a[3], const float b[3])
{
	for (int i = 0; i < 3; i++)
		a[i] = a[i] - b[i];
}

void psvr2_ct_vec3_scale_inplace(float a[3], float s)
{
	for (int i = 0; i < 3; i++)
		a[i] = s * a[i];
}

void psvr2_ct_quat_from_to(float out[4], const float a[3], const float b[3])
{
	float d = ((a[0] * b[0] + a[1] * b[1]) + a[2] * b[2]) + 1.0f;
	const float s = sqrtf(d + d);
	const float inv = 1.0f / s;
	const float x = (a[1] * b[2] - a[2] * b[1]) * inv;
	const float y = (a[2] * b[0] - a[0] * b[2]) * inv;
	const float z = (a[0] * b[1] - a[1] * b[0]) * inv;

	out[0] = x;
	out[1] = y;
	out[2] = z;
	out[3] = s * 0.5f;
}

void psvr2_ct_quatd_normalize(double out[4], const double q[4])
{
	const double inv = 1.0 / sqrt(((q[0] * q[0] + q[1] * q[1]) + q[2] * q[2]) + q[3] * q[3]);

	for (int i = 0; i < 4; i++)
		out[i] = q[i] * inv;
}

void psvr2_ct_quatd_mul(double out[4], const double p[4], const double q[4])
{
	const double r0 = ((p[0] * q[3] + p[3] * q[0]) + p[1] * q[2]) - p[2] * q[1];
	const double r1 = ((p[1] * q[3] + p[3] * q[1]) + p[2] * q[0]) - p[0] * q[2];
	const double r2 = ((p[2] * q[3] + p[3] * q[2]) + p[0] * q[1]) - p[1] * q[0];
	const double r3 = ((p[3] * q[3] - p[0] * q[0]) - p[1] * q[1]) - p[2] * q[2];

	out[0] = r0;
	out[1] = r1;
	out[2] = r2;
	out[3] = r3;
}

void psvr2_ct_quat_cap(float out[4], const float v[3], float c, float r)
{
	const float n = (v[0] * v[0] + v[1] * v[1]) + v[2] * v[2];
	const float r2 = r * r;
	const float t = sqrtf((1.0f - c * c) * n + r2);
	const float inv = 1.0f / r;
	const float w = (t * r - n * c) / (r2 + n);
	const float k = w + c;

	out[3] = w;
	for (int i = 0; i < 3; i++)
		out[i] = (v[i] * inv) * k;
}

void psvr2_ct_quat_cap_d(double out[4], const float v[3], double c, double r)
{
	const double n = (double)((v[0] * v[0] + v[1] * v[1]) + v[2] * v[2]);
	const double r2 = r * r;
	const double t = sqrt((1.0 - c * c) * n + r2);
	const double w = (t * r - n * c) / (r2 + n);
	const float k = (float)(w + c);
	const float inv = (float)(1.0 / r);

	for (int i = 0; i < 3; i++)
		out[i] = (double)((v[i] * inv) * k);
	out[3] = w;
}

void psvr2_ct_mat3_to_quat(float out[4], const float m[3][4])
{
#define a(r, c) m[c][r]
	float m00 = a(0, 0), m11 = a(1, 1), m22 = a(2, 2);
	float m01 = a(0, 1), m12 = a(1, 2), m20 = a(2, 0);
	const int neg = 0.0f > (m11 + m00) + m22;
	/* With a negative trace, negate every diagonal element except the
	 * largest one, together with one off-diagonal element each. */
	const int fz = neg && !(m22 > m00 && m22 > m11);
	const int fx = neg && (m11 > m00 || m22 > m00);
	const int fy = neg && !(m11 > m00 && !(m22 > m11));

	if (fz) {
		m22 = -m22;
		m01 = -m01;
	}
	if (fx) {
		m00 = -m00;
		m12 = -m12;
	}
	if (fy) {
		m11 = -m11;
		m20 = -m20;
	}
	const float t = ((m11 + m00) + m22) + 1.0f;
	const float h = 0.5f / sqrtf(t);
	const float d_yz = a(2, 1) - m12;
	const float d_zx = a(0, 2) - m20;
	const float d_xy = a(1, 0) - m01;
#undef a
	/* Lanes in w-largest order, then rotated for the z/y/x-largest cases. */
	const float p0 = fz ? t : d_yz;
	const float p1 = fz ? d_zx : d_xy;
	const float p2 = fz ? d_xy : d_zx;
	const float p3 = fz ? d_yz : t;

	out[0] = (fx ? p2 : p0) * h;
	out[1] = (fx ? p0 : p2) * h;
	out[2] = (fx ? p3 : p1) * h;
	out[3] = (fx ? p1 : p3) * h;
}

float psvr2_ct_quat_rel_yaw(const float p[4], const float q[4])
{
	static const float up[3] = { 0.0f, 1.0f, 0.0f }, ex[3] = { 1.0f, 0.0f, 0.0f };
	const float qc[4] = { -q[0], -q[1], -q[2], q[3] };
	float r[4], u[3], c[4], g[4], e[3], a, b;

	psvr2_ct_quat_mul(r, p, qc);
	psvr2_ct_quat_rotate(u, r, up);
	psvr2_ct_quat_from_to(c, u, up);   /* remove the tilt */
	psvr2_ct_quat_mul(g, c, r);
	psvr2_ct_quat_rotate(e, g, ex);
	return psvr2_ct_azimuth_diff(&a, &b, ex, e);
}

void psvr2_ct_quat_exp(float out[4], const float v[3])
{
	const float len = psvr2_ct_vec3_length(v);
	const float half = len * 0.5f;
	float k;

	/* Below sqrt(FLT_EPSILON) use a series; Sony's has the wrong sign on
	 * the quadratic term, which is below half an ulp of 0.5 there. */
	if (len < sqrtf(1.1920928955078125e-07f))
		k = (len * len) / 48.0f + 0.5f;
	else
		k = sinf(half) / len;
	out[0] = v[0] * k;
	out[1] = v[1] * k;
	out[2] = v[2] * k;
	out[3] = cosf(half);
}

void psvr2_ct_quatd_integrate(double out[4], const double q[4], const float w[3], float dt)
{
	const float v[3] = { dt * w[0], dt * w[1], dt * w[2] };
	float e[4];
	double d[4], r[4];

	psvr2_ct_quat_exp(e, v);
	for (int i = 0; i < 4; i++)
		d[i] = (double)e[i];
	psvr2_ct_quatd_mul(r, q, d);
	psvr2_ct_quatd_normalize(out, r);
}

void psvr2_ct_set_ref_and_diff(float ref_out[4], float diff_out[3], const float cur[3],
			       const float ref[4])
{
	float d[3];

	for (int i = 0; i < 3; i++)
		d[i] = cur[i] - ref[i];
	for (int i = 0; i < 4; i++)
		ref_out[i] = ref[i];
	for (int i = 0; i < 3; i++)
		diff_out[i] = d[i];
}
