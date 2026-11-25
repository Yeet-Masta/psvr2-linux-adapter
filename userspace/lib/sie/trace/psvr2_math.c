// SPDX-License-Identifier: GPL-2.0
/*
 * Shared geometry helpers; see psvr2_math.h and docs/sony-re/vrtrace.md.
 * Sums are evaluated in index order (k = 0, 1, 2, ...), which is what makes
 * the results bit-identical; keep that order when editing.
 */
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "psvr2_math.h"

void psvr2_math_mat3_mul(const float a[9], const float b[9], float out[9])
{
	for (int c = 0; c < 3; c++)
		for (int r = 0; r < 3; r++)
			out[c * 3 + r] = a[r] * b[c * 3] + a[3 + r] * b[c * 3 + 1] + a[6 + r] * b[c * 3 + 2];
}

void psvr2_math_mat3_mul_vec3(const float m[9], const float v[3], float out[3])
{
	float x = v[0], y = v[1], z = v[2];

	for (int r = 0; r < 3; r++)
		out[r] = m[r] * x + m[3 + r] * y + m[6 + r] * z;
}

void psvr2_math_mat4_mul(const float a[16], const float b[16], float out[16])
{
	for (int c = 0; c < 4; c++)
		for (int r = 0; r < 4; r++)
			out[c * 4 + r] = a[r] * b[c * 4] + a[4 + r] * b[c * 4 + 1] +
					 a[8 + r] * b[c * 4 + 2] + a[12 + r] * b[c * 4 + 3];
}

void psvr2_math_mat4_mul_inplace(float a[16], const float b[16])
{
	float t[16];

	psvr2_math_mat4_mul(a, b, t);
	memcpy(a, t, sizeof(t));
}

void psvr2_math_quat_to_mat4(const float q[4], float m[16])
{
	float n = sqrtf(q[1] * q[1] + q[2] * q[2] + q[3] * q[3] + q[0] * q[0]);
	float w = q[0] / n, x = q[1] / n, y = q[2] / n, z = q[3] / n;
	float w2 = w + w, x2 = x + x;
	float yy2 = (y + y) * y, zz2 = (z + z) * z, yz2 = (y + y) * z;
	float one_xx2 = 1.0f - x2 * x;

	m[0] = (1.0f - yy2) - zz2;
	m[1] = w2 * z + x2 * y;
	m[2] = x2 * z - w2 * y;
	m[4] = x2 * y - w2 * z;
	m[5] = one_xx2 - zz2;
	m[6] = w2 * x + yz2;
	m[8] = x2 * z + w2 * y;
	m[9] = yz2 - w2 * x;
	m[10] = one_xx2 - yy2;
	m[3] = m[7] = m[11] = 0.0f;
	m[12] = m[13] = m[14] = 0.0f;
	m[15] = 1.0f;
}

void psvr2_math_pose_to_mat4(const float p[10], float out[16])
{
	float r[16], s[16] = { 0 };

	memset(out, 0, 16 * sizeof(float));
	out[0] = out[5] = out[10] = out[15] = 1.0f;
	out[12] = p[0];
	out[13] = p[1];
	out[14] = p[2];
	psvr2_math_quat_to_mat4(p + 3, r);
	s[0] = p[7];
	s[5] = p[8];
	s[10] = p[9];
	s[15] = 1.0f;
	psvr2_math_mat4_mul_inplace(out, r);
	psvr2_math_mat4_mul_inplace(out, s);
}

void psvr2_math_pose_to_mat4_inv(const float p[10], float out[16])
{
	float r[16], rt[16], t[16] = { 0 };

	memset(out, 0, 16 * sizeof(float));
	out[0] = 1.0f / p[7];
	out[5] = 1.0f / p[8];
	out[10] = 1.0f / p[9];
	out[15] = 1.0f;
	psvr2_math_quat_to_mat4(p + 3, r);
	for (int c = 0; c < 4; c++)
		for (int k = 0; k < 4; k++)
			rt[c * 4 + k] = r[k * 4 + c];
	t[0] = t[5] = t[10] = t[15] = 1.0f;
	t[12] = -p[0];
	t[13] = -p[1];
	t[14] = -p[2];
	psvr2_math_mat4_mul_inplace(out, rt);
	psvr2_math_mat4_mul_inplace(out, t);
}

void psvr2_math_pose_identity(float p[10])
{
	static const float id[10] = { 0, 0, 0, 1, 0, 0, 0, 1, 1, 1 };

	memcpy(p, id, sizeof(id));
}

void psvr2_math_euler_to_quat(float q[4], float a, float b, float c)
{
	double cc = cos((double)(c * 0.5f)), sc = sin((double)(c * 0.5f));
	double ca = cos((double)(a * 0.5f)), sa = sin((double)(a * 0.5f));
	double cb = cos((double)(b * 0.5f)), sb = sin((double)(b * 0.5f));

	q[0] = (float)(ca * cc * cb + sa * sc * sb);
	q[1] = (float)(sa * cc * cb - ca * sc * sb);
	q[2] = (float)(ca * cc * sb + sa * sc * cb);
	q[3] = (float)(ca * sc * cb - sa * cc * sb);
}

void psvr2_math_frustum(float m[16], float l, float r, float b, float t, float n, float f)
{
	memset(m, 0, 16 * sizeof(float));
	m[0] = (n + n) / (r - l);
	m[5] = (n + n) / (t - b);
	m[8] = (l + r) / (r - l);
	m[9] = (b + t) / (t - b);
	m[10] = -(n + f) / (f - n);
	m[11] = -1.0f;
	m[14] = (f * -2.0f * n) / (f - n);
}

void psvr2_math_mat4_mul_vec4(const float m[16], const float v[4], float out[4])
{
	float x = v[0], y = v[1], z = v[2], w = v[3];

	for (int r = 0; r < 4; r++)
		out[r] = m[r] * x + m[4 + r] * y + m[8 + r] * z + m[12 + r] * w;
}

void psvr2_math_mat3_mul_inplace(float a[9], const float b[9])
{
	float t[9];

	psvr2_math_mat3_mul(a, b, t);
	memcpy(a, t, sizeof(t));
}

void psvr2_math_quat_mul(const float a[4], const float b[4], float o[4])
{
	o[0] = ((a[0] * b[0] - a[1] * b[1]) - a[2] * b[2]) - a[3] * b[3];
	o[1] = (a[2] * b[3] - a[3] * b[2]) + a[0] * b[1] + a[1] * b[0];
	o[2] = (a[3] * b[1] - a[1] * b[3]) + a[0] * b[2] + a[2] * b[0];
	o[3] = (a[1] * b[2] - a[2] * b[1]) + a[0] * b[3] + a[3] * b[0];
}

void psvr2_math_quat_mul_inplace(float a[4], const float b[4])
{
	float t[4];

	psvr2_math_quat_mul(a, b, t);
	memcpy(a, t, sizeof(t));
}

void psvr2_math_quat_rotate(const float q[4], const float v[3], float out[3])
{
	float m[16], x = v[0], y = v[1], z = v[2];

	psvr2_math_quat_to_mat4(q, m);
	for (int r = 0; r < 3; r++)
		out[r] = x * m[r] + y * m[4 + r] + z * m[8 + r];
}

void psvr2_math_pose_mul(const float a[10], const float b[10], float out[10])
{
	float q[4], p[3];

	psvr2_math_quat_mul(a + 3, b + 3, q);
	psvr2_math_quat_rotate(a + 3, b, p);
	out[0] = p[0] + a[0];
	out[1] = p[1] + a[1];
	out[2] = p[2] + a[2];
	memcpy(out + 3, q, sizeof(q));
	out[7] = b[7] * a[7];
	out[8] = b[8] * a[8];
	out[9] = b[9] * a[9];
}

void psvr2_math_vec3_normalize(float v[3])
{
	float x = v[0], y = v[1], z = v[2];
	float n = sqrtf(x * x + y * y + z * z);

	if ((double)n < 1e-10)
		n = 1.0f;
	v[0] = x / n;
	v[1] = y / n;
	v[2] = z / n;
}

float psvr2_math_vec2_angle_deg(const float a2[2], const float b2[2])
{
	float a[3] = { a2[0], a2[1], 0.0f }, b[3] = { b2[0], b2[1], 0.0f };
	float ang;

	psvr2_math_vec3_normalize(a);
	psvr2_math_vec3_normalize(b);
	ang = acosf(b[1] * a[1] + b[0] * a[0] + b[2] * a[2]);
	ang = (ang * 180.0f) / 3.14159274f;
	if (b[1] * a[0] - a[1] * b[0] < 0.0f)
		ang = 360.0f - ang;
	return ang;
}

void psvr2_math_axis_angle_to_quat(float q[4], float angle, const float axis[3])
{
	float h = angle * 0.5f, s = sinf(h), c = cosf(h);

	q[0] = c;
	q[1] = s * axis[0];
	q[2] = s * axis[1];
	q[3] = s * axis[2];
}

void psvr2_math_axis_angle_to_mat4(float out[16], float angle, const float axis[3])
{
	float q[4];

	psvr2_math_axis_angle_to_quat(q, angle, axis);
	psvr2_math_quat_to_mat4(q, out);
}

void psvr2_math_trs_to_mat4(float out[16], const float s[3], const float q[4], const float pos[3])
{
	float p[10] = { pos[0], pos[1], pos[2], q[0], q[1], q[2], q[3], s[0], s[1], s[2] };

	psvr2_math_pose_to_mat4(p, out);
}

/* Normalise unless the length is exactly zero; squares summed y, x, z. */
static void norm_nz(float v[3])
{
	float n = sqrtf(v[1] * v[1] + v[0] * v[0] + v[2] * v[2]);

	if (n != 0.0f) {
		v[0] /= n;
		v[1] /= n;
		v[2] /= n;
	}
}

void psvr2_math_look_at(float m[16], const float eye[3], const float target[3], const float up[3])
{
	float f[3] = { eye[0] - target[0], eye[1] - target[1], eye[2] - target[2] };
	float s[3], u[3], t[16] = { 0 };

	norm_nz(f);
	s[0] = f[2] * up[1] - f[1] * up[2];
	s[1] = f[0] * up[2] - f[2] * up[0];
	s[2] = f[1] * up[0] - f[0] * up[1];
	u[0] = s[2] * f[1] - s[1] * f[2];
	u[1] = s[0] * f[2] - s[2] * f[0];
	u[2] = s[1] * f[0] - s[0] * f[1];
	norm_nz(s);
	norm_nz(u);
	for (int c = 0; c < 3; c++) {
		m[c * 4] = s[c];
		m[c * 4 + 1] = u[c];
		m[c * 4 + 2] = f[c];
		m[c * 4 + 3] = 0.0f;
	}
	m[12] = m[13] = m[14] = 0.0f;
	m[15] = 1.0f;
	t[0] = t[5] = t[10] = t[15] = 1.0f;
	t[12] = -eye[0];
	t[13] = -eye[1];
	t[14] = -eye[2];
	psvr2_math_mat4_mul_inplace(m, t);
}

void psvr2_math_perspective(float m[16], float fovy_deg, float aspect, float a, float b)
{
	double f = 1.0 / tan((double)(((fovy_deg * 6.28318548f) / 360.0f) * 0.5f));

	memset(m, 0, 16 * sizeof(float));
	m[0] = (float)f / aspect;
	m[5] = (float)f;
	m[10] = (a + b) / (a - b);
	m[11] = -1.0f;
	m[14] = ((b + b) * a) / (a - b);
}

/* Shared body of the matrix-to-quaternion helpers; s = column stride (3 or 4). */
static void rot_to_quat(const float *m, int s, float q[4])
{
	float m00 = m[0], m11 = m[s + 1], m22 = m[2 * s + 2];
	float m10 = m[1], m20 = m[2], m01 = m[s], m21 = m[s + 2], m02 = m[2 * s], m12 = m[2 * s + 1];
	float t[4], half, f;
	int k;

	q[0] = 1.0f;
	q[1] = q[2] = q[3] = 0.0f;
	t[0] = ((m00 - m11) - m22) + 1.0f;
	t[1] = ((m11 - m00) - m22) + 1.0f;
	t[2] = ((-m00 - m11) + m22) + 1.0f;
	t[3] = ((m11 + m00) + m22) + 1.0f;
	k = t[0] < t[1];
	if (!(t[2] <= t[k]))
		k = 2;
	if (!(t[3] <= t[k]))
		k = 3;
	if (t[k] < 0.0f)
		return;
	half = sqrtf(t[k]) * 0.5f;
	f = 0.25f / half;
	switch (k) {
	case 0:
		q[1] = half;
		q[2] = (m01 + m10) * f;
		q[3] = (m02 + m20) * f;
		q[0] = (m21 - m12) * f;
		break;
	case 1:
		q[2] = half;
		q[1] = (m01 + m10) * f;
		q[3] = (m12 + m21) * f;
		q[0] = (m02 - m20) * f;
		break;
	case 2:
		q[3] = half;
		q[1] = (m02 + m20) * f;
		q[2] = (m12 + m21) * f;
		q[0] = (m10 - m01) * f;
		break;
	default:
		q[0] = half;
		q[1] = (m21 - m12) * f;
		q[2] = (m02 - m20) * f;
		q[3] = (m10 - m01) * f;
		break;
	}
}

void psvr2_math_mat3_to_quat(const float m[9], float q[4])
{
	rot_to_quat(m, 3, q);
}

void psvr2_math_mat4_to_quat(const float m[16], float q[4])
{
	rot_to_quat(m, 4, q);
}

int psvr2_math_mat4_decompose(const float m[16], float scale[3], float q[4], float pos[3])
{
	float r[16] = { 0 };

	pos[0] = m[12];
	pos[1] = m[13];
	pos[2] = m[14];
	for (int c = 0; c < 3; c++) {
		const float *v = m + 4 * c;
		float sign = (v[0] * v[1] * v[2] * v[3] == 0.0f) ? 1.0f : -1.0f;

		scale[c] = (float)sqrt((double)(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])) * sign;
	}
	if (scale[0] == 0.0f || scale[1] == 0.0f || scale[2] == 0.0f) {
		q[0] = 1.0f;
		q[1] = q[2] = q[3] = 0.0f;
		return 0;
	}
	for (int c = 0; c < 3; c++)
		for (int k = 0; k < 3; k++)
			r[4 * c + k] = m[4 * c + k] / scale[c];
	r[15] = 1.0f;
	psvr2_math_mat4_to_quat(r, q);
	return 1;
}

int psvr2_math_mat4_to_pose(float p[10], const float m[16])
{
	return psvr2_math_mat4_decompose(m, p + 7, p + 3, p) ? 0 : -1;
}

/* Degrees to radians the way Sony does it: (deg * pi) / 180, in float. */
static float rad(float deg)
{
	return (deg * 3.14159274f) / 180.0f;
}

static void mat4_identity(float m[16])
{
	memset(m, 0, 16 * sizeof(float));
	m[0] = m[5] = m[10] = m[15] = 1.0f;
}

void psvr2_math_mat4_rot_x_deg(float m[16], float deg)
{
	float a = rad(deg), s = sinf(a), c = cosf(a);

	mat4_identity(m);
	m[5] = c;
	m[6] = s;
	m[9] = -s;
	m[10] = c;
}

void psvr2_math_mat4_rot_y_deg(float m[16], float deg)
{
	float a = rad(deg), s = sinf(a), c = cosf(a);

	mat4_identity(m);
	m[0] = c;
	m[2] = -s;
	m[8] = s;
	m[10] = c;
}

void psvr2_math_mat4_rot_z_deg(float m[16], float deg)
{
	float a = rad(deg), s = sinf(a), c = cosf(a);

	mat4_identity(m);
	m[0] = c;
	m[1] = s;
	m[4] = -s;
	m[5] = c;
}

void psvr2_math_mat4_translation(float m[16], float x, float y, float z)
{
	mat4_identity(m);
	m[12] = x;
	m[13] = y;
	m[14] = z;
}

/*
 * Term tables for psvr2_math_mat4_invert. A triple {a, b, c} is
 * (m[a] * m[b]) * m[c]; a quadruple adds "* m[d]". Rounding depends on
 * this grouping and on the summation order: first three terms added left
 * to right, then the last three subtracted one by one.
 */
static const uint8_t inv_cof[16][6][3] = {
	[0]  = { {5, 10, 15}, {9, 14, 7}, {13, 6, 11}, {5, 14, 11}, {9, 6, 15}, {13, 10, 7} },
	[1]  = { {9, 2, 15}, {14, 1, 11}, {10, 13, 3}, {10, 1, 15}, {14, 9, 3}, {13, 2, 11} },
	[2]  = { {6, 1, 15}, {14, 5, 3}, {2, 13, 7}, {14, 1, 7}, {5, 2, 15}, {6, 13, 3} },
	[3]  = { {2, 5, 11}, {10, 1, 7}, {6, 9, 3}, {6, 1, 11}, {10, 5, 3}, {2, 9, 7} },
	[4]  = { {8, 6, 15}, {14, 4, 11}, {12, 10, 7}, {10, 4, 15}, {14, 8, 7}, {12, 6, 11} },
	[5]  = { {14, 8, 3}, {0, 10, 15}, {12, 2, 11}, {0, 14, 11}, {8, 2, 15}, {10, 12, 3} },
	[6]  = { {4, 2, 15}, {14, 0, 7}, {6, 12, 3}, {0, 6, 15}, {14, 4, 3}, {2, 12, 7} },
	[7]  = { {6, 0, 11}, {10, 4, 3}, {2, 8, 7}, {10, 0, 7}, {2, 4, 11}, {6, 8, 3} },
	[8]  = { {8, 13, 7}, {4, 9, 15}, {12, 5, 11}, {4, 13, 11}, {5, 8, 15}, {12, 9, 7} },
	[9]  = { {1, 8, 15}, {0, 13, 11}, {12, 9, 3}, {0, 9, 15}, {8, 13, 3}, {12, 1, 11} },
	[10] = { {4, 13, 3}, {0, 5, 15}, {12, 1, 7}, {0, 13, 7}, {4, 1, 15}, {12, 5, 3} },
	[11] = { {4, 1, 11}, {0, 9, 7}, {5, 8, 3}, {0, 5, 11}, {4, 9, 3}, {1, 8, 7} },
	[12] = { {8, 5, 14}, {13, 4, 10}, {12, 9, 6}, {4, 9, 14}, {8, 13, 6}, {12, 5, 10} },
	[13] = { {13, 8, 2}, {9, 0, 14}, {12, 1, 10}, {0, 13, 10}, {1, 8, 14}, {12, 9, 2} },
	[14] = { {4, 1, 14}, {13, 0, 6}, {12, 5, 2}, {0, 5, 14}, {4, 13, 2}, {12, 1, 6} },
	[15] = { {9, 4, 2}, {5, 0, 10}, {8, 1, 6}, {0, 9, 6}, {4, 1, 10}, {5, 8, 2} },
};
static const uint8_t inv_det[24][4] = {
	{5, 0, 10, 15}, {9, 0, 14, 7}, {13, 0, 6, 11}, {4, 1, 14, 11},
	{9, 4, 2, 15}, {13, 4, 10, 3}, {8, 1, 6, 15}, {8, 5, 14, 3},
	{13, 8, 2, 7}, {12, 1, 10, 7}, {12, 5, 2, 11}, {12, 9, 6, 3},
	{0, 5, 14, 11}, {0, 9, 6, 15}, {0, 13, 10, 7}, {4, 1, 10, 15},
	{4, 9, 14, 3}, {4, 13, 2, 11}, {1, 8, 14, 7}, {5, 8, 2, 15},
	{8, 13, 6, 3}, {12, 1, 6, 11}, {12, 5, 10, 3}, {12, 9, 2, 7},
};

void psvr2_math_mat4_invert(float m[16])
{
	float a[16], det, inv;
	int i, k;

	memcpy(a, m, sizeof(a));
	det = 0.0f;
	for (k = 0; k < 24; k++) {
		const uint8_t *t = inv_det[k];
		float p = a[t[0]] * a[t[1]] * a[t[2]] * a[t[3]];

		if (k == 0)
			det = p;
		else if (k < 12)
			det += p;
		else
			det -= p;
	}
	inv = 1.0f / det;
	for (i = 0; i < 16; i++) {
		float s = 0.0f;

		for (k = 0; k < 6; k++) {
			const uint8_t *t = inv_cof[i][k];
			float p = a[t[0]] * a[t[1]] * a[t[2]];

			if (k == 0)
				s = p;
			else if (k < 3)
				s += p;
			else
				s -= p;
		}
		m[i] = s * inv;
	}
}

void psvr2_math_rodrigues_to_mat3d(const double r[3], double R[9])
{
	double th = sqrt(r[1] * r[1] + r[0] * r[0] + r[2] * r[2]);

	if (th < 2.220446049250313e-16) {
		for (int i = 0; i < 9; i++)
			R[i] = (i % 4 == 0) ? 1.0 : 0.0;
		return;
	}
	double c = cos(th), s = sin(th), inv = 1.0 / th, omc = 1.0 - c;
	double k[3] = { r[0] * inv, r[1] * inv, r[2] * inv };
	double kx[9] = { 0, -k[2], k[1], k[2], 0, -k[0], -k[1], k[0], 0 };

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R[i * 3 + j] = (c * (i == j ? 1.0 : 0.0) + omc * (k[i] * k[j])) + s * kx[i * 3 + j];
}
