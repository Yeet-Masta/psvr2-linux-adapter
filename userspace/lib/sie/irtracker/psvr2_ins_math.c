// SPDX-License-Identifier: GPL-2.0
/* See psvr2_ins_math.h. Build with -ffp-contract=off: results must match bit for bit. */
#include "psvr2_ins_math.h"

#include <math.h>
#include <string.h>

float *psvr2_irtracker_quat_conj_inplace(float *q)
{
	for (int i = 1; i < 4; i++)
		q[i] = q[i] * -1.0f;
	return q;
}

float *psvr2_irtracker_pose_negate(float *p)
{
	static const int idx[] = { 0, 1, 2, 4, 5, 6 };
	for (int i = 0; i < 6; i++)
		p[idx[i]] = p[idx[i]] * -1.0f;
	return p;
}

float *psvr2_irtracker_vec3_lerp(float *out, const float *a, const float *b, float t)
{
	const float u = 1.0f - t;
	for (int i = 0; i < 3; i++)
		out[i] = u * a[i] + t * b[i];
	return out;
}

float psvr2_irtracker_vec2_len(const float *v)
{
	return sqrtf(v[0] * v[0] + v[1] * v[1]);
}

float psvr2_irtracker_vec3_len(const float *v)
{
	return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

void psvr2_irtracker_quat_normalize(float *q)
{
	/* The scalar part is summed last. */
	const float n = sqrtf(q[1] * q[1] + q[2] * q[2] + q[3] * q[3] + q[0] * q[0]);
	for (int i = 0; i < 4; i++)
		q[i] = q[i] / n;
}

void psvr2_irtracker_vec2_normalize(float *v)
{
	float n = psvr2_irtracker_vec2_len(v);
	if (n < 1e-4f)
		n = 1.0f;
	v[0] = v[0] / n;
	v[1] = v[1] / n;
}

void psvr2_irtracker_vec3_normalize(float *v)
{
	float n = psvr2_irtracker_vec3_len(v);
	if ((double)n < 1e-10)
		n = 1.0f;
	for (int i = 0; i < 3; i++)
		v[i] = v[i] / n;
}

/*
 * Hamilton product r = a * b (w first). The summation order is fixed:
 * w = aw bw - ax bx - ay by - az bz, and for each vector component the
 * cross-product term comes first, then b's part scaled by aw, then a's by bw.
 */
static void qmul(float *r, const float *a, const float *b)
{
	const float w = ((a[0] * b[0] - a[1] * b[1]) - a[2] * b[2]) - a[3] * b[3];
	const float x = (a[2] * b[3] - a[3] * b[2]) + b[1] * a[0] + b[0] * a[1];
	const float y = (a[3] * b[1] - a[1] * b[3]) + b[2] * a[0] + b[0] * a[2];
	const float z = (a[1] * b[2] - a[2] * b[1]) + b[3] * a[0] + b[0] * a[3];
	r[0] = w; r[1] = x; r[2] = y; r[3] = z;
}

float *psvr2_irtracker_axis_angle_rotate(const float *obj, float *out, float angle, const float *v)
{
	const float h = angle * 0.5f;
	const float c = cosf(h), s = sinf(h);
	const float q[4] = { c, s * obj[1], s * obj[2], s * obj[3] };
	const float qc[4] = { c, -obj[1] * s, -obj[2] * s, -obj[3] * s };
	const float pv[4] = { 0.0f, v[0], v[1], v[2] };
	float t[4], r[4];
	qmul(t, pv, qc);
	qmul(r, q, t);
	memcpy(out, r + 1, 3 * sizeof(float));
	return out;
}

float *psvr2_irtracker_mat4_identity(float *m)
{
	for (int i = 0; i < 16; i++)
		m[i] = (i % 5 == 0) ? 1.0f : 0.0f;
	return m;
}

float *psvr2_irtracker_quat_slerp(const float *a, float *out, float t, float *b)
{
	float d = a[1] * b[1] + a[2] * b[2] + a[3] * b[3] + a[0] * b[0];
	if (d < 0.0f) {
		for (int i = 0; i < 4; i++)
			b[i] = -b[i];
		d = -d;
	}
	float ka = 1.0f - t, kb = t;
	/* A NaN dot product takes the sine path. */
	if (!(d > 0.9999f)) {
		const float s = sqrtf(1.0f - d * d);
		const float th = atan2f(s, d);
		ka = sinf(ka * th) * (1.0f / s);
		kb = sinf(th * t) * (1.0f / s);
	}
	for (int i = 0; i < 4; i++)
		out[i] = a[i] * ka + b[i] * kb;
	return out;
}

float *psvr2_irtracker_mat4_transpose(float *m)
{
	for (int r = 0; r < 4; r++)
		for (int c = r + 1; c < 4; c++) {
			const float x = m[r * 4 + c];
			m[r * 4 + c] = m[c * 4 + r];
			m[c * 4 + r] = x;
		}
	return m;
}

bool psvr2_irtracker_gauss_merge(float **mean_out, float *sigma_out, const float *ma, float sa, float wa,
				 const float *mb, float sb, float wb)
{
	const float w = wa + wb;
	float m[3];
	for (int i = 0; i < 3; i++)
		m[i] = (ma[i] * wa + mb[i] * wb) / w;
	/* Squared norms are summed z, y, x. */
	const float ea = ma[2] * ma[2] + ma[1] * ma[1] + ma[0] * ma[0];
	const float eb = mb[2] * mb[2] + mb[1] * mb[1] + mb[0] * mb[0];
	const float second = (eb * wb + ea * wa) / w + (sa * sa * wa + sb * sb * wb) / w;
	const float var = second - (m[1] * m[1] + m[2] * m[2] + m[0] * m[0]);
	if (!(var > 0.0f))
		return false;
	memcpy(mean_out[0], m, sizeof m);
	*sigma_out = sqrtf(var);
	return true;
}
