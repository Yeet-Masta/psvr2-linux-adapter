// SPDX-License-Identifier: GPL-2.0
#include "psvr2_cb_math.h"
#include <math.h>
#include <string.h>

void psvr2_cb_rigid_get(psvr2_m44 *out, const psvr2_m44 *m, int invert)
{
	psvr2_m44 r;

	if (!invert) {
		*out = *m;
		return;
	}
	/* Rows of the 3x3 become columns; lane 3 of each new column is c2.w. */
	for (int i = 0; i < 3; i++) {
		r.c[i].v[0] = m->c[0].v[i];
		r.c[i].v[1] = m->c[1].v[i];
		r.c[i].v[2] = m->c[2].v[i];
		r.c[i].v[3] = m->c[2].v[3];
	}
	for (int k = 0; k < 4; k++) {
		float t = r.c[0].v[k] * -m->c[3].v[0];

		t = fmaf(r.c[1].v[k], -m->c[3].v[1], t);
		r.c[3].v[k] = fmaf(r.c[2].v[k], -m->c[3].v[2], t);
	}
	*out = r;
}

void psvr2_cb_rotate(psvr2_v4 *out, const psvr2_m44 *m, const psvr2_v4 *v)
{
	psvr2_v4 r;

	for (int k = 0; k < 4; k++) {
		float t = v->v[0] * m->c[0].v[k];

		t = fmaf(v->v[1], m->c[1].v[k], t);
		r.v[k] = fmaf(v->v[2], m->c[2].v[k], t);
	}
	*out = r;
}

float psvr2_cb_depth(const psvr2_m44 *m, const psvr2_v4 *v)
{
	float t = fmaf(v->v[0], m->c[0].v[2], m->c[3].v[2]);

	t = fmaf(v->v[1], m->c[1].v[2], t);
	return fmaf(v->v[2], m->c[2].v[2], t);
}

void psvr2_cb_v2_mul_sub(float out[2], const float a[2], const float b[2], const float c[2])
{
	out[0] = fmaf(b[0], a[0], -c[0]);
	out[1] = fmaf(b[1], a[1], -c[1]);
}

float psvr2_cb_v2_len(const float a[2])
{
	return sqrtf(a[1] * a[1] + a[0] * a[0]);
}

void psvr2_cb_v2_normalize(float out[2], const float a[2])
{
	const float inv = 1.0f / sqrtf(a[1] * a[1] + a[0] * a[0]);

	out[0] = inv * a[0];
	out[1] = inv * a[1];
}

int psvr2_cb_v2_ccw(const float a[2], const float b[2])
{
	return a[1] * b[0] < a[0] * b[1];
}

void psvr2_cb_stat_finalize(psvr2_cb_stat *s)
{
	float n, var;

	if (s->count == 0) {
		s->valid = 0.0f;
		return;
	}
	n = (float)s->count;
	s->sum = s->sum / n;
	var = s->sumsq / n - s->sum * s->sum;
	s->sumsq = var;
	if (var >= 0.0f)
		s->std = sqrtf(var);
}

void psvr2_cb_ransac_update(psvr2_cb_ransac *r)
{
	double num, den;
	float w;
	int n;

	if (!r->adaptive)
		return;
	num = log(1.0 - (double)r->confidence);
	w = r->inlier_ratio < 0.9f ? r->inlier_ratio : 0.9f;
	den = log(1.0 - pow((double)w, 6.0));
	n = (int)(int64_t)(num / den);
	if (n < r->max_iters)
		r->max_iters = n;
}

int psvr2_cb_eig2_slope(float a, float b, float c, float d, float *out)
{
	const float t = -d - a;
	const float disc = t * t - (a * d - b * c) * 4.0f;
	float l;

	if (!(disc > 1e-6f))
		return 0;
	l = (sqrtf(disc) - t) * 0.5f - d;
	if (l == 0.0f)
		return 0;
	*out = c / l;
	return 1;
}

float psvr2_cb_falloff(float x, float lo, float hi)
{
	float s = sqrtf(x);

	s = lo > s ? lo : s;
	s = hi < s ? hi : s;
	return 1.0f - (s - lo) / (hi - lo);
}

int psvr2_cb_speed_class(const float vel[3])
{
	const float n = sqrtf((vel[1] * vel[1] + vel[0] * vel[0]) + vel[2] * vel[2]);

	if (100.0f > n)
		return 2;
	return 150.0f > n;
}

int psvr2_cb_facing(int enabled, float dist, const psvr2_v4 *axis, const psvr2_v4 *scale, const psvr2_m44 *m)
{
	psvr2_v4 r;
	float d;

	if (!enabled || !(10.0f > dist))
		return 1;
	for (int k = 0; k < 4; k++) {
		float t = axis->v[0] * m->c[0].v[k];

		t = fmaf(axis->v[1], m->c[1].v[k], t);
		r.v[k] = fmaf(axis->v[2], m->c[2].v[k], t) * scale->v[k];
	}
	d = (r.v[1] + r.v[0]) + r.v[2];
	/* Facing unless cos(20 deg) >= d. */
	return !(cos(0.3490658503988659) >= (double)d);
}

void psvr2_cb_proj2(float out[2], float s, const psvr2_m44 *m, const float w[3])
{
	for (int k = 0; k < 2; k++) {
		float t = fmaf(m->c[1].v[k], w[0], 0.0f);

		t = fmaf(m->c[2].v[k], w[1], t);
		t = fmaf(m->c[3].v[k], w[2], t);
		out[k] = t * s;
	}
}
