// SPDX-License-Identifier: GPL-2.0
/* See psvr2_ins_cfg.h. Build with -ffp-contract=off. */
#include "psvr2_ins_cfg.h"

#include <math.h>
#include <string.h>

static const psvr2_irtracker_ins_cfg defaults = {
	.k00 = 0.05f, .k04 = 0.025f, .k08 = 0.005f, .k0c = 0.0025f,
	.init_flag = 0.1f,
	.k14 = 0.3f, .k18 = 50.0f,
	.lim1_lo = -10.0f, .lim1_hi = 10.0f, .lim2_lo = -5.0f, .lim2_hi = 5.0f, .lim3_lo = -10.0f, .lim3_hi = 10.0f,
	.cov_small = { 1e-7f, 1e-4f, 1e-5f },
	.cov_far = { 1e-4f, 5e-4f, 1e-4f },
	.k4c = 0.1f, .k50 = 1.0f, .k54 = 0.01f, .k58 = 0.05f,
	.near_dt = 1.0f, .far_dt = 10.0f,
	.cov_floor = { 1e-8f, 1e-6f, 1e-6f },
	.gravity = 9.8f,
	.k74 = 0.05235988f, .k78 = 0.05235988f, .k7c = 0.05235988f,
};

const psvr2_irtracker_ins_cfg *psvr2_irtracker_ins_cfg_default(void)
{
	return &defaults;
}

bool psvr2_irtracker_mat3_near_transpose(const float *a, const float *b, float tol)
{
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++)
			if (!(fabsf(a[r * 3 + c] - b[c * 3 + r]) < tol))
				return false;
	return true;
}

bool psvr2_irtracker_cov3_valid(const float *m)
{
	if (!(m[0] >= 0.0f && m[4] >= 0.0f && m[8] >= 0.0f))
		return false;
	return psvr2_irtracker_mat3_near_transpose(m, m, 1e-6f);
}

static float *iso(float *out, float v)
{
	const float z = v * 0.0f;
	for (int i = 0; i < 9; i++)
		out[i] = (i % 4 == 0) ? v : z;
	return out;
}

float *psvr2_irtracker_cov3_floor(float *out, const float *m, float floor)
{
	for (int i = 0; i < 9; i++)
		if (!(floor < m[i]))
			return iso(out, floor);
	memcpy(out, m, 9 * sizeof(float));
	return out;
}

float *psvr2_irtracker_cov3_for_gap(const psvr2_irtracker_ins_cfg *c, float *out, const float *m, float a, float b,
				    int which)
{
	const float d = fabsf(a - b);
	if (d < c->near_dt)
		return psvr2_irtracker_cov3_floor(out, m, c->cov_floor[which]);
	/* A NaN gap counts as far. */
	return iso(out, !(d < c->far_dt) ? c->cov_far[which] : c->cov_small[which]);
}

bool psvr2_irtracker_cov3_small(const psvr2_irtracker_ins_cfg *c, const float *m, int which)
{
	return (m[8] + m[4] + m[0]) / 3.0f < c->cov_small[which] && psvr2_irtracker_cov3_valid(m);
}

bool psvr2_irtracker_vec_pair_small(const psvr2_irtracker_ins_cfg *c, const float *a, const float *b)
{
	return a[2] * a[2] + a[1] * a[1] + a[0] * a[0] < c->k00 * c->k00 * 3.0f &&
	       b[2] * b[2] + b[1] * b[1] + b[0] * b[0] < c->k08 * c->k08 * 3.0f;
}
