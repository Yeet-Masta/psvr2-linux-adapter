// SPDX-License-Identifier: GPL-2.0
/* See psvr2_ct_cov.h. */
#include "psvr2_ct_cov.h"
#include "psvr2_ct_math.h"

#include <string.h>

void psvr2_ct_cov_reset_block(float *p, int b)
{
	for (int i = b; i < b + 3; i++)
		if (i < PSVR2_CT_COV_N)
			memset(p + psvr2_ct_cov_idx(i, i), 0, sizeof(float) * (PSVR2_CT_COV_N - i));
	for (int r = 0; r < b; r++)
		for (int k = 0; k < 3; k++)
			p[psvr2_ct_cov_idx(r, b) + k] = 0.0f;
	for (int i = b; i < b + 3; i++)
		p[psvr2_ct_cov_idx(i, i)] = 1.0f;
}

void psvr2_ct_cov_set_block(float *p, int b, const float m[3][4])
{
	psvr2_ct_cov_reset_block(p, b);
	for (int c = 0; c < 3; c++)
		for (int r = 0; r <= c; r++)
			p[psvr2_ct_cov_idx(b + r, b + c)] = m[r][c];
}

void psvr2_ct_cov_get_block(float m[3][4], const float *p, int b)
{
	for (int c = 0; c < 3; c++)
		for (int r = 0; r <= c; r++)
			m[c][r] = m[r][c] = p[psvr2_ct_cov_idx(b + r, b + c)];
}

void psvr2_ct_lever_transfer(float out_pos[3], float out_vel[3], float out_rot[3][4],
			     const float pos[3], const float vel[3], const float r[3][4],
			     const float w[3], const float b[3][4], const float lever[3])
{
	float rl[4], p[3], v[3];

	psvr2_ct_mat43_mul_vec3(rl, r, lever);
	for (int i = 0; i < 3; i++)
		p[i] = rl[i] + pos[i];
	v[0] = (rl[2] * w[1] - rl[1] * w[2]) + vel[0];
	v[1] = (rl[0] * w[2] - rl[2] * w[0]) + vel[1];
	v[2] = (rl[1] * w[0] - rl[0] * w[1]) + vel[2];
	memcpy(out_pos, p, sizeof p);
	memcpy(out_vel, v, sizeof v);
	psvr2_ct_mat3_mul(out_rot, r, b);
}

static int packed_idx(int n, int i, int j)
{
	return ((2 * n - 1 - i) * i) / 2 + j;
}

void psvr2_ct_widen15(double *x, double *pd, int n, const float a[3], const float b[3],
		      const float c[3], const float d[3], const float *p)
{
	for (int k = 0; k < 3; k++) {
		x[k] = a[k];
		x[3 + k] = b[k];
		x[6 + k] = 0.0;
		x[9 + k] = c[k];
		x[12 + k] = d[k];
	}
	for (int j = 0; j < PSVR2_CT_COV_N; j++)
		for (int i = 0; i <= j; i++)
			pd[packed_idx(n, i, j)] = p[psvr2_ct_cov_idx(i, j)];
}

void psvr2_ct_widen6(double *x, double *pd, int n, const float b[3], const float *p)
{
	for (int k = 0; k < 3; k++) {
		x[k] = b[k];
		x[3 + k] = 0.0;
	}
	for (int j = 3; j < 9; j++)
		for (int i = 3; i <= j; i++)
			pd[packed_idx(n, i - 3, j - 3)] = p[psvr2_ct_cov_idx(i, j)];
}
