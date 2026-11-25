/* SPDX-License-Identifier: GPL-2.0 */
/*
 * INS tuning parameters and covariance helpers. Field meanings are our best
 * reading of how each value is used [likely]; see docs/sony-re/headset.md part B.
 */
#ifndef PSVR2_SIE_INS_CFG_H
#define PSVR2_SIE_INS_CFG_H
#include <stdbool.h>

typedef struct {
	float k00, k04, k08, k0c;
	float init_flag;             /* 0.1 once initialised (negative before) */
	float k14, k18;
	float lim1_lo, lim1_hi, lim2_lo, lim2_hi, lim3_lo, lim3_hi;
	float cov_small[3];          /* trace/3 limits used by cov3_small(0..2) */
	float cov_far[3];            /* isotropic covariance when far apart */
	float k4c, k50, k54, k58;
	float near_dt;               /* |a-b| below this: keep the given covariance */
	float far_dt;                /* |a-b| at or above this: cov_far, else cov_small */
	float cov_floor[3];
	float gravity;               /* m/s^2 */
	float k74, k78, k7c;
} psvr2_irtracker_ins_cfg;

const psvr2_irtracker_ins_cfg *psvr2_irtracker_ins_cfg_default(void);

/* true when every |a[i] - b[transposed i]| < tol (a is close to b^T). */
bool psvr2_irtracker_mat3_near_transpose(const float *a, const float *b, float tol);
/* Non-negative diagonal and symmetric within 1e-6. */
bool psvr2_irtracker_cov3_valid(const float *m);
/* Copy m when every element exceeds floor, else floor * identity. */
float *psvr2_irtracker_cov3_floor(float *out, const float *m, float floor);
/* Pick the covariance for time gap |a-b|; which = 0, 1, 2 (see doc). */
float *psvr2_irtracker_cov3_for_gap(const psvr2_irtracker_ins_cfg *c, float *out, const float *m, float a, float b,
				    int which);
/* trace(m)/3 below cov_small[which] and m valid. */
bool psvr2_irtracker_cov3_small(const psvr2_irtracker_ins_cfg *c, const float *m, int which);
/* |a|^2 < c->k00^2 * 3 and |b|^2 < c->k08^2 * 3. */
bool psvr2_irtracker_vec_pair_small(const psvr2_irtracker_ins_cfg *c, const float *a, const float *b);
#endif
