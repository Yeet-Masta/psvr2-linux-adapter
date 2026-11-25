/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Controller tracker: packed 15x15 symmetric covariance helpers and the
 * lever-arm pose transfer. See docs/sony-re/controller-tracking.md part H ("Packed
 * covariance"). Build with -ffp-contract=off.
 */
#ifndef PSVR2_SIE_CT_COV_H
#define PSVR2_SIE_CT_COV_H

#define PSVR2_CT_COV_N 15
#define PSVR2_CT_COV_PACKED (PSVR2_CT_COV_N * (PSVR2_CT_COV_N + 1) / 2)

/* Index of element (i, j), i <= j, in the packed upper triangle. */
static inline int psvr2_ct_cov_idx(int i, int j)
{
	return ((2 * PSVR2_CT_COV_N - 1 - i) * i) / 2 + j;
}

/* Clear rows/columns b..b+2 and put 1 on their diagonal. */
void psvr2_ct_cov_reset_block(float *p, int b);
/* As reset, then store the symmetric 3x3 block m (three 4-float columns). */
void psvr2_ct_cov_set_block(float *p, int b, const float m[3][4]);
/* Read the 3x3 diagonal block at b into m (lane 3 of each column untouched). */
void psvr2_ct_cov_get_block(float m[3][4], const float *p, int b);

/*
 * Rigid lever arm: a point at body offset `lever` of a body with position
 * pos, velocity vel, orientation R (three 4-float columns) and angular
 * velocity w. out_pos = R*lever + pos, out_vel = w x (R*lever) + vel,
 * out_rot = R * B.
 */
void psvr2_ct_lever_transfer(float out_pos[3], float out_vel[3], float out_rot[3][4],
			     const float pos[3], const float vel[3], const float r[3][4],
			     const float w[3], const float b[3][4], const float lever[3]);

/*
 * Widen the filter state to double for the UKF: x[0..14] = (a, b, 0, c, d)
 * (four float 3-vectors and a zero block), and copy the packed float
 * covariance into the packed double matrix pd of dimension n (n >= 15).
 */
void psvr2_ct_widen15(double *x, double *pd, int n, const float a[3], const float b[3],
		      const float c[3], const float d[3], const float *p);
/* Same for the 6-dimensional sub-state 3..8: x = (b, 0), pd (dimension n)
 * gets the covariance block rows/columns 3..8. */
void psvr2_ct_widen6(double *x, double *pd, int n, const float b[3], const float *p);
#endif
