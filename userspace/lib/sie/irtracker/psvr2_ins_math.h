/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Small vector / quaternion / matrix helpers used by the headset INS.
 *
 * Written from docs/sony-re/irtracker.md (behaviour only) and checked
 * against the reference driver by scripts/sony-re/difftest. Quaternions are
 * stored w first: q[0] = w, q[1..3] = x, y, z. Matrices are 4x4 floats,
 * column-major (element (r,c) at m[c*4 + r]). Build with -ffp-contract=off.
 */
#ifndef PSVR2_SIE_INS_MATH_H
#define PSVR2_SIE_INS_MATH_H
#include <stdbool.h>

/* q[1..3] *= -1 (conjugate in place). Returns q. */
float *psvr2_irtracker_quat_conj_inplace(float *q);
/* p[0..2] *= -1 and q at p[3..6]: x,y,z *= -1. Returns p. */
float *psvr2_irtracker_pose_negate(float *p);
/* out = (1-t)*a + t*b, 3 components. Returns out. */
float *psvr2_irtracker_vec3_lerp(float *out, const float *a, const float *b, float t);
float psvr2_irtracker_vec2_len(const float *v);
float psvr2_irtracker_vec3_len(const float *v);
void psvr2_irtracker_quat_normalize(float *q);
/* Divide by length; length below 1e-4 counts as 1. */
void psvr2_irtracker_vec2_normalize(float *v);
/* Divide by length; length below 1e-10 counts as 1. */
void psvr2_irtracker_vec3_normalize(float *v);
/*
 * axis = obj[1..3]; rotate v by `angle`
 * radians about axis (q v q*, axis not normalised). Returns out.
 */
float *psvr2_irtracker_axis_angle_rotate(const float *obj, float *out, float angle, const float *v);
float *psvr2_irtracker_mat4_identity(float *m);
/* Spherical interpolation a -> b. b is negated in place when a.b < 0. */
float *psvr2_irtracker_quat_slerp(const float *a, float *out, float t, float *b);
float *psvr2_irtracker_mat4_transpose(float *m);
/*
 * Merge two weighted isotropic Gaussians (mean, sigma, weight). On success
 * writes the merged mean to *mean_out[0], the sigma to *sigma_out and returns
 * true; returns false (nothing written) when the merged variance is not > 0.
 */
bool psvr2_irtracker_gauss_merge(float **mean_out, float *sigma_out, const float *ma, float sa, float wa,
				 const float *mb, float sb, float wb);
#endif
