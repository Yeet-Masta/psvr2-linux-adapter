/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Small vector / statistics helpers of the controller boot stage.
 * Spec: docs/sony-re/controller-tracking.md part D ("Math helpers"). Matrices are 4 columns
 * of 4 floats (column-major, translation in column 3). Several helpers use
 * fused multiply-add exactly where the reference does.
 */
#ifndef PSVR2_CB_MATH_H
#define PSVR2_CB_MATH_H
#include <stdint.h>

typedef struct { float v[4]; } psvr2_v4;
typedef struct { psvr2_v4 c[4]; } psvr2_m44;

/* Copy m, or its rigid inverse [R^T | -R^T t] when invert != 0. */
void psvr2_cb_rigid_get(psvr2_m44 *out, const psvr2_m44 *m, int invert);
/* out = c0*v.x + c1*v.y + c2*v.z (all 4 lanes, fused after the first product). */
void psvr2_cb_rotate(psvr2_v4 *out, const psvr2_m44 *m, const psvr2_v4 *v);
/* Lane 2 of c3 + c0*v.x + c1*v.y + c2*v.z (camera-space depth). */
float psvr2_cb_depth(const psvr2_m44 *m, const psvr2_v4 *v);
/* 2-D helpers. */
void psvr2_cb_v2_mul_sub(float out[2], const float a[2], const float b[2], const float c[2]);
float psvr2_cb_v2_len(const float a[2]);
void psvr2_cb_v2_normalize(float out[2], const float a[2]);
int psvr2_cb_v2_ccw(const float a[2], const float b[2]);
/* Running mean / std accumulator: sum, sumsq, std, -, flag, count. */
typedef struct { float sum, sumsq, std, pad; float valid; int32_t count; } psvr2_cb_stat;
void psvr2_cb_stat_finalize(psvr2_cb_stat *s);
/* RANSAC iteration bound. */
typedef struct {
	int32_t pad0;
	int32_t max_iters;
	int32_t pad1;
	uint8_t adaptive, pad2[7];
	float confidence;
	float pad3;
	float inlier_ratio;
} psvr2_cb_ransac;
void psvr2_cb_ransac_update(psvr2_cb_ransac *r);
/* Larger-eigenvalue eigenvector slope of [[a b][c d]]; 0 when degenerate. */
int psvr2_cb_eig2_slope(float a, float b, float c, float d, float *out);
/* 1 - (clamp(sqrt(x), lo, hi) - lo) / (hi - lo). */
float psvr2_cb_falloff(float x, float lo, float hi);
/* Speed class of a velocity: 2 below 100, 1 below 150, else 0. */
int psvr2_cb_speed_class(const float vel[3]);
/* Facing test of an LED normal (see spec). */
int psvr2_cb_facing(int enabled, float dist, const psvr2_v4 *axis, const psvr2_v4 *scale, const psvr2_m44 *m);
/* out = (m.c1 . w, ...) scaled; see spec "projected offset". */
void psvr2_cb_proj2(float out[2], float s, const psvr2_m44 *m, const float w[3]);

#endif
