/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Complex-number, 2x2 and small block helpers of the controller boot stage.
 * Spec: docs/sony-re/controller-tracking.md part D ("Complex and 2x2 helpers"). The 2x2
 * inverse and 2-D normalise use the SSE reciprocal estimates like the
 * reference, so they match it only on the same CPU.
 */
#ifndef PSVR2_CB_CPLX_H
#define PSVR2_CB_CPLX_H

void psvr2_cb_c_div(float out[2], const float a[2], const float b[2]);
void psvr2_cb_c_cbrt(float out[2], const float z[2]);
void psvr2_cb_c_sqrt(float out[2], const float z[2]);
void psvr2_cb_c_polar(float out[2], float r, float angle);
void psvr2_cb_m2_inverse(float out[4], const float m[4]);
void psvr2_cb_m2_mul_v2(float out[2], const float m[4], const float v[2]);
void psvr2_cb_v2_normalize_fast(float out[2], const float v[2]);
void psvr2_cb_v2_outer(float out[4], const float a[2], const float b[2]);
void psvr2_cb_v4_outer2(float out[12], const float v[4], const float b[2]);
void psvr2_cb_add12(float a[12], const float b[12]);
void psvr2_cb_scale12(float a[12], float s);
/* Camera-from-R,t matrix with the z axis flipped and metres to millimetres. */
void psvr2_cb_flip_pose(float out[16], const float r[9], const float t[3]);

#endif
