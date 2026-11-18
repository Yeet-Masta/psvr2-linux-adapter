/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Controller tracker: small vector, quaternion and matrix kernels.
 * Written from docs/sony-re/ctrltracker.md; checked bit-for-bit by
 * scripts/sony-re/difftest/t_ctrltracker_math.c. Build with -ffp-contract=off.
 *
 * Quaternions are x, y, z, w. Matrices are column-major with 4-float
 * (16-byte) columns.
 */
#ifndef PSVR2_SIE_CT_MATH_H
#define PSVR2_SIE_CT_MATH_H
#include <stdint.h>

void psvr2_ct_quat_rotate(float out[3], const float q[4], const float v[3]);
float psvr2_ct_quat_length(const float q[4]);
void psvr2_ct_quat_normalize(float out[4], const float q[4]);
/* Writes lanes 0..2 of three columns; lane 3 is left untouched. */
void psvr2_ct_quat_to_mat3(float out[3][4], const float q[4]);
void psvr2_ct_mat4_mul_vec4(float out[4], const float m[4][4], const float v[4]);
void psvr2_ct_mat43_mul_vec3(float out[4], const float m[3][4], const float v[3]);
void psvr2_ct_mat4_inverse(float out[4][4], const float m[4][4]);

void psvr2_ct_vec3_add_inplace(float a[3], const float b[3]);
float psvr2_ct_vec3_length(const float v[3]);
void psvr2_ct_vec3_cross(float out[3], const float a[3], const float b[3]);
/* out = p (x) q, Hamilton product. */
void psvr2_ct_quat_mul(float out[4], const float p[4], const float q[4]);
/* Rotation about +Y by angle (rad): (0, sin(a/2), 0, cos(a/2)). */
void psvr2_ct_quat_yaw(float out[4], float angle);
/* out = a * b for 3x3 matrices (lanes 0..2 of each column written). */
void psvr2_ct_mat3_mul(float out[3][4], const float a[3][4], const float b[3][4]);
/* Rotation matrix about +Y (lanes 0..2 written). */
void psvr2_ct_mat3_rot_y(float out[3][4], float angle);
/* Azimuths of a and b in the XZ plane (atan2(z, x)) and their difference
 * wrapped once into [-pi, pi]. */
float psvr2_ct_azimuth_diff(float *az_a, float *az_b, const float a[3], const float b[3]);
/* Multiply-with-carry generator (two 16-bit lags); returns uniform in
 * (lo, hi). state is two u32. */
float psvr2_ct_mwc_uniform(uint32_t state[2], float lo, float hi);
void psvr2_ct_vec3_sub_inplace(float a[3], const float b[3]);
void psvr2_ct_vec3_scale_inplace(float a[3], float s);
/* Shortest-arc rotation taking unit vector a onto unit vector b. */
void psvr2_ct_quat_from_to(float out[4], const float a[3], const float b[3]);
void psvr2_ct_quatd_normalize(double out[4], const double q[4]);
void psvr2_ct_quatd_mul(double out[4], const double p[4], const double q[4]);
/* See docs ("sphere-cap quaternion"). */
void psvr2_ct_quat_cap(float out[4], const float v[3], float c, float r);
void psvr2_ct_quat_cap_d(double out[4], const float v[3], double c, double r);
/* Rotation matrix (three 4-float columns) to quaternion. */
void psvr2_ct_mat3_to_quat(float out[4], const float m[3][4]);
/* Yaw (about +Y) of p relative to q, after removing the tilt; see docs. */
float psvr2_ct_quat_rel_yaw(const float p[4], const float q[4]);
/* Rotation vector (axis * angle) to unit quaternion. */
void psvr2_ct_quat_exp(float out[4], const float v[3]);
/* out = normalize(q (x) exp(w * dt)), q and out in double. */
void psvr2_ct_quatd_integrate(double out[4], const double q[4], const float w[3], float dt);
/* Store ref (4 floats) at dst, and dst_diff = cur - ref (lanes 0..2). */
void psvr2_ct_set_ref_and_diff(float ref_out[4], float diff_out[3], const float cur[3],
			       const float ref[4]);
#endif
