/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Shared geometry helpers, behaviour-identical to the PSVR2 SteamVR
 * driver's math module (Sony VA 0x180160000-0x18016ffff, build 1741940190).
 * Spec: docs/sony-re/vrtrace.md section 8. Every function is a plain C11
 * leaf: copy this header and psvr2_math.c into another subsystem freely.
 * Build with -ffp-contract=off (no FMA); the results are then bit-exact
 * with Sony's (see scripts/sony-re/status/trace.tsv for each function).
 *
 * Conventions (Sony's):
 *  - matrices are column-major: element (row r, col c) of an N x N matrix
 *    is m[c * N + r]; vectors are columns, so v' = M v.
 *  - quaternions are w-first: q[0] = w, q[1..3] = x, y, z.
 *  - a "pose" is float[10]: position[3], quaternion[4] (w-first),
 *    scale[3].
 */
#ifndef PSVR2_MATH_H
#define PSVR2_MATH_H

/* out = a * b (3x3). out may not alias a or b. Sony 0x18016baf0. */
void psvr2_math_mat3_mul(const float a[9], const float b[9], float out[9]);
/* out = m * v (3x3 times vec3). Sony 0x18016bc90. */
void psvr2_math_mat3_mul_vec3(const float m[9], const float v[3], float out[3]);
/* out = a * b (4x4). out may not alias a or b. Sony 0x18016bd20. */
void psvr2_math_mat4_mul(const float a[16], const float b[16], float out[16]);
/* a = a * b (4x4, in place). Sony 0x18016c810. */
void psvr2_math_mat4_mul_inplace(float a[16], const float b[16]);
/* Unit-normalise q, then the 4x4 rotation matrix (no translation). Sony 0x18016df80. */
void psvr2_math_quat_to_mat4(const float q[4], float out[16]);
/* Pose to 4x4: translate * rotate * scale. Sony 0x18016dc50. */
void psvr2_math_pose_to_mat4(const float pose[10], float out[16]);
/* Inverse pose to 4x4: scale^-1 * rotate^T * translate(-p). Sony 0x18016dda0. */
void psvr2_math_pose_to_mat4_inv(const float pose[10], float out[16]);
/* Identity pose: position 0, quaternion (1,0,0,0), scale 1. Sony 0x18016ebe0. */
void psvr2_math_pose_identity(float pose[10]);
/*
 * Euler angles (radians) to w-first quaternion, computed in double from
 * the half angles: a about x, b about y, c about z [likely]. Sony 0x18016ec10.
 */
void psvr2_math_euler_to_quat(float q[4], float a, float b, float c);
/*
 * OpenGL-style projection from frustum bounds left, right, bottom, top
 * at the near plane, near, far. Sony 0x18016ed80 (and its duplicate 0x18016d370).
 */
void psvr2_math_frustum(float out[16], float l, float r, float b, float t, float n, float f);

/* out = m * v (4x4 times vec4). Sony 0x18016c130. */
void psvr2_math_mat4_mul_vec4(const float m[16], const float v[4], float out[4]);
/* a = a * b (3x3, in place). Sony 0x18016c630. */
void psvr2_math_mat3_mul_inplace(float a[9], const float b[9]);
/* out = a * b (Hamilton product, w-first). out may alias neither. Sony 0x18016c220. */
void psvr2_math_quat_mul(const float a[4], const float b[4], float out[4]);
/* a = a * b (in place). Sony 0x18016cc80. */
void psvr2_math_quat_mul_inplace(float a[4], const float b[4]);
/* out = R(q) v, R from psvr2_math_quat_to_mat4 (q need not be unit). Sony 0x18016c360. */
void psvr2_math_quat_rotate(const float q[4], const float v[3], float out[3]);
/*
 * Pose composition out = a * b: quat = a.q * b.q, pos = R(a.q) b.pos + a.pos
 * (b.pos is not scaled by a.scale), scale = b.scale * a.scale. Sony 0x18016c420.
 */
void psvr2_math_pose_mul(const float a[10], const float b[10], float out[10]);
/* Normalise a vec3 in place; a length below 1e-10 (as double) leaves it unchanged. Sony 0x180170840. */
void psvr2_math_vec3_normalize(float v[3]);
/*
 * Angle in degrees, [0, 360), from 2D vector a to b (counter-clockwise when
 * b.y*a.x - a.y*b.x >= 0). Sony 0x18016d170.
 */
float psvr2_math_vec2_angle_deg(const float a[2], const float b[2]);
/* Axis (not normalised) and angle (radians) to quaternion (float sinf/cosf). Sony 0x18016d270 (and 0x18016f2d0 with arguments q, axis, angle). */
void psvr2_math_axis_angle_to_quat(float q[4], float angle, const float axis[3]);
/* Axis-angle to 4x4 rotation (normalises the quaternion). Sony 0x18016d9a0. */
void psvr2_math_axis_angle_to_mat4(float out[16], float angle, const float axis[3]);
/* Translate(pos) * rotate(q) * scale(s) as 4x4. Sony 0x18016db00. */
void psvr2_math_trs_to_mat4(float out[16], const float s[3], const float q[4], const float pos[3]);
/*
 * View matrix from eye, target and up (gluLookAt with the forward axis
 * eye - target): rows s = up x f, u = f x s, f; then translate(-eye).
 * Zero-length vectors are left unnormalised. Sony 0x18016d430 (duplicate 0x18016ee40).
 */
void psvr2_math_look_at(float out[16], const float eye[3], const float target[3], const float up[3]);
/*
 * Perspective projection; fovy in degrees, tan evaluated in double.
 * m10 = (a + b)/(a - b), m14 = 2ab/(a - b) for the two clip planes a, b
 * as Sony passes them. Sony 0x18016d800 (duplicate 0x18016f210).
 */
void psvr2_math_perspective(float out[16], float fovy_deg, float aspect, float a, float b);

/*
 * Rotation matrix to w-first quaternion (Shepperd: pick the largest of
 * 1+m00-m11-m22, 1-m00+m11-m22, 1-m00-m11+m22, 1+m00+m11+m22, ties to the
 * earlier one). Returns identity if that candidate is negative.
 * mat3: Sony 0x18016e150; mat4 (upper 3x3): Sony 0x18016e3c0.
 */
void psvr2_math_mat3_to_quat(const float m[9], float q[4]);
void psvr2_math_mat4_to_quat(const float m[16], float q[4]);
/*
 * Split an affine 4x4 into scale, rotation, translation. scale[i] = |column
 * i| (sqrt in double), negated if all four entries of column i are non-zero.
 * If any scale is 0 the quaternion is identity and the call fails.
 * psvr2_math_mat4_decompose returns 1 on success, 0 on failure (Sony
 * 0x18016e630); psvr2_math_mat4_to_pose fills a pose and returns 0 or -1
 * (Sony 0x18016e910).
 */
int psvr2_math_mat4_decompose(const float m[16], float scale[3], float q[4], float pos[3]);
int psvr2_math_mat4_to_pose(float pose[10], const float m[16]);
/* Elementary 4x4 rotations, angle in degrees (sinf/cosf). Sony 0x18016f350 / 0x18016f3e0 / 0x18016f470. */
void psvr2_math_mat4_rot_x_deg(float m[16], float deg);
void psvr2_math_mat4_rot_y_deg(float m[16], float deg);
void psvr2_math_mat4_rot_z_deg(float m[16], float deg);
/* Translation 4x4. Sony 0x18016f540. */
void psvr2_math_mat4_translation(float m[16], float x, float y, float z);
/*
 * In-place 4x4 inverse by adjugate / determinant, no singularity check
 * (det 0 gives inf/NaN). Each cofactor is three positive and three
 * negative triple products in a fixed grouping; the determinant is 12
 * positive then 12 negative quadruple products (docs/sony-re/vrtrace.md
 * section 8). Sony 0x18016f590.
 */
void psvr2_math_mat4_invert(float m[16]);

/*
 * Rodrigues rotation vector to 3x3 rotation matrix, double precision,
 * ROW-major: R = (c I + (1 - c) k k^T) + s [k]x, theta = |r| (squares
 * summed y, x, z), k = r * (1/theta), c = cos theta, s = sin theta.
 * theta < 2^-52 gives the identity (NaN takes the rotation path). Sony 0x180168e40 (its output is a
 * small matrix object: 9 doubles then int64 rows = 3, cols = 3, row
 * step = 24 bytes, the last three written only on the non-identity path).
 */
void psvr2_math_rodrigues_to_mat3d(const double r[3], double R[9]);

#endif
