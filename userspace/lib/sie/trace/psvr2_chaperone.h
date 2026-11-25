/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VrTracker2 universe / chaperone helpers (Sony VA 0x18011e190-0x18011f420,
 * plus the quaternion-to-matrix helper 0x180121360 they call), behaviour-
 * identical to build 1741940190. Spec: docs/sony-re/vrtrace.md section 10.
 * Build with -ffp-contract=off.
 *
 * Conventions here differ from psvr2_math.h (they follow the VrTracker2
 * core, not the math module):
 *  - quaternions are x-first: q[0..2] = x, y, z, q[3] = w.
 *  - a "pose7" is float[7]: position[3], then an x-first quaternion.
 *  - 3x3 matrices: R[0..8] in the order Sony stores them; R[0], R[1],
 *    R[2] are the image of the x axis (first column of the rotation).
 */
#ifndef PSVR2_CHAPERONE_H
#define PSVR2_CHAPERONE_H

#include <stdint.h>

/* out = a * b: position a.p + rotate(a.q, b.p), orientation a.q * b.q. Sony 0x18011e190. */
void psvr2_chap_pose_mul(const float a[7], const float b[7], float out[7]);
/* out = a^-1: conjugate orientation (no normalisation), -rotate(conj, a.p). out may alias a. Sony 0x1801613a0. */
void psvr2_chap_pose_inv(const float a[7], float out[7]);
/* Rotation matrix of a unit x-first quaternion (no normalisation). Sony 0x180121360. */
void psvr2_chap_quat_to_mat3(const float q[4], float R[9]);
/* Signed yaw (radians) of the rotation of q about the vertical axis. Sony 0x18011f420. */
float psvr2_chap_yaw(const float q[4]);

/* Play-area block that the chaperone_info writer reads (0x830 bytes). */
#define PSVR2_CHAP_MAX_POINTS 256
struct psvr2_chap_area {
	uint32_t flags;			/* +0x000, 0 in the default area */
	float width, depth;		/* +0x004, "play_area" */
	uint32_t count;			/* +0x00c, polygon points */
	float pt[PSVR2_CHAP_MAX_POINTS][2];	/* +0x010, (x, z) per point */
	float seated[4];		/* +0x810, translation x, y, z, then yaw */
	float standing[4];		/* +0x820 */
};

/*
 * Default area around the headset pose hmd (pose7 in tracking space):
 * a 24-point circle of radius 1.5 m, bounding size sqrt(2) * 1.5, seated
 * origin at the headset, standing origin 1.5 m below it, both rotated by
 * the headset's inverse yaw. Sony 0x18011ef90 (which first reads the
 * pose from shared memory, 0x18011f1b0).
 */
void psvr2_chap_default_area(const float hmd[7], struct psvr2_chap_area *out);

#endif
