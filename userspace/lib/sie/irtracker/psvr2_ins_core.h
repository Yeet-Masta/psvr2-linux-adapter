/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Headset INS core kernels: strapdown propagation, rotation increment,
 * output prediction, mode-1 blend, sanity check, IMU calibration.
 *
 * Written from docs/sony-re/tracking.md §2.2, §5.1, §5.2 (behaviour only)
 * and checked against the reference driver by
 * scripts/sony-re/difftest/t_irtracker_inscore.c. Unlike psvr2_ins_math.h,
 * quaternions here are stored the way the INS stores them: x, y, z, w
 * (w last). A pose is 7 floats: position[3], quaternion[4].
 * Build with -ffp-contract=off.
 */
#ifndef PSVR2_SIE_INS_CORE_H
#define PSVR2_SIE_INS_CORE_H
#include <stdbool.h>
#include <stdint.h>

/* One calibrated IMU sample as the INS keeps it (28 bytes). */
struct psvr2_ins_sample {
	float accel[3];		/* m/s^2 */
	float gyro[3];		/* rad/s */
	int32_t t_us;		/* device time minus the 1250 us group delay */
};

/* One strapdown filter (Sony keeps two: "VIO" and output). */
struct psvr2_ins_state {
	float pose[7];		/* wTi: position, quaternion x,y,z,w */
	float vel[3];		/* wVi, m/s */
	float gyro_bias[3];
	float accel_bias[3];
	float gravity[3];	/* (0, -9.8, 0) */
	float prev_accel[3];	/* last accel - bias, body frame */
	float prev_gyro[3];	/* last gyro - bias */
	float delta[7];		/* output <- filter frame offset (latched) */
};

/* r = a (x) b, Hamilton product, x,y,z,w. r may alias neither input. */
void psvr2_ins_quat_mul(float *r, const float *a, const float *b);

/*
 * Rotation increment over one sample period from the previous and current
 * bias-free gyro: midpoint rotation vector plus the coning term
 * (w0 x w1) dt^2/24. Small-angle series below the threshold.
 */
void psvr2_ins_rotinc(float *q, float dt, const float *w0, const float *w1);

/*
 * Integrate n samples of period dt from s. Writes the new pose, velocity,
 * prev_accel and prev_gyro into out (biases, gravity and delta copied from
 * s). The new quaternion is not normalised (Sony does not).
 */
void psvr2_ins_strapdown(struct psvr2_ins_state *out, const struct psvr2_ins_state *s, float dt,
			 unsigned int n, const struct psvr2_ins_sample *smp);

/*
 * Predict dt seconds ahead (dt capped at 1 s; callers cap at 0.05 s) with
 * the last gyro and the velocity, then map through delta. On a failed
 * sanity check writes the identity pose and returns false.
 */
bool psvr2_ins_predict(float *out, const struct psvr2_ins_state *s, float dt);

/* Sony's pose sanity check: no NaN and |q|^2 in [0.9, 1.1]. */
bool psvr2_ins_pose_ok(const float *pose);

/*
 * Blender mode 1: snap to target when any position axis differs by more
 * than pos_gate or the rotation by more than rot_gate (rad), else move
 * min(4 dt, 1) of the way (lerp + slerp). A target with a NaN is ignored.
 * vel/tvel: the velocity copied from target to output on success (3 floats,
 * may be NULL).
 */
void psvr2_ins_blend1(float *out, const float *target, float dt, float pos_gate, float rot_gate, float *vel,
		      const float *tvel);

/* IMU calibration: a = A·raw + a_bias, w = G·raw + g_bias. */
struct psvr2_ins_imucal {
	float A[16], G[16];	/* rows: 3 basis rows + bias row (Sony 4x4) */
	bool valid;
};

/*
 * Build from the VD type 12 (imu_calib, 0x78 bytes) and type 13
 * (imu_set_calib, 0x34 bytes) records. Returns false on short input.
 */
bool psvr2_ins_imucal_build(struct psvr2_ins_imucal *c, const void *t12, unsigned int n12, const void *t13,
			    unsigned int n13);

/* Sony's 4x4 builder: rows i = R[3i..3i+2]*s[i], bias row = sum_i R row i * s[i] * off[i]. */
void psvr2_ins_imucal_mat(float *m, const float *R, const float *s, const float *off);

/* Convert one raw sample (accel s16[3], gyro s16[3]). */
void psvr2_ins_imucal_apply(const struct psvr2_ins_imucal *c, const int16_t *raw, float *accel, float *gyro);

#endif
