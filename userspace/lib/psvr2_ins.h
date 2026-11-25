/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Headset INS: Sony's host-side pose path (docs/sony-re/controller-tracking.md §A.3.1,
 * §5.1, §5.2). The headset sends its on-device SLAM pose (EP 0x83) at a
 * low rate; this keeps two strapdown filters over the 2 kHz IMU, a "VIO"
 * filter reset to every valid SLAM sample and re-integrated up to the
 * newest IMU sample, and an output filter pulled toward it by the mode-1
 * blender. Queries predict the output filter to the asked time (<= 50 ms).
 *
 * The kernels (strapdown, rotation increment, prediction, blend, IMU
 * calibration) are Sony's, bit-exact (sie/irtracker/psvr2_ins_core.c). The
 * orchestration follows Sony's sequence with these differences [ours]:
 * - Frame check: the quaternion order of the SLAM record and the IMU axes
 *   are found from the data (gyro integral vs SLAM rotation over 96
 *   candidates) before the INS is used; until then psvr2_ins_ready() is 0.
 * - 6DoF entry needs only a valid SLAM sample and no bad quality (Sony also
 *   wants n > 30 good samples and a +0x188 value we do not get).
 * - 3DoF tilt correction is a plain complementary step (Sony's
 *   FUN_180171400 is not ported). The output position is held in 3DoF.
 * - Blender: mode 1 on the pose only. Sony's mode-0 blender moves the
 *   output's delta pose toward the VIO delta from the SLAM sample
 *   (controller-tracking.md §A.5.2); here both deltas stay the identity.
 */
#ifndef PSVR2_INS_H
#define PSVR2_INS_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "sie/irtracker/psvr2_ins_core.h"

#define PSVR2_INS_RING		1024	/* 0.5 s of IMU */
#define PSVR2_INS_NFRAME	96	/* 4 quaternion orders x 24 axis maps */
#define PSVR2_INS_GROUP_DELAY_NS 1250000 /* Sony subtracts 1250 us */

struct psvr2_ins_slam {
	int64_t t_ns;			/* capture time, host clock */
	int valid;			/* Sony's rule (state 3, flag, substate 5) */
	int reset_count;
	uint32_t map_id;
	float pos[3];			/* raw frame, Y up */
	float quat[4];			/* exactly as libpsvr2 reports it */
	float vel[3], gyro_bias[3], accel_bias[3];
	float variance;			/* record +0x184 */
};

struct psvr2_ins {
	struct psvr2_ins_sample ring[PSVR2_INS_RING];	/* IMU frame, calibrated */
	int64_t ring_ns[PSVR2_INS_RING];
	unsigned int head, count;
	int64_t t_ins_ns;		/* newest integrated sample */
	float period;			/* s, mean sample period */

	struct psvr2_ins_imucal cal;
	float accel_scale, gyro_scale;	/* used without a calibration */

	struct psvr2_ins_state vio, out;
	int have_state, sixdof, latched;
	int64_t last_valid_ns;
	int last_reset;
	uint32_t last_map;

	float q_err;			/* SLAM quality monitor (§4.1) */
	int64_t q_prev_ns;

	int frame;			/* -1 until found */
	double det_err[PSVR2_INS_NFRAME], det_motion;
	int det_pairs;
	struct psvr2_ins_slam det_prev;
	int det_have;

	unsigned int latches, resets, slam_drops;
};

void psvr2_ins_init(struct psvr2_ins *ins, float accel_scale, float gyro_scale);
/* Use the factory calibration (VD types 12 and 13). Returns 0 if taken. */
int psvr2_ins_set_calibration(struct psvr2_ins *ins, const void *imu_calib, unsigned int n12,
			      const void *imu_set_calib, unsigned int n13);
/* One raw IMU sample (accel s16[3], gyro s16[3]) at its capture time. */
void psvr2_ins_push_imu(struct psvr2_ins *ins, const int16_t *accel, const int16_t *gyro, int64_t t_ns);
/* Integrate the pushed samples into both filters. */
void psvr2_ins_tick(struct psvr2_ins *ins);
/* One SLAM sample. Call psvr2_ins_tick() first so the IMU is current. */
void psvr2_ins_push_slam(struct psvr2_ins *ins, const struct psvr2_ins_slam *s);
/* 1 once the frame is known and a SLAM sample was latched. */
int psvr2_ins_ready(const struct psvr2_ins *ins);
/*
 * Pose at t_ns (predicted at most 50 ms past the newest IMU sample):
 * position and velocity in the raw frame, quaternion in libpsvr2's order
 * so callers can treat it like a SLAM sample. Returns 1 in 6DoF, 2 in 3DoF
 * (orientation only), 0 if not ready.
 */
int psvr2_ins_query(const struct psvr2_ins *ins, int64_t t_ns, float *pos, float *quat, float *vel);
/* Found frame: quaternion order 0..3 and axis map 0..23, or -1. */
int psvr2_ins_frame(const struct psvr2_ins *ins, int *quat_order, int *axis_map, double *residual);

#ifdef __cplusplus
}
#endif
#endif
