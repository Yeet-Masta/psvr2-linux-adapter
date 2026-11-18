/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Sense controller IMU calibration (HID feature report 0x05, two parts).
 * Written from docs/sony-driver-re.md; checked bit-for-bit by
 * scripts/sony-re/difftest.
 */
#ifndef PSVR2_SIE_SENSECAL_H
#define PSVR2_SIE_SENSECAL_H
#include <stdint.h>

typedef struct {
	float gyro_bias[3];   /* raw LSB */
	float gyro_scale[3];  /* rad/s per LSB */
	float accel_offset[3];/* raw LSB */
	float accel_scale[3]; /* g per LSB */
} psvr2_sense_cal;

/* c: the 42 s16 values (part 0 bytes 2.., then part 1 bytes 2..).
 * Returns 0 on success, -1 if any gyro range is zero. */
int psvr2_sense_cal_compute(const int16_t c[42], psvr2_sense_cal *out);
#endif
