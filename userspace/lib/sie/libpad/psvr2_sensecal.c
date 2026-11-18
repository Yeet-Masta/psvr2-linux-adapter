// SPDX-License-Identifier: GPL-2.0
/* See psvr2_sensecal.h. Build with -ffp-contract=off. */
#include "psvr2_sensecal.h"

#include <stdlib.h>

int psvr2_sense_cal_compute(const int16_t c[42], psvr2_sense_cal *out)
{
	static const int gyro[3][3] = { { 18, 21, 24 }, { 19, 28, 31 }, { 20, 35, 38 } };
	static const int accel[3][2] = { { 0, 3 }, { 7, 10 }, { 14, 17 } };
	int range[3];

	for (int a = 0; a < 3; a++)
		range[a] = abs(c[gyro[a][1]] - c[gyro[a][0]]) + abs(c[gyro[a][2]] - c[gyro[a][0]]);
	if (!range[0] || !range[1] || !range[2])
		return -1;

	const float speed = (float)(c[39] + c[40]) * 0.5f * 0.01745329238474369f;
	for (int a = 0; a < 3; a++) {
		out->gyro_bias[a] = (float)c[gyro[a][0]];
		out->gyro_scale[a] = speed / ((float)range[a] * 0.5f);
		const float r = (float)(abs(c[accel[a][0]]) + abs(c[accel[a][1]]));
		out->accel_offset[a] = (float)c[accel[a][0]] - r * 0.5f;
		out->accel_scale[a] = 2.0f / r;
	}
	return 0;
}
