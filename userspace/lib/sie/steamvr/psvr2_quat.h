// SPDX-License-Identifier: GPL-2.0
/*
 * Quaternion product shared by the steamvr files. Quaternions are
 * (w, x, y, z). The operation order follows the reference so the results
 * match bit for bit.
 */
#ifndef PSVR2_QUAT_H
#define PSVR2_QUAT_H

static inline void psvr2_qmul(const float a[4], const float b[4], float o[4])
{
	const float w = ((a[0] * b[0] - a[1] * b[1]) - a[2] * b[2]) - a[3] * b[3];
	const float x = ((b[3] * a[2] - a[3] * b[2]) + b[1] * a[0]) + a[1] * b[0];
	const float y = ((a[3] * b[1] - b[3] * a[1]) + b[2] * a[0]) + a[2] * b[0];
	const float z = ((b[2] * a[1] - a[2] * b[1]) + b[3] * a[0]) + a[3] * b[0];

	o[0] = w;
	o[1] = x;
	o[2] = y;
	o[3] = z;
}

#endif
