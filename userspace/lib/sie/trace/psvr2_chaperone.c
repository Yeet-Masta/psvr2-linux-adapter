// SPDX-License-Identifier: GPL-2.0
/*
 * VrTracker2 universe / chaperone helpers. See psvr2_chaperone.h and
 * docs/sony-re/trace.md section 10. Evaluation order is part of the
 * contract: every sum is written in the order Sony evaluates it.
 */
#include <math.h>
#include <string.h>
#include "psvr2_chaperone.h"

void psvr2_chap_pose_mul(const float a[7], const float b[7], float out[7])
{
	float ax = a[3], ay = a[4], az = a[5], aw = a[6];
	float bx = b[3], by = b[4], bz = b[5], bw = b[6];
	float tx, ty, tz, r[7];

	/* t = 2 (a.q.xyz x b.p) */
	tx = ay * b[2] - b[1] * az;
	ty = az * b[0] - ax * b[2];
	tz = ax * b[1] - ay * b[0];
	tx = tx + tx;
	ty = ty + ty;
	tz = tz + tz;
	/* p = a.p + b.p + w t + q.xyz x t */
	r[0] = ((aw * tx + b[0]) + (tz * ay - ty * az)) + a[0];
	r[1] = ((ty * aw + b[1]) + (az * tx - tz * ax)) + a[1];
	r[2] = ((tz * aw + b[2]) + (ty * ax - ay * tx)) + a[2];
	r[3] = (aw * bx + ay * bz) + (bw * ax - az * by);
	r[4] = (aw * by + az * bx) + (bw * ay - ax * bz);
	r[5] = (aw * bz + ax * by) + (bw * az - ay * bx);
	r[6] = (-(ay * by + az * bz)) + (bw * aw - ax * bx);
	memcpy(out, r, sizeof(r));
}

void psvr2_chap_pose_inv(const float a[7], float out[7])
{
	float cx = -a[3], cy = -a[4], cz = -a[5], cw = a[6];
	float p0 = a[0], p1 = a[1], p2 = a[2];
	float tx, ty, tz;

	/* t = 2 (conj.xyz x p), then -(p + w t + conj.xyz x t) */
	tz = p1 * cx - p0 * cy;
	tz = tz + tz;
	ty = p0 * cz - p2 * cx;
	tx = p2 * cy - p1 * cz;
	ty = ty + ty;
	tx = tx + tx;
	out[0] = -((tx * cw + p0) + (tz * cy - ty * cz));
	out[1] = -((ty * cw + p1) + (tx * cz - tz * cx));
	out[2] = -((tz * cw + p2) + (ty * cx - tx * cy));
	out[3] = cx;
	out[4] = cy;
	out[5] = cz;
	out[6] = cw;
}

void psvr2_chap_quat_to_mat3(const float q[4], float R[9])
{
	float x = q[0], y = q[1], z = q[2], w = q[3];
	float y2 = y + y, z2 = z + z;
	float xx2 = x * (x + x), wx2 = w * (x + x);

	R[0] = 1.0f - (z * z2 + y * y2);
	R[1] = x * y2 + w * z2;
	R[2] = x * z2 - w * y2;
	R[3] = x * y2 - w * z2;
	R[4] = 1.0f - (z * z2 + xx2);
	R[5] = y * z2 + wx2;
	R[6] = x * z2 + w * y2;
	R[7] = y * z2 - wx2;
	R[8] = 1.0f - (y * y2 + xx2);
}

float psvr2_chap_yaw(const float q[4])
{
	float R[9], s, n, c, sgn = 0.0f;
	float ax, ay, az, fx, fy, fz;

	psvr2_chap_quat_to_mat3(q, R);
	/* image of the x axis and of the z axis, as Sony forms them */
	ax = (R[6] * 0.0f + R[3] * 0.0f) + R[0];
	ay = (R[7] * 0.0f + R[4] * 0.0f) + R[1];
	az = (R[8] * 0.0f + R[5] * 0.0f) + R[2];
	fx = (R[3] * 0.0f + R[6]) + R[0] * 0.0f;
	fy = (R[4] * 0.0f + R[7]) + R[1] * 0.0f;
	fz = (R[5] * 0.0f + R[8]) + R[2] * 0.0f;

	if (fabsf(fy) <= fabsf(ay)) {
		/* the z axis is closer to horizontal: use its xz direction */
		n = (fz * fz + 0.0f) + fx * fx;
		c = fz;
		s = fx;
		if (0.0f < n) {
			float l = sqrtf(n);

			c = fz / l;
			s = fx / l;
		}
	} else {
		n = (az * az + 0.0f) + ax * ax;
		c = ax;
		s = az;
		if (!(n <= 0.0f)) {
			float l = sqrtf(n);

			c = ax / l;
			s = az / l;
		}
		s = -s;
	}
	if (s != 0.0f)
		sgn = (s / fabsf(s)) * -1.0f;
	return acosf(c) * sgn;
}

void psvr2_chap_default_area(const float hmd[7], struct psvr2_chap_area *out)
{
	const float r = 1.5f, step = 0.2617993950843811f; /* 15 degrees */
	float inv[4], n, yaw;
	int i;

	/* yaw of the inverse orientation: conjugate / |q|^2, zero quaternion unless |q|^2 > 0 */
	n = (hmd[3] * hmd[3] + hmd[5] * hmd[5]) + (hmd[4] * hmd[4] + hmd[6] * hmd[6]);
	if (!(n > 0.0f)) { /* NaN too */
		inv[0] = inv[1] = inv[2] = inv[3] = 0.0f;
	} else {
		inv[0] = (-hmd[3]) / n;
		inv[1] = (-hmd[4]) / n;
		inv[2] = (-hmd[5]) / n;
		inv[3] = hmd[6] / n;
	}
	yaw = psvr2_chap_yaw(inv);
	out->standing[0] = -hmd[0];
	out->standing[1] = -(hmd[1] - r);
	out->standing[2] = -hmd[2];
	out->standing[3] = yaw;
	out->seated[0] = -hmd[0];
	out->seated[1] = -hmd[1];
	out->seated[2] = -hmd[2];
	out->seated[3] = yaw;
	out->flags = 0;
	for (i = 0; i < 24; i++) {
		out->pt[i][0] = cosf((float)i * step) * r;
		out->pt[i][1] = sinf((float)i * step) * r;
	}
	out->count = 24;
	out->width = sqrtf(2.0f) * r;
	out->depth = sqrtf(2.0f) * r;
}
