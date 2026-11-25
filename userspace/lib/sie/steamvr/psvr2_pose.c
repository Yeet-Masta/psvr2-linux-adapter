// SPDX-License-Identifier: GPL-2.0
/*
 * Sense controller DriverPose_t, see docs/sony-re/steamvr-and-display.md part A
 * ("Controller pose").
 */
#include <math.h>
#include <string.h>
#include "psvr2_steamvr.h"
#include "psvr2_quat.h"

/* Rotate v by q (normalised first). */
static void qrot(const float q[4], const float v[3], float o[3])
{
	const float n2 = ((q[1] * q[1] + q[2] * q[2]) + q[3] * q[3]) + q[0] * q[0];
	const float n = sqrtf(n2);
	const float w = q[0] / n, x = q[1] / n, y = q[2] / n, z = q[3] / n;
	const float w2 = w + w, x2 = x + x, y2 = y + y, z2 = z + z;
	const float wy = w2 * y, yy = y2 * y, xy = x2 * y, zz = z2 * z, yz = y2 * z;
	const float wx = w2 * x, wz = w2 * z, xz = x2 * z, xx = x2 * x;
	/* rows of the 3x3 part of the 4x4 matrix */
	const float m0 = (1.0f - yy) - zz, m1 = wz + xy, m2 = xz - wy;
	const float m4 = xy - wz, m5 = (1.0f - xx) - zz, m6 = wx + yz;
	const float m8 = xz + wy, m9 = yz - wx, m10 = (1.0f - xx) - yy;

	o[0] = (v[0] * m0 + v[1] * m4) + v[2] * m8;
	o[1] = (v[0] * m1 + v[1] * m5) + v[2] * m9;
	o[2] = (v[0] * m2 + v[1] * m6) + v[2] * m10;
}

/* Tip frame: 39 degrees about +x (Sony computes cos/sin of 19.5 degrees at
 * run time; these are its results). */
static const float Q_TIP[4] = { 0x1.e2a1e8p-1f, 0x1.55d178p-2f, 0.0f, 0.0f };

void psvr2_steamvr_sense_pose(int side, const psvr2_tracker_pose *t, int connected, int hold,
			      int invalidate, const psvr2_driver_pose *prev, psvr2_driver_pose *o)
{
	/* Origin of the tracked frame relative to the tracker pose (m), and the
	 * grip offset (mm), per side. */
	static const float base[2][3] = { { 0x1.d53cdep-7f, -0x1.f97aaap-8f, -0x1.a9f16cp-4f },
					  { -0x1.d53cdep-7f, -0x1.f97aaap-8f, -0x1.a9f16cp-4f } };
	static const float grip[2][3] = { { 0x1.2bed28p+3f, -0x1.87f07p+3f, -0x1.803fe6p+2f },
					  { -0x1.4129c8p+4f, -0x1.87f07p+3f, -0x1.803fe6p+2f } };
	const int r = side != PSVR2_SIDE_LEFT;

	memset(o, 0, sizeof(*o));
	if (!connected)
		return;
	if (hold) {
		o->connected = 1;
		if (prev->connected)
			*o = *prev;
		memset(o->vel, 0, sizeof(o->vel));
		memset(o->ang_vel, 0, sizeof(o->ang_vel));
		if (invalidate)
			o->valid = 0;
		return;
	}
	float q[4], off[3], rot[3];

	o->q_world_from_driver[0] = 1.0;
	o->q_driver_from_head[0] = 1.0;
	psvr2_qmul(t->q, Q_TIP, q);
	for (int i = 0; i < 3; i++) {
		const float g = grip[r][i] * 0.001f;

		off[i] = base[r][i] + g;
		o->driver_from_head[i] = (double)-g;
	}
	qrot(q, off, rot);
	for (int i = 0; i < 3; i++) {
		o->pos[i] = (double)t->pos[i] + (double)rot[i];
		o->vel[i] = (double)t->vel[i];
		o->ang_vel[i] = (double)t->ang_vel[i];
	}
	for (int i = 0; i < 4; i++)
		o->q[i] = (double)q[i];
	o->result = 200;
	o->valid = t->valid;
	o->connected = 1;
}

int psvr2_steamvr_hmd_pose(psvr2_hmd_pose_state *st, const psvr2_hmd_pose_in *in, psvr2_driver_pose *o)
{
	const psvr2_tracker_pose *t = in->trk;

	memset(o, 0, sizeof(*o));
	if (!in->ready) {
		*o = st->cache;
		o->valid = 1;
		o->connected = 0;
	} else if (!in->paused) {
		o->q_world_from_driver[0] = 1.0;
		for (int i = 0; i < 4; i++) {
			o->q_driver_from_head[i] = (double)in->head_q[i];
			o->q[i] = (double)t->q[i];
		}
		for (int i = 0; i < 3; i++) {
			o->driver_from_head[i] = (double)in->head_t[i];
			o->pos[i] = (double)t->pos[i];
			o->vel[i] = (double)t->vel[i];
			o->ang_vel[i] = (double)t->ang_vel[i];
		}
		o->result = in->needs_update ? 1 : 200;
		o->valid = t->valid;
		o->connected = in->ready;
		st->cache = *o;
	} else {
		o->connected = in->ready;
		if (st->prev.connected)
			*o = st->prev;
		memset(o->vel, 0, sizeof(o->vel));
		memset(o->ang_vel, 0, sizeof(o->ang_vel));
	}
	int fresh = 1, send = 1;

	if (!in->worn) {
		const psvr2_driver_pose *l = &st->last_sent;

		fresh = l->connected != o->connected || l->valid != o->valid || l->result != o->result;
		for (int i = 0; i < 3 && !fresh; i++)
			/* Signed tests (not |d|): only a move towards -axis or a
			 * positive angular rate counts, as in the reference. */
			fresh = !(l->pos[i] - o->pos[i] < (double)0.005f) || 0.03 <= o->ang_vel[i];
		/* Not worn and still: keep reporting for 500 ms, then go quiet. */
		if (!fresh)
			send = (uint64_t)(in->t_sample_us - st->last_fresh_us) < 500001;
	}
	if (fresh) {
		st->last_sent = *o;
		st->last_fresh_us = in->t_sample_us;
	}
	if (send)
		o->time_offset = (double)((float)(in->t_sample_us - in->t_send_us) * 0.001f * 0.001f);
	st->prev = *o;
	return send;
}
