// SPDX-License-Identifier: GPL-2.0
/*
 * Headset INS orchestration (see psvr2_ins.h). Sony's sequence is
 * FUN_18017cdb0 (docs/sony-re/controller-tracking.md §A.3.1); the numeric kernels are the bit-exact
 * ports in sie/irtracker/psvr2_ins_core.c.
 */
#include "psvr2_ins.h"

#include <math.h>
#include <string.h>

#define POS_GATE	0.5f		/* blender +0xa0 (FUN_180181400) */
#define ROT_GATE	0.1f		/* blender +0xac */
#define PREDICT_MAX	0.05f		/* INS +0x53c */
#define SIXDOF_TIMEOUT	500000000LL	/* 500 ms without valid SLAM */
#define IMU_STALL	100000000LL	/* 100 ms without IMU */
#define BATCH		64

/* ---- frame candidates --------------------------------------------------- */

/* 24 proper signed permutations: body = M * imu. */
static float g_maps[24][9];
static int g_maps_ready;

static void build_maps(void)
{
	static const int perm[6][3] = { { 0, 1, 2 }, { 0, 2, 1 }, { 1, 0, 2 }, { 1, 2, 0 }, { 2, 0, 1 }, { 2, 1, 0 } };
	static const int psign[6] = { 1, -1, -1, 1, 1, -1 };
	int n = 0;

	if (g_maps_ready)
		return;
	for (int p = 0; p < 6; p++)
		for (int s = 0; s < 8; s++) {
			const int sx = (s & 1) ? -1 : 1, sy = (s & 2) ? -1 : 1, sz = (s & 4) ? -1 : 1;
			if (psign[p] * sx * sy * sz != 1)
				continue;
			const int sg[3] = { sx, sy, sz };
			memset(g_maps[n], 0, sizeof(g_maps[n]));
			for (int r = 0; r < 3; r++)
				g_maps[n][3 * r + perm[p][r]] = (float)sg[r];
			n++;
		}
	g_maps_ready = 1;
}

static void map3(float *o, const float *M, const float *v)
{
	for (int r = 0; r < 3; r++)
		o[r] = M[3 * r] * v[0] + M[3 * r + 1] * v[1] + M[3 * r + 2] * v[2];
}

/* libpsvr2 quaternion -> INS x,y,z,w for order k, and back. */
static void decode_q(float *q, const float *o, int k)
{
	switch (k) {
	case 0: q[0] = o[0]; q[1] = o[1]; q[2] = o[2]; q[3] = o[3]; break;
	case 1: q[0] = o[1]; q[1] = o[2]; q[2] = o[3]; q[3] = o[0]; break;
	case 2: q[0] = -o[0]; q[1] = -o[1]; q[2] = -o[2]; q[3] = o[3]; break;
	default: q[0] = -o[1]; q[1] = -o[2]; q[2] = -o[3]; q[3] = o[0]; break;
	}
}

static void encode_q(float *o, const float *q, int k)
{
	switch (k) {
	case 0: o[0] = q[0]; o[1] = q[1]; o[2] = q[2]; o[3] = q[3]; break;
	case 1: o[0] = q[3]; o[1] = q[0]; o[2] = q[1]; o[3] = q[2]; break;
	case 2: o[0] = -q[0]; o[1] = -q[1]; o[2] = -q[2]; o[3] = q[3]; break;
	default: o[0] = q[3]; o[1] = -q[0]; o[2] = -q[1]; o[3] = -q[2]; break;
	}
}

static void qnorm(float *q)
{
	const float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (n > 0.0f)
		for (int i = 0; i < 4; i++)
			q[i] /= n;
}

/* Rotation vector of a unit quaternion (x,y,z,w). */
static void rotvec(double *r, const float *q)
{
	double x = q[0], y = q[1], z = q[2], w = q[3];
	if (w < 0) {
		x = -x; y = -y; z = -z; w = -w;
	}
	const double s = sqrt(x * x + y * y + z * z);
	const double k = s > 1e-12 ? 2.0 * atan2(s, w) / s : 2.0;
	r[0] = x * k;
	r[1] = y * k;
	r[2] = z * k;
}

static void qrot(float *o, const float *q, const float *v)
{
	const float t[3] = { 2.0f * (q[1] * v[2] - q[2] * v[1]), 2.0f * (q[2] * v[0] - q[0] * v[2]),
			     2.0f * (q[0] * v[1] - q[1] * v[0]) };
	o[0] = v[0] + q[3] * t[0] + (q[1] * t[2] - q[2] * t[1]);
	o[1] = v[1] + q[3] * t[1] + (q[2] * t[0] - q[0] * t[2]);
	o[2] = v[2] + q[3] * t[2] + (q[0] * t[1] - q[1] * t[0]);
}

/* ---- setup -------------------------------------------------------------- */

void psvr2_ins_init(struct psvr2_ins *ins, float accel_scale, float gyro_scale)
{
	memset(ins, 0, sizeof(*ins));
	build_maps();
	ins->period = 0.0005f;
	ins->accel_scale = accel_scale;
	ins->gyro_scale = gyro_scale;
	ins->frame = -1;
	ins->last_reset = -1;
	ins->last_map = 0xffffffffu;
}

int psvr2_ins_set_calibration(struct psvr2_ins *ins, const void *t12, unsigned int n12, const void *t13,
			      unsigned int n13)
{
	struct psvr2_ins_imucal c;

	if (!psvr2_ins_imucal_build(&c, t12, n12, t13, n13))
		return -1;
	/*
	 * Sanity [ours]: the diagonal must be within 20 % of the IIO scale, so
	 * a misread record never replaces a working fallback.
	 */
	for (int i = 0; i < 3; i++) {
		const float a = fabsf(c.A[5 * i]), g = fabsf(c.G[5 * i]);
		if (!(a > 0.8f * ins->accel_scale && a < 1.25f * ins->accel_scale) ||
		    !(g > 0.8f * ins->gyro_scale && g < 1.25f * ins->gyro_scale))
			return -1;
	}
	ins->cal = c;
	return 0;
}

/* ---- IMU ---------------------------------------------------------------- */

static unsigned int ring_idx(const struct psvr2_ins *ins, unsigned int age)
{
	/* age 0 = newest */
	return (ins->head + PSVR2_INS_RING - 1 - age) % PSVR2_INS_RING;
}

void psvr2_ins_push_imu(struct psvr2_ins *ins, const int16_t *accel, const int16_t *gyro, int64_t t_ns)
{
	struct psvr2_ins_sample s;

	t_ns -= PSVR2_INS_GROUP_DELAY_NS;
	if (ins->count && t_ns <= ins->ring_ns[ring_idx(ins, 0)])
		return;
	if (ins->cal.valid) {
		const int16_t raw[6] = { accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2] };
		psvr2_ins_imucal_apply(&ins->cal, raw, s.accel, s.gyro);
	} else {
		for (int i = 0; i < 3; i++) {
			s.accel[i] = (float)accel[i] * ins->accel_scale;
			s.gyro[i] = (float)gyro[i] * ins->gyro_scale;
		}
	}
	s.t_us = (int32_t)(t_ns / 1000);
	ins->ring[ins->head] = s;
	ins->ring_ns[ins->head] = t_ns;
	ins->head = (ins->head + 1) % PSVR2_INS_RING;
	if (ins->count < PSVR2_INS_RING)
		ins->count++;

	/* Mean period over the ring, accepted in (400, 600) us like Sony. */
	if (ins->count >= 100 && ins->head % 64 == 0) {
		const double span = (double)(ins->ring_ns[ring_idx(ins, 0)] - ins->ring_ns[ring_idx(ins, ins->count - 1)]);
		const double p = span / (ins->count - 1) * 1e-9;
		if (p > 400e-6 && p < 600e-6)
			ins->period = (float)p;
	}
}

/* Body-frame copy of samples [age_hi .. age_lo] (oldest first). */
static unsigned int gather(const struct psvr2_ins *ins, struct psvr2_ins_sample *out, unsigned int age_hi,
			   unsigned int n)
{
	const float *M = g_maps[ins->frame % 24];
	for (unsigned int i = 0; i < n; i++) {
		const struct psvr2_ins_sample *s = &ins->ring[ring_idx(ins, age_hi - i)];
		map3(out[i].accel, M, s->accel);
		map3(out[i].gyro, M, s->gyro);
		out[i].t_us = s->t_us;
	}
	return n;
}

/* Number of ring samples newer than t_ns. */
static unsigned int newer_than(const struct psvr2_ins *ins, int64_t t_ns)
{
	unsigned int n = 0;
	while (n < ins->count && ins->ring_ns[ring_idx(ins, n)] > t_ns)
		n++;
	return n;
}

static void tilt_correct(struct psvr2_ins_state *s, const struct psvr2_ins_sample *b, unsigned int n, float T)
{
	float m[3] = { 0, 0, 0 }, w[3];
	for (unsigned int i = 0; i < n; i++)
		for (int k = 0; k < 3; k++)
			m[k] += (b[i].accel[k] - s->accel_bias[k]) / (float)n;
	qrot(w, s->pose + 3, m);
	const float an = sqrtf(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
	if (fabsf(an - 9.8f) > 0.5f)
		return;
	/* error = measured up x world up, rotate toward it at 0.5 /s */
	const float e[3] = { -w[2] / an, 0.0f, w[0] / an };
	const float k = 0.5f * T * 0.5f;
	const float d[4] = { e[0] * k, e[1] * k, e[2] * k, 1.0f };
	float q[4];
	psvr2_ins_quat_mul(q, d, s->pose + 3);
	qnorm(q);
	memcpy(s->pose + 3, q, sizeof(q));
}

void psvr2_ins_tick(struct psvr2_ins *ins)
{
	struct psvr2_ins_sample b[BATCH];
	unsigned int pend;

	if (!ins->count)
		return;
	if (ins->frame < 0 || !ins->have_state) {
		ins->t_ins_ns = ins->ring_ns[ring_idx(ins, 0)];
		return;
	}
	pend = newer_than(ins, ins->t_ins_ns);
	if (pend && ins->ring_ns[ring_idx(ins, pend - 1)] - ins->t_ins_ns > IMU_STALL) {
		/* IMU stall: restart the time base, keep the pose, drop to 3DoF. */
		ins->t_ins_ns = ins->ring_ns[ring_idx(ins, 0)];
		ins->sixdof = 0;
		memset(ins->out.vel, 0, sizeof(ins->out.vel));
		return;
	}
	while (pend) {
		const unsigned int n = pend < BATCH ? pend : BATCH;
		const float T = (float)n * ins->period;
		float hold[3];

		gather(ins, b, pend - 1, n);
		memcpy(hold, ins->out.pose, sizeof(hold));
		psvr2_ins_strapdown(&ins->out, &ins->out, ins->period, n, b);
		psvr2_ins_strapdown(&ins->vio, &ins->vio, ins->period, n, b);
		if (ins->sixdof) {
			psvr2_ins_blend1(ins->out.pose, ins->vio.pose, T, POS_GATE, ROT_GATE, ins->out.vel,
					 ins->vio.vel);
			memcpy(ins->out.gyro_bias, ins->vio.gyro_bias, sizeof(ins->out.gyro_bias));
			memcpy(ins->out.accel_bias, ins->vio.accel_bias, sizeof(ins->out.accel_bias));
		} else {
			memcpy(ins->out.pose, hold, sizeof(hold));
			memset(ins->out.vel, 0, sizeof(ins->out.vel));
			tilt_correct(&ins->out, b, n, T);
		}
		/* "INS filtered pose is corrupted ... Use VIO pose" */
		if (!psvr2_ins_pose_ok(ins->out.pose))
			ins->out = ins->vio;
		else
			qnorm(ins->out.pose + 3);
		qnorm(ins->vio.pose + 3);
		ins->t_ins_ns = ins->ring_ns[ring_idx(ins, pend - n)];
		pend -= n;
	}
	if (ins->sixdof && ins->t_ins_ns - ins->last_valid_ns > SIXDOF_TIMEOUT)
		ins->sixdof = 0;
}

/* ---- frame detection [ours] --------------------------------------------- */

static void detect(struct psvr2_ins *ins, const struct psvr2_ins_slam *s)
{
	const struct psvr2_ins_slam *p = &ins->det_prev;
	const int64_t dt = s->t_ns - p->t_ns;

	if (!ins->det_have || !p->valid || !s->valid || p->reset_count != s->reset_count || dt < 3000000 ||
	    dt > 40000000)
		return;
	/* gyro integral over (t_prev, t_cur] */
	unsigned int hi = newer_than(ins, p->t_ns), lo = newer_than(ins, s->t_ns);
	if (hi <= lo || (double)(hi - lo) < 0.8 * (double)dt * 1e-9 / ins->period)
		return;
	double th[3] = { 0, 0, 0 };
	for (unsigned int a = lo; a < hi; a++)
		for (int k = 0; k < 3; k++)
			th[k] += ins->ring[ring_idx(ins, a)].gyro[k] * (double)ins->period;
	const double m2 = th[0] * th[0] + th[1] * th[1] + th[2] * th[2];
	if (m2 < 1e-4)
		return; /* not moving enough to tell */
	for (int k = 0; k < 4; k++) {
		float q0[4], q1[4], c[4], r[4];
		double rv[3];
		decode_q(q0, p->quat, k);
		decode_q(q1, s->quat, k);
		qnorm(q0);
		qnorm(q1);
		c[0] = -q0[0]; c[1] = -q0[1]; c[2] = -q0[2]; c[3] = q0[3];
		psvr2_ins_quat_mul(r, c, q1);
		rotvec(rv, r);
		for (int m = 0; m < 24; m++) {
			const float *M = g_maps[m];
			double e = 0;
			for (int i = 0; i < 3; i++) {
				const double v = M[3 * i] * th[0] + M[3 * i + 1] * th[1] + M[3 * i + 2] * th[2];
				e += (rv[i] - v) * (rv[i] - v);
			}
			ins->det_err[k * 24 + m] += e;
		}
	}
	ins->det_motion += m2;
	ins->det_pairs++;
	if (ins->det_pairs < 200 || ins->det_motion < 0.05)
		return;
	/*
	 * Some candidates tie exactly: a component permutation of the
	 * quaternion that equals q (x) k for a constant k relabels the body
	 * axes, and the matching axis map absorbs it. Those give the same
	 * output, so accept the tie group and prefer the identity IMU map
	 * (the SLAM biases are then in the right axes). Everything outside the
	 * group must fit at least 4x worse.
	 */
	int best = 0;
	for (int i = 1; i < PSVR2_INS_NFRAME; i++)
		if (ins->det_err[i] < ins->det_err[best])
			best = i;
	const double eb = ins->det_err[best], tie = 1.5 * eb + 1e-9 * ins->det_motion;
	double other = INFINITY;
	int pick = best;
	for (int i = 0; i < PSVR2_INS_NFRAME; i++) {
		if (ins->det_err[i] <= tie) {
			if (i % 24 == 0 && pick % 24 != 0)
				pick = i;
		} else if (ins->det_err[i] < other) {
			other = ins->det_err[i];
		}
	}
	if (eb < 0.1 * ins->det_motion && other > 4.0 * eb)
		ins->frame = pick;
}

/* ---- SLAM --------------------------------------------------------------- */

static int bad_vec(const float *v, float lim)
{
	const float n = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
	return isnan(v[0]) || isnan(v[1]) || isnan(v[2]) || n > lim * lim;
}

void psvr2_ins_push_slam(struct psvr2_ins *ins, const struct psvr2_ins_slam *s)
{
	if (ins->frame < 0) {
		detect(ins, s);
		ins->det_prev = *s;
		ins->det_have = 1;
		if (ins->frame < 0)
			return;
	}

	/* Quality monitor (FUN_180181260): error EMA; bad() = err > 2.5. */
	if (s->t_ns - ins->q_prev_ns <= 100000000LL && s->valid && s->variance > 0.0f) {
		const float e = fminf(sqrtf(s->variance), 10.0f);
		ins->q_err += (e - ins->q_err) * 0.04f;
	}
	ins->q_prev_ns = s->t_ns;
	if (ins->sixdof && ins->q_err > 2.5f)
		ins->sixdof = 0;

	if (s->reset_count != ins->last_reset) {
		if (ins->last_reset >= 0) {
			ins->sixdof = 0; /* "Detect SLAM Reset" */
			ins->latched = 0;
			ins->resets++;
		}
		ins->last_reset = s->reset_count;
	}
	if ((s->map_id >> 16) != (ins->last_map >> 16)) {
		ins->latched = 0; /* "Map Latch" */
		ins->last_map = s->map_id;
	}
	if (!s->valid)
		return;

	/* Reset the VIO filter to the SLAM state (FUN_18017b850/FUN_180180200). */
	struct psvr2_ins_state v;
	memset(&v, 0, sizeof(v));
	memcpy(v.pose, s->pos, sizeof(s->pos));
	decode_q(v.pose + 3, s->quat, ins->frame / 24);
	qnorm(v.pose + 3);
	if (!psvr2_ins_pose_ok(v.pose)) {
		memset(v.pose, 0, sizeof(v.pose));
		v.pose[6] = 1.0f;
	}
	if (!bad_vec(s->vel, 1e9f))
		memcpy(v.vel, s->vel, sizeof(v.vel));
	if (!bad_vec(s->gyro_bias, 1.0f))
		memcpy(v.gyro_bias, s->gyro_bias, sizeof(v.gyro_bias));
	if (!bad_vec(s->accel_bias, 1.7f))
		memcpy(v.accel_bias, s->accel_bias, sizeof(v.accel_bias));
	v.gravity[1] = -9.8f;
	v.delta[6] = 1.0f;

	/* Previous sample (at or before t_slam) seeds the trapezoid. */
	const unsigned int n = newer_than(ins, s->t_ns);
	if (n < ins->count) {
		struct psvr2_ins_sample p;
		gather(ins, &p, n, 1);
		for (int k = 0; k < 3; k++) {
			v.prev_accel[k] = p.accel[k] - v.accel_bias[k];
			v.prev_gyro[k] = p.gyro[k] - v.gyro_bias[k];
		}
	}
	/* Integrate (t_slam, newest]. */
	unsigned int left = n;
	while (left) {
		struct psvr2_ins_sample b[BATCH];
		const unsigned int k = left < BATCH ? left : BATCH;
		gather(ins, b, left - 1, k);
		psvr2_ins_strapdown(&v, &v, ins->period, k, b);
		qnorm(v.pose + 3);
		left -= k;
	}
	ins->vio = v;
	/*
	 * "INS latched Slam delta pose": output starts at the VIO state. Also
	 * on 3DoF -> 6DoF [ours]: the held 3DoF position is stale, and the
	 * mode-1 blend would swim toward SLAM for ~250 ms.
	 */
	if (!ins->have_state || !ins->latched || (!ins->sixdof && ins->q_err <= 2.5f)) {
		ins->out = v;
		ins->latched = 1;
		ins->latches++;
	}
	if (!ins->have_state || n == 0)
		ins->t_ins_ns = n ? ins->ring_ns[ring_idx(ins, 0)] : (s->t_ns > ins->t_ins_ns ? s->t_ns : ins->t_ins_ns);
	ins->have_state = 1;
	if (s->t_ns - ins->last_valid_ns > 24000000LL && ins->last_valid_ns)
		ins->slam_drops++;
	ins->last_valid_ns = s->t_ns;
	if (ins->q_err <= 2.5f)
		ins->sixdof = 1;
}

/* ---- output ------------------------------------------------------------- */

int psvr2_ins_ready(const struct psvr2_ins *ins)
{
	return ins->frame >= 0 && ins->have_state;
}

int psvr2_ins_query(const struct psvr2_ins *ins, int64_t t_ns, float *pos, float *quat, float *vel)
{
	float o[7];
	float dt;

	if (!psvr2_ins_ready(ins))
		return 0;
	dt = (float)((double)(t_ns - ins->t_ins_ns) * 1e-9);
	if (!(dt > 0.0f))
		dt = 0.0f;
	if (dt > PREDICT_MAX)
		dt = PREDICT_MAX;
	psvr2_ins_predict(o, &ins->out, dt);
	memcpy(pos, o, 3 * sizeof(float));
	encode_q(quat, o + 3, ins->frame / 24);
	if (vel)
		memcpy(vel, ins->out.vel, sizeof(ins->out.vel));
	return ins->sixdof ? 1 : 2;
}

int psvr2_ins_frame(const struct psvr2_ins *ins, int *order, int *map, double *residual)
{
	if (ins->frame < 0)
		return -1;
	if (order)
		*order = ins->frame / 24;
	if (map)
		*map = ins->frame % 24;
	if (residual)
		*residual = ins->det_motion > 0 ? ins->det_err[ins->frame] / ins->det_motion : 0.0;
	return 0;
}
