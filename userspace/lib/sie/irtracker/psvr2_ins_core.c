// SPDX-License-Identifier: GPL-2.0
/*
 * Headset INS core kernels (see psvr2_ins_core.h). The float expression
 * order follows the reference so results match bit for bit; keep it.
 */
#include "psvr2_ins_core.h"

#include <math.h>
#include <string.h>

void psvr2_ins_quat_mul(float *r, const float *a, const float *b)
{
	r[0] = (a[3] * b[0] + a[1] * b[2]) + (b[3] * a[0] - a[2] * b[1]);
	r[1] = (a[3] * b[1] + a[2] * b[0]) + (b[3] * a[1] - a[0] * b[2]);
	r[2] = (a[3] * b[2] + a[0] * b[1]) + (b[3] * a[2] - a[1] * b[0]);
	r[3] = -(a[1] * b[1] + a[2] * b[2]) + (b[3] * a[3] - a[0] * b[0]);
}

void psvr2_ins_rotinc(float *q, float dt, const float *w0, const float *w1)
{
	const float h = dt * 0.5f;
	const float x = (w0[0] + w1[0]) * h, y = (w0[1] + w1[1]) * h, z = (w0[2] + w1[2]) * h;
	const float t2 = (y * y + x * x) + z * z;
	float s, c;

	/* The reference compares |theta|^2 against sqrt(2.29e-5) (~0.0048). */
	if (sqrtf(0.000022888184f) <= t2) {
		const float t = sqrtf(t2);
		s = sinf(t * 0.5f) / t;
		c = cosf(t * 0.5f);
	} else {
		s = 0.5f - ((y * y + x * x) + z * z) / 48.0f;
		c = 1.0f - t2 * 0.125f;
	}
	const float k = (dt * dt) / 24.0f;
	q[0] = (w0[1] * w1[2] - w0[2] * w1[1]) * k + x * s;
	q[1] = (w0[2] * w1[0] - w1[2] * w0[0]) * k + y * s;
	q[2] = (w0[0] * w1[1] - w1[0] * w0[1]) * k + z * s;
	q[3] = c;
}

/* r = q f q*, written the way the reference does (t = 2 q x f). */
static void rot(float *r, const float *q, const float *f)
{
	const float tz = (f[1] * q[0] - f[0] * q[1]) + (f[1] * q[0] - f[0] * q[1]);
	const float ty = (f[0] * q[2] - f[2] * q[0]) + (f[0] * q[2] - f[2] * q[0]);
	const float tx = (f[2] * q[1] - f[1] * q[2]) + (f[2] * q[1] - f[1] * q[2]);
	r[0] = (f[0] + tx * q[3]) + (tz * q[1] - ty * q[2]);
	r[1] = (f[1] + ty * q[3]) + (tx * q[2] - tz * q[0]);
	r[2] = (f[2] + tz * q[3]) + (ty * q[0] - tx * q[1]);
}

void psvr2_ins_strapdown(struct psvr2_ins_state *out, const struct psvr2_ins_state *s, float dt,
			 unsigned int n, const struct psvr2_ins_sample *smp)
{
	float wp[3] = { s->prev_gyro[0], s->prev_gyro[1], s->prev_gyro[2] };
	float fl[3] = { s->prev_accel[0], s->prev_accel[1], s->prev_accel[2] };
	float dq[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
	float dv[3] = { 0.0f, 0.0f, 0.0f }, dp[3] = { 0.0f, 0.0f, 0.0f };

	if (n) {
		const float hn = (float)n * 0.5f;
		float sa[3] = { fl[0] * 0.5f, fl[1] * 0.5f, fl[2] * 0.5f };
		float ssa[3] = { fl[0] * hn, fl[1] * hn, fl[2] * hn };

		for (unsigned int k = 0; k < n; k++) {
			const float w[3] = { smp[k].gyro[0] - s->gyro_bias[0], smp[k].gyro[1] - s->gyro_bias[1],
					     smp[k].gyro[2] - s->gyro_bias[2] };
			float qi[4], r[3];

			psvr2_ins_rotinc(qi, dt, wp, w);
			if (k == 0) {
				memcpy(dq, qi, sizeof(dq));
			} else {
				float t[4];
				psvr2_ins_quat_mul(t, dq, qi);
				memcpy(dq, t, sizeof(dq));
			}
			memcpy(wp, w, sizeof(wp));
			fl[0] = smp[k].accel[0] - s->accel_bias[0];
			fl[1] = smp[k].accel[1] - s->accel_bias[1];
			fl[2] = smp[k].accel[2] - s->accel_bias[2];
			rot(r, dq, fl);
			if (k + 1 == n) {
				r[0] *= 0.5f;
				r[1] *= 0.5f;
				r[2] *= 0.5f;
			} else {
				const float m = (float)(n - (k + 1));
				ssa[0] = ssa[0] + r[0] * m;
				ssa[1] = ssa[1] + r[1] * m;
				ssa[2] = ssa[2] + r[2] * m;
			}
			sa[0] = sa[0] + r[0];
			sa[1] = sa[1] + r[1];
			sa[2] = sa[2] + r[2];
		}
		dv[0] = sa[0] * dt;
		dv[1] = sa[1] * dt;
		dv[2] = sa[2] * dt;
		dp[0] = (dt * dt) * ssa[0];
		dp[1] = (dt * dt) * ssa[1];
		dp[2] = (dt * dt) * ssa[2];
	}

	const float nn = (dq[0] * dq[0] + dq[2] * dq[2]) + (dq[1] * dq[1] + dq[3] * dq[3]);
	if (nn > 0.0f) {
		const float l = sqrtf(nn);
		dq[0] /= l;
		dq[1] /= l;
		dq[2] /= l;
		dq[3] /= l;
	}

	const float T = (float)n * dt;
	const float *p = s->pose, *q = s->pose + 3, *v = s->vel, *g = s->gravity;
	const float qx = q[0], qy = q[1], qz = q[2], qw = q[3];
	struct psvr2_ins_state o = *s;

	psvr2_ins_quat_mul(o.pose + 3, q, dq);

	const float tz = (dv[1] * qx - qy * dv[0]) + (dv[1] * qx - qy * dv[0]);
	const float ty = (qz * dv[0] - dv[2] * qx) + (qz * dv[0] - dv[2] * qx);
	const float tx = (qy * dv[2] - qz * dv[1]) + (qy * dv[2] - qz * dv[1]);
	o.vel[0] = (((tz * qy - ty * qz) + (qw * tx + dv[0])) + v[0]) + T * g[0];
	o.vel[1] = (((qw * ty + dv[1]) + (tx * qz - tz * qx)) + v[1]) + T * g[1];
	o.vel[2] = (((qw * tz + dv[2]) + (ty * qx - tx * qy)) + v[2]) + T * g[2];

	const float a = dp[0] * qz - qx * dp[2];
	const float pz = (qx * dp[1] - dp[0] * qy) + (qx * dp[1] - dp[0] * qy);
	const float py = a + a;
	const float px = (dp[2] * qy - dp[1] * qz) + (dp[2] * qy - dp[1] * qz);
	const float TT = T * T;
	o.pose[0] = ((T * v[0] + p[0]) + ((pz * qy - py * qz) + (px * qw + dp[0]))) + (g[0] * 0.5f) * TT;
	o.pose[1] = ((T * v[1] + p[1]) + ((qw * py + dp[1]) + (px * qz - pz * qx))) + (g[1] * 0.5f) * TT;
	o.pose[2] = ((T * v[2] + p[2]) + (((qw * pz + dp[2]) + (py * qx - px * qy)))) + (g[2] * 0.5f) * TT;

	memcpy(o.prev_accel, fl, sizeof(fl));
	memcpy(o.prev_gyro, wp, sizeof(wp));
	*out = o;
}

bool psvr2_ins_pose_ok(const float *p)
{
	const float n = (p[3] * p[3] + p[5] * p[5]) + (p[4] * p[4] + p[6] * p[6]);
	for (int i = 0; i < 7; i++)
		if (isnan(p[i]))
			return false;
	return n >= 0.9f && n <= 1.1f;
}

bool psvr2_ins_predict(float *out, const struct psvr2_ins_state *s, float dt)
{
	float p[3] = { s->pose[0], s->pose[1], s->pose[2] };
	float q[4] = { s->pose[3], s->pose[4], s->pose[5], s->pose[6] };
	const float *A = s->delta;

	dt = fminf(dt, 1.0f);
	if (dt > 0.0f) {
		float qi[4], t[4];
		psvr2_ins_rotinc(qi, dt, s->prev_gyro, s->prev_gyro);
		p[0] = p[0] + s->vel[0] * dt;
		p[1] = p[1] + s->vel[1] * dt;
		p[2] = p[2] + s->vel[2] * dt;
		psvr2_ins_quat_mul(t, q, qi);
		memcpy(q, t, sizeof(q));
	}
	psvr2_ins_quat_mul(out + 3, A + 3, q);

	const float qx = A[3], qy = A[4], qz = A[5], qw = A[6];
	const float tz = (p[1] * qx - p[0] * qy) + (p[1] * qx - p[0] * qy);
	const float ty = (p[0] * qz - p[2] * qx) + (p[0] * qz - p[2] * qx);
	const float tx = (p[2] * qy - p[1] * qz) + (p[2] * qy - p[1] * qz);
	out[0] = ((tx * qw + p[0]) + (tz * qy - qz * ty)) + A[0];
	out[1] = ((ty * qw + p[1]) + (qz * tx - tz * qx)) + A[1];
	out[2] = ((tz * qw + p[2]) + (ty * qx - tx * qy)) + A[2];
	if (!psvr2_ins_pose_ok(out)) {
		memset(out, 0, 7 * sizeof(float));
		out[6] = 1.0f;
		return false;
	}
	return true;
}

void psvr2_ins_blend1(float *out, const float *tg, float dt, float pos_gate, float rot_gate, float *vel,
		      const float *tvel)
{
	for (int i = 0; i < 7; i++)
		if (isnan(tg[i]))
			return;

	const float dx = tg[0] - out[0], dy = tg[1] - out[1], dz = tg[2] - out[2];
	const float ox = out[0], oy = out[1], oz = out[2];
	const float cg = cosf(rot_gate * 0.5f);
	const float q[4] = { out[3], out[4], out[5], out[6] };
	int snap = fabsf(dx) > pos_gate || fabsf(dy) > pos_gate || fabsf(dz) > pos_gate;

	if (!snap) {
		const float n = (q[0] * q[0] + q[2] * q[2]) + (q[1] * q[1] + q[3] * q[3]);
		float inv[4] = { 0.0f, 0.0f, 0.0f, 0.0f }, e[4];
		if (n > 0.0f) {
			inv[0] = -q[0] / n;
			inv[1] = -q[1] / n;
			inv[2] = -q[2] / n;
			inv[3] = q[3] / n;
		}
		psvr2_ins_quat_mul(e, tg + 3, inv);
		snap = cg > fabsf(e[3]);
	}
	if (snap) {
		memcpy(out, tg, 7 * sizeof(float));
	} else {
		float a = fminf(dt * 4.0f, 1.0f), b = 1.0f - a;
		out[0] = dx * a + ox;
		out[1] = dy * a + oy;
		out[2] = a * dz + oz;
		const float d = (tg[3] * q[0] + tg[5] * q[2]) + (tg[4] * q[1] + tg[6] * q[3]);
		const float ad = fabsf(d);
		if (ad < 0.9999999f) {
			const float th = acosf(ad), st = sinf(th);
			b = sinf(th * b) / st;
			a = sinf(th * a) / st;
		}
		if (d < 0.0f)
			a = -a;
		for (int i = 0; i < 4; i++)
			out[3 + i] = b * q[i] + a * tg[3 + i];
	}
	if (vel && tvel)
		memcpy(vel, tvel, 3 * sizeof(float));
}

void psvr2_ins_imucal_mat(float *m, const float *R, const float *s, const float *o)
{
	memset(m, 0, 16 * sizeof(float));
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			m[4 * i + j] = R[3 * i + j] * s[i];
	for (int j = 0; j < 3; j++)
		m[12 + j] = ((s[1] * R[3 + j]) * o[1] + (s[2] * R[6 + j]) * o[2]) + (R[j] * s[0]) * o[0];
	m[15] = 1.0f;
}

static float rdf(const uint8_t *p, unsigned int off)
{
	float f;
	memcpy(&f, p + off, 4);
	return f;
}

bool psvr2_ins_imucal_build(struct psvr2_ins_imucal *c, const void *t12, unsigned int n12, const void *t13,
			    unsigned int n13)
{
	const uint8_t *a = t12, *b = t13;

	c->valid = false;
	if (!a || !b || n12 < 0x78 || n13 < 0x34)
		return false;

	/* Accel: unit upper-triangular misalignment times one scale k. */
	const float k = rdf(a, 0x74);
	const float Ra[9] = { 1.0f * k, 0.0f * k, 0.0f * k, rdf(a, 0x00) * k, 1.0f * k, 0.0f * k,
			      rdf(a, 0x04) * k, rdf(a, 0x08) * k, k };
	const float sa[3] = { rdf(a, 0x0c), rdf(a, 0x10), rdf(a, 0x14) };
	const float oa[3] = { -rdf(a, 0x18), -rdf(a, 0x1c), -rdf(a, 0x20) };
	psvr2_ins_imucal_mat(c->A, Ra, sa, oa);

	/* Gyro: full off-diagonal misalignment; offset from type 13. */
	const float Rg[9] = { 1.0f, rdf(a, 0x30), rdf(a, 0x34), rdf(a, 0x24), 1.0f,
			      rdf(a, 0x38), rdf(a, 0x28), rdf(a, 0x2c), 1.0f };
	const float sg[3] = { rdf(a, 0x3c), rdf(a, 0x40), rdf(a, 0x44) };
	const float og[3] = { -rdf(b, 0x28), -rdf(b, 0x2c), -rdf(b, 0x30) };
	psvr2_ins_imucal_mat(c->G, Rg, sg, og);

	c->valid = true;
	return true;
}

void psvr2_ins_imucal_apply(const struct psvr2_ins_imucal *c, const int16_t *raw, float *accel, float *gyro)
{
	const float ax = (float)raw[0], ay = (float)raw[1], az = (float)raw[2];
	const float gx = (float)raw[3], gy = (float)raw[4], gz = (float)raw[5];
	for (int j = 0; j < 3; j++) {
		accel[j] = 1.0f * c->A[12 + j] + ((ax * c->A[j] + ay * c->A[4 + j]) + az * c->A[8 + j]);
		gyro[j] = 1.0f * c->G[12 + j] + ((gx * c->G[j] + gy * c->G[4 + j]) + gz * c->G[8 + j]);
	}
}
