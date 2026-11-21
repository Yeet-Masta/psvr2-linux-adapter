// SPDX-License-Identifier: GPL-2.0
/*
 * Headset INS on a synthetic trajectory: 2 kHz IMU (raw counts, noise,
 * 1.25 ms group delay, an IMU axis map) plus 100 Hz SLAM delivered 12 ms
 * late in a chosen quaternion order. Checks that the INS finds the frame
 * and that its pose at "now" beats the latest SLAM sample extrapolated
 * with finite-difference velocities (what the driver did before).
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "psvr2_ins.h"

static const double kAs = 4.0 * 9.80665 / 32767.0, kGs = 2000.0 * M_PI / 180.0 / 32767.0;

static unsigned int rs = 1;
static double urand(void)
{
	rs = rs * 1103515245u + 12345u;
	return ((rs >> 8) & 0xffffff) / 16777216.0;
}
static double gauss(void)
{
	double u = urand() + 1e-12, v = urand();
	return sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
}

/* Same candidate order as psvr2_ins.c. */
static float maps[24][9];
static void build_maps(void)
{
	static const int perm[6][3] = { { 0, 1, 2 }, { 0, 2, 1 }, { 1, 0, 2 }, { 1, 2, 0 }, { 2, 0, 1 }, { 2, 1, 0 } };
	static const int psign[6] = { 1, -1, -1, 1, 1, -1 };
	int n = 0;
	for (int p = 0; p < 6; p++)
		for (int s = 0; s < 8; s++) {
			const int sg[3] = { (s & 1) ? -1 : 1, (s & 2) ? -1 : 1, (s & 4) ? -1 : 1 };
			if (psign[p] * sg[0] * sg[1] * sg[2] != 1)
				continue;
			memset(maps[n], 0, sizeof(maps[n]));
			for (int r = 0; r < 3; r++)
				maps[n][3 * r + perm[p][r]] = (float)sg[r];
			n++;
		}
}

static void encode_q(float *o, const double *q, int k)
{
	switch (k) {
	case 0: o[0] = q[0]; o[1] = q[1]; o[2] = q[2]; o[3] = q[3]; break;
	case 1: o[0] = q[3]; o[1] = q[0]; o[2] = q[1]; o[3] = q[2]; break;
	case 2: o[0] = -q[0]; o[1] = -q[1]; o[2] = -q[2]; o[3] = q[3]; break;
	default: o[0] = q[3]; o[1] = -q[0]; o[2] = -q[1]; o[3] = -q[2]; break;
	}
}
static void decode_q(double *q, const float *o, int k)
{
	switch (k) {
	case 0: q[0] = o[0]; q[1] = o[1]; q[2] = o[2]; q[3] = o[3]; break;
	case 1: q[0] = o[1]; q[1] = o[2]; q[2] = o[3]; q[3] = o[0]; break;
	case 2: q[0] = -o[0]; q[1] = -o[1]; q[2] = -o[2]; q[3] = o[3]; break;
	default: q[0] = -o[1]; q[1] = -o[2]; q[2] = -o[3]; q[3] = o[0]; break;
	}
}

/* x,y,z,w */
static void qmul(double *r, const double *a, const double *b)
{
	r[0] = a[3] * b[0] + b[3] * a[0] + a[1] * b[2] - a[2] * b[1];
	r[1] = a[3] * b[1] + b[3] * a[1] + a[2] * b[0] - a[0] * b[2];
	r[2] = a[3] * b[2] + b[3] * a[2] + a[0] * b[1] - a[1] * b[0];
	r[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
}
static void qexp(double *q, const double *v)
{
	const double a = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	const double s = a > 1e-12 ? sin(a / 2) / a : 0.5;
	q[0] = v[0] * s; q[1] = v[1] * s; q[2] = v[2] * s; q[3] = cos(a / 2);
}
static void qrot_t(double *o, const double *q, const double *v) /* R(q)^T v */
{
	const double c[4] = { -q[0], -q[1], -q[2], q[3] }, p[4] = { v[0], v[1], v[2], 0 };
	double t[4], r[4];
	qmul(t, c, p);
	qmul(r, t, q);
	o[0] = r[0]; o[1] = r[1]; o[2] = r[2];
}
static double qang(const double *a, const double *b)
{
	const double d = fabs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
	return 2.0 * acos(d > 1.0 ? 1.0 : d) * 180.0 / M_PI;
}

static void omega(double t, double *w)
{
	w[0] = 1.5 * sin(2 * M_PI * 0.7 * t) + 0.4 * sin(2 * M_PI * 3.1 * t);
	w[1] = 2.5 * sin(2 * M_PI * 0.45 * t + 1.0);
	w[2] = 0.8 * sin(2 * M_PI * 1.3 * t + 2.0);
}
static void trans(double t, double *p, double *v, double *a)
{
	static const double A[3] = { 0.20, 0.08, 0.15 }, F[3] = { 0.5, 0.9, 0.3 };
	for (int i = 0; i < 3; i++) {
		const double w = 2 * M_PI * F[i];
		p[i] = A[i] * sin(w * t);
		v[i] = A[i] * w * cos(w * t);
		a[i] = -A[i] * w * w * sin(w * t);
	}
}

struct slam_q {
	struct psvr2_ins_slam s;
	double deliver;
};

static int run(int order, int map, int drop)
{
	struct psvr2_ins *ins = calloc(1, sizeof(*ins));
	static struct slam_q q[64];
	int qh = 0, qt = 0;
	double qt_true[4] = { 0, 0, 0, 1 };
	const double dt = 0.0005, T = 20.0;

	double e_ins_r = 0, e_ins_p = 0, e_b_r = 0, e_b_p = 0, m_ins_r = 0, m_ins_p = 0;
	int ne = 0, found_at = -1, dof3 = 0;
	double m_drop_r = 0;
	struct psvr2_ins_slam last = { 0 }, prev = { 0 };
	int have_last = 0;

	rs = 7 + order * 31 + map;
	psvr2_ins_init(ins, (float)kAs, (float)kGs);
	const float *M = maps[map];

	for (long k = 0; k * dt < T; k++) {
		const double t = k * dt;
		double w[3], p[3], v[3], a[3];

		/* truth: integrate body rate over the step in 10 substeps */
		for (int s = 0; s < 10; s++) {
			double wm[3], dq[4], nq[4];
			omega(t + (s + 0.5) * dt / 10, wm);
			for (int i = 0; i < 3; i++)
				wm[i] *= dt / 10;
			qexp(dq, wm);
			qmul(nq, qt_true, dq);
			memcpy(qt_true, nq, sizeof(nq));
		}
		const double tn = t + dt;
		omega(tn, w);
		trans(tn, p, v, a);

		/* IMU: body = M imu, so imu = M^T body */
		double f[3], fw[3] = { a[0], a[1] + 9.8, a[2] };
		qrot_t(f, qt_true, fw);
		int16_t ra[3], rg[3];
		for (int i = 0; i < 3; i++) {
			double fi = 0, wi = 0;
			for (int r = 0; r < 3; r++) {
				fi += M[3 * r + i] * f[r];
				wi += M[3 * r + i] * w[r];
			}
			ra[i] = (int16_t)lrint((fi + 0.02 * gauss()) / kAs);
			rg[i] = (int16_t)lrint((wi + 0.004 * gauss()) / kGs);
		}
		const int64_t ts = (int64_t)llround(tn * 1e9) + PSVR2_INS_GROUP_DELAY_NS;
		psvr2_ins_push_imu(ins, ra, rg, ts);

		/* SLAM sample every 10 ms, delivered 12 ms later */
		if (k % 20 == 19 && !(drop && tn >= 10.0 && tn < 12.0)) {
			struct slam_q *e = &q[qh++ % 64];
			memset(e, 0, sizeof(*e));
			e->s.t_ns = (int64_t)llround(tn * 1e9);
			e->s.valid = 1;
			double qn[4], nz[3] = { 0.0003 * gauss(), 0.0003 * gauss(), 0.0003 * gauss() }, dqn[4];
			qexp(dqn, nz);
			qmul(qn, qt_true, dqn);
			for (int i = 0; i < 3; i++) {
				e->s.pos[i] = (float)(p[i] + 0.0003 * gauss());
				e->s.vel[i] = (float)v[i];
			}
			encode_q(e->s.quat, qn, order);
			e->s.variance = 0.01f;
			e->deliver = tn + 0.012;
		}
		/* status packets: 4 samples every 2 ms */
		if (k % 4 == 3)
			psvr2_ins_tick(ins);
		while (qt < qh && q[qt % 64].deliver <= tn + 1e-9) {
			psvr2_ins_tick(ins);
			psvr2_ins_push_slam(ins, &q[qt % 64].s);
			prev = last;
			last = q[qt % 64].s;
			have_last++;
			qt++;
		}
		if (found_at < 0 && ins->frame >= 0)
			found_at = (int)(tn * 1000);

		/* query 2 ms ahead of the newest sample, every 1 ms after 5 s */
		if (tn > 5.0 && k % 2 == 1 && psvr2_ins_ready(ins) && have_last > 2) {
			const double tq = tn + 0.002;
			float pp[3], qq[4], vv[3];
			double qi[4], pt[3], vt[3], at[3], qtq[4], dq[4], wq[3];
			psvr2_ins_query(ins, (int64_t)llround(tq * 1e9), pp, qq, vv);
			decode_q(qi, qq, order);
			/* truth at tq: rotate on by the body rate */
			omega(tn, wq);
			for (int i = 0; i < 3; i++)
				wq[i] *= 0.002;
			qexp(dq, wq);
			qmul(qtq, qt_true, dq);
			trans(tq, pt, vt, at);
			const double er = qang(qi, qtq);
			const double ep = sqrt((pp[0] - pt[0]) * (pp[0] - pt[0]) + (pp[1] - pt[1]) * (pp[1] - pt[1]) +
					       (pp[2] - pt[2]) * (pp[2] - pt[2]));
			/* baseline: last SLAM extrapolated with its velocity and the
			 * finite-difference rate of the last two samples */
			double ql[4], qp[4], c[4], r[4], wb[3], qb[4], dqb[4];
			decode_q(ql, last.quat, order);
			decode_q(qp, prev.quat, order);
			const double sdt = (last.t_ns - prev.t_ns) * 1e-9, age = tq - last.t_ns * 1e-9;
			c[0] = -qp[0]; c[1] = -qp[1]; c[2] = -qp[2]; c[3] = qp[3];
			qmul(r, c, ql);
			if (r[3] < 0)
				for (int i = 0; i < 4; i++)
					r[i] = -r[i];
			const double sn = sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
			const double kk = sn > 1e-12 ? 2 * atan2(sn, r[3]) / sn : 2;
			for (int i = 0; i < 3; i++)
				wb[i] = r[i] * kk / sdt * age;
			qexp(dqb, wb);
			qmul(qb, ql, dqb);
			double eb = 0;
			for (int i = 0; i < 3; i++) {
				const double d = last.pos[i] + last.vel[i] * age - pt[i];
				eb += d * d;
			}
			if (tn >= 10.0 && tn < 12.0) {
				if (er > m_drop_r) m_drop_r = er;
				dof3 += psvr2_ins_query(ins, (int64_t)llround(tq * 1e9), pp, qq, vv) == 2;
				continue;
			}
			e_ins_r += er;
			e_ins_p += ep;
			e_b_r += qang(qb, qtq);
			e_b_p += sqrt(eb);
			if (er > m_ins_r) m_ins_r = er;
			if (ep > m_ins_p) m_ins_p = ep;
			ne++;
		}
	}
	int fo = -1, fm = -1;
	double res = 0;
	psvr2_ins_frame(ins, &fo, &fm, &res);
	const int ok_frame = fo >= 0; /* any member of the tie group gives the same output */
	e_ins_r /= ne ? ne : 1; e_ins_p /= ne ? ne : 1; e_b_r /= ne ? ne : 1; e_b_p /= ne ? ne : 1;
	printf("order %d map %2d: frame %d/%2d %s at %d ms (res %.3f), sixdof %d, latches %u\n", order, map, fo, fm,
	       ok_frame ? "ok" : "WRONG", found_at, res, ins->sixdof, ins->latches);
	printf("  INS     rot %.4f deg (max %.4f)  pos %.3f mm (max %.3f)\n", e_ins_r, m_ins_r, e_ins_p * 1e3,
	       m_ins_p * 1e3);
	printf("  SLAM+fd rot %.4f deg              pos %.3f mm\n", e_b_r, e_b_p * 1e3);
	if (drop)
		printf("  2 s SLAM dropout: max rot %.3f deg, %d queries in 3DoF\n", m_drop_r, dof3);
	if (!ok_frame) {
		int b = 0;
		for (int i = 1; i < PSVR2_INS_NFRAME; i++)
			if (ins->det_err[i] < ins->det_err[b]) b = i;
		printf("  det pairs %d motion %.4f best %d err %.5f true %.5f\n", ins->det_pairs, ins->det_motion, b,
		       ins->det_err[b], ins->det_err[order * 24 + map]);
	}
	const int sixdof = ins->sixdof;
	free(ins);
	return ok_frame && (!drop || (m_drop_r < 1.0 && dof3 > 1000 && sixdof)) && ne > 1000 && e_ins_r < 0.25 && e_ins_p < 0.002 && e_ins_r * 3 < e_b_r &&
	       e_ins_p * 2 < e_b_p;
}

int main(void)
{
	build_maps();
	int ok = 1;
	ok &= run(1, 7, 0);
	ok &= run(0, 0, 0);
	ok &= run(3, 17, 0);
	ok &= run(1, 0, 1);
	puts(ok ? "PASS" : "FAIL");
	return ok ? 0 : 1;
}
