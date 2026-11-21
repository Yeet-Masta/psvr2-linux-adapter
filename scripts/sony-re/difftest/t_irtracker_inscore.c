#include "sony.h"
#include "irt_difftest.h"
#include "psvr2_ins_core.h"
#include <math.h>

#define N 200000

/* Moderate gyro/accel-like sample stream. */
static void rsamples(struct psvr2_ins_sample *s, unsigned int n, int wild)
{
	for (unsigned int i = 0; i < n; i++) {
		for (int k = 0; k < 3; k++) {
			s[i].accel[k] = wild ? rf() : rfs(20.0f);
			s[i].gyro[k] = wild ? rf() : rfs((i & 7) ? 3.0f : 0.02f);
		}
		s[i].t_us = (int32_t)(i * 500);
	}
}

static void rquat(float *q)
{
	for (int k = 0; k < 4; k++)
		q[k] = rfs(1.0f);
	float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (n > 1e-3f)
		for (int k = 0; k < 4; k++)
			q[k] /= n;
}

static void rstate(struct psvr2_ins_state *s, int wild)
{
	memset(s, 0, sizeof(*s));
	for (int k = 0; k < 3; k++) {
		s->pose[k] = wild ? rf() : rfs(3.0f);
		s->vel[k] = wild ? rf() : rfs(2.0f);
		s->gyro_bias[k] = rfs(0.05f);
		s->accel_bias[k] = rfs(0.5f);
		s->prev_accel[k] = rfs(20.0f);
		s->prev_gyro[k] = rfs(3.0f);
		s->delta[k] = rfs(1.0f);
	}
	s->gravity[1] = -9.8f;
	rquat(s->pose + 3);
	rquat(s->delta + 3);
	if (wild)
		rfill(s->pose + 3, 4);
}

/* Sony's filter is a struct of pointers (two levels for most fields). */
struct sony_state {
	void *slot[9];
	float *p1[8];
};

static void to_sony(struct sony_state *ss, struct psvr2_ins_state *s)
{
	ss->p1[1] = s->vel;
	ss->p1[2] = s->gyro_bias;
	ss->p1[3] = s->accel_bias;
	ss->p1[5] = s->gravity;
	ss->p1[6] = s->prev_accel;
	ss->p1[7] = s->prev_gyro;
	ss->slot[0] = s->pose;
	for (int i = 1; i < 8; i++)
		ss->slot[i] = &ss->p1[i];
	ss->slot[8] = s->delta;
}

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	int st = 0;

	{
		typedef void *(*f)(float *, const float *, const float *);
		int bad = 0;
		for (int i = 0; i < N; i++) {
			float a[4], b[4], r1[4], r2[4];
			rfill(a, 4);
			rfill(b, 4);
			((f)SONY(0x1801805a0))(r1, a, b);
			psvr2_ins_quat_mul(r2, a, b);
			if (!feqn(r1, r2, 4))
				bad++;
		}
		st |= report("quat_mul 0x1801805a0", bad, N);
	}
	{
		typedef void *(*f)(float *, float, const float *, const float *);
		int bad = 0, worst = 0;
		for (int i = 0; i < N; i++) {
			float a[3], b[3], r1[4], r2[4], dt = (i & 3) ? 0.0005f : rfs(0.1f);
			for (int k = 0; k < 3; k++) {
				a[k] = (i & 1) ? rfs(0.2f) : rfs(20.0f);
				b[k] = (i & 1) ? rfs(0.2f) : rfs(20.0f);
			}
			if ((i & 15) == 15) { rfill(a, 3); dt = rf(); }
			((f)SONY(0x18017f2c0))(r1, dt, a, b);
			psvr2_ins_rotinc(r2, dt, a, b);
			if (!feqn(r1, r2, 4)) { bad++; int u = fulpn(r1, r2, 4); if (u > worst) worst = u; }
		}
		printf("  (worst %d ulp)\n", worst);
		st |= report("rotinc 0x18017f2c0", bad, N);
	}
	{
		typedef void *(*f)(void *, void *, float, int, const void *);
		int bad = 0, worst = 0;
		const int M = N / 20;
		for (int i = 0; i < M; i++) {
			struct psvr2_ins_state s, o2;
			struct psvr2_ins_sample smp[64];
			struct sony_state ss;
			float o1[23];
			unsigned int n = (i % 9 == 0) ? 0 : 1 + rnd() % 64;
			rstate(&s, (i & 31) == 31);
			rsamples(smp, n, (i & 63) == 63);
			to_sony(&ss, &s);
			((f)SONY(0x18017f8e0))(&ss, o1, 0.0005f, (int)n, smp);
			psvr2_ins_strapdown(&o2, &s, 0.0005f, n, smp);
			float mine[23];
			memcpy(mine, o2.delta, 28);
			memcpy(mine + 7, o2.pose, 28);
			memcpy(mine + 14, o2.vel, 12);
			memcpy(mine + 17, o2.prev_accel, 12);
			memcpy(mine + 20, o2.prev_gyro, 12);
			if (!feqn(o1, mine, 23)) {
				int u = fulpn(o1, mine, 23);
				if (u > worst) worst = u;
				if (bad++ < 3)
					printf("  strapdown i=%d n=%u p %g/%g q %g/%g v %g/%g\n", i, n, o1[7], mine[7],
					       o1[10], mine[10], o1[14], mine[14]);
			}
		}
		printf("  (worst %d ulp)\n", worst);
		st |= report("strapdown 0x18017f8e0", bad, M);
	}
	{
		typedef char (*f)(void *, float, float *);
		int bad = 0, worst = 0;
		for (int i = 0; i < N; i++) {
			struct psvr2_ins_state s;
			struct sony_state ss;
			float o1[7], o2[7], dt = (i & 7) ? rfs(0.05f) : rf();
			rstate(&s, (i & 31) == 31);
			to_sony(&ss, &s);
			char r1 = ((f)SONY(0x18017f500))(&ss, dt, o1);
			bool r2 = psvr2_ins_predict(o2, &s, dt);
			if (!feqn(o1, o2, 7) || (r1 != 0) != r2) {
				int u = fulpn(o1, o2, 7);
				if (u > worst) worst = u;
				bad++;
			}
		}
		printf("  (worst %d ulp)\n", worst);
		st |= report("predict 0x18017f500", bad, N);
	}
	{
		typedef char (*f)(const float *);
		int bad = 0;
		for (int i = 0; i < N; i++) {
			float p[7];
			rfill(p, 7);
			if (i & 1) rquat(p + 3);
			if ((((f)SONY(0x18017bac0))(p) != 0) != psvr2_ins_pose_ok(p))
				bad++;
		}
		st |= report("pose_ok 0x18017bac0", bad, N);
	}
	{
		typedef void (*f)(void *, float *, float *, const float *, const float *, float);
		int bad = 0, worst = 0;
		for (int i = 0; i < N; i++) {
			uint8_t bl[256] = { 0 };
			float pg = 0.5f, rg = 0.1f, dt = (i & 3) ? rfs(0.01f) + 0.01f : rf();
			float out1[7], out2[7], tg[7], v1[3], v2[3], tv[3];
			if (i & 4) { pg = rfs(1.0f); rg = rfs(1.0f); }
			memcpy(bl + 160, &pg, 4);
			memcpy(bl + 172, &rg, 4);
			for (int k = 0; k < 3; k++) {
				out1[k] = rfs(1.0f);
				tg[k] = out1[k] + rfs((i & 8) ? 1.0f : 0.05f);
				v1[k] = rfs(1.0f);
				tv[k] = rfs(1.0f);
			}
			rquat(out1 + 3);
			memcpy(tg + 3, out1 + 3, 16);
			for (int k = 3; k < 7; k++) tg[k] += rfs((i & 16) ? 0.5f : 0.02f);
			if ((i & 63) == 63) rfill(tg, 7);
			memcpy(out2, out1, sizeof out1);
			memcpy(v2, v1, sizeof v1);
			((f)SONY(0x180181a50))(bl, out1, v1, tg, tv, dt);
			psvr2_ins_blend1(out2, tg, dt, pg, rg, v2, tv);
			if (!feqn(out1, out2, 7) || !feqn(v1, v2, 3)) {
				int u = fulpn(out1, out2, 7);
				if (u > worst) worst = u;
				bad++;
			}
		}
		printf("  (worst %d ulp)\n", worst);
		st |= report("blend1 0x180181a50", bad, N);
	}
	{
		typedef void *(*f)(float *, float **, const float *);
		int bad = 0;
		for (int i = 0; i < N; i++) {
			float R[9], s[3], o[3], m1[16], m2[16];
			for (int k = 0; k < 9; k++) R[k] = (i & 1) ? rf() : rfs(1.0f);
			for (int k = 0; k < 3; k++) { s[k] = rfs(0.01f); o[k] = rfs(500.0f); }
			float *a2[3] = { R, NULL, s };
			((f)SONY(0x1801733f0))(m1, a2, o);
			psvr2_ins_imucal_mat(m2, R, s, o);
			if (!feqn(m1, m2, 16))
				bad++;
		}
		st |= report("imucal_mat 0x1801733f0", bad, N);
	}
	return st;
}
