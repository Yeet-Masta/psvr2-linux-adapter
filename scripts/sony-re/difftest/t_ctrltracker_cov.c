#include "sony.h"
#include "ct_difftest.h"
#include "psvr2_ct_cov.h"

typedef void (*f_reset)(float *, int);
typedef float *(*f_get)(float *, const float *, int);
typedef void (*f_set)(float *, int, const float *);
typedef void (*f_lever)(float *, float *, float *, const float *, const float *, const float *,
			const float *, const float *, const float *);

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	const int n = 50000;
	int fail = 0, bad = 0;

	for (int i = 0; i < n; i++) {
		float a[160], c[160], m[3][4], x[3][4], y[3][4];
		const int b = rnd() % 13;
		for (int k = 0; k < 160; k++) a[k] = c[k] = rf();
		for (int k = 0; k < 12; k++) m[k / 4][k % 4] = rf();
		((f_reset)SONY(0x18006c020))(a, b);
		psvr2_ct_cov_reset_block(c, b);
		bad += !feqn(a, c, 160);
		for (int k = 0; k < 160; k++) a[k] = c[k] = rf();
		((f_set)SONY(0x18006c1b0))(a, b, &m[0][0]);
		psvr2_ct_cov_set_block(c, b, m);
		bad += !feqn(a, c, 160);
		for (int k = 0; k < 12; k++) x[k / 4][k % 4] = y[k / 4][k % 4] = rf();
		((f_get)SONY(0x18006c100))(&x[0][0], a, b);
		psvr2_ct_cov_get_block(y, a, b);
		bad += !feqn(&x[0][0], &y[0][0], 12);
	}
	fail |= report("cov reset/set/get 0x18006c020/1b0/100", bad, n);

	bad = 0;
	for (int i = 0; i < 200000; i++) {
		float pos[4], vel[4], r[3][4], w[4], bm[3][4], lv[4];
		float sp[4], sv[4], sr[3][4], op[3], ov[3], orr[3][4];
		for (int k = 0; k < 4; k++) { pos[k] = rf(); vel[k] = rf(); w[k] = rf(); lv[k] = rf(); }
		for (int k = 0; k < 12; k++) { r[k / 4][k % 4] = rf(); bm[k / 4][k % 4] = rf(); }
		((f_lever)SONY(0x18006c340))(sp, sv, &sr[0][0], pos, vel, &r[0][0], w, &bm[0][0], lv);
		psvr2_ct_lever_transfer(op, ov, orr, pos, vel, r, w, bm, lv);
		bad += !feqn(sp, op, 3) || !feqn(sv, ov, 3);
		for (int c = 0; c < 3; c++)
			bad += !feqn(sr[c], orr[c], 3);
	}
	fail |= report("lever_transfer 0x18006c340", bad, 200000);

	typedef void (*f_widen)(void *, void *, const void *, const float *);
	struct dv { double *p; int64_t n; };
	bad = 0;
	for (int i = 0; i < 20000; i++) {
		static double xs[32], xo[32], ps[600], po[600];
		static float st[0x100 / 4], P[128];
		for (int k = 0; k < 64; k++) st[k] = rf();
		for (int k = 0; k < 128; k++) P[k] = rf();
		for (int k = 0; k < 600; k++) ps[k] = po[k] = (double)rf();
		for (int k = 0; k < 32; k++) xs[k] = xo[k] = (double)rf();
		const int six = i & 1;
		const int nn = six ? 6 + (int)(rnd() % 4) : 15 + (int)(rnd() % 4);
		struct dv X = { xs, 32 }, PD = { ps, nn };
		((f_widen)SONY(six ? 0x18006ce80 : 0x18006c4c0))(&X, &PD, st, P);
		if (six)
			psvr2_ct_widen6(xo, po, nn, st + 0x50 / 4, P);
		else
			psvr2_ct_widen15(xo, po, nn, st + 0x40 / 4, st + 0x50 / 4, st + 0xc0 / 4,
					 st + 0xd0 / 4, P);
		for (int k = 0; k < 32; k++) bad += !deq(xs[k], xo[k]);
		for (int k = 0; k < 600; k++) bad += !deq(ps[k], po[k]);
	}
	fail |= report("widen15/widen6 0x18006c4c0/ce80", bad, 20000);
	return fail;
}
