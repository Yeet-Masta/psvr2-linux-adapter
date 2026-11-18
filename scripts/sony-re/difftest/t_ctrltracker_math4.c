#include "sony.h"
#include "ct_difftest.h"
#include "psvr2_ct_math.h"

typedef float (*f_yaw)(const float *, const float *);
typedef float *(*f_exp)(float *, const float *);
typedef double *(*f_int)(double *, const double *, const float *, float);
typedef void (*f_set)(void *, const float *);

static float unitish(void)
{
	return ((int32_t)rnd()) * (1.0f / 2147483648.0f);
}

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	const int n = 200000;
	int fail = 0, bad;

	bad = 0;
	for (int i = 0; i < n; i++) {
		float p[4], q[4];
		for (int k = 0; k < 4; k++) {
			p[k] = (i & 3) ? unitish() : rf();
			q[k] = (i & 3) ? unitish() : rf();
		}
		if (i & 1) {
			psvr2_ct_quat_normalize(p, p);
			psvr2_ct_quat_normalize(q, q);
		}
		bad += !feq(((f_yaw)SONY(0x18006b790))(p, q), psvr2_ct_quat_rel_yaw(p, q));
	}
	fail |= report("quat_rel_yaw 0x18006b790", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		float v[3], x[4], y[4];
		const float sc = (i % 3 == 0) ? 1e-4f : (i % 3 == 1 ? 1.0f : 10.0f);
		for (int k = 0; k < 3; k++) v[k] = (i & 8) ? rf() : unitish() * sc;
		((f_exp)SONY(0x18006bb50))(x, v);
		psvr2_ct_quat_exp(y, v);
		bad += !feqn(x, y, 4);
		double q[4], X[4], Y[4];
		for (int k = 0; k < 4; k++) q[k] = (double)unitish();
		const float dt = (i & 16) ? rf() : 0.011f;
		((f_int)SONY(0x18006bcd0))(X, q, v, dt);
		psvr2_ct_quatd_integrate(Y, q, v, dt);
		for (int k = 0; k < 4; k++) bad += !deq(X[k], Y[k]);
	}
	fail |= report("quat_exp/integrate 0x18006bb50/bcd0", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		float a[0x40] __attribute__((aligned(16))), b[0x40] __attribute__((aligned(16)));
		float p[4];
		for (int k = 0; k < 0x40; k++) a[k] = b[k] = rf();
		for (int k = 0; k < 4; k++) p[k] = rf();
		((f_set)SONY(0x18006bf80))(a, p);
		psvr2_ct_set_ref_and_diff(b + 0x30, b + 0x38, b, p);
		bad += !feqn(a, b, 0x3b) || !feqn(a + 0x3c, b + 0x3c, 4);
		for (int k = 0; k < 0x40; k++) a[k] = b[k] = rf();
		((f_set)SONY(0x18006bfd0))(a, p);
		psvr2_ct_set_ref_and_diff(b + 0x34, b + 0x3c, b + 4, p);
		bad += !feqn(a, b, 0x3f);
	}
	fail |= report("set_ref_and_diff 0x18006bf80/bfd0", bad, n);
	return fail;
}
