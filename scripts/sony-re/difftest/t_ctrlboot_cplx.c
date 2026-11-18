#include "sony.h"
#include "psvr2_cb_cplx.h"
#include <math.h>

static float rf(void)
{
	union { uint32_t u; float f; } v;
	switch (rnd() % 8) {
	case 0: v.u = rnd(); return v.f;
	case 1: return 0.0f;
	case 2: return -0.0f;
	case 3: return (float)((int32_t)rnd() % 1000) / 7.0f;
	case 4: return (float)((int32_t)rnd()) * 1e-9f;
	default: return (float)(rnd() % 200000) / 100000.0f * (rnd() & 1 ? -1.f : 1.f);
	}
}
static void rfill(float *f, int n) { for (int i = 0; i < n; i++) f[i] = rf(); }
static int feq(const float *x, const float *y, int n)
{
	for (int i = 0; i < n; i++)
		if (memcmp(&x[i], &y[i], 4) && !(x[i] != x[i] && y[i] != y[i]))
			return 0;
	return 1;
}
static long bad[16];
static uint32_t maxulp[16];
static int32_t ord(float f) { int32_t i; memcpy(&i, &f, 4); return i < 0 ? (int32_t)0x80000000 - i : i; }
static int fulp(int id, const float *x, const float *y, int n)
{
	for (int i = 0; i < n; i++) {
		if (x[i] != x[i] && y[i] != y[i]) continue;
		if (x[i] != x[i] || y[i] != y[i]) return 0;
		int64_t d = (int64_t)ord(x[i]) - ord(y[i]); if (d < 0) d = -d;
		if (d > maxulp[id]) maxulp[id] = (uint32_t)d;
		if (d > 4) return 0;
	}
	return 1;
}
static const char *nm[] = { "div", "cbrt", "sqrt", "polar", "inv", "m2v", "nfast", "outer", "outer2", "add12", "scale12", "flip" };
#define CHECK(id, c) do { if (!(c) && bad[id]++ < 3) printf("mismatch %s it=%d\n", nm[id], it); } while (0)

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	for (int it = 0; it < 300000; it++) {
		float a[16], b[16], o1[16], o2[16];
		uint64_t za;
		rfill(a, 16); rfill(b, 16);
		memcpy(&za, a, 8);
		((void *(*)(void *, void *, void *))SONY(0x1800f4100))(o1, a, b); psvr2_cb_c_div(o2, a, b); CHECK(0, feq(o1, o2, 2));
		((void *(*)(void *, uint64_t))SONY(0x1800f41e0))(o1, za); psvr2_cb_c_cbrt(o2, a); CHECK(1, fulp(1, o1, o2, 2));
		((void *(*)(void *, uint64_t))SONY(0x1800f42a0))(o1, za); psvr2_cb_c_sqrt(o2, a); CHECK(2, feq(o1, o2, 2));
		((void *(*)(void *, void *, void *))SONY(0x1800f5d30))(o1, &a[0], &b[0]); psvr2_cb_c_polar(o2, a[0], b[0]); CHECK(3, fulp(3, o1, o2, 2));
		((void *(*)(void *, void *))SONY(0x1800b1b20))(o1, a); psvr2_cb_m2_inverse(o2, a); CHECK(4, feq(o1, o2, 4));
		memset(o1, 0, 8); ((void (*)(void *, void *, void *))SONY(0x1800b1aa0))(a, o1, b); psvr2_cb_m2_mul_v2(o2, a, b); CHECK(5, feq(o1, o2, 2));
		((void (*)(void *, void *))SONY(0x1800f3220))(o1, a); psvr2_cb_v2_normalize_fast(o2, a); CHECK(6, feq(o1, o2, 2));
		((void *(*)(void *, void *, void *))SONY(0x1800b2ba0))(o1, a, b); psvr2_cb_v2_outer(o2, a, b); CHECK(7, feq(o1, o2, 4));
		((void (*)(void *, void *, void *))SONY(0x1800b2c20))(o1, a, b); psvr2_cb_v4_outer2(o2, a, b); CHECK(8, feq(o1, o2, 12));
		memcpy(o1, a, 48); memcpy(o2, a, 48);
		((void *(*)(void *, void *))SONY(0x1800b1e90))(o1, b); psvr2_cb_add12(o2, b); CHECK(9, feq(o1, o2, 12));
		((void *(*)(void *, float))SONY(0x1800b1f30))(o1, b[3]); psvr2_cb_scale12(o2, b[3]); CHECK(10, feq(o1, o2, 12));
		((void *(*)(void *, void *, void *))SONY(0x1800b0130))(o1, a, b); psvr2_cb_flip_pose(o2, a, b); CHECK(11, feq(o1, o2, 16));
	}
	long t = 0;
	for (int i = 0; i < 12; i++) t += bad[i];
	printf("libm-dependent max ulp: cbrt %u, polar %u\n", maxulp[1], maxulp[3]);
	printf("ctrlboot complex/2x2: %ld mismatches (300000 iterations x 12 functions)\n", t);
	return t != 0;
}
