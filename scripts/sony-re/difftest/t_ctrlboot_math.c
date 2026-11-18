#include "sony.h"
#include "psvr2_cb_math.h"
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
static void rfill(void *p, size_t n) { float *f = p; for (size_t i = 0; i < n / 4; i++) f[i] = rf(); }
/* Bitwise compare, but any NaN equals any NaN (payload/sign of NaN not compared). */
static int feq(const void *a, const void *b, size_t n)
{
	const float *x = a, *y = b;
	for (size_t i = 0; i < n / 4; i++)
		if (memcmp(&x[i], &y[i], 4) && !(x[i] != x[i] && y[i] != y[i]))
			return 0;
	return 1;
}
static long bad[32];
#define CHECK(id, c) do { if (!(c)) { if (bad[id]++ < 3) printf("mismatch %s it=%d\n", #id, it); } } while (0)
enum { RIGID, ROT, DEPTH, MULSUB, LEN, NORM, CCW, STAT, RANSAC, EIG, FALL, SPEED, FACE, PROJ, NCHK };

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	static uint8_t big[0x11000];
	for (int it = 0; it < 200000; it++) {
		psvr2_m44 m, a, b; psvr2_v4 v, va, vb;
		rfill(&m, sizeof m); rfill(&v, sizeof v);
		/* rigid get: table at +0x10a40 */
		{
			int idx = rnd() % 4, inv = rnd() % 2;
			memcpy(big + 0x10a40 + idx * 0x40, &m, 64);
			((void (*)(void *, void *, int, int))SONY(0x1800cbd70))(big, &a, idx, inv);
			psvr2_cb_rigid_get(&b, &m, inv);
			CHECK(RIGID, feq(&a, &b, 64));
			/* PoseCost / PoseCostMultiCam: inverse of the pose at +0x40 / +0x50. */
			static uint8_t pc[0xa0];
			memcpy(pc + 0x40, &m, 64);
			((void *(*)(void *, void *))SONY(0x1800f72d0))(pc, &a);
			psvr2_cb_rigid_get(&b, &m, 1);
			CHECK(RIGID, feq(&a, &b, 64));
			memcpy(pc + 0x50, &m, 64);
			((void *(*)(void *, void *))SONY(0x1800f7580))(pc, &a);
			CHECK(RIGID, feq(&a, &b, 64));
		}
		((void (*)(void *, void *, void *))SONY(0x1800c05f0))(&m, &va, &v);
		psvr2_cb_rotate(&vb, &m, &v);
		CHECK(ROT, feq(&va, &vb, 16));
		{
			uint8_t pose[0x500] = { 0 }; memcpy(pose, &m, 64); pose[0x4ac] = rnd() % 2;
			void *pv = &v; uint8_t obj[0x1900]; memcpy(obj + 0x18f8, &pv, 8);
			float x = ((float (*)(void *, void *))SONY(0x1800e0a60))(obj, pose);
			float y = pose[0x4ac] ? psvr2_cb_depth(&m, &v) : -1.0f;
			CHECK(DEPTH, feq(&x, &y, 4));
		}
		{
			float p[2], q[2], r[2], o1[2], o2[2];
			rfill(p, 8); rfill(q, 8); rfill(r, 8);
			((void *(*)(void *, void *, void *, void *))SONY(0x1800c0220))(o1, p, q, r);
			psvr2_cb_v2_mul_sub(o2, p, q, r);
			CHECK(MULSUB, feq(o1, o2, 8));
			float l1 = ((float (*)(void *))SONY(0x1800c04c0))(p), l2 = psvr2_cb_v2_len(p);
			CHECK(LEN, feq(&l1, &l2, 4));
			((void (*)(void *, void *))SONY(0x1800c0520))(o1, p);
			psvr2_cb_v2_normalize(o2, p);
			CHECK(NORM, feq(o1, o2, 8));
			int c1 = ((uint8_t (*)(void *, void *))SONY(0x1800c5290))(p, q), c2 = psvr2_cb_v2_ccw(p, q);
			CHECK(CCW, c1 == c2);
		}
		{
			psvr2_cb_stat s1, s2; rfill(&s1, sizeof s1);
			s1.count = rnd() % 4 == 0 ? 0 : (int32_t)(rnd() % 1000) - 10; s2 = s1;
			((void (*)(void *))SONY(0x1800c7200))(&s1);
			psvr2_cb_stat_finalize(&s2);
			CHECK(STAT, feq(&s1, &s2, sizeof s1));
		}
		{
			psvr2_cb_ransac r1 = { 0 }, r2;
			r1.max_iters = rnd() % 5000; r1.adaptive = rnd() % 4 != 0;
			r1.confidence = (float)(rnd() % 10001) / 10000.f; r1.inlier_ratio = (float)(rnd() % 10001) / 10000.f;
			if (rnd() % 8 == 0) r1.confidence = rf();
			if (rnd() % 8 == 0) r1.inlier_ratio = rf();
			r2 = r1;
			((void (*)(void *))SONY(0x1800c8b50))(&r1);
			psvr2_cb_ransac_update(&r2);
			CHECK(RANSAC, r1.max_iters == r2.max_iters);
		}
		{
			float e[4], o1 = 7, o2 = 7; rfill(e, 16);
			int k1 = ((uint8_t (*)(float, float, float, float, float *))SONY(0x1800cc4b0))(e[0], e[1], e[2], e[3], &o1);
			int k2 = psvr2_cb_eig2_slope(e[0], e[1], e[2], e[3], &o2);
			CHECK(EIG, k1 == k2 && feq(&o1, &o2, 4));
			float f1 = ((float (*)(float, float, float))SONY(0x1800d0750))(e[0], e[1], e[2]);
			float f2 = psvr2_cb_falloff(e[0], e[1], e[2]);
			CHECK(FALL, feq(&f1, &f2, 4));
		}
		{
			static uint8_t obj[0x4ca0]; uint8_t x[0x40]; rfill(x, sizeof x);
			if (rnd() & 1) { float *f = (float *)(x + 0x30); for (int i = 0; i < 3; i++) f[i] = (float)((int)(rnd() % 400) - 200); }
			((void (*)(void *, void *))SONY(0x1800db5f0))(obj, x);
			CHECK(SPEED, *(int32_t *)(obj + 0x4c94) == psvr2_cb_speed_class((float *)(x + 0x30)));
		}
		{
			static uint8_t obj[0x3600]; psvr2_v4 ax, sc;
			rfill(&ax, 16); rfill(&sc, 16);
			if (rnd() & 1) { sc = (psvr2_v4){ { 1, 1, 1, 1 } }; }
			float dist = rf(); int en = rnd() % 4 != 0;
			obj[0x3360] = en; memcpy(obj + 0x35e0, &dist, 4); memcpy(obj + 0x3310, &ax, 16); memcpy(obj + 0x3300, &sc, 16);
			int f1 = ((uint8_t (*)(void *, void *))SONY(0x1800e1c90))(obj, &m);
			CHECK(FACE, f1 == psvr2_cb_facing(en, dist, &ax, &sc, &m));
		}
		{
			uint8_t o1[0x20], o2[0x20]; float wv[8]; rfill(wv, 32);
			float s = rf();
			((void (*)(void *, float, void *, void *))SONY(0x1800e80c0))(o1, s, &m, wv);
			psvr2_cb_proj2((float *)(o2 + 0x10), s, &m, wv + 4);
			CHECK(PROJ, feq(o1 + 0x10, o2 + 0x10, 8));
		}
	}
	long t = 0;
	for (int i = 0; i < NCHK; i++) t += bad[i];
	printf("ctrlboot math: %ld mismatches (200000 iterations x %d functions)\n", t, NCHK);
	return t != 0;
}
