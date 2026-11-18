#include "sony.h"
#include "irt_difftest.h"
#include "psvr2_ins_cfg.h"

#define N 300000

/* Random 3x3 matrix, often symmetric / positive / tiny. */
static void rmat(float *m)
{
	uint32_t r = rnd();
	for (int i = 0; i < 9; i++) m[i] = (r & 1) ? rfs(1e-3f) + 1e-3f : rf();
	if (r & 2) for (int i = 0; i < 3; i++) for (int j = 0; j < i; j++) m[i * 3 + j] = m[j * 3 + i] + ((r & 4) ? 0 : rfs(2e-6f));
	if ((r & 24) == 8) m[4] = -m[4];
	if ((r & 96) == 32) for (int i = 0; i < 9; i++) m[i] *= 1e-5f;
}

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	int st = 0, bad = 0;
	const psvr2_irtracker_ins_cfg *c = psvr2_irtracker_ins_cfg_default();
	const void *sc = ((const void *(*)(void))SONY(0x180172f90))();
	st |= report("ins_cfg_default 0x180172f90", memcmp(sc, c, sizeof *c) != 0, 1);

	{ /* the symmetry checker takes a small closure: A at +0x18, B at +0x20, tol at +0x3c */
		bad = 0;
		for (int i = 0; i < N; i++) {
			float a[9], b[9], tol = (i & 3) ? rfs(1e-3f) + 1e-3f : rf();
			rmat(a);
			for (int k = 0; k < 9; k++) b[k] = a[(k % 3) * 3 + k / 3] + ((rnd() & 7) ? 0 : rfs(2e-3f));
			if ((i & 15) == 0) rmat(b);
			uint8_t clo[0x40] = { 0 };
			const float *pa = a, *pb = b;
			memcpy(clo + 0x18, &pa, 8); memcpy(clo + 0x20, &pb, 8); memcpy(clo + 0x3c, &tol, 4);
			int r1 = ((uint64_t (*)(void *))SONY(0x1801729d0))(clo) & 0xff;
			if (r1 != psvr2_irtracker_mat3_near_transpose(a, b, tol)) bad++;
		}
		st |= report("mat3_near_transpose 0x1801729d0", bad, N);
	}
	bad = 0;
	for (int i = 0; i < N; i++) {
		float m[9]; rmat(m);
		int r1 = ((uint64_t (*)(void *, const float *))SONY(0x180172b90))(NULL, m) & 0xff;
		if (r1 != psvr2_irtracker_cov3_valid(m)) bad++;
	}
	st |= report("cov3_valid 0x180172b90", bad, N);
	bad = 0;
	for (int i = 0; i < N; i++) {
		float m[9], o1[9], o2[9], fl = (i & 3) ? rfs(1e-3f) : rf();
		rmat(m);
		((float *(*)(float *, const float *, float))SONY(0x180172aa0))(o1, m, fl);
		psvr2_irtracker_cov3_floor(o2, m, fl);
		if (!feqn(o1, o2, 9)) bad++;
	}
	st |= report("cov3_floor 0x180172aa0", bad, N);
	{
		static const uint64_t va[3] = { 0x180172da0, 0x180172d10, 0x180172e30 };
		static const uint64_t vs[3] = { 0x1801732b0, 0x180173270, 0x180173370 };
		for (int w = 0; w < 3; w++) {
			int b1 = 0, b2 = 0;
			for (int i = 0; i < N; i++) {
				float m[9], o1[9], o2[9], a = rfs(20.0f), b = (i & 3) ? a + rfs(12.0f) : rf();
				rmat(m);
				((float *(*)(const void *, float *, const float *, float, float))SONY(va[w]))(c, o1, m, a, b);
				psvr2_irtracker_cov3_for_gap(c, o2, m, a, b, w);
				if (!feqn(o1, o2, 9)) b1++;
				int r1 = ((uint64_t (*)(const void *, const float *))SONY(vs[w]))(c, m) & 0xff;
				if (r1 != psvr2_irtracker_cov3_small(c, m, w)) b2++;
			}
			char name[64];
			snprintf(name, sizeof name, "cov3_for_gap(%d) 0x%llx", w, (unsigned long long)va[w]);
			st |= report(name, b1, N);
			snprintf(name, sizeof name, "cov3_small(%d) 0x%llx", w, (unsigned long long)vs[w]);
			st |= report(name, b2, N);
		}
	}
	bad = 0;
	for (int i = 0; i < N; i++) {
		float a[3], b[3];
		for (int k = 0; k < 3; k++) { a[k] = (i & 1) ? rfs(0.1f) : rf(); b[k] = rfs(0.01f); }
		int r1 = ((uint64_t (*)(const void *, const float *, const float *))SONY(0x1801732f0))(c, a, b) & 0xff;
		if (r1 != psvr2_irtracker_vec_pair_small(c, a, b)) bad++;
	}
	st |= report("vec_pair_small 0x1801732f0", bad, N);
	return st;
}
