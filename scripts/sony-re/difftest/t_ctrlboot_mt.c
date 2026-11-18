#include "sony.h"
#include "psvr2_mt.h"

struct sony_mt { void *vt; uint32_t state[624]; uint32_t index; uint32_t pad; };

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	void (*seed)(void *, uint32_t) = SONY(0x18009f3e0);
	void *(*block)(void *, uint32_t *, int) = SONY(0x18009f530);
	uint32_t (*u32)(void *) = SONY(0x18009f630);
	float (*f01)(void *) = SONY(0x1800ac850);
	double (*d01)(void *) = SONY(0x1800ac8a0);
	float (*uni)(void *, float, float) = SONY(0x1800abe10);
	static struct sony_mt s;
	static psvr2_mt o;
	long bad = 0, calls = 0;
	for (int t = 0; t < 200; t++) {
		const uint32_t sd = t < 3 ? (uint32_t[]){ 0, 0x14, 5489 }[t] : rnd();
		seed(&s, sd);
		psvr2_mt_seed(&o, sd);
		if (memcmp(s.state, o.state, sizeof o.state) || s.index != o.index)
			bad++;
		for (int i = 0; i < 5000; i++, calls++) {
			uint32_t a[8] __attribute__((aligned(32))), b[8];
			float fa, fb;
			double da, db;
			switch (rnd() % 6) {
			case 0: {
				int step = rnd() % 9;
				block(&s, a, step);
				psvr2_mt_block(&o, b, step);
				if (memcmp(a, b, sizeof a)) bad++;
				break;
			}
			case 1: if (u32(&s) != psvr2_mt_u32(&o)) bad++; break;
			case 2: fa = f01(&s); fb = psvr2_mt_float01(&o); if (memcmp(&fa, &fb, 4)) bad++; break;
			case 3: da = d01(&s); db = psvr2_mt_double01(&o); if (memcmp(&da, &db, 8)) bad++; break;
			default: {
				union { uint32_t u; float f; } lo, hi;
				lo.u = rnd(); hi.u = rnd();
				if (rnd() & 1) { lo.f = (float)(int)(rnd() % 2000) / 3.f - 300.f; hi.f = lo.f + (float)(rnd() % 1000) / 7.f; }
				fa = uni(&s, lo.f, hi.f); fb = psvr2_mt_uniform(&o, lo.f, hi.f);
				if (memcmp(&fa, &fb, 4) && !(fa != fa && fb != fb)) bad++;
			}
			}
			if (s.index != o.index || memcmp(s.state, o.state, sizeof o.state)) { bad++; break; }
		}
	}
	printf("mersenne twister: %ld mismatches over %ld calls\n", bad, calls);
	return bad != 0;
}
