#include "sony.h"
#include "psvr2_robust.h"

typedef void (*slot_fn)(void *self, void *out, void *in, float s);
typedef float (*rho_fn)(void *self, float x);

struct vec { uint8_t pad[0x30]; float *data; int64_t n; };
struct obj { void *vt; float c; };

static float rfloat(void)
{
	union { uint32_t u; float f; } v;
	switch (rnd() % 6) {
	case 0: v.u = rnd(); return v.f;
	case 1: return 0.0f;
	case 2: return (float)((int32_t)rnd() % 1000) / 7.0f;
	case 3: return (float)((int32_t)rnd()) * 1e-9f;
	default: return (float)(rnd() % 100000) / 10000.0f * (rnd() & 1 ? -1.f : 1.f);
	}
}

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	static const struct { uint64_t vt; int kind; } k[4] = {
		{ 0x1803196c8, PSVR2_ROBUST_NULL }, { 0x180319680, PSVR2_ROBUST_HUBER },
		{ 0x180319648, PSVR2_ROBUST_CLIP }, { 0x180319610, PSVR2_ROBUST_TUKEY },
	};
	long bad[4][4] = { 0 }, n = 0;
	for (int it = 0; it < 40000; it++) {
		for (int m = 0; m < 4; m++) {
			void **vt = SONY(k[m].vt);
			struct obj o = { vt, rfloat() };
			psvr2_robust rb = { k[m].kind, o.c };
			float in[16], a[16], b[16];
			for (int i = 0; i < 16; i++)
				in[i] = rfloat();
			struct vec vi = { .data = in, .n = 16 }, vo = { .data = a, .n = 16 };
			const float s = rfloat();
			for (int slot = 0; slot < 3; slot++) {
				memset(a, 0x55, sizeof a);
				((slot_fn)vt[slot])(&o, &vo, &vi, s);
				if (slot == 0) psvr2_robust_weights(&rb, b, in, 16, s);
				if (slot == 1) psvr2_robust_weights_sq(&rb, b, in, 16, s);
				if (slot == 2) psvr2_robust_sqrt_weights(&rb, b, in, 16, s);
				for (int i = 0; i < 16; i++)
					if (memcmp(&a[i], &b[i], 4)) {
						if (bad[m][slot]++ < 3)
							printf("kind %d slot %d c=%a s=%a in=%a sony=%a ours=%a\n", m, slot, o.c, s, in[i], a[i], b[i]);
					}
			}
			if (m == 1) {
				const float x = in[0];
				float r1 = ((rho_fn)vt[6])(&o, x), r2 = psvr2_robust_huber_rho(o.c, x);
				float q1 = ((rho_fn)vt[7])(&o, x), q2 = psvr2_robust_huber_rho_over_x2(o.c, x);
				if (memcmp(&r1, &r2, 4) || memcmp(&q1, &q2, 4))
					if (bad[1][3]++ < 3) printf("huber rho c=%a x=%a %a/%a %a/%a\n", o.c, x, r1, r2, q1, q2);
				float p = ((float (*)(void *))vt[3])(&o);
				int kk = ((int (*)(void))vt[4])();
				if (kk != m || memcmp(&p, m ? &o.c : &(float){1.0f}, 4))
					bad[m][3]++;
			}
			n++;
		}
	}
	long tot = 0;
	for (int m = 0; m < 4; m++)
		for (int s = 0; s < 4; s++)
			tot += bad[m][s];
	printf("robustifiers: %ld mismatches over %ld objects x 3 slots x 16 elems\n", tot, n);
	return tot != 0;
}
