#include "sony.h"
#include "psvr2_libpad.h"
#include "psvr2_sensecal.h"
#include <windows.h>

/* Route libm to ucrtbase.dll, the CRT Sony's DLL imports. */
static FARPROC ucrt(const char *n)
{
	static HMODULE h;
	if (!h)
		h = LoadLibraryA("ucrtbase.dll");
	return GetProcAddress(h, n);
}
float sinf(float x) { static float (*p)(float); if (!p) p = (float (*)(float))ucrt("sinf"); return p(x); }
float cosf(float x) { static float (*p)(float); if (!p) p = (float (*)(float))ucrt("cosf"); return p(x); }
float atan2f(float y, float x) { static float (*p)(float, float); if (!p) p = (float (*)(float, float))ucrt("atan2f"); return p(y, x); }
void sincosf(float x, float *s, float *c) { *s = sinf(x); *c = cosf(x); }

typedef void (*init_fn)(void);
typedef void (*sinit_fn)(void *, uint8_t, uint8_t);
typedef void (*push_fn)(int32_t, psvr2_sense_input *, uint8_t);

/* Sony's slot with its ring pointer rebased onto ours. */
static void rebase(psvr2_pad *dst, const psvr2_pad *sony, const psvr2_pad *ours)
{
	*dst = *sony;
	if (sony->last)
		dst->last = (psvr2_pad_sample *)((uint8_t *)ours + ((const uint8_t *)sony->last - (const uint8_t *)sony));
}

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	((init_fn)SONY(0x1801d4320))();
	sinit_fn s_init = (sinit_fn)SONY(0x1801c1660);
	push_fn s_push = (push_fn)SONY(0x1801c6ee0);
	memcpy(psvr2_imu_axes, SONY(0x180351668), sizeof psvr2_imu_axes);
	psvr2_pad *a = (psvr2_pad *)SONY(0x1805974e0);
	static psvr2_pad b, t;
	static const uint8_t modes[5] = { 0x05, 0x21, 0x22, 0x25, 0x26 };
	const uint32_t g0[4] = { 23000, 13000, 4300, 500 };
	int bad = 0, n = 0, cov_est = 0, cov_gap = 0;

	for (int sc = 0; sc < 300; sc++) {
		memcpy(SONY(0x180351638), g0, 16);
		memcpy(&psvr2_sync_g, g0, 16);
		memset(a, 0, sizeof *a);
		a->in_use = 1;
		a->handle = 100 + sc;
		a->vid = 0x054c;
		a->pid = rnd() % 4 ? 0x0e45 : 0x0ce6;
		a->trig_level = rnd() % 2;
		a->eff_mode0 = modes[rnd() % 5];
		a->eff_mode1 = modes[rnd() % 5];
		for (int i = 0; i < 10; i++) {
			a->eff0[i] = (uint8_t)rnd();
			a->eff1[i] = (uint8_t)rnd();
		}
		int16_t c[42];
		for (int k = 0; k < 42; k++)
			c[k] = (int16_t)(rnd() % 200 - 100);
		c[0] = 8000; c[3] = -8000; c[7] = 8000; c[10] = -8000; c[14] = 8000; c[17] = -8000;
		c[21] = c[18] + 9000; c[28] = c[19] + 9000; c[35] = c[20] + 9000;
		c[24] = c[18] - 9000; c[31] = c[19] - 9000; c[38] = c[20] - 9000;
		c[39] = c[40] = 540;
		psvr2_sense_cal cal;
		psvr2_sense_cal_compute(c, &cal);
		memcpy(a->motion.gyro_bias, &cal, sizeof cal);
		a->motion.flags = rnd() % 8;
		a->motion.tick_scale = 1.0f / 3.0f;
		a->motion.axis = rnd() % 4;
		psvr2_gyro_still_init_ps5(&a->motion.still, c);
		const uint8_t bt = rnd() % 2;
		s_init(&a->sync, bt, rnd() % 2);
		b = *a;

		uint64_t host = (uint64_t)rnd() * 16;
		uint32_t ticks = rnd(), cticks = rnd();
		for (int k = 0; k < 1500; k++, n++) {
			const uint32_t dt = sc % 4 == 0 ? 100 + rnd() % 300 : bt ? 4000 + rnd() % 300 : 1000 + rnd() % 4000;
			host += dt;
			ticks += dt * 3;
			cticks += dt * 3 + rnd() % 5;
			if (rnd() % 200 == 0) {
				ticks += 30000 + rnd() % 100000;
				cov_gap++;
			}
			psvr2_sense_input in, in2;
			memset(&in, 0, sizeof in);
			uint8_t *ib = (uint8_t *)&in;
			for (size_t i = 0x10; i < sizeof in; i++)
				ib[i] = (uint8_t)rnd();
			in.host_us = host;
			in.imu_ticks = ticks;
			in.ctrl_ticks = cticks;
			in.echo = (uint32_t)(host - 3000 - rnd() % 5000);
			for (int i = 0; i < 3; i++) {
				in.gyro[i] = (int16_t)(c[18 + i] + (int)(rnd() % 400) - 200);
				in.accel[i] = (int16_t)((int)(rnd() % 4000) - 2000);
			}
			in.accel[1] = (int16_t)(7000 + rnd() % 2000);
			in.flag0 &= 1;
			in.flag3 &= 8;
			in2 = in;
			cov_est += !a->trig_level;
			s_push(a->handle, &in, bt);
			psvr2_pad_push_sample(&b, &in2, bt);
			rebase(&t, a, &b);
			if (memcmp(&t, &b, sizeof b) || memcmp(&in, &in2, sizeof in)) {
				if (bad < 5) {
					size_t o = 0;
					while (o < sizeof b && ((uint8_t *)&t)[o] == ((uint8_t *)&b)[o])
						o++;
					printf("ring sc %d k %d trig_level %d: pad diff 0x%zx, input %s\n", sc, k,
					       a->trig_level, o, memcmp(&in, &in2, sizeof in) ? "differs" : "same");
				}
				bad++;
				b = t;
			}
		}
	}
	printf("sample ring push: %d/%d mismatches (estimated %d, gaps %d)\n", bad, n, cov_est, cov_gap);
	return bad != 0;
}
