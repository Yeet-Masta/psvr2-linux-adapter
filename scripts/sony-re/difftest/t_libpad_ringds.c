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
typedef void (*push_fn)(int32_t, psvr2_sense_input *);

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	((init_fn)SONY(0x1801d4320))();
	push_fn s_push = (push_fn)SONY(0x1801c6b20);
	memcpy(psvr2_imu_axes, SONY(0x180351668), sizeof psvr2_imu_axes);
	/* Slot 4: FUN_1801c3970 only searches slots 0-3, so its call is inert. */
	psvr2_pad *a = (psvr2_pad *)SONY(0x1805974e0) + 4;
	static psvr2_pad b, t;
	static const uint8_t modes[5] = { 0x05, 0x21, 0x22, 0x25, 0x26 };
	int bad = 0, n = 0, cov_est = 0, cov_led = 0, cov_touch = 0;

	for (int sc = 0; sc < 300; sc++) {
		memset(a, 0, sizeof *a);
		a->in_use = 1;
		a->handle = 200 + sc;
		a->out_handle = 0x7000 + sc;
		a->vid = 0x054c;
		a->pid = rnd() % 2 ? 0x0ce6 : 0x05c4;
		a->out_flags = rnd() % 2 ? 0 : rnd() % 4;
		a->trig_level = rnd() % 2;
		a->est_trig[0] = rnd() % 4 == 0;
		a->est_trig[1] = rnd() % 4 == 0;
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
		a->motion.tick_scale = 1.0f;
		a->motion.axis = rnd() % 4;
		psvr2_gyro_still_init_ps5(&a->motion.still, c);
		b = *a;

		uint64_t host = (uint64_t)rnd() * 16;
		uint32_t ticks = rnd() & 0xffff;
		for (int k = 0; k < 1500; k++, n++) {
			const uint32_t dt = sc % 4 == 0 ? 100 + rnd() % 300 : 1000 + rnd() % 4000;
			host += dt;
			ticks += dt;
			if (a->pid == 0x05c4)
				ticks &= 0xffff;
			psvr2_sense_input in;
			uint8_t *ib = (uint8_t *)&in;
			for (size_t i = 0; i < sizeof in; i++)
				ib[i] = (uint8_t)rnd();
			in.host_us = host;
			in.imu_ticks = ticks;
			if (rnd() % 8)
				in.status[0] = a->last ? a->last->status[0] : 0;
			for (int i = 0; i < 3; i++) {
				in.gyro[i] = (int16_t)(c[18 + i] + (int)(rnd() % 400) - 200);
				in.accel[i] = (int16_t)((int)(rnd() % 4000) - 2000);
			}
			in.accel[1] = (int16_t)(7000 + rnd() % 2000);
			cov_est += !a->trig_level || a->est_trig[0] || a->est_trig[1];
			cov_touch += !(in.touch_id[0] & 0x80) && !(in.touch_id[1] & 0x80);
			const psvr2_sense_input in2 = in;
			s_push(a->handle, &in);
			cov_led += psvr2_pad_push_sample_ds(&b, &in2) >= 0;
			t = *a;
			if (a->last)
				t.last = &b.ring[a->last - a->ring];
			if (memcmp(&t, &b, sizeof b) || memcmp(&in, &in2, sizeof in)) {
				if (bad < 5) {
					size_t o = 0;
					while (o < sizeof b && ((uint8_t *)&t)[o] == ((uint8_t *)&b)[o])
						o++;
					printf("ring ds sc %d k %d: pad diff 0x%zx\n", sc, k, o);
				}
				bad++;
				b = t;
			}
		}
	}
	printf("DualSense sample ring push: %d/%d mismatches (estimated %d, led calls %d, 2 fingers %d)\n",
	       bad, n, cov_est, cov_led, cov_touch);
	return bad != 0;
}
