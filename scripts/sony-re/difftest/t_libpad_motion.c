#include "sony.h"
#include "psvr2_libpad.h"
#include "psvr2_sensecal.h"
#include <windows.h>

/* Route libm to ucrtbase.dll, the CRT Sony's DLL imports, so the test
 * compares our arithmetic and not libm accuracy. */
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

typedef int32_t (*still_fn)(void *, int16_t, int16_t, int16_t, float *, float *);
typedef void (*sinit_fn)(void *, const int16_t *);
typedef void (*mot_fn)(uint8_t *, uint8_t *, const void *);

static float frand(float a) { return ((int32_t)rnd() / 2147483648.0f) * a; }

static int first_diff(const void *a, const void *b, size_t n)
{
	for (size_t o = 0; o < n; o++)
		if (((const uint8_t *)a)[o] != ((const uint8_t *)b)[o])
			return (int)o;
	return -1;
}

static void make_cal(int16_t c[42])
{
	for (int k = 0; k < 42; k++)
		c[k] = (int16_t)(rnd() % 200 - 100);
	c[18] = rnd() % 40 - 20; c[19] = rnd() % 40 - 20; c[20] = rnd() % 40 - 20;
	c[21] = c[18] + 8000 + rnd() % 2000; c[24] = c[18] - 8000 - rnd() % 2000;
	c[28] = c[19] + 8000 + rnd() % 2000; c[31] = c[19] - 8000 - rnd() % 2000;
	c[35] = c[20] + 8000 + rnd() % 2000; c[38] = c[20] - 8000 - rnd() % 2000;
	c[39] = 500 + rnd() % 100; c[40] = c[39];
	c[0] = 8000 + rnd() % 500; c[3] = -8000 - rnd() % 500;
	c[7] = 8000 + rnd() % 500; c[10] = -8000 - rnd() % 500;
	c[14] = 8000 + rnd() % 500; c[17] = -8000 - rnd() % 500;
}

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	still_fn s_still = (still_fn)SONY(0x1801d5d80);
	sinit_fn s_init5 = (sinit_fn)SONY(0x1801d5c30), s_init4 = (sinit_fn)SONY(0x1801d5ad0);
	mot_fn s_mot = (mot_fn)SONY(0x1801d4a90);
	int bad = 0, n = 0;

	/* Still detector. */
	for (int sc = 0; sc < 3000; sc++) {
		int16_t c[42];
		make_cal(c);
		psvr2_gyro_still a, b;
		memset(&a, 0, sizeof a);
		if (sc % 2) {
			s_init5(&a, c);
			psvr2_gyro_still_reset(&b);
			psvr2_gyro_still_init_ps5(&b, c);
		} else {
			const int16_t d[8] = { c[18], c[19], c[20], c[21], c[28], c[35], c[39], c[40] };
			s_init4(&a, d);
			psvr2_gyro_still_reset(&b);
			psvr2_gyro_still_init_ds4(&b, d);
		}
		if (sc % 7 == 0)
			a.flags = b.flags = rnd() % 4;
		bad += memcmp(&a, &b, sizeof a) != 0;
		const int noise = 1 + rnd() % (sc % 3 ? 4 : 60);
		const int drift = rnd() % 20 - 10;
		for (int k = 0; k < 2000; k++, n++) {
			int16_t v[3];
			for (int i = 0; i < 3; i++)
				v[i] = (int16_t)(c[18 + i] + drift + (int)(rnd() % (2 * noise + 1)) - noise);
			if (rnd() % 500 == 0)
				v[rnd() % 3] += 3000;
			float ba[3] = { 0 }, ca[3] = { 0 }, bb[3] = { 0 }, cb[3] = { 0 };
			const int ra = s_still(&a, v[0], v[1], v[2], ba, ca) & 0xff;
			const int rb = psvr2_gyro_still_update(&b, v[0], v[1], v[2], bb, cb);
			if (ra != rb || memcmp(&a, &b, sizeof a) || (ra && (memcmp(ba, bb, 12) || memcmp(ca, cb, 12)))) {
				if (bad < 5)
					printf("still sc %d k %d: r %d/%d diff 0x%x\n", sc, k, ra, rb, first_diff(&a, &b, sizeof a));
				bad++;
				b = a;
			}
		}
	}
	printf("gyro still detector: %d/%d mismatches\n", bad, n);
	int fails = bad;

	/* IMU conversion + orientation. */
	memcpy(psvr2_imu_axes, SONY(0x180351668), sizeof psvr2_imu_axes);
	static uint8_t pad[0x3410], smp[2][0xa0];
	bad = 0;
	n = 0;
	for (int sc = 0; sc < 2000; sc++) {
		memset(pad, 0, sizeof pad);
		*(uint16_t *)(pad + 2) = 0x054c;
		*(uint16_t *)(pad + 4) = rnd() % 4 ? 0x0e45 : 0x0ce6;
		int16_t c[42];
		make_cal(c);
		psvr2_sense_cal cal;
		psvr2_sense_cal_compute(c, &cal);
		psvr2_motion *m = (psvr2_motion *)(pad + 0x6c);
		memcpy(m->gyro_bias, &cal, sizeof cal);
		m->flags = rnd() % 8;
		m->tick_scale = rnd() % 2 ? 1.0f / 3.0f : 1.0f;
		m->axis = rnd() % 4;
		psvr2_gyro_still_init_ps5(&m->still, c);
		static psvr2_motion mb;
		mb = *m;
		uint32_t ts = rnd();
		float rate[3] = { frand(3000), frand(3000), frand(3000) };
		int cur = 0;
		*(void **)(pad + 0x170) = NULL;
		uint8_t *prev = NULL;
		for (int k = 0; k < 600; k++, n++) {
			psvr2_sense_input in;
			memset(&in, 0, sizeof in);
			ts += 3000 + rnd() % 200;
			if (rnd() % 300 == 0)
				ts = rnd();
			in.imu_ticks = ts;
			in.temperature = (uint8_t)rnd();
			for (int i = 0; i < 3; i++) {
				if (rnd() % 50 == 0)
					rate[i] = frand(3000);
				in.gyro[i] = (int16_t)(c[18 + i] + (int)rate[i] + (int)(rnd() % 9) - 4);
			}
			/* Gravity of about 1 g along a slowly varying axis. */
			in.accel[0] = (int16_t)(c[0] / 2 + (int)(rnd() % 2000) - 1000);
			in.accel[1] = (int16_t)(c[7] * 3 / 4 + (int)(rnd() % 400) - 200);
			in.accel[2] = (int16_t)((int)(rnd() % 4000) - 2000);
			uint8_t *s = smp[cur];
			memset(s, 0, 0xa0);
			static uint8_t s2[0xa0];
			memcpy(s2, s, 0xa0);
			s_mot(pad, s, &in);
			psvr2_motion_update(&mb, (psvr2_motion_out *)(s2 + 0x58),
					    prev ? (const psvr2_motion_out *)(prev + 0x58) : NULL,
					    psvr2_pad_is_dualsense(0x054c, *(uint16_t *)(pad + 4)), &in);
			if (memcmp(m, &mb, sizeof mb) || memcmp(s, s2, 0xa0)) {
				if (bad < 5)
					printf("motion sc %d k %d flags %d axis %d: pad diff 0x%x, sample diff 0x%x\n", sc, k,
					       m->flags, m->axis, first_diff(m, &mb, sizeof mb), first_diff(s, s2, 0xa0));
				bad++;
				mb = *m;
			}
			*(uint8_t **)(pad + 0x170) = s;
			prev = s;
			cur ^= 1;
		}
	}
	printf("motion update: %d/%d mismatches\n", bad, n);
	fails += bad;
	return fails != 0;
}
