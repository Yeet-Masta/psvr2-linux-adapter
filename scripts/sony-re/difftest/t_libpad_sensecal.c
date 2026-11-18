#include "sony.h"
#include "psvr2_sensecal.h"

typedef uint32_t (*ref_fn)(void *ctx, int16_t *c);

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	ref_fn ref = (ref_fn)SONY(0x1801d54e0);
	int bad = 0, n = 200000;
	for (int i = 0; i < n; i++) {
		int16_t c[42];
		for (int k = 0; k < 42; k++)
			c[k] = (int16_t)(rnd() % 4 == 0 ? 0 : rnd());
		if (i % 7 == 0)
			c[18] = c[21];
		uint8_t ctx[0x200] = { 0 };
		const int ref_ok = ref(ctx, c) == 0 && ctx[0x70];
		psvr2_sense_cal o;
		const int ok = psvr2_sense_cal_compute(c, &o) == 0;
		if (ok != ref_ok) { bad++; continue; }
		if (!ok)
			continue;
		/* Reference layout: gyro bias 0x74, gyro scale 0x80, accel offset 0x8c, accel scale 0x98. */
		if (memcmp(ctx + 0x74, &o, sizeof o))
			bad++;
	}
	printf("sense calibration: %d/%d mismatches\n", bad, n);
	return bad != 0;
}
