#include "sony.h"
#include "psvr2_libpad.h"

typedef int32_t (*cal_fn)(psvr2_pad *, int16_t *);

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	cal_fn s_cal = (cal_fn)SONY(0x1801d5170);
	static psvr2_pad a, b;
	static const uint16_t pids[5] = { 0x05c4, 0x09cc, 0x0ce6, 0x0df2, 0x0ba0 };
	int bad = 0, n = 0, cov_fail = 0, cov_fix = 0;
	for (int k = 0; k < 400000; k++, n++) {
		uint8_t *ap = (uint8_t *)&a;
		for (size_t i = 0; i < 0x160; i++)
			ap[i] = (uint8_t)rnd();
		a.vid = rnd() % 8 ? 0x054c : (uint16_t)rnd();
		a.pid = pids[rnd() % 5];
		a.hid_ver = rnd() % 3 ? 0x0f + rnd() % 8 : (int32_t)rnd();
		b = a;
		int16_t ca[18], cb[18];
		for (int i = 0; i < 18; i++)
			ca[i] = (int16_t)(rnd() % 3 ? (int)(rnd() % 20000) - 10000 : (int)rnd());
		if (rnd() % 20 == 0)
			ca[3 + 2 * (rnd() % 3)] = ca[4 + 2 * (rnd() % 3)] = ca[rnd() % 3];
		memcpy(cb, ca, sizeof ca);
		const int32_t ra = s_cal(&a, ca);
		const int32_t rb = psvr2_motion_cal_ds(&b.motion, b.vid, b.pid, b.hid_ver, cb);
		cov_fail += ra != 0;
		cov_fix += ca[9] == 540 && ca[10] == 540;
		if (ra != rb || memcmp(&a, &b, 0x160) || memcmp(ca, cb, sizeof ca)) {
			if (bad < 5) {
				size_t o = 0;
				while (o < 0x160 && ((uint8_t *)&a)[o] == ((uint8_t *)&b)[o])
					o++;
				printf("ds cal k %d: ret %x/%x pad diff 0x%zx\n", k, ra, rb, o);
			}
			bad++;
		}
	}
	printf("DS4/DualSense motion calibration: %d/%d mismatches (failed %d, speed fix %d)\n",
	       bad, n, cov_fail, cov_fix);
	return bad != 0;
}
