// SPDX-License-Identifier: GPL-2.0
/* Calibration blob parser vs Sony 0x180161b70. */
#include "sony.h"
#include "trace_difftest.h"
#include "psvr2_calib.h"

static uint8_t blob[0x10000], o1[PSVR2_CALIB_OUT_MAX + 0x100], o2[PSVR2_CALIB_OUT_MAX + 0x100];

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	typedef uint64_t (*f_parse)(void *, const uint8_t *, uint64_t, uint8_t *);
	int bad = 0, n = 20000, ok = 0;
	static const char *vers[] = { "1.0", "v2:d", "d:x", "ab:cd", ":", "x:yd", "12345678", "" };

	for (int i = 0; i < n; i++) {
		for (size_t k = 0; k < 0x3000; k += 4) { uint32_t r = rnd(); memcpy(blob + k, &r, 4); }
		if (i % 4) { blob[0] = 'S'; blob[1] = 'I'; blob[2] = 'E'; blob[3] = rnd() % 3; }
		const char *v = vers[rnd() % 8];
		memset(blob + 7, 0, 8);
		memcpy(blob + 7, v, strlen(v));
		if (rnd() % 8 == 0) blob[7 + rnd() % 8] = (uint8_t)rnd();
		blob[0xf] = (i % 50) ? rnd() % 6 : (uint8_t)rnd();
		blob[0x10] = (i % 50) ? rnd() % 5 : (uint8_t)rnd();
		int dbl = strchr(v, ':') && strchr(strchr(v, ':'), 'd');
		size_t size = 0x18 + blob[0xf] * (dbl ? 0xe4 : 0x94) + blob[0x10] * 0x34;
		if (i % 7 == 0) size += (int)(rnd() % 9) - 4;
		if (size > 0xffff) size &= 0xffff;
		uint16_t s16 = (uint16_t)size;
		if (i % 11 == 0) s16++;
		memcpy(blob + 4, &s16, 2);
		memset(o1, 0x5a, sizeof(o1));
		memset(o2, 0x5a, sizeof(o2));
		int r1 = (int)(((f_parse)SONY(0x180161b70))(NULL, blob, size, o1) & 0xff);
		int r2 = psvr2_calib_parse(blob, size, o2);
		ok += r2;
		if (r1 != r2 || memcmp(o1, o2, sizeof(o1))) {
			if (bad++ < 5) {
				size_t d = 0;
				while (d < sizeof(o1) && o1[d] == o2[d]) d++;
				printf("calib: ncam %u nimu %u ver '%s' sony %d ours %d diff@0x%zx\n",
				       blob[0xf], blob[0x10], v, r1, r2, d);
			}
		}
	}
	printf("(%d accepted)\n", ok);
	return report("calib parse 0x180161b70", bad, n);
}
