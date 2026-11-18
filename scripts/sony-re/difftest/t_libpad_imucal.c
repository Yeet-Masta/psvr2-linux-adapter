#include "sony.h"
#include "psvr2_libpad.h"

typedef void (*parse_fn)(int16_t *, const uint8_t *, char);

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	parse_fn s_parse = (parse_fn)SONY(0x1801d2cf0);
	int bad = 0, n = 0;
	for (int k = 0; k < 200000; k++, n++) {
		uint8_t rep[0x29];
		for (size_t i = 0; i < sizeof rep; i++)
			rep[i] = (uint8_t)rnd();
		int16_t a[20], b[20];
		memset(a, 0x5a, sizeof a);
		memset(b, 0x5a, sizeof b);
		const char g = (char)(k % 3 ? k % 2 : rnd());
		s_parse(a, rep, g);
		psvr2_pad_imu_cal_parse(b, rep, g != 0);
		bad += memcmp(a, b, sizeof a) != 0;
	}
	printf("imu calibration report parse: %d/%d mismatches\n", bad, n);
	return bad != 0;
}
