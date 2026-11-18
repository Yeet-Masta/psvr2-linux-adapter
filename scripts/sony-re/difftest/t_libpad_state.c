#include "sony.h"
#include "psvr2_libpad.h"

typedef void (*st_fn)(uint8_t *, uint8_t *);
#define B(va) ((uint8_t *)SONY(va))

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	st_fn st = (st_fn)SONY(0x1801c6250);
	static uint8_t pad[0x3410];
	static const uint16_t pids[4] = { 0x0ce7, 0x0e45, 0x0e46, 0x0ce6 };
	int bad = 0, n = 200000;
	for (int i = 0; i < n; i++) {
		for (size_t k = 0; k < sizeof pad; k += 4)
			*(uint32_t *)(pad + k) = rnd();
		*(uint16_t *)(pad + 2) = 0x054c;
		*(uint16_t *)(pad + 4) = pids[rnd() % 4];
		pad[0x54] = rnd() % 3;
		pad[0xa] = rnd() % 2 ? 0 : (uint8_t)rnd();
		pad[0xb] = rnd() % 2 ? 0 : (uint8_t)rnd();
		B(0x1805e349d)[0] = rnd() % 2;
		*(uint32_t *)SONY(0x1805e3498) = rnd() % 4 ? rnd() % 2 : rnd();
		uint8_t b1[0x40], b2[0x40];
		for (int k = 0; k < 0x40; k++)
			b1[k] = (uint8_t)rnd();
		memcpy(b2, b1, sizeof b1);
		st(b1, pad);
		const int left = psvr2_pad_is_vr_left(0x054c, *(uint16_t *)(pad + 4), pad[0x54]);
		psvr2_sense_fill_state(b2, pad[0xb], pad[0xa], pad + (left ? 0x2a19 : 0x2a24),
				       *B(0x1805e349d), (uint8_t)*(uint32_t *)SONY(0x1805e3498));
		bad += memcmp(b1, b2, sizeof b1) != 0;
	}
	printf("sense full-state fill: %d/%d mismatches\n", bad, n);
	return bad != 0;
}
