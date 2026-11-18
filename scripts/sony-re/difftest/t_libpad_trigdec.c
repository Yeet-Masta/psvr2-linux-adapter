#include "sony.h"
#include "psvr2_libpad.h"

typedef uint32_t (*dec_fn)(uint8_t, const uint8_t *, uint8_t *, uint8_t);

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	dec_fn dec = (dec_fn)SONY(0x1801c65f0);
	static const uint8_t modes[] = { 0x05, 0x21, 0x22, 0x25, 0x26, 0x00, 0x23 };
	int bad = 0, n = 400000;
	for (int i = 0; i < n; i++) {
		uint8_t w[16], p1[0x78], p2[0x78];
		for (int k = 0; k < 16; k++)
			w[k] = (uint8_t)rnd();
		if (rnd() % 3 == 0)
			w[1] &= 3, w[0] = (uint8_t)(1u << (rnd() % 8)) | (rnd() % 2 ? w[0] & 0x30 : 0);
		const uint8_t m = rnd() % 8 ? modes[rnd() % 7] : (uint8_t)rnd();
		if ((m == 0x21 || m == 0x26) && !((w[0] | w[1] << 8) & 0x3ff))
			w[0] |= 1; /* all-zero mask reads uninitialised stack in Sony */
		for (int k = 0; k < 0x78; k++)
			p1[k] = (uint8_t)rnd();
		memcpy(p2, p1, sizeof p1);
		const uint8_t slot = rnd() % 2;
		const uint32_t r1 = dec(m, w, p1, slot);
		const uint32_t r2 = (uint32_t)psvr2_pad_trigger_effect_decode(m, w, p2, slot);
		if (r1 != r2 || memcmp(p1, p2, sizeof p1)) {
			if (bad < 5)
				printf("trigdec mode %02x: r %x/%x\n", m, r1, r2);
			bad++;
		}
	}
	typedef uint64_t (*st_fn)(uint8_t, const uint8_t *, uint8_t, uint8_t);
	st_fn stf = (st_fn)SONY(0x1801c90e0);
	int bad2 = 0;
	for (int i = 0; i < 400000; i++) {
		uint8_t w[4];
		for (int k = 0; k < 4; k++)
			w[k] = (uint8_t)rnd();
		if (rnd() % 2)
			w[0] = (uint8_t)(1u << (rnd() % 8) | 1u << (rnd() % 8)), w[1] &= 3;
		const uint8_t m = rnd() % 8 ? modes[rnd() % 7] : (uint8_t)rnd();
		const uint8_t v = (uint8_t)rnd(), prev = rnd() % 2 ? 5 : (uint8_t)(rnd() % 8);
		bad2 += (uint8_t)stf(m, w, v, prev) != psvr2_pad_trigger_state(m, w, v, prev);
	}
	printf("trigger state: %d/400000 mismatches\n", bad2);
	bad += bad2;
	printf("trigger effect decode: %d/%d mismatches\n", bad, n);
	return bad != 0;
}
