#include "sony.h"
#include "psvr2_libpad.h"

typedef void (*dec_fn)(void *, const uint8_t *, uint64_t, uint8_t, void *);

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	dec_fn dec = (dec_fn)SONY(0x1801d1d10);
	static uint8_t dev[0x2200];
	int bad = 0, n = 300000;
	for (int i = 0; i < n; i++) {
		uint8_t rep[0x60];
		for (size_t k = 0; k < sizeof rep; k++)
			rep[k] = (uint8_t)rnd();
		const uint8_t bt = rnd() % 2;
		if (rnd() % 4 == 0)
			memset(rep + bt + 0x30, 0, 4);
		const uint8_t hand = rnd() % 4 ? rnd() % 2 : (uint8_t)rnd();
		dev[0x1e4] = hand;
		psvr2_sense_input a, b;
		/* Callers pass a zeroed record; also test that existing bits survive. */
		memset(&a, 0, sizeof a);
		if (rnd() % 4 == 0)
			for (size_t k = 0; k < sizeof a; k++)
				((uint8_t *)&a)[k] = (uint8_t)rnd();
		b = a;
		dec(dev, rep, bt ? 0x4e : 0x40, bt, &a);
		psvr2_sense_input_decode(hand, rep, bt, &b);
		if (memcmp(&a, &b, sizeof a)) {
			if (bad < 3)
				for (size_t o = 0; o < sizeof a; o++)
					if (((uint8_t *)&a)[o] != ((uint8_t *)&b)[o]) {
						printf("decode: first diff at 0x%zx\n", o);
						break;
					}
			bad++;
		}
	}
	printf("sense input decode: %d/%d mismatches\n", bad, n);
	return bad != 0;
}
