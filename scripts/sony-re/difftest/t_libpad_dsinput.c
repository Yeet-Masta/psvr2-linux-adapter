#include "sony.h"
#include "psvr2_libpad.h"

typedef uint32_t (*bits_fn)(const uint8_t *, int, int, char);
typedef void (*dec_fn)(void *, const uint8_t *, uint64_t, uint64_t, psvr2_sense_input *);

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	bits_fn s_bits = (bits_fn)SONY(0x1801c83c0);
	dec_fn s_dec = (dec_fn)SONY(0x1801d1f50);
	int bad = 0, n = 0, fails = 0;

	for (int k = 0; k < 300000; k++, n++) {
		uint8_t p[16];
		for (int i = 0; i < 16; i++)
			p[i] = (uint8_t)rnd();
		const int bit = rnd() % 64, cnt = rnd() % 33, sg = rnd() % 2;
		bad += s_bits(p, bit, cnt, (char)sg) != psvr2_pad_read_bits(p, bit, cnt, sg);
	}
	printf("bit field read: %d/%d mismatches\n", bad, n);
	fails += bad;

	bad = n = 0;
	int cov_edge = 0;
	for (int k = 0; k < 300000; k++, n++) {
		uint8_t rep[0x50], dev[0x40] = { 0 };
		for (size_t i = 0; i < sizeof rep; i++)
			rep[i] = (uint8_t)rnd();
		if (rnd() % 3 == 0)
			rep[0x32 + rep[0] % 2] &= 0xf0;
		const uint16_t pids[4] = { 0x0ce6, 0x0df2, 0x0df2, 0x0e45 };
		*(uint16_t *)(dev + 0x20) = 0x054c;
		*(uint16_t *)(dev + 0x22) = pids[rnd() % 4];
		const int nodev = rnd() % 8 == 0;
		const uint8_t off = rnd() % 2;
		psvr2_sense_input a, b;
		uint8_t *ab = (uint8_t *)&a;
		for (size_t i = 0; i < sizeof a; i++)
			ab[i] = (uint8_t)rnd();
		if (rnd() % 2)
			a.buttons = a.edge_buttons = 0;
		b = a;
		const int edge = !nodev && psvr2_pad_is_edge(0x054c, *(uint16_t *)(dev + 0x22));
		cov_edge += edge;
		s_dec(nodev ? NULL : dev, rep, rnd(), 0xab00 | off, &a);
		psvr2_ds_input_decode(rep, off, edge, &b);
		if (memcmp(&a, &b, sizeof a)) {
			if (bad < 5) {
				size_t o = 0;
				while (((uint8_t *)&a)[o] == ((uint8_t *)&b)[o])
					o++;
				printf("ds decode k %d edge %d: diff at 0x%zx\n", k, edge, o);
			}
			bad++;
		}
	}
	printf("dualsense input decode: %d/%d mismatches (edge %d)\n", bad, n, cov_edge);
	fails += bad;
	return fails != 0;
}
