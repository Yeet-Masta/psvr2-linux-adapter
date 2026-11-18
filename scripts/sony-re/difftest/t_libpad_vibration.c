#include "sony.h"
#include "psvr2_libpad.h"

typedef void (*init_fn)(void);
typedef int32_t (*vib_fn)(int32_t, uint64_t, uint64_t);

#define B(va) ((uint8_t *)SONY(va))

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	((init_fn)SONY(0x1801d4320))();
	InitializeCriticalSection((CRITICAL_SECTION *)SONY(0x1805e2f58));
	vib_fn vib = (vib_fn)SONY(0x1801c3020);
	uint8_t *pad = B(0x1805974e0), *dev = B(0x1805b1580);
	static const uint16_t pids[3] = { 0x0ce7, 0x0e45, 0x0e46 };
	int bad = 0, n = 200000;
	for (int i = 0; i < n; i++) {
		const int32_t h = 0x100 + rnd() % 1000;
		memset(pad, 0, 0x3410);
		memset(dev, 0, 0x2200);
		pad[0] = 1;
		*(uint16_t *)(pad + 2) = 0x054c;
		*(uint16_t *)(pad + 4) = pids[rnd() % 3];
		pad[6] = rnd() % 7;
		pad[0xa] = 1 + rnd() % 8;
		*(int32_t *)(pad + 0x10) = h;
		*(int32_t *)(pad + 0x1c) = h;
		*(uint64_t *)dev = 1;
		*(int32_t *)(dev + 8) = h;
		*(uint16_t *)(dev + 0x22) = *(uint16_t *)(pad + 4);
		*(uint16_t *)(dev + 0x20) = 0x054c;
		pad[0x33e0 + 0x18] = 2;	/* sync mode set: no HID probe */
		B(0x1805b1572)[0] = rnd() % 4 == 0;
		B(0x1805b1573)[0] = rnd() % 4 == 0;
		const int fresh = rnd() % 2;
		uint8_t *buf = pad + 0x2978;
		uint8_t ref[0x3c];
		if (!fresh) {
			/* Buffer 0 in flight; the call fills buffer 1 and queues it. */
			*(uint8_t **)(pad + 0x29f0) = buf;
			buf = pad + 0x29b4;
			for (int k = 0; k < 0x3c; k++)
				buf[k] = (uint8_t)rnd();
		}
		memcpy(ref, buf, sizeof ref);
		if (fresh)
			memset(ref, 0, sizeof ref);
		const uint8_t big = (uint8_t)rnd(), small = (uint8_t)rnd();
		vib(h, big, small);
		psvr2_sense_set_vibration(ref, fresh, pad[6], *B(0x1805b1572), *B(0x1805b1573), small);
		/* A fresh buffer is also sent at once; the send only ORs 0x02/0x10/0x80
		 * into byte 1 (checked in t_libpad_output), so compare the rest. */
		const int ok = fresh ? buf[0] == ref[0] && (buf[1] & 0x61) == (ref[1] & 0x61) &&
					      buf[3] == ref[3]
				     : !memcmp(buf, ref, sizeof ref) &&
					       *(uint8_t **)(pad + 0x29f8) == buf;
		if (!ok) {
			if (bad < 5)
				printf("vib fresh %d conn %d: %02x %02x %02x vs %02x %02x %02x\n", fresh, pad[6],
				       buf[0], buf[1], buf[3], ref[0], ref[1], ref[3]);
			bad++;
		}
	}
	printf("sense vibration: %d/%d mismatches\n", bad, n);
	return bad != 0;
}
