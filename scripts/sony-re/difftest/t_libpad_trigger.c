#include "sony.h"
#include "psvr2_libpad.h"

typedef void (*init_fn)(void);
typedef int32_t (*trg_fn)(int32_t, uint64_t);
typedef int32_t (*eff_fn)(int32_t, const uint8_t *);

#define B(va) ((uint8_t *)SONY(va))

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	((init_fn)SONY(0x1801d4320))();
	InitializeCriticalSection((CRITICAL_SECTION *)SONY(0x1805e2f58));
	trg_fn trg = (trg_fn)SONY(0x1801c4a30);
	eff_fn effset = (eff_fn)SONY(0x1801c44a0);
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
		pad[0x54] = rnd() % 3;
		pad[0xb] = rnd() % 3 ? rnd() % 12 : 0;
		for (int k = 0; k < 22; k++)
			pad[0x2a19 + k] = (uint8_t)rnd();
		const uint8_t old = pad[0xb], lvl = rnd() % 3 ? rnd() % 12 : 0;
		const uint16_t pid = *(uint16_t *)(pad + 4);
		const uint8_t *eff = psvr2_pad_is_vr_left(0x054c, pid, pad[0x54]) ? pad + 0x2a19 :
				     psvr2_pad_is_vr_right(0x054c, pid, pad[0x54]) ? pad + 0x2a24 : NULL;
		uint8_t effc[11];
		if (eff)
			memcpy(effc, eff, 11);
		uint8_t wire[23];
		const int use_eff = i % 2;
		for (int k = 0; k < 23; k++)
			wire[k] = (uint8_t)rnd();
		if (use_eff) {
			*(int32_t *)(pad + 0x18) = rnd() % 6;
			effset(h, wire);
			if (old)
				psvr2_sense_set_trigger_effect(ref, psvr2_pad_is_vr_left(0x054c, pid, pad[0x54]),
							       psvr2_pad_is_vr_right(0x054c, pid, pad[0x54]), wire);
		} else {
			trg(h, lvl);
			psvr2_sense_set_trigger_level(ref, fresh, old, lvl, pad[0xa], eff ? effc : NULL);
			if (pad[0xb] != lvl)
				bad++;
		}
		/* A fresh buffer is also sent at once; the send only ORs 0x02/0x10/0x80
		 * into byte 1 (checked in t_libpad_output), so compare the rest. */
		/* The send may set buf[1] bits 0x02/0x80, and 0x10 + byte 0x23 only
		 * when 0x10 was clear; mask those. */
		const int sent = fresh && (use_eff ? old != 0 : !(lvl == 0 && old == 0));
		if (sent) {
			buf[1] &= 0x6d; ref[1] &= 0x6d;
			if (!(ref[1] & 0x10))
				buf[0x23] = ref[0x23];
			memcpy(buf + 0x10, ref + 0x10, 0x13); /* stamp + LED fields */
			memcpy(buf + 0x2, ref + 0x2, 1);
		}
		const int ok = fresh ? !memcmp(buf, ref, 0x30)
				     : !memcmp(buf, ref, sizeof ref) &&
					       *(uint8_t **)(pad + 0x29f8) == ((use_eff ? old : (lvl || old)) ? buf : NULL);
		if (!ok) {
			if (bad < 5)
				printf("trg fresh %d old %d lvl %d: %02x %02x %02x %02x vs %02x %02x %02x %02x\n", fresh, old, lvl,
				       buf[0], buf[1], buf[5], buf[0x23], ref[0], ref[1], ref[5], ref[0x23]);
			bad++;
		}
	}
	printf("sense trigger level/effect: %d/%d mismatches\n", bad, n);
	return bad != 0;
}
