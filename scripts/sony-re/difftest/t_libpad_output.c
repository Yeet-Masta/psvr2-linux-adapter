#include "sony.h"
#include "psvr2_libpad.h"

typedef void (*init_fn)(void);
typedef int32_t (*out_fn)(int32_t, uint8_t *, uint64_t, uint64_t);

#define B(va) ((uint8_t *)SONY(va))
static uint64_t g_time;
static void sony_cb(uint64_t *t) { *t = g_time; }
static uint64_t our_cb(void) { return g_time; }

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	((init_fn)SONY(0x1801d4320))();
	InitializeCriticalSection((CRITICAL_SECTION *)SONY(0x1805e2f58));
	out_fn out = (out_fn)SONY(0x1801cbeb0);
	uint8_t *pad = B(0x1805974e0), *dev = B(0x1805b1580);
	static const uint16_t pids[3] = { 0x0ce7, 0x0e45, 0x0e46 };
	*(void **)SONY(0x1805974d0) = (void *)sony_cb;
	psvr2_sync_headset_time = our_cb;
	int bad = 0, n = 200000;
	for (int i = 0; i < n; i++) {
		const int32_t h = 0x100 + rnd() % 1000;
		memset(pad, 0, 0x3410);
		memset(dev, 0, 0x2200);
		pad[0] = 1;
		*(uint16_t *)(pad + 2) = 0x054c;
		*(uint16_t *)(pad + 4) = pids[rnd() % 3];
		pad[6] = rnd() % 7;
		pad[0xa] = rnd() % 4 ? rnd() % 9 : (uint8_t)rnd();
		pad[0xb] = rnd() % 4 ? rnd() % 12 : (uint8_t)rnd();
		*(int32_t *)(pad + 0x1c) = h;
		*(uint16_t *)(pad + 0x50) = rnd() % 2 ? 0x200 + rnd() % 0x100 : (uint16_t)rnd();
		pad[0x54] = rnd() % 2;
		*(int32_t *)(pad + 0x68) = rnd() % 3;
		*(uint64_t *)dev = 1;
		*(int32_t *)(dev + 8) = h;
		*(uint16_t *)(dev + 0x20) = 0x054c;
		*(uint16_t *)(dev + 0x22) = *(uint16_t *)(pad + 4);
		*(uint32_t *)(dev + 0x1d0) = rnd() % 4 ? 0x10000 + rnd() % 0x40 : rnd() % 0x20000;
		psvr2_sync_state *st = (psvr2_sync_state *)(pad + 0x2a60);
		st->fit_off = rnd() % 2 ? (int64_t)(int32_t)rnd() : 0;
		st->drift_ppb = rnd() % 2 ? (int32_t)rnd() % 50000 : 0;
		st->ref_ctrl_us = rnd();
		st->simple_off = (int32_t)rnd();
		psvr2_ledsync *ls = (psvr2_ledsync *)(pad + 0x33e0);
		for (size_t k = 0; k < sizeof *ls; k++)
			((uint8_t *)ls)[k] = (uint8_t)rnd();
		ls->phase = rnd() % 8;
		ls->sync_mode = rnd() % 3 ? rnd() % 3 : 0;
		ls->probes = rnd() % 8;
		if (ls->sync_mode == 0 && ls->probes <= 3)
			*(uint32_t *)(dev + 0x1d0) = rnd() % 0x1006d; /* no HID probe */
		const int lr = rnd() % 3;
		ls->last_resync = lr == 0 ? 0 : lr == 1 ? 1 : ~0ull;
		g_time = rnd() % 4 ? (uint64_t)rnd() * 1000 : 0;
		B(0x1805b1572)[0] = rnd() % 4 == 0;
		B(0x1805b1573)[0] = rnd() % 4 == 0;
		B(0x1805b157b)[0] = rnd() % 2;
		B(0x1805e349c)[0] = rnd() % 2;
		psvr2_sync_fit_enabled = *B(0x180351658) = rnd() % 4 != 0;

		uint8_t b1[0x40], b2[0x40];
		for (int k = 0; k < 0x40; k++)
			b1[k] = (uint8_t)rnd();
		b1[0] = 2;
		memcpy(b2, b1, sizeof b1);
		psvr2_sense_out_ctx c = {
			.conn = pad[6], .vol_haptic = pad[0xa], .vol_trigger = pad[0xb],
			.hand = pad[0x54], .fw = *(uint16_t *)(pad + 0x50),
			.link = *(int32_t *)(pad + 0x68), .dev_fw = *(uint32_t *)(dev + 0x1d0),
			.g_rumble_a = *B(0x1805b1572), .g_rumble_b = *B(0x1805b1573),
			.g_flag2_10 = *B(0x1805b157b), .g_flag2_02 = *B(0x1805e349c),
		};
		psvr2_sync_state sb = *st;
		psvr2_ledsync lb = *ls;
		uint64_t last_out = 0;

		out(h, b1, 0x30, 0);

		uint32_t stamp;
		memcpy(&stamp, b1 + 0x10, 4);
		const uint64_t now = *(uint64_t *)(pad + 0x2a58);
		const uint64_t rnow = (ls->last_resync != lb.last_resync && lb.last_resync) ?
				      ls->last_resync : 0x7fffffffffffull;
		psvr2_sense_output_fill(&c, &sb, &lb, b2, &last_out, now, stamp, rnow);
		if (memcmp(b1, b2, 0x30) || memcmp(ls, &lb, sizeof lb) || last_out != now) {
			if (bad < 5) {
				for (int o = 0; o < 0x30; o++)
					if (b1[o] != b2[o]) {
						printf("out: buf diff at 0x%x (%02x vs %02x) phase %d\n", o, b1[o], b2[o], ls->phase);
						break;
					}
				if (memcmp(ls, &lb, sizeof lb))
					printf("out: ledsync differs phase %d\n", ls->phase);
			}
			bad++;
		}
	}
	printf("sense output fill: %d/%d mismatches\n", bad, n);
	return bad != 0;
}
