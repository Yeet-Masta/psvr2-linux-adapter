#include "sony.h"
#include "psvr2_libpad.h"

typedef void (*init_fn)(void);
typedef void (*sinit_fn)(void *, uint8_t, uint8_t);
typedef void (*fit_fn)(void *, uint64_t, uint32_t, uint32_t, uint32_t, uint8_t);
typedef int32_t (*c2h_fn)(void *, uint64_t, int64_t *);
typedef int32_t (*h2c_fn)(void *, int64_t, int32_t *);
typedef int32_t (*cb_fn)(int32_t, int32_t);
typedef uint32_t (*cc_fn)(void *, uint32_t);
typedef uint8_t (*cp_fn)(void *, uint8_t);
typedef void (*mode_fn)(void *, uint8_t);
typedef void (*cmd_fn)(void *, void *, const uint8_t *, uint8_t);
typedef uint32_t (*t2u_fn)(uint32_t);

#define G(va, T) (*(T *)SONY(va))
static uint64_t g_time;
static void sony_cb(uint64_t *t) { *t = g_time; }
static uint64_t our_cb(void) { return g_time; }

static void sync_globals(void)
{
	memcpy(&psvr2_sync_g.rtt_max, SONY(0x180351638), 8);
	memcpy(&psvr2_sync_g.latency, SONY(0x180351640), 8);
	psvr2_sync_fit_enabled = G(0x180351658, uint8_t);
}

static int globals_equal(void)
{
	return !memcmp(&psvr2_sync_g.rtt_max, SONY(0x180351638), 8) &&
	       !memcmp(&psvr2_sync_g.latency, SONY(0x180351640), 8);
}

static int64_t srnd64(int bits) { return (int64_t)(((uint64_t)rnd() << 32 | rnd()) >> (64 - bits)) - (bits ? (1ll << (bits - 1)) : 0); }

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	((init_fn)SONY(0x1801d4320))();
	sinit_fn s_init = (sinit_fn)SONY(0x1801c1660);
	fit_fn s_fit = (fit_fn)SONY(0x1801c0b90);
	c2h_fn s_c2h = (c2h_fn)SONY(0x1801c1520);
	h2c_fn s_h2c = (h2c_fn)SONY(0x1801c09e0);
	cb_fn s_cb = (cb_fn)SONY(0x1801c08a0);
	cc_fn s_cc = (cc_fn)SONY(0x1801c08f0);
	cp_fn s_cp = (cp_fn)SONY(0x1801c0980);
	mode_fn s_mode = (mode_fn)SONY(0x1801c1820);
	cmd_fn s_cmd = (cmd_fn)SONY(0x1801c1880);
	t2u_fn s_t2u = (t2u_fn)SONY(0x1801c1500);
	static psvr2_sync_state a, b;
	int fails = 0, bad = 0, n = 0;

	/* Small helpers. */
	for (int i = 0; i < 200000; i++) {
		uint32_t t = rnd();
		bad += s_t2u(t) != psvr2_sync_ticks_to_us(t);
		int32_t h = (int32_t)(rnd() % 3 ? rnd() % 5000 : rnd()), v = (int32_t)rnd();
		if (rnd() % 2)
			v %= 4000;
		bad += s_cb(h, v) != psvr2_sync_clamp_base(h, v);
	}
	printf("ticks_to_us + clamp_base: %d/400000 mismatches\n", bad);
	fails += bad;

	bad = 0;
	for (int i = 0; i < 2000; i++) {
		s_init(&a, rnd() % 2, rnd() % 2);
		psvr2_sync_init(&b, a.bt, a.hand);
		bad += memcmp(&a, &b, sizeof a) != 0;
		for (int k = 0; k < 50; k++) {
			uint32_t ns = 15683350 + (int32_t)(rnd() % 4000000) - 2000000;
			if (rnd() % 8 == 0)
				ns = rnd();
			uint8_t p = (uint8_t)(rnd() % 3 ? rnd() % 80 : rnd());
			bad += s_cc(&a, ns) != psvr2_sync_clamp_cycle(&b, ns);
			bad += s_cp(&a, p) != psvr2_sync_clamp_period(&b, p);
		}
		bad += memcmp(&a, &b, sizeof a) != 0;
	}
	printf("init + clamp_cycle/period: %d/%d mismatches\n", bad, 2000 * 102);
	fails += bad;

	/* Clock conversions with random fit parameters. */
	bad = 0;
	for (int i = 0; i < 200000; i++) {
		G(0x180351658, uint8_t) = rnd() % 4 != 0;
		sync_globals();
		memset(&a, 0, sizeof a);
		a.simple_off = srnd64(rnd() % 2 ? 40 : 64);
		a.fit_off = rnd() % 16 ? srnd64(rnd() % 2 ? 40 : 64) : 0;
		a.drift_ppb = rnd() % 16 ? srnd64(rnd() % 2 ? 18 : 40) : 0;
		a.ref_ctrl_us = rnd() % 16 ? (uint64_t)rnd() : 0;
		if (a.drift_ppb == -1000000000 || a.drift_ppb == -1000000001)
			a.drift_ppb = 0;
		b = a;
		int64_t h1 = 0, h2 = 0;
		uint32_t t = rnd();
		s_c2h(&a, t, &h1);
		psvr2_sync_ctrl_to_host(&b, t, &h2);
		bad += h1 != h2;
		int64_t host = rnd() % 2 ? srnd64(64) : (int64_t)t + a.fit_off + (int32_t)rnd() % 100000;
		int32_t c1 = 0x5a5a, c2 = 0x5a5a;
		int32_t r1 = s_h2c(&a, host, &c1), r2 = psvr2_sync_host_to_ctrl(&b, host, &c2);
		bad += r1 != r2 || c1 != c2;
	}
	printf("ctrl<->host conversion: %d/200000 mismatches\n", bad);
	fails += bad;

	/* Clock fit over random report sequences. */
	bad = 0;
	n = 0;
	int cov_fit[2] = { 0, 0 }, cov_drift = 0, cov_reset = 0, cov_wrap = 0;
	uint32_t g0[4] = { 23000, 13000, 4300, 500 };
	for (int sc = 0; sc < 400; sc++) {
		memcpy(SONY(0x180351638), g0, 16);
		G(0x180351658, uint8_t) = rnd() % 4 != 0;
		sync_globals();
		const uint8_t bt = rnd() % 2;
		s_init(&a, bt, rnd() % 2);
		psvr2_sync_init(&b, a.bt, a.hand);
		uint64_t host = (uint64_t)rnd() << (rnd() % 2 ? 8 : 0);
		if (sc % 5 == 0)
			host = 0x100000000ull - 3000000 + rnd() % 2000000;
		uint64_t ctrl_ticks = rnd();
		uint32_t t2 = rnd();
		if (sc % 3 == 0) { /* start near the 32-bit tick wrap */
			ctrl_ticks = 0xffffffffu - rnd() % 6000000;
			t2 = (uint32_t)ctrl_ticks + rnd() % 3000;
		}
		const int64_t drift = (int32_t)(rnd() % 200001) - 100000; /* ticks per 1e9 */
		const int base_rtt = bt ? 3000 + rnd() % 15000 : 2000 + rnd() % 20000;
		const int steps = 300 + rnd() % 1500;
		for (int k = 0; k < steps; k++, n++) {
			const uint64_t dt = bt ? 4000 + rnd() % 300 : 1000 + rnd() % 4000;
			host += dt;
			ctrl_ticks += dt * 3 + (int64_t)dt * 3 * drift / 1000000000 + (rnd() % 7) - 3;
			t2 += (uint32_t)(dt * 3);
			uint32_t rtt = base_rtt + rnd() % 3000;
			if (rnd() % 50 == 0)
				rtt = rnd() % 40000;
			uint32_t echo = (uint32_t)(host - rtt);
			if (rnd() % 20 == 0)
				echo = a.last_echo;
			if (rnd() % 100 == 0)
				echo = 0;
			if (rnd() % 400 == 0)
				ctrl_ticks += (int64_t)(int32_t)rnd() / 64;
			if (rnd() % 300 == 0)
				a.mode_req = b.mode_req = rnd() % 2;
			if (rnd() % 500 == 0)
				a.force_reset = b.force_reset = 1;
			if (rnd() % 700 == 0)
				a.started = b.started = 1;
			const int64_t off0 = a.fit_off, dr0 = a.drift_ppb;
			const uint8_t rs0 = a.resets;
			cov_wrap += a.wrap == 0x11;
			s_fit(&a, host, echo, (uint32_t)ctrl_ticks, t2, bt);
			cov_fit[bt] += a.fit_off != off0 && a.resets == rs0;
			cov_drift += a.drift_ppb != dr0 && a.drift_ppb != 0;
			cov_reset += a.resets != rs0;
			psvr2_sync_update(&b, host, echo, (uint32_t)ctrl_ticks, t2, bt);
			if (memcmp(&a, &b, sizeof a) || !globals_equal()) {
				if (bad < 3) {
					for (size_t o = 0; o < sizeof a; o++)
						if (((uint8_t *)&a)[o] != ((uint8_t *)&b)[o]) {
							printf("fit sc %d step %d bt %d: first diff at 0x%zx\n", sc, k, bt, o);
							break;
						}
				}
				bad++;
				b = a;
				sync_globals();
			}
		}
	}
	printf("clock fit: %d/%d steps mismatched (coverage: fit usb %d bt %d, drift %d, resets %d, wraps %d)\n",
	       bad, n, cov_fit[0], cov_fit[1], cov_drift, cov_reset, cov_wrap);
	fails += bad;

	/* LED-sync mode + commands. */
	bad = 0;
	G(0x1805974d0, void *) = (void *)sony_cb;
	psvr2_sync_headset_time = our_cb;
	for (int i = 0; i < 300000; i++) {
		G(0x180351658, uint8_t) = rnd() % 4 != 0;
		sync_globals();
		s_init(&a, rnd() % 2, rnd() % 2);
		a.simple_off = srnd64(34);
		a.fit_off = rnd() % 8 ? srnd64(34) : 0;
		a.drift_ppb = rnd() % 8 ? srnd64(18) : 0;
		a.ref_ctrl_us = rnd() % 8 ? rnd() : 0;
		((uint8_t *)&a)[0x868] = rnd() % 2;
		b = a;
		psvr2_ledsync la, lb;
		for (size_t k = 0; k < sizeof la; k++)
			((uint8_t *)&la)[k] = (uint8_t)rnd();
		if (rnd() % 2)
			la.half = rnd() % 3000;
		if (rnd() % 2)
			la.cycle_ns = 16683350 + rnd() % 100000;
		lb = la;
		uint8_t cmd[16];
		for (int k = 0; k < 16; k++)
			cmd[k] = (uint8_t)rnd();
		cmd[0] = rnd() % 9;
		cmd[1] = rnd() % 8;
		if (rnd() % 2) {
			float f = ((int32_t)rnd() % 2000) / 1e5f;
			memcpy(cmd + 1, &f, 4);
		}
		static const uint8_t lens[] = { 2, 5, 7, 9, 11 };
		uint8_t len = rnd() % 8 ? lens[rnd() % 5] : (uint8_t)rnd();
		g_time = rnd() % 8 ? (uint64_t)rnd() * (rnd() % 1000 + 1) : 0;
		if (cmd[0] == 7 && cmd[1] == 3 && !(a.stats[0] == 0 && cmd[2]))
			continue; /* stop path writes a log file */
		if (i % 3 == 0) {
			uint8_t m = rnd() % 4;
			s_mode(&la, m);
			psvr2_ledsync_set_mode(&lb, m);
		} else {
			s_cmd(&a, &la, cmd, len);
			psvr2_ledsync_command(&b, &lb, cmd, len);
		}
		if (memcmp(&la, &lb, sizeof la) || memcmp(&a, &b, sizeof a)) {
			if (bad < 5)
				printf("ledsync cmd %d sub %d len %d\n", cmd[0], cmd[1], len);
			bad++;
		}
	}
	printf("ledsync mode/command: %d/300000 mismatches\n", bad);
	fails += bad;
	return fails != 0;
}
