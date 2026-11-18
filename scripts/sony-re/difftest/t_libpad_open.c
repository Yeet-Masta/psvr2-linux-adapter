#include "sony.h"
#include "psvr2_libpad.h"
#include <windows.h>

typedef uint32_t (*open_fn)(psvr2_pad *);

/* Sony's report writer and post-send hook are redirected here. */
static uint8_t cap[4][0x30];
static uint32_t cap_len[4];
static int ncap, nposts;
static uint32_t __attribute__((ms_abi)) fake_send(int32_t h, const uint8_t *buf, uint32_t len)
{
	(void)h;
	if (ncap < 4) {
		memset(cap[ncap], 0, 0x30);
		memcpy(cap[ncap], buf, len > 0x30 ? 0x30 : len);
		cap_len[ncap] = len;
	}
	ncap++;
	return 0;
}
static uint32_t __attribute__((ms_abi)) fake_post(int32_t h) { (void)h; nposts++; return 0; }

static void hook(uint64_t va, void *fn)
{
	uint8_t *p = (uint8_t *)SONY(va);
	DWORD old;
	VirtualProtect(p, 12, PAGE_EXECUTE_READWRITE, &old);
	p[0] = 0x48;
	p[1] = 0xb8;	/* mov rax, imm64 */
	memcpy(p + 2, &fn, 8);
	p[10] = 0xff;
	p[11] = 0xe0;	/* jmp rax */
	VirtualProtect(p, 12, old, &old);
	FlushInstructionCache(GetCurrentProcess(), p, 12);
}

static void rand_effects(uint8_t *blk)
{
	static const uint8_t modes[6] = { 0x05, 0x21, 0x22, 0x25, 0x26, 0x00 };
	blk[0] = rnd() % 4;
	for (int s = 0; s < 2; s++) {
		uint8_t *e = blk + 1 + 0xb * s;
		e[0] = modes[rnd() % 6];
		for (int i = 1; i < 11; i++)
			e[i] = (uint8_t)rnd();
		if (!(e[1] | (e[2] & 3)))
			e[1] = 1 << (rnd() % 8);
	}
}

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	open_fn s_open = (open_fn)SONY(0x1801c8570);
	hook(0x1801cba20, (void *)fake_send);
	hook(0x1801d2ef0, (void *)fake_post);
	static psvr2_pad a, b;
	static const uint16_t pids[8] = { 0x0ce6, 0x0df2, 0x0ce7, 0x0e45, 0x0e46, 0x05c4, 0x09cc, 0x0ba0 };
	int bad = 0, n = 0, cov[3] = { 0 };
	for (int k = 0; k < 200000; k++, n++) {
		uint8_t *ap = (uint8_t *)&a;
		for (size_t i = 0; i < 0x70; i++)
			ap[i] = (uint8_t)rnd();
		a.in_use = rnd() % 5 ? 1 : (uint8_t)rnd();
		a.vid = rnd() % 10 ? 0x054c : (uint16_t)rnd();
		a.pid = pids[rnd() % 8];
		a.hid_ver = rnd() % 2 ? 0x10 + rnd() % 16 : (int32_t)rnd();
		a.fw = rnd() % 2 ? 0x200 + rnd() % 0x100 : (uint16_t)rnd();
		a.player = rnd() % 7 - 1;
		a.link = rnd() % 3;
		a.mic_led = rnd() % 6;
		a.hand = rnd() % 3;
		for (int i = 0; i < 3; i++) {
			a.ds_rgb[i] = (uint8_t)rnd();
			a.ds4_rgb[i] = (uint8_t)rnd();
		}
		rand_effects(&a.eff_mask);
		const psvr2_pad_switches sw = { rnd() % 2, rnd() % 2, rnd() % 2, rnd() % 2, (uint8_t)rnd() };
		*(uint8_t *)SONY(0x1805b1573) = sw.haptics_b;
		*(uint8_t *)SONY(0x1805e349c) = sw.flag02;
		*(uint8_t *)SONY(0x1805b157b) = sw.flag10;
		*(uint8_t *)SONY(0x1805e349d) = sw.state_en;
		*(uint32_t *)SONY(0x1805e3498) = sw.state_val | (rnd() & 0xffffff00);
		b = a;
		ncap = nposts = 0;
		s_open(&a);
		uint8_t rep[2][0x30], len[2];
		const int nr = psvr2_pad_open_reports(&b, &sw, rep, len);
		int ok = nr == ncap && !memcmp(&a, &b, sizeof a);
		for (int i = 0; ok && i < nr; i++)
			ok = len[i] == cap_len[i] && !memcmp(rep[i], cap[i], len[i]);
		cov[nr]++;
		if (!ok) {
			if (bad < 5)
				printf("open k %d pid %04x: reports %d/%d pad %s\n", k, a.pid, ncap, nr,
				       memcmp(&a, &b, sizeof a) ? "differs" : "same");
			bad++;
		}
	}
	printf("pad open reports: %d/%d mismatches (0/1/2 reports: %d %d %d)\n", bad, n, cov[0], cov[1], cov[2]);
	return bad != 0;
}
