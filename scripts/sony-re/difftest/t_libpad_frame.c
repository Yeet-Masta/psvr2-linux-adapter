#include "sony.h"
#include "psvr2_libpad.h"
#include <windows.h>

typedef uint32_t (*write_fn)(uint32_t, uint8_t *, uint16_t);

/* Stubs for the HID and file calls of Sony's writer (patched into its IAT). */
static uint16_t g_report_len;
static uint8_t g_wr[0x200];
static uint32_t g_wr_n;
static int g_writes;
static BOOLEAN __attribute__((ms_abi)) st_getpp(HANDLE h, void **pp) { (void)h; *pp = (void *)1; return 1; }
static LONG __attribute__((ms_abi)) st_caps(void *pp, uint8_t *caps)
{
	(void)pp;
	memset(caps, 0, 64);
	memcpy(caps + 6, &g_report_len, 2);
	return 0x110000;
}
static BOOLEAN __attribute__((ms_abi)) st_freepp(void *pp) { (void)pp; return 1; }
static HANDLE __attribute__((ms_abi)) st_event(void *a, BOOL b, BOOL c, const void *d) { (void)a; (void)b; (void)c; (void)d; return (HANDLE)0x1234; }
static BOOL __attribute__((ms_abi)) st_close(HANDLE h) { (void)h; return 1; }
static BOOL __attribute__((ms_abi)) st_write(HANDLE h, const void *buf, DWORD n, DWORD *done, void *ov)
{
	(void)h; (void)ov;
	memcpy(g_wr, buf, n > sizeof g_wr ? sizeof g_wr : n);
	g_wr_n = n;
	g_writes++;
	if (done)
		*done = n;
	return 1;
}

static void iat(uint64_t va, void *fn)
{
	void **slot = (void **)SONY(va);
	DWORD old;
	VirtualProtect(slot, 8, PAGE_READWRITE, &old);
	*slot = fn;
	VirtualProtect(slot, 8, old, &old);
}

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	((void (*)(void))SONY(0x1801d4320))();
	write_fn s_write = (write_fn)SONY(0x1801cba20);
	iat(0x180247090, (void *)st_getpp);
	iat(0x180247070, (void *)st_caps);
	iat(0x180247098, (void *)st_freepp);
	iat(0x1802471a0, (void *)st_event);
	iat(0x1802473a0, (void *)st_close);
	iat(0x180247160, (void *)st_write);

	uint8_t *dev = (uint8_t *)SONY(0x1805b1580);
	psvr2_pad *pad = (psvr2_pad *)SONY(0x1805974e0);
	static psvr2_sync_state mine;
	static const uint16_t pids[6] = { 0x0ce6, 0x0df2, 0x0e45, 0x0e46, 0x05c4, 0x09cc };
	static const uint16_t lens[8] = { 0x20, 0x30, 0x40, 0x4e, 0x4f, 0x60, 0x78, 0x2f };
	int bad = 0, n = 0, cov_bt = 0, cov_clamp = 0;
	for (int k = 0; k < 200000; k++, n++) {
		memset(dev, 0, 0x40);
		*(uint64_t *)dev = 1;
		*(uint32_t *)(dev + 8) = 77;
		*(uint64_t *)(dev + 0x10) = 0x99;
		*(uint16_t *)(dev + 0x20) = 0x054c;
		const uint16_t pid = pids[rnd() % 6];
		*(uint16_t *)(dev + 0x22) = pid;
		memset(pad, 0, sizeof *pad);
		pad->in_use = 1;
		pad->handle = 77;
		uint8_t *sp = (uint8_t *)&pad->sync;
		for (size_t i = 0x950; i < sizeof pad->sync; i++)
			sp[i] = (uint8_t)rnd();
		mine = pad->sync;
		g_report_len = rnd() % 4 ? lens[rnd() % 8] : (uint16_t)(1 + rnd() % 0x100);
		const uint16_t len = rnd() % 8 ? (uint16_t)(1 + rnd() % g_report_len) : (uint16_t)(rnd() % 0x110);
		uint8_t ba[0x120], bb[0x120], dst[0x200];
		for (size_t i = 0; i < sizeof ba; i++)
			ba[i] = (uint8_t)rnd();
		if (rnd() % 2) {	/* realistic LED-sync fields */
			ba[0x16] = rnd() % 0x50;
			const int32_t c3 = (int32_t)(rnd() % 20000000) * 3;
			memcpy(ba + 0x1b, &c3, 4);
		}
		memcpy(bb, ba, sizeof ba);
		g_writes = 0;
		const int32_t ra = (int32_t)s_write(77, ba, len);
		const int kind = (pid == 0x0ce6 || pid == 0x0df2) ? PSVR2_DEV_DUALSENSE
				 : (pid == 0x0e45 || pid == 0x0e46) ? PSVR2_DEV_SENSE : PSVR2_DEV_DS4;
		if (kind == PSVR2_DEV_SENSE && g_report_len >= len) {
			memcpy(bb + 0x10, ba + 0x10, 4);	/* host time stamp */
			psvr2_sense_output_clamp(bb, &mine);
			cov_clamp += memcmp(&mine, &pad->sync, sizeof mine) != 0 || ba[0x16] == 0x3c;
		}
		uint32_t tw = 0;
		const int32_t rb = psvr2_pad_frame_output(kind, pid == 0x0df2, g_report_len, bb, len, dst, &tw);
		cov_bt += g_report_len > (pid == 0x0df2 ? 0x40 : 0x30);
		int ok = ra == rb && !memcmp(ba, bb, sizeof ba) && !memcmp(&mine, &pad->sync, sizeof mine);
		if (ok && rb >= 0)
			ok = g_writes == 1 && g_wr_n == tw && !memcmp(g_wr, dst, tw);
		else if (ok)
			ok = g_writes == 0;
		if (!ok) {
			if (bad < 6)
				printf("frame k %d pid %04x L %#x len %#x: ret %x/%x writes %d n %u/%u\n", k, pid,
				       g_report_len, len, ra, rb, g_writes, g_wr_n, tw);
			bad++;
			pad->sync = mine;
		}
	}
	printf("output framing: %d/%d mismatches (bt-sized %d, clamp hits %d)\n", bad, n, cov_bt, cov_clamp);
	return bad != 0;
}
