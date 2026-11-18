/* ServerDriver refresh-rate / EDID choice (0x1801a4390). */
#include "t_steamvr_host.h"
#include "psvr2_steamvr.h"
#include <math.h>

static uint8_t g_cur;
static int g_set;
static int64_t fake_get_edid(void *m, uint8_t *o) { (void)m; *o = g_cur; return 0; }
static int64_t fake_set_edid(void *m, uint8_t v) { (void)m; g_set = v; return 0; }
static void WINAPI fake_sleep(DWORD ms) { (void)ms; }

static float rf(float lo, float hi) { return lo + (hi - lo) * (float)(rnd() & 0xffffff) / 16777216.0f; }

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	host_install();
	hook(0x180124970, (void *)fake_get_edid);
	hook(0x180127e80, (void *)fake_set_edid);
	hook(0x1801a5310, (void *)stub_ret0);       /* direct_mode.enable from steamvr.vrsettings */
	{
		/* Sony sleeps 1 s after changing the EDID; patch kernel32!Sleep. */
		uint8_t *p = (uint8_t *)GetProcAddress(GetModuleHandleA("kernel32.dll"), "Sleep");
		void *fn = (void *)fake_sleep;
		DWORD old;
		VirtualProtect(p, 12, PAGE_EXECUTE_READWRITE, &old);
		p[0] = 0x48; p[1] = 0xb8; memcpy(p + 2, &fn, 8); p[10] = 0xff; p[11] = 0xe0;
		VirtualProtect(p, 12, old, &old);
	}
	void (*ref)(void *, const char *, uint8_t *) = (void (*)(void *, const char *, uint8_t *))SONY(0x1801a4390);
	static uint8_t server[0x200];
	int bad = 0, n = 200000;

	for (int it = 0; it < n; it++) {
		const int m = rnd() % 8;
		float pref = m == 0 ? 90.0f : m == 1 ? 120.0f : m == 2 ? rf(89.8f, 90.2f) : m == 3 ? rf(119.8f, 120.2f) :
			     m == 4 ? rf(0, 200) : m == 5 ? NAN : m == 6 ? rf(89.89f, 89.91f) : 72.0f;
		if (m == 7) { uint32_t b = rnd(); memcpy(&pref, &b, 4); }
		g_cur = (uint8_t)(rnd() % 4 == 0 ? rnd() : rnd() % 2);
		g_set = -1;
		g_set_float[0] = pref;
		g_set_float_writes = 0;
		memset(server, 0, sizeof server);
		uint8_t changed = 0x55;
		ref(server, "test", &changed);
		float rate;
		memcpy(&rate, server + 0x88, 4);

		psvr2_edid_choice c = psvr2_steamvr_edid_select(pref, g_cur);
		const int ok = !memcmp(&rate, &c.rate, 4) && (g_set_float_writes != 0) == c.store_setting &&
			       (!c.store_setting || g_set_float_written == 90.0f) && g_set == c.set_edid &&
			       changed == (c.set_edid >= 0);
		if (!ok && bad++ < 8)
			printf("pref %a cur %u: ref rate %a store %d set %d chg %u / ours %a %d %d\n", pref, g_cur,
			       rate, g_set_float_writes, g_set, changed, c.rate, c.store_setting, c.set_edid);
	}
	printf("edid select: %d/%d mismatches\n", bad, n);
	return bad != 0;
}
