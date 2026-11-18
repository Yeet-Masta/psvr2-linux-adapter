/* HmdDevice::Activate (0x18019d1b0): the property writes, in order. */
#include "t_steamvr_host.h"
#include "psvr2_steamvr.h"
#include <math.h>

static uint32_t g_sw_version;
static uint8_t g_ipd_mm;
static int64_t fake_hmd_info(void *m, uint8_t *o) { (void)m; memset(o, 0, 0x48); memcpy(o, &g_sw_version, 4); return 0; }
static int64_t fake_hmd_status(void *m, uint8_t *o) { (void)m; for (int k = 0; k < 0x20; k++) o[k] = (uint8_t)rnd(); o[5] = g_ipd_mm; return 0; }
static uint8_t g_mgr[0x100];
static void *fake_mgr(void) { return g_mgr; }

static float rf(float lo, float hi) { return lo + (hi - lo) * (float)(rnd() & 0xffffff) / 16777216.0f; }

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	host_install();
	hook(0x180124a00, (void *)fake_hmd_info);
	hook(0x1801251a0, (void *)fake_hmd_status);
	hook(0x18011f300, (void *)fake_mgr);
	hook(0x18011fc50, (void *)stub_ret0);   /* registers the HMD with the map manager */
	hook(0x1801a1a00, (void *)stub_ret0);   /* lens geometry: t_steamvr_lens */
	hook(0x1801a10c0, (void *)stub_ret0);   /* hidden-area mesh: t_steamvr_lens */
	hook(0x18016b660, (void *)stub_ret0);   /* thread start */
	int64_t (*activate)(void *, uint32_t) = (int64_t (*)(void *, uint32_t))SONY(0x18019d1b0);
	static uint8_t dev[0x400], server[0x200];
	static char path[3000];
	int bad = 0, n = 20000;

	for (int it = 0; it < n; it++) {
		psvr2_hmd_prop_in in = { 0 };
		const int m = rnd() % 4;
		in.edid_product_id = m == 0 ? rnd() : m == 1 ? 0x80000000u : m == 2 ? 0 : rnd() & 0xffff;
		in.vsync_to_photons_us = rnd() % 3 ? 0 : (int32_t)(rnd() % 4000) - 1000;
		in.photons_to_vblank_us = rnd() % 3 ? 0 : (int32_t)(rnd() % 20000) - 1000;
		in.hmd_sw_version = rnd() % 2 ? 0x06000000u + (rnd() % 3) - 1 : rnd();
		in.ipd_mm = (uint8_t)rnd();
		const int r = rnd() % 5;
		in.refresh_rate = r == 0 ? 90.0f : r == 1 ? 120.0f : r == 2 ? rf(89.8f, 90.2f) : r == 3 ? rf(0, 200) : NAN;
		in.ipd_setting = rf(0.05f, 0.08f);
		const int pm = rnd() % 4;
		if (pm == 0) {
			in.user_config_path = NULL;
		} else {
			int len = pm == 1 ? 1500 + rnd() % 1000 : rnd() % 60;  /* > 1024 takes Sony's re-read path */
			for (int k = 0; k < len; k++)
				path[k] = 'a' + rnd() % 26;
			path[len] = 0;
			in.user_config_path = path;
		}
		g_user_config_path = in.user_config_path;
		g_sw_version = in.hmd_sw_version;
		g_ipd_mm = in.ipd_mm;
		g_set_float[1] = in.ipd_setting;
		memset(dev, 0, sizeof dev);
		memset(server, 0, sizeof server);
		*(void **)(dev + 0x18) = server;
		memcpy(server + 0x54, &in.edid_product_id, 4);
		memcpy(server + 0x58, &in.vsync_to_photons_us, 4);
		memcpy(server + 0x5c, &in.photons_to_vblank_us, 4);
		memcpy(server + 0x88, &in.refresh_rate, 4);
		const uint32_t id = rnd() % 16;
		g_nprops = 0;
		g_ncomps = 0;
		activate(dev, id);

		psvr2_prop_list l;
		psvr2_steamvr_hmd_props(&in, &l);
		int ok = g_nprops == l.n && g_ncomps == 1 && !strcmp(g_comps[0].path, "/proximity") &&
			 *(uint32_t *)(dev + 0x10) == id;
		for (int k = 0; ok && k < l.n; k++) {
			const prop_rec *a = &g_props[k];
			const psvr2_prop *b = &l.p[k];
			if (a->container != 0x1000 + id || a->write_type != 0 || a->prop != b->prop ||
			    a->tag != b->tag || a->size != b->size || memcmp(a->data, b->data, b->size > 4096 ? 4096 : b->size)) {
				ok = 0;
				if (bad < 5)
					printf("prop #%d: ref %u tag %u size %u %08x / ours %u tag %u size %u %08x\n", k,
					       a->prop, a->tag, a->size, *(uint32_t *)a->data, b->prop, b->tag, b->size,
					       *(const uint32_t *)b->data);
			}
		}
		if (!ok && bad++ < 5)
			printf("  it %d: nprops %d/%d ncomps %d\n", it, g_nprops, l.n, g_ncomps);
	}
	printf("hmd activate properties: %d/%d mismatches\n", bad, n);
	return bad != 0;
}
