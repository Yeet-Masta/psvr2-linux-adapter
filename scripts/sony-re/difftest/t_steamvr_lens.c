/* Lens geometry (FUN_1801a1a00) and hidden-area mesh (FUN_1801a10c0). */
#include "t_steamvr_host.h"
#include "psvr2_steamvr.h"
#include <math.h>

static uint8_t g_calib[256];
static int64_t __cdecl fake_get_calib(void *mgr, uint8_t *out) { (void)mgr; memcpy(out, g_calib, 256); return 0; }

static float rf(float lo, float hi) { return lo + (hi - lo) * (float)(rnd() & 0xffffff) / 16777216.0f; }

static void put_f(uint8_t *c, int off, float f) { memcpy(c + off, &f, 4); }

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	host_install();
	hook(0x180124e30, (void *)fake_get_calib);
	void (*ref_geom)(void *) = (void (*)(void *))SONY(0x1801a1a00);
	void (*ref_mesh)(void *) = (void (*)(void *))SONY(0x1801a10c0);
	uint8_t *ref_cache = SONY(0x180350fc0);
	uint8_t *ref_vbuf = SONY(0x1803631c0);
	const float *lut = SONY(0x180262180);
	static uint8_t obj[0x400];
	int max_ulp = 0, ulp1 = 0;
	int bad_geom = 0, bad_mesh = 0, n = 20000, nmesh = 0;
	static float v[PSVR2_HIDDEN_AREA_MAX][2];

	for (int it = 0; it < n; it++) {
		const int mode = rnd() % 8;
		for (int k = 0; k < 256; k++)
			g_calib[k] = (uint8_t)rnd();
		g_calib[0] = (uint8_t)(rnd() % 8);
		if (mode < 6) {
			/* plausible: small pixel offsets and angles */
			put_f(g_calib, 0x08, rf(-60, 60));
			put_f(g_calib, 0x0c, rf(-60, 60));
			put_f(g_calib, 0x10, rf(-60, 60));
			put_f(g_calib, 0x14, rf(-60, 60));
			put_f(g_calib, 0x18, rf(-5, 5));
			put_f(g_calib, 0x1c, rf(-5, 5));
			put_f(g_calib, 0x20, rf(-200, 200));
			if (mode == 5)
				put_f(g_calib, 0x0c, 0.0f);
		}
		psvr2_lens_state st;
		if (rnd() % 16 == 0)
			memcpy(ref_cache, g_calib, 256);    /* unchanged block: no update */
		else
			for (int k = 0; k < 256; k++)
				ref_cache[k] = (uint8_t)rnd();
		memcpy(st.calib, ref_cache, 256);
		for (int k = 0; k < 0x400; k++)
			obj[k] = (uint8_t)rnd();
		memcpy(&st.geom, obj + 0x260, sizeof st.geom);

		ref_geom(obj);
		psvr2_steamvr_lens_update(&st, g_calib);
		/* Centres must match bit for bit; rotations go through cosf/sinf,
		 * whose CRT implementations may differ by an ulp. */
		int geom_bad = memcmp(obj + 0x260, &st.geom, 16) || memcmp(ref_cache, st.calib, 256);
		for (int k = 4; k < 8; k++) {
			int32_t a, b;
			memcpy(&a, obj + 0x260 + 4 * k, 4);
			memcpy(&b, (uint8_t *)&st.geom + 4 * k, 4);
			int64_t d = (int64_t)a - b;
			if (d < 0) d = -d;
			if (d > max_ulp && d < 1000) max_ulp = (int)d;
			if (d > 1) geom_bad = 1;
			if (d == 1) ulp1++;
		}
		if (geom_bad) {
			if (bad_geom++ < 5) {
				const float *a = (const float *)(obj + 0x260), *b = (const float *)&st.geom;
				printf("geom mismatch v%d:", g_calib[0]);
				for (int k = 0; k < 8; k++)
					printf(" %08x/%08x", ((uint32_t *)a)[k], ((uint32_t *)b)[k]);
				printf("\n");
			}
			continue;
		}
		/* NaN geometry makes the ray march spin forever (in Sony's code
		 * too), so only plausible geometry goes on to the mesh. */
		int finite = mode < 6 && g_calib[0] >= 4;
		for (int k = 0; k < 8; k++)
			finite = finite && isfinite(((float *)&st.geom)[k]);
		if (it % 4 || !finite)
			continue;
		/* Hidden area from the geometry just computed. */
		nmesh++;
		for (int k = 0; k < PSVR2_HIDDEN_AREA_MAX * 8; k++)
			ref_vbuf[k] = (uint8_t)rnd();
		memcpy(v, ref_vbuf, sizeof v);
		g_nprops = 0;
		ref_mesh(obj);
		const int nv = psvr2_steamvr_hidden_area(&st.geom, lut, v);
		int ok = g_nprops == 2 && g_props[0].prop == 5100 && g_props[1].prop == 5101 &&
			 g_props[0].tag == 30 && g_props[1].tag == 30 &&
			 g_props[0].size == (uint32_t)nv * 8 && g_props[1].size == (uint32_t)nv * 8 &&
			 g_props[0].container == 0x1000;
		ok = ok && !memcmp(g_props[0].data, v, nv * 8);
		psvr2_steamvr_hidden_area_mirror(v, nv);
		ok = ok && !memcmp(g_props[1].data, v, nv * 8);
		if (!ok && bad_mesh++ < 5) {
			printf("mesh mismatch: nprops %d size %u/%d\n", g_nprops, g_props[0].size, nv * 8);
			for (int k = 0; k < nv && k < 108; k++) {
				const float *a = (const float *)g_props[0].data + 2 * k;
				if (memcmp(a, v[k], 8))
					printf("  v%d ref %a %a\n", k, a[0], a[1]);
			}
		}
	}
	printf("lens geometry: %d/%d mismatches (rotation: %d values 1 ulp off, max %d ulp); hidden-area mesh: %d/%d mismatches\n",
	       bad_geom, n, ulp1, max_ulp, bad_mesh, nmesh);
	return bad_geom || bad_mesh;
}
