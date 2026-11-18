/* IVRDisplayComponent methods of sie::psvr2::HmdDevice (component at +8):
 * GetWindowBounds 0x18019edf0, GetRecommendedRenderTargetSize 0x18019edb0,
 * GetEyeOutputViewport 0x18019eca0, GetProjectionRaw 0x18019ed60,
 * ComputeDistortion 0x18019e860. */
#include "t_steamvr_host.h"
#include "psvr2_steamvr.h"
#include <math.h>

typedef void (*bounds_fn)(void *, uint32_t *, uint32_t *, uint32_t *, uint32_t *);
typedef void (*rt_fn)(void *, uint32_t *, uint32_t *);
typedef void (*vp_fn)(void *, int, uint32_t *, uint32_t *, uint32_t *, uint32_t *);
typedef void (*proj_fn)(void *, int, float *, float *, float *, float *);
typedef float *(*dist_fn)(void *, float *, int, float, float);

static float rf(float lo, float hi) { return lo + (hi - lo) * (float)(rnd() & 0xffffff) / 16777216.0f; }

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	bounds_fn bounds = (bounds_fn)SONY(0x18019edf0);
	rt_fn rt = (rt_fn)SONY(0x18019edb0);
	vp_fn vp = (vp_fn)SONY(0x18019eca0);
	proj_fn proj = (proj_fn)SONY(0x18019ed60);
	dist_fn dist = (dist_fn)SONY(0x18019e860);
	const float *lut = SONY(0x180261180);
	static uint8_t dev[0x400];
	uint8_t *disp = dev + 8;
	int bad = 0, nan_sign = 0, n = 300000;

	for (int it = 0; it < n; it++) {
		uint32_t pw = it == 0 ? PSVR2_PANEL_W : rnd(), ph = it == 0 ? PSVR2_PANEL_H : rnd();
		if (it % 3 == 0) { pw = PSVR2_PANEL_W; ph = PSVR2_PANEL_H; }
		memcpy(dev + 0x258, &pw, 4);
		memcpy(dev + 0x25c, &ph, 4);
		uint32_t a[4], b[4];
		bounds(disp, &a[0], &a[1], &a[2], &a[3]);
		psvr2_rect r = psvr2_steamvr_window_bounds(pw, ph);
		if (memcmp(a, &r, 16)) bad++;
		rt(disp, &a[0], &a[1]);
		psvr2_steamvr_render_target(pw, ph, &b[0], &b[1]);
		if (memcmp(a, b, 8)) bad++;
		const int eye = rnd() & 1;
		vp(disp, eye, &a[0], &a[1], &a[2], &a[3]);
		r = psvr2_steamvr_eye_viewport(pw, ph, eye);
		if (memcmp(a, &r, 16)) bad++;
		float fa[4], fb[4];
		proj(disp, eye, &fa[0], &fa[1], &fa[2], &fa[3]);
		psvr2_steamvr_projection_raw(eye, &fb[0], &fb[1], &fb[2], &fb[3]);
		if (memcmp(fa, fb, 16)) bad++;

		/* Distortion: plausible geometry, u/v in and around 0..1. */
		psvr2_lens_geom g;
		g.centre_l[0] = rf(-0.15f, -0.05f); g.centre_l[1] = rf(-0.05f, 0.05f);
		g.centre_r[0] = rf(0.05f, 0.15f);   g.centre_r[1] = rf(-0.05f, 0.05f);
		float al = rf(-0.1f, 0.1f), ar = rf(-0.1f, 0.1f);
		g.rot_l[0] = cosf(al); g.rot_l[1] = sinf(al);
		g.rot_r[0] = cosf(ar); g.rot_r[1] = sinf(ar);
		if (it % 50 == 0)
			for (int k = 0; k < 8; k++)
				((uint32_t *)&g)[k] = rnd();   /* any bits, incl. NaN */
		memcpy(dev + 0x260, &g, sizeof g);
		float u = rf(-0.2f, 1.2f), v = rf(-0.2f, 1.2f);
		float ref[6];
		dist(disp, ref, eye, u, v);
		psvr2_distortion d = psvr2_steamvr_distort(&g, lut, lut + 1024, lut + 2048, eye, u, v);
		if (memcmp(ref, &d, 24)) {
			/* Both NaN in the same places: only the NaN sign differs
			 * (operand order of commutative ops), from NaN geometry. */
			int same_nan = 1;
			for (int k = 0; k < 6; k++) {
				float o = ((float *)&d)[k];
				if (isnan(ref[k]) != isnan(o) || (!isnan(o) && memcmp(&ref[k], &o, 4)))
					same_nan = 0;
			}
			if (same_nan) { nan_sign++; continue; }
			if (bad++ < 5)
				printf("distort eye %d u %a v %a: ref %a %a %a / ours %a %a %a\n", eye, u, v,
				       ref[0], ref[1], ref[4], d.rgb[0][0], d.rgb[0][1], d.rgb[2][0]);
		}
	}
	printf("display component: %d/%d mismatches (%d NaN-sign-only, NaN geometry)\n", bad, n, nan_sign);
	return bad != 0;
}
