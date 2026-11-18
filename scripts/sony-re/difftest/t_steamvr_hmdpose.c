/* HMD update pass (0x18019ee30): the HMD DriverPose_t and its state. */
#include "t_steamvr_host.h"
#include "psvr2_steamvr.h"
#include <math.h>

static psvr2_tracker_pose g_trk;
static uint8_t g_status[0x20], g_cfg[0x800], g_fake[0x1000];
static int g_pause, g_ready, g_ti;
static int64_t g_times[8];
static void *fake_obj(void) { return g_fake; }
static void *fake_cfg(void) { return g_cfg; }
static int64_t fake_status(void *m, uint8_t *o) { (void)m; memcpy(o, g_status, 0x20); return 0; }
static int64_t fake_trk(void *t, void *out, int idx, int64_t ts) { (void)t; (void)idx; (void)ts; memcpy(out, &g_trk, 0x50); return 0; }
static int64_t fake_time(void) { return g_times[g_ti++ & 7]; }
static int64_t f_pause(void) { return g_pause; }
static int64_t f_ready(void) { return g_ready; }
static int64_t ret_m1(void) { return -1; }
static int64_t com_fail(void) { return -1; }
static void *g_com_vt[8] = { 0, 0, 0, (void *)com_fail };
static struct { void **vt; } g_com = { g_com_vt };

static float rf(float lo, float hi) { return lo + (hi - lo) * (float)(rnd() & 0xffffff) / 16777216.0f; }

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	host_install();
	hook(0x1801251a0, (void *)fake_status);
	hook(0x180130970, (void *)fake_obj);
	hook(0x180131af0, (void *)f_pause);
	hook(0x180125480, (void *)stub_ret0);
	hook(0x18016b110, (void *)fake_time);
	hook(0x18016b180, (void *)stub_ret0);
	hook(0x1801253c0, (void *)f_ready);
	hook(0x180161140, (void *)fake_obj);
	hook(0x180160230, (void *)fake_trk);
	hook(0x180156470, (void *)fake_cfg);
	hook(0x18015bbd0, (void *)fake_obj);
	hook(0x18015a250, (void *)ret_m1);
	hook(0x180124a00, (void *)stub_ret0);
	hook(0x180125080, (void *)stub_ret0);
	hook(0x180132330, (void *)stub_ret0);
	void (*upd)(void *) = (void (*)(void *))SONY(0x18019ee30);
	static uint8_t dev[0x400], server[0x100];
	psvr2_hmd_pose_state st;
	int bad = 0, n = 100000;

	for (int it = 0; it < n; it++) {
		if (it % 50 == 0) {
			for (int k = 0; k < (int)sizeof dev; k++)
				dev[k] = (uint8_t)rnd();
			memcpy(&st.prev, dev + 0x28, sizeof st.prev);
			memcpy(&st.cache, dev + 0x140, sizeof st.cache);
			memcpy(&st.last_sent, dev + 0x2a8, sizeof st.last_sent);
			memcpy(&st.last_fresh_us, dev + 0x3c0, 8);
		}
		*(uint32_t *)(dev + 0x10) = rnd() % 4;
		*(void **)(dev + 0x18) = server;
		*(void **)(dev + 0x290) = &g_com;
		dev[0x288] = 0;
		dev[0x298] = dev[0x299] = 0;
		const int64_t now = 1000000000ll + it * 3000ll;
		memcpy(dev + 0x2a0, &now, 8);              /* audio scan not due */
		const int needs_update = rnd() % 8 == 0;
		dev[0x289] = (uint8_t)needs_update;
		/* small motion sometimes, so the not-worn throttle triggers */
		const int still = rnd() % 2;
		for (int k = 0; k < 3; k++) {
			g_trk.pos[k] = still ? (float)st.last_sent.pos[k] + rf(-0.004f, 0.004f) : rf(-2, 2);
			g_trk.vel[k] = rf(-1, 1);
			g_trk.ang_vel[k] = still ? rf(-0.02f, 0.02f) : rf(-3, 3);
		}
		for (int k = 0; k < 4; k++)
			g_trk.q[k] = rf(-1, 1);
		g_trk.valid = (uint8_t)(rnd() % 4 != 0);
		for (int k = 0; k < 7; k++)
			((float *)(g_cfg + 0x694))[k] = rf(-1, 1);
		memset(g_status, 0, sizeof g_status);
		g_status[1] = (uint8_t)(rnd() % 2);
		g_pause = rnd() % 6 == 0;
		g_ready = rnd() % 6 ? (int)(rnd() % 3 + 1) : 0;
		g_ti = 0;
		for (int k = 0; k < 8; k++)
			g_times[k] = now + k * 250 + (rnd() % 2 ? 0 : (int64_t)(rnd() % 700000));
		g_npose = 0;
		upd(dev);

		psvr2_hmd_pose_in in = { (uint8_t)g_ready, g_pause, needs_update, g_status[1],
					 g_times[1], g_times[3], &g_trk,
					 (const float *)(g_cfg + 0x6a0), (const float *)(g_cfg + 0x694) };
		psvr2_driver_pose o;
		const int sent = psvr2_steamvr_hmd_pose(&st, &in, &o);
		const int ok = g_npose == sent && (!sent || !memcmp(g_pose, &o, sizeof o)) &&
			       !memcmp(dev + 0x28, &st.prev, sizeof o) && !memcmp(dev + 0x140, &st.cache, sizeof o) &&
			       !memcmp(dev + 0x2a8, &st.last_sent, sizeof o) && !memcmp(dev + 0x3c0, &st.last_fresh_us, 8);
		if (!ok) {
			if (bad++ < 4) {
				printf("it %d ready %d pause %d worn %d still %d npose %d: sent %d prev %d cache %d last %d t %d\n",
				       it, g_ready, g_pause, g_status[1], still, g_npose, memcmp(g_pose, &o, sizeof o) != 0,
				       memcmp(dev + 0x28, &st.prev, sizeof o) != 0, memcmp(dev + 0x140, &st.cache, sizeof o) != 0,
				       memcmp(dev + 0x2a8, &st.last_sent, sizeof o) != 0, memcmp(dev + 0x3c0, &st.last_fresh_us, 8) != 0);
				for (int k = 0; k < 35; k++) {
					double a, b;
					memcpy(&a, g_pose + 8 * k, 8);
					memcpy(&b, (uint8_t *)&o + 8 * k, 8);
					if (memcmp(&a, &b, 8)) printf("   [%d] %a / %a\n", k, a, b);
				}
			}
			/* resync */
			memcpy(&st.prev, dev + 0x28, sizeof st.prev);
			memcpy(&st.cache, dev + 0x140, sizeof st.cache);
			memcpy(&st.last_sent, dev + 0x2a8, sizeof st.last_sent);
			memcpy(&st.last_fresh_us, dev + 0x3c0, 8);
		}
	}
	printf("hmd pose: %d/%d mismatches\n", bad, n);
	return bad != 0;
}
