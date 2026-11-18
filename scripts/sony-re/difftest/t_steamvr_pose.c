/* Sense controller pose report (0x1801a7bf0): DriverPose_t passed to
 * IVRServerDriverHost::TrackedDevicePoseUpdated. */
#include "t_steamvr_host.h"
#include "psvr2_steamvr.h"
#include <math.h>

static psvr2_tracker_pose g_trk;
static uint8_t g_ctl[0x460];
static uint8_t g_fake[0x1000];
static int g_flag[4];
static void *fake_obj(void) { return g_fake; }
static int64_t fake_trk(void *t, void *out, int idx, uint64_t ts) { (void)t; (void)idx; (void)ts; memcpy(out, &g_trk, 0x50); return 0; }
static int64_t fake_ctl(void *m, void *out, int left) { (void)m; (void)left; memcpy(out, g_ctl, sizeof g_ctl); return 0; }
static uint64_t fake_ts(void) { return 1234567; }
static int64_t ret_m1(void) { return -1; }
static int64_t f0(void) { return g_flag[0]; }
static int64_t f1(void) { return g_flag[1]; }
static int64_t f2(void) { return g_flag[2]; }
static int64_t f3(void) { return g_flag[3]; }

static float rf(float lo, float hi) { return lo + (hi - lo) * (float)(rnd() & 0xffffff) / 16777216.0f; }

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	host_install();
	hook(0x180130970, (void *)fake_obj);
	hook(0x180131af0, (void *)f0);          /* pause conditions */
	hook(0x1801a47b0, (void *)f1);
	hook(0x180125480, (void *)f2);
	hook(0x1801193d0, (void *)f3);
	hook(0x1801189d0, (void *)fake_obj);
	hook(0x180118a10, (void *)fake_ctl);    /* controller state */
	hook(0x18016b110, (void *)fake_ts);
	hook(0x18016b180, (void *)fake_ts);
	hook(0x180161140, (void *)fake_obj);
	hook(0x180160230, (void *)fake_trk);    /* tracker pose */
	hook(0x18015bbd0, (void *)fake_obj);
	hook(0x18015a250, (void *)ret_m1);      /* shared-memory slot: skip */
	void (*upd)(void *) = (void (*)(void *))SONY(0x1801a7bf0);
	static uint8_t dev[0x800];
	int bad = 0, n = 100000;

	for (int it = 0; it < n; it++) {
		const int side = rnd() & 1;
		memset(dev, 0, sizeof dev);
		*(uint32_t *)(dev + 0xc) = side;
		*(uint32_t *)(dev + 8) = 1 + rnd() % 8;
		psvr2_driver_pose prev;
		for (int k = 0; k < (int)sizeof prev; k++)
			((uint8_t *)&prev)[k] = (uint8_t)rnd();
		if (rnd() & 1)
			prev.connected = 0;
		memcpy(dev + 0x18, &prev, sizeof prev);
		for (int k = 0; k < 3; k++) {
			g_trk.pos[k] = rf(-3, 3);
			g_trk.vel[k] = rf(-5, 5);
			g_trk.ang_vel[k] = rf(-20, 20);
			g_trk.pad0[k] = rf(-1, 1);
			g_trk.pad1[k] = rf(-1, 1);
		}
		float q[4], nq = 0;
		for (int k = 0; k < 4; k++) { q[k] = rf(-1, 1); nq += q[k] * q[k]; }
		nq = sqrtf(nq);
		for (int k = 0; k < 4; k++)
			g_trk.q[k] = rnd() % 4 ? q[k] / nq : q[k];   /* sometimes unnormalised */
		g_trk.valid = (uint8_t)(rnd() % 3 ? 1 : 0);
		memset(g_flag, 0, sizeof g_flag);
		if (rnd() % 4 == 0)
			g_flag[rnd() % 4] = 1;
		memset(g_ctl, 0, sizeof g_ctl);
		g_ctl[0x40] = (uint8_t)(rnd() % 8 != 0);
		g_ctl[0x5c] = (uint8_t)(rnd() % 8 == 0);
		const int connected = g_ctl[0x40] && !g_ctl[0x5c];
		const int hold = g_flag[0] | g_flag[1] | g_flag[2] | g_flag[3];

		g_npose = 0;
		upd(dev);
		psvr2_driver_pose o;
		psvr2_steamvr_sense_pose(side, &g_trk, connected, hold, g_flag[0], &prev, &o);
		/* Disconnected: Sony clears 0x117 bytes and leaves the last one. */
		const size_t cmp = connected ? sizeof o : sizeof o - 1;
		if (g_npose != 1 || g_pose_id != *(uint32_t *)(dev + 8) || memcmp(g_pose, &o, cmp) ||
		    memcmp(dev + 0x18, g_pose, sizeof o)) {
			if (bad++ < 5) {
				printf("it %d side %d conn %d hold %d:\n", it, side, connected, hold);
				for (size_t k = 0; k < sizeof o / 8; k++) {
					double a, b;
					memcpy(&a, g_pose + 8 * k, 8);
					memcpy(&b, (uint8_t *)&o + 8 * k, 8);
					if (memcmp(&a, &b, 8))
						printf("  [%zu] %a / %a\n", k, a, b);
				}
			}
		}
	}
	printf("sense pose: %d/%d mismatches\n", bad, n);
	return bad != 0;
}
