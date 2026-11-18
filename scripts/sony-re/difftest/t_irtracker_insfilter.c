#include "sony.h"
#include "irt_difftest.h"
#include "psvr2_ins_filter.h"

/* A window filter with its pointed-to state stored alongside. */
typedef struct {
	psvr2_irtracker_wf f;
	float est[3], init[3], gt[3], gl[3], alpha;
	int fill, widx, locked, status, miss, cap;
	uint8_t settled, enabled;
} wfbox;

static void wf_bind(wfbox *b)
{
	b->f.est = b->est; b->f.init = b->init; b->f.gate_track = b->gt; b->f.gate_lock = b->gl; b->f.alpha = &b->alpha;
	b->f.fill = &b->fill; b->f.widx = &b->widx; b->f.locked = &b->locked; b->f.status = &b->status;
	b->f.miss = &b->miss; b->f.cap = &b->cap; b->f.settled = &b->settled; b->f.enabled = &b->enabled;
}

static void wf_clone(wfbox *d, const wfbox *s) { memcpy(d, s, sizeof *s); wf_bind(d); }

static int wf_same(const wfbox *a, const wfbox *b)
{
	return feqn(a->est, b->est, 3) && feqn(a->init, b->init, 3) && feqn(a->gt, b->gt, 3) && feqn(a->gl, b->gl, 3) &&
	       feq(a->alpha, b->alpha) && feqn(&a->f.ring[0][0], &b->f.ring[0][0], 24) && a->fill == b->fill &&
	       a->widx == b->widx && a->locked == b->locked && a->status == b->status && a->miss == b->miss &&
	       a->cap == b->cap && a->settled == b->settled && a->enabled == b->enabled;
}

typedef struct {
	psvr2_irtracker_still s;
	psvr2_irtracker_still_cfg cfg;
	wfbox wf;
	int phase, count;
	float ref[3], acc[3], result[3], var[3], rate;
} stbox;

static void st_bind(stbox *b)
{
	wf_bind(&b->wf);
	b->s.phase = &b->phase; b->s.count = &b->count; b->s.ref = b->ref; b->s.acc = b->acc; b->s.result = b->result;
	b->s.wf = &b->wf.f; b->s.cfg = &b->cfg; b->cfg.var = b->var; b->cfg.rate = &b->rate;
}

static void st_clone(stbox *d, const stbox *s) { memcpy(d, s, sizeof *s); st_bind(d); }

static int st_same(const stbox *a, const stbox *b)
{
	return wf_same(&a->wf, &b->wf) && a->phase == b->phase && a->count == b->count && feqn(a->ref, b->ref, 3) &&
	       feqn(a->acc, b->acc, 3) && feqn(a->result, b->result, 3);
}

static float noisy(float c, float s) { return (rnd() & 63) ? c + rfs(s) : rf(); }

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	typedef void (*f_init)(void *, int, int, float, const float *, const float *, const float *);
	typedef float *(*f_upd)(void *, float *, const float *);
	typedef float *(*f_get)(const void *, float *);
	typedef void (*f_v)(void *);
	int st = 0, bad = 0, bad2 = 0, n = 0;
	for (int it = 0; it < 20000; it++) {
		wfbox a, b;
		memset(&a, 0x5a, sizeof a);
		wf_bind(&a);
		float gt[3], gl[3], in0[3];
		for (int k = 0; k < 3; k++) { gt[k] = rfs(0.2f) + 0.2f; gl[k] = rfs(0.05f) + 0.05f; in0[k] = rfs(2.0f); }
		if ((it & 31) == 0) gt[1] = rf();
		int cap = (int)(rnd() % 11) - 1;
		float alpha = (it & 7) ? rfs(0.5f) + 0.5f : rf();
		wf_clone(&b, &a);
		((f_init)SONY(0x1801726a0))(&a.f, (rnd() % 8) != 0, cap, alpha, gt, gl, in0);
		/* our init: same args (enabled flag passed via the copy) */
		psvr2_irtracker_wf_init(&b.f, a.enabled, cap, alpha, gt, gl, in0);
		if (!wf_same(&a, &b)) { bad++; continue; }
		float c[3] = { rfs(2.0f), rfs(2.0f), rfs(2.0f) };
		for (int s = 0; s < 60; s++, n++) {
			float in[3], o1[3], o2[3];
			if ((rnd() & 15) == 0) for (int k = 0; k < 3; k++) c[k] = rfs(2.0f);
			float sc = (rnd() & 1) ? 0.01f : 0.3f;
			for (int k = 0; k < 3; k++) in[k] = noisy(c[k], sc);
			((f_upd)SONY(0x1801720a0))(&a.f, o1, in);
			psvr2_irtracker_wf_update(&b.f, o2, in);
			if (!feqn(o1, o2, 3) || !wf_same(&a, &b)) {
				if (bad++ < 3) printf("  wf it=%d s=%d est %g/%g st %d/%d miss %d/%d in %g %g %g gt %g %g %g lk %d\n", it, s, a.est[0], b.est[0], a.status, b.status, a.miss, b.miss, in[0], in[1], in[2], gt[0], gt[1], gt[2], a.locked);
				break;
			}
			((f_get)SONY(0x180172620))(&a.f, o1); psvr2_irtracker_wf_mean(&b.f, o2);
			if (!feqn(o1, o2, 3)) bad2++;
			((f_get)SONY(0x180172490))(&a.f, o1); psvr2_irtracker_wf_spread(&b.f, o2);
			if (!feqn(o1, o2, 3)) bad2++;
			if ((rnd() & 127) == 0) { ((f_v)SONY(0x180172040))(&a.f); psvr2_irtracker_wf_reset(&b.f); }
		}
	}
	st |= report("wf init/update/reset 0x1801726a0/0x1801720a0/0x180172040", bad, n);
	st |= report("wf mean/spread 0x180172620/0x180172490", bad2, n);

	typedef int (*f_step)(void *, float *, const float *);
	typedef void (*f_feed)(void *, const float *);
	typedef void (*f_sget)(const void *, float *, float *, uint8_t *);
	bad = 0; bad2 = 0; n = 0;
	for (int it = 0; it < 3000; it++) {
		stbox a, b;
		memset(&a, 0, sizeof a);
		st_bind(&a);
		float gt[3] = { 0.3f, 0.3f, 0.3f }, gl[3] = { 0.02f, 0.02f, 0.02f }, z[3] = { 0, 0, 0 };
		((f_init)SONY(0x1801726a0))(&a.wf.f, 1, 2 + rnd() % 7, 0.1f, gt, gl, z);
		for (int k = 0; k < 3; k++) a.var[k] = (it & 15) ? rfs(1e-4f) + 1e-4f : rf();
		a.rate = (it & 7) ? 20.0f + (float)(rnd() % 400) : rf();
		((f_v)SONY(0x180171d00))(&a.s);
		st_clone(&b, &a);
		float c[3] = { rfs(1.0f), rfs(1.0f), rfs(1.0f) };
		for (int s = 0; s < 1500; s++, n++) {
			float in[3], o1[3] = { 0 }, o2[3] = { 0 };
			if ((rnd() & 255) == 0) for (int k = 0; k < 3; k++) c[k] = rfs(1.0f);
			for (int k = 0; k < 3; k++) in[k] = (rnd() & 1023) ? c[k] + rfs(0.01f) : rf();
			if (s & 1) {
				int r1 = ((f_step)SONY(0x180171d40))(&a.s, o1, in) & 0xff;
				int r2 = psvr2_irtracker_still_step(&b.s, o2, in);
				if (r1 != r2 || (r1 && !feqn(o1, o2, 3))) { bad++; break; }
			} else {
				((f_feed)SONY(0x180171c10))(&a.s, in);
				psvr2_irtracker_still_feed(&b.s, in);
			}
			if (!st_same(&a, &b)) { if (bad++ < 3) printf("  still it=%d s=%d ph %d/%d cnt %d/%d\n", it, s, a.phase, b.phase, a.count, b.count); break; }
			if ((s & 63) == 0) {
				float v1[3], v2[3], c1[9], c2[9]; uint8_t f1 = 7; bool f2;
				((f_sget)SONY(0x180171c80))(&a.s, v1, c1, &f1);
				psvr2_irtracker_still_get(&b.s, v2, c2, &f2);
				if (!feqn(v1, v2, 3) || !feqn(c1, c2, 9) || f1 != f2) bad2++;
			}
		}
	}
	st |= report("still step/feed/reset 0x180171d40/0x180171c10/0x180171d00", bad, n);
	st |= report("still get 0x180171c80", bad2, n / 64);
	return st;
}
