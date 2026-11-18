// SPDX-License-Identifier: GPL-2.0
/*
 * Controller clock model ("HID_SYNC") and the LED-sync command handler.
 * Behaviour from docs/sony-re/libpad.md, sections "Controller clock fit"
 * and "LED-sync commands".
 */
#include <string.h>
#include "psvr2_libpad.h"

psvr2_sync_globals psvr2_sync_g = { { 23000, 13000 }, { 4300, 500 } };
uint8_t psvr2_sync_fit_enabled = 1;
uint64_t (*psvr2_sync_headset_time)(void);

static const int32_t window_len[2] = { 1, 30 };
static const uint8_t fit_thresh_init[8] = { 100, 100 };
static const int32_t gain_off[2][2] = { { 50, 40 }, { 5, 100 } };
static const int32_t gain_drift[2][2] = { { 10000, 50000 }, { 5000, 30000 } };
#define USB_WARMUP 2
#define TICK_WRAP_US 0x55555555u	/* 2^32 ticks of 1/3 us, in us */

uint32_t psvr2_sync_ticks_to_us(uint32_t ticks)
{
	return ticks / 3 + (ticks % 3 == 2);
}

int32_t psvr2_sync_ctrl_to_host(const psvr2_sync_state *st, uint32_t ctrl_us, int64_t *host)
{
	int64_t corr;
	if (!psvr2_sync_fit_enabled) {
		corr = -st->simple_off;
	} else {
		const uint64_t dt = (uint64_t)ctrl_us - st->ref_ctrl_us;
		corr = (int64_t)(dt * (uint64_t)st->drift_ppb) / 1000000000 + st->fit_off;
	}
	*host = corr + (int64_t)ctrl_us;
	return 0;
}

int32_t psvr2_sync_host_to_ctrl(const psvr2_sync_state *st, int64_t host, int32_t *ctrl)
{
	if (!psvr2_sync_fit_enabled) {
		*ctrl = (int32_t)(uint32_t)(st->simple_off + host);
		return 0;
	}
	if (!st->fit_off && !st->ref_ctrl_us && !st->drift_ppb)
		return PSVR2_PAD_ERR_LOCK;
	const uint64_t num = (uint64_t)st->drift_ppb * st->ref_ctrl_us +
			     (uint64_t)(host - st->fit_off) * 1000000000u;
	*ctrl = (int32_t)((int64_t)num / (st->drift_ppb + 1000000000));
	return 0;
}

int32_t psvr2_sync_clamp_base(int32_t half, int32_t v)
{
	if (v > half)
		return half;
	if (v < -half)
		return -half;
	return v;
}

uint32_t psvr2_sync_clamp_cycle(psvr2_sync_state *st, uint32_t ns)
{
	if (ns > 17683350) {
		st->n_cycle_hi++;
		if (ns > st->max_cycle)
			st->max_cycle = ns;
		return 17683350;
	}
	if (ns < 15683350) {
		st->n_cycle_lo++;
		if (ns < st->min_cycle)
			st->min_cycle = ns;
		return 15683350;
	}
	return ns;
}

uint8_t psvr2_sync_clamp_period(psvr2_sync_state *st, uint8_t id)
{
	if (id <= 60)
		return id;
	st->n_period_clamp++;
	if (id > st->max_period)
		st->max_period = id;
	return 60;
}

static void put32(uint8_t *b, size_t off, uint32_t v) { memcpy(b + off, &v, 4); }
static void put64(uint8_t *b, size_t off, uint64_t v) { memcpy(b + off, &v, 8); }

void psvr2_sync_stats_reset(psvr2_sync_state *st)
{
	uint8_t *b = (uint8_t *)st;
	/* Zero the measurement block, then seed the min/max trackers. */
	memset(b + 0x868, 0, 2);
	memset(b + 0x86c, 0, 0x880 + 4 - 0x86c);
	memset(b + 0x888, 0, 2);
	memset(b + 0x898, 0, 4);
	memset(b + 0x8a0, 0, 4);
	put32(b, 0x884, 0xffff0000u);
	put64(b, 0x890, ~0ull);
	put32(b, 0x89c, 0xffffffffu);
	memset(b + 0x8a4, 0, 10);
	put32(b, 0x8b0, 0);
	put32(b, 0x8b4, 0xffffffffu);
	put32(b, 0x8b8, 0);
	memset(b + 0x8c0, 0, 0x12);
	put32(b, 0x8d4, 0);
	put32(b, 0x8d8, 0xffffffffu);
	put32(b, 0x8dc, 0);
	memset(b + 0x8e0, 0, 0x91e - 0x8e0);
}

void psvr2_sync_init(psvr2_sync_state *st, uint8_t bt, uint8_t hand)
{
	memset(st, 0, sizeof *st);
	st->bt = bt;
	st->simple_off = 0x7fffffffffff;
	st->fit_thresh = fit_thresh_init[bt & 7];
	st->hand = hand;
	st->run_slow = st->run_fast = st->run_mid = 20;
	st->min_cycle = 0xffffffffu;
	st->min_cycle1 = 0xffffffffu;
	psvr2_sync_stats_reset(st);
}

static int64_t sat_inc(uint8_t *c)
{
	if (*c < 21)
		(*c)++;
	return *c;
}

/* Classify one USB round trip and adapt the latency bias. */
static void usb_latency(psvr2_sync_state *st, uint32_t rtt)
{
	if (rtt >= psvr2_sync_g.rtt_max[0])
		return;
	if (rtt > 14000) {
		st->run_fast = st->run_mid = 0;
		if (sat_inc(&st->run_slow) > 20)
			st->usb_bias = 4000;
		st->n_slow++;
		return;
	}
	st->run_slow = 0;
	if (rtt < 7700) {
		st->run_mid = 0;
		if (sat_inc(&st->run_fast) > 20)
			st->usb_bias = -1000;
		st->n_fast++;
	} else {
		st->run_fast = 0;
		if (st->run_mid > 20 || ++st->run_mid > 20)
			st->usb_bias = 0;
		st->n_mid++;
	}
}

/* Keep the simple (fit-less) offset following the best sample. */
static void track_simple(psvr2_sync_state *st, const psvr2_sync_sample *b,
			 uint32_t cur_ctrl_us, uint32_t echo)
{
	const int64_t d = (int64_t)cur_ctrl_us - st->simple_off;
	const int64_t lo = (int64_t)(b->host & 0xffffffff00000000ull) - 500 + echo;
	const int64_t hi = (int64_t)b->host + 500;
	if (d < lo || d > hi)
		st->simple_off = b->ctrl_minus_mid;
	else
		st->simple_off = (st->simple_off + b->ctrl_minus_mid) / 2;
}

void psvr2_sync_update(psvr2_sync_state *st, uint64_t host_us, uint32_t echo,
		       uint32_t ticks, uint32_t ticks2, uint8_t bt)
{
	const int m = bt ? 1 : 0;	/* bt is 0 or 1 */

	if (st->mode_req != st->mode_cur && bt) {
		psvr2_sync_g.latency[bt] = st->mode_req ? -4500 : 500;
		psvr2_sync_g.rtt_max[bt] = st->mode_req ? 20000 : 13000;
	}
	st->mode_cur = st->mode_req;
	if (st->last_ticks && ticks < st->last_ticks)
		st->wrap |= 0x01;
	if (st->last_ticks2 && ticks2 < st->last_ticks2)
		st->wrap |= 0x10;
	st->calls++;
	st->last_ticks = ticks;
	st->last_ticks2 = ticks2;
	if ((echo == st->last_echo || echo == 0) && st->wrap != 0x11) {
		st->last_echo = echo;
		return;
	}
	st->last_echo = echo;
	if (!st->started)
		st->first_host = host_us;

	/* Record the sample. */
	const uint8_t i = (uint8_t)st->idx;
	psvr2_sync_sample *s = &st->s[i];
	const uint32_t ctrl_us = psvr2_sync_ticks_to_us(ticks);
	const uint32_t rtt = (uint32_t)host_us - echo;
	s->ticks = ticks;
	s->host = host_us;
	s->echo = echo;
	s->ctrl_us = ctrl_us;
	s->rtt = rtt;

	uint64_t mid;
	int32_t corr;
	if (!bt) {
		if (st->warmup < USB_WARMUP) {
			st->warmup++;
			return;
		}
		usb_latency(st, rtt);
		corr = st->usb_bias + psvr2_sync_g.latency[0];
		mid = host_us;
	} else {
		/* Midpoint of send (echoed host stamp) and receive. */
		uint64_t h = host_us;
		if (echo > (uint32_t)host_us && host_us >= 0x100000000ull)
			h = host_us - 0x100000000ull;
		mid = (uint64_t)((int64_t)(host_us + (h & 0xffffffff00000000ull) + echo) / 2);
		corr = psvr2_sync_g.latency[bt];
	}
	s->mid = (int64_t)(mid - (uint64_t)(int64_t)corr);
	s->host_hi = host_us & 0xffffffff00000000ull;
	s->mid_minus_ctrl = s->mid - (int64_t)ctrl_us;
	s->ctrl_minus_mid = (int64_t)ctrl_us - s->mid;

	const uint8_t wrap = st->wrap;
	if (wrap == 0x11) {
		st->wrap_idx = i;
		st->idx = 0;
	} else {
		uint32_t n = st->idx + 1;
		if ((int32_t)n >= window_len[m])
			n = 0;
		st->idx = n;
		if (n)
			return;
	}

	/* Pick the reference sample. */
	psvr2_sync_sample best;
	int jumped = 0;
	if (!bt) {
		uint32_t best_rtt = 0xffffffffu;
		if (wrap == 0x11) {
			best = st->s[st->wrap_idx];
			best_rtt = best.rtt;
		} else {
			for (int k = 0; k < window_len[0]; k++) {
				if (st->s[k].rtt < best_rtt && st->s[k].rtt <= psvr2_sync_g.rtt_max[0]) {
					best = st->s[k];
					best_rtt = best.rtt;
				}
			}
			st->best_rtt = best_rtt;
		}
		if (best_rtt == 0xffffffffu || best.ctrl_us == (uint32_t)st->ref_ctrl_us)
			return;
		track_simple(st, &best, ctrl_us, echo);
		const int64_t prev = st->prev_delta;
		const int64_t cur = (int64_t)(best.host - best.ctrl_us);
		st->prev_delta = cur;
		const int64_t jump = (prev && cur) ? cur - prev : 0;
		if (st->nfit < 100 && st->nfit && (jump > 2000 || jump < -2000)) {
			jumped = 1;
			st->fit_thresh = (uint8_t)(st->nfit + 120);
		}
	} else {
		if (wrap == 0x11) {
			best = st->s[st->wrap_idx];
		} else {
			/* Median by (mid - ctrl): selection sort of a copy. */
			psvr2_sync_sample t[PSVR2_SYNC_WINDOW_MAX];
			const int n = window_len[1];
			memcpy(t, st->s, sizeof t);
			for (int a = 0; a < n - 1; a++) {
				int mi = a;
				for (int b = a; b < n; b++)
					if (t[b].mid_minus_ctrl < t[mi].mid_minus_ctrl)
						mi = b;
				if (mi > a) {
					psvr2_sync_sample x = t[a];
					t[a] = t[mi];
					t[mi] = x;
				}
			}
			best = t[n / 2];
			st->best_rtt = best.rtt;
			if (best.rtt >= psvr2_sync_g.rtt_max[1])
				return;
		}
		if (st->nfit < fit_thresh_init[1] || st->jump_since)
			st->key_delta[st->nfit % 3] = best.mid_minus_ctrl - st->prev_key;
		st->prev_key = best.mid_minus_ctrl;
		/* jump_since is never set by this library build, so the
		 * jump counter never grows; only its reset is kept. */
		if (st->jump_count > 2) {
			st->jump_since = 0;
			st->jump_count = 0;
		}
		if (wrap != 0x11)
			st->key_hist[st->nfit % 3] = best.mid_minus_ctrl;
		if (best.ctrl_us == (uint32_t)st->ref_ctrl_us)
			return;
		track_simple(st, &best, ctrl_us, echo);
	}

	/* Fit offset and drift. */
	if (!st->fit_off || jumped || st->force_reset) {
		st->force_reset = 0;
reset:
		st->resets++;
		st->fit_off = -best.ctrl_minus_mid;
		st->drift_ppb = 0;
		st->fit_thresh = fit_thresh_init[bt];
		st->nfit = 0;
	} else {
		int64_t pred = 0;
		uint32_t ref = best.ctrl_us;
		if (st->wrap == 0x11) {
			ref += TICK_WRAP_US;
			st->wrap = 0;
		}
		psvr2_sync_ctrl_to_host(st, ref, &pred);
		const int64_t err = best.mid - pred;
		uint32_t nf = st->nfit + 1;
		if (!nf)
			nf = st->fit_thresh;
		st->nfit = nf;
		const int k = nf >= st->fit_thresh;
		st->fit_off = (int64_t)((uint64_t)(err / gain_off[bt][k]) - best.ctrl_us + (uint64_t)pred);
		const int64_t span = (int64_t)(uint32_t)(best.ctrl_us - (uint32_t)st->ref_ctrl_us);
		st->drift_ppb += (int64_t)((uint64_t)err * 1000000000u) / span / gain_drift[bt][k];
		if (st->drift_ppb > 70000 || st->drift_ppb < -70000)
			goto reset;
	}
	st->ref_ctrl_us = best.ctrl_us;
}

/* ---- LED-sync record ---- */

static const int32_t mode_width[2] = { 1000, 500 };
static const int32_t mode_step[2] = { 105, 50 };

void psvr2_ledsync_set_mode(psvr2_ledsync *ls, uint8_t mode)
{
	ls->probes = 0;
	ls->sync_mode = mode;
	if (mode == 1 || mode == 2) {
		ls->width = mode_width[mode - 1];
		ls->step = mode_step[mode - 1];
		ls->half = ls->step * 15;
	}
}

static uint32_t rd32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }

static uint32_t scale_cycle(uint32_t cycle, const uint8_t *fp)
{
	float f;
	memcpy(&f, fp, 4);
	const float x = (f + 1.0f) * (float)cycle;
	/* x86 float->int64 conversion: out of range gives INT64_MIN. */
	if (!(x > -9223372036854775808.0f && x < 9223372036854775808.0f))
		return 0;
	return (uint32_t)(int64_t)x;
}

void psvr2_ledsync_command(psvr2_sync_state *st, psvr2_ledsync *ls,
			   const uint8_t *cmd, uint8_t len)
{
	const uint8_t c = cmd[0];
	if (c >= 1 && c <= 5)
		ls->base_time = 0;
	switch (c) {
	case 1: {
		const uint8_t sub = cmd[1];
		if (sub == 1) {
			if (len != 7 && len != 11)
				return;
		} else if ((sub >= 2 && sub <= 4 && len != 11) ||
			   (sub >= 5 && sub <= 6 && len != 2)) {
			return;
		}
		ls->phase = sub;
		if (sub != 1) {
			if (sub >= 2 && sub <= 4) {
				ls->period_id = psvr2_sync_clamp_period(st, cmd[2]);
				memcpy(ls->led, cmd + 3, 4);
				ls->base_time = psvr2_sync_clamp_base(ls->half, (int32_t)rd32(cmd + 7));
			}
			break;
		}
		/* PRESCAN: align base time to the headset clock. */
		ls->period_id = psvr2_sync_clamp_period(st, cmd[2]);
		memcpy(ls->led, cmd + 3, 4);
		const uint64_t t = psvr2_sync_headset_time ? psvr2_sync_headset_time() : 0;
		int32_t ctrl = 0;
		if (!t || psvr2_sync_host_to_ctrl(st, (int64_t)t, &ctrl))
			return;
		ls->base_time = -((int32_t)((uint32_t)ls->period_id * (uint32_t)ls->step) / 2) -
				ls->width / 2 + ctrl;
		if (len == 11)
			ls->cycle_ns = psvr2_sync_clamp_cycle(st, rd32(cmd + 7));
		break;
	}
	case 2:
		if (len != 5)
			return;
		memcpy(ls->led, cmd + 1, 4);
		break;
	case 3:
		if (len != 5)
			return;
		ls->cycle_ns = psvr2_sync_clamp_cycle(st, scale_cycle(ls->cycle_ns, cmd + 1));
		break;
	case 4:
		if (len != 5)
			return;
		ls->base_time = psvr2_sync_clamp_base(ls->half, (int32_t)rd32(cmd + 1));
		break;
	case 5:
		if (len != 9)
			return;
		ls->base_time = psvr2_sync_clamp_base(ls->half, (int32_t)rd32(cmd + 5));
		ls->cycle_ns = psvr2_sync_clamp_cycle(st, scale_cycle(ls->cycle_ns, cmd + 1));
		break;
	case 7:
		if (cmd[1] == 1) {
			ls->phase = 5;
			ls->sync_mode = 0;
		} else if (cmd[1] == 2) {
			/* Reset both records. Sony passes the old hand byte as the
			 * transport and vice versa; kept for equivalence. */
			const uint8_t old_bt = st->bt, old_hand = st->hand;
			psvr2_sync_init(st, old_hand != 0, old_bt != 0);
			memset(ls->led, 0xff, 4);
			ls->last_resync = 0;
			ls->sync_mode = 0;
			ls->width = 1000;
			ls->step = 105;
			ls->half = 1575;
			ls->probes = 0;
			ls->phase = 5;
			ls->seq = 0;
			ls->period_id = 0;
			ls->base_time = 0;
			ls->cycle_ns = 16683350;
		} else if (cmd[1] == 3) {
			uint8_t *b = (uint8_t *)st;
			if (!b[0x868] && cmd[2]) {
				psvr2_sync_stats_reset(st);
			} else {
				/* Measurement report: Sony logs and appends a CSV line
				 * to ../ledsync/<date>.txt (not reproduced). */
				uint16_t n;
				memcpy(&n, b + 0x8f4, 2);
				if (!n) {
					put32(b, 0x89c, 0);
					put32(b, 0x8b4, 0);
					put32(b, 0x8d8, 0);
				}
			}
			b[0x868] = cmd[2] != 0;
		}
		break;
	default:
		return;
	}
	ls->seq++;
}
