// SPDX-License-Identifier: GPL-2.0
/* Per-pad input sample ring (docs/sony-re/libpad.md, "Sample ring"). */
#include <string.h>
#include "psvr2_libpad.h"

/* Trigger state from the mean travel over the last 20 ms of samples, for
 * controllers that do not report it themselves. */
static void estimate_trigger_state(psvr2_pad *p, psvr2_pad_sample *e)
{
	int j = p->ring_idx;
	uint16_t sum0 = e->l2, sum1 = e->r2;
	uint8_t cnt = 1;

	for (int steps = 0;;) {
		j = j == 0 ? PSVR2_PAD_RING - 1 : j - 1;
		if (e->host_us - p->ring[j].host_us > 20000)
			break;
		cnt++;
		sum0 += p->ring[j].l2;
		sum1 += p->ring[j].r2;
		if (++steps > PSVR2_PAD_RING - 1)
			break;
	}
	/* The previous state comes from the entry the walk stopped on. */
	const uint8_t prev0 = p->ring[j].trig_state[0], prev1 = p->ring[j].trig_state[1];
	e->trig_state[0] = psvr2_pad_trigger_state(p->eff_mode0, p->eff0, (uint8_t)(sum0 / cnt), prev0);
	e->trig_state[1] = psvr2_pad_trigger_state(p->eff_mode1, p->eff1, (uint8_t)(sum1 / cnt), prev1);
	e->trig_mode = 0xf0;
}

/* Clear the next ring entry and copy the fields both device kinds share. */
static psvr2_pad_sample *start_sample(psvr2_pad *p, const psvr2_sense_input *in)
{
	psvr2_pad_sample *e = &p->ring[p->ring_idx];

	memset(e, 0, sizeof *e);
	e->host_us = in->host_us;
	e->buttons = in->buttons;
	e->lx = in->lx;
	e->ly = in->ly;
	e->rx = in->rx;
	e->ry = in->ry;
	e->l2 = in->l2;
	e->r2 = in->r2;
	e->counter = in->counter;
	e->battery = in->battery;
	return e;
}

static void advance(psvr2_pad *p)
{
	p->ring_idx = (p->ring_idx + 1) % PSVR2_PAD_RING;
	if (p->ring_n < PSVR2_PAD_RING)
		p->ring_n++;
	else
		p->ring_oldest = p->ring_idx;
}

void psvr2_pad_push_sample(psvr2_pad *p, psvr2_sense_input *in, uint8_t bt)
{
	psvr2_pad_sample *e = start_sample(p, in);

	e->touch = in->touch;
	e->trig_prox = in->trig_prox;
	e->grip_prox = in->grip_prox;
	e->flag0 = in->flag0;
	e->flag3 = in->flag3;
	if (p->trig_level) {
		e->trig_state[0] = in->trig_state_r;	/* offset 0x3e, set for the left hand */
		e->trig_state[1] = in->trig_state_l;
		e->trig_mode = in->trig_mode;
	} else {
		estimate_trigger_state(p, e);
	}

	psvr2_motion_update(&p->motion, &e->motion, p->last ? &p->last->motion : NULL,
			    psvr2_pad_is_dualsense(p->vid, p->pid), in);
	p->sync.mode_req = in->flag0;

	/* IMU timestamp gap (the controller clock is 3 ticks per us and wraps). */
	const uint32_t now = psvr2_sync_ticks_to_us(in->imu_ticks);
	const uint32_t before = psvr2_sync_ticks_to_us(p->sync.last_ticks2);
	p->sync.imu_gap_us = (int16_t)(now - before);
	const uint32_t gap = (now < before ? now + 0x55555555u : now) - before;
	if (gap >= 10000) {
		p->sync.gap_count++;
		p->sync.gap_runs = p->sync.gap_runs == 0xff ? 1 : (uint8_t)(p->sync.gap_runs + 1);
	}
	/* Performance statistics (sync.stats[0] set) are not reproduced. */

	psvr2_sync_update(&p->sync, in->host_us, in->echo, in->ctrl_ticks, in->imu_ticks, bt);
	psvr2_sync_ctrl_to_host(&p->sync, psvr2_sync_ticks_to_us(in->imu_ticks),
				(int64_t *)&in->ctrl_host_us);
	e->ctrl_host_us = in->ctrl_host_us;

	advance(p);
	p->last = e;
}

/* DualSense / DS4 variant: no clock fit (the caller has already put the
 * sample time in ctrl_host_us), touchpad points, and the trigger state is
 * estimated also when firmware effects need it (est_trig). Returns -1, or,
 * when status bit 1 changed and out_flags bits 0-1 are clear, the value
 * Sony passes (twice) to FUN_1801c3970, which sets out_flags bits 2-3 and
 * queues an output report: 1 when the bit went to 0. */
int psvr2_pad_push_sample_ds(psvr2_pad *p, const psvr2_sense_input *in)
{
	psvr2_pad_sample *e = start_sample(p, in);

	e->ctrl_host_us = in->ctrl_host_us;
	e->status[0] = in->status[0];
	e->status[1] = in->status[1];
	if (!p->trig_level || p->est_trig[0] || p->est_trig[1]) {
		estimate_trigger_state(p, e);
	} else {
		e->trig_state[0] = in->trig_state_r;
		e->trig_state[1] = in->trig_state_l;
		e->trig_mode = in->trig_mode;
	}
	e->touch_counter = in->touch_counter;
	uint8_t n = 0;
	for (int i = 0; i < 2; i++)
		if (!(in->touch_id[i] & 0x80)) {
			e->touch_pt[n].id = in->touch_id[i] & 0x7f;
			e->touch_pt[n].x = in->touch_xy[i][0];
			e->touch_pt[n].y = in->touch_xy[i][1];
			n++;
		}
	e->touch_n = n;

	psvr2_motion_update(&p->motion, &e->motion, p->last ? &p->last->motion : NULL,
			    psvr2_pad_is_dualsense(p->vid, p->pid), in);
	advance(p);
	int led = -1;
	if (!(p->out_flags & 3)) {
		const uint8_t bit = e->status[0] & 2;
		if (!p->last || (p->last->status[0] & 2) != bit)
			led = !bit;
	}
	p->last = e;
	return led;
}
