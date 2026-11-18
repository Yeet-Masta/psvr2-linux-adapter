// SPDX-License-Identifier: GPL-2.0
/* See psvr2_ins_filter.h. Build with -ffp-contract=off. */
#include "psvr2_ins_filter.h"

#include <math.h>
#include <string.h>

static void v3cpy(float *d, const float *s) { d[0] = s[0]; d[1] = s[1]; d[2] = s[2]; }

void psvr2_irtracker_wf_reset(psvr2_irtracker_wf *f)
{
	v3cpy(f->est, f->init);
	*f->locked = 0;
	*f->fill = 0;
	*f->widx = 0;
	*f->miss = 0;
	*f->status = 0;
	*f->settled = 0;
}

void psvr2_irtracker_wf_init(psvr2_irtracker_wf *f, bool enabled, int cap, float alpha, const float *gate_track,
			     const float *half_gate_lock, const float *init)
{
	*f->enabled = enabled;
	*f->cap = (cap >= 2 && cap <= PSVR2_IRT_WF_MAX) ? cap : PSVR2_IRT_WF_MAX;
	v3cpy(f->gate_track, gate_track);
	for (int i = 0; i < 3; i++)
		f->gate_lock[i] = half_gate_lock[i] + half_gate_lock[i];
	*f->alpha = alpha;
	v3cpy(f->init, init);
	psvr2_irtracker_wf_reset(f);
}

float *psvr2_irtracker_wf_mean(const psvr2_irtracker_wf *f, float *out)
{
	const int n = *f->fill;
	if (n < 1 || n > PSVR2_IRT_WF_MAX) {
		out[0] = out[1] = out[2] = 0.0f;
		return out;
	}
	float s[3] = { 0.0f, 0.0f, 0.0f };
	for (int i = 0; i < n; i++)
		for (int k = 0; k < 3; k++)
			s[k] = s[k] + f->ring[i][k];
	for (int k = 0; k < 3; k++)
		out[k] = s[k] / (float)n;
	return out;
}

/* max - min per axis over the filled window; 1000 when it has < 2 or > 8 entries. */
static void spread(const psvr2_irtracker_wf *f, float *out)
{
	const int n = *f->fill;
	if (n < 2 || n > PSVR2_IRT_WF_MAX) {
		out[0] = out[1] = out[2] = 1000.0f;
		return;
	}
	float hi[3], lo[3];
	v3cpy(hi, f->ring[0]);
	v3cpy(lo, f->ring[0]);
	for (int i = 1; i < n; i++)
		for (int k = 0; k < 3; k++) {
			const float x = f->ring[i][k];
			if (hi[k] < x)
				hi[k] = x;
			if (x < lo[k])
				lo[k] = x;
		}
	for (int k = 0; k < 3; k++)
		out[k] = hi[k] - lo[k];
}

float *psvr2_irtracker_wf_spread(const psvr2_irtracker_wf *f, float *out)
{
	spread(f, out);
	return out;
}

/* Passes unless some |d| is strictly above its gate (NaN passes). */
static bool within(const float *d, const float *gate)
{
	for (int k = 0; k < 3; k++)
		if (gate[k] < fabsf(d[k]))
			return false;
	return true;
}

float *psvr2_irtracker_wf_update(psvr2_irtracker_wf *f, float *out, const float *in)
{
	if (!*f->enabled) {
		v3cpy(out, in);
		return out;
	}
	v3cpy(f->ring[*f->widx], in);
	*f->widx = *f->widx + 1 >= *f->cap ? 0 : *f->widx + 1;
	*f->fill = *f->fill + 1 >= *f->cap ? *f->cap : *f->fill + 1;

	float d[3];
	if (*f->fill >= *f->cap) {
		spread(f, d);
		if (within(d, f->gate_lock)) {
			psvr2_irtracker_wf_mean(f, d);
			v3cpy(f->est, d);
			*f->locked = 1;
			*f->status = 3;
			*f->miss = 0;
			*f->settled = 1;
			goto done;
		}
	}
	for (int k = 0; k < 3; k++)
		d[k] = in[k] - f->est[k];
	if (within(d, *f->locked ? f->gate_lock : f->gate_track)) {
		const float a = *f->alpha, b = 1.0f - a;
		for (int k = 0; k < 3; k++)
			f->est[k] = a * in[k] + b * f->est[k];
		*f->status = 2;
		*f->miss = 0;
		goto done;
	}
	*f->status = 1;
	if (*f->miss > (*f->settled ? 32 : 8)) {
		psvr2_irtracker_wf_mean(f, d);
		v3cpy(f->est, d);
		*f->locked = 0;
		*f->miss = 0;
		*f->settled = 0;
	} else {
		*f->miss = *f->miss + 1;
	}
done:
	v3cpy(out, f->est);
	return out;
}

void psvr2_irtracker_still_reset(psvr2_irtracker_still *s)
{
	const float qnan = -NAN;
	for (int k = 0; k < 3; k++) {
		s->result[k] = qnan;
		s->ref[k] = 0.0f;
		s->acc[k] = 0.0f;
	}
	*s->phase = 0;
	*s->count = 0;
}

bool psvr2_irtracker_still_step(psvr2_irtracker_still *s, float *avg_out, const float *in)
{
	const int rate = (int)*s->cfg->rate;
	const float *var = s->cfg->var;
	if (*s->phase == 0) {
		if (*s->count == 0)
			v3cpy(s->ref, in);
		*s->count = *s->count + 1;
	} else {
		bool moved = false;
		for (int k = 0; k < 3; k++)
			/* NaN counts as movement. */
			if (!(fabsf(in[k] - s->ref[k]) < sqrtf(var[k]) * 4.0f))
				moved = true;
		if (moved) {
			*s->phase = 0;
			*s->count = 1;
			v3cpy(s->ref, in);
			v3cpy(s->acc, in);
			goto check;
		}
		*s->count = *s->count + 1;
	}
	for (int k = 0; k < 3; k++)
		s->acc[k] = in[k] + s->acc[k];
check:
	if (*s->phase == 0) {
		if (*s->count >= rate / 10) {
			const float n = (float)*s->count;
			for (int k = 0; k < 3; k++)
				s->ref[k] = s->acc[k] / n;
			*s->phase = 1;
			*s->count = 0;
			s->acc[0] = s->acc[1] = s->acc[2] = 0.0f;
		}
	} else if (*s->phase == 1) {
		if (*s->count >= (int)((float)rate * 1.5f)) {
			const float n = (float)*s->count;
			for (int k = 0; k < 3; k++)
				avg_out[k] = s->acc[k] / n;
			psvr2_irtracker_still_reset(s);
			return true;
		}
	}
	return false;
}

void psvr2_irtracker_still_feed(psvr2_irtracker_still *s, const float *in)
{
	float avg[3], f[3];
	if (psvr2_irtracker_still_step(s, avg, in)) {
		psvr2_irtracker_wf_update(s->wf, f, avg);
		v3cpy(s->result, f);
	}
}

void psvr2_irtracker_still_get(const psvr2_irtracker_still *s, float *vec, float *cov9, bool *settled)
{
	v3cpy(vec, s->result);
	memset(cov9, 0, 9 * sizeof(float));
	const float r = *s->cfg->rate;
	for (int k = 0; k < 3; k++)
		cov9[k * 4] = s->cfg->var[k] / r;
	*settled = *s->wf->settled != 0;
}
