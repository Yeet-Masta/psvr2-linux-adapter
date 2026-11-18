/* SPDX-License-Identifier: GPL-2.0 */
/*
 * INS helper filters (accelerometer/gyro smoothing, stillness averaging).
 *
 * The reference driver keeps these objects as structs of pointers to their
 * state; the layouts below match it so the differential tests can share one
 * object between both implementations. Behaviour: docs/sony-re/irtracker.md.
 */
#ifndef PSVR2_SIE_INS_FILTER_H
#define PSVR2_SIE_INS_FILTER_H
#include <stdbool.h>
#include <stdint.h>

#define PSVR2_IRT_WF_MAX 8

/* Windowed 3-vector smoother with a "settled" lock (0x1801720a0 family). */
typedef struct {
	float *est;                  /* current estimate (vec3) */
	void *r0[2];
	float ring[PSVR2_IRT_WF_MAX][3]; /* last inputs */
	int *fill;                   /* valid ring entries */
	int *widx;                   /* next ring slot */
	int *locked;                 /* 1 after the window settled */
	uint8_t *settled;
	int *status;                 /* 1 outlier, 2 tracked, 3 settled */
	int *miss;                   /* consecutive outliers */
	uint8_t *enabled;
	void *r1;
	int *cap;                    /* window length, 2..8 */
	float *init;                 /* estimate after reset (vec3) */
	void *r2[2];
	float *gate_track;           /* per-axis gate while not locked (vec3) */
	void *r3[2];
	float *gate_lock;            /* per-axis gate when locked, and window spread limit (vec3) */
	void *r4[2];
	float *alpha;                /* blend factor for accepted inputs */
} psvr2_irtracker_wf;

void psvr2_irtracker_wf_init(psvr2_irtracker_wf *f, bool enabled, int cap, float alpha, const float *gate_track,
			     const float *half_gate_lock, const float *init);
void psvr2_irtracker_wf_reset(psvr2_irtracker_wf *f);
float *psvr2_irtracker_wf_mean(const psvr2_irtracker_wf *f, float *out);
float *psvr2_irtracker_wf_spread(const psvr2_irtracker_wf *f, float *out);
float *psvr2_irtracker_wf_update(psvr2_irtracker_wf *f, float *out, const float *in);

/* Variance source of the stillness averager. */
typedef struct {
	float *var;                  /* per-axis noise variance (vec3) */
	void *r0[2];
	float *rate;                 /* samples per second */
} psvr2_irtracker_still_cfg;

/* Two-phase stillness averager feeding a window filter (0x180171c10 family). */
typedef struct {
	int *phase;                  /* 0 collecting reference, 1 averaging */
	int *count;
	float *ref;                  /* reference sample (vec3) */
	void *r0[2];
	float *acc;                  /* running sum (vec3) */
	void *r1[2];
	float *result;               /* last filtered average (vec3), NaN after reset */
	void *r2[5];
	psvr2_irtracker_wf *wf;
	psvr2_irtracker_still_cfg *cfg;
} psvr2_irtracker_still;

void psvr2_irtracker_still_reset(psvr2_irtracker_still *s);
bool psvr2_irtracker_still_step(psvr2_irtracker_still *s, float *avg_out, const float *in);
void psvr2_irtracker_still_feed(psvr2_irtracker_still *s, const float *in);
void psvr2_irtracker_still_get(const psvr2_irtracker_still *s, float *vec, float *cov9, bool *settled);
#endif
