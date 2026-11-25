/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MT19937 random generator with the reference driver's output schedule
 * (see docs/sony-re/controller-tracking.md part D, "Random generator"). Checked by
 * scripts/sony-re/difftest/t_ctrlboot_mt.c.
 */
#ifndef PSVR2_CTRLBOOT_MT_H
#define PSVR2_CTRLBOOT_MT_H
#include <stdint.h>

#define PSVR2_MT_N 624

typedef struct {
	uint32_t state[PSVR2_MT_N];
	uint32_t index;
} psvr2_mt;

void psvr2_mt_seed(psvr2_mt *mt, uint32_t seed);
void psvr2_mt_twist(psvr2_mt *mt);
/* Tempered words state[index..index+7] into out, then advance by step. */
void psvr2_mt_block(psvr2_mt *mt, uint32_t out[8], uint32_t step);
uint32_t psvr2_mt_u32(psvr2_mt *mt);
float psvr2_mt_float01(psvr2_mt *mt);   /* [0, 1] in float */
double psvr2_mt_double01(psvr2_mt *mt); /* [0, 1) */
float psvr2_mt_uniform(psvr2_mt *mt, float lo, float hi);

#endif
