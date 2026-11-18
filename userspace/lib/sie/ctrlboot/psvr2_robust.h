/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Robust loss functions (IRLS weights) used by the controller pose solver.
 * Written from docs/sony-re/ctrlboot.md; checked against the reference
 * driver by scripts/sony-re/difftest/t_ctrlboot_robust.c.
 */
#ifndef PSVR2_CTRLBOOT_ROBUST_H
#define PSVR2_CTRLBOOT_ROBUST_H
#include <stddef.h>

enum psvr2_robust_kind {
	PSVR2_ROBUST_NULL = 0,
	PSVR2_ROBUST_HUBER = 1,
	PSVR2_ROBUST_CLIP = 2,
	PSVR2_ROBUST_TUKEY = 3,
};

typedef struct {
	enum psvr2_robust_kind kind;
	float c; /* cut-off in units of the noise scale */
} psvr2_robust;

/* Weight per residual r[i] at noise scale s. */
void psvr2_robust_weights(const psvr2_robust *rb, float *out, const float *r, size_t n, float s);
/* Weight per non-negative error e[i] against threshold t (c unused). */
void psvr2_robust_weights_sq(const psvr2_robust *rb, float *out, const float *e, size_t n, float t);
/* Square root of psvr2_robust_weights. */
void psvr2_robust_sqrt_weights(const psvr2_robust *rb, float *out, const float *r, size_t n, float s);
/* Huber only. */
float psvr2_robust_huber_rho(float c, float x);
float psvr2_robust_huber_rho_over_x2(float c, float x);

#endif
