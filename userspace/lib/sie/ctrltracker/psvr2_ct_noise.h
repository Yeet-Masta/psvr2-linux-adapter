/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Controller tracker: UKF process-noise helpers. See
 * docs/sony-re/controller-tracking.md part H ("Process noise"). Build with -ffp-contract=off.
 */
#ifndef PSVR2_SIE_CT_NOISE_H
#define PSVR2_SIE_CT_NOISE_H
#include <stdint.h>

/*
 * Per-axis variance driven by a rate vector v: for axis i,
 *   var_i = v_i^2 * own + v_j^2 * cross + v_k^2 * cross + base_i [+ extra]
 * or fixed_value when fixed[i] is set. extra is added only if has_extra.
 */
void psvr2_ct_axis_noise(float out[3], const float v[3], const uint8_t fixed[3],
			 float own, float cross, const float base[3],
			 int has_extra, float extra, float fixed_value);
#endif
