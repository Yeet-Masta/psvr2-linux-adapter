// SPDX-License-Identifier: GPL-2.0
/* See psvr2_ct_noise.h. */
#include "psvr2_ct_noise.h"

void psvr2_ct_axis_noise(float out[3], const float v[3], const uint8_t fixed[3],
			 float own, float cross, const float base[3],
			 int has_extra, float extra, float fixed_value)
{
	const float sq[3] = { v[0] * v[0], v[1] * v[1], v[2] * v[2] };
	const float cr[3] = { sq[0] * cross, sq[1] * cross, sq[2] * cross };
	/* The two other axes in ascending order. */
	static const int other[3][2] = { { 1, 2 }, { 0, 2 }, { 0, 1 } };
	float r[3];

	for (int i = 0; i < 3; i++) {
		if (fixed[i]) {
			r[i] = fixed_value;
			continue;
		}
		float s = (sq[i] * own + cr[other[i][0]]) + cr[other[i][1]];
		if (has_extra)
			s = s + extra;
		r[i] = s + base[i];
	}
	for (int i = 0; i < 3; i++)
		out[i] = r[i];
}
