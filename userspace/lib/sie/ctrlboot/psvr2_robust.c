// SPDX-License-Identifier: GPL-2.0
#include "psvr2_robust.h"
#include <math.h>

static float weight(const psvr2_robust *rb, float r, float s, int root)
{
	const float a = fabsf(r);
	const float k = s * rb->c;
	float u, w;

	switch (rb->kind) {
	case PSVR2_ROBUST_HUBER:
		if (!(a > k))
			return 1.0f;
		return root ? sqrtf(k / a) : k / a;
	case PSVR2_ROBUST_CLIP:
		return k < a ? 0.0f : 1.0f;
	case PSVR2_ROBUST_TUKEY:
		if (!(a < k))
			return root ? 0.0f : 1.0f;
		u = a / k;
		w = 1.0f - u * u;
		w = w * w;
		return root ? sqrtf(w) : w;
	default:
		return 1.0f;
	}
}

void psvr2_robust_weights(const psvr2_robust *rb, float *out, const float *r, size_t n, float s)
{
	for (size_t i = 0; i < n; i++)
		out[i] = weight(rb, r[i], s, 0);
}

void psvr2_robust_sqrt_weights(const psvr2_robust *rb, float *out, const float *r, size_t n, float s)
{
	for (size_t i = 0; i < n; i++)
		out[i] = weight(rb, r[i], s, 1);
}

void psvr2_robust_weights_sq(const psvr2_robust *rb, float *out, const float *e, size_t n, float t)
{
	for (size_t i = 0; i < n; i++) {
		float w = 1.0f;

		switch (rb->kind) {
		case PSVR2_ROBUST_HUBER:
			if (t < e[i])
				w = sqrtf(t / e[i]);
			break;
		case PSVR2_ROBUST_CLIP:
			w = t < e[i] ? 0.0f : 1.0f;
			break;
		case PSVR2_ROBUST_TUKEY:
			w = 0.0f;
			if (e[i] < t) {
				w = 1.0f - e[i] / t;
				w = w * w;
			}
			break;
		default:
			break;
		}
		out[i] = w;
	}
}

float psvr2_robust_huber_rho(float c, float x)
{
	const float a = fabsf(x);

	if (a < c)
		return a * a;
	return ((a + a) - c) * c;
}

float psvr2_robust_huber_rho_over_x2(float c, float x)
{
	if (fabsf(x) < 0.001f)
		return 2.0f;
	return psvr2_robust_huber_rho(c, x) / (x * x);
}
