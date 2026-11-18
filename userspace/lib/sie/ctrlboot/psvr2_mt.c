// SPDX-License-Identifier: GPL-2.0
#include "psvr2_mt.h"

#define M 397

static uint32_t mix(uint32_t a, uint32_t b, uint32_t far)
{
	const uint32_t y = (a & 0x80000000u) | (b & 0x7fffffffu);

	return far ^ (y >> 1) ^ ((y & 1) ? 0x9908b0dfu : 0);
}

void psvr2_mt_twist(psvr2_mt *mt)
{
	uint32_t *s = mt->state;
	int i;

	for (i = 0; i < PSVR2_MT_N - 1; i++)
		s[i] = mix(s[i], s[i + 1], s[(i + M) % PSVR2_MT_N]);
	s[i] = mix(s[i], s[0], s[M - 1]);
}

void psvr2_mt_seed(psvr2_mt *mt, uint32_t seed)
{
	mt->state[0] = seed;
	for (uint32_t i = 1; i < PSVR2_MT_N; i++)
		mt->state[i] = (mt->state[i - 1] ^ (mt->state[i - 1] >> 30)) * 1812433253u + i;
	mt->index = 0;
	psvr2_mt_twist(mt);
}

void psvr2_mt_block(psvr2_mt *mt, uint32_t out[8], uint32_t step)
{
	for (int k = 0; k < 8; k++) {
		uint32_t y = mt->state[mt->index + k];

		y ^= y >> 11;
		y ^= (y << 7) & 0x9d2c5680u;
		y ^= (y << 15) & 0xefc60000u;
		out[k] = y ^ (y >> 18);
	}
	mt->index += step;
	/* A full 8-word window must remain; otherwise regenerate. */
	if (mt->index + 8 > PSVR2_MT_N) {
		mt->index = 0;
		psvr2_mt_twist(mt);
	}
}

uint32_t psvr2_mt_u32(psvr2_mt *mt)
{
	uint32_t b[8];

	psvr2_mt_block(mt, b, 1);
	return b[0];
}

float psvr2_mt_float01(psvr2_mt *mt)
{
	return (float)psvr2_mt_u32(mt) * 0x1p-32f;
}

double psvr2_mt_double01(psvr2_mt *mt)
{
	return (double)psvr2_mt_u32(mt) * 0x1p-32;
}

float psvr2_mt_uniform(psvr2_mt *mt, float lo, float hi)
{
	return (float)((double)psvr2_mt_u32(mt) * 0x1p-32 * (double)(hi - lo) + (double)lo);
}
