// SPDX-License-Identifier: GPL-2.0
/* See psvr2_reloc_rp.h and docs/sony-re/reloc.md ("RP frame"). */
#include "psvr2_reloc_rp.h"

#include <string.h>

/* The frame is the record body shifted down by the 0x2c-byte record header. */
#define RP_SHIFT 0x2c

static uint32_t rd32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static void wr32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }

static void copy_span(uint8_t *out, const uint8_t *rp, uint32_t from, uint32_t to)
{
	memcpy(out + from, rp + from + RP_SHIFT, to - from);
}

void psvr2_reloc_rp_unpack(uint8_t *out, const uint8_t *rp)
{
	/* Fixed-layout body: plain copy, except two 32-bit flags turned into bytes. */
	copy_span(out, rp, 0, PSVR2_RP_F_IMG_VALID);
	out[PSVR2_RP_F_IMG_VALID] = rd32(rp + PSVR2_RP_F_IMG_VALID + RP_SHIFT) != 0;
	copy_span(out, rp, PSVR2_RP_F_IMG_INFO, PSVR2_RP_F_POSE_VALID);
	out[PSVR2_RP_F_POSE_VALID] = rd32(rp + PSVR2_RP_F_POSE_VALID + RP_SHIFT) != 0;
	copy_span(out, rp, PSVR2_RP_F_POSE_VALID + 4, PSVR2_RP_F_ANCHOR0);

	/* Up to two anchor records; missing ones get fixed "invalid" contents. */
	const uint32_t n = rd32(out + PSVR2_RP_F_ANCHOR_N);
	if (n >= 1) {
		copy_span(out, rp, PSVR2_RP_F_ANCHOR0, PSVR2_RP_F_ANCHOR1);
	} else {
		static const uint32_t none0[7] = { 0xffffffffu, 0, 0, 0, 0xffffffffu, 0xffffffffu, 0xffffffffu };
		memcpy(out + PSVR2_RP_F_ANCHOR0, none0, sizeof none0);
	}
	if (n >= 2) {
		copy_span(out, rp, PSVR2_RP_F_ANCHOR1, PSVR2_RP_FRAME_SIZE);
	} else {
		memset(out + PSVR2_RP_F_ANCHOR1, 0, PSVR2_RP_FRAME_SIZE - PSVR2_RP_F_ANCHOR1);
		wr32(out + PSVR2_RP_F_ANCHOR1, 0xffc00000u); /* quiet NaN */
	}
}
