// SPDX-License-Identifier: GPL-2.0
/* Calibration blob parser; see psvr2_calib.h and docs/sony-re/trace.md section 12. */
#include <string.h>
#include "psvr2_calib.h"

static void put_u32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }
static uint16_t get_u16(const uint8_t *p) { uint16_t v; memcpy(&v, p, 2); return v; }

/* n floats at src widened to doubles at dst. */
static void widen(uint8_t *dst, const uint8_t *src, int n)
{
	for (int i = 0; i < n; i++) {
		float f;
		double d;

		memcpy(&f, src + 4 * i, 4);
		d = (double)f;
		memcpy(dst + 8 * i, &d, 8);
	}
}

int psvr2_calib_parse(const uint8_t *blob, size_t size, uint8_t *out)
{
	unsigned ncam, nimu, i;
	int dbl = 0, colon = 0;
	size_t cam_size, need;
	const uint8_t *p;

	memset(out + PSVR2_CALIB_VERSION, 0, 16);
	memcpy(out + PSVR2_CALIB_VERSION, blob + 7, 8);
	out[PSVR2_CALIB_KIND] = blob[6];
	ncam = blob[0xf];
	nimu = blob[0x10];
	put_u32(out + PSVR2_CALIB_NCAM, ncam);
	put_u32(out + PSVR2_CALIB_NIMU, nimu);

	/* A 'd' somewhere after a ':' in the version string: doubles stored. */
	for (i = 0; i < 16; i++) {
		char c = (char)out[PSVR2_CALIB_VERSION + i];

		if (c == ':') {
			colon = 1;
		} else if (c == 'd' && colon) {
			dbl = 1;
			break;
		}
	}
	cam_size = dbl ? 0xe4 : 0x94;
	need = (size_t)nimu * 0x34 + 0x18 + cam_size * ncam;
	if (size < need)
		return 0;
	if (blob[0] != 'S' || blob[1] != 'I' || blob[2] != 'E' || blob[3] > 1 ||
	    get_u16(blob + 4) != size)
		return 0;

	p = blob + 0x18;
	for (i = 0; i < ncam; i++, p += cam_size) {
		uint8_t *o = out + PSVR2_CALIB_CAM_STRIDE * i;

		put_u32(o, p[0]);
		put_u32(o + 4, get_u16(p + 4));
		put_u32(o + 8, get_u16(p + 6));
		widen(o + 0x10, p + 8, 9);
		if (dbl)
			memcpy(o + 0x70, p + 0x2c, 20 * 8);
		else
			widen(o + 0x70, p + 0x2c, 20);
	}
	for (i = 0; i < nimu; i++, p += 0x34) {
		uint8_t *o = out + PSVR2_CALIB_IMU + PSVR2_CALIB_IMU_STRIDE * i;

		put_u32(o, p[0]);
		put_u32(o + 4, p[1]);
		widen(o + 8, p + 4, 9);
		widen(o + 0x68, p + 0x28, 3);
	}
	return 1;
}
