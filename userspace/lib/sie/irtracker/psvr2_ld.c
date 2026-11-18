// SPDX-License-Identifier: GPL-2.0
/* See psvr2_ld.h. Build with -ffp-contract=off: results must match bit for bit. */
#include "psvr2_ld.h"

#include <float.h>
#include <math.h>
#include <string.h>

void psvr2_ld_decode_blob(const uint8_t *rec, int extended, psvr2_blob_set *out, int *count,
			  psvr2_blob_summary *sum) {
	uint16_t flags; memcpy(&flags, rec, 2);
	int16_t x0, x1, y0, y1; memcpy(&x0, rec + 4, 2); memcpy(&x1, rec + 6, 2); memcpy(&y0, rec + 8, 2); memcpy(&y1, rec + 10, 2);
	uint32_t m00, m10, m01, m11, m20, m02;
	memcpy(&m00, rec + 12, 4); memcpy(&m10, rec + 16, 4); memcpy(&m01, rec + 20, 4);
	memcpy(&m11, rec + 24, 4); memcpy(&m20, rec + 28, 4); memcpy(&m02, rec + 32, 4);
	const int w = x1 - x0 + 1, h = y1 - y0 + 1;
	const int dmax = (w < h ? h : w), dmin = (h < w ? h : w);
	const int kind = flags & 0xff;
	float x = -FLT_MAX, y = -FLT_MAX, major = -FLT_MAX, minor = -FLT_MAX, angle = -1.0f, mass = 0, weight = 1.0f;
	uint8_t id = 0, tag = 0;
	int emit = 0;
	if (m00 == 0) {
		flags |= 0x200;
	} else if (kind == 5 && extended) {
		id = rec[2];
		const float fm = (float)m00;
		x = (float)m10 / fm; y = (float)m01 / fm;
		const float fill = (float)(h * w) / fm;
		major = (float)dmax; minor = (float)dmin;
		weight = fill > 5.0f ? fill : 4.0f;
		flags |= fill > 5.0f ? 0x500 : 0x100;
		tag = 0x40;
		if (*count < 256) {
			int i = *count;
			out->x[i] = x; out->y[i] = y; out->major[i] = major; out->minor[i] = minor;
			out->angle[i] = -1.0f; out->link[i] = -1; out->mass[i] = (float)(h * w * 0xff);
			out->w[i] = (int16_t)w; out->h[i] = (int16_t)h; out->weight[i] = weight;
			out->id[i] = id; out->flags[i] = flags; *count = i + 1;
		}
	} else if (kind == 1 || kind == 2) {
		const float fm = (float)m00;
		const float mx = (float)m10 / fm, my = (float)m01 / fm;
		x = (float)x0 + mx; y = (float)y0 + my;
		mass = fm;
		if (w == 1 || h == 1) {
			major = (float)dmax; minor = (float)dmin; mass = 0.0f; angle = -1.0f;
		} else {
			const float cxx = (float)m20 / fm - mx * mx;
			const float cxy = (float)m11 / fm - my * mx;
			const float cyy = (float)m02 / fm - my * my;
			const float det = cyy * cxx - cxy * cxy;
			if (fabsf(det) >= 1e-6f) {
				if (cxy * cxy > 1e-11f) {
					const float mid = (cyy + cxx) * 0.5f;
					const float r = sqrtf(mid * mid - det);
					major = sqrtf(mid + r); minor = sqrtf(mid - r);
				} else {
					major = sqrtf(cxx); minor = sqrtf(cyy);
				}
			} else {
				flags |= 0x100; major = 1.0f; minor = 1.0f;
			}
			const float b = -cyy - cxx;
			const float disc = b * b - det * 4.0f;
			if (disc <= 1e-6f) angle = -1.0f;
			else {
				const float d = (sqrtf(disc) - b) * 0.5f - cyy;
				angle = d == 0.0f ? -1.0f : atanf(cxy / d) + 6.2831854820251465f;
			}
		}
		if (!extended) { id = 0xff; const int not2 = kind != 2; if (not2) weight = 4.0f; tag = not2 ? 0x40 : 0x10; }
		else { tag = rec[3]; id = rec[2]; weight = (float)tag * 0.0625f; }
		emit = 1;
	}
	if (emit && *count < 256) {
		int i = *count;
		out->x[i] = x; out->y[i] = y; out->major[i] = major; out->minor[i] = minor; out->angle[i] = angle;
		out->link[i] = -1; out->mass[i] = mass; out->w[i] = (int16_t)w; out->h[i] = (int16_t)h;
		out->weight[i] = weight; out->id[i] = id; out->flags[i] = flags; *count = i + 1;
	}
	if (sum) {
		sum->flags = flags; sum->link = 0xffff;
		memcpy(sum->bbox, rec + 4, 8);
		sum->x = x; sum->y = y; sum->major = major; sum->minor = minor; sum->id = id; sum->kind = tag;
	}
}
