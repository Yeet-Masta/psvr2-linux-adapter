// SPDX-License-Identifier: GPL-2.0
/* VrTrace formats: image header check against Sony, and writer/reader round trips. */
#include "sony.h"
#include "trace_difftest.h"
#include "psvr2_trace.h"

struct sony_buf { const void *p; uint32_t size; };
typedef int (*f_chk)(struct sony_buf *);

static uint8_t big[0x110000];
static uint8_t out[0x500000];

static void rand_image_header(uint8_t *h)
{
	for (int k = 0; k < 0x40; k++)
		h[k] = (uint8_t)rnd();
	if (rnd() % 8) { h[0] = 'V'; h[1] = 'I'; }
	uint16_t ver = (rnd() % 8) ? 0x200 : (rnd() % 2 ? 0x100 : (uint16_t)rnd());
	memcpy(h + 2, &ver, 2);
	uint16_t v[5];
	for (int k = 0; k < 5; k++)
		v[k] = (rnd() % 4) ? (uint16_t)(rnd() % 1300) : (uint16_t)rnd();
	if (rnd() % 2) v[3] = 1, v[4] = 1;
	memcpy(h + 0x12, v, 10);
	uint32_t off = (rnd() % 2) ? 0x100 : rnd() % 0x200000, len = rnd() % 0x120000;
	memcpy(h + 0x1c, &off, 4);
	memcpy(h + 0x20, &len, 4);
}

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	int fail = 0, n = 300000, bad = 0;

	for (int i = 0; i < n; i++) {
		rand_image_header(big);
		uint32_t size = (rnd() % 4) ? rnd() % 0x110000 : rnd() % 0x80;
		struct sony_buf b = { (i % 997) ? big : NULL, size };
		int r1 = ((f_chk)SONY(0x18010a4f0))(&b);
		int r2 = psvr2_trace_check_image_header(b.p, size);
		if (r1 != r2 && bad++ < 5)
			printf("size %u sony %d ours %d\n", size, r1, r2);
	}
	fail |= report("check_image_header 0x18010a4f0", bad, n);

	/* Round trips: write a few records of each kind, read them back. */
	bad = 0;
	int trips = 0;
	for (int kind = 0; kind < PSVR2_TRACE_KIND_COUNT; kind++) {
		if (kind == PSVR2_TRACE_RELOC_SNAPSHOT)
			continue; /* 32 MiB records; same code path as the other fixed kinds */
		size_t pos = 0;
		size_t lens[4];
		if (kind == PSVR2_TRACE_API)
			pos += psvr2_trace_write_api_header(7, out, sizeof(out));
		if (kind == PSVR2_TRACE_IMU) {
			memset(out, 0, PSVR2_TRACE_IMU_HEADER_SIZE);
			pos += PSVR2_TRACE_IMU_HEADER_SIZE;
		}
		for (int r = 0; r < 4; r++) {
			uint32_t sz = 0x40 + rnd() % 0x200;
			for (size_t k = 0; k < sizeof(big); k++)
				big[k] = (uint8_t)(k * 7 + r);
			uint64_t v = (kind == PSVR2_TRACE_DB && r == 0) ? 4 : 5;
			if (kind == PSVR2_TRACE_DB) { memcpy(big, &v, 8); memcpy(big + 8, &sz, 4); }
			if (kind == PSVR2_TRACE_IMAGE) { uint16_t w = 33, h = 5; memcpy(big + 0x16, &w, 2); memcpy(big + 0x14, &h, 2); }
			if (kind != PSVR2_TRACE_DB && kind != PSVR2_TRACE_IMAGE) memcpy(big + 4, &sz, 4);
			long w = psvr2_trace_write(kind, big, sizeof(big), out + pos, sizeof(out) - pos);
			if (w <= 0) { bad++; printf("write kind %d failed %ld\n", kind, w); break; }
			lens[r] = (size_t)w - (kind == PSVR2_TRACE_IMAGE ? 4 : 0);
			pos += (size_t)w;
		}
		struct psvr2_trace_iter it;
		const uint8_t *rec;
		size_t len;
		int r = 0, ret;
		psvr2_trace_iter_init(&it, kind, out, pos);
		while ((ret = psvr2_trace_next(&it, &rec, &len)) == 1) {
			if (r >= 4 || len != lens[r] || (rec[len - 1] != (uint8_t)((len - 1) * 7 + r) && kind != PSVR2_TRACE_IMAGE))
				bad++;
			r++;
		}
		if (ret != 0 || r != 4 || (kind == PSVR2_TRACE_API && it.capture_type != 7)) {
			bad++;
			printf("read kind %d: ret %d records %d\n", kind, ret, r);
		}
		trips++;
	}
	fail |= report("trace file round trips", bad, trips);

	/* P2TR round trip. */
	bad = 0;
	size_t pos = psvr2_p2tr_write_header(out, sizeof(out));
	uint32_t sizes[50];
	for (int r = 0; r < 50; r++) {
		sizes[r] = rnd() % 100;
		struct psvr2_p2tr_rec rec = { PSVR2_P2TR_TAG_BULK(0x89), sizes[r], (uint64_t)r * 1000, big };
		pos += psvr2_p2tr_write(&rec, out + pos, sizeof(out) - pos);
	}
	struct psvr2_p2tr_iter pit;
	struct psvr2_p2tr_rec rec;
	int r = 0;
	psvr2_p2tr_iter_init(&pit, out, pos);
	while (psvr2_p2tr_next(&pit, &rec) == 1) {
		if (rec.size != sizes[r] || rec.t_ns != (uint64_t)r * 1000 || memcmp(rec.data, big, rec.size))
			bad++;
		r++;
	}
	bad += r != 50;
	fail |= report("p2tr round trip", bad, 50);
	return fail;
}
