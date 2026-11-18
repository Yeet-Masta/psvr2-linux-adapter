// SPDX-License-Identifier: GPL-2.0
/*
 * Differential test of the VrTrace dumpers' file output: call Sony's
 * per-record write functions on a fake dumper object (FILE* at +8, record
 * at +0x48) with a ucrtbase FILE, and compare the bytes with
 * psvr2_trace_write / psvr2_trace_imu_*.
 */
#include "sony.h"
#include "trace_difftest.h"
#include "psvr2_trace.h"

typedef void *(*f_fopen)(const char *, const char *);
typedef int (*f_fclose)(void *);
typedef size_t (*f_fread)(void *, size_t, size_t, void *);
typedef int (*f_fseek)(void *, long, int);
typedef long (*f_ftell)(void *);
static f_fopen u_fopen; static f_fclose u_fclose; static f_fread u_fread;
static f_fseek u_fseek; static f_ftell u_ftell;

static uint8_t rec[0x110000], got[0x100000], want[0x100000];
static uint8_t dumper[0x100];
static void *fp;

static void open_tmp(void)
{
	if (fp)
		u_fclose(fp);
	fp = u_fopen("t_trace_dump.tmp", "w+b");
	memset(dumper, 0, sizeof(dumper));
	memcpy(dumper + 8, &fp, 8);
	void *r = rec;
	memcpy(dumper + 0x48, &r, 8);
}

static size_t file_bytes(void)
{
	long n = u_ftell(fp);
	u_fseek(fp, 0, 0);
	size_t k = u_fread(got, 1, (size_t)n, fp);
	u_fseek(fp, 0, 2);
	return k;
}

static void put32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }
static void put64(uint8_t *p, uint64_t v) { memcpy(p, &v, 8); }

/* Random ST frame: fixed layout (v4) or a TOC with random sections. */
static uint32_t rand_st(void)
{
	uint32_t size = 0x60000 + rnd() % 0x20000;
	for (uint32_t k = 0; k < size; k += 4)
		put32(rec + k, rnd());
	rec[0] = 'S'; rec[1] = 'T';
	rec[2] = 0x10; rec[3] = 0;
	if (rnd() % 4 == 0) {
		put64(rec + 0x20, 4);
		return size;
	}
	put64(rec + 0x20, 5 + rnd() % 3);
	put32(rec + 0x28, rnd() % 4 ? 0x1000 + rnd() % 0x7f000 : rnd());
	uint32_t n = 1 + rnd() % 12;
	put32(rec + 0x30, n);
	static const uint16_t ids[] = { 0, 2, 6, 0xd, 1, 5, 3 };
	for (uint32_t i = 0; i < n; i++) {
		uint8_t *e = rec + 0x44 + 12 * i;
		uint16_t id = (rnd() % 8) ? ids[rnd() % 7] : (uint16_t)rnd();
		uint32_t sz = rnd() % 0xc000, off = 0x200 + rnd() % 0x40000;
		memcpy(e, &id, 2);
		put32(e + 4, sz);
		put32(e + 8, off);
	}
	return size;
}

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	HMODULE u = LoadLibraryA("ucrtbase.dll");
	u_fopen = (f_fopen)GetProcAddress(u, "fopen");
	u_fclose = (f_fclose)GetProcAddress(u, "fclose");
	u_fread = (f_fread)GetProcAddress(u, "fread");
	u_fseek = (f_fseek)GetProcAddress(u, "fseek");
	u_ftell = (f_ftell)GetProcAddress(u, "ftell");
	typedef int (*f_dump)(void *);
	int fail = 0;

	/* Self-sized and fixed-size record kinds. */
	struct { const char *name; uint64_t va; enum psvr2_trace_kind kind; uint32_t maxsz; } k[] = {
		{ "dumpApiData 0x18010b710", 0x18010b710, PSVR2_TRACE_API, 0x400 },
		{ "dumpControllerTrackingData 0x18010b890", 0x18010b890, PSVR2_TRACE_CONTROLLER_TRACKING, 0x9000 },
		{ "dumpControllerPadData 0x18010b830", 0x18010b830, PSVR2_TRACE_CONTROLLER_PAD, 0x2400 },
		{ "dumpPlayArea 0x18010c120", 0x18010c120, PSVR2_TRACE_PLAYAREA, 0x8000 },
		{ "dumpInsData 0x18010c0d0", 0x18010c0d0, PSVR2_TRACE_INS, 0 },
		{ "dumpRelocalizerContext 0x18010c170", 0x18010c170, PSVR2_TRACE_RELOC_CONTEXT, 0 },
	};
	for (unsigned t = 0; t < sizeof(k) / sizeof(k[0]); t++) {
		int bad = 0, n = 2000;
		if (k[t].kind == PSVR2_TRACE_RELOC_CONTEXT) n = 1;
		for (int i = 0; i < n; i++) {
			for (int b = 0; b < 0x10000; b += 4) put32(rec + b, rnd());
			if (k[t].maxsz) put32(rec + 4, (i % 50) ? rnd() % k[t].maxsz : rnd());
			open_tmp();
			((f_dump)SONY(k[t].va))(dumper);
			size_t g = file_bytes();
			long w = psvr2_trace_write(k[t].kind, rec, sizeof(rec), want, sizeof(want));
			if (w < 0) w = 0;
			if ((size_t)w != g || memcmp(got, want, g))
				if (bad++ < 3) printf("  %s: size %u sony %zu ours %ld\n", k[t].name, *(uint32_t *)(rec + 4), g, w);
		}
		fail |= report(k[t].name, bad, n);
	}

	/* packed_image.bin: magic, then 0x40 + pitch(+0x16) * height(+0x14) bytes. */
	{
		int bad = 0, n = 2000;
		for (int i = 0; i < n; i++) {
			for (int b = 0; b < 0x40; b += 4) put32(rec + b, rnd());
			uint16_t w = (uint16_t)(rnd() % 1024), h = (uint16_t)(rnd() % 1024);
			memcpy(rec + 0x16, &w, 2); memcpy(rec + 0x14, &h, 2);
			for (uint32_t b = 0x40; b < 0x40 + (uint32_t)w * h; b += 4) put32(rec + b, rnd());
			open_tmp();
			((f_dump)SONY(0x18010c390))(dumper);
			size_t g = file_bytes();
			long wr = psvr2_trace_write(PSVR2_TRACE_IMAGE, rec, sizeof(rec), want, sizeof(want));
			if ((size_t)wr != g || memcmp(got, want, g))
				if (bad++ < 3) printf("  image: %ux%u sony %zu ours %ld\n", w, h, g, wr);
		}
		fail |= report("dumpVrImage 0x18010c390", bad, n);
	}

	/* basic_Info.bin header (slot 9: dumper, capture type, dir). */
	{
		int bad = 0;
		for (int i = 0; i < 100; i++) {
			uint32_t type = rnd();
			open_tmp();
			((int (*)(void *, uint32_t, const char *))SONY(0x18010d440))(dumper, type, ".");
			size_t g = file_bytes();
			long w = psvr2_trace_write_api_header(type, want, sizeof(want));
			bad += (size_t)w != g || memcmp(got, want, g);
		}
		fail |= report("api header 0x18010d440", bad, 100);
	}

	/* db_000.dat. */
	{
		int bad = 0, n = 2000;
		for (int i = 0; i < n; i++) {
			uint32_t size = rand_st();
			open_tmp();
			((f_dump)SONY(0x18010c2e0))(dumper);
			size_t g = file_bytes();
			uint32_t len = 0;
			const uint8_t *pl = psvr2_trace_st_db_payload(rec, 0x80000, &len);
			long w = pl ? psvr2_trace_write(PSVR2_TRACE_DB, pl, 0x80000 - 0x20, want, sizeof(want)) : 0;
			(void)size;
			if ((size_t)w != g || memcmp(got, want, g))
				if (bad++ < 3) printf("  db: sony %zu ours %ld\n", g, w);
		}
		fail |= report("dumpDbData 0x18010c2e0", bad, n);
	}

	/* ST TOC copy helpers: (id, dst, frame + 0x20) -> 1 and min(size, clamp) bytes, or 0. */
	{
		static const struct { uint64_t va; uint32_t clamp; } h[] = {
			{ 0x18010a840, 0x50 }, { 0x18010a8c0, 0xbc10 }, { 0x18010a940, 0x30 },
			{ 0x18010a9c0, 0x990 }, { 0x18010aa40, 0x20 }, { 0x18010aac0, 0x160 },
		};
		static const uint16_t ids[] = { 0, 1, 2, 5, 6, 0xd, 3 };
		static uint8_t d1[0xc000], d2[0xc000];
		int bad = 0, n = 0;
		for (int i = 0; i < 3000; i++) {
			rand_st();
			if (rec[0x20] == 4)
				continue;
			for (unsigned k = 0; k < 6; k++, n++) {
				uint16_t id = ids[rnd() % 7];
				memset(d1, 0xa5, sizeof(d1)); memset(d2, 0xa5, sizeof(d2));
				int r1 = ((int (*)(uint32_t, void *, void *))SONY(h[k].va))(id, d1, rec + 0x20) & 0xff;
				uint32_t len = 0;
				const uint8_t *p = psvr2_trace_st_find(rec, 0x80000, id, &len);
				int r2 = p != NULL;
				if (p) memcpy(d2, p, len < h[k].clamp ? len : h[k].clamp);
				if (r1 != r2 || memcmp(d1, d2, sizeof(d1)))
					if (bad++ < 3) printf("  toc 0x%llx id %u: sony %d ours %d\n", (unsigned long long)h[k].va, id, r1, r2);
			}
		}
		fail |= report("ST TOC copy 0x18010a840..aac0", bad, n);
	}

	/* imu.bin: first write has the header. */
	{
		int bad = 0, n = 3000;
		for (int i = 0; i < n; i++) {
			uint32_t size = rand_st();
			int first = i & 1;
			open_tmp();
			if (!first) { uint8_t z[8] = { 0 }; ((size_t (*)(void *, size_t, size_t, void *))GetProcAddress(u, "fwrite"))(z, 1, 8, fp); }
			((f_dump)SONY(0x18010b8f0))(dumper);
			size_t g = file_bytes();
			size_t w = first ? 0 : 8;
			memset(want, 0, 8);
			uint8_t h[PSVR2_TRACE_IMU_HEADER_SIZE];
			int ok = psvr2_trace_imu_record(rec, 0x80000, want + w + (first ? PSVR2_TRACE_IMU_HEADER_SIZE : 0)) == 0;
			if (ok && first) {
				if (psvr2_trace_imu_header(rec, 0x80000, h) == 0) {
					memcpy(want, h, sizeof(h));
					w = PSVR2_TRACE_IMU_HEADER_SIZE + PSVR2_TRACE_IMU_RECORD_SIZE;
				}
			} else if (ok) {
				w += PSVR2_TRACE_IMU_RECORD_SIZE;
			}
			(void)size;
			if (w != g || memcmp(got, want, g))
				if (bad++ < 400 && rec[0x20] != 4) { size_t d = 0; while (d < g && d < w && got[d] == want[d]) d++;
					printf("  imu: v=%u first %d sony %zu ours %zu diff@%zu\n", (unsigned)rec[0x20], first, g, w, d); }
		}
		fail |= report("imu dump 0x18010b8f0", bad, n);
	}
	u_fclose(fp);
	return fail;
}
