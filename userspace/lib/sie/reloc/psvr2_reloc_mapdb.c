// SPDX-License-Identifier: GPL-2.0
/* See psvr2_reloc_mapdb.h and docs/sony-re/room.md part B ("Room map file"). */
#include "psvr2_reloc_mapdb.h"
#include "psvr2_reloc_sha256.h"

#include <math.h>
#include <string.h>

/* ---- raw access ---------------------------------------------------------- */

static uint32_t rd32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static int32_t rdi(const uint8_t *p) { return (int32_t)rd32(p); }
static uint64_t rd64(const uint8_t *p) { uint64_t v; memcpy(&v, p, 8); return v; }
static void wr32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }
static void wr64(uint8_t *p, uint64_t v) { memcpy(p, &v, 8); }

#define KF(db, i)    ((db)->img + PSVR2_MAPDB_KF + (size_t)(i) * PSVR2_MAPDB_KF_STRIDE)
#define FEAT(db, k, i) (KF(db, k) + PSVR2_MAPDB_FEAT + (size_t)(i) * PSVR2_MAPDB_FEAT_STRIDE)
#define IDR(db, i)   ((db)->img + PSVR2_MAPDB_ID + (size_t)(i) * 0x10)
#define EDGE(db, i)  ((db)->img + PSVR2_MAPDB_EDGE + (size_t)(i) * PSVR2_MAPDB_EDGE_SIZE)

/* Keyframe slot fields. */
enum { KF_INDEX = 0x00, KF_SERIAL = 0x04, KF_TAG = 0x08, KF_POSE = 0x0c, KF_ROT = 0x18,
       KF_INFO_END = 0xcc, KF_STATE = 0xcc, KF_GROUP = 0xd0, KF_NFEAT = 0xd4, KF_TIME = 0xd8,
       KF_LINK = 0xe0, KF_HITS = 0xe8, KF_MATCHES = 0xec };
/* Feature record fields. */
enum { FT_SERIAL = 0x04, FT_INVALID = 0x250, FT_WORD = 0x294, FT_POS = 0x298, FT_COV = 0x2a4,
       FT_COV_ID = 0x2c8 };
/* Edge fields. */
enum { ED_SERIAL = 0x04, ED_A = 0x08, ED_B = 0x0c, ED_TYPE = 0x10, ED_REMOVED = 0x14 };

/* Table bookkeeping words. */
enum {
	KF_HINT = 0x11cb438, KF_HIGH = 0x11cb43c, KF_COUNTER = 0x11cb440, FT_COUNTER = 0x11cb444,
	ID_HINT = 0x11cba70, ID_HIGH = 0x11cba74, ID_COUNTER = 0x11cba78,
	MARK_LOW = 0x11cba90, MARK_HIGH = 0x11cba94,
	ED_HINT = 0x12cb778, ED_HIGH = 0x12cb77c, ED_COUNTER = 0x12cb780,
	TR_CREATED = 0x12cb788, TR_LOADS = 0x12cb798, TR_SECONDS = 0x12cb79c,
};

static int bit(const psvr2_mapdb *db, uint32_t bits, uint32_t i)
{
	return (int)(rd64(db->img + bits + (size_t)(i >> 6) * 8) >> (i & 63) & 1);
}

static void set_bit(psvr2_mapdb *db, uint32_t bits, uint32_t i)
{
	uint8_t *w = db->img + bits + (size_t)(i >> 6) * 8;
	wr64(w, rd64(w) | 1ull << (i & 63));
}

static int kf_live(const psvr2_mapdb *db, uint32_t kf)
{
	return kf < PSVR2_MAPDB_KF_MAX && bit(db, PSVR2_MAPDB_KF_BITS, kf);
}

static int top_bit(uint64_t v) { int n = -1; while (v) { n++; v >>= 1; } return n; }

/*
 * Shared allocator: first clear bit, scanning words from the hint. The hint
 * is updated to the word where a clear bit was seen even when the index is
 * then rejected as out of range.
 */
static int table_alloc(psvr2_mapdb *db, uint32_t bits, uint32_t nwords, uint32_t cap, uint32_t hint,
		       uint32_t high, uint32_t *index)
{
	for (uint32_t w = rd32(db->img + hint); w < nwords; w++) {
		uint64_t freebits = ~rd64(db->img + bits + (size_t)w * 8);
		if (!freebits)
			continue;
		wr32(db->img + hint, w);
		int lo = 0;
		while (!(freebits >> lo & 1))
			lo++;
		uint32_t i = (uint32_t)lo + w * 64;
		if (i > cap - 1)
			return -1;
		set_bit(db, bits, i);
		if (rdi(db->img + high) < (int32_t)i)
			wr32(db->img + high, i);
		*index = i;
		return 0;
	}
	return -1;
}

/* Shared release: clear the bit, lower the hint, re-derive "highest". */
static void table_release(psvr2_mapdb *db, uint32_t bits, uint32_t hint, uint32_t high, uint32_t i)
{
	uint32_t w = i >> 6;
	uint8_t *wp = db->img + bits + (size_t)w * 8;
	wr64(wp, rd64(wp) & ~(1ull << (i & 63)));
	if (w < rd32(db->img + hint))
		wr32(db->img + hint, w);
	if (i == rd32(db->img + high)) {
		uint32_t j = w;
		while (rd64(db->img + bits + (size_t)j * 8) == 0 && j != 0)
			j--;
		wr32(db->img + high, (uint32_t)((int32_t)j * 64 + top_bit(rd64(db->img + bits + (size_t)j * 8))));
	}
}

/* ---- whole image -------------------------------------------------------- */

void psvr2_mapdb_reset(psvr2_mapdb *db, const psvr2_mapdb_time *now)
{
	memset(db->img, 0, PSVR2_MAPDB_SIZE);
	wr64(db->img + PSVR2_MAPDB_EVENT_NO, 1);
	db->img[0] = 'M';
	db->img[1] = 'D';
	db->img[2] = 2;
	wr32(db->img + 4, PSVR2_MAPDB_SIZE);
	wr64(db->img + TR_CREATED, now->sec);
	wr64(db->img + TR_CREATED + 8, now->usec);
}

void psvr2_mapdb_new_map_id(psvr2_mapdb *db)
{
	db->id_count++;
	memset(db->img + PSVR2_MAPDB_MAPID_SIG, 0, 9);
	memcpy(db->img + PSVR2_MAPDB_MAPID_SIG, "map_id", 6);
	db->img[0x31] = 0;
	wr32(db->img + PSVR2_MAPDB_MAPID, (db->id_seed + db->id_count) << 16);
}

void psvr2_mapdb_bump_map_id(psvr2_mapdb *db)
{
	uint32_t v = rd32(db->img + PSVR2_MAPDB_MAPID);
	wr32(db->img + PSVR2_MAPDB_MAPID, ((v & 0xff) + 1) | (v & 0xffff0000u));
}

int psvr2_mapdb_get_map_id(const psvr2_mapdb *db, uint32_t *id)
{
	if (memcmp(db->img + PSVR2_MAPDB_MAPID_SIG, "map_id", 7))
		return PSVR2_MAPDB_ENOENT;
	*id = rd32(db->img + PSVR2_MAPDB_MAPID);
	return 0;
}

int psvr2_mapdb_check_hash(const psvr2_mapdb *db, size_t body_end)
{
	uint8_t h[32];
	psvr2_sha256(db->img + 0x28, body_end - 0x28, h);
	return memcmp(h, db->img + 8, 32) ? PSVR2_MAPDB_EBADFILE : 0;
}

void psvr2_mapdb_seal(psvr2_mapdb *db)
{
	psvr2_sha256(db->img + 0x28, PSVR2_MAPDB_SIZE - 0x28, db->img + 8);
}

int psvr2_mapdb_export(psvr2_mapdb *db, uint8_t *buf, uint32_t size)
{
	if (size < PSVR2_MAPDB_SNAPSHOT)
		return PSVR2_MAPDB_EINVAL;
	psvr2_mapdb_seal(db);
	memcpy(buf, db->img, PSVR2_MAPDB_SIZE);
	wr32(buf + PSVR2_MAPDB_SIZE, db->id_seed);
	wr32(buf + PSVR2_MAPDB_SIZE + 4, db->id_count);
	return 0;
}

int psvr2_mapdb_import(psvr2_mapdb *db, const uint8_t *buf, uint32_t size)
{
	if (size < PSVR2_MAPDB_SNAPSHOT)
		return PSVR2_MAPDB_EINVAL;
	memcpy(db->img, buf, PSVR2_MAPDB_SIZE);
	int r = psvr2_mapdb_check_hash(db, PSVR2_MAPDB_SIZE);
	if (r < 0)
		return r;
	db->id_seed = rd32(buf + PSVR2_MAPDB_SIZE);
	db->id_count = rd32(buf + PSVR2_MAPDB_SIZE + 4);
	return 0;
}

int psvr2_mapdb_load(psvr2_mapdb *db, const uint8_t *file, size_t len, const psvr2_mapdb_time *now)
{
	int r = -1;
	if (file) {
		r = PSVR2_MAPDB_EBADFILE;
		uint16_t ver = 0;
		if (len >= 0x28 && file[0] == 'M' && file[1] == 'D')
			memcpy(&ver, file + 2, 2);
		size_t need = ver == 1 ? PSVR2_MAPDB_V1_END : ver == 2 ? PSVR2_MAPDB_SIZE : 0;
		if (need && len >= need) {
			memset(db->img, 0, PSVR2_MAPDB_SIZE);
			memcpy(db->img, file, need);
			if (psvr2_mapdb_check_hash(db, need) == 0) {
				db->img[2] = 2;
				db->img[3] = 0;
				wr32(db->img + TR_LOADS, rd32(db->img + TR_LOADS) + 1);
				db->opened = *now;
				return 0;
			}
		}
	}
	psvr2_mapdb_reset(db, now);
	psvr2_mapdb_new_map_id(db);
	return r;
}

void psvr2_mapdb_account_time(psvr2_mapdb *db, const psvr2_mapdb_time *now)
{
	wr32(db->img + TR_SECONDS, rd32(db->img + TR_SECONDS) + ((uint32_t)now->sec - (uint32_t)db->opened.sec));
	db->opened = *now;
}

const char *psvr2_mapdb_obfuscate(psvr2_mapdb *db, const char *text)
{
	static const char key[7] = { 'R', 'u', 'B', '1', 'C', 'O', 'n' };
	db->obf[0] = 0;
	size_t n = strnlen(text, 127);
	for (size_t i = 0; i < n; i++)
		db->obf[i] = (char)((key[i % 7] ^ text[i]) + 0x20);
	return db->obf;
}

static void log_put(psvr2_mapdb *db, uint32_t base, int32_t idx, uint64_t no, const char *text)
{
	uint8_t *rec = db->img + base + (int64_t)idx * 0x70;
	wr64(rec, no);
	strncpy((char *)rec + 8, psvr2_mapdb_obfuscate(db, text), 99);
	rec[8 + 99] = 0;
}

uint64_t psvr2_mapdb_log(psvr2_mapdb *db, const char *text)
{
	uint8_t *img = db->img;
	const uint64_t no = rd64(img + PSVR2_MAPDB_EVENT_NO);
	log_put(db, PSVR2_MAPDB_ELOG, rdi(img + PSVR2_MAPDB_ELOG_IDX), no, text);
	int32_t e = rdi(img + PSVR2_MAPDB_ELOG_IDX) + 1;
	wr32(img + PSVR2_MAPDB_ELOG_IDX, e > 299 ? 0 : (uint32_t)e);
	if (!memcmp(text, "Query", 6)) {
		log_put(db, PSVR2_MAPDB_QLOG, rdi(img + PSVR2_MAPDB_QLOG_IDX), no, text);
		uint32_t q = rd32(img + PSVR2_MAPDB_QLOG_IDX) + 1;
		wr32(img + PSVR2_MAPDB_QLOG_IDX, q > 199 ? 0 : q);
	}
	wr64(img + PSVR2_MAPDB_EVENT_NO, no + 1);
	return no;
}

int64_t psvr2_mapdb_last_event(const psvr2_mapdb *db)
{
	return (int64_t)rd64(db->img + PSVR2_MAPDB_EVENT_NO) - 1;
}

int64_t psvr2_mapdb_oldest_query(const psvr2_mapdb *db)
{
	int64_t v = (int64_t)rd64(db->img + PSVR2_MAPDB_QLOG + (int64_t)rdi(db->img + PSVR2_MAPDB_QLOG_IDX) * 0x70);
	return v ? v : (int64_t)rd64(db->img + PSVR2_MAPDB_QLOG);
}

/* ---- keyframes ---------------------------------------------------------- */

int psvr2_mapdb_kf_alloc(psvr2_mapdb *db, const uint8_t *kf, const float *intr, const float *stats,
			 uint32_t group, uint32_t nfeat, uint64_t ts, const uint32_t *tag,
			 uint32_t *index, const psvr2_mapdb_time *now)
{
	uint32_t i;
	if (table_alloc(db, PSVR2_MAPDB_KF_BITS, 2, PSVR2_MAPDB_KF_MAX, KF_HINT, KF_HIGH, &i))
		return -1;
	uint8_t *s = KF(db, i);
	memcpy(s + KF_POSE, kf + 4, KF_INFO_END - KF_POSE);
	wr32(s + KF_NFEAT, nfeat);
	wr64(s + KF_TIME, ts);
	wr32(s + KF_STATE, 0);
	wr64(s + KF_HITS, 0);
	wr32(s + KF_GROUP, group);
	wr64(s + KF_LINK, UINT64_MAX);
	wr32(s + KF_TAG, *tag);
	/* Stored order: a1 a2 a0, camera 1 fx fy cx cy, camera 2 fx fy cx cy. */
	static const int pick[11] = { 1, 2, 0, 3, 7, 5, 8, 12, 16, 14, 17 };
	for (int k = 0; k < 11; k++)
		memcpy(s + PSVR2_MAPDB_KF_INTR + 4 * k, &intr[pick[k]], 4);
	const uint32_t serial = rd32(db->img + KF_COUNTER) + 1;
	wr32(db->img + KF_COUNTER, serial);
	wr32(s + KF_SERIAL, serial);
	wr32(s + KF_INDEX, i);
	uint8_t *st = db->img + PSVR2_MAPDB_KF_STATS + (size_t)i * 0x100;
	wr32(st + 0x14, 2);
	wr32(st + 0x10, rd32(db->img + TR_LOADS));
	wr64(st, now->sec);
	wr64(st + 8, now->usec);
	memcpy(st + 0x20, &stats[1], 12);
	memcpy(st + 0x40, &stats[5], 12);
	*index = i;
	return 0;
}

int psvr2_mapdb_kf_set_feature(psvr2_mapdb *db, uint32_t kf, int32_t i, const uint8_t *rec)
{
	if (!kf_live(db, kf) || i < 0 || rdi(KF(db, kf) + KF_NFEAT) <= i)
		return PSVR2_MAPDB_ENOENT;
	uint8_t *f = FEAT(db, kf, i);
	memcpy(f, rec, FT_INVALID + 1);
	memcpy(f + 0x252, rec + 0x252, 0x40);
	memcpy(f + FT_WORD, rec + FT_WORD, PSVR2_MAPDB_FEAT_STRIDE - FT_WORD);
	const uint32_t serial = rd32(db->img + FT_COUNTER) + 1;
	wr32(db->img + FT_COUNTER, serial);
	wr32(f + FT_SERIAL, serial);
	return 0;
}

int psvr2_mapdb_kf_get_feature(const psvr2_mapdb *db, uint32_t kf, uint32_t i, float pos[3],
			       uint32_t *word, float cov[9])
{
	if (!kf_live(db, kf) || i == UINT32_MAX || rd32(KF(db, kf) + KF_NFEAT) <= i)
		return PSVR2_MAPDB_ENOENT;
	const uint8_t *f = FEAT(db, kf, i);
	memcpy(pos, f + FT_POS, 12);
	*word = rd32(f + FT_WORD);
	for (int k = 0; k < 9; k++)
		if (rdi(f + FT_COV_ID + 4 * k) != -1)
			memcpy(&cov[k], f + FT_COV + 4 * k, 4);
	return 0;
}

static void intr_matrix(const uint8_t *s, float k[9])
{
	memset(k, 0, 36);
	memcpy(&k[0], s, 4);
	memcpy(&k[4], s + 4, 4);
	memcpy(&k[2], s + 8, 4);
	memcpy(&k[5], s + 12, 4);
	k[8] = 1.0f;
}

int psvr2_mapdb_kf_get_intrinsics(const psvr2_mapdb *db, uint32_t kf, float *a1, float *a2,
				  float *a0, float k1[9], float k2[9])
{
	if (!kf_live(db, kf))
		return PSVR2_MAPDB_ENOENT;
	const uint8_t *s = KF(db, kf) + PSVR2_MAPDB_KF_INTR;
	memcpy(a1, s, 4);
	memcpy(a2, s + 4, 4);
	memcpy(a0, s + 8, 4);
	intr_matrix(s + 12, k1);
	intr_matrix(s + 28, k2);
	return 0;
}

int psvr2_mapdb_kf_get_time(const psvr2_mapdb *db, uint32_t kf, uint64_t *ts)
{
	if (!kf_live(db, kf))
		return PSVR2_MAPDB_ENOENT;
	*ts = rd64(KF(db, kf) + KF_TIME);
	return 0;
}

int psvr2_mapdb_kf_get_tag(const psvr2_mapdb *db, uint32_t kf, uint32_t *tag)
{
	if (!kf_live(db, kf))
		return PSVR2_MAPDB_ENOENT;
	*tag = rd32(KF(db, kf) + KF_TAG);
	return 0;
}

int psvr2_mapdb_kf_get_state(const psvr2_mapdb *db, uint32_t kf, uint32_t *state)
{
	if (!kf_live(db, kf))
		return PSVR2_MAPDB_ENOENT;
	*state = rd32(KF(db, kf) + KF_STATE);
	return 0;
}

int psvr2_mapdb_kf_get_group(const psvr2_mapdb *db, uint32_t kf, uint32_t *group)
{
	if (!kf_live(db, kf) || rd32(KF(db, kf) + KF_STATE) == 2)
		return PSVR2_MAPDB_ENOENT;
	*group = rd32(KF(db, kf) + KF_GROUP);
	return 0;
}

int psvr2_mapdb_kf_set_group(psvr2_mapdb *db, uint32_t kf, uint32_t group)
{
	if (!kf_live(db, kf) || rd32(KF(db, kf) + KF_STATE) == 2)
		return PSVR2_MAPDB_ENOENT;
	wr32(KF(db, kf) + KF_GROUP, group);
	return 0;
}

int psvr2_mapdb_kf_get_pose(const psvr2_mapdb *db, uint32_t kf, float t[3], float r[9])
{
	if (!kf_live(db, kf))
		return PSVR2_MAPDB_ENOENT;
	memcpy(t, KF(db, kf) + KF_POSE, 12);
	memcpy(r, KF(db, kf) + KF_ROT, 36);
	return 0;
}

int psvr2_mapdb_kf_get_info(const psvr2_mapdb *db, uint32_t kf, uint8_t *out)
{
	if (!kf_live(db, kf))
		return PSVR2_MAPDB_ENOENT;
	memcpy(out + 4, KF(db, kf) + KF_POSE, KF_INFO_END - KF_POSE);
	out[0] = 1;
	return 0;
}

int psvr2_mapdb_kf_set_info(psvr2_mapdb *db, uint32_t kf, const uint8_t *in)
{
	if (!in[0])
		return -1;
	if (!kf_live(db, kf))
		return PSVR2_MAPDB_ENOENT;
	memcpy(KF(db, kf) + KF_POSE, in + 4, KF_INFO_END - KF_POSE);
	return 0;
}

int psvr2_mapdb_kf_dump(const psvr2_mapdb *db, uint32_t kf, uint8_t *out)
{
	if (kf >= PSVR2_MAPDB_KF_MAX)
		return PSVR2_MAPDB_ENOENT;
	const uint8_t *s = KF(db, kf);
	out[0] = (uint8_t)bit(db, PSVR2_MAPDB_KF_BITS, kf);
	memcpy(out + 4, s, KF_STATE);
	out[0xd0] = s[KF_GROUP];
	memcpy(out + 0xd4, s + KF_NFEAT, 4);
	memcpy(out + 0xd8, s + KF_HITS, 8);
	const float unset = -INFINITY;
	for (int i = 0; i < PSVR2_MAPDB_FEAT_MAX; i++) {
		const uint8_t *f = FEAT(db, kf, i);
		uint8_t *o = out + 0xe0 + 12 * i;
		if (f[FT_INVALID])
			for (int k = 0; k < 3; k++)
				memcpy(o + 4 * k, &unset, 4);
		else
			memcpy(o, f + FT_POS, 12);
	}
	return 0;
}

int psvr2_mapdb_kf_get_stats(const psvr2_mapdb *db, uint32_t kf, uint8_t *out)
{
	if (kf >= PSVR2_MAPDB_KF_MAX)
		return PSVR2_MAPDB_ENOENT;
	memcpy(out, db->img + PSVR2_MAPDB_KF_STATS + (size_t)kf * 0x100, 0x100);
	return 0;
}

uint32_t psvr2_mapdb_kf_nfeat(const psvr2_mapdb *db, uint32_t kf)
{
	return rd32(KF(db, kf) + KF_NFEAT);
}

void psvr2_mapdb_kf_count_hit(psvr2_mapdb *db, uint32_t kf)
{
	wr32(KF(db, kf) + KF_HITS, rd32(KF(db, kf) + KF_HITS) + 1);
}

void psvr2_mapdb_kf_count_match(psvr2_mapdb *db, uint32_t kf, int ok)
{
	if (ok)
		wr32(KF(db, kf) + KF_MATCHES, rd32(KF(db, kf) + KF_MATCHES) + 1);
}

void psvr2_mapdb_kf_set_state(psvr2_mapdb *db, uint32_t kf, uint32_t state)
{
	wr32(KF(db, kf) + KF_STATE, state);
}

void psvr2_mapdb_kf_set_link(psvr2_mapdb *db, uint32_t kf, uint64_t link)
{
	wr64(KF(db, kf) + KF_LINK, link);
}

int psvr2_mapdb_kf_count(const psvr2_mapdb *db)
{
	int n = 0;
	for (uint32_t i = 0; i < PSVR2_MAPDB_KF_MAX; i++)
		n += bit(db, PSVR2_MAPDB_KF_BITS, i);
	return n;
}

int psvr2_mapdb_kf_count_state(const psvr2_mapdb *db, int32_t state, int32_t tag)
{
	int n = 0;
	for (uint32_t i = 0; i < PSVR2_MAPDB_KF_MAX; i++)
		if (bit(db, PSVR2_MAPDB_KF_BITS, i) && rdi(KF(db, i) + KF_STATE) == state &&
		    (tag < 0 || rdi(KF(db, i) + KF_TAG) == tag))
			n++;
	return n;
}

void psvr2_mapdb_kf_purge(psvr2_mapdb *db)
{
	/* "highest" shrinks while we go; the bound is re-read every step. */
	for (int32_t i = 0; i <= rdi(db->img + KF_HIGH); i++)
		if (rd32(KF(db, i) + KF_STATE) == 2)
			table_release(db, PSVR2_MAPDB_KF_BITS, KF_HINT, KF_HIGH, (uint32_t)i);
}

/* ---- keyframe-id table -------------------------------------------------- */

int psvr2_mapdb_id_alloc(psvr2_mapdb *db, uint32_t kf, uint32_t *index)
{
	uint32_t i;
	if (table_alloc(db, PSVR2_MAPDB_ID_BITS, 2, PSVR2_MAPDB_KF_MAX, ID_HINT, ID_HIGH, &i))
		return -1;
	uint8_t *r = IDR(db, i);
	wr32(r + 0xc, kf);
	const uint32_t serial = rd32(db->img + ID_COUNTER) + 1;
	wr32(db->img + ID_COUNTER, serial);
	wr32(r + 4, serial);
	wr32(r, i);
	/* A fresh id is never "marked". */
	table_release(db, PSVR2_MAPDB_MARK_BITS, MARK_LOW, MARK_HIGH, i);
	*index = i;
	return 0;
}

int psvr2_mapdb_id_get_serial(const psvr2_mapdb *db, uint32_t id, uint32_t *serial)
{
	if (id >= PSVR2_MAPDB_KF_MAX || !bit(db, PSVR2_MAPDB_ID_BITS, id))
		return PSVR2_MAPDB_ENOENT;
	*serial = rd32(IDR(db, id) + 4);
	return 0;
}

int psvr2_mapdb_id_get_kf(const psvr2_mapdb *db, uint32_t id, uint32_t *kf)
{
	if (id >= PSVR2_MAPDB_KF_MAX || !bit(db, PSVR2_MAPDB_ID_BITS, id))
		return PSVR2_MAPDB_ENOENT;
	*kf = rd32(IDR(db, id) + 0xc);
	return 0;
}

int psvr2_mapdb_id_find_any(const psvr2_mapdb *db, int32_t kf, uint32_t *id)
{
	for (uint32_t i = 0; i < PSVR2_MAPDB_KF_MAX; i++)
		if (rdi(IDR(db, i) + 0xc) == kf) {
			*id = i;
			return 0;
		}
	return PSVR2_MAPDB_ENOENT;
}

void psvr2_mapdb_id_find(const psvr2_mapdb *db, int32_t kf, uint32_t *id)
{
	*id = UINT32_MAX;
	for (uint32_t i = 0; i < PSVR2_MAPDB_KF_MAX; i++)
		if (bit(db, PSVR2_MAPDB_ID_BITS, i) && rdi(IDR(db, i) + 0xc) == kf) {
			*id = i;
			return;
		}
}

void psvr2_mapdb_id_mark(psvr2_mapdb *db, uint32_t n, const uint32_t *ids)
{
	for (uint32_t k = 0; k < n; k++) {
		set_bit(db, PSVR2_MAPDB_MARK_BITS, ids[k]);
		if (rdi(db->img + MARK_HIGH) < (int32_t)ids[k])
			wr32(db->img + MARK_HIGH, ids[k]);
	}
}

int psvr2_mapdb_id_count_active(const psvr2_mapdb *db, int32_t *n)
{
	/* Counts unmarked ids of normal keyframes, below the highest id (exclusive). */
	int c = 0;
	for (int32_t i = rdi(db->img + ID_HIGH) - 1; i >= 0; i--)
		if (i < PSVR2_MAPDB_KF_MAX && bit(db, PSVR2_MAPDB_ID_BITS, (uint32_t)i) &&
		    !bit(db, PSVR2_MAPDB_MARK_BITS, (uint32_t)i) &&
		    rdi(db->img + PSVR2_MAPDB_KF + (int64_t)rdi(IDR(db, i) + 0xc) * PSVR2_MAPDB_KF_STRIDE + KF_STATE) == 0)
			c++;
	*n = c;
	return 0;
}

static int collect_live(psvr2_mapdb *db)
{
	db->live_n = 0;
	for (uint32_t i = 0; i < PSVR2_MAPDB_KF_MAX; i++)
		if (bit(db, PSVR2_MAPDB_KF_BITS, i) && rdi(KF(db, i) + KF_STATE) < 2) {
			db->live[db->live_n][0] = rd32(KF(db, i) + KF_SERIAL);
			db->live[db->live_n][1] = i;
			db->live_n++;
		}
	return db->live_n ? 0 : -1;
}

int psvr2_mapdb_id_count_live(psvr2_mapdb *db, int32_t *n)
{
	*n = 0;
	if (collect_live(db))
		return -1;
	for (uint32_t k = 0; k < db->live_n; k++)
		for (uint32_t i = 0; i < PSVR2_MAPDB_KF_MAX; i++)
			if (bit(db, PSVR2_MAPDB_ID_BITS, i) && rd32(IDR(db, i) + 0xc) == db->live[k][1])
				(*n)++;
	return 0;
}

int psvr2_mapdb_id_purge(psvr2_mapdb *db)
{
	db->dead_n = 0;
	for (uint32_t i = 0; i < PSVR2_MAPDB_KF_MAX; i++)
		if (bit(db, PSVR2_MAPDB_KF_BITS, i) && rdi(KF(db, i) + KF_STATE) == 2)
			db->dead[db->dead_n++] = i;
	if (!db->dead_n)
		return -1;
	for (uint32_t k = 0; k < db->dead_n; k++)
		for (int32_t j = 0; j <= rdi(db->img + ID_HIGH); j++)
			if (rd32(IDR(db, j) + 0xc) == db->dead[k])
				table_release(db, PSVR2_MAPDB_ID_BITS, ID_HINT, ID_HIGH, (uint32_t)j);
	return 0;
}

/* ---- pose-graph edges --------------------------------------------------- */

void psvr2_mapdb_edge_copy(uint8_t *dst, const uint8_t *src)
{
	/* Record copy as the reference does it: the 3 padding bytes after the flag are kept. */
	memcpy(dst, src, ED_REMOVED + 1);
	memcpy(dst + 0x18, src + 0x18, PSVR2_MAPDB_EDGE_SIZE - 0x18);
}

int psvr2_mapdb_edge_alloc(psvr2_mapdb *db, const uint8_t *edge, uint32_t *index)
{
	uint32_t i;
	if (table_alloc(db, PSVR2_MAPDB_EDGE_BITS, 0x4c, PSVR2_MAPDB_EDGE_MAX, ED_HINT, ED_HIGH, &i))
		return -1;
	uint8_t *e = EDGE(db, i);
	psvr2_mapdb_edge_copy(e, edge);
	const uint32_t serial = rd32(db->img + ED_COUNTER) + 1;
	wr32(db->img + ED_COUNTER, serial);
	wr32(e + ED_SERIAL, serial);
	wr32(e, i);
	*index = i;
	return 0;
}

static int edge_live(const psvr2_mapdb *db, uint32_t e)
{
	return e < PSVR2_MAPDB_EDGE_MAX && bit(db, PSVR2_MAPDB_EDGE_BITS, e);
}

int psvr2_mapdb_edge_get(const psvr2_mapdb *db, uint32_t e, uint8_t *out)
{
	if (!edge_live(db, e) || EDGE(db, e)[ED_REMOVED] == 1)
		return PSVR2_MAPDB_ENOENT;
	psvr2_mapdb_edge_copy(out, EDGE(db, e));
	return 0;
}

void psvr2_mapdb_edge_set_removed(psvr2_mapdb *db, uint32_t e)
{
	EDGE(db, e)[ED_REMOVED] = 1;
}

void psvr2_mapdb_edge_free(psvr2_mapdb *db, uint32_t e)
{
	table_release(db, PSVR2_MAPDB_EDGE_BITS, ED_HINT, ED_HIGH, e);
}

void psvr2_mapdb_edge_purge(psvr2_mapdb *db)
{
	/* Visits the range [0, highest] as it was on entry. */
	const int32_t n = rdi(db->img + ED_HIGH) + 1;
	for (int32_t i = 0; i < n; i++)
		if (edge_live(db, (uint32_t)i) && EDGE(db, i)[ED_REMOVED] == 1)
			table_release(db, PSVR2_MAPDB_EDGE_BITS, ED_HINT, ED_HIGH, (uint32_t)i);
}

int psvr2_mapdb_edge_count(const psvr2_mapdb *db)
{
	int c = 0;
	const int32_t n = rdi(db->img + ED_HIGH) + 1;
	for (int32_t i = 0; i < n; i++)
		if (edge_live(db, (uint32_t)i) && EDGE(db, i)[ED_REMOVED] == 0)
			c++;
	return c;
}

int psvr2_mapdb_edge_count_kf(const psvr2_mapdb *db, int32_t kf)
{
	int c = 0;
	const int32_t n = rdi(db->img + ED_HIGH) + 1;
	for (int32_t i = 0; i < n; i++) {
		const uint8_t *e = EDGE(db, i);
		if (edge_live(db, (uint32_t)i) && (rdi(e + ED_A) == kf || rdi(e + ED_B) == kf) && e[ED_REMOVED] == 0)
			c++;
	}
	return c;
}

void psvr2_mapdb_edge_exists(const psvr2_mapdb *db, int32_t a, int32_t b, int32_t type, uint8_t *found)
{
	for (int32_t i = 0; i <= rdi(db->img + ED_HIGH); i++) {
		const uint8_t *e = EDGE(db, i);
		const int32_t ea = rdi(e + ED_A), eb = rdi(e + ED_B);
		if (edge_live(db, (uint32_t)i) && ((ea == a && eb == b) || (ea == b && eb == a)) &&
		    rdi(e + ED_TYPE) == type && e[ED_REMOVED] == 0) {
			*found = 1;
			return;
		}
	}
}

void psvr2_mapdb_edge_find(const psvr2_mapdb *db, int32_t a, int32_t b, int32_t type, uint32_t *idx)
{
	for (int32_t i = 0; i <= rdi(db->img + ED_HIGH); i++) {
		const uint8_t *e = EDGE(db, i);
		if (edge_live(db, (uint32_t)i) && rdi(e + ED_A) == a && rdi(e + ED_B) == b && rdi(e + ED_TYPE) == type) {
			*idx = (uint32_t)i;
			return;
		}
	}
}

int psvr2_mapdb_summary(const psvr2_mapdb *db, uint8_t *out)
{
	if (!out)
		return PSVR2_MAPDB_EINVAL;
	/* Edge part: 4848 x {u8 used, idx, serial, a, b, type}; ids follow at 0x1c680. */
	const int32_t eh = rdi(db->img + ED_HIGH);
	for (int32_t i = 0; i < eh; i++) {
		uint8_t *o = out + (size_t)i * 0x18;
		o[0] = (uint8_t)edge_live(db, (uint32_t)i);
		memcpy(o + 4, EDGE(db, i), 0x14);
	}
	/* Entries from "highest" on only get their flag byte cleared (none if highest < 0). */
	for (uint32_t i = (uint32_t)eh; i < PSVR2_MAPDB_EDGE_MAX; i++)
		out[(size_t)i * 0x18] = 0;
	uint8_t *ids = out + (size_t)PSVR2_MAPDB_EDGE_MAX * 0x18;
	const int32_t ih = rdi(db->img + ID_HIGH);
	for (int32_t i = 0; i < ih; i++) {
		uint8_t *o = ids + (size_t)i * 0x10;
		o[0] = (uint8_t)(i < PSVR2_MAPDB_KF_MAX && bit(db, PSVR2_MAPDB_ID_BITS, (uint32_t)i));
		memcpy(o + 4, IDR(db, i), 8);
		memcpy(o + 0xc, IDR(db, i) + 0xc, 4);
	}
	for (uint32_t i = (uint32_t)ih; i < PSVR2_MAPDB_KF_MAX; i++)
		ids[(size_t)i * 0x10] = 0;
	return 0;
}
