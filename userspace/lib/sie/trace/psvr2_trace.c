// SPDX-License-Identifier: GPL-2.0
/* VrTrace file formats; see psvr2_trace.h and docs/sony-re/vrtrace.md. */
#include <string.h>
#include "psvr2_trace.h"

static uint16_t rd16(const uint8_t *p) { uint16_t v; memcpy(&v, p, 2); return v; }
static uint32_t rd32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static uint64_t rd64(const uint8_t *p) { uint64_t v; memcpy(&v, p, 8); return v; }

static const char *const names[PSVR2_TRACE_KIND_COUNT] = {
	[PSVR2_TRACE_DB] = "db_000.dat",
	[PSVR2_TRACE_API] = "basic_Info.bin",
	[PSVR2_TRACE_IMAGE] = "packed_image.bin",
	[PSVR2_TRACE_RELOC_CONTEXT] = "reloc_context.bin",
	[PSVR2_TRACE_CAMERA_SELFCALIB] = "vrt_camera_selfcalib_vrtrace.bin",
	[PSVR2_TRACE_INS] = "ins.bin",
	[PSVR2_TRACE_CONTROLLER_TRACKING] = "controller_tracking.bin",
	[PSVR2_TRACE_CONTROLLER_PAD] = "controller_pad0.bin",
	[PSVR2_TRACE_RELOC_SNAPSHOT] = "reloc_snapshot.bin",
	[PSVR2_TRACE_PLAYAREA] = "playarea_data.bin",
	[PSVR2_TRACE_IMU] = "imu.bin",
};

const char *psvr2_trace_file_name(enum psvr2_trace_kind kind)
{
	return (unsigned)kind < PSVR2_TRACE_KIND_COUNT ? names[kind] : NULL;
}

int psvr2_trace_kind_from_name(const char *name)
{
	if (!strcmp(name, "controller_pad1.bin"))
		return PSVR2_TRACE_CONTROLLER_PAD;
	for (int k = 0; k < PSVR2_TRACE_KIND_COUNT; k++)
		if (!strcmp(name, names[k]))
			return k;
	return -1;
}

/* Fixed record size of a kind, or 0 if records are self-sized. */
static size_t fixed_size(enum psvr2_trace_kind kind)
{
	switch (kind) {
	case PSVR2_TRACE_RELOC_CONTEXT: return 0xfce90;
	case PSVR2_TRACE_CAMERA_SELFCALIB: return 0x800;
	case PSVR2_TRACE_INS: return 0x3dd0;
	case PSVR2_TRACE_RELOC_SNAPSHOT: return 0x2000030;
	case PSVR2_TRACE_IMU: return PSVR2_TRACE_IMU_RECORD_SIZE;
	default: return 0;
	}
}

/* Largest valid value of the size field at +4, for kinds that have one. */
static uint32_t size_field_max(enum psvr2_trace_kind kind)
{
	switch (kind) {
	case PSVR2_TRACE_CONTROLLER_TRACKING: return 0x8440;
	case PSVR2_TRACE_CONTROLLER_PAD: return 0x2130;
	case PSVR2_TRACE_PLAYAREA: return 0x7fffffff;
	default: return 0;
	}
}

#define DB_V4_SIZE 0x796b0u
#define DB_MAX_SIZE 0x7ffe0u
#define API_MAX_SIZE 0x240u

/* db_000.dat record length from its first bytes; 0 if unknown yet/invalid. */
static size_t db_len(const uint8_t *p, size_t avail)
{
	uint32_t n;

	if (avail < 12)
		return 0;
	if (rd64(p) == 4)
		return DB_V4_SIZE;
	n = rd32(p + 8);
	return n > DB_MAX_SIZE ? DB_MAX_SIZE : n;
}

int psvr2_trace_iter_init(struct psvr2_trace_iter *it, enum psvr2_trace_kind kind,
			  const void *buf, size_t size)
{
	memset(it, 0, sizeof(*it));
	it->buf = buf;
	it->size = size;
	it->kind = kind;
	if (kind == PSVR2_TRACE_API) {
		if (size < PSVR2_TRACE_API_HEADER_SIZE || rd64(it->buf) != PSVR2_TRACE_API_FORMAT)
			return -1;
		it->capture_type = rd32(it->buf + 8);
		it->pos = PSVR2_TRACE_API_HEADER_SIZE;
	}
	if (kind == PSVR2_TRACE_IMU) {
		if (size && size < PSVR2_TRACE_IMU_HEADER_SIZE)
			return -1;
		it->pos = size ? PSVR2_TRACE_IMU_HEADER_SIZE : 0;
	}
	return (unsigned)kind < PSVR2_TRACE_KIND_COUNT ? 0 : -1;
}

int psvr2_trace_next(struct psvr2_trace_iter *it, const uint8_t **rec, size_t *len)
{
	const uint8_t *p = it->buf + it->pos;
	size_t avail = it->size - it->pos, skip = 0, n;
	uint32_t f;

	if (!avail)
		return 0;
	switch (it->kind) {
	case PSVR2_TRACE_DB:
		n = db_len(p, avail);
		break;
	case PSVR2_TRACE_API:
		if (avail < 8)
			goto bad;
		f = rd32(p + 4);
		n = f > API_MAX_SIZE ? API_MAX_SIZE : f;
		break;
	case PSVR2_TRACE_IMAGE:
		if (avail < 4 + PSVR2_IMAGE_HEADER_SIZE || rd32(p) != PSVR2_TRACE_IMAGE_MAGIC)
			goto bad;
		skip = 4;
		n = PSVR2_IMAGE_HEADER_SIZE + (size_t)rd16(p + 4 + 0x16) * rd16(p + 4 + 0x14);
		break;
	case PSVR2_TRACE_CONTROLLER_TRACKING:
	case PSVR2_TRACE_CONTROLLER_PAD:
	case PSVR2_TRACE_PLAYAREA:
		if (avail < 8)
			goto bad;
		f = rd32(p + 4);
		if (f < 1 || f > size_field_max(it->kind))
			goto bad;
		n = f;
		break;
	default:
		n = fixed_size(it->kind);
		break;
	}
	if (!n || n > avail - skip)
		goto bad;
	*rec = p + skip;
	*len = n;
	it->pos += skip + n;
	return 1;
bad:
	it->pos = it->size;
	return -1;
}

long psvr2_trace_write_api_header(uint32_t capture_type, void *out, size_t cap)
{
	uint8_t h[PSVR2_TRACE_API_HEADER_SIZE] = { 0 };
	uint64_t fmt = PSVR2_TRACE_API_FORMAT;

	if (cap < sizeof(h))
		return -1;
	memcpy(h, &fmt, 8);
	memcpy(h + 8, &capture_type, 4);
	memcpy(out, h, sizeof(h));
	return sizeof(h);
}

long psvr2_trace_write(enum psvr2_trace_kind kind, const void *rec, size_t len,
		       void *out, size_t cap)
{
	const uint8_t *r = rec;
	uint8_t *o = out;
	size_t n = len, pre = 0;
	uint32_t f;

	switch (kind) {
	case PSVR2_TRACE_DB:
		n = db_len(r, len);
		break;
	case PSVR2_TRACE_API:
		if (len < 8)
			return 0;
		f = rd32(r + 4);
		n = f > API_MAX_SIZE ? API_MAX_SIZE : f;
		break;
	case PSVR2_TRACE_IMAGE:
		if (len < PSVR2_IMAGE_HEADER_SIZE)
			return 0;
		pre = 4;
		n = PSVR2_IMAGE_HEADER_SIZE + (size_t)rd16(r + 0x16) * rd16(r + 0x14);
		break;
	case PSVR2_TRACE_CONTROLLER_TRACKING:
	case PSVR2_TRACE_CONTROLLER_PAD:
	case PSVR2_TRACE_PLAYAREA:
		if (len < 8)
			return 0;
		f = rd32(r + 4);
		if (f < 1 || f > size_field_max(kind))
			return 0;
		n = f;
		break;
	default:
		n = fixed_size(kind);
		break;
	}
	if (!n || n > len)
		return 0;
	if (pre + n > cap)
		return -1;
	if (pre) {
		uint32_t m = PSVR2_TRACE_IMAGE_MAGIC;

		memcpy(o, &m, 4);
	}
	memcpy(o + pre, r, n);
	return (long)(pre + n);
}

int psvr2_trace_check_image_header(const void *buf, uint32_t size)
{
	const uint8_t *p = buf;
	uint16_t ver;

	if (!p || size < 0x40)
		return -1;
	if (p[0] != 'V' || p[1] != 'I')
		return -2;
	ver = rd16(p + 2);
	if (ver == 0x0100)
		return -3;
	if (ver != 0x0200)
		return -1000;
	if (rd16(p + 0x16) < rd16(p + 0x12))
		return -5;
	/* 32-bit wrapping arithmetic, as Sony's. */
	if (size < (uint32_t)rd16(p + 0x1a) * rd16(p + 0x18) * rd16(p + 0x14) * rd16(p + 0x16) +
		   rd32(p + 0x1c))
		return -6;
	if (size < rd32(p + 0x20) + rd32(p + 0x1c))
		return -7;
	if (size > 0x100000)
		return -8;
	return 0;
}

uint32_t psvr2_trace_image_trace_header(const void *buf, uint32_t size,
					uint8_t hdr[PSVR2_IMAGE_HEADER_SIZE])
{
	const uint8_t *p = buf;
	uint16_t w, h, pitch;

	if (psvr2_trace_check_image_header(buf, size))
		return 0;
	memcpy(hdr, p, PSVR2_IMAGE_HEADER_SIZE);
	w = (uint16_t)(rd16(p + 0x12) * rd16(p + 0x18));
	h = (uint16_t)(rd16(p + 0x1a) * rd16(p + 0x14));
	pitch = (uint16_t)(rd16(p + 0x16) * rd16(p + 0x18));
	memcpy(hdr + 0x12, &w, 2);
	memcpy(hdr + 0x14, &h, 2);
	memcpy(hdr + 0x16, &pitch, 2);
	return PSVR2_IMAGE_HEADER_SIZE + (uint32_t)pitch * h;
}

const uint8_t *psvr2_trace_st_find(const void *frame, uint32_t size, uint16_t id, uint32_t *len)
{
	const uint8_t *p = frame;
	uint32_t n;

	if (size < 0x44 || rd64(p + 0x20) <= 4)
		return NULL;
	n = rd32(p + 0x30);
	for (uint32_t i = 0; i < n && i < 128; i++) {
		const uint8_t *e = p + 0x44 + 12 * i;

		if (e + 12 > p + size)
			return NULL;
		if (rd16(e) == id) {
			uint32_t sz = rd32(e + 4), off = rd32(e + 8);

			if ((uint64_t)0x20 + off + sz > size)
				return NULL;
			*len = sz;
			return p + 0x20 + off;
		}
	}
	return NULL;
}

int psvr2_trace_st_check(const void *frame, uint32_t size)
{
	const uint8_t *p = frame;
	uint32_t len;

	if (!p || !size || size > PSVR2_ST_MAX_SIZE || size < 0x30)
		return -1;
	if (p[0] != 'S' || p[1] != 'T')
		return -2;
	if (rd16(p + 2) != PSVR2_ST_VERSION)
		return -3;
	if (rd64(p + 0x20) < 4)
		return -4;
	if (rd64(p + 0x20) == 4)
		return 0;
	return psvr2_trace_st_find(frame, size, 0, &len) ? 0 : -5;
}

const uint8_t *psvr2_trace_st_db_payload(const void *frame, uint32_t size, uint32_t *len)
{
	const uint8_t *p = frame;
	size_t n;

	if (size < 0x30)
		return NULL;
	n = db_len(p + 0x20, size - 0x20);
	if (!n)
		return NULL;
	*len = (uint32_t)n;
	return p + 0x20;
}

long psvr2_p2tr_write_header(void *out, size_t cap)
{
	uint32_t h[4] = { PSVR2_P2TR_MAGIC, PSVR2_P2TR_VERSION, 0, 0 };

	if (cap < sizeof(h))
		return -1;
	memcpy(out, h, sizeof(h));
	return sizeof(h);
}

long psvr2_p2tr_write(const struct psvr2_p2tr_rec *r, void *out, size_t cap)
{
	size_t pad = (8 - (r->size & 7)) & 7, n = PSVR2_P2TR_REC_HEADER_SIZE + r->size + pad;
	uint8_t *o = out;

	if (n > cap)
		return -1;
	memcpy(o, &r->tag, 4);
	memcpy(o + 4, &r->size, 4);
	memcpy(o + 8, &r->t_ns, 8);
	memcpy(o + 16, r->data, r->size);
	memset(o + 16 + r->size, 0, pad);
	return (long)n;
}

int psvr2_p2tr_iter_init(struct psvr2_p2tr_iter *it, const void *buf, size_t size)
{
	it->buf = buf;
	it->size = size;
	it->pos = PSVR2_P2TR_HEADER_SIZE;
	if (size < PSVR2_P2TR_HEADER_SIZE || rd32(it->buf) != PSVR2_P2TR_MAGIC ||
	    rd32(it->buf + 4) != PSVR2_P2TR_VERSION) {
		it->pos = size;
		return -1;
	}
	return 0;
}

int psvr2_p2tr_next(struct psvr2_p2tr_iter *it, struct psvr2_p2tr_rec *r)
{
	size_t avail = it->size - it->pos, n;
	const uint8_t *p = it->buf + it->pos;

	if (!avail)
		return 0;
	if (avail < PSVR2_P2TR_REC_HEADER_SIZE)
		goto bad;
	r->tag = rd32(p);
	r->size = rd32(p + 4);
	r->t_ns = rd64(p + 8);
	n = PSVR2_P2TR_REC_HEADER_SIZE + (size_t)r->size;
	if (n > avail)
		goto bad;
	r->data = p + PSVR2_P2TR_REC_HEADER_SIZE;
	n += (8 - (r->size & 7)) & 7;
	it->pos += n > avail ? avail : n;
	return 1;
bad:
	it->pos = it->size;
	return -1;
}

/*
 * Copy up to max bytes of section id into dst (zero-filled first). For the
 * fixed layout (info version 4) the section is at a fixed offset instead.
 * Returns 0, or -1 if a TOC frame lacks the section.
 */
static int st_section(const uint8_t *f, uint32_t size, uint16_t id, uint32_t v4_off,
		      uint32_t v4_len, uint32_t max, uint8_t *dst)
{
	const uint8_t *s;
	uint32_t n;

	memset(dst, 0, max);
	if (rd64(f + 0x20) == 4) {
		if ((uint64_t)v4_off + v4_len > size)
			return -1;
		memcpy(dst, f + v4_off, v4_len);
		return 0;
	}
	s = psvr2_trace_st_find(f, size, id, &n);
	if (!s)
		return -1;
	memcpy(dst, s, n > max ? max : n);
	return 0;
}

#define ST_HDR_MAX 0x20
#define ST_FEAT_MAX 0xbc10
#define ST_IMU_MAX 0x990
#define ST_CAL_MAX 0x160

int psvr2_trace_imu_record(const void *frame, uint32_t size, uint8_t out[PSVR2_TRACE_IMU_RECORD_SIZE])
{
	static uint8_t feat[ST_FEAT_MAX];
	uint8_t hdr[ST_HDR_MAX], imu[ST_IMU_MAX];
	const uint8_t *f = frame;

	if (size < 0x30 || st_section(f, size, 0, 0x30, ST_HDR_MAX, ST_HDR_MAX, hdr) ||
	    st_section(f, size, 6, 0x4130, ST_FEAT_MAX, ST_FEAT_MAX, feat) ||
	    st_section(f, size, 0xd, 0x5ab70, ST_IMU_MAX, ST_IMU_MAX, imu))
		return -1;
	memcpy(out, hdr, 4);			/* frame header +0 */
	memcpy(out + 4, hdr + 8, 4);		/* device timestamp */
	memcpy(out + 8, imu + 0x294, 0xc);
	memcpy(out + 0x14, imu + 0x2a0, 0x24);
	memcpy(out + 0x38, feat + 0x93e8, 0x2810);
	return 0;
}

int psvr2_trace_imu_header(const void *frame, uint32_t size, uint8_t out[PSVR2_TRACE_IMU_HEADER_SIZE])
{
	uint8_t cal[ST_CAL_MAX], imu[ST_IMU_MAX];
	const uint8_t *f = frame;
	static const uint16_t vec_off[3] = { 0x1f8, 0x480, 0x708 };
	uint8_t *o = out;

	if (size < 0x30 || st_section(f, size, 2, 0x80, ST_CAL_MAX, ST_CAL_MAX, cal) ||
	    st_section(f, size, 0xd, 0x5ab70, ST_IMU_MAX, ST_IMU_MAX, imu))
		return -1;
	memcpy(o, cal + 4, 0xc);
	o += 0xc;
	memcpy(o, cal + 0x10, 0x6c);
	o += 0x6c;
	for (int i = 0; i < 3; i++, o += 0xc)
		memcpy(o, imu + vec_off[i], 0xc);
	for (int i = 0; i < 3; i++, o += 0x24)
		memcpy(o, imu + vec_off[i] + 0xc, 0x24);
	return 0;
}
