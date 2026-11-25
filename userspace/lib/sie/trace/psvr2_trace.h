/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VrTrace capture files: readers/writers for every stream file Sony's
 * VrTrace writes (docs/sony-re/vrtrace.md §4), the headset image
 * header check, the 'ST' SLAM-trace frame table of contents, and our own
 * "P2TR" container for USB-level captures used by replay tests (section 7).
 *
 * All functions work on memory buffers (no I/O), little-endian host.
 */
#ifndef PSVR2_TRACE_H
#define PSVR2_TRACE_H

#include <stddef.h>
#include <stdint.h>

/* ---- Sony stream files ---- */

enum psvr2_trace_kind {
	PSVR2_TRACE_DB,			/* db_000.dat: ST payloads from +0x20 */
	PSVR2_TRACE_API,		/* basic_Info.bin: 32-byte header, size@+4 records <= 0x240 */
	PSVR2_TRACE_IMAGE,		/* packed_image.bin: 0xeadca262, 0x40 header, w*h bytes */
	PSVR2_TRACE_RELOC_CONTEXT,	/* reloc_context.bin: 0xfce90 */
	PSVR2_TRACE_CAMERA_SELFCALIB,	/* vrt_camera_selfcalib_vrtrace.bin: 0x800 */
	PSVR2_TRACE_INS,		/* ins.bin: 0x3dd0 */
	PSVR2_TRACE_CONTROLLER_TRACKING,/* controller_tracking.bin: size@+4, 1..0x8440 */
	PSVR2_TRACE_CONTROLLER_PAD,	/* controller_pad%d.bin: size@+4, 1..0x2130 */
	PSVR2_TRACE_RELOC_SNAPSHOT,	/* reloc_snapshot.bin: 0x2000030 */
	PSVR2_TRACE_PLAYAREA,		/* playarea_data.bin: i32 size@+4 */
	PSVR2_TRACE_IMU,		/* imu.bin: 0x108-byte header, 0x2848-byte records */
	PSVR2_TRACE_KIND_COUNT
};

/* Sony's file name for a kind (pad: controller_pad0.bin), or NULL. */
const char *psvr2_trace_file_name(enum psvr2_trace_kind kind);
/* Map a file name to its kind; -1 if unknown. */
int psvr2_trace_kind_from_name(const char *name);

#define PSVR2_TRACE_IMAGE_MAGIC 0xeadca262u
#define PSVR2_TRACE_API_HEADER_SIZE 32
#define PSVR2_TRACE_API_FORMAT 2

/*
 * Record iterator over a whole file image. psvr2_trace_next() returns 1
 * and sets *rec / *len to the next record (for PSVR2_TRACE_IMAGE: the
 * 0x40-byte trace header plus pixels, without the magic), 0 at a clean
 * end of file, or -1 if the data is truncated or malformed (the iterator
 * then stops). For PSVR2_TRACE_API the file header is skipped and its
 * capture type stored in it->capture_type.
 */
struct psvr2_trace_iter {
	const uint8_t *buf;
	size_t size, pos;
	enum psvr2_trace_kind kind;
	uint32_t capture_type;
};
int psvr2_trace_iter_init(struct psvr2_trace_iter *it, enum psvr2_trace_kind kind,
			  const void *buf, size_t size);
int psvr2_trace_next(struct psvr2_trace_iter *it, const uint8_t **rec, size_t *len);

/*
 * Writer: append one record in the file's framing to out (capacity cap).
 * Returns bytes written, 0 if the record is skipped the way Sony skips it
 * (size field out of range), or -1 if out is too small. For
 * PSVR2_TRACE_IMAGE, rec is the 0x40-byte trace header plus pixels.
 * psvr2_trace_write_api_header writes the basic_Info.bin header.
 */
long psvr2_trace_write(enum psvr2_trace_kind kind, const void *rec, size_t len,
		       void *out, size_t cap);
long psvr2_trace_write_api_header(uint32_t capture_type, void *out, size_t cap);

/* ---- Headset image header (V0200), Sony checkImageHeader 0x18010a4f0 ---- */

#define PSVR2_IMAGE_HEADER_SIZE 0x40
/*
 * 0 if buf (size bytes) holds a valid V0200 image, else Sony's code:
 * -1 null/short, -2 magic, -3 V0100, -1000 other version, -5 pitch < width,
 * -6 too small for the tiles, -7 too small for the data, -8 > 0x100000.
 */
int psvr2_trace_check_image_header(const void *buf, uint32_t size);
/*
 * Build the trace header that receiveImage stores (tiles merged into one
 * image). Returns the trace record size (header + pixel area), or 0 if
 * the header is invalid. The pixel bytes to copy are
 * buf + u32@0x1c, length u32@0x20.
 */
uint32_t psvr2_trace_image_trace_header(const void *buf, uint32_t size,
					uint8_t hdr[PSVR2_IMAGE_HEADER_SIZE]);

/* ---- 'ST' SLAM-trace frame ---- */

#define PSVR2_ST_MAX_SIZE 0x80000u
#define PSVR2_ST_VERSION 0x0010
/*
 * 0 if frame is a capturable ST frame: magic "ST", version 0x0010,
 * frame-info version (u64 @0x20) >= 4, and for versions > 4 a TOC entry
 * with id 0. -1 bad size, -2 magic, -3 version, -4 info version < 4,
 * -5 no frame-header entry.
 */
int psvr2_trace_st_check(const void *frame, uint32_t size);
/*
 * TOC lookup in a frame with info version > 4: first entry (of at most
 * 128) with this id. Returns a pointer to its data (frame + 0x20 + offset)
 * and sets *len (Sony clamps: id 0 to 0x20, id 1 to 0x50, id 5 to 0x30
 * when copying), or NULL.
 */
const uint8_t *psvr2_trace_st_find(const void *frame, uint32_t size, uint16_t id, uint32_t *len);
/* Payload Sony writes to db_000.dat for this frame (from +0x20) and its length. */
const uint8_t *psvr2_trace_st_db_payload(const void *frame, uint32_t size, uint32_t *len);

/* ---- imu.bin (Sony VrTraceImuDumper 0x18010b8f0) ---- */

#define PSVR2_TRACE_IMU_HEADER_SIZE 0x108
#define PSVR2_TRACE_IMU_RECORD_SIZE 0x2848
/*
 * Build the imu.bin parts for one ST frame. Returns 0, or -1 when Sony
 * writes nothing for this frame (a needed TOC entry missing: 0 frame
 * header, 6 feature block, 0xd IMU block; for the header also 2 = IMU
 * calibration). The header is written only before the first record.
 */
int psvr2_trace_imu_record(const void *frame, uint32_t size, uint8_t out[PSVR2_TRACE_IMU_RECORD_SIZE]);
int psvr2_trace_imu_header(const void *frame, uint32_t size, uint8_t out[PSVR2_TRACE_IMU_HEADER_SIZE]);

/* ---- P2TR: our USB-level capture container ---- */

/*
 * File: "P2TR", u32 version (1), u64 reserved; then records:
 * u32 tag, u32 size, u64 host_time_ns, size payload bytes, zero padding
 * to a multiple of 8.
 */
#define PSVR2_P2TR_MAGIC 0x52543250u /* "P2TR" */
#define PSVR2_P2TR_VERSION 1
#define PSVR2_P2TR_HEADER_SIZE 16
#define PSVR2_P2TR_REC_HEADER_SIZE 16

/* Tags: bulk IN endpoint (0x83 SLAM, 0x87 image, 0x88 status/IMU, 0x89 LD,
 * 0x8a RP, 0x8b VD, 0x8d log), control Get reply 0x100|report, control Set
 * 0x200|report, Sense HID input 0x300|pad, Sense feature reply 0x400|id. */
#define PSVR2_P2TR_TAG_BULK(ep) ((uint32_t)(ep))
#define PSVR2_P2TR_TAG_GET(id) (0x100u | (id))
#define PSVR2_P2TR_TAG_SET(id) (0x200u | (id))
#define PSVR2_P2TR_TAG_PAD_INPUT(pad) (0x300u | (pad))
#define PSVR2_P2TR_TAG_PAD_FEATURE(id) (0x400u | (id))

struct psvr2_p2tr_rec {
	uint32_t tag;
	uint32_t size;
	uint64_t t_ns;
	const uint8_t *data;
};

long psvr2_p2tr_write_header(void *out, size_t cap);
long psvr2_p2tr_write(const struct psvr2_p2tr_rec *r, void *out, size_t cap);
/* Same iterator contract as psvr2_trace_next. */
struct psvr2_p2tr_iter {
	const uint8_t *buf;
	size_t size, pos;
};
int psvr2_p2tr_iter_init(struct psvr2_p2tr_iter *it, const void *buf, size_t size);
int psvr2_p2tr_next(struct psvr2_p2tr_iter *it, struct psvr2_p2tr_rec *r);

#endif
