/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PSVR2 headset LED-detection ("LD", USB EP 0x89) blob records.
 *
 * Written from the behavioural spec in docs/sony-driver-re.md and checked
 * bit-for-bit against the reference driver by scripts/sony-re/difftest.
 */
#ifndef PSVR2_SIE_LD_H
#define PSVR2_SIE_LD_H
#include <stdint.h>

#define PSVR2_LD_PACKET_SIZE 0x9050
#define PSVR2_LD_CAMERAS 4
#define PSVR2_LD_MAX_BLOBS 256
#define PSVR2_LD_BLOB_SIZE 36

/* Decoded blobs of one camera, structure-of-arrays. */
typedef struct {
	int32_t reserved;
	float x[256], y[256];          /* centroid, pixels */
	float major[256], minor[256];  /* ellipse semi-axes (std dev, pixels) */
	float angle[256];              /* ellipse angle + 2*pi, or -1 if undefined */
	int32_t link[256];             /* -1; filled by later matching stages */
	float mass[256];               /* m00 (0 for 1-pixel-wide blobs) */
	int16_t w[256], h[256];        /* bounding box size */
	float weight[256];             /* confidence weight */
	uint8_t id[256];               /* headset-assigned id, 0xff if none */
	uint16_t flags[256];           /* record flags | 0x100 degenerate | 0x200 empty | 0x400 sparse */
} psvr2_blob_set;

typedef struct {
	uint16_t flags, link, bbox[4];
	float x, y, major, minor;
	uint8_t id, kind;
} psvr2_blob_summary;

/*
 * Decode one 36-byte blob record. extended = LD format version > 0xff.
 * Appends to out (when *count < 256) and fills sum if non-NULL.
 */
void psvr2_ld_decode_blob(const uint8_t *rec, int extended, psvr2_blob_set *out, int *count,
			  psvr2_blob_summary *sum);
#endif
