/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PSVR2 relocaliser input ("RP" record, USB EP 0x8a).
 *
 * Written from the behavioural spec in docs/sony-re/room.md part B and checked
 * bit-for-bit against the reference driver by scripts/sony-re/difftest.
 */
#ifndef PSVR2_SIE_RELOC_RP_H
#define PSVR2_SIE_RELOC_RP_H
#include <stdint.h>

#define PSVR2_RP_PACKET_SIZE 0xc8780
/* Bytes of the unpacked relocaliser frame. */
#define PSVR2_RP_FRAME_SIZE 0x9b040

/* Byte offsets inside the unpacked frame (see docs/sony-re/room.md part B, "RP frame"). */
#define PSVR2_RP_F_HEADER      0x00000 /* u32 timestamp-ish, u32 */
#define PSVR2_RP_F_KP_UV       0x00008 /* 324 x { u16 u, u16 v } */
#define PSVR2_RP_F_KP_DESC     0x00518 /* 162 x 128 bytes */
#define PSVR2_RP_F_KP_FLAGS    0x05618 /* 324 x { u8, u8 } */
#define PSVR2_RP_F_KP_POS      0x058a0 /* 162 x 24 bytes */
#define PSVR2_RP_F_KP_EXTRA    0x067d0 /* 162 x 72 bytes */
#define PSVR2_RP_F_KP_PATCH    0x09560 /* 162 x 1152 bytes */
#define PSVR2_RP_F_IMG_VALID   0x36e60 /* bool (1 byte of a 4-byte slot) */
#define PSVR2_RP_F_IMG_INFO    0x36e64 /* u32 */
#define PSVR2_RP_F_IMG         0x36e68 /* 640 x 640 u8 */
#define PSVR2_RP_F_POSE_VALID  0x9ae94 /* bool (1 byte of a 4-byte slot) */
#define PSVR2_RP_F_ANCHOR_N    0x9aff0 /* u32 */
#define PSVR2_RP_F_ANCHOR0     0x9aff4 /* 7 x u32 */
#define PSVR2_RP_F_ANCHOR1     0x9b010 /* 12 x u32 */

/*
 * Unpack a raw RP record into the relocaliser frame layout. Bytes of
 * out that the reference leaves untouched (the 3 upper bytes of the two
 * boolean slots) are left as they were.
 */
void psvr2_reloc_rp_unpack(uint8_t *out, const uint8_t *rp);
#endif
