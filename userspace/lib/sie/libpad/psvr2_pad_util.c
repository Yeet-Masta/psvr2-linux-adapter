// SPDX-License-Identifier: GPL-2.0
/* Device-id predicates, report CRC, small helpers (docs/sony-re/controllers.md part A). */
#include <string.h>
#include "psvr2_libpad.h"

#define SONY_VID 0x054c

int psvr2_pad_is_vr(uint16_t vid, uint16_t pid)
{
	return vid == SONY_VID && (pid == 0x0ce7 || pid == 0x0e45 || pid == 0x0e46);
}

int psvr2_pad_is_dualsense(uint16_t vid, uint16_t pid)
{
	return vid == SONY_VID && (pid == 0x0ce6 || pid == 0x0df2);
}

int psvr2_pad_is_dualsense_std(uint16_t vid, uint16_t pid)
{
	return vid == SONY_VID && pid == 0x0ce6;
}

int psvr2_pad_is_edge(uint16_t vid, uint16_t pid)
{
	return vid == SONY_VID && pid == 0x0df2;
}

int psvr2_pad_is_0ba0(uint16_t vid, uint16_t pid)
{
	return vid == SONY_VID && pid == 0x0ba0;
}

int psvr2_pad_is_ds4_new(uint16_t vid, uint16_t pid, int32_t hid_ver)
{
	return vid == SONY_VID && (pid == 0x05c4 || pid == 0x09cc) && hid_ver > 22;
}

int psvr2_pad_is_ps5_family(uint16_t vid, uint16_t pid)
{
	return psvr2_pad_is_dualsense(vid, pid) || psvr2_pad_is_vr(vid, pid);
}

int psvr2_pad_is_vr_left(uint16_t vid, uint16_t pid, uint8_t hand)
{
	/* The combined id 0ce7 counts as left only when the hand byte says 1. */
	if (!psvr2_pad_is_vr(vid, pid))
		return 0;
	return pid == 0x0e45 || hand == 1;
}

int psvr2_pad_is_vr_right(uint16_t vid, uint16_t pid, uint8_t hand)
{
	/* Any VR id counts as right when the hand byte is 0. */
	return psvr2_pad_is_vr(vid, pid) && (hand == 0 || pid == 0x0e46);
}

int psvr2_pad_is_ds4v2_fw(uint16_t vid, uint16_t pid, uint16_t fw)
{
	if (vid != SONY_VID || pid != 0x09cc || fw < 0x3000)
		return 0;
	switch (fw & 0xf000) {
	case 0x3000: case 0x4000: case 0x5000: case 0x8000: case 0x9000:
		return 0;
	}
	return 1;
}

static uint32_t crc_table[256];

static void crc_init(void)
{
	if (crc_table[1])
		return;
	for (uint32_t i = 0; i < 256; i++) {
		uint32_t c = i;
		for (int k = 0; k < 8; k++)
			c = (c & 1) ? (c >> 1) ^ 0xedb88320u : c >> 1;
		crc_table[i] = c;
	}
}

uint32_t psvr2_pad_crc32_update(uint32_t state, const uint8_t *p, int n)
{
	crc_init();
	for (; n > 0; n--)
		state = (state >> 8) ^ crc_table[(state ^ *p++) & 0xff];
	return state;
}

uint32_t psvr2_pad_crc32(uint8_t seed, const uint8_t *p, int len)
{
	uint32_t s = psvr2_pad_crc32_update(0xffffffffu, &seed, 1);
	return ~psvr2_pad_crc32_update(s, p, len - 4);
}

int psvr2_pad_crc32_bad(uint8_t seed, const uint8_t *p, int len)
{
	const uint8_t *t = p + (len - 4);
	const uint32_t want = t[0] | t[1] << 8 | t[2] << 16 | (uint32_t)t[3] << 24;
	return want != psvr2_pad_crc32(seed, p, len);
}

void psvr2_pad_quat_mul(const float a[4], float out[4], const float b[4])
{
	/* Hamilton product a*b, quaternions stored x, y, z, w. */
	const float ax = a[0], ay = a[1], az = a[2], aw = a[3];
	const float bx = b[0], by = b[1], bz = b[2], bw = b[3];
	out[0] = (ax * bw + bx * aw + ay * bz) - az * by;
	out[1] = (ay * bw + by * aw + az * bx) - bz * ax;
	out[3] = ((bw * aw - ax * bx) - by * ay) - az * bz;
	out[2] = (az * bw + bz * aw + by * ax) - ay * bx;
}

/* DS4 / DualSense IMU calibration feature report: 18 little-endian int16
 * from byte 1. Output: gyro bias xyz, gyro (plus, minus) per axis, gyro
 * speed plus/minus, accel (plus, minus) per axis. With grouped set the
 * report holds the three gyro plus values before the three minus values;
 * otherwise they are already paired. */
void psvr2_pad_imu_cal_parse(int16_t out[18], const uint8_t *rep, int grouped)
{
	int16_t w[18];
	memcpy(w, rep + 1, sizeof w);
	memcpy(out, w, sizeof w);
	if (grouped)
		for (int i = 0; i < 3; i++) {
			out[3 + 2 * i] = w[3 + i];
			out[4 + 2 * i] = w[6 + i];
		}
}
