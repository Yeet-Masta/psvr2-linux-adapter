/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Headset calibration blob ("SIE" header, cameras + IMUs) to the driver's
 * calibration struct, behaviour-identical to Sony 0x180161b70 (build
 * 1741940190). Spec: docs/sony-re/trace.md section 12.
 */
#ifndef PSVR2_CALIB_H
#define PSVR2_CALIB_H

#include <stddef.h>
#include <stdint.h>

/* Output layout (offsets into the struct Sony fills). */
#define PSVR2_CALIB_CAM_STRIDE 0x110	/* camera i at 0x110 * i */
#define PSVR2_CALIB_NCAM 0x550		/* u32 camera count */
#define PSVR2_CALIB_IMU 0x558		/* IMU i at 0x558 + 0x98 * i */
#define PSVR2_CALIB_IMU_STRIDE 0x98
#define PSVR2_CALIB_NIMU 0x7b8		/* u32 IMU count */
#define PSVR2_CALIB_KIND 0x7bc		/* u8, blob byte 6 */
#define PSVR2_CALIB_VERSION 0x7bd	/* char[16], blob bytes 7..14, zero padded */
/*
 * Like Sony, counts above 5 cameras / 4 IMUs write past the nominal
 * struct (camera 5 / IMU 4 land on the counts); out must hold
 * PSVR2_CALIB_OUT_MAX bytes to be safe for any blob.
 */
#define PSVR2_CALIB_OUT_MAX (PSVR2_CALIB_CAM_STRIDE * 255)

/*
 * Parse blob (size bytes) into out. Returns 1 on success, 0 if the header
 * is rejected. The count / kind / version fields of out are written
 * even when the blob is rejected, as Sony does.
 */
int psvr2_calib_parse(const uint8_t *blob, size_t size, uint8_t *out);

#endif
