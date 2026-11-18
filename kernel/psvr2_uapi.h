/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * PSVR2 userspace ABI.
 *
 * The character device /dev/psvr2-pose delivers a stream of fixed-size
 * struct psvr2_pose_sample records (read() returns whole samples; poll()
 * signals POLLIN when samples are available). These come from the headset's
 * onboard 6DoF tracker (USB interface 3, "SLA" packets).
 *
 * Copyright (C) 2026 PSVR2 Linux project
 */
#ifndef _UAPI_PSVR2_H_
#define _UAPI_PSVR2_H_

#include <linux/types.h>

/*
 * position[] and orientation[] are IEEE-754 little-endian 32-bit floats carried
 * as raw bit patterns (reinterpret in userspace, e.g. memcpy into float[]).
 * They are reported in the device's native wire order.
 *
 * Axis convention (device-native raw axes), per the Monado PSVR2 driver's
 * process_slam_record remap, which we adopt as the reference:
 *   position[0]: FORWARD   position[1]: UP        position[2]: RIGHT
 *   orientation[0..3] = quaternion (w, x, y, z) in the same raw frame.
 *
 * To convert into a right-handed +x=right, +y=up, -z=forward frame (OpenVR /
 * OpenXR), apply:
 *   out.position    = ( pos[2],  pos[1], -pos[0] )
 *   out.orientation = ( w=orient[0], x=-orient[2], y=-orient[1], z=orient[3] )
 * See steamvr/driver_psvr2/src/pose_source.cpp for the implementation.
 *
 * The module itself applies NO transform — it carries the raw wire values
 * through; the remap lives in userspace so the convention can change without a
 * module rebuild.
 */
struct psvr2_pose_sample {
	__u64	timestamp_ns;	/* host CLOCK_MONOTONIC receive time */
	__u32	device_vts_us;	/* device video timestamp (microseconds) */
	__u32	flags;		/* PSVR2_POSE_FLAG_* */
	__le32	position[3];	/* float32 LE bit patterns */
	__le32	orientation[4];	/* float32 LE bit patterns, [0] = w */
	/*
	 * Tracker state from the same record (docs/sony-re/tracking.md 2.1).
	 * Vectors are float32 bit patterns in the position's raw frame.
	 */
	__u8	state;		/* +0x0c: 3 = SLAM tracking */
	__u8	substate;	/* +0x0d: 5 = valid as well */
	__u8	valid_flag;	/* +0x0e: 1 = valid as well */
	__u8	reset_count;	/* +0x1e0: changes on a SLAM reset */
	__le32	velocity[3];	/* +0x48: m/s, world frame */
	__le32	gyro_bias[3];	/* +0x54: rad/s */
	__le32	accel_bias[3];	/* +0x60: m/s^2 */
	__le32	error_variance;	/* +0x184 */
	__u32	map_id;		/* +0x1e4: upper 16 bits change = map latch */
};

#define PSVR2_POSE_FLAG_VALID	(1u << 0)	/* well-formed SLP record */
/*
 * timestamp_ns is the headset capture time (device_vts_us mapped onto
 * CLOCK_MONOTONIC through the 0xde clock sync) rather than the receive time.
 */
#define PSVR2_POSE_FLAG_DEVICE_TIME	(1u << 1)
/* 6DoF valid by Sony's rule: state 3, or valid_flag 1, or substate 5. */
#define PSVR2_POSE_FLAG_TRACKING	(1u << 2)

/*
 * Eye / gaze tracking. The character device /dev/psvr2-gaze delivers a stream
 * of struct psvr2_gaze_sample records (read() returns whole samples; poll()
 * signals POLLIN). Only the well-understood fields are surfaced.
 *
 * All vectors/scalars are IEEE-754 little-endian floats carried as raw bits.
 * Positions and pupil diameter are in millimetres, in the device's native
 * wire frame (Monado negates x and z — apply in userspace if desired).
 * Directions are unnormalised gaze vectors. Each field has a *_valid flag.
 */
struct psvr2_gaze_eye {
	__u32	gaze_point_valid;
	__le32	gaze_point_mm[3];
	__u32	gaze_direction_valid;
	__le32	gaze_direction[3];
	__u32	pupil_diameter_valid;
	__le32	pupil_diameter_mm;
	__u32	blink_valid;
	__u32	blink;			/* 0 = open, 1 = closed */
};

struct psvr2_gaze_combined {
	__u32	gaze_point_valid;
	__le32	gaze_point_mm[3];
	__u32	gaze_direction_valid;
	__le32	gaze_direction[3];	/* normalised gaze */
};

struct psvr2_gaze_sample {
	__u64	timestamp_ns;		/* host CLOCK_MONOTONIC receive time */
	__u32	device_timestamp_us;	/* device sample timestamp */
	__u32	flags;			/* PSVR2_GAZE_FLAG_* */
	struct psvr2_gaze_eye		left;
	struct psvr2_gaze_eye		right;
	struct psvr2_gaze_combined	combined;
};

#define PSVR2_GAZE_FLAG_VALID	(1u << 0)	/* well-formed "GS" packet */

/*
 * Raw tracking streams. /dev/psvr2-ld (LED detector, IF8, "LD" packets of up
 * to 0x9050 bytes) and /dev/psvr2-rp (relocaliser keyframes, IF9, "RP"
 * packets of 0xc8780 bytes) each return one packet per read(): a struct
 * psvr2_packet_hdr followed by `length` payload bytes. The buffer must hold
 * the whole packet, else read() fails with EMSGSIZE and the packet stays
 * queued. The payload is the headset's wire format unchanged; see
 * docs/sony-re/ledsync.md and docs/sony-re/room.md.
 */
struct psvr2_packet_hdr {
	__u64	timestamp_ns;	/* host CLOCK_MONOTONIC receive time */
	__u32	length;		/* payload bytes that follow */
	__u32	sequence;	/* +1 per packet received from the headset */
	__u32	dropped;	/* packets lost to a full queue since the last read */
	__u32	reserved;
};

#define PSVR2_LD_MAX_LEN	0x9050
#define PSVR2_RP_LEN		0xc8780

/*
 * /dev/psvr2-ctrl: vendor control reports (ep0) and the headset clock map.
 *
 * PSVR2_IOC_SET / PSVR2_IOC_GET send one report with the 8-byte header the
 * headset expects. Only the reports Sony's driver uses are accepted
 * (EPERM otherwise), so a stray program cannot reach firmware-update or
 * factory commands. For GET, `len` bytes of the reply are copied to data[].
 */
#define PSVR2_CTRL_MAX_LEN	504

struct psvr2_ctrl_report {
	__u16	report_id;
	__u16	subcmd;		/* Sony sends 1, or 2 for report 0x05 */
	__u16	len;		/* bytes of data[] used, <= PSVR2_CTRL_MAX_LEN */
	__u16	reserved;
	__u8	data[PSVR2_CTRL_MAX_LEN];
};

/*
 * Headset clock map from the 0xde clock sync (every 500 ms). For a device
 * time d (us, 32-bit, wraps):
 *   ext  = ref_device_us + (__s32)(d - (__u32)ref_device_us)
 *   host = ext * 1000 + offset_ns + (ext - ref_device_us) * drift_ppb / 1e6
 * gives CLOCK_MONOTONIC ns. valid is 0 until enough samples arrived, or when
 * the firmware does not support the 0xde report.
 */
struct psvr2_clock_map {
	__u64	ref_device_us;	/* 64-bit extended device time of the fit */
	__s64	offset_ns;	/* host_ns - device_us * 1000 at ref */
	__s32	drift_ppb;	/* host clock rate minus device clock rate */
	__u32	valid;
	__u32	samples;	/* accepted 0xde samples in the window */
	__u32	rtt_us;		/* round trip of the last accepted sample */
};

#define PSVR2_IOC_MAGIC		'P'
#define PSVR2_IOC_SET		_IOW(PSVR2_IOC_MAGIC, 1, struct psvr2_ctrl_report)
#define PSVR2_IOC_GET		_IOWR(PSVR2_IOC_MAGIC, 2, struct psvr2_ctrl_report)
#define PSVR2_IOC_CLOCK		_IOR(PSVR2_IOC_MAGIC, 3, struct psvr2_clock_map)

#endif /* _UAPI_PSVR2_H_ */
