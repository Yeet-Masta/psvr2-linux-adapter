/* SPDX-License-Identifier: GPL-2.0 */
/*
 * libpsvr2 — userspace access to the PSVR2 kernel module's device nodes.
 *
 * Wraps the raw interfaces exposed by the psvr2 module (the /dev/psvr2-pose and
 * /dev/psvr2-gaze character devices, the psvr2_imu IIO device, the V4L2 camera
 * node, and the brightness sysfs attribute) behind a small C API that returns
 * host-native floats. Wire data is little-endian IEEE-754; this library does the
 * reinterpretation and IIO scaling for you.
 *
 * Copyright (C) 2026 PSVR2 Linux project
 */
#ifndef LIBPSVR2_H
#define LIBPSVR2_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct psvr2 psvr2_t;

/* 6DoF pose from the headset's onboard tracker, in the device's native frame. */
struct psvr2_pose {
	uint64_t timestamp_ns;		/* host CLOCK_MONOTONIC */
	uint32_t device_vts_us;
	int	 valid;
	float	 position[3];		/* metres */
	float	 orientation[4];	/* quaternion: w, x, y, z */
	int	 device_time;		/* timestamp_ns is the capture time */
	int	 tracking;		/* 6DoF valid by Sony's rule */
	int	 state, substate, reset_count;
	uint32_t map_id;
	float	 velocity[3];		/* m/s, raw frame like position */
	float	 gyro_bias[3];		/* rad/s */
	float	 accel_bias[3];		/* m/s^2 */
	float	 error_variance;
};

struct psvr2_eye {
	int	gaze_point_valid;
	float	gaze_point_mm[3];
	int	gaze_direction_valid;
	float	gaze_direction[3];
	int	pupil_diameter_valid;
	float	pupil_diameter_mm;
	int	blink_valid;
	int	blink;			/* 0 = open, 1 = closed */
};

struct psvr2_gaze {
	uint64_t timestamp_ns;
	uint32_t device_timestamp_us;
	int	 valid;
	struct psvr2_eye left;
	struct psvr2_eye right;
	struct {
		int	gaze_point_valid;
		float	gaze_point_mm[3];
		int	gaze_direction_valid;
		float	gaze_direction[3];	/* normalised */
	} combined;
};

/* Latest scaled IMU sample (m/s^2 and rad/s), device-native axes. */
struct psvr2_imu {
	float accel_m_s2[3];
	float gyro_rad_s[3];
};

/*
 * Open the headset. Discovers whichever interfaces the module has exposed;
 * missing ones simply make the corresponding calls return -1 / no data.
 * Returns NULL only on allocation failure (a headset with no nodes still opens).
 */
psvr2_t *psvr2_open(void);
void psvr2_close(psvr2_t *p);

/* True (1) if the named interface was found. */
int psvr2_has_pose(const psvr2_t *p);
int psvr2_has_gaze(const psvr2_t *p);
int psvr2_has_imu(const psvr2_t *p);

/* Path of the V4L2 camera node (e.g. "/dev/video3"), or NULL if not found. */
const char *psvr2_camera_path(const psvr2_t *p);

/* File descriptors for poll()/select(), or -1 if unavailable. */
int psvr2_pose_fd(const psvr2_t *p);
int psvr2_gaze_fd(const psvr2_t *p);

/*
 * Read one sample. With block != 0 the call waits for data; otherwise it returns
 * 0 immediately if none is pending. Returns 1 on a sample, 0 if none (nonblock),
 * -1 on error.
 */
int psvr2_read_pose(psvr2_t *p, struct psvr2_pose *out, int block);
int psvr2_read_gaze(psvr2_t *p, struct psvr2_gaze *out, int block);

/* Read the latest IMU sample (scaled). Returns 0 on success, -1 on error. */
int psvr2_read_imu(psvr2_t *p, struct psvr2_imu *out);

/*
 * Buffered IMU stream: every 2 kHz sample with its capture time, through
 * the IIO kfifo. Raw counts in the device-native axes (scale them with
 * psvr2_imu_scales(), or with the factory calibration, see psvr2_ins.h).
 */
struct psvr2_imu_raw {
	int16_t accel[3];
	int16_t gyro[3];
	int64_t timestamp_ns;		/* host CLOCK_MONOTONIC */
};

/*
 * Enable the IIO buffer (needs write access to its sysfs attributes, see
 * kernel/99-psvr2.rules) and open the buffer node. Returns the fd (poll it
 * for POLLIN) or -1.
 */
int psvr2_imu_stream_open(psvr2_t *p);
int psvr2_imu_stream_fd(const psvr2_t *p);
/* Non-blocking: up to max samples; 0 if none pending, -1 on error. */
int psvr2_imu_stream_read(psvr2_t *p, struct psvr2_imu_raw *out, int max);
void psvr2_imu_stream_close(psvr2_t *p);
/* IIO scales: m/s^2 and rad/s per count. */
void psvr2_imu_scales(const psvr2_t *p, double *accel, double *gyro);

/* Headset controls snapshot from the module's input device. */
struct psvr2_controls {
	int ipd_mm;		/* IPD dial, 59..72 */
	int worn;		/* proximity sensor: 1 = on a head */
	int function_button;	/* 1 = pressed */
};

/* Read the current controls. Returns 0 on success, -1 if unavailable. */
int psvr2_read_controls(psvr2_t *p, struct psvr2_controls *out);

/* Set panel brightness, 0..31. Returns 0 on success, -1 on error. */
int psvr2_set_brightness(psvr2_t *p, int level);

/*
 * Run the headset rumble motor at 0..25 Hz (0 = off). Only a jailbroken
 * headset (vr2jb) acts on it. Returns 0 on success, -1 on error.
 */
int psvr2_set_rumble(psvr2_t *p, int hz);

/*
 * Per-unit lens distortion parameters for psvr2_distort(): per-eye centre
 * offsets (left x, y, right x, y) and rotations (left cos, sin, right cos,
 * sin). Fills defaults when the block is missing. Returns 0 when read from
 * the headset, 1 for an old-format block (defaults), -1 on error (defaults).
 */
int psvr2_get_lens_distortion(psvr2_t *p, float out[8]);

/*
 * Sony's lens model (docs/sony-re/steamvr-and-display.md §B.5.2): maps the output-panel
 * coordinate (u, v) in [0, 1] of eye 0 (left) or 1 (right) to the render
 * coordinate to sample, per colour channel: out[0..1] red, [2..3] green,
 * [4..5] blue.
 */
void psvr2_distort(const float params[8], int eye, float u, float v,
		   float out[6]);

/*
 * Hidden-area mesh for eye 0/1 (Prop_DisplayHiddenArea_Binary_Start + eye):
 * triangles in texture space (0..1), as Sony's driver builds it. Returns the
 * vertex count (108).
 */
#define PSVR2_HIDDEN_AREA_VERTS	108
int psvr2_hidden_area(const float params[8], int eye,
		      float verts[PSVR2_HIDDEN_AREA_VERTS][2]);

/* ---- control reports (/dev/psvr2-ctrl) --------------------------------- */

/*
 * Send / read one vendor control report. Only the reports Sony's driver uses
 * are accepted by the kernel. Returns 0 on success, -1 on error (errno set).
 */
int psvr2_ctrl_set(psvr2_t *p, int report_id, int subcmd, const void *data,
		   unsigned int len);
int psvr2_ctrl_get(psvr2_t *p, int report_id, int subcmd, void *data,
		   unsigned int len);

/* Headset clock map from the kernel's 0xde clock sync. */
struct psvr2_clock {
	uint64_t ref_device_us;
	int64_t	 offset_ns;
	int32_t	 drift_ppb;
	int	 valid;
	unsigned samples;
	unsigned rtt_us;
};
int psvr2_get_clock(psvr2_t *p, struct psvr2_clock *out);
/* Device us (32-bit, wrapping) to CLOCK_MONOTONIC ns through a clock map. */
int64_t psvr2_clock_to_host_ns(const struct psvr2_clock *c, uint32_t dev_us);

/* SteamVR quit (Set 0x27 u16 1000), sent by Sony's driver on quit. */
int psvr2_send_quit(psvr2_t *p);

/* See-through (passthrough on the headset itself). 0/1, -1 on error. */
int psvr2_get_see_through(psvr2_t *p);
int psvr2_set_see_through(psvr2_t *p, int on);

/*
 * EDID type: 0 = 90/120 Hz modes, 1 = 90 Hz only. After a change the
 * DisplayPort sink re-enumerates; wait about a second before using it.
 */
int psvr2_get_edid_type(psvr2_t *p);
int psvr2_set_edid_type(psvr2_t *p, int type);

/* Values read by the kernel's init sequence (sysfs, hex). 0 if unknown. */
uint32_t psvr2_firmware_version(psvr2_t *p);
uint32_t psvr2_min_driver_version(psvr2_t *p);
uint32_t psvr2_board_stage(psvr2_t *p);
/* 1 once the headset finished init on a SuperSpeed link. */
int psvr2_ready(psvr2_t *p);

/*
 * Factory calibration blob from the headset ("camera_calib0",
 * "camera_calib1", "imu_calib", "imu_set_calib"). Returns the byte count
 * (0 if not received yet) or -1.
 */
int psvr2_read_calibration(psvr2_t *p, const char *name, void *buf,
			   unsigned int len);

/* ---- Tracking-camera calibration (VD types 10/11) --------------------- */

#define PSVR2_CAM_MAX		4
#define PSVR2_CAM_RAW		20
#define PSVR2_CAM_EXT_MAX	8

struct psvr2_cam_intrinsics {
	unsigned int id, width, height;
	double	K[9];			/* 3x3, row-major [likely] */
	double	raw[PSVR2_CAM_RAW];	/* distortion record, order unknown */
};

struct psvr2_cam_extrinsic {
	unsigned int from, to;
	float	R[9];			/* row-major */
	float	t[3];
};

struct psvr2_cam_calib {
	unsigned int version, model_d;	/* model_d: 0xe4-byte (double) records */
	char	model[9];
	unsigned int ncam, next;
	struct psvr2_cam_intrinsics cam[PSVR2_CAM_MAX];
	struct psvr2_cam_extrinsic ext[PSVR2_CAM_EXT_MAX];
};

/* Parse a "SIE" camera-calibration blob. 0 = ok, -1 = malformed. */
int psvr2_cam_calib_parse(const void *blob, unsigned int len,
			  struct psvr2_cam_calib *out);

/*
 * Tracking-camera model (docs/sony-re/overview.md §A.8). OpenCV's rational model with
 * two more orders in numerator and denominator, tangential p1/p2 and tilt
 * tau_x/tau_y; thin-prism s1..s4 exist in the calibration but Sony ignores
 * them. Intrinsics are rescaled to a 508 x 508 model image, as Sony does.
 *
 * Points are in the OpenCV camera frame (+Z forward, +x right, +y down),
 * mm or any unit. Sony's tracker frame is this rotated 180 degrees about Z:
 * use psvr2_cam_project_rig() for points in that frame.
 */
#define PSVR2_CAM_MODEL_SIZE	508

struct psvr2_cam_model {
	double	fx, fy, cx, cy;		/* in the 508 x 508 model image */
	double	num[6], den[6];		/* r^2 .. r^12 coefficients */
	double	p1, p2, tau_x, tau_y;
	double	tilt[9];		/* row-major, from tau_x / tau_y */
	unsigned int width, height;	/* 508, 508 */
};

/* Build camera `index` of a parsed calibration. 0 = ok, -1 = no such camera. */
int psvr2_cam_model_from_calib(const struct psvr2_cam_calib *cal, unsigned int index,
			       struct psvr2_cam_model *m);

/* Project an OpenCV camera-frame point. 0 = ok, -1 = behind / out of model. */
int psvr2_cam_project(const struct psvr2_cam_model *m, const double X[3],
		      double px[2]);
/* Same for a point in Sony's tracker camera frame (X, Y negated). */
int psvr2_cam_project_rig(const struct psvr2_cam_model *m, const double X[3],
			  double px[2]);

/*
 * Calibration extrinsic -> Sony rig convention: R' = G R G, t' = 1000 G t
 * (mm), G = diag(-1, -1, 1). R' row-major.
 */
void psvr2_cam_extrinsic_to_rig(const struct psvr2_cam_extrinsic *e,
				double R[9], double t_mm[3]);

/* ---- LED detector stream (/dev/psvr2-ld) ------------------------------- */

#define PSVR2_LD_PACKET_MAX	0x9050

struct psvr2_ld_packet {
	uint64_t timestamp_ns;		/* host receive time */
	uint32_t sequence;
	uint32_t dropped;
	uint32_t length;
	uint8_t	 data[PSVR2_LD_PACKET_MAX];	/* "LD" packet, wire format */
};

int psvr2_ld_fd(const psvr2_t *p);
/* 1 = packet read, 0 = none (non-blocking), -1 = error / no device. */
int psvr2_read_ld(psvr2_t *p, struct psvr2_ld_packet *out, int block);

#ifdef __cplusplus
}
#endif

#endif /* LIBPSVR2_H */
