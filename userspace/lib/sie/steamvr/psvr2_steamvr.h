/* SPDX-License-Identifier: GPL-2.0 */
/*
 * SteamVR-facing behaviour of the PS VR2 headset driver: lens geometry,
 * hidden-area mesh, display timing, HMD properties.
 *
 * Written from the behavioural spec in docs/sony-re/steamvr.md and checked
 * against the reference driver by scripts/sony-re/difftest/t_steamvr_*.c.
 */
#ifndef PSVR2_SIE_STEAMVR_H
#define PSVR2_SIE_STEAMVR_H
#include <stdint.h>

/* ---- Lens geometry from the 256-byte display calibration block ---- */

typedef struct {
	float centre_l[2];   /* left lens centre, normalised (x, y) */
	float centre_r[2];   /* right lens centre, normalised (x, y) */
	float rot_l[2];      /* left panel rotation (cos, sin) */
	float rot_r[2];      /* right panel rotation (cos, sin) */
} psvr2_lens_geom;

typedef struct {
	uint8_t calib[256];  /* last block seen; all-zero initially */
	psvr2_lens_geom geom;
} psvr2_lens_state;

/* Feed the current calibration block. Recomputes geom only when the block
 * differs from the last one; returns 1 if it did. */
int psvr2_steamvr_lens_update(psvr2_lens_state *st, const uint8_t calib[256]);

/* ---- Hidden-area mesh (Prop_DisplayHiddenArea_Binary_Start + eye) ---- */

#define PSVR2_HIDDEN_AREA_MAX 108  /* vertices: 32 segments + 4 corner fills, 3 each */

/* lut: radial distortion scale indexed by r^2 * 1023 (Sony ships it in the
 * driver; this project does not carry it, see docs/sony-re/steamvr.md).
 * Builds the left-eye mesh in v (pairs x, y in 0..1 texture space); returns the
 * vertex count. The right-eye mesh is the left one with x mirrored (1 - x).
 * v must persist between calls: a ray that leaves the lens on its first step
 * keeps the previous value, as the reference driver does. */
#define PSVR2_LENS_LUT_SIZE 1024
int psvr2_steamvr_hidden_area(const psvr2_lens_geom *g, const float lut[PSVR2_LENS_LUT_SIZE],
			      float v[PSVR2_HIDDEN_AREA_MAX][2]);
void psvr2_steamvr_hidden_area_mirror(float v[][2], int n);

/* ---- IVRDisplayComponent ---- */

#define PSVR2_PANEL_W 4000  /* both eyes, side by side */
#define PSVR2_PANEL_H 2040

typedef struct { uint32_t x, y, w, h; } psvr2_rect;

psvr2_rect psvr2_steamvr_window_bounds(uint32_t panel_w, uint32_t panel_h);
void psvr2_steamvr_render_target(uint32_t panel_w, uint32_t panel_h, uint32_t *w, uint32_t *h);
psvr2_rect psvr2_steamvr_eye_viewport(uint32_t panel_w, uint32_t panel_h, int eye);
/* Tangents of the half-angles: left, right, top, bottom. */
void psvr2_steamvr_projection_raw(int eye, float *l, float *r, float *t, float *b);

/* DistortionCoordinates_t: red, green, blue (u, v) each. Three 1024-entry
 * radial tables (red, green, blue; same indexing as the hidden-area table,
 * which is the green one). eye 0 = left. */
typedef struct { float rgb[3][2]; } psvr2_distortion;
psvr2_distortion psvr2_steamvr_distort(const psvr2_lens_geom *g, const float *lut_r,
				       const float *lut_g, const float *lut_b,
				       int eye, float u, float v);

/* ---- Property writes (ETrackedDeviceProperty, PropertyTypeTag_t) ---- */

enum {
	PSVR2_TAG_FLOAT = 1, PSVR2_TAG_INT32 = 2, PSVR2_TAG_BOOL = 4,
	PSVR2_TAG_STRING = 5, PSVR2_TAG_HIDDEN_AREA = 30,
};

typedef struct {
	uint32_t prop, tag, size;
	const void *data;     /* points into inl, a string literal or the list */
	uint8_t inl[8];
} psvr2_prop;

typedef struct {
	int n;
	psvr2_prop p[40];
	char chaperone_path[4096];
} psvr2_prop_list;

/* ---- HMD Activate ---- */

typedef struct {
	/* driver debug config (Sony's ini); 0 = use default */
	uint32_t edid_product_id;     /* 0: Sony default 0xc207 */
	int32_t vsync_to_photons_us;  /* both < 1: built-in timing per refresh rate */
	int32_t photons_to_vblank_us;
	/* headset */
	uint32_t hmd_sw_version;      /* > 0x06000000: 120 Hz capable */
	uint8_t ipd_mm;               /* IPD dial position from the status report */
	/* SteamVR */
	float refresh_rate;           /* chosen at init (see psvr2_steamvr_edid_select) */
	float ipd_setting;            /* steamvr/ipd */
	const char *user_config_path; /* Prop_UserConfigPath_String; NULL if unavailable */
} psvr2_hmd_prop_in;

/* Properties the HMD writes in Activate, in Sony's order. The hidden-area mesh
 * (written in the middle of Activate, before Prop_DistortionMeshResolution) is
 * not included. */
void psvr2_steamvr_hmd_props(const psvr2_hmd_prop_in *in, psvr2_prop_list *out);

/* ---- Refresh rate / EDID type at init (ServerDriver) ---- */

enum { PSVR2_EDID_90_120 = 0, PSVR2_EDID_90_ONLY = 1 };

typedef struct {
	float rate;          /* rate the driver runs at (ServerDriver +0x88) */
	int store_setting;   /* write rate back to steamvr/preferredRefreshRate */
	int set_edid;        /* EDID type to send (Set 0x29), or -1 */
} psvr2_edid_choice;

/* preferred: steamvr/preferredRefreshRate; cur_edid: Get 0xa9 result (the
 * caller leaves it 0 if the read fails). After a set_edid the reference waits
 * 1 s and reports that the display mode changed. */
psvr2_edid_choice psvr2_steamvr_edid_select(float preferred, uint8_t cur_edid);

/* ---- Sense controller Activate ---- */

enum { PSVR2_SIDE_LEFT = 0, PSVR2_SIDE_RIGHT = 1 };

enum {
	PSVR2_COMP_BOOL, PSVR2_COMP_SCALAR, PSVR2_COMP_HAPTIC, PSVR2_COMP_SKELETON,
};

typedef struct {
	const char *path;
	int kind;
	int scalar_two_sided;     /* EVRScalarUnits: 0 one-sided, 1 two-sided */
	const char *skeleton;     /* skeleton path, for PSVR2_COMP_SKELETON */
	const char *base_pose;
	int slot;                 /* Sony's handle slot (qword index in the device) */
} psvr2_component;

#define PSVR2_SENSE_COMPONENTS 20

/* Properties in Sony's write order; out->chaperone_path unused. */
void psvr2_steamvr_sense_props(int side, psvr2_prop_list *out);
/* Input components in creation order; returns the count. */
int psvr2_steamvr_sense_components(int side, psvr2_component out[PSVR2_SENSE_COMPONENTS]);

/* ---- Sense controller pose (DriverPose_t) ---- */

/* vr::DriverPose_t, 0x118 bytes. */
typedef struct {
	double time_offset;
	double q_world_from_driver[4];       /* w, x, y, z */
	double world_from_driver[3];
	double q_driver_from_head[4];
	double driver_from_head[3];
	double pos[3], vel[3], acc[3];
	double q[4];
	double ang_vel[3], ang_acc[3];
	int32_t result;                      /* ETrackingResult: 200 = Running_OK */
	uint8_t valid, drift_in_yaw, head_model, connected;
} psvr2_driver_pose;

/* Controller pose from the tracker (0x50 bytes: position @0, quaternion
 * w,x,y,z @0xc, velocity @0x28, angular velocity @0x40, valid byte @0x4c). */
typedef struct {
	float pos[3];
	float q[4];
	float pad0[3];
	float vel[3];
	float pad1[3];
	float ang_vel[3];
	uint8_t valid, pad2[3];
} psvr2_tracker_pose;

/* connected: controller link up; hold: one of Sony's pause conditions is
 * active (pose then repeats the previous one with zero velocities);
 * invalidate: the first condition (it also clears poseIsValid).
 * prev is the previous report (Sony keeps it in the device). */
void psvr2_steamvr_sense_pose(int side, const psvr2_tracker_pose *t, int connected, int hold,
			      int invalidate, const psvr2_driver_pose *prev, psvr2_driver_pose *out);

/* ---- HMD pose ---- */

typedef struct {
	psvr2_driver_pose prev;       /* last report (Sony +0x28) */
	psvr2_driver_pose cache;      /* last pose built while ready (+0x140) */
	psvr2_driver_pose last_sent;  /* throttle reference (+0x2a8) */
	int64_t last_fresh_us;        /* time of last_sent (+0x3c0) */
} psvr2_hmd_pose_state;

typedef struct {
	uint8_t ready;          /* headset stream up (any non-zero value) */
	int paused;             /* dialog/see-through pause */
	int needs_update;       /* firmware newer than known: result 1 */
	int worn;               /* proximity sensor */
	int64_t t_sample_us;    /* time the tracker pose was sampled for */
	int64_t t_send_us;      /* time of the report */
	const psvr2_tracker_pose *trk;
	const float *head_q;    /* w, x, y, z: driver-from-head rotation */
	const float *head_t;    /* driver-from-head translation */
} psvr2_hmd_pose_in;

/* Builds the HMD report into out and advances st; returns 0 when Sony would
 * not send a report this pass (not worn and still for 500 ms). */
int psvr2_steamvr_hmd_pose(psvr2_hmd_pose_state *st, const psvr2_hmd_pose_in *in, psvr2_driver_pose *out);


/* Hand skeleton (31 OpenVR hand bones). The bind-pose table is two sides
 * (index = the side value given to the skeleton) of 31 rows of
 * {x, y, z, rot_x_deg, rot_y_deg, rot_z_deg}. */
#define PSVR2_SKEL_BONES 31

typedef struct psvr2_bone {
	float pos[3];
	float q[4];	/* w, x, y, z */
	float scale[3];
} psvr2_bone;

void psvr2_steamvr_skel_bind(int side, const float *table, psvr2_bone *out);


/* Per-frame controller record the skeleton reads (0x78 bytes, from the
 * input manager). Arrays are [left, right]; buttons use libpad bits. */
typedef struct psvr2_skel_input {
	uint32_t buttons;	/* 0x00 */
	uint8_t stick[2];	/* 0x04 x, y */
	uint8_t r06[2];
	uint8_t trigger[2];	/* 0x08 */
	uint8_t r0a[2];
	uint32_t touch;		/* 0x0c */
	uint8_t grip_prox[2];	/* 0x10 */
	uint8_t trig_prox[2];	/* 0x12 */
	uint8_t r14[0x2c];
	uint8_t valid;		/* 0x40 */
	uint8_t r41[0x37];
} psvr2_skel_input;
_Static_assert(sizeof(psvr2_skel_input) == 0x78, "skeleton input size");

void psvr2_steamvr_skel_open_thumb(int side, const psvr2_skel_input *in, int stick_centred,
				   psvr2_bone *off);
uint32_t psvr2_steamvr_skel_ctrl_fingers(int side, const psvr2_skel_input *in, const psvr2_skel_input *prev,
					 psvr2_bone *off);
void psvr2_steamvr_skel_ctrl_thumb(int side, const psvr2_skel_input *in, int stick_centred,
				   psvr2_bone *off);
void psvr2_steamvr_skel_open_curl(int side, const psvr2_skel_input *in, psvr2_bone *off);

#endif
