/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Sense controller pose from headset LED blobs (docs/RESEARCH.md §9).
 *
 * Written from the behavioural description, not from Sony's code:
 * projection-driven association (view-angle cone, self-occlusion cull,
 * face-on first), gated nearest-blob matching, a weighted Levenberg-
 * Marquardt refine with Huber IRLS, an MSAC score, and the
 * orientation-prior 2-point boot.
 *
 * Frames: everything is in the headset "rig" frame, in mm. A camera's pose
 * is rig <- camera, with the camera frame in Sony's convention (OpenCV
 * rotated 180 degrees about Z; see psvr2_cam_project_rig). The controller
 * pose is rig <- controller (LED model frame).
 */
#ifndef PSVR2_LED_SOLVER_H
#define PSVR2_LED_SOLVER_H
#include <stdint.h>

#include "libpsvr2.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PSVR2_LED_MAX		24
#define PSVR2_SOLVER_CAMS	4
#define PSVR2_SOLVER_BLOBS	64
#define PSVR2_RING_MAX		16
#define PSVR2_RING_LEN		32

struct psvr2_led_model {
	int	count;
	double	cone_deg;		/* 170 (rev 1-3) or 180 (rev 4-7) */
	double	pos[PSVR2_LED_MAX][3];	/* mm, controller frame */
	double	nrm[PSVR2_LED_MAX][3];	/* unit */
	/* LED rings for the RANSAC boot's first sampler stage: LED index
	 * sequences, consecutive LEDs neighbours on the controller (model
	 * block words 198.., docs/sony-re/notdone.md 2.19). */
	int	rings;
	int	ring_len[PSVR2_RING_MAX];
	int	ring[PSVR2_RING_MAX][PSVR2_RING_LEN];
};

struct psvr2_rig_camera {
	int	active;
	struct psvr2_cam_model model;
	double	R[9], t[3];		/* rig <- camera, row-major, mm */
};

struct psvr2_pose3 {
	double	R[9], t[3];		/* rig <- controller, row-major, mm */
};

struct psvr2_blob_list {
	int	count;
	double	xy[PSVR2_SOLVER_BLOBS][2];	/* pixels in the 508 model image */
	/* Blob ellipse angles (LD record, rad; < 0 = undefined), only read
	 * when has_angle (the grazing check). */
	int	has_angle;
	double	angle[PSVR2_SOLVER_BLOBS];
	/* Blob ellipse major semi-axis (LD record, px; <= 0 = unknown). The
	 * few-LED solve skips blobs within major x pi/2 of the image border. */
	double	major[PSVR2_SOLVER_BLOBS];
};

struct psvr2_solve_result {
	int	matched;			/* LED-blob pairs used */
	int	expected;		/* predicted LEDs facing a camera */
	double	score;			/* MSAC / T: -matched (perfect) .. 0 */
	double	rms_px;
	/* 0 = full 6-DoF solve; 2 = P2P, 3 = P1P: rotation from the IMU
	 * orientation prior, translation from 2+ / 1 matched LED rays
	 * (Sony's method enum at tracker +0x4c94). */
	int	method;
	int	led_blob[PSVR2_SOLVER_CAMS][PSVR2_LED_MAX];	/* -1 = none */
};

/* Boot config values (RESEARCH.md §9.2), as Sony ships them. */
struct psvr2_solver_cfg {
	double	search_radius_px;	/* SEARCHRADIUS 3, scaled by 1/distance(m) */
	double	pothole;		/* POTHOLETHRESHOLD 1 (px^2), scaled */
	int	min_inliers_boot;	/* 7 */
	int	min_inliers_orientation;/* 4 */
	int	max_iterations;		/* LM, 250 */
	double	min_depth_mm;		/* LED 0 camera depth window, 5 */
	double	max_depth_mm;		/* 1500 (boot object +0x688/+0x68c) */
	/*
	 * Our acceptance check on top of Sony's (>= 3/4 of the facing LEDs
	 * matched, rms <= 1.5 px). 1 = on (default), 0 = Sony's rule only.
	 */
	int	own_accept;
	/* 1 = Sony's three-pass gated match with neighbour support
	 * (FUN_1800df5b0); 0 = ours (image-shift consensus, nearest blob). */
	int	sony_match;
	/* 1 = with <= 3 matched LEDs and an orientation prior, solve the
	 * translation with the IMU rotation (Sony FUN_1800dcf00, default). */
	int	few_led;
	/* 1 = in the close-range states, drop grazing LEDs whose blob ellipse
	 * is not across the LED normal (Sony FUN_1800e4ec0, default). */
	int	grazing_check;
	/* 1 = psvr2_led_track_state enters Sony's edge state when <= 3 LEDs
	 * are predicted well inside the images (FUN_1800db420, default);
	 * 0 = it stays in the normal track step there. */
	int	edge_state;
	/* Sony's RANSAC boot (FUN_1800c2960): INLIERTHRESHOLD 2 px (6 px when
	 * the sample is closer than 200 mm with blobs > 3 px), RANSACMAXSAMPLES
	 * 200, RANSAC-BOOTINTRACK-MAXSAMPLES 50. */
	double	inlier_px;
	int	ransac_samples;
	int	ransac_samples_in_track;
	/* RANSACRINGBOOTRATIO 0.5: share of RANSACMAXSAMPLES drawn from the
	 * LED rings (first sampler stage). */
	double	ransac_ring_ratio;
};
void psvr2_solver_default_cfg(struct psvr2_solver_cfg *cfg);

/*
 * IMU priors for Sony's pose check (FUN_1800d89f0). Gravity directions are
 * unit vectors: measured in the controller (LED model) frame, and expected
 * in the rig frame. The orientation prior is rig <- controller.
 */
struct psvr2_imu_prior {
	int	gravity_valid;
	double	g_model[3], g_rig[3];
	double	gravity_age_s;		/* checked only when < 10 s */
	int	orient_valid;
	double	R[9];
	double	orient_age_s;		/* checked only when < 5 s */
	double	spin_rad_s;		/* skipped at >= pi rad/s */
	int	p1p_frames;		/* consecutive P1P results so far (caller) */
};

/*
 * Sony's pose rejection, in its order: 0 = ok; 2 = gravity more than 20 deg
 * off; 1 = rotation more than 30 deg from the orientation prior; 3 = LED 0
 * outside [min_depth, max_depth] in a camera that matched LEDs (the minimum
 * is not applied when `close`). prior may be NULL (depth check only).
 */
int psvr2_led_pose_check(const struct psvr2_solver_cfg *cfg,
			 const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS],
			 const struct psvr2_led_model *model,
			 const struct psvr2_imu_prior *prior,
			 const struct psvr2_pose3 *pose,
			 const struct psvr2_solve_result *res, int close);

/*
 * Track from a predicted pose: associate, match, refine. 0 = pose found
 * (out, res filled), -1 = fewer than 3 matches, the refine failed, or the
 * result fails a check (res still filled; boot again).
 */
int psvr2_led_track(const struct psvr2_solver_cfg *cfg,
		    const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS],
		    const struct psvr2_blob_list blobs[PSVR2_SOLVER_CAMS],
		    const struct psvr2_led_model *model,
		    const struct psvr2_pose3 *predicted,
		    struct psvr2_pose3 *out, struct psvr2_solve_result *res);

/*
 * Sony's tracker states (FUN_1800db690), by the controller's distance from
 * the nearest camera (mm) and the IMU orientation prior:
 *   NORMAL: psvr2_led_track_ex. -> CLOSE at < 80, NEAR at < 130 (IMU set);
 *     -> EDGE first when <= 3 LEDs are predicted well inside the images.
 *   NEAR (-> NORMAL > 170, CLOSE < 80), CLOSE (-> NEAR > 120), EDGE
 *     (-> NEAR < 130 with the IMU, NORMAL at 6+ matches): one gated match
 *     at SEARCHRADIUS x scale, a nearest-blob fallback under 3 matches,
 *     the grazing check, then the few-LED solve or the refine.
 *   LOST: the last call failed; the next call runs as NORMAL (Sony
 *     re-boots; the caller decides that here).
 * *state is updated; returns as psvr2_led_track_ex.
 */
enum { PSVR2_TRACK_NORMAL = 0, PSVR2_TRACK_NEAR = 1, PSVR2_TRACK_CLOSE = 2, PSVR2_TRACK_EDGE = 3,
       PSVR2_TRACK_LOST = 4 };
/*
 * Per-controller tracker memory across frames (Sony's tracker object),
 * kept by the caller; zero it, or use psvr2_led_track_mem_boot after a
 * boot. prior->p1p_frames is not read when this is given.
 */
struct psvr2_track_mem {
	int	state;			/* PSVR2_TRACK_* (FUN_1800db690, +0x4c94..) */
	int	p1p_frames;		/* consecutive P1P results (+0x5dc) */
	int	few_mode;		/* 1 = IMU-rotation solve kept (+0x4c98) */
	int	few_good;		/* frames with >= 5 matches in it (+0x4c9c) */
	uint64_t prev[PSVR2_SOLVER_CAMS];	/* LEDs matched last frame */
};
int psvr2_led_track_state(const struct psvr2_solver_cfg *cfg,
			  const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS],
			  const struct psvr2_blob_list blobs[PSVR2_SOLVER_CAMS],
			  const struct psvr2_led_model *model,
			  const struct psvr2_pose3 *predicted,
			  const struct psvr2_imu_prior *prior, double dt_s,
			  struct psvr2_track_mem *mem,
			  struct psvr2_pose3 *out, struct psvr2_solve_result *res);
/* The state after a boot (FUN_1800d5070): CLOSE < 100 mm, NEAR < 150. */
int psvr2_led_boot_state(const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS], const struct psvr2_pose3 *pose);
/* Reset *mem after a boot of `pose`, matched as in res (may be NULL). */
void psvr2_led_track_mem_boot(struct psvr2_track_mem *mem, const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS],
			      const struct psvr2_pose3 *pose, const struct psvr2_solve_result *res);

/*
 * Cross-camera support of a pose (FUN_1800e27a0 / FUN_1800efd70): per
 * camera with >= 2 LEDs facing it (< 90 deg) and in the image, the
 * fraction matched in res times (1 - mean distance / radius_px x scale);
 * the mean over those cameras, 0 when none. Sony compares it with 0.5.
 * The radius (Sony's normaliser) is a guess: SEARCHRADIUS.
 */
double psvr2_led_support(const struct psvr2_solver_cfg *cfg,
			 const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS],
			 const struct psvr2_blob_list blobs[PSVR2_SOLVER_CAMS],
			 const struct psvr2_led_model *model,
			 const struct psvr2_pose3 *pose,
			 const struct psvr2_solve_result *res);

/*
 * Sony's track step (FUN_1800db980): first gate SEARCHRADIUS x scale x
 * 3 x (dt / 16.7 ms), refine; second gate 1.5 x POTHOLE x scale, >= 3
 * matches, refine; then psvr2_led_pose_check with `prior` (may be NULL),
 * then our acceptance check when cfg->own_accept. With 1-3 matches, an
 * orientation prior and the prediction < 700 mm away, Sony's few-LED solve
 * instead (res->method 2 or 3; P1P at most 6 frames in a row, the caller
 * counts them in prior->p1p_frames; pair estimates within 100 mm of the
 * prediction, weighted by their consistency). psvr2_led_track is this with dt = 1/60 s and
 * no prior.
 */
int psvr2_led_track_ex(const struct psvr2_solver_cfg *cfg,
		       const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS],
		       const struct psvr2_blob_list blobs[PSVR2_SOLVER_CAMS],
		       const struct psvr2_led_model *model,
		       const struct psvr2_pose3 *predicted,
		       const struct psvr2_imu_prior *prior, double dt_s,
		       struct psvr2_pose3 *out, struct psvr2_solve_result *res);

/*
 * Boot with a known orientation (from the controller IMU): try LED pairs x
 * blob pairs per camera, solve the translation from the two rays, score in
 * every camera, refine the best. 0 = pose found.
 */
/*
 * Sony's boot (FUN_1800d8b60 -> FUN_1800c2960), per camera: RANSAC over
 * 3 LEDs (one and two of its 4 nearest in the model) x 3 blobs (one and
 * two of its 4 nearest in the image), P3P, MSAC score in that camera;
 * the best pose over the cameras is refined in all of them. Every
 * hypothesis passes psvr2_led_pose_check (close = 1) first. prior: with
 * orient_valid, Sony mode 7 (only LEDs facing the camera under prior->R
 * are sampled and the sample's image direction must agree with it), else
 * mode 6; NULL = no IMU. in_track: booting while tracking (50 samples,
 * else 200). Below MININLIERSFORBOOT the pose needs gravity within 3 deg
 * (>= 5 inliers) or the orientation within 15 deg (>= 4), section 2.9 of
 * docs/sony-re/notdone.md. 0 = pose found.
 */
int psvr2_led_boot_ransac(const struct psvr2_solver_cfg *cfg,
			  const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS],
			  const struct psvr2_blob_list blobs[PSVR2_SOLVER_CAMS],
			  const struct psvr2_led_model *model,
			  const struct psvr2_imu_prior *prior, int in_track,
			  struct psvr2_pose3 *out, struct psvr2_solve_result *res);

int psvr2_led_boot_oriented(const struct psvr2_solver_cfg *cfg,
			    const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS],
			    const struct psvr2_blob_list blobs[PSVR2_SOLVER_CAMS],
			    const struct psvr2_led_model *model,
			    const double R_prior[9],
			    struct psvr2_pose3 *out, struct psvr2_solve_result *res);

/*
 * Fill `model` with Sony's LED model for Sense hardware rev `rev` (feature
 * 0x20 +0x18; 1..6, anything else = the "other" model), as stored (right
 * hand). Sony mirrors it in x for the left hand (docs/sony-re/opticalpose.md
 * §2b.2); the caller does that (steamvr optical_pose.cpp), not this loader.
 */
void psvr2_led_model_load(int rev, struct psvr2_led_model *model);

/* Sony's left-hand model (FUN_1800d3ae0, mirror 'X'): x of positions and
 * normals negated, every LED ring reversed (a mirror flips its winding). */
void psvr2_led_model_mirror_x(struct psvr2_led_model *model);

/* Project LED `led` of a pose into camera `cam`; 0 = visible, -1 = not. */
int psvr2_led_project(const struct psvr2_rig_camera *cam,
		      const struct psvr2_led_model *model,
		      const struct psvr2_pose3 *pose, int led,
		      double px[2], double *view_deg, double *depth_mm);

#ifdef __cplusplus
}
#endif
#endif
