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

#include "libpsvr2.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PSVR2_LED_MAX		24
#define PSVR2_SOLVER_CAMS	4
#define PSVR2_SOLVER_BLOBS	64

struct psvr2_led_model {
	int	count;
	double	cone_deg;		/* 170 (rev 1-3) or 180 (rev 4-7) */
	double	pos[PSVR2_LED_MAX][3];	/* mm, controller frame */
	double	nrm[PSVR2_LED_MAX][3];	/* unit */
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
};

struct psvr2_solve_result {
	int	matched;			/* LED-blob pairs used */
	int	expected;		/* predicted LEDs facing a camera */
	double	score;			/* MSAC / T: -matched (perfect) .. 0 */
	double	rms_px;
	int	led_blob[PSVR2_SOLVER_CAMS][PSVR2_LED_MAX];	/* -1 = none */
};

/* Boot config values (RESEARCH.md §9.2), as Sony ships them. */
struct psvr2_solver_cfg {
	double	search_radius_px;	/* SEARCHRADIUS 3, scaled by 1/distance(m) */
	double	pothole;		/* POTHOLETHRESHOLD 1 (px^2), scaled */
	int	min_inliers_boot;	/* 7 */
	int	min_inliers_orientation;/* 4 */
	int	max_iterations;		/* LM, 250 */
};
void psvr2_solver_default_cfg(struct psvr2_solver_cfg *cfg);

/*
 * Track from a predicted pose: associate, match, refine. 0 = pose found
 * (out, res filled), -1 = fewer than 3 matches, the refine failed, or the
 * result fails the acceptance check (res still filled; boot again).
 */
int psvr2_led_track(const struct psvr2_solver_cfg *cfg,
		    const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS],
		    const struct psvr2_blob_list blobs[PSVR2_SOLVER_CAMS],
		    const struct psvr2_led_model *model,
		    const struct psvr2_pose3 *predicted,
		    struct psvr2_pose3 *out, struct psvr2_solve_result *res);

/*
 * Boot with a known orientation (from the controller IMU): try LED pairs x
 * blob pairs per camera, solve the translation from the two rays, score in
 * every camera, refine the best. 0 = pose found.
 */
int psvr2_led_boot_oriented(const struct psvr2_solver_cfg *cfg,
			    const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS],
			    const struct psvr2_blob_list blobs[PSVR2_SOLVER_CAMS],
			    const struct psvr2_led_model *model,
			    const double R_prior[9],
			    struct psvr2_pose3 *out, struct psvr2_solve_result *res);

/*
 * Fill `model` with Sony's LED model for Sense hardware rev `rev` (feature
 * 0x20 +0x18; 1..6, anything else = the "other" model). Both hands use the
 * same model; what Sony's hand-1 descriptor flag changes is not traced
 * (docs/RESEARCH.md §11), so no mirroring is applied.
 */
void psvr2_led_model_load(int rev, struct psvr2_led_model *model);

/* Project LED `led` of a pose into camera `cam`; 0 = visible, -1 = not. */
int psvr2_led_project(const struct psvr2_rig_camera *cam,
		      const struct psvr2_led_model *model,
		      const struct psvr2_pose3 *pose, int led,
		      double px[2], double *view_deg, double *depth_mm);

#ifdef __cplusplus
}
#endif
#endif
