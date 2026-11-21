// SPDX-License-Identifier: GPL-2.0
// Controller pose from the headset's LED blobs (docs/sony-re/opticalpose.md).
//
// Per LD frame, for every controller whose LEDs are synced (STABLE):
//   1. place the camera rig in the world: head pose at the frame time,
//      then a fixed head <- rig transform;
//   2. solve the controller's LED model against the blobs
//      (psvr2_led_solver): track from a prediction, else boot from the IMU
//      orientation;
//   3. hand the controller its IMU's world position and orientation
//      (Psvr2ControllerDriver::OpticalFix), which takes the position and
//      the heading from it.
//
// Rig: camera 0 at Sony's built-in rig pose (variant 2, which matches the
// hardware calibration to 3 mm / 2 deg), cameras 1-3 from the calibration
// extrinsics, where an extrinsic "a -> b" is b <- a (checked on hardware,
// 2026-11-18). Rig frame: mm, +x wearer's left, +y up, +z forward.
//
// Own method, not Sony's (not traced): the rotation between the IMU frame
// our fusion uses and the LED-model frame is not known for sure, so until it
// is learned the boot also tries all 24 axis-aligned rotations. A guess is
// kept once 30 tracked frames over 30 deg of controller rotation agree with
// gravity; it is then saved in the settings. The right hand defaults to the
// rotation fitted on hardware; the left hand uses Sony's x-mirrored model.
#pragma once

#include <cstdint>
#include <cstdio>

#include "libpsvr2.h"
#include "psvr2_led_solver.h"
#include "sie_c.h"
#include "vr_math.h"

class Psvr2ControllerDriver;
class Psvr2HmdDriver;

class OpticalPose
{
public:
	// controllers[0] = left, [1] = right.
	OpticalPose( Psvr2HmdDriver *hmd, Psvr2ControllerDriver *const controllers[2] );
	~OpticalPose();

	// Build the rig from the camera calibration; true once it exists.
	bool BuildRig( psvr2_t *dev );

	// One LD frame. synced[hand]: that controller's LEDs are lit (STABLE).
	void Frame( uint64_t frame_us, const psvr2_blob_set blobs[PSVR2_LD_CAMERAS], const int counts[PSVR2_LD_CAMERAS],
	            const bool synced[2] );

	// Frame time of the last fix handed to controller `hand`; 0 = none.
	uint64_t LastFixUs( int hand ) const { return hands_[hand].fix_us; }

	// LED-sync input from the pose solve (Sony feeds LED sync from its
	// pose association, FUN_1801999f0): true when `hand` was solved in
	// frame `frame_us`. Then, per camera and model LED, the weight of the
	// matched blob (< 0 = no blob) and the LED's view angle (deg, 0 =
	// face-on).
	bool HasSyncView( int hand, uint64_t frame_us ) const
	{
		return hands_[hand].view_us && hands_[hand].view_us == frame_us;
	}
	uint64_t LastViewUs( int hand ) const { return hands_[hand].view_us; }
	int LedCount( int hand ) const { return hands_[hand].rev >= 0 ? hands_[hand].model[0].count : 0; }
	float ViewWeight( int hand, int cam, int led ) const { return hands_[hand].view_w[cam][led]; }
	float ViewAngle( int hand, int cam, int led ) const { return hands_[hand].view_deg[cam][led]; }

private:
	struct Hand
	{
		int rev = -1;
		psvr2_led_model model[2];  // [0] as stored, [1] mirrored in x
		// Learned IMU -> model rotation (index into the 24); -1 = unknown.
		// mirror: model index, 1 (x-mirrored) for the left hand.
		int remap = -1, mirror = 0;
		unsigned hyp = 0;  // next boot hypothesis
		// Tracking state. Candidate remap while not yet confirmed.
		bool tracking = false;
		int cand_remap = 0;
		vrmath::Pose last;  // world <- model, metres
		uint64_t last_us = 0;
		vrmath::Vec3 vel;
		vr::HmdQuaternion_t q_last = vrmath::Identity();  // fused IMU at last
		int fails = 0, good = 0;
		uint64_t fix_us = 0;  // frame time of the last fix handed to the controller
		// Sync view of the last solved frame (HasSyncView).
		uint64_t view_us = 0;
		float view_w[PSVR2_SOLVER_CAMS][PSVR2_LED_MAX] = {};
		float view_deg[PSVR2_SOLVER_CAMS][PSVR2_LED_MAX] = {};
		vr::HmdQuaternion_t span_ref = vrmath::Identity();
		double span = 0;
		// Stats for the periodic log.
		int frames = 0, solved = 0, boots = 0, fixes = 0, matched = 0;
		int reboots = 0;  // tracked frames where a boot replaced the track
		int p1p = 0;      // consecutive one-LED (P1P) tracks; > 5 stops them (Sony)
		double rms = 0;
	};

	// wt: blob weight per list entry, kept parallel to lists.
	void SolveHand( int hand, uint64_t frame_us, const vrmath::Pose &world_from_rig,
	                psvr2_blob_list lists[PSVR2_SOLVER_CAMS], float wt[PSVR2_SOLVER_CAMS][PSVR2_SOLVER_BLOBS] );
	bool Boot( int hand, Hand &h, uint64_t frame_us, const vrmath::Pose &world_from_rig, const vr::HmdQuaternion_t &q_imu, double yaw_now,
	           const psvr2_blob_list lists[PSVR2_SOLVER_CAMS], psvr2_pose3 &out, psvr2_solve_result &res,
	           bool quiet = false );
	void Accept( int hand, Hand &h, uint64_t frame_us, const vrmath::Pose &world_from_rig, const psvr2_pose3 &rig_pose,
	             const vr::HmdQuaternion_t &q_imu, const psvr2_solve_result &res );
	void LoadHand( int hand );
	void Confirm( int hand, Hand &h );
	vrmath::Pose HeadFromRig() const;

	Psvr2HmdDriver *hmd_;
	Psvr2ControllerDriver *controllers_[2];
	Hand hands_[2];
	bool have_rig_ = false;
	psvr2_rig_camera cams_[PSVR2_SOLVER_CAMS] = {};
	psvr2_solver_cfg cfg_{};

	// head <- rig (settings optical_rig_offset_x/y/z in m, head axes, and
	// optical_rig_tilt_deg about the head's x axis).
	vrmath::Vec3 rig_offset_;
	double rig_tilt_deg_ = 0.0;

	uint64_t log_at_us_ = 0;
	FILE *pose_log_ = nullptr;  // setting optical_pose_log
	uint64_t pose_log_until_us_ = 0;
};
