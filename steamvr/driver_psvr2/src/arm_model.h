// SPDX-License-Identifier: GPL-2.0
// Arm model used as the controller filter's no-optics measurement, after
// Sony's tracker (docs/sony-re/controller-tracking.md §F.3). Implementation
// is our own.
//
// Taken from Sony (PC path, §F.3.2-§F.3.4):
//   - shoulder at the full head pose (pitch and roll included) times
//     (+-0.175, -0.185, 0); hand 0 = right (+x);
//   - upper arm 0.28 m, forearm 0.27 m, grip to wrist 0.08 m along the
//     controller's +z, reach limit 1.0 m;
//   - elbow hint (+-0.4, -0.6, z_h), z_h from +0.4 (hand >= 0.4 m below the
//     head) to -0.3 (hand at head height or above);
//   - elbow hinge 0..160 deg;
//   - out-of-view test: >= 80 deg off the head's view axis pitched 5 deg
//     down, and within 1.0 m of the shoulder;
//   - the measurement is position only, sigma 0.005 m while the model agrees
//     with the filter within 0.1 m, else 0.05 m; it starts once the optical
//     fix is 0.3 s old or the out-of-view test passes after recent motion.
// Own (not Sony's): Sony runs a stateful forward-kinematics chain with
// joint limits and swing-twist splits (§F.3.4 step 4, §F.3.5). Here the
// forearm follows the controller's +z axis and the upper arm takes the
// bisector of the hint direction and the forearm's forward direction,
// which keeps the elbow inside the hinge range; no joint state is kept.
#pragma once

#include "vr_math.h"

namespace armmodel
{

struct Result
{
	vrmath::Vec3 shoulder, elbow, wrist, hand;  // world, m; hand = grip point
};

// head_pos / head_rot: world <- head. ctrl_rot: world <- controller grip
// frame (+z back). hand_y: the hand's current height estimate (world y),
// for the elbow hint. right: hand 0.
Result Solve( const vrmath::Vec3 &head_pos, const vr::HmdQuaternion_t &head_rot, const vr::HmdQuaternion_t &ctrl_rot,
              double hand_y, bool right );

// Sony's out-of-view test (0x1801b9560 / 0x1801b93f0).
bool OutOfView( const vrmath::Vec3 &head_pos, const vr::HmdQuaternion_t &head_rot, const vrmath::Vec3 &ctrl_pos,
                bool right );

constexpr double kSigmaAgree = 0.005, kSigmaDisagree = 0.05, kAgreeDist = 0.1;  // m
constexpr double kStaleS = 0.3;           // optical fix age that starts the model
constexpr double kMovedSpeed = 0.0125;    // m/s, "moved" for the out-of-view gate
constexpr double kMovedWindowS = 1.5;

}  // namespace armmodel
