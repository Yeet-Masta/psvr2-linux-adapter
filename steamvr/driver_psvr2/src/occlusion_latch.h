// SPDX-License-Identifier: GPL-2.0
// Occluded-controller latch, after Sony's tracker (docs/sony-re/controller-tracking.md §F.2,
// 0x1801b81c0). Implementation is our own.
//
// When the farther controller loses its LEDs behind the nearer one, its
// position is carried along with the nearer ("carrier") controller: it keeps
// its angular offset and depth relative to the line of sight to the
// carrier. Everything here is in the camera-rig frame, +z forward.
//
// Taken from Sony: roles by rig-frame z; entry (carrier fix < 0.3 s old,
// latched hand's fix 0.3..0.6 s old, both < 1 m, overlap of 8.75 cm spheres
// within min(20 deg, |rho_c - rho_l| + 5 deg), rays < 90 deg apart); the
// view-ray-frame transform; states latched / release (0.5 s blend, abort
// beyond 0.15 m) / dropped (hold, no output) and their exits.
// Own: one recorded transform (Sony also keeps a t + 100 ms copy as a
// prediction hint for its association, which this driver does not use);
// the release blend pulls toward the hand's own optical fix (Sony: its UKF
// position); no UKF status-bit 0x20 test.
#pragma once

#include <cstdint>

#include "vr_math.h"

class OcclusionLatch
{
public:
	struct HandIn
	{
		bool has_state = false;      // the hand's filter is running
		vrmath::Vec3 pos;            // filter position at the frame time, rig frame, m
		bool fix_now = false;        // the hand has its own optical fix this frame
		vrmath::Vec3 fix_pos;        // that fix, rig frame, m
		uint64_t last_fix_us = 0;    // its newest optical fix before this frame (0 none)
	};
	enum Action
	{
		kNone,
		kPublish,   // give `hand` a position-only fix at `pos`
		kReplace,   // replace `hand`'s own fix position by `pos` (release blend)
	};
	struct Out
	{
		Action action = kNone;
		int hand = -1;
		vrmath::Vec3 pos;  // rig frame
	};

	Out Frame( uint64_t t_us, const HandIn in[2] );
	int State() const { return state_; }
	int Latched() const { return state_ ? latched_ : -1; }
	unsigned Entries() const { return entries_; }

private:
	int state_ = 0;  // 0 idle, 1 latched, 2 release, 3 dropped
	int carrier_ = -1, latched_ = -1;
	vrmath::Pose rel_;           // carrier ray frame -> latched ray frame
	vrmath::Vec3 held_;
	uint64_t release_us_ = 0, last_end_us_ = 0;
	unsigned entries_ = 0;
};
