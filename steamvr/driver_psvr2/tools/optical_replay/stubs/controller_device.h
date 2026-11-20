// SPDX-License-Identifier: GPL-2.0
// Replay stand-in for the controller: serves the IMU state recorded in an
// LD trace ("P" lines) and records the optical fixes OpticalPose hands it.
#pragma once

#include <cstdint>

#include "openvr_driver.h"
#include "vr_math.h"

class Psvr2HmdDriver;

class Psvr2ControllerDriver
{
public:
	bool ImuOrientation( vr::HmdQuaternion_t &q, double &yaw )
	{
		q = q_;
		yaw = yaw_;
		return ok_;
	}
	int HardwareRevision() const { return rev_; }
	bool OpticalFix( uint64_t frame_us, const vrmath::Vec3 &pos, const vr::HmdQuaternion_t &world_from_imu );
	Psvr2HmdDriver *Hmd() const { return hmd_; }

	// Replay side.
	int hand = 0;
	bool ok_ = false;
	vr::HmdQuaternion_t q_ = vrmath::Identity();
	double yaw_ = 0;
	int rev_ = 0;
	Psvr2HmdDriver *hmd_ = nullptr;
};
