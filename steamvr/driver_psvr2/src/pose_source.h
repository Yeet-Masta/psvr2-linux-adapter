// SPDX-License-Identifier: GPL-2.0
#pragma once

#include <cstdint>
#include <deque>

#include "openvr_driver.h"

// Bridges libpsvr2's 6DoF stream to an OpenVR DriverPose_t. Owns the libpsvr2
// handle and converts the device-native frame into the OpenVR frame.
class PoseSource
{
public:
	PoseSource();
	~PoseSource();

	// True once the /dev/psvr2-pose node was found and opened.
	bool HasPose() const;

	// Block (up to a short timeout) for the next sample and fill `out` in the
	// OpenVR frame. Returns true if `out` was updated, false on timeout/EOF.
	bool ReadPose( vr::DriverPose_t &out );

private:
	struct Sample
	{
		double t;  // device time, seconds
		double pos[3];
		vr::HmdQuaternion_t rot;
	};
	void EstimateVelocity( const Sample &now, vr::DriverPose_t &out );

	struct psvr2 *dev_;
	// Recent samples for velocity estimation, newest last.
	std::deque<Sample> history_;
	uint32_t last_vts_ = 0;
	double device_time_ = 0.0;  // unwrapped device_vts_us
	double vel_[3]{}, ang_vel_[3]{};
};
