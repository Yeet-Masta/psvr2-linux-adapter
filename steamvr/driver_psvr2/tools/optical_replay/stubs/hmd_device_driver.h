// SPDX-License-Identifier: GPL-2.0
// Replay stand-in for the HMD: PoseAt returns the head pose recorded for
// the current LD frame.
#pragma once

#include <cstdint>

#include "openvr_driver.h"

class Psvr2HmdDriver
{
public:
	bool PoseAt( uint64_t, vr::DriverPose_t &out )
	{
		out = pose_;
		return ok_;
	}
	bool ok_ = false;
	vr::DriverPose_t pose_{};
};
