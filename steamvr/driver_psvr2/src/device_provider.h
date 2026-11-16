// SPDX-License-Identifier: GPL-2.0
#pragma once

#include <chrono>
#include <memory>

#include "controller_device.h"
#include "hmd_device_driver.h"
#include "openvr_driver.h"
#include "toolkit_ipc.h"

// The IServerTrackedDeviceProvider is vrserver's handle on our driver: it owns
// the device lifecycle and is pumped once per server frame.
class Psvr2DeviceProvider : public vr::IServerTrackedDeviceProvider
{
public:
	vr::EVRInitError Init( vr::IVRDriverContext *pDriverContext ) override;
	void Cleanup() override;
	const char *const *GetInterfaceVersions() override;

	void RunFrame() override;

	bool ShouldBlockStandbyMode() override;
	void EnterStandby() override;
	void LeaveStandby() override;

private:
	// Look for Sense controllers that appeared since the last scan.
	void ScanControllers();
	void SetTriggerEffect( int hand, const TriggerEffect &fx );

	std::unique_ptr<Psvr2HmdDriver> hmd_;
	std::unique_ptr<Psvr2ControllerDriver> controllers_[2];  // [0] left, [1] right
	bool controller_added_[2] = { false, false };
	std::unique_ptr<ToolkitIpcServer> ipc_;
	std::chrono::steady_clock::time_point next_scan_{};
};
