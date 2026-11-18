// SPDX-License-Identifier: GPL-2.0
#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "controller_device.h"
#include "hmd_device_driver.h"
#include "optical_tracker.h"
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
	// Open Sense controllers that are not attached. Opening reads the
	// calibration feature report, which can block for seconds while a
	// Bluetooth controller is out of range, so after Init it runs on
	// scan_thread_; RunFrame only registers what it attached.
	void ScanControllers();
	void ScanLoop();
	void RegisterControllers();
	void SetTriggerEffect( int hand, const TriggerEffect *fx );

	std::unique_ptr<Psvr2HmdDriver> hmd_;
	std::unique_ptr<Psvr2ControllerDriver> controllers_[2];  // [0] left, [1] right
	bool controller_added_[2] = { false, false };
	std::unique_ptr<ToolkitIpcServer> ipc_;
	std::unique_ptr<OpticalTracker> optical_;
	std::atomic<bool> scanning_{ false };
	std::thread scan_thread_;
};
