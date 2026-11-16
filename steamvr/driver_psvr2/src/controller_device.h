// SPDX-License-Identifier: GPL-2.0
// PS VR2 Sense controller as a SteamVR tracked controller.
//
// Inputs, battery, rumble / PCM haptics and adaptive triggers come straight
// from the controller over hidraw (sense_hid.h). Orientation is 3DoF from the
// controller IMU. Position comes from an arm model anchored to the headset:
// tracking the controller LEDs with the headset cameras (6DoF) is not
// implemented yet (docs/roadmap.md).
#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "imu_fusion.h"
#include "openvr_driver.h"
#include "sense_hid.h"
#include "trigger_effect.h"

class Psvr2HmdDriver;

class Psvr2ControllerDriver : public vr::ITrackedDeviceServerDriver
{
public:
	Psvr2ControllerDriver( bool left, Psvr2HmdDriver *hmd );
	~Psvr2ControllerDriver();

	vr::EVRInitError Activate( uint32_t unObjectId ) override;
	void Deactivate() override;
	void EnterStandby() override;
	void *GetComponent( const char *pchComponentNameAndVersion ) override;
	void DebugRequest( const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize ) override;
	vr::DriverPose_t GetPose() override;

	const std::string &GetSerialNumber() const { return serial_; }
	bool IsLeft() const { return left_; }

	// Hand a freshly discovered hidraw node to this controller. Ignored while
	// a node is already attached.
	void Attach( const SenseHidInfo &info );
	bool Attached() const { return attached_; }

	// SteamVR haptic event for our haptic component.
	void OnHaptic( const vr::VREvent_HapticVibration_t &h );
	vr::VRInputComponentHandle_t HapticHandle() const { return haptic_; }

	// Adaptive trigger effect (from the PSVR2 Toolkit IPC server).
	void SetTriggerEffect( const TriggerEffect &fx );

	// Point the controller's forward along the headset's forward.
	// Holding the menu button does the same (1 s), or calibrates (3 s).
	void Recenter();

private:
	void IoThread();
	void HandleState( const SenseState &s );
	void FillOutput( SenseOutput &out );
	vr::DriverPose_t BuildPose();
	void Pulse( double seconds );
	std::string CalibrationKey( char axis ) const;
	void LoadCalibration();
	void SaveCalibration();
	void Disconnected();

	const bool left_;
	Psvr2HmdDriver *const hmd_;
	std::string serial_;
	bool enhanced_haptics_ = true;

	std::atomic<uint32_t> device_index_{ vr::k_unTrackedDeviceIndexInvalid };
	vr::PropertyContainerHandle_t container_ = vr::k_ulInvalidPropertyContainer;

	// Input components.
	vr::VRInputComponentHandle_t system_click_ = 0, menu_click_ = 0;
	vr::VRInputComponentHandle_t lower_click_ = 0, lower_touch_ = 0, upper_click_ = 0, upper_touch_ = 0;
	vr::VRInputComponentHandle_t trigger_value_ = 0, trigger_click_ = 0, trigger_touch_ = 0, trigger_prox_ = 0;
	vr::VRInputComponentHandle_t grip_value_ = 0, grip_click_ = 0, grip_touch_ = 0;
	vr::VRInputComponentHandle_t stick_x_ = 0, stick_y_ = 0, stick_click_ = 0, stick_touch_ = 0;
	vr::VRInputComponentHandle_t haptic_ = 0;

	std::mutex hid_mutex_;  // guards hid_ / info swaps
	std::unique_ptr<SenseHid> hid_;
	std::atomic<bool> attached_{ false };
	std::atomic<bool> running_{ false };
	std::thread io_thread_;

	// Tracking state, IO thread only (copied into pose_ under pose_mutex_).
	ImuFusion fusion_;
	double last_imu_time_ = -1.0;
	vrmath::Vec3 gyro_world_;
	double yaw_offset_ = 0.0;
	bool yaw_anchored_ = false;
	std::chrono::steady_clock::time_point menu_down_since_{};
	bool menu_was_down_ = false;
	int menu_stage_ = 0;
	std::atomic<bool> recenter_requested_{ false };
	std::atomic<bool> calibrate_requested_{ false };
	// Orientation fix in the controller frame, from the hold-to-calibrate
	// gesture; persisted in the driver_psvr2 settings section.
	vr::HmdQuaternion_t calibration_ = vrmath::Identity();

	std::mutex pose_mutex_;
	vr::DriverPose_t pose_{};

	// Output state.
	std::mutex out_mutex_;
	float haptic_amplitude_ = 0.0f;
	float haptic_frequency_ = 0.0f;
	std::chrono::steady_clock::time_point haptic_end_{};
	double pcm_phase_ = 0.0;
	TriggerEffect trigger_;
	bool trigger_dirty_ = false;

	float battery_ = -1.0f;
	bool charging_ = false;
};
