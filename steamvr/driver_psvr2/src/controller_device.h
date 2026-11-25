// SPDX-License-Identifier: GPL-2.0
// PS VR2 Sense controller as a SteamVR tracked controller.
//
// Inputs, battery, rumble / PCM haptics and adaptive triggers come straight
// from the controller over hidraw (sense_hid.h). Orientation is 3DoF from the
// controller IMU. Position comes from an arm model anchored to the headset:
// tracking the controller LEDs with the headset cameras (6DoF) is not
// implemented yet (docs/development.md part C).
#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "controller_ukf.h"
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
	Psvr2HmdDriver *Hmd() const { return hmd_; }

	// Hand a freshly discovered hidraw node to this controller. Ignored while
	// a node is already attached.
	void Attach( const SenseHidInfo &info );
	bool Attached() const { return attached_; }
	// LED sync (optical tracking): forward a tracker command to the open
	// controller and read its sync record. False / default when detached.
	bool LedSyncCommand( const uint8_t *cmd, uint8_t len );
	bool LedTestSweep( int32_t offset_us );
	bool LedSyncStatus( SenseHid::LedSyncStatus &out );

	// ---- Optical tracking (called from the OpticalTracker thread) ----
	// Sense hardware revision (selects the LED model); 0 when unknown.
	int HardwareRevision();
	// Fused IMU orientation (gravity-aligned, arbitrary heading) and the
	// heading correction in use: world <- IMU = RotY(yaw) * q.
	bool ImuOrientation( vr::HmdQuaternion_t &q, double &yaw );
	// An optical fix for the LED frame at host time frame_us: the IMU's
	// position (m) and orientation in the world. Returns false when the
	// innovation gate rejects it (docs/sony-re/controller-tracking.md §C.4.1).
	// head_rel: the IMU position in the head frame (m); cam_dist: distance
	// from the camera rig (m). Both select the filter's measurement noise.
	bool OpticalFix( uint64_t frame_us, const vrmath::Vec3 &pos, const vr::HmdQuaternion_t &world_from_imu,
	                 const vrmath::Vec3 &head_rel, double cam_dist );

	// SteamVR haptic event for our haptic component.
	void OnHaptic( const vr::VREvent_HapticVibration_t &h );
	vr::VRInputComponentHandle_t HapticHandle() const { return haptic_; }

	// Adaptive trigger effect from the PSVR2 Toolkit IPC server; nullptr
	// hands the trigger back to the default_trigger_effect setting.
	void SetTriggerEffect( const TriggerEffect *fx );

	// Point the controller's forward along the headset's forward.
	// Holding the menu button does the same (1 s), or calibrates (3 s).
	void Recenter();

private:
	void IoThread();
	void HandleState( const SenseState &s );
	void FeedUkf( const SenseState &s, double dt );
	void FeedArmModel( uint64_t now_us );
	void FillOutput( SenseOutput &out );
	vr::DriverPose_t BuildPose();
	// Queue an alert buzz: alternating on/off durations in seconds.
	void Buzz( std::initializer_list<double> pattern );
	void UpdateSkeleton( const SenseState &s );
	void UpdateBattery( const SenseState &s );
	void SaveGyroBias();
	void LoadSettings();
	std::string CalibrationKey( char axis ) const;
	void LoadCalibration();
	void SaveCalibration();
	std::string SettingKey( const char *name ) const;
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
	vr::VRInputComponentHandle_t skeleton_ = 0;

	std::mutex hid_mutex_;  // guards hid_ / info swaps
	std::unique_ptr<SenseHid> hid_;
	std::atomic<bool> attached_{ false };
	std::atomic<bool> running_{ false };
	std::thread io_thread_;

	// Tracking state, IO thread only (copied into pose_ under pose_mutex_).
	ImuFusion fusion_;
	double last_imu_time_ = -1.0;
	std::chrono::steady_clock::time_point last_input_{};
	vrmath::Vec3 gyro_world_;
	double yaw_offset_ = 0.0;
	bool yaw_anchored_ = false;
	std::chrono::steady_clock::time_point menu_down_since_{};
	bool menu_was_down_ = false;
	int menu_stage_ = 0;
	std::atomic<bool> recenter_requested_{ false };
	std::atomic<bool> calibrate_requested_{ false };
	bool bias_saved_ = false;

	// Prediction / smoothing (IO thread).
	double prediction_s_ = 0.012;
	vrmath::Vec3 smoothed_gyro_world_;
	vrmath::Vec3 last_hand_, hand_velocity_;
	double last_pose_time_ = -1.0;

	// Recenter when the headset is put on, once the controller is held level.
	bool recenter_on_wear_ = true;
	uint32_t seen_put_on_ = 0;
	std::chrono::steady_clock::time_point wear_recenter_until_{};

	// Finger curls for the skeleton (thumb, index, middle, ring, pinky).
	float curls_[5]{};
	std::chrono::steady_clock::time_point next_skeleton_{};

	bool battery_alerts_ = true;
	int vibration_strength_ = 1;
	bool props_dirty_ = false;
	uint64_t universe_set_ = 0;
	// Orientation fix in the controller frame, from the hold-to-calibrate
	// gesture; persisted in the driver_psvr2 settings section.
	vr::HmdQuaternion_t calibration_ = vrmath::Identity();

	std::mutex pose_mutex_;
	vr::DriverPose_t pose_{};

	// Optical state (optical_mutex_). Position: the newest accepted fix,
	// extrapolated for 0.3 s and then held, as Sony does. Heading: each new
	// fix pulls yaw_offset_ toward the optical heading.
	struct Optical
	{
		bool have = false;
		uint64_t us = 0, seq = 0;
		vrmath::Vec3 pos, vel;
		vr::HmdQuaternion_t rot = vrmath::Identity();
		vrmath::Vec3 head_rel;
		double cam_dist = 0.0;
		int rejects = 0;
	};
	std::mutex optical_mutex_;
	Optical optical_;
	uint64_t optical_seq_used_ = 0;  // IoThread

	// Optical + IMU filter (IoThread). Setting optical_ukf (default on);
	// off falls back to the plain optical position above.
	ControllerUkf ukf_;
	bool ukf_enabled_ = true;
	uint64_t ukf_hold_us_ = 300000;  // Sony: optical position held after 0.3 s
	uint64_t ukf_seq_used_ = 0;
	// Arm-model pseudo measurements (setting optical_arm_model, default on;
	// docs/sony-re/controller-tracking.md §F.3.4).
	bool arm_model_enabled_ = true;
	uint64_t arm_last_us_ = 0, arm_moved_us_ = 0;
	unsigned int arm_fixes_ = 0;
	uint64_t imu_host_us_ = 0;       // IMU sample time on the host clock
	std::mutex imu_mutex_;
	vr::HmdQuaternion_t imu_q_ = vrmath::Identity();
	bool imu_valid_ = false;
	std::atomic<double> yaw_pub_{ 0.0 };

	// Output state.
	std::mutex out_mutex_;
	float haptic_amplitude_ = 0.0f;
	float haptic_frequency_ = 0.0f;
	std::chrono::steady_clock::time_point haptic_end_{};
	std::chrono::steady_clock::time_point haptic_start_{};
	double pcm_phase_ = 0.0;
	TriggerEffect trigger_;         // what is on the trigger now
	TriggerEffect default_trigger_; // default_trigger_effect setting
	bool app_trigger_ = false;      // a Toolkit client owns the trigger
	bool trigger_dirty_ = false;
	struct BuzzSegment
	{
		std::chrono::steady_clock::time_point start, end;
	};
	std::vector<BuzzSegment> buzz_;

	float battery_ = -1.0f;
	float alerted_battery_ = 1.0f;  // lowest alert threshold already buzzed
	bool charging_ = false;
};
