// SPDX-License-Identifier: GPL-2.0
#pragma once

#include "camera_component.h"
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "openvr_driver.h"
#include "pose_source.h"

// Geometry of the PSVR2 panel as the compositor needs to see it. Defaults are
// the native panel (4000x2040, two 2000x2040 eyes); they can be overridden from
// resources/settings/default.vrsettings.
struct Psvr2DisplayConfig
{
	int32_t window_x = 0;          // top-left of the panel in the X desktop
	int32_t window_y = 0;
	int32_t window_width = 4000;   // full stereo panel
	int32_t window_height = 2040;
	int32_t render_width = 2000;   // per-eye render target
	int32_t render_height = 2040;
	// Left-eye field of view in degrees (Sony's values, via Monado's PSVR2
	// driver); the right eye mirrors it horizontally.
	// 0 = Sony's projection tangents (1.84177 outer, 0.94719 inner, 1.32898).
	float   fov_outer_deg = 0.0f;
	float   fov_inner_deg = 0.0f;
	float   fov_vertical_deg = 0.0f;

	// Per-unit lens parameters for psvr2_distort() (libpsvr2).
	float   lens[8] = { -0.09919293f, 0.0f, 0.09919293f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f };

	// Direct mode (default): report the panel as a real, NON-desktop display so
	// SteamVR's compositor acquires it directly via DRM leasing (no TTY, desktop
	// keeps running). When false, the panel is treated as an extended desktop
	// monitor instead (the bring-up scaffold — see docs/getting-started.md part C).
	bool    direct_mode = true;
};

// IVRDisplayComponent: tells vrcompositor where/how to render. Extended-display
// mode for now (the panel is a normal desktop output we light up via KMS);
// direct mode is future work (roadmap M7).
class Psvr2DisplayComponent : public vr::IVRDisplayComponent
{
public:
	explicit Psvr2DisplayComponent( const Psvr2DisplayConfig &config );

	bool IsDisplayOnDesktop() override;
	bool IsDisplayRealDisplay() override;
	void GetRecommendedRenderTargetSize( uint32_t *pnWidth, uint32_t *pnHeight ) override;
	void GetEyeOutputViewport( vr::EVREye eEye, uint32_t *pnX, uint32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight ) override;
	void GetProjectionRaw( vr::EVREye eEye, float *pfLeft, float *pfRight, float *pfTop, float *pfBottom ) override;
	vr::DistortionCoordinates_t ComputeDistortion( vr::EVREye eEye, float fU, float fV ) override;
	void GetWindowBounds( int32_t *pnX, int32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight ) override;
	bool ComputeInverseDistortion( vr::HmdVector2_t *pResult, vr::EVREye eEye, uint32_t unChannel, float fU, float fV ) override;

	const float *Lens() const { return config_.lens; }

private:
	Psvr2DisplayConfig config_;
};

// One eye's tracking sample, in the headset's native gaze frame.
struct Psvr2EyeData
{
	bool valid = false;
	bool origin_valid = false, dir_valid = false, pupil_valid = false, blink_valid = false;
	float origin_mm[3]{};  // device frame, as PSVR2 Toolkit reports it
	float dir[3]{};        // device frame, normalised
	float pupil_mm = 0.0f;
	bool blink = false;
};

struct Psvr2GazeData
{
	Psvr2EyeData left, right;
	bool combined_valid = false;
	vr::HmdVector3_t origin{};  // head frame, metres
	vr::HmdVector3_t dir{};     // head frame, normalised
};

// The HMD itself.
class Psvr2HmdDriver : public vr::ITrackedDeviceServerDriver
{
public:
	Psvr2HmdDriver();
	~Psvr2HmdDriver();

	vr::EVRInitError Activate( uint32_t unObjectId ) override;
	void Deactivate() override;
	void EnterStandby() override;
	void *GetComponent( const char *pchComponentNameAndVersion ) override;
	void DebugRequest( const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize ) override;
	vr::DriverPose_t GetPose() override;
	// Head pose at a host CLOCK_MONOTONIC time (µs), interpolated from the
	// recent poses; false when there is none within 100 ms. Includes
	// standing_height_, like GetPose().
	bool PoseAt( uint64_t host_us, vr::DriverPose_t &out );

	const std::string &GetSerialNumber() const { return serial_number_; }

	// Latest eye-tracking data (for the PSVR2 Toolkit IPC server).
	Psvr2GazeData GetGaze();

	// Headset rumble motor, 0..25 Hz (needs a jailbroken headset).
	void SetRumble( int hz );
	vr::VRInputComponentHandle_t HapticHandle() const { return haptic_; }
	void OnHaptic( const vr::VREvent_HapticVibration_t &h );

	// Called from the provider's frame loop.
	void RunFrame();

	// Bumped each time the headset is put on (proximity sensor).
	uint32_t PutOnCount() const { return put_on_count_; }
	// Universe of the play area this driver published, 0 = none yet.
	uint64_t UniverseId() const { return universe_id_; }
	// Short headset rumble to flag an event (e.g. a controller dropped out).
	void RequestAlert() { alert_requested_ = true; }

private:
	void PoseThread();
	void SelectEdid( const char *why );
	void WriteChaperone( const vr::DriverPose_t &pose );
	void SubmitPose( const vr::DriverPose_t &pose, bool worn );
	void GazeThread();
	void PollControls();

	std::unique_ptr<Psvr2DisplayComponent> display_;
	std::unique_ptr<PoseSource> pose_source_;
	std::unique_ptr<Psvr2CameraComponent> camera_;  // null when passthrough_camera is off

	std::string model_number_;
	std::string serial_number_;
	float display_frequency_ = 90.0f;
	// DisplayAvailableFrameRates entries; the one in use equals
	// steamvr/preferredRefreshRate exactly.
	float rate_90_ = 90.0f, rate_120_ = 120.0f;
	bool direct_mode_ = true;

	std::atomic<bool> active_{ false };
	std::atomic<uint32_t> device_index_{ vr::k_unTrackedDeviceIndexInvalid };
	vr::PropertyContainerHandle_t container_ = vr::k_ulInvalidPropertyContainer;

	std::mutex pose_mutex_;
	vr::DriverPose_t last_pose_{};
	struct TimedPose
	{
		uint64_t host_us;
		vr::DriverPose_t pose;
	};
	std::deque<TimedPose> pose_history_;  // ~0.5 s, oldest first; pose_mutex_
	std::thread pose_thread_;

	// Gaze, controls and rumble go through a second libpsvr2 handle.
	struct psvr2 *dev_ = nullptr;
	std::thread gaze_thread_;
	std::mutex gaze_mutex_;
	Psvr2GazeData gaze_{};
	vr::VRInputComponentHandle_t eye_tracking_ = 0;
	vr::VRInputComponentHandle_t haptic_ = 0;
	vr::VRInputComponentHandle_t proximity_ = 0;

	int ipd_mm_ = 0;
	int worn_ = -1;
	std::atomic<uint32_t> put_on_count_{ 0 };
	std::atomic<bool> alert_requested_{ false };
	float standing_height_ = 0.0f;  // metres added to the tracker's height
	int rumble_hz_ = 0;
	uint32_t firmware_version_ = 0;
	bool supports_120_ = true;
	int see_through_ = -1;
	int see_through_poll_ = 0;
	bool chaperone_written_ = false;
	std::atomic<uint64_t> universe_id_{ 0 };
	// Not-worn throttle (docs/sony-re/steamvr-and-display.md §B.5.5).
	vr::DriverPose_t ref_pose_{};
	std::chrono::steady_clock::time_point ref_time_{};
	bool have_ref_ = false;
	std::chrono::steady_clock::time_point rumble_end_{};
};
