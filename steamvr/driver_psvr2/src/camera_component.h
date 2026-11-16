// SPDX-License-Identifier: GPL-2.0
// The headset's two bottom tracking cameras as a SteamVR camera, so Room
// View / passthrough works.
//
// The kernel module exposes them as one V4L2 device: 1280x640 8-bit grey, the
// two 640x640 views side by side (kernel/psvr2_camera.c). Frames are
// expanded to RGB24 into the buffers SteamVR hands us. The lens model is not
// known yet, so the cameras are described as plain pinholes with an
// estimated field of view; the image is usable but not geometrically exact.
#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "openvr_driver.h"

class Psvr2CameraComponent : public vr::IVRCameraComponent
{
public:
	static constexpr uint32_t kWidth = 1280, kHeight = 640;  // both views
	static constexpr uint32_t kViewSize = 640;               // one view, square

	// pose: the headset's current pose, stamped onto each frame.
	explicit Psvr2CameraComponent( std::function<vr::DriverPose_t()> pose );
	~Psvr2CameraComponent();

	// Camera properties for the HMD's property container.
	static void SetProperties( vr::PropertyContainerHandle_t container );

	bool GetCameraFrameDimensions( vr::ECameraVideoStreamFormat fmt, uint32_t *w, uint32_t *h ) override;
	bool GetCameraFrameBufferingRequirements( int *queue, uint32_t *size ) override;
	bool SetCameraFrameBuffering( int count, void **buffers, uint32_t size ) override;
	bool SetCameraVideoStreamFormat( vr::ECameraVideoStreamFormat fmt ) override;
	vr::ECameraVideoStreamFormat GetCameraVideoStreamFormat() override;
	bool StartVideoStream() override;
	void StopVideoStream() override;
	bool IsVideoStreamActive( bool *paused, float *elapsed ) override;
	const vr::CameraVideoStreamFrame_t *GetVideoStreamFrame() override;
	void ReleaseVideoStreamFrame( const vr::CameraVideoStreamFrame_t *frame ) override;
	bool SetAutoExposure( bool enable ) override;
	bool PauseVideoStream() override;
	bool ResumeVideoStream() override;
	bool GetCameraDistortion( uint32_t camera, float u, float v, float *out_u, float *out_v ) override;
	bool GetCameraProjection( uint32_t camera, vr::EVRTrackedCameraFrameType type, float znear, float zfar,
	                          vr::HmdMatrix44_t *projection ) override;
	bool SetFrameRate( int isp, int sensor ) override;
	bool SetCameraVideoSinkCallback( vr::ICameraVideoSinkCallback *callback ) override;
	bool GetCameraCompatibilityMode( vr::ECameraCompatibilityMode *mode ) override;
	bool SetCameraCompatibilityMode( vr::ECameraCompatibilityMode mode ) override;
	bool GetCameraFrameBounds( vr::EVRTrackedCameraFrameType type, uint32_t *left, uint32_t *top, uint32_t *w,
	                           uint32_t *h ) override;
	bool GetCameraIntrinsics( uint32_t camera, vr::EVRTrackedCameraFrameType type, vr::HmdVector2_t *focal,
	                          vr::HmdVector2_t *center, vr::EVRDistortionFunctionType *distortion,
	                          double coefficients[vr::k_unMaxDistortionFunctionParameters] ) override;

private:
	void StreamThread();
	static std::string FindDevice();

	std::function<vr::DriverPose_t()> pose_;
	std::mutex mutex_;  // buffers_, frame_, sink_
	std::vector<void *> buffers_;
	uint32_t buffer_size_ = 0;
	vr::CameraVideoStreamFrame_t frame_{};
	bool have_frame_ = false;
	vr::ICameraVideoSinkCallback *sink_ = nullptr;

	std::thread thread_;
	std::atomic<bool> streaming_{ false };
	std::atomic<bool> paused_{ false };
	double start_time_ = 0.0;
};
