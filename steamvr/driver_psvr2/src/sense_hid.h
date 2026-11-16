// SPDX-License-Identifier: GPL-2.0
// PS VR2 Sense controller over hidraw (Bluetooth or USB): input parsing,
// IMU calibration, and the output report that carries rumble, PCM haptics
// and the adaptive-trigger effect.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "sense_protocol.h"
#include "trigger_effect.h"
#include "vr_math.h"

struct SenseHidInfo
{
	std::string path;  // /dev/hidrawN
	bool left = false;
	bool bluetooth = false;
	std::string uniq;  // controller MAC, for a stable serial number
};

// Every hidraw node that is a Sense controller.
std::vector<SenseHidInfo> FindSenseControllers();

struct SenseState
{
	double imu_time_s = 0;  // device IMU clock, unwrapped
	vrmath::Vec3 accel;     // m/s^2, sensor frame
	vrmath::Vec3 gyro;      // rad/s, sensor frame

	float stick_x = 0, stick_y = 0;  // -1..1, +y up
	float trigger = 0;               // 0..1
	float trigger_proximity = 0;     // 0..1 (capacitive)
	float grip_proximity = 0;        // 0..1 (capacitive)
	bool trigger_click = false, trigger_touch = false;
	bool grip_click = false, grip_touch = false;
	bool stick_click = false, stick_touch = false;
	bool face_lower_click = false, face_lower_touch = false;  // square (L) / cross (R)
	bool face_upper_click = false, face_upper_touch = false;  // triangle (L) / circle (R)
	bool menu_click = false;                                  // create (L) / options (R)
	bool ps_click = false;

	bool battery_valid = false;
	bool charging = false;
	float battery = 0;  // 0..1
};

// What the next output report should do. Owned by the caller.
struct SenseOutput
{
	uint8_t rumble_amplitude = 0;  // 0..255, classic rumble when no PCM
	uint8_t rumble_mode = sense::kFlag1VibrationClassic;
	TriggerEffect trigger;
	bool trigger_dirty = false;
	int8_t pcm[sense::kPcmSamplesPerReport]{};
	bool has_pcm = false;
};

class SenseHid
{
public:
	explicit SenseHid( const SenseHidInfo &info );
	~SenseHid();
	SenseHid( const SenseHid & ) = delete;
	SenseHid &operator=( const SenseHid & ) = delete;

	// Opens the node, reads the IMU calibration (which also switches the
	// controller from compatibility to full reports) and sets the PC polling
	// rate. False on any failure.
	bool Open();
	bool IsOpen() const { return fd_ >= 0; }
	int Fd() const { return fd_; }

	// Reads one pending input report into `out`. 1 = new state, 0 = nothing
	// (or a report we skip), -1 = device gone.
	int Read( SenseState &out );

	// Sends one output report built from `out`. False when the write fails.
	bool Write( const SenseOutput &out );

	const SenseHidInfo &Info() const { return info_; }

private:
	bool ReadCalibration();
	bool SetPollingRate();
	void Parse( const sense::InputCommon &in, SenseState &out );

	SenseHidInfo info_;
	int fd_ = -1;

	vrmath::Vec3 gyro_bias_, gyro_scale_{ 1, 1, 1 }, accel_bias_, accel_scale_{ 1, 1, 1 };

	uint32_t last_ticks_ = 0;
	uint64_t total_ticks_ = 0;
	bool have_ticks_ = false;

	uint8_t seq_ = 0;
	uint8_t counter_ = 0;
	EncodedTriggerEffect trigger_{ 0x05, {} };
};
