// SPDX-License-Identifier: GPL-2.0
// PS VR2 Sense controller over hidraw (Bluetooth or USB): input parsing,
// IMU calibration, and the output report that carries rumble, PCM haptics
// and the adaptive-trigger effect.
#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "sie_c.h"

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
	uint8_t rumble_amplitude = 0;  // 0..20 (Sony's scale), classic rumble when no PCM
	uint8_t rumble_mode = sense::kFlag1VibrationClassic;
	uint8_t reduction = 0;         // low bits: vibration strength (0 strong, 2 medium, 3 weak)
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
	// From feature 0x20; 0 when it could not be read.
	uint32_t FirmwareVersion() const { return fw_version_; }
	uint16_t FirmwareRevision() const { return fw_revision_; }
	uint16_t HardwareRevision() const { return hw_revision_; }
	// Use Sony's Bluetooth output framing (31 a2 + body); false = the
	// DualSense-style 31 <seq> 10 framing.
	static void SetSonyFraming( bool on ) { sony_framing_ = on; }

	// Reads one pending input report into `out`. 1 = new state, 0 = nothing
	// (or a report we skip), -1 = device gone.
	int Read( SenseState &out );

	// Sends one output report built from `out`. False when the write fails.
	bool Write( const SenseOutput &out );

	const SenseHidInfo &Info() const { return info_; }

	// LED sync (docs/sony-re/controllers.md part C). Off: the LED block says
	// "LED ALL OFF" as before. On: every input report feeds libpad's
	// host<->controller clock fit and every output report carries the
	// LED-sync record, which the optical tracker drives with commands.
	static void SetLedSync( bool on ) { led_sync_enabled_ = on; }
	static bool LedSyncEnabled() { return led_sync_enabled_; }
	// One tracker -> libpad command (docs/sony-re/controllers.md §C.2.1).
	void LedSyncCommand( const uint8_t *cmd, uint8_t len );
	// LED test only: phase LED ALL ON with base time = the controller time
	// of the headset clock at the first call (the anchor) + offset_us.
	// False while the clock fit or headset time is missing.
	bool LedTestSweep( int32_t offset_us );
	struct LedSyncStatus
	{
		uint8_t phase = 0, sync_mode = 0, seq = 0, period_id = 0;
		int32_t step = 105, width = 1000, base_time = 0;
		uint32_t cycle_ns = 0;
		bool clock_ready = false;
		// Fit's controller time at the newest input report's arrival
		// minus that report's own timestamp (µs): the BT latency if the
		// fit is right.
		int32_t fit_err_us = 0;
	};
	LedSyncStatus LedSync();

private:
	bool ReadCalibration();
	void ReadFirmwareInfo();
	bool ApplyCalibration( const sense::CalibrationData &cal );
	bool SetPollingRate();
	void Parse( const sense::InputCommon &in, SenseState &out );

	SenseHidInfo info_;
	int fd_ = -1;

	vrmath::Vec3 gyro_bias_, gyro_scale_{ 1, 1, 1 }, accel_bias_, accel_scale_{ 1, 1, 1 };

	uint32_t last_ticks_ = 0;
	uint64_t total_ticks_ = 0;
	bool have_ticks_ = false;

	uint8_t seq_ = 0;
	uint32_t fw_version_ = 0;
	uint16_t fw_revision_ = 0, hw_revision_ = 0;
	static inline bool sony_framing_ = true;
	uint8_t counter_ = 0;
	// Last Bluetooth report body sent (host timestamp zeroed) and when.
	sense::OutputSettings last_bt_settings_{};
	int32_t test_anchor_ = 0;
	uint64_t last_rx_us_ = 0;
	uint32_t last_ctrl_us_ = 0;
	uint8_t logged_phase_ = 0xff;  // phase of the last logged output report
	void LogReport( const void *r, size_t n, uint8_t phase );
	bool test_anchored_ = false;
	std::chrono::steady_clock::time_point last_bt_write_{};
	EncodedTriggerEffect trigger_{ 0x05, {} };

	void FeedSyncFit( const sense::InputCommon &in );
	void FillLedSync( sense::OutputSettings &st, uint64_t now_us );
	uint8_t ProbeSyncMode();
	static inline bool led_sync_enabled_ = false;
	std::mutex led_mutex_;
	psvr2_sync_state sync_{};
	psvr2_ledsync led_{};
};
