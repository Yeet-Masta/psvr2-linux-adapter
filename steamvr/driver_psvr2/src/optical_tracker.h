// SPDX-License-Identifier: GPL-2.0
// Headset side of controller optical tracking (docs/sony-re/ledsync.md).
//
// Reads the headset's LED-detection packets ("LD", /dev/psvr2-ld), keeps the
// headset's blob-detection settings (Set 0x09 / 0x14) in the state Sony's
// driver keeps them, publishes the newest camera frame time for libpad's
// PRESCAN base time, and runs one LedSyncer per controller.
//
// Blobs are not yet associated with a controller by predicted pose during
// sync, so this syncs one controller at a time and treats every blob a
// camera sees as that controller's sync LED. Once a controller is STABLE,
// OpticalPose solves its pose. Gated by the "optical_tracking" setting.
#pragma once

#include <cstdio>

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

#include "blink_finder.h"
#include "led_syncer.h"
#include "optical_pose.h"

#include "libpsvr2.h"
#include "sie_c.h"

class Psvr2ControllerDriver;

class OpticalTracker
{
public:
	// controllers[0] = left, [1] = right; they must outlive the tracker.
	explicit OpticalTracker( Psvr2ControllerDriver *const controllers[2] );
	~OpticalTracker();
	OpticalTracker( const OpticalTracker & ) = delete;
	OpticalTracker &operator=( const OpticalTracker & ) = delete;

	void Start();
	void Stop();

	// Host CLOCK_MONOTONIC µs of the newest camera frame; 0 before the first.
	static uint64_t HeadsetFrameMicros() { return frame_us_.load(); }

private:
	struct Header
	{
		uint16_t version;
		uint32_t device_us, cam_flags, counter;
		float echo_a, echo_b;
		uint8_t no_detection;
		uint32_t count[PSVR2_LD_CAMERAS];
	};

	void Run();
	void LogCameraCalibration();
	bool ParseHeader( const psvr2_ld_packet &pkt, Header &h ) const;
	void UpdateFrameTime( const psvr2_ld_packet &pkt, const Header &h );
	void UpdateBlobMode( const Header &h );
	void PushDetectionConfig( const psvr2_ld_packet &pkt, const Header &h );
	void DecodeBlobs( const psvr2_ld_packet &pkt, const Header &h );
	int GlobalPreset();
	LedObservation Observe( int hand, int phase_owner );
	void RunSyncers( uint64_t now_us );
	void SolvePoses( uint64_t now_us );

	Psvr2ControllerDriver *controllers_[2];
	std::unique_ptr<LedSyncer> syncers_[2];
	std::unique_ptr<OpticalPose> pose_;
	uint64_t rig_try_us_ = 0;
	psvr2_t *dev_ = nullptr;
	std::thread thread_;
	std::atomic<bool> running_{ false };
	static inline std::atomic<uint64_t> frame_us_{ 0 };

	struct psvr2_clock clock_ {};
	uint64_t clock_at_us_ = 0;

	// Blob-detection mode (ledsync.md §4.3): 0 Normal, 1 Many Blob.
	int blob_mode_ = 0;
	int quiet_frames_ = 0;
	uint32_t last_09_counter_ = 0;
	bool pushed_09_ = false;
	int pushed_14_preset_ = 0;

	// Headset-wide preset (ledsync.md §2.4) and its owner hand (-1 none).
	int preset_ = 0, preset_owner_ = -1;

	bool see_through_ = false;
	uint64_t see_through_at_us_ = 0;

	psvr2_blob_set blobs_[PSVR2_LD_CAMERAS];
	int blob_count_[PSVR2_LD_CAMERAS] = {};
	// The blinking sync LED per camera, for the controller in BROAD/BG.
	BlinkFinder finders_[PSVR2_LD_CAMERAS];

	// Opt-in blob log for hardware bring-up (setting optical_log_blobs).
	void LogBlobs( const Header &h );
	struct BlobStats
	{
		int frames = 0, seen = 0, max_count = 0, max_w = 0, max_h = 0;
		float min_x = 1e9f, max_x = -1e9f, min_y = 1e9f, max_y = -1e9f;
	};
	bool log_blobs_ = false;
	// Opt-in per-frame trace for bring-up (setting optical_trace): every
	// blob and every LED-sync command, to ~/psvr2-ld-trace.txt, first minute.
	void TraceFrame( const Header &h, int period );
	void TraceCommand( int hand, const uint8_t *cmd, uint8_t len );
	FILE *trace_ = nullptr;
	// Setting optical_led_test: instead of syncing, switch every
	// controller between LED ALL ON and ALL OFF every 2 s, to find
	// whether the cameras see its LEDs.
	bool led_test_ = false;
	int led_test_step_ = 0;
	uint64_t led_test_at_us_ = 0;
	int trace_frames_ = 0;
	uint64_t blob_log_at_us_ = 0;
	BlobStats blob_stats_[PSVR2_LD_CAMERAS];
	// Logical sync LED per hand; moves on after a failed BROAD round.
	int sync_led_[2] = { 0, 0 };
	LedSyncer::Phase last_phase_[2] = { LedSyncer::kInit, LedSyncer::kInit };
};
