// SPDX-License-Identifier: GPL-2.0
// Per-controller LED-sync state machine (docs/sony-re/ledsync.md §3, §4, §5).
//
// The headset only reports LED blobs; it never tells the controller when its
// camera shutters open. The PC finds that time offset and passes it to the
// controller in every output report. This class decides the phase
// (INIT -> PRESCAN -> BROAD -> BG -> STABLE) and emits the small libpad
// commands (ledsync.md §2.1) that SenseHid turns into the output report's
// LED block.
//
// Written from the behavioural spec; timing constants are the values the doc
// lists for Sony's presets. Parts the doc tags [likely]/[guess] are marked
// here too.
#pragma once

#include <cstdint>
#include <functional>

constexpr int kLedCameras = 4;

// What the optical tracker saw of this controller in one LD frame.
struct LedObservation
{
	// At least one blob associated with this controller, weight > 1e-6.
	bool visible = false;
	// Per camera: this controller's blob count, and the weight of the sync
	// LED's blob (0 when not seen).
	uint16_t count[kLedCameras] = {};
	float sync_weight[kLedCameras] = {};
	// Physical LED indices to light in BROAD/BG (ff = unused).
	uint8_t sync_leds[4] = { 0xff, 0xff, 0xff, 0xff };
};

class LedSyncer
{
public:
	enum Phase : uint8_t
	{
		kInit = 0,
		kPrescan = 1,
		kBroad = 2,
		kBg = 3,
		kStable = 4,
		kAllOff = 5,
	};
	using Send = std::function<void( const uint8_t *cmd, uint8_t len )>;

	explicit LedSyncer( Send send ) : send_( std::move( send ) ) {}

	// preset: 1 or 2 (ledsync.md §2.4). now_us: host CLOCK_MONOTONIC µs.
	void Frame( uint64_t now_us, int preset, const LedObservation &obs );
	// Forced transitions (ledsync.md §3.2).
	void ForceInit();
	void ForceAllOff();
	Phase Current() const { return phase_; }
	// Stay in PRESCAN (all LEDs lit, base re-synced every 1 s) instead of
	// going on to BROAD. Own method, see OpticalTracker.
	void HoldPrescan( bool on ) { hold_prescan_ = on; }
	// Defer STABLE's periodic BG re-check (while another controller syncs).
	void HoldBg( bool on ) { hold_bg_ = on; }
	// BROAD code frame divider: 1 = one code frame per LD frame (Sony);
	// 3 = one per three LD frames (hypothesis for firmware 0x0306, which
	// lights the sync LED on every third LD frame in BROAD; DEV-NOTES).
	// Both are always decoded for the log; this picks the one acted on.
	void SetBroadDivider( int d ) { broad_div_ = d == 3 ? 3 : 1; }
	static const char *Name( Phase p );

private:
	struct Preset
	{
		int step, prescan, broad, bg, stable;  // µs
	};
	static Preset Timing( int preset );

	// Drift fit (ledsync.md §5.5): runs of equal values, then one
	// least-squares slope over all runs but the first, latched until reset.
	struct DriftFit
	{
		static constexpr int kMax = 64;
		double v[kMax], t[kMax];
		int n = 0, pushes = 0;
		uint64_t t0 = 0;
		bool fitted = false;
		double slope = 0;
		void Reset() { *this = DriftFit{}; }
		void Push( uint64_t now_us, double value );
		bool Fit();
	};

	void Enter( Phase p, uint64_t now_us );
	void SetGood( bool good, uint64_t now_us );
	uint64_t InState( uint64_t now_us ) const { return now_us - sub_since_; }
	void SendPhase( uint8_t phase, uint8_t period, const uint8_t *leds, bool with_value, int32_t value );
	void SendOffset( int32_t offset_us );
	void SendDrift( double rate, bool with_offset, int32_t offset_us );
	void Broad( uint64_t now_us, const Preset &ps, const LedObservation &obs );
	void Bg( uint64_t now_us, const Preset &ps, const LedObservation &obs );
	static int DecodeSlot( const uint8_t *state );
	// BG dark-gap centre for one camera; -1 when the window is unusable.
	double BgEstimate( int cam ) const;

	Send send_;
	Phase phase_ = kInit;
	Phase prev_ = kInit;
	int preset_ = 1;
	bool good_ = false;
	bool hold_prescan_ = false;
	bool hold_bg_ = false;
	uint64_t sub_since_ = 0, phase_since_ = 0, last_frame_ = 0;
	int retries_ = 0;
	uint8_t leds_[4] = { 0xff, 0xff, 0xff, 0xff };

	// Per-camera frame history (newest last), RESEARCH.md §7.8.
	static constexpr int kHist = 33;  // BG 20, BROAD 11 (33 with divider 3)
	float w_[kLedCameras][kHist] = {};
	uint16_t n_[kLedCameras][kHist] = {};
	int hist_len_ = 0;
	void Push( const LedObservation &obs );
	// Lit (2) / dark (0) states of the newest L frames, oldest first.
	// Returns false (no states) for the doc's statuses 4, 9 and 10.
	bool States( int cam, int L, uint8_t *state, float *max_w ) const;
	// The same over every div-th frame of the newest 11·div, starting at
	// frame phase (0..div-1) of that span.
	bool StatesEvery( int cam, int div, int phase, uint8_t *state ) const;
	int DecodeBest( int div, int *phase_hits ) const;
	int broad_div_ = 1;
	// BROAD decode statistics for the 1 s log.
	int dec_hits_[4] = {};  // [0] divider 1, [1..3] divider 3 phases
	int dec_frames_ = 0;
	uint64_t dec_log_at_ = 0;

	// BROAD: last three accepted decodes, grid skips, accumulated slots.
	int slot_hist_[3] = { -1, -1, -1 };
	int prev_slot_ = 4;
	int grid_skips_ = 0;
	uint64_t last_code_ = 0, centred_since_ = 0;
	int slot_sum_ = 0;
	DriftFit broad_fit_, bg_fit_;
	int bg_fits_ = 0;
	double last_slope_ = 1;

	// BG: last accepted dark-gap centre x (steps) and the accumulated
	// correction (phase = x + acc).
	double x_ = 10, acc_ = 0;
	int x_skips_ = 0;
	bool bg_from_stable_ = false;
	uint64_t stable_left_ = 0;
};
