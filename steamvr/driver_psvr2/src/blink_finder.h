// SPDX-License-Identifier: GPL-2.0
// Finds the blinking sync LED among a camera's blobs (our own method).
//
// Sony feeds LED sync the weight of the blob its pose-driven association
// assigned to the sync LED (docs/RESEARCH.md §7.8). Before a controller has
// a pose there is nothing to associate, and a lit room gives 50-80 steady
// blobs per camera (hardware test, 2026-11-18). This class tracks blob
// positions over the last kFrames frames and picks the one whose on/off
// pattern repeats with the phase's period (BROAD 11 frames, BG 20) while
// switching within it. Room lights are on in nearly every frame; flickering
// lamps switch, but at random, so they do not repeat.
//
// "Lit" is judged by weight, not presence: on hardware (2026-11-18) a dark
// sync LED often still showed a dim blob at its spot (a neighbouring LED),
// weight ~4 against ~13 lit, so a presence-only test saw it lit every frame.
#pragma once

#include <cstdint>

class BlinkFinder
{
public:
	static constexpr int kFrames = 48;

	// One LD frame of one camera. weight <= 0 entries are ignored. period:
	// the pattern length of the current phase (11 or 20), 0 = no pattern
	// (PRESCAN, STABLE): keep the chosen site but pick no new one.
	void Frame( const float *x, const float *y, const float *weight, int count, int period );
	// The chosen site this frame: present (blob seen now) and its weight.
	// False when no site qualifies.
	bool Sync( bool &present, float &weight ) const;
	// Position of the chosen site (for logs). False when none.
	bool Where( float &x, float &y ) const;
	void Reset();

private:
	static constexpr int kSites = 160;  // rooms show up to ~80 blobs per camera
	static constexpr float kRadius = 6.0f;  // px in the 508 image; hand jitter
	struct Site
	{
		bool used = false;
		float x = 0, y = 0;
		uint64_t seen = 0;  // presence bits, bit 0 = newest frame
		float weight = 0;   // newest weight when present
		float w[kFrames] = {};  // weight per frame (0 = absent), [0] = newest
		int age = 0;        // frames since the site was created
	};
	// Fraction of frames that match the frame one period earlier, or -1
	// when the site does not qualify. keep: the looser bar for the site
	// already chosen (the BG sweep and its corrections disturb a few frames).
	double Periodic( const Site &s, int period, bool keep = false ) const;
	// Lit bits over the last n frames: weight above the middle of the
	// site's own range when that range is wide enough, else presence.
	static uint64_t Lit( const Site &s, int n );

	Site sites_[kSites];
	int chosen_ = -1;
};
