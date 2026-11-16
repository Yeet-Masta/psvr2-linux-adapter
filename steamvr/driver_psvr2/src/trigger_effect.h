// SPDX-License-Identifier: GPL-2.0
// Adaptive trigger effects for the Sense controller.
//
// The effect kinds and parameter ranges are the ones PSVR2 Toolkit's IPC
// exposes to games (positions 0..9 along the trigger travel, strengths and
// amplitudes 0..8, frequency in Hz). Encode() turns one into the mode byte
// and ten parameter bytes of the controller's output report, using the
// DualSense zone encoding (mode ids 0x21 feedback, 0x25 weapon, 0x26
// vibration, which the Sense firmware shares).
#pragma once

#include <array>
#include <cstdint>

enum class TriggerEffectKind : uint8_t
{
	Off,
	Feedback,                   // position, strength
	Weapon,                     // start, end, strength
	Vibration,                  // position, amplitude, frequency
	MultiplePositionFeedback,   // strength[10]
	SlopeFeedback,              // start, end, start strength, end strength
	MultiplePositionVibration,  // frequency, amplitude[10]
};

struct TriggerEffect
{
	TriggerEffectKind kind = TriggerEffectKind::Off;
	uint8_t start = 0;          // position / start position
	uint8_t end = 0;            // end position
	uint8_t strength = 0;       // strength / amplitude / start strength
	uint8_t end_strength = 0;
	uint8_t frequency = 0;
	std::array<uint8_t, 10> zones{};  // per-position strength / amplitude

	bool operator==( const TriggerEffect &o ) const;
	bool operator!=( const TriggerEffect &o ) const { return !( *this == o ); }
};

struct EncodedTriggerEffect
{
	uint8_t mode;
	std::array<uint8_t, 10> params;
};

EncodedTriggerEffect EncodeTriggerEffect( const TriggerEffect &effect );
