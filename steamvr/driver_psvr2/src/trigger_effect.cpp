// SPDX-License-Identifier: GPL-2.0
#include "trigger_effect.h"

#include <algorithm>

namespace
{
constexpr uint8_t kModeOff = 0x05;
constexpr uint8_t kModeFeedback = 0x21;
constexpr uint8_t kModeWeapon = 0x25;
constexpr uint8_t kModeVibration = 0x26;

// Per-zone 3-bit force levels (level 1..8 stored as 0..7) plus a 10-bit mask
// of the zones that are active.
struct Zones
{
	uint16_t active = 0;
	uint32_t force = 0;

	void Set( int zone, uint8_t level )
	{
		if ( level == 0 )
			return;
		active |= uint16_t( 1u << zone );
		force |= uint32_t( ( std::min<uint8_t>( level, 8 ) - 1 ) & 7 ) << ( 3 * zone );
	}
};

EncodedTriggerEffect Off()
{
	return { kModeOff, {} };
}

EncodedTriggerEffect ZoneEffect( uint8_t mode, const Zones &z, uint8_t frequency )
{
	if ( z.active == 0 )
		return Off();
	EncodedTriggerEffect e{ mode, {} };
	e.params[0] = uint8_t( z.active );
	e.params[1] = uint8_t( z.active >> 8 );
	for ( int i = 0; i < 4; i++ )
		e.params[2 + i] = uint8_t( z.force >> ( 8 * i ) );
	e.params[8] = frequency;  // vibration only; zero for feedback
	return e;
}
} // namespace

bool TriggerEffect::operator==( const TriggerEffect &o ) const
{
	return kind == o.kind && start == o.start && end == o.end && strength == o.strength &&
	       end_strength == o.end_strength && frequency == o.frequency && zones == o.zones;
}

EncodedTriggerEffect EncodeTriggerEffect( const TriggerEffect &fx )
{
	Zones z;
	switch ( fx.kind )
	{
	case TriggerEffectKind::Off:
		return Off();

	case TriggerEffectKind::Feedback:
		if ( fx.start > 9 )
			return Off();
		for ( int i = fx.start; i < 10; i++ )
			z.Set( i, fx.strength );
		return ZoneEffect( kModeFeedback, z, 0 );

	case TriggerEffectKind::MultiplePositionFeedback:
		for ( int i = 0; i < 10; i++ )
			z.Set( i, fx.zones[i] );
		return ZoneEffect( kModeFeedback, z, 0 );

	case TriggerEffectKind::SlopeFeedback:
	{
		if ( fx.start >= fx.end || fx.end > 9 || fx.strength < 1 || fx.end_strength < 1 )
			return Off();
		const float slope = float( fx.end_strength - fx.strength ) / float( fx.end - fx.start );
		for ( int i = fx.start; i < 10; i++ )
		{
			const int at = std::min<int>( i, fx.end );
			z.Set( i, uint8_t( fx.strength + slope * float( at - fx.start ) + 0.5f ) );
		}
		return ZoneEffect( kModeFeedback, z, 0 );
	}

	case TriggerEffectKind::Weapon:
	{
		if ( fx.start < 2 || fx.start > 7 || fx.end <= fx.start || fx.end > 8 || fx.strength == 0 )
			return Off();
		const uint16_t stops = uint16_t( ( 1u << fx.start ) | ( 1u << fx.end ) );
		EncodedTriggerEffect e{ kModeWeapon, {} };
		e.params[0] = uint8_t( stops );
		e.params[1] = uint8_t( stops >> 8 );
		e.params[2] = uint8_t( std::min<uint8_t>( fx.strength, 8 ) - 1 );
		return e;
	}

	case TriggerEffectKind::Vibration:
		if ( fx.start > 9 || fx.frequency == 0 )
			return Off();
		for ( int i = fx.start; i < 10; i++ )
			z.Set( i, fx.strength );
		return ZoneEffect( kModeVibration, z, fx.frequency );

	case TriggerEffectKind::MultiplePositionVibration:
		if ( fx.frequency == 0 )
			return Off();
		for ( int i = 0; i < 10; i++ )
			z.Set( i, fx.zones[i] );
		return ZoneEffect( kModeVibration, z, fx.frequency );
	}
	return Off();
}
