// SPDX-License-Identifier: GPL-2.0
#include "blink_finder.h"

#include <algorithm>
#include <cstring>

namespace
{
constexpr uint64_t kMask = ( uint64_t( 1 ) << BlinkFinder::kFrames ) - 1;

int Ones( uint64_t v ) { return __builtin_popcountll( v ); }
// Frames since last seen (bit 0 = newest frame).
int Idle( uint64_t seen ) { return seen ? __builtin_ctzll( seen ) : 64; }
}  // namespace

void BlinkFinder::Reset()
{
	for ( auto &s : sites_ )
		s = Site{};
	chosen_ = -1;
}

void BlinkFinder::Frame( const float *x, const float *y, const float *weight, int count, int period )
{
	bool matched[kSites] = {};

	for ( auto &s : sites_ )
		if ( s.used )
		{
			s.seen = ( s.seen << 1 ) & kMask;
			std::memmove( s.w + 1, s.w, sizeof( s.w ) - sizeof( s.w[0] ) );
			s.w[0] = 0;
			s.age++;
		}
	for ( int i = 0; i < count; i++ )
	{
		if ( !( weight[i] > 0 ) )
			continue;
		int best = -1, free_slot = -1;
		float bd = kRadius * kRadius;
		for ( int k = 0; k < kSites; k++ )
		{
			Site &s = sites_[k];
			if ( !s.used )
			{
				if ( free_slot < 0 )
					free_slot = k;
				continue;
			}
			const float dx = s.x - x[i], dy = s.y - y[i], d2 = dx * dx + dy * dy;
			if ( !matched[k] && d2 <= bd )
			{
				bd = d2;
				best = k;
			}
		}
		if ( best < 0 )
		{
			// Full: drop the site absent longest, never the chosen one.
			if ( free_slot < 0 )
			{
				int worst = -1;
				for ( int k = 0; k < kSites; k++ )
					if ( k != chosen_ && ( worst < 0 || Idle( sites_[k].seen ) > Idle( sites_[worst].seen ) ) )
						worst = k;
				free_slot = worst;
			}
			sites_[free_slot] = Site{};
			sites_[free_slot].used = true;
			sites_[free_slot].x = x[i];
			sites_[free_slot].y = y[i];
			best = free_slot;
		}
		Site &s = sites_[best];
		matched[best] = true;
		s.seen |= 1;
		s.weight = weight[i];
		s.w[0] = weight[i];
		s.x += 0.5f * ( x[i] - s.x );  // follow a slowly moving hand
		s.y += 0.5f * ( y[i] - s.y );
	}
	for ( int k = 0; k < kSites; k++ )
		if ( sites_[k].used && Idle( sites_[k].seen ) >= kFrames )
		{
			sites_[k] = Site{};
			if ( k == chosen_ )
				chosen_ = -1;
		}

	// Keep the chosen site while it is seen at all and, when a pattern runs,
	// still follows it: STABLE (period 0) lights the LED every frame. In BG
	// (period 20) keep it unconditionally: it is the LED BROAD found, and
	// the BG sweep is re-centred after every estimate, so one period rarely
	// matches the next (hardware trace, 2026-11-18). A wrong site gives no
	// estimates and the syncer falls back to PRESCAN after 13 s.
	if ( chosen_ >= 0 && Idle( sites_[chosen_].seen ) < 24 &&
	     ( period == 0 || period == 20 || Periodic( sites_[chosen_], period, true ) >= 0 ) )
		return;
	chosen_ = -1;
	if ( period <= 0 )
		return;
	double best_p = -1;
	for ( int k = 0; k < kSites; k++ )
	{
		const Site &s = sites_[k];
		if ( !s.used )
			continue;
		const double p = Periodic( s, period );
		if ( p < 0 )
			continue;
		if ( p > best_p || ( p == best_p && s.weight > sites_[chosen_].weight ) )
		{
			best_p = p;
			chosen_ = k;
		}
	}
}

// Over the last (kFrames - period) frames, the share that equal the frame
// one period earlier. Qualifies at >= 90 % with 2..period-2 lit frames in
// the newest period (so it really switches), >= 80 % of the dark frames
// repeating, after 3 periods of history (at most the whole buffer). To keep
// a chosen site: >= 75 % and >= 60 %.
double BlinkFinder::Periodic( const Site &s, int period, bool keep ) const
{
	const int need = 3 * period < kFrames ? 3 * period : kFrames;
	if ( period <= 0 || s.age + 1 < need )
		return -1;
	const int have = s.age + 1 < kFrames ? s.age + 1 : kFrames;
	const int n = have - period;
	const uint64_t window = ( uint64_t( 1 ) << n ) - 1;
	const uint64_t on = Lit( s, have );
	const int same = n - Ones( ( on ^ ( on >> period ) ) & window );
	const int lit = Ones( on & ( ( uint64_t( 1 ) << period ) - 1 ) );
	if ( lit < 2 || lit > period - 2 )
		return -1;
	// The dark frames themselves must repeat: a steady light's random
	// dropouts pass the overall share but not this.
	const uint64_t dark_now = ~on & window, dark_then = ~( on >> period ) & window;
	if ( 5 * Ones( dark_now & dark_then ) < ( keep ? 3 : 4 ) * Ones( dark_now | dark_then ) )
		return -1;
	const double p = double( same ) / n;
	return p >= ( keep ? 0.75 : 0.9 ) ? p : -1;
}

uint64_t BlinkFinder::Lit( const Site &s, int n )
{
	// Robust range: 10th and 90th percentile, so one spike or dropout does
	// not move the threshold.
	float v[kFrames];
	std::copy( s.w, s.w + n, v );
	std::sort( v, v + n );
	const float lo = v[n / 10], hi = v[n - 1 - n / 10];
	// Room lights jitter by about +-0.5 (weights 1.4-2.8); a blinking LED
	// spans several units.
	if ( hi - lo < 2.0f )
		return s.seen & ( ( n < 64 ? uint64_t( 1 ) << n : 0 ) - 1 );
	const float thr = 0.5f * ( hi + lo );
	uint64_t bits = 0;
	for ( int k = 0; k < n; k++ )
		if ( s.w[k] > thr )
			bits |= uint64_t( 1 ) << k;
	return bits;
}

bool BlinkFinder::Sync( bool &present, float &weight ) const
{
	if ( chosen_ < 0 )
		return false;
	present = sites_[chosen_].seen & 1;
	weight = sites_[chosen_].weight;
	return true;
}

bool BlinkFinder::Where( float &x, float &y ) const
{
	if ( chosen_ < 0 )
		return false;
	x = sites_[chosen_].x;
	y = sites_[chosen_].y;
	return true;
}
