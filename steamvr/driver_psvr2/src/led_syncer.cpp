// SPDX-License-Identifier: GPL-2.0
#include "led_syncer.h"

#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "driverlog.h"

namespace
{
constexpr uint64_t kSec = 1000000;
constexpr uint32_t kDefaultCycle = 16683350;  // libpad's 1frame_cycle reset value

void Put32( uint8_t *b, int32_t v ) { std::memcpy( b, &v, 4 ); }
}  // namespace

const char *LedSyncer::Name( Phase p )
{
	static const char *const names[] = { "INIT", "PRESCAN", "BROAD", "BG", "STABLE", "LED ALL OFF" };
	return p <= kAllOff ? names[p] : "?";
}

// ledsync.md §2.4: step and per-phase search windows (µs), preset 1 / 2.
LedSyncer::Preset LedSyncer::Timing( int preset )
{
	if ( preset == 2 )
		return { 50, 2000, 2100, 1500, 600 };
	return { 105, 4200, 5250, 3150, 1260 };
}

// ---- commands (ledsync.md §2.1) -------------------------------------------

void LedSyncer::SendPhase( uint8_t phase, uint8_t period, const uint8_t *leds, bool with_value, int32_t value )
{
	uint8_t c[11] = { 1, phase, period, 0xff, 0xff, 0xff, 0xff };
	if ( leds )
		std::memcpy( c + 3, leds, 4 );
	if ( with_value )
		Put32( c + 7, value );
	send_( c, with_value ? 11 : 7 );
}

void LedSyncer::SendOffset( int32_t offset_us )
{
	uint8_t c[5] = { 4 };
	Put32( c + 1, offset_us );
	send_( c, 5 );
}

// rate: dimensionless cycle correction; libpad applies cycle·(1 + rate).
// Sony is documented to send the fit slope raw (steps/µs), but on hardware
// (2026-11-18) that changed the cycle by 0.14 ppm against an 11 ppm drift,
// so every BG correction was >= 40 µs, the fit kept resetting, and BG never
// locked. The callers convert the slope to µs/µs instead (own method).
void LedSyncer::SendDrift( double rate, bool with_offset, int32_t offset_us )
{
	uint8_t c[9] = { uint8_t( with_offset ? 5 : 3 ) };
	const float f = float( std::fmax( -2e-4, std::fmin( 2e-4, rate ) ) );  // ±200 ppm
	std::memcpy( c + 1, &f, 4 );
	if ( with_offset )
		Put32( c + 5, offset_us );
	send_( c, with_offset ? 9 : 5 );
	retries_ = 0;  // a successful correction resets the retry counter (§4.2)
}

// ---- drift fit (ledsync.md §5.5) ------------------------------------------

void LedSyncer::DriftFit::Push( uint64_t now_us, double value )
{
	if ( n == 0 )
		t0 = now_us;
	pushes++;
	if ( n > 0 && std::fabs( v[n - 1] - value ) <= DBL_EPSILON )
		return;  // same value: one run, keep its earliest time
	if ( n == kMax )
	{
		std::memmove( v, v + 1, sizeof( double ) * ( kMax - 1 ) );
		std::memmove( t, t + 1, sizeof( double ) * ( kMax - 1 ) );
		n--;
	}
	v[n] = value;
	t[n] = double( now_us - t0 );
	n++;
}

bool LedSyncer::DriftFit::Fit()
{
	if ( fitted )
		return true;
	if ( n < 4 )
		return false;
	double sv = 0, st = 0, svt = 0, stt = 0;
	const int m = n - 1;  // every run but the first
	for ( int i = 1; i < n; i++ )
	{
		sv += v[i];
		st += t[i];
		svt += v[i] * t[i];
		stt += t[i] * t[i];
	}
	const double den = m * stt - st * st;
	if ( std::fabs( den ) <= DBL_EPSILON )
		return false;
	slope = ( m * svt - sv * st ) / den;
	fitted = true;
	return true;
}

// ---- phases ----------------------------------------------------------------

void LedSyncer::SetGood( bool good, uint64_t now_us )
{
	if ( good != good_ )
	{
		good_ = good;
		sub_since_ = now_us;
	}
}

void LedSyncer::ForceInit()
{
	if ( phase_ == kStable )
		stable_left_ = last_frame_;
	phase_ = kInit;
	good_ = false;
	hist_len_ = 0;
}

void LedSyncer::ForceAllOff()
{
	if ( phase_ == kAllOff )
		return;
	if ( phase_ == kStable )
		stable_left_ = last_frame_;
	prev_ = phase_;
	phase_ = kAllOff;
	retries_ = 2;
	hist_len_ = 0;
	const uint8_t c[2] = { 1, kAllOff };
	send_( c, 2 );
}

void LedSyncer::Enter( Phase p, uint64_t now_us )
{
	const Preset ps = Timing( preset_ );
	if ( phase_ == kStable )
		stable_left_ = now_us;
	prev_ = phase_;
	phase_ = p;
	phase_since_ = sub_since_ = now_us;
	good_ = false;
	hist_len_ = 0;
	DriverLog( "psvr2: ledsync: %s -> %s (retries %d)", Name( prev_ ), Name( p ), retries_ );

	switch ( p )
	{
	case kPrescan:
		if ( prev_ == kBroad && retries_ < 2 )
			retries_++;
		// Len 7 (no frame-cycle reset) until two BROAD rounds failed.
		if ( retries_ < 2 )
			SendPhase( kPrescan, uint8_t( ps.prescan / ps.step ), nullptr, false, 0 );
		else
			SendPhase( kPrescan, uint8_t( ps.prescan / ps.step ), nullptr, true, int32_t( kDefaultCycle ) );
		break;
	case kBroad:
		for ( int &s : slot_hist_ )
			s = -1;
		prev_slot_ = 4;
		grid_skips_ = 0;
		slot_sum_ = 0;
		broad_fit_.Reset();
		last_code_ = now_us;
		centred_since_ = 0;
		SendPhase( kBroad, uint8_t( ps.broad / ps.step ), leds_, true, 0 );
		break;
	case kBg:
	{
		bg_from_stable_ = prev_ == kStable;
		// Keep the STABLE drift fit for a fast re-lock if it is < 300 s old.
		if ( !( bg_from_stable_ && now_us - stable_left_ < 300 * kSec ) )
		{
			bg_fit_.Reset();
			bg_fits_ = 0;
		}
		x_skips_ = 0;
		const int32_t v = bg_from_stable_ ? ps.stable / 2 - ps.bg / 2 : 21 * ps.step - ps.bg / 2;
		SendPhase( kBg, uint8_t( ps.bg / ps.step ), leds_, true, v );
		break;
	}
	case kStable:
	{
		const int32_t v = ps.bg / 2 - ps.stable / 2 + int32_t( std::lround( ps.step * ( 10.0 - x_ ) ) );
		SendPhase( kStable, uint8_t( ps.stable / ps.step ), nullptr, true, v );
		break;
	}
	case kAllOff:
	{
		retries_ = 2;
		const uint8_t c[2] = { 1, kAllOff };
		send_( c, 2 );
		break;
	}
	default:
		break;
	}
}

void LedSyncer::Frame( uint64_t now_us, int preset, const LedObservation &obs )
{
	if ( preset != 1 && preset != 2 )
		preset = 1;
	// > 5 s since the last pass, or a preset change: start over (§3.2).
	if ( ( last_frame_ && now_us - last_frame_ > 5 * kSec ) || preset != preset_ )
		ForceInit();
	preset_ = preset;
	last_frame_ = now_us;
	const Preset ps = Timing( preset_ );
	Push( obs );

	switch ( phase_ )
	{
	case kInit:
	case kAllOff:
		Enter( kPrescan, now_us );
		break;
	case kPrescan:
		SetGood( obs.visible, now_us );
		if ( !hold_prescan_ && good_ && InState( now_us ) >= ( retries_ >= 2 ? 3 : 1 ) * kSec )
		{
			std::memcpy( leds_, obs.sync_leds, 4 );
			Enter( kBroad, now_us );
		}
		break;
	case kBroad:
		Broad( now_us, ps, obs );
		break;
	case kBg:
		Bg( now_us, ps, obs );
		break;
	case kStable:
		SetGood( obs.visible, now_us );
		if ( !good_ && InState( now_us ) >= 15 * kSec )
			Enter( kPrescan, now_us );
		else if ( good_ && !hold_bg_ && now_us - phase_since_ >= 100 * kSec )
			Enter( kBg, now_us );  // periodic phase re-check
		break;
	}
}

// ---- per-camera history (RESEARCH.md §7.8) ---------------------------------

void LedSyncer::Push( const LedObservation &obs )
{
	for ( int c = 0; c < kLedCameras; c++ )
	{
		std::memmove( w_[c], w_[c] + 1, sizeof( float ) * ( kHist - 1 ) );
		std::memmove( n_[c], n_[c] + 1, sizeof( uint16_t ) * ( kHist - 1 ) );
		w_[c][kHist - 1] = obs.sync_weight[c];
		n_[c][kHist - 1] = obs.count[c];
	}
	if ( hist_len_ < kHist )
		hist_len_++;
}

bool LedSyncer::States( int cam, int L, uint8_t *state, float *max_w ) const
{
	if ( hist_len_ < L )
		return false;
	const float *w = w_[cam] + kHist - L;
	const uint16_t *n = n_[cam] + kHist - L;
	float hi = 0, lo = FLT_MAX;
	for ( int k = L - 1; k >= 0; k-- )
	{
		if ( std::fabs( w[k] ) <= FLT_EPSILON )
			return false;  // history shorter than L (status 4)
		if ( n[k] > 0 )
		{
			hi = std::fmax( hi, w[k] );
			lo = std::fmin( lo, w[k] );
		}
	}
	if ( hi <= FLT_EPSILON || lo == FLT_MAX || hi - lo < 0.1f )
		return false;  // "NO MAX MIN" / "NO DIF MAX MIN"
	const float thr = ( hi + lo ) * 0.5f;
	for ( int k = 0; k < L; k++ )
		state[k] = n[k] > 0 && w[k] > thr ? 2 : 0;
	if ( max_w )
		*max_w = hi;
	return true;
}

// BROAD blink code (§7.9): frames 0..4 dark, 5..7 lit, 8..10 the 3-bit
// Gray code of column/2 (frame 8 = MSB). Only even columns 0..12 exist.
int LedSyncer::DecodeSlot( const uint8_t *state )
{
	for ( int col = 0; col <= 12; col += 2 )
	{
		const int k = col / 2, gray = k ^ ( k >> 1 );
		bool match = true;
		for ( int f = 0; f < 11 && match; f++ )
		{
			int bit = f >= 5 && f <= 7;
			if ( f >= 8 )
				bit = ( gray >> ( 10 - f ) ) & 1;
			if ( ( state[f] == 2 ) != ( bit == 1 ) )
				match = false;
		}
		if ( match )
			return col;
	}
	return -1;
}

bool LedSyncer::StatesEvery( int cam, int div, int phase, uint8_t *state ) const
{
	const int span = 11 * div;
	if ( hist_len_ < span )
		return false;
	const float *w = w_[cam] + kHist - span;
	const uint16_t *n = n_[cam] + kHist - span;
	float hi = 0, lo = FLT_MAX;
	for ( int f = 0; f < 11; f++ )
	{
		const int k = phase + f * div;
		if ( std::fabs( w[k] ) <= FLT_EPSILON )
			return false;
		if ( n[k] > 0 )
		{
			hi = std::fmax( hi, w[k] );
			lo = std::fmin( lo, w[k] );
		}
	}
	if ( hi <= FLT_EPSILON || lo == FLT_MAX || hi - lo < 0.1f )
		return false;
	const float thr = ( hi + lo ) * 0.5f;
	for ( int f = 0; f < 11; f++ )
	{
		const int k = phase + f * div;
		state[f] = n[k] > 0 && w[k] > thr ? 2 : 0;
	}
	return true;
}

// Across cameras (and, with div 3, the three frame phases) the slot closest
// to the previous accepted one; -1 if none decodes. phase_hits[p] counts the
// cameras that decoded in phase p.
int LedSyncer::DecodeBest( int div, int *phase_hits ) const
{
	int best = -1;
	for ( int p = 0; p < div; p++ )
		for ( int c = 0; c < kLedCameras; c++ )
		{
			uint8_t st[11];
			if ( !( div == 1 ? States( c, 11, st, nullptr ) : StatesEvery( c, div, p, st ) ) )
				continue;
			const int s = DecodeSlot( st );
			if ( s < 0 )
				continue;
			if ( phase_hits )
				phase_hits[p]++;
			if ( std::abs( s - prev_slot_ ) <= 32 &&
			     ( best < 0 || std::abs( s - prev_slot_ ) < std::abs( best - prev_slot_ ) ) )
				best = s;
		}
	return best;
}

void LedSyncer::Broad( uint64_t now_us, const Preset &ps, const LedObservation &obs )
{
	if ( std::memcmp( leds_, obs.sync_leds, 4 ) != 0 )
	{
		std::memcpy( leds_, obs.sync_leds, 4 );
		uint8_t c[5] = { 2 };
		std::memcpy( c + 1, leds_, 4 );
		send_( c, 5 );
		hist_len_ = 0;  // a different LED restarts the code
	}
	SetGood( obs.visible, now_us );

	// Across cameras, the slot closest to the previous accepted one. Both
	// dividers are decoded for the log; broad_div_ selects the one used.
	int h1[1] = {}, h3[3] = {};
	const int best1 = DecodeBest( 1, h1 ), best3 = DecodeBest( 3, h3 );
	const int best = broad_div_ == 3 ? best3 : best1;
	dec_frames_++;
	dec_hits_[0] += h1[0] > 0;
	for ( int p = 0; p < 3; p++ )
		dec_hits_[1 + p] += h3[p] > 0;
	if ( now_us >= dec_log_at_ )
	{
		if ( dec_log_at_ )
			DriverLog( "psvr2: ledsync: BROAD decodes in %d frames: every frame %d, every 3rd %d/%d/%d (phase 0/1/2); "
			           "divider %d, slot %d",
			           dec_frames_, dec_hits_[0], dec_hits_[1], dec_hits_[2], dec_hits_[3], broad_div_, best );
		dec_log_at_ = now_us + kSec;
		dec_frames_ = 0;
		for ( int &v : dec_hits_ )
			v = 0;
	}

	// "3 consecutive" counts decodes that reached the test, not frames: the
	// 11-frame window lines up with the repeating code once per cycle.
	if ( best >= 0 )
	{
		slot_hist_[0] = slot_hist_[1];
		slot_hist_[1] = slot_hist_[2];
		slot_hist_[2] = best;
		last_code_ = now_us;
		if ( slot_hist_[0] == best && slot_hist_[1] == best )
		{
			if ( best - 4 > 4 )
			{
				if ( ++grid_skips_ >= 6 )
				{
					Enter( kBroad, now_us );  // GRID SKIP x6: restart BROAD
					return;
				}
			}
			else if ( best != 4 )
			{
				// One Gray step (2 slots) = 5 steps.
				const int32_t off = ( best - 4 ) * ps.step * 5 / 2;
				slot_sum_ += best - 4;
				broad_fit_.Push( now_us, slot_sum_ );
				if ( broad_fit_.pushes >= 10 && broad_fit_.Fit() && std::fabs( broad_fit_.slope ) > DBL_EPSILON )
				{
					// Slots/µs to µs/µs (a slot is 2.5 steps). A positive
					// offset moves the LED window later; the corrections
					// keep coming in the direction the cycle must change.
					SendDrift( broad_fit_.slope * ps.step * 5 / 2, true, off );
					broad_fit_.Reset();
				}
				else
					SendOffset( off );
				centred_since_ = 0;
				hist_len_ = 0;  // the controller moved; old frames are stale
				for ( int &s : slot_hist_ )
					s = -1;
			}
			else if ( !centred_since_ )
				centred_since_ = now_us;
			prev_slot_ = best;
		}
	}

	if ( centred_since_ && now_us - centred_since_ >= ( retries_ ? 13 : 3 ) * kSec )
		Enter( kBg, now_us );
	else if ( now_us - last_code_ >= 10 * kSec )
		Enter( kPrescan, now_us );
}

// ---- BG (RESEARCH.md §7.10) ----------------------------------------------

// The 20-frame window must carry the framing marker (frames 0-3 lit, dark,
// dark, lit; frames 17-19 lit, dark, lit). Frames 4..16 hold the sweep; x
// is the centre of the frames darker than half the window's maximum.
double LedSyncer::BgEstimate( int cam ) const
{
	constexpr int L = kHist;
	uint8_t st[L];
	float hi = 0;
	if ( !States( cam, L, st, &hi ) )
		return -1;
	static const uint8_t marker[7][2] = { { 0, 2 }, { 1, 0 }, { 2, 0 }, { 3, 2 }, { 17, 2 }, { 18, 0 }, { 19, 2 } };
	for ( const auto &m : marker )
		if ( st[m[0]] != m[1] )
			return -1;  // status 5
	const float *w = w_[cam] + kHist - L;
	const double floor = hi * 0.5;
	double num = 0, den = 0;
	for ( int k = 4; k <= L - 4; k++ )
	{
		const double d = std::fmax( 0.0, floor - w[k] );
		num += k * d;
		den += d;
	}
	return num != 0 && den != 0 ? num / den : -1;
}

void LedSyncer::Bg( uint64_t now_us, const Preset &ps, const LedObservation &obs )
{
	if ( std::memcmp( leds_, obs.sync_leds, 4 ) != 0 )
	{
		std::memcpy( leds_, obs.sync_leds, 4 );
		uint8_t c[5] = { 2 };
		std::memcpy( c + 1, leds_, 4 );
		send_( c, 5 );
		hist_len_ = 0;
	}
	if ( !obs.visible )
		SetGood( false, now_us );

	// Among the cameras, the estimate closest to the last accepted x.
	double x = -1;
	for ( int c = 0; c < kLedCameras; c++ )
	{
		const double e = BgEstimate( c );
		if ( e >= 0 && ( x < 0 || std::fabs( e - x_ ) < std::fabs( x - x_ ) ) )
			x = e;
	}
	if ( x >= 0 )
	{
		SetGood( true, now_us );
		// Skip: with a fit under way, an x more than one step off centre is
		// dropped, at most three times in a row.
		if ( bg_fit_.n >= 2 && bg_fits_ > 0 && std::fabs( x - 10 ) > 1 && x_skips_ < 3 )
			x_skips_++;
		else
		{
			x_skips_ = 0;
			x_ = x;
			bg_fit_.Push( now_us, x + acc_ );
			if ( x < 9 || x > 11 )
			{
				const double corr = ( 10 - x ) * ps.step;
				if ( std::fabs( corr ) >= 40 )
					bg_fit_.Reset();
				acc_ -= 10 - x;
				SendOffset( int32_t( corr ) );
				hist_len_ = 0;  // the sweep moved; wait for a fresh window
			}
			// Sony needs 40 samples for every fit after the first. On
			// hardware (2026-11-18) BG gave ~1.3 samples/s, and a ~4 ppm
			// leftover drift reached the 40 µs fit reset in ~25 s, before
			// 40 samples: the second fit never came and BG never locked.
			// 10 samples for every fit (own choice).
			const int need = 10;
			if ( bg_fit_.pushes >= need && bg_fit_.Fit() )
			{
				// Steps/µs to µs/µs. The pushed value x + acc falls as the
				// corrections (10 - x)·step rise, so the cycle changes
				// against the slope.
				if ( std::fabs( bg_fit_.slope ) > DBL_EPSILON )
					SendDrift( -bg_fit_.slope * ps.step, false, 0 );
				last_slope_ = bg_fit_.slope;
				bg_fits_++;
				bg_fit_.Reset();
			}
		}
	}

	const bool fast = bg_from_stable_ && now_us - stable_left_ < 300 * kSec;
	if ( good_ && bg_fits_ >= 2 && std::fabs( last_slope_ ) < 2e-6 && InState( now_us ) > ( fast ? 3 : 15 ) * kSec )
		Enter( kStable, now_us );
	else if ( !good_ && InState( now_us ) >= 13 * kSec )
		Enter( kPrescan, now_us );
}
