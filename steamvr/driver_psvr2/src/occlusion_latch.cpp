// SPDX-License-Identifier: GPL-2.0
// See occlusion_latch.h.
#include "occlusion_latch.h"

#include <algorithm>
#include <cmath>

namespace
{
using vrmath::Vec3;

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr uint64_t kTrackedUs = 300000, kMaxGapUs = 600000, kReleaseUs = 500000;
constexpr double kMaxDist = 1.0, kRadius = 0.0875, kCapRad = 20.0 * kDeg, kMarginRad = 5.0 * kDeg;
constexpr double kAbortDist = 0.15;

Vec3 Sub( const Vec3 &a, const Vec3 &b ) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
double Dot( const Vec3 &a, const Vec3 &b ) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double Norm( const Vec3 &a ) { return std::sqrt( Dot( a, a ) ); }

// View-ray frame of a point (Sony 0x1801b91d0): at the point, z along the
// ray from the rig origin (shortest rotation from +z).
vrmath::Pose RayFrame( const Vec3 &p )
{
	const double n = Norm( p );
	vrmath::Pose f;
	f.pos = p;
	f.rot = vrmath::Identity();
	if ( n < 1e-9 )
		return f;
	const Vec3 u{ p.x / n, p.y / n, p.z / n };
	const Vec3 axis{ -u.y, u.x, 0.0 };  // z x u
	const double s = Norm( axis ), c = u.z;
	if ( s < 1e-12 )
	{
		if ( c < 0.0 )
			f.rot = vrmath::AxisAngle( 1, 0, 0, 3.14159265358979323846 );
		return f;
	}
	f.rot = vrmath::AxisAngle( axis.x / s, axis.y / s, 0.0, std::atan2( s, c ) );
	return f;
}

double AngularRadius( double dist ) { return std::asin( std::min( 1.0, kRadius / std::max( dist, kRadius ) ) ); }

}  // namespace

OcclusionLatch::Out OcclusionLatch::Frame( uint64_t t, const HandIn in[2] )
{
	Out o;
	auto tracked = [&]( const HandIn &h ) { return h.fix_now || ( h.last_fix_us && t < h.last_fix_us + kTrackedUs ); };

	if ( state_ == 0 )
	{
		if ( !in[0].has_state && !in[1].has_state )
			return o;
		if ( !in[0].has_state || !in[1].has_state )
			return o;  // both positions are needed for the roles
		const double z0 = in[0].pos.z, z1 = in[1].pos.z;
		if ( z0 <= 0.0 || z1 <= 0.0 || z0 == z1 )
			return o;
		const int c = z0 < z1 ? 0 : 1, l = 1 - c;
		const HandIn &C = in[c], &L = in[l];
		if ( !tracked( C ) || tracked( L ) || !L.last_fix_us || t >= L.last_fix_us + kMaxGapUs )
			return o;
		const Vec3 pc = C.fix_now ? C.fix_pos : C.pos;
		const double dc = Norm( pc ), dl = Norm( L.pos );
		if ( dl >= kMaxDist || dc >= kMaxDist )
			return o;
		const double cosang = Dot( pc, L.pos ) / ( dc * dl );
		if ( cosang <= 0.0 )
			return o;
		const double theta = std::acos( std::clamp( cosang, -1.0, 1.0 ) );
		const double lim = std::min( kCapRad, std::fabs( AngularRadius( dc ) - AngularRadius( dl ) ) + kMarginRad );
		if ( theta > lim )
			return o;
		carrier_ = c;
		latched_ = l;
		rel_ = vrmath::Compose( vrmath::Invert( RayFrame( pc ) ), RayFrame( L.pos ) );
		held_ = L.pos;
		state_ = 1;
		entries_++;
		// Fall through: output this frame already.
	}

	const HandIn &C = in[carrier_], &L = in[latched_];
	switch ( state_ )
	{
	case 1:
		if ( L.fix_now )
		{
			state_ = 2;
			release_us_ = t;
			break;
		}
		if ( !C.fix_now || Norm( C.fix_pos ) >= kMaxDist )
		{
			state_ = 3;
			last_end_us_ = t;
			return o;
		}
		held_ = vrmath::Compose( RayFrame( C.fix_pos ), rel_ ).pos;
		o.action = kPublish;
		o.hand = latched_;
		o.pos = held_;
		return o;
	case 3:
		if ( L.fix_now )
		{
			state_ = 0;
			last_end_us_ = t;
		}
		return o;
	default:
		break;
	}

	// Release: pull the held position onto the hand's own fix within 0.5 s.
	if ( !L.fix_now )
		return o;  // keep the held position; resume when it is seen again
	const double frac = std::min( 1.0, double( t - release_us_ ) / double( kReleaseUs ) );
	held_ = vrmath::Add( held_, vrmath::Scale( Sub( L.fix_pos, held_ ), frac ) );
	if ( frac >= 1.0 || Norm( Sub( held_, L.fix_pos ) ) > kAbortDist )
	{
		state_ = 0;
		last_end_us_ = t;
		return o;
	}
	o.action = kReplace;
	o.hand = latched_;
	o.pos = held_;
	return o;
}
