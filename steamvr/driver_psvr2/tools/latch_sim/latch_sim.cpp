// SPDX-License-Identifier: GPL-2.0
// Offline check of OcclusionLatch (docs/sony-re/controller-tracking.md §F.2): the far
// controller hides behind the near one, the near one moves sideways, the
// far one reappears. Exits non-zero on a wrong state or position.
//
//   ./build.sh && ./latch_sim
#include <cmath>
#include <cstdio>

#include "occlusion_latch.h"

using vrmath::Vec3;

namespace
{
double Dist( const Vec3 &a, const Vec3 &b )
{
	const double x = a.x - b.x, y = a.y - b.y, z = a.z - b.z;
	return std::sqrt( x * x + y * y + z * z );
}
int g_fail = 0;
void Check( bool ok, const char *what )
{
	std::printf( "%s %s\n", ok ? "ok  " : "FAIL", what );
	if ( !ok )
		g_fail++;
}
}  // namespace

int main()
{
	OcclusionLatch latch;
	const uint64_t dt = 16667;
	uint64_t t = 1000000;
	// Rig frame, +z forward: carrier 0.40 m out, the other hand 0.10 m
	// behind it on almost the same ray.
	Vec3 carrier{ 0.0, -0.10, 0.40 }, hidden{ 0.01, -0.12, 0.50 };
	OcclusionLatch::HandIn in[2];
	for ( auto &h : in )
		h.has_state = true;
	uint64_t hidden_fix = 0, carrier_fix = 0;
	auto frame = [&]( bool carrier_seen, bool hidden_seen, const Vec3 &hidden_true ) {
		in[0].pos = carrier;
		in[0].fix_now = carrier_seen;
		in[0].fix_pos = carrier;
		in[0].last_fix_us = carrier_fix;
		in[1].pos = hidden;  // the filter drifts on: keep its last estimate
		in[1].fix_now = hidden_seen;
		in[1].fix_pos = hidden_true;
		in[1].last_fix_us = hidden_fix;
		const OcclusionLatch::Out o = latch.Frame( t, in );
		if ( carrier_seen )
			carrier_fix = t;
		if ( hidden_seen )
			hidden_fix = t;
		t += dt;
		return o;
	};

	// Both seen for 0.5 s.
	for ( int i = 0; i < 30; i++ )
		frame( true, true, hidden );
	Check( latch.State() == 0, "idle while both are seen" );
	// Hidden hand lost: no latch before 0.3 s.
	OcclusionLatch::Out o;
	for ( int i = 0; i < 15; i++ )
		o = frame( true, false, hidden );
	Check( latch.State() == 0, "no latch within 0.3 s of the last fix" );
	for ( int i = 0; i < 4 && latch.State() == 0; i++ )
		o = frame( true, false, hidden );
	Check( latch.State() == 1 && latch.Latched() == 1, "latches the far hand after 0.3 s" );
	Check( o.action == OcclusionLatch::kPublish && Dist( o.pos, hidden ) < 1e-6, "first carried position = filter position" );
	// Carrier moves 10 cm right: the hidden hand keeps its ray offset.
	const Vec3 c0 = carrier;
	for ( int i = 0; i < 20; i++ )
	{
		carrier.x += 0.005;
		o = frame( true, false, hidden );
	}
	const double ang0 = std::atan2( hidden.x, hidden.z ) - std::atan2( c0.x, c0.z );
	const double ang1 = std::atan2( o.pos.x, o.pos.z ) - std::atan2( carrier.x, carrier.z );
	const double gap0 = Dist( hidden, {} ) - Dist( c0, {} ), gap1 = Dist( o.pos, {} ) - Dist( carrier, {} );
	std::printf( "     ray offset %.4f -> %.4f rad, depth gap %.4f -> %.4f m\n", ang0, ang1, gap0, gap1 );
	Check( o.action == OcclusionLatch::kPublish && std::fabs( ang1 - ang0 ) < 0.005 && std::fabs( gap1 - gap0 ) < 0.002,
	       "carried with the carrier (ray offset and depth gap kept)" );
	// Hidden hand reappears 3 cm from the carried point: release blend.
	const Vec3 held = o.pos;
	const Vec3 seen{ held.x + 0.03, held.y, held.z };
	o = frame( true, true, seen );
	Check( latch.State() == 2 && o.action == OcclusionLatch::kReplace && Dist( o.pos, held ) < 1e-9,
	       "release starts at the carried position" );
	bool monotone = true;
	double last = Dist( held, seen );
	int frames = 0;
	while ( latch.State() == 2 && frames < 60 )
	{
		o = frame( true, true, seen );
		frames++;
		if ( o.action == OcclusionLatch::kReplace )
		{
			const double d = Dist( o.pos, seen );
			monotone = monotone && d <= last + 1e-9;
			last = d;
		}
	}
	Check( latch.State() == 0 && monotone && frames <= 31, "blends onto the own fix within 0.5 s" );
	// Carrier lost while latched: dropped, no output until the hand is seen.
	carrier = c0;
	for ( int i = 0; i < 25; i++ )
		frame( true, false, hidden );
	Check( latch.State() == 1, "latches again" );
	o = frame( false, false, hidden );
	Check( latch.State() == 3 && o.action == OcclusionLatch::kNone, "carrier lost: dropped" );
	frame( true, true, hidden );
	Check( latch.State() == 0, "idle once the hand is seen again" );
	// No latch when the hands do not overlap.
	OcclusionLatch l2;
	OcclusionLatch::HandIn a[2];
	a[0].has_state = a[1].has_state = true;
	a[0].pos = a[0].fix_pos = { -0.3, 0.0, 0.4 };
	a[0].fix_now = true;
	a[1].pos = { 0.3, 0.0, 0.5 };
	a[1].last_fix_us = 1000000;
	l2.Frame( 1400000, a );
	Check( l2.State() == 0, "no latch for hands far apart" );
	std::printf( "%s\n", g_fail ? "FAIL" : "PASS" );
	return g_fail ? 1 : 0;
}
