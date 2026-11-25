// SPDX-License-Identifier: GPL-2.0
// See arm_model.h.
#include "arm_model.h"

#include <algorithm>
#include <cmath>

namespace armmodel
{
namespace
{
using vrmath::Vec3;

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr double kShoulderX = 0.175, kShoulderY = -0.185;
constexpr double kUpper = 0.28, kFore = 0.27, kGripToWrist = 0.08, kReach = 1.0;
constexpr double kHintX = 0.4, kHintY = -0.6, kHintBack = 0.4, kHintFront = 0.3, kHintRange = 0.4;
constexpr double kHingeMax = 160.0 * kDeg;
constexpr double kViewPitch = 5.0 * kDeg, kViewCone = 80.0 * kDeg, kShoulderDist = 1.0;

Vec3 Sub( const Vec3 &a, const Vec3 &b ) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
double Dot( const Vec3 &a, const Vec3 &b ) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double Norm( const Vec3 &a ) { return std::sqrt( Dot( a, a ) ); }
Vec3 Unit( const Vec3 &a, const Vec3 &fallback )
{
	const double n = Norm( a );
	return n > 1e-9 ? vrmath::Scale( a, 1.0 / n ) : fallback;
}

Vec3 Shoulder( const Vec3 &head_pos, const vr::HmdQuaternion_t &head_rot, bool right )
{
	return vrmath::Add( head_pos, vrmath::Rotate( head_rot, { right ? kShoulderX : -kShoulderX, kShoulderY, 0.0 } ) );
}

}  // namespace

Result Solve( const Vec3 &head_pos, const vr::HmdQuaternion_t &head_rot, const vr::HmdQuaternion_t &ctrl_rot,
              double hand_y, bool right )
{
	Result r;
	r.shoulder = Shoulder( head_pos, head_rot, right );

	// Elbow hint in the head frame (Sony's PC table).
	const double d = std::clamp( hand_y - head_pos.y, -kHintRange, 0.0 );
	const double f = ( d + kHintRange ) / kHintRange;
	const double zh = ( 1.0 - f ) * kHintBack - f * kHintFront;
	const Vec3 hint = vrmath::Add( head_pos, vrmath::Rotate( head_rot, { right ? kHintX : -kHintX, kHintY, zh } ) );

	// Forearm along the controller's +z (grip -> wrist -> elbow) [ours].
	const Vec3 back = Unit( vrmath::Rotate( ctrl_rot, { 0.0, 0.0, 1.0 } ), { 0.0, 0.0, 1.0 } );
	const Vec3 fwd = vrmath::Scale( back, -1.0 );  // elbow -> wrist
	const Vec3 to_hint = Unit( Sub( hint, r.shoulder ), { 0.0, -1.0, 0.0 } );
	Vec3 upper = Unit( vrmath::Add( to_hint, fwd ), to_hint );
	// Hinge: the bend (angle between upper arm and forearm) stays <= 160 deg.
	if ( std::acos( std::clamp( Dot( upper, fwd ), -1.0, 1.0 ) ) > kHingeMax )
		upper = to_hint;

	r.elbow = vrmath::Add( r.shoulder, vrmath::Scale( upper, kUpper ) );
	r.wrist = vrmath::Add( r.elbow, vrmath::Scale( fwd, kFore ) );
	r.hand = vrmath::Add( r.wrist, vrmath::Scale( fwd, kGripToWrist ) );

	// Longest reach 1.0 m: a straight arm beyond it.
	const Vec3 sh = Sub( r.hand, r.shoulder );
	const double n = Norm( sh );
	if ( n > kReach )
		r.hand = vrmath::Add( r.shoulder, vrmath::Scale( sh, kReach / n ) );
	return r;
}

bool OutOfView( const Vec3 &head_pos, const vr::HmdQuaternion_t &head_rot, const Vec3 &ctrl_pos, bool right )
{
	// View axis: head -z pitched 5 deg down.
	const Vec3 axis = vrmath::Rotate( head_rot, { 0.0, -std::sin( kViewPitch ), -std::cos( kViewPitch ) } );
	const Vec3 to = Unit( Sub( ctrl_pos, head_pos ), axis );
	const bool off = std::acos( std::clamp( Dot( axis, to ), -1.0, 1.0 ) ) >= kViewCone;
	return off && Norm( Sub( ctrl_pos, Shoulder( head_pos, head_rot, right ) ) ) <= kShoulderDist;
}

}  // namespace armmodel
