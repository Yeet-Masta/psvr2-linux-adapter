// SPDX-License-Identifier: GPL-2.0
// Small quaternion / vector helpers on the OpenVR types (w, x, y, z).
#pragma once

#include <cmath>

#include "openvr_driver.h"

namespace vrmath
{

struct Vec3
{
	double x = 0, y = 0, z = 0;
};

inline vr::HmdQuaternion_t Identity()
{
	return { 1.0, 0.0, 0.0, 0.0 };
}

// Hamilton product a * b.
inline vr::HmdQuaternion_t Mul( const vr::HmdQuaternion_t &a, const vr::HmdQuaternion_t &b )
{
	return {
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
	};
}

inline vr::HmdQuaternion_t Conj( const vr::HmdQuaternion_t &q )
{
	return { q.w, -q.x, -q.y, -q.z };
}

inline vr::HmdQuaternion_t Normalize( vr::HmdQuaternion_t q )
{
	const double n = std::sqrt( q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z );
	if ( n <= 0.0 )
		return Identity();
	return { q.w / n, q.x / n, q.y / n, q.z / n };
}

inline vr::HmdQuaternion_t AxisAngle( double x, double y, double z, double radians )
{
	const double s = std::sin( radians * 0.5 );
	return { std::cos( radians * 0.5 ), x * s, y * s, z * s };
}

inline Vec3 Rotate( const vr::HmdQuaternion_t &q, const Vec3 &v )
{
	const vr::HmdQuaternion_t p{ 0.0, v.x, v.y, v.z };
	const vr::HmdQuaternion_t r = Mul( Mul( q, p ), Conj( q ) );
	return { r.x, r.y, r.z };
}

inline Vec3 Add( const Vec3 &a, const Vec3 &b )
{
	return { a.x + b.x, a.y + b.y, a.z + b.z };
}

inline Vec3 Scale( const Vec3 &a, double s )
{
	return { a.x * s, a.y * s, a.z * s };
}

// Rotation about +Y (up) of q's forward (-Z) axis, in radians.
inline double Yaw( const vr::HmdQuaternion_t &q )
{
	const Vec3 f = Rotate( q, { 0.0, 0.0, -1.0 } );
	return std::atan2( -f.x, -f.z );
}

// A rigid transform: rotate, then translate.
struct Pose
{
	vr::HmdQuaternion_t rot = Identity();
	Vec3 pos;
};

inline Pose Compose( const Pose &a, const Pose &b )
{
	return { Mul( a.rot, b.rot ), Add( a.pos, Rotate( a.rot, b.pos ) ) };
}

inline Pose Invert( const Pose &p )
{
	const vr::HmdQuaternion_t r = Conj( p.rot );
	return { r, Scale( Rotate( r, p.pos ), -1.0 ) };
}

// Rotation matrix, row-major, of a unit quaternion.
inline void ToMatrix( const vr::HmdQuaternion_t &q, double R[9] )
{
	const double w = q.w, x = q.x, y = q.y, z = q.z;
	R[0] = 1 - 2 * ( y * y + z * z );
	R[1] = 2 * ( x * y - w * z );
	R[2] = 2 * ( x * z + w * y );
	R[3] = 2 * ( x * y + w * z );
	R[4] = 1 - 2 * ( x * x + z * z );
	R[5] = 2 * ( y * z - w * x );
	R[6] = 2 * ( x * z - w * y );
	R[7] = 2 * ( y * z + w * x );
	R[8] = 1 - 2 * ( x * x + y * y );
}

// Unit quaternion of a row-major rotation matrix.
inline vr::HmdQuaternion_t FromMatrix( const double R[9] )
{
	vr::HmdQuaternion_t q;
	const double tr = R[0] + R[4] + R[8];
	if ( tr > 0 )
	{
		const double s = std::sqrt( tr + 1.0 ) * 2;
		q = { 0.25 * s, ( R[7] - R[5] ) / s, ( R[2] - R[6] ) / s, ( R[3] - R[1] ) / s };
	}
	else if ( R[0] > R[4] && R[0] > R[8] )
	{
		const double s = std::sqrt( 1.0 + R[0] - R[4] - R[8] ) * 2;
		q = { ( R[7] - R[5] ) / s, 0.25 * s, ( R[1] + R[3] ) / s, ( R[2] + R[6] ) / s };
	}
	else if ( R[4] > R[8] )
	{
		const double s = std::sqrt( 1.0 + R[4] - R[0] - R[8] ) * 2;
		q = { ( R[2] - R[6] ) / s, ( R[1] + R[3] ) / s, 0.25 * s, ( R[5] + R[7] ) / s };
	}
	else
	{
		const double s = std::sqrt( 1.0 + R[8] - R[0] - R[4] ) * 2;
		q = { ( R[3] - R[1] ) / s, ( R[2] + R[6] ) / s, ( R[5] + R[7] ) / s, 0.25 * s };
	}
	return Normalize( q );
}

// Normalised linear blend; fine for the small steps it is used for.
inline vr::HmdQuaternion_t Nlerp( const vr::HmdQuaternion_t &a, vr::HmdQuaternion_t b, double t )
{
	if ( a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z < 0 )
		b = { -b.w, -b.x, -b.y, -b.z };
	return Normalize( { a.w + ( b.w - a.w ) * t, a.x + ( b.x - a.x ) * t, a.y + ( b.y - a.y ) * t,
	                    a.z + ( b.z - a.z ) * t } );
}

// Rotation angle of q, radians, 0..pi.
inline double Angle( const vr::HmdQuaternion_t &q )
{
	const double v = std::sqrt( q.x * q.x + q.y * q.y + q.z * q.z );
	return 2.0 * std::atan2( v, std::fabs( q.w ) );
}

// Wrap an angle to (-pi, pi].
inline double WrapPi( double a )
{
	while ( a > M_PI )
		a -= 2 * M_PI;
	while ( a <= -M_PI )
		a += 2 * M_PI;
	return a;
}

} // namespace vrmath
