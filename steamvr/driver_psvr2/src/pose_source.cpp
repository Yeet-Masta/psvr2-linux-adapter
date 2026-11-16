// SPDX-License-Identifier: GPL-2.0
#include "pose_source.h"

#include "libpsvr2.h"

#include <algorithm>
#include <cmath>
#include <ctime>

// ---------------------------------------------------------------------------
// Device -> OpenVR frame remap.
//
// libpsvr2 reports the headset's native raw frame (see kernel/psvr2_uapi.h):
//   position[0]: forward,  position[1]: up,  position[2]: right
//   orientation: quaternion (w, x, y, z) in that same raw frame
//
// OpenVR is right-handed: +x right, +y up, -z forward, metres. OpenXR uses the
// same convention, so we mirror the Monado PSVR2 driver's process_slam_record
// remap verbatim (see docs/references.md) — adopted as the reference after the
// in-headset pose-guide superseded the earlier hand-timed capture.
//
// The tracker's native upright is additionally rolled 90deg from OpenVR's, so
// after the basis change we left-multiply a fixed +90deg rotation about Z (the
// forward axis) — Monado's SLAM_POSE_CORRECTION. Confirmed on hardware: without
// it the rendered view sits 90deg rolled to the left.
// ---------------------------------------------------------------------------

// Hamilton product r = a * b, both/all (w, x, y, z).
static vr::HmdQuaternion_t QuatMul( const vr::HmdQuaternion_t &a, const vr::HmdQuaternion_t &b )
{
	return {
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
	};
}

static void RemapPose( const struct psvr2_pose &in, vr::DriverPose_t &out )
{
	// Position: raw (forward, up, right) -> OpenVR (right, up, -forward).
	out.vecPosition[0] = in.position[2];
	out.vecPosition[1] = in.position[1];
	out.vecPosition[2] = -in.position[0];

	// Orientation: basis change on the quaternion (matches process_slam_record).
	const vr::HmdQuaternion_t remapped = {
		in.orientation[0],   // w
		-in.orientation[2],  // x
		-in.orientation[1],  // y
		in.orientation[3],   // z
	};

	// Fixed +90deg roll about Z (sqrt(2)/2 ~= cos/sin 45deg), left-multiplied.
	static const double s = 0.70710678118654752;
	static const vr::HmdQuaternion_t kRollCorrection = { s, 0.0, 0.0, s };
	out.qRotation = QuatMul( kRollCorrection, remapped );
}

PoseSource::PoseSource() : dev_( psvr2_open() ) {}

PoseSource::~PoseSource()
{
	if ( dev_ )
		psvr2_close( dev_ );
}

bool PoseSource::HasPose() const
{
	return dev_ && psvr2_has_pose( dev_ );
}

// SteamVR extrapolates each pose to photon time with these velocities; with
// zero velocity every head turn judders (90 Hz) or doubles (120 Hz). Finite
// differences over a ~10 ms window on the device clock keep noise down.
void PoseSource::EstimateVelocity( const Sample &now, vr::DriverPose_t &out )
{
	constexpr double kWindow = 0.010, kMaxAge = 0.1, kSmooth = 0.5;
	history_.push_back( now );
	while ( history_.size() > 2 && now.t - history_[1].t >= kWindow )
		history_.pop_front();
	const Sample &old = history_.front();
	const double dt = now.t - old.t;
	if ( dt > 1e-4 && dt < kMaxAge )
	{
		// Rotation from old to now in the world frame: dq = now * conj(old).
		const vr::HmdQuaternion_t a = now.rot;
		const vr::HmdQuaternion_t b{ old.rot.w, -old.rot.x, -old.rot.y, -old.rot.z };
		vr::HmdQuaternion_t dq = QuatMul( a, b );
		if ( dq.w < 0 )
			dq = { -dq.w, -dq.x, -dq.y, -dq.z };
		const double s = std::sqrt( dq.x * dq.x + dq.y * dq.y + dq.z * dq.z );
		const double angle = 2.0 * std::atan2( s, dq.w );
		const double k = s > 1e-9 ? angle / ( s * dt ) : 2.0 / dt;
		const double w[3] = { dq.x * k, dq.y * k, dq.z * k };
		for ( int i = 0; i < 3; i++ )
		{
			const double v = ( now.pos[i] - old.pos[i] ) / dt;
			vel_[i] += kSmooth * ( v - vel_[i] );
			ang_vel_[i] += kSmooth * ( w[i] - ang_vel_[i] );
		}
	}
	else if ( dt >= kMaxAge )
	{
		// A gap in the stream: start over rather than extrapolate stale motion.
		history_.clear();
		history_.push_back( now );
		for ( int i = 0; i < 3; i++ )
			vel_[i] = ang_vel_[i] = 0.0;
	}
	for ( int i = 0; i < 3; i++ )
	{
		out.vecVelocity[i] = vel_[i];
		out.vecAngularVelocity[i] = ang_vel_[i];
	}
}

bool PoseSource::ReadPose( vr::DriverPose_t &out )
{
	struct psvr2_pose sample{};
	if ( !dev_ || psvr2_read_pose( dev_, &sample, /*block=*/1 ) != 1 )
		return false;

	out = vr::DriverPose_t{};

	// Identity offsets: the device already reports head pose in world space.
	out.qWorldFromDriverRotation.w = 1.0;
	out.qDriverFromHeadRotation.w = 1.0;

	RemapPose( sample, out );

	// Device clock (wraps every ~71 min) for velocity, host clock for age.
	if ( history_.empty() )
		last_vts_ = sample.device_vts_us;
	device_time_ += double( uint32_t( sample.device_vts_us - last_vts_ ) ) * 1e-6;
	last_vts_ = sample.device_vts_us;
	EstimateVelocity( { device_time_, { out.vecPosition[0], out.vecPosition[1], out.vecPosition[2] }, out.qRotation },
	                  out );
	timespec ts{};
	clock_gettime( CLOCK_MONOTONIC, &ts );
	const double age = ( double( ts.tv_sec ) * 1e9 + double( ts.tv_nsec ) - double( sample.timestamp_ns ) ) * 1e-9;
	// The sample is this old already; SteamVR predicts forward from here.
	out.poseTimeOffset = -std::clamp( age, 0.0, 0.05 );

	out.poseIsValid = sample.valid != 0;
	out.deviceIsConnected = true;
	out.result = sample.valid ? vr::TrackingResult_Running_OK
	                          : vr::TrackingResult_Running_OutOfRange;
	out.shouldApplyHeadModel = false; // we provide a real 6DoF pose

	return true;
}
