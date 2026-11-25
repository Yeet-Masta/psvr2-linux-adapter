// SPDX-License-Identifier: GPL-2.0
// Controller pose filter: the controller IMU fused with the optical pose
// from the LED solver. Behaviour follows Sony's UkfManager
// (docs/sony-re/controller-tracking.md part G); the implementation is our own.
//
// Taken from Sony (docs/sony-re/controller-tracking.md part G, docs/sony-re/controller-tracking.md §A.5.5, docs/sony-re/controller-tracking.md §C.4):
//   - 15-dim error state: position, velocity, attitude, gyro bias, accel bias;
//   - symmetric sigma points (2n, weights 1/2n, spread sqrt(n): the
//     unscented transform with alpha 1, kappa 0) for the optical update;
//   - one covariance step per measurement, over the whole interval since the
//     last one, with the time-weighted mean IMU sample and Sony's
//     rate-dependent IMU noise;
//   - measurement = position + the tips of a 10 cm body frame (12 scalars),
//     or position only; noise from the head-relative ellipsoid;
//   - innovation gate 5 cm below 1 m/s, else 20 cm;
//   - P0, stationary velocity damping, divergence limits;
//   - delayed measurements applied at their own time, then the IMU replayed.
// Own choices (not Sony's): the error-state transition is linearised (Sony
// propagates the sigma points), the attitude error is body-frame, a floor
// on the rotation noise (Sony's is 0), bias random-walk and bias P0 values,
// and the 10-rejects reinitialisation.
#pragma once

#include <cstdint>
#include <deque>

#include "vr_math.h"

class ControllerUkf
{
public:
	static constexpr int N = 15;

	struct Fix
	{
		uint64_t us = 0;           // frame time, host steady clock
		vrmath::Vec3 pos;          // world position of the IMU, m
		vr::HmdQuaternion_t rot;   // world <- IMU
		bool has_rot = true;       // false: position only
		vrmath::Vec3 head_rel;     // IMU position in the head frame, m (noise zone)
		double cam_dist = 0.0;     // controller distance from the camera rig, m
	};

	enum Result
	{
		kApplied = 1,
		kInitialized = 2,
		kGated = 0,
		kRejected = -1,  // too old, failed or reset
	};

	// The gyro bias to start from (the complementary filter's estimate).
	void SetGyroBiasHint( const vrmath::Vec3 &b ) { bg_hint_ = b; }
	// Floors on the optical noise sigmas (m). Sony: position none, rotation 0.
	void SetRotSigmaFloor( double m ) { rot_sigma_floor_ = m; }
	void SetPosSigmaFloor( double m ) { pos_sigma_floor_ = m; }
	// Gyro clip range (rad/s) for the process noise, set per hardware rev
	// like Sony's setImuPose (docs/sony-re/controller-tracking.md §C.4.3).
	void SetGyroRange( double r ) { gyro_range_ = r; }
	static double GyroRangeForRev( int rev );

	void Reset();

	// One IMU sample in the IMU frame: rad/s, m/s^2 (specific force, +y
	// up at rest). us: host steady clock.
	void AddImu( uint64_t us, const vrmath::Vec3 &gyro, const vrmath::Vec3 &accel );
	Result AddFix( const Fix &f );

	bool Initialized() const { return init_; }
	uint64_t TimeUs() const { return cur_.us; }
	uint64_t LastFixUs() const { return last_fix_us_; }
	const vrmath::Vec3 &Position() const { return cur_.p; }
	const vrmath::Vec3 &Velocity() const { return cur_.v; }
	const vr::HmdQuaternion_t &Orientation() const { return cur_.q; }  // world <- IMU
	const vrmath::Vec3 &GyroBias() const { return cur_.bg; }
	const vrmath::Vec3 &AccelBias() const { return cur_.ba; }
	// Position at the last applied fix (Sony holds it once fixes are 0.3 s old).
	const vrmath::Vec3 &FixPosition() const { return fix_pos_; }
	int Rejects() const { return rejects_; }

private:
	struct State
	{
		uint64_t us = 0;
		vrmath::Vec3 p, v, bg, ba;
		vr::HmdQuaternion_t q = vrmath::Identity();
		vrmath::Vec3 a_prev;  // world linear accel of the previous step (stationary test)
	};
	struct Sample
	{
		uint64_t us;
		vrmath::Vec3 gyro, accel;
		State after;  // nominal state after this sample (valid once init_)
	};

	static void Propagate( State &s, const vrmath::Vec3 &gyro, const vrmath::Vec3 &accel, double dt );
	void Init( const Fix &f, const State &at );
	bool CovarianceStep( uint64_t to_us, const State &start );
	bool Update( State &s, const Fix &f );
	void Guard( State &s );
	void ResetBlock( int b, double var );
	void SetAttitudeP0( const vr::HmdQuaternion_t &q );

	std::deque<Sample> ring_;
	State cur_;
	bool init_ = false;
	double P_[N][N] = {};
	uint64_t p_us_ = 0;  // time the covariance refers to
	uint64_t last_fix_us_ = 0;
	vrmath::Vec3 fix_pos_;
	int rejects_ = 0;
	vrmath::Vec3 bg_hint_;
	double rot_sigma_floor_ = 0.001;
	double pos_sigma_floor_ = 0.0;
	double gyro_range_ = 43.6332;  // Sony's config default (2500 deg/s)
};
