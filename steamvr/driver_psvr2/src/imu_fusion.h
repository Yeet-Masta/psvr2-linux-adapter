// SPDX-License-Identifier: GPL-2.0
// 3DoF orientation from gyro + accelerometer (Mahony complementary filter).
//
// The filter frame has +Y opposite to gravity; yaw is relative to the
// orientation at the first sample and drifts slowly, so callers re-anchor yaw. Gyro bias is learned whenever the
// sensor is still
// (see controller_device.cpp).
#pragma once

#include <cmath>

#include "vr_math.h"

class ImuFusion
{
public:
	// accel in m/s^2, gyro in rad/s, both in the sensor frame; dt in seconds.
	void Update( const vrmath::Vec3 &accel, const vrmath::Vec3 &gyro, double dt )
	{
		if ( dt <= 0.0 || dt > 0.1 )
			dt = 0.0;

		const double an = std::sqrt( accel.x * accel.x + accel.y * accel.y + accel.z * accel.z );

		// Learn the gyro bias while the controller is held still: the rate is
		// small and the accelerometer reads only gravity.
		const vrmath::Vec3 db{ gyro.x - bias_.x, gyro.y - bias_.y, gyro.z - bias_.z };
		if ( std::fabs( an - 9.81 ) < 0.5 && db.x * db.x + db.y * db.y + db.z * db.z < kStillRate * kStillRate )
		{
			const double k = bias_samples_ < kSettledSamples ? 1.0 / ++bias_samples_ : kBiasRate;
			bias_ = vrmath::Add( bias_, vrmath::Scale( db, k ) );
		}
		vrmath::Vec3 w{ gyro.x - bias_.x, gyro.y - bias_.y, gyro.z - bias_.z };

		if ( !initialized_ )
		{
			if ( an < 1.0 )
				return;
			// Tilt the sensor so its measured "up" maps to world +Y.
			q_ = FromTo( { accel.x / an, accel.y / an, accel.z / an }, { 0.0, 1.0, 0.0 } );
			initialized_ = true;
			return;
		}

		// Trust gravity only while the controller is not accelerating hard.
		if ( std::fabs( an - 9.81 ) < 2.0 )
		{
			const vrmath::Vec3 up_s = vrmath::Rotate( vrmath::Conj( q_ ), { 0.0, 1.0, 0.0 } );
			const vrmath::Vec3 a{ accel.x / an, accel.y / an, accel.z / an };
			// Error = measured x expected, drives the gyro towards gravity.
			const vrmath::Vec3 e{ a.y * up_s.z - a.z * up_s.y, a.z * up_s.x - a.x * up_s.z,
			                      a.x * up_s.y - a.y * up_s.x };
			w = vrmath::Add( w, vrmath::Scale( e, kGain ) );
		}

		const vr::HmdQuaternion_t dq{ 0.0, w.x * 0.5 * dt, w.y * 0.5 * dt, w.z * 0.5 * dt };
		const vr::HmdQuaternion_t d = vrmath::Mul( q_, dq );
		q_ = vrmath::Normalize( { q_.w + d.w, q_.x + d.x, q_.y + d.y, q_.z + d.z } );
	}

	bool Initialized() const { return initialized_; }
	// Gyro bias, in rad/s: restore a saved estimate so drift stays low from
	// the first second; Settled() once enough still samples went into it.
	const vrmath::Vec3 &Bias() const { return bias_; }
	void SetBias( const vrmath::Vec3 &b )
	{
		bias_ = b;
		bias_samples_ = kSettledSamples;
	}
	bool Settled() const { return bias_samples_ >= kSettledSamples; }
	vrmath::Vec3 Unbiased( const vrmath::Vec3 &gyro ) const { return { gyro.x - bias_.x, gyro.y - bias_.y, gyro.z - bias_.z }; }
	const vr::HmdQuaternion_t &Orientation() const { return q_; }

private:
	static vr::HmdQuaternion_t FromTo( const vrmath::Vec3 &a, const vrmath::Vec3 &b )
	{
		const vrmath::Vec3 c{ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
		const double d = a.x * b.x + a.y * b.y + a.z * b.z;
		if ( d < -0.9999 )
			return { 0.0, 1.0, 0.0, 0.0 };
		return vrmath::Normalize( { 1.0 + d, c.x, c.y, c.z } );
	}

	static constexpr double kGain = 1.0;         // rad/s per unit tilt error
	static constexpr double kStillRate = 0.08;   // rad/s, below this counts as still
	static constexpr double kBiasRate = 0.002;   // per-sample bias update once settled
	static constexpr int kSettledSamples = 200;

	vrmath::Vec3 bias_;
	int bias_samples_ = 0;

	vr::HmdQuaternion_t q_ = vrmath::Identity();
	bool initialized_ = false;
};
