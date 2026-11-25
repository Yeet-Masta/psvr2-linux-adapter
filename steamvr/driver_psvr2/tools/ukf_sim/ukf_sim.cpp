// SPDX-License-Identifier: GPL-2.0
// Offline check of ControllerUkf on a simulated controller: 1 kHz IMU with
// noise and bias, 60 Hz optical fixes with noise, 25 ms latency, outliers and
// a 1 s dropout. Prints error statistics; exits non-zero if they are off.
// UKF_ARM=1: during the dropout, arm-model pseudo fixes (truth + a 4 cm
// model error + 1 cm noise) as the driver feeds them after 0.3 s.
//
//   ./build.sh && ./ukf_sim && UKF_ARM=1 ./ukf_sim
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <random>

#include "controller_ukf.h"

using vrmath::Vec3;
using Quat = vr::HmdQuaternion_t;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.80665;

Vec3 TruePos( double t )
{
	return { 0.2 * std::sin( 2 * kPi * 0.5 * t ), 1.0 + 0.1 * std::sin( 2 * kPi * 0.7 * t ),
	         -0.4 + 0.15 * std::cos( 2 * kPi * 0.4 * t ) };
}
Quat TrueRot( double t )
{
	const Quat yaw = vrmath::AxisAngle( 0, 1, 0, 0.8 * std::sin( 2 * kPi * 0.3 * t ) + 0.5 );
	const Quat pitch = vrmath::AxisAngle( 1, 0, 0, 0.5 * std::sin( 2 * kPi * 0.45 * t ) );
	const Quat roll = vrmath::AxisAngle( 0, 0, 1, 0.3 * std::sin( 2 * kPi * 0.6 * t ) );
	return vrmath::Normalize( vrmath::Mul( vrmath::Mul( yaw, pitch ), roll ) );
}
Vec3 Sub( const Vec3 &a, const Vec3 &b ) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
double Norm( const Vec3 &a ) { return std::sqrt( a.x * a.x + a.y * a.y + a.z * a.z ); }
// Rotation vector of a unit quaternion.
Vec3 Log( Quat q )
{
	if ( q.w < 0 )
		q = { -q.w, -q.x, -q.y, -q.z };
	const double s = std::sqrt( q.x * q.x + q.y * q.y + q.z * q.z );
	const double a = 2.0 * std::atan2( s, q.w );
	const double k = s < 1e-12 ? 2.0 : a / s;
	return { q.x * k, q.y * k, q.z * k };
}
Quat Exp( const Vec3 &w )
{
	const double a = Norm( w );
	const double k = a < 1e-12 ? 0.5 : std::sin( 0.5 * a ) / a;
	return { std::cos( 0.5 * a ), w.x * k, w.y * k, w.z * k };
}

struct Pending
{
	double deliver;
	ControllerUkf::Fix fix;
};
}  // namespace

int main()
{
	std::mt19937 rng( 1234 );
	std::normal_distribution<double> n01( 0.0, 1.0 );
	std::uniform_real_distribution<double> u01( 0.0, 1.0 );
	auto noise = [&]( double s ) { return Vec3{ s * n01( rng ), s * n01( rng ), s * n01( rng ) }; };

	const Vec3 bg{ 0.02, -0.01, 0.015 }, ba{ 0.10, -0.05, 0.08 };
	const double gyro_sigma = 0.01, accel_sigma = 0.05;
	const double pos_sigma = 0.001, rot_sigma = 0.5 * kPi / 180.0;
	const double imu_dt = 0.001, fix_dt = 1.0 / 60.0, latency = 0.025, T = 20.0;
	const double h = 1e-4;

	ControllerUkf ukf;
	if ( const char *e = std::getenv( "UKF_POS_FLOOR_MM" ) )
		ukf.SetPosSigmaFloor( std::atof( e ) * 1e-3 );
	if ( const char *e = std::getenv( "UKF_ROT_FLOOR_MM" ) )
		ukf.SetRotSigmaFloor( std::atof( e ) * 1e-3 );
	const bool arm = std::getenv( "UKF_ARM" ) && std::atoi( std::getenv( "UKF_ARM" ) ) != 0;
	double next_arm = 0.0, drop_end_err = 0.0;
	int arm_applied = 0;
	std::deque<Pending> pending;
	double next_fix = 0.05;
	int applied = 0, gated = 0, inits = 0, outliers = 0;
	double se_pos = 0, se_rot = 0, max_pos = 0, se_hold = 0;
	int n_err = 0;
	double drop_max = 0;
	Vec3 last_fix_pos;
	bool have_fix = false;

	for ( double t = 0.0; t < T; t += imu_dt )
	{
		const uint64_t us = uint64_t( std::llround( t * 1e6 ) );
		// IMU sample (holds over the previous interval: use the mid time).
		const double tm = t - 0.5 * imu_dt;
		const Quat q0 = TrueRot( tm - 0.5 * h ), q1 = TrueRot( tm + 0.5 * h );
		const Vec3 w = vrmath::Scale( Log( vrmath::Mul( vrmath::Conj( q0 ), q1 ) ), 1.0 / h );
		const Vec3 pa = TruePos( tm - h ), pb = TruePos( tm ), pc = TruePos( tm + h );
		Vec3 acc_w{ ( pa.x - 2 * pb.x + pc.x ) / ( h * h ), ( pa.y - 2 * pb.y + pc.y ) / ( h * h ),
		            ( pa.z - 2 * pb.z + pc.z ) / ( h * h ) };
		acc_w.y += kG;
		const Vec3 acc_b = vrmath::Rotate( vrmath::Conj( TrueRot( tm ) ), acc_w );
		const Vec3 gyro = vrmath::Add( vrmath::Add( w, bg ), noise( gyro_sigma ) );
		const Vec3 accel = vrmath::Add( vrmath::Add( acc_b, ba ), noise( accel_sigma ) );
		ukf.AddImu( us, gyro, accel );

		// Optical fixes: frame time t, delivered `latency` later; none in 10-11 s.
		if ( t >= next_fix )
		{
			next_fix += fix_dt;
			if ( !( t >= 10.0 && t < 11.0 ) )
			{
				ControllerUkf::Fix f;
				f.us = us;
				f.pos = vrmath::Add( TruePos( t ), noise( pos_sigma ) );
				if ( u01( rng ) < 0.05 )
				{
					f.pos = vrmath::Add( f.pos, { 0.10, 0.0, 0.0 } );
					outliers++;
				}
				f.rot = vrmath::Normalize( vrmath::Mul( TrueRot( t ), Exp( noise( rot_sigma ) ) ) );
				f.head_rel = { 0.15, -0.3, -0.4 };
				f.cam_dist = 0.5;
				pending.push_back( { t + latency, f } );
			}
		}
		if ( arm && ukf.Initialized() && t >= next_arm && us > ukf.LastFixUs() + 300000 )
		{
			next_arm = t + fix_dt;
			ControllerUkf::Fix f;
			f.us = us;
			f.pos = vrmath::Add( vrmath::Add( TruePos( t ), { 0.04, 0.0, 0.0 } ), noise( 0.01 ) );
			f.has_rot = false;
			f.pseudo = true;
			f.sigma = Norm( Sub( f.pos, ukf.Position() ) ) < 0.1 ? 0.005 : 0.05;
			if ( ukf.AddFix( f ) == ControllerUkf::kApplied )
				arm_applied++;
		}
		if ( ukf.Initialized() && t >= 11.0 && t < 11.0 + imu_dt )
			drop_end_err = Norm( Sub( ukf.Position(), TruePos( t ) ) );
		while ( !pending.empty() && pending.front().deliver <= t )
		{
			const auto r = ukf.AddFix( pending.front().fix );
			if ( r == ControllerUkf::kApplied )
				applied++;
			else if ( r == ControllerUkf::kGated )
				gated++;
			else if ( r == ControllerUkf::kInitialized )
				inits++;
			last_fix_pos = pending.front().fix.pos;
			have_fix = true;
			pending.pop_front();
		}

		if ( ukf.Initialized() && t > 2.0 )
		{
			const Vec3 e = Sub( ukf.Position(), TruePos( t ) );
			const double ang = Norm( Log( vrmath::Mul( vrmath::Conj( TrueRot( t ) ), ukf.Orientation() ) ) );
			if ( t >= 10.0 && t < 11.1 )
			{
				// What the driver reports: the filter while fixes are < 0.3 s
				// old, then the position of the last fix, held.
				const bool fresh = us < ukf.LastFixUs() + 300000;
				const Vec3 out = fresh ? ukf.Position() : ukf.FixPosition();
				drop_max = std::max( drop_max, Norm( Sub( out, TruePos( t ) ) ) );
			}
			else
			{
				se_pos += Norm( e ) * Norm( e );
				se_rot += ang * ang;
				max_pos = std::max( max_pos, Norm( e ) );
				if ( have_fix )
				{
					const double eh = Norm( Sub( last_fix_pos, TruePos( t ) ) );
					se_hold += eh * eh;
				}
				n_err++;
			}
		}
	}
	const double rms_pos = std::sqrt( se_pos / n_err ) * 1000.0, rms_rot = std::sqrt( se_rot / n_err ) * 180.0 / kPi;
	const double rms_hold = std::sqrt( se_hold / n_err ) * 1000.0;
	const Vec3 gb = ukf.GyroBias();
	std::printf( "fixes: applied %d, gated %d (outliers %d), inits %d\n", applied, gated, outliers, inits );
	std::printf( "pos rms %.2f mm (max %.2f), rot rms %.3f deg; latest-fix hold rms %.2f mm\n", rms_pos,
	             max_pos * 1000.0, rms_rot, rms_hold );
	std::printf( "1 s dropout: max pos error %.1f mm; filter error at its end %.1f mm%s\n", drop_max * 1000.0,
	             drop_end_err * 1000.0, arm ? " (arm model)" : "" );
	if ( arm )
		std::printf( "arm-model pseudo fixes applied %d\n", arm_applied );
	const Vec3 ab = ukf.AccelBias();
	std::printf( "accel bias est (%.3f %.3f %.3f), true (%.3f %.3f %.3f)\n", ab.x, ab.y, ab.z, ba.x, ba.y, ba.z );
	std::printf( "gyro bias est (%.4f %.4f %.4f), true (%.4f %.4f %.4f)\n", gb.x, gb.y, gb.z, bg.x, bg.y, bg.z );

	// One init, plus one restart after the 1 s dropout. The dropout error is
	// the reported output (held after 0.3 s, as Sony), so it only has to stay
	// bounded by the motion over the gap.
	const bool bias_ok = Norm( Sub( gb, bg ) ) < 0.2 * Norm( bg ) && Norm( Sub( ab, ba ) ) < 0.2 * Norm( ba );
	bool ok = inits <= 2 && rms_pos < 2.0 && rms_rot < 0.3 && gated >= outliers && gated <= outliers + 2 && bias_ok &&
	          drop_max < 0.35 && ( !arm || ( drop_end_err < 0.08 && arm_applied > 30 ) );
	std::printf( "%s\n", ok ? "PASS" : "FAIL" );
	return ok ? 0 : 1;
}
