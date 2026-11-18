// SPDX-License-Identifier: GPL-2.0
#include "optical_pose.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>

#include "controller_device.h"
#include "driverlog.h"
#include "hmd_device_driver.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
const char kSection[] = "driver_psvr2";

uint64_t NowMicros()
{
	return uint64_t( std::chrono::duration_cast<std::chrono::microseconds>(
				 std::chrono::steady_clock::now().time_since_epoch() )
	                     .count() );
}

// Sony's built-in rig, variant 2 (opticalpose.md §2.1): rig <- camera,
// rotation given by columns, translation in mm.
struct RigEntry
{
	double c0[3], c1[3], c2[3], t[3];
};
const RigEntry kRigVariant2[4] = {
	{ { 0.96593, 0.14845, -0.21201 }, { 0, 0.81915, 0.57358 }, { 0.25882, -0.55403, 0.79124 }, { 40.0, -39.0685, 79.3971 } },
	{ { 0.96593, -0.14845, 0.21201 }, { 0, 0.81915, 0.57358 }, { -0.25882, -0.55403, 0.79124 }, { -40.0, -39.0685, 79.3971 } },
	{ { 0.70941, 0.49374, -0.50295 }, { -0.49673, 0.85651, 0.14019 }, { 0.5, 0.15038, 0.85287 }, { 67.5, 32.1406, 80.106 } },
	{ { 0.70941, -0.49374, 0.50295 }, { 0.49673, 0.85651, 0.14019 }, { -0.5, 0.15038, 0.85287 }, { -67.5, 32.1406, 80.106 } },
};

// IMU position in the LED-model frame, mm (pad +0x48, opticalpose.md §2b.1).
const double kImuInModel[2][3] = {
	{ 9.3727, -12.2481, -6.0039 },   // left
	{ -20.0727, -12.2481, -6.0039 }, // right
};

// 4x4 rigid transforms as R (row-major 3x3) and t.
struct Rt
{
	double R[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
	double t[3] = { 0, 0, 0 };
};

Rt Mul( const Rt &a, const Rt &b )
{
	Rt o;
	for ( int i = 0; i < 3; i++ )
	{
		for ( int j = 0; j < 3; j++ )
			o.R[3 * i + j] = a.R[3 * i] * b.R[j] + a.R[3 * i + 1] * b.R[3 + j] + a.R[3 * i + 2] * b.R[6 + j];
		o.t[i] = a.R[3 * i] * b.t[0] + a.R[3 * i + 1] * b.t[1] + a.R[3 * i + 2] * b.t[2] + a.t[i];
	}
	return o;
}

Rt Inv( const Rt &a )
{
	Rt o;
	for ( int i = 0; i < 3; i++ )
		for ( int j = 0; j < 3; j++ )
			o.R[3 * i + j] = a.R[3 * j + i];
	for ( int i = 0; i < 3; i++ )
		o.t[i] = -( o.R[3 * i] * a.t[0] + o.R[3 * i + 1] * a.t[1] + o.R[3 * i + 2] * a.t[2] );
	return o;
}

Rt FromRig( const RigEntry &e )
{
	Rt o;
	for ( int i = 0; i < 3; i++ )
	{
		o.R[3 * i] = e.c0[i];
		o.R[3 * i + 1] = e.c1[i];
		o.R[3 * i + 2] = e.c2[i];
		o.t[i] = e.t[i];
	}
	return o;
}

double AngleBetween( const Rt &a, const Rt &b )
{
	double tr = 0;
	for ( int i = 0; i < 3; i++ )
		for ( int k = 0; k < 3; k++ )
			tr += a.R[3 * k + i] * b.R[3 * k + i];
	return std::acos( std::clamp( ( tr - 1 ) / 2, -1.0, 1.0 ) ) / kDeg;
}

// The 24 proper rotations that map axes onto axes; [0] is the identity.
struct Remaps
{
	double R[24][9];
	Remaps()
	{
		static const int perm[6][3] = { { 0, 1, 2 }, { 0, 2, 1 }, { 1, 0, 2 }, { 1, 2, 0 }, { 2, 0, 1 }, { 2, 1, 0 } };
		int n = 0;
		for ( const auto &p : perm )
			for ( int s = 0; s < 8; s++ )
			{
				double M[9] = {};
				for ( int i = 0; i < 3; i++ )
					M[3 * i + p[i]] = ( s >> i ) & 1 ? -1.0 : 1.0;
				const double det = M[0] * ( M[4] * M[8] - M[5] * M[7] ) - M[1] * ( M[3] * M[8] - M[5] * M[6] ) +
				                   M[2] * ( M[3] * M[7] - M[4] * M[6] );
				if ( det > 0 )
					std::memcpy( R[n++], M, sizeof( M ) );
			}
	}
};
const Remaps kRemap;

void MatMul( const double A[9], const double B[9], double C[9] )
{
	for ( int i = 0; i < 3; i++ )
		for ( int j = 0; j < 3; j++ )
			C[3 * i + j] = A[3 * i] * B[j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j];
}

void MatT( const double A[9], double T[9] )
{
	for ( int i = 0; i < 3; i++ )
		for ( int j = 0; j < 3; j++ )
			T[3 * i + j] = A[3 * j + i];
}

// Pose (quaternion, m) <-> Rt (mm).
Rt ToRt( const vrmath::Pose &p )
{
	Rt o;
	vrmath::ToMatrix( p.rot, o.R );
	o.t[0] = p.pos.x * 1000;
	o.t[1] = p.pos.y * 1000;
	o.t[2] = p.pos.z * 1000;
	return o;
}

vrmath::Pose ToPose( const Rt &a )
{
	return { vrmath::FromMatrix( a.R ), { a.t[0] * 1e-3, a.t[1] * 1e-3, a.t[2] * 1e-3 } };
}

// Angle of the world "up" after rotation c, degrees: 0 = heading-only.
double TiltDeg( const vr::HmdQuaternion_t &c )
{
	const vrmath::Vec3 up = vrmath::Rotate( c, { 0.0, 1.0, 0.0 } );
	return std::acos( std::clamp( up.y, -1.0, 1.0 ) ) / kDeg;
}

int GetInt( const char *key, int def )
{
	vr::EVRSettingsError e = vr::VRSettingsError_None;
	const int v = vr::VRSettings()->GetInt32( kSection, key, &e );
	return e == vr::VRSettingsError_None ? v : def;
}

float GetFloat( const char *key, float def )
{
	vr::EVRSettingsError e = vr::VRSettingsError_None;
	const float v = vr::VRSettings()->GetFloat( kSection, key, &e );
	return e == vr::VRSettingsError_None ? v : def;
}

const char *HandName( int hand ) { return hand == 0 ? "left" : "right"; }
}  // namespace

OpticalPose::OpticalPose( Psvr2HmdDriver *hmd, Psvr2ControllerDriver *const controllers[2] ) : hmd_( hmd )
{
	controllers_[0] = controllers[0];
	controllers_[1] = controllers[1];
	psvr2_solver_default_cfg( &cfg_ );
	rig_offset_ = { GetFloat( "optical_rig_offset_x", 0.0f ), GetFloat( "optical_rig_offset_y", 0.0f ),
	                GetFloat( "optical_rig_offset_z", 0.0f ) };
	rig_tilt_deg_ = GetFloat( "optical_rig_tilt_deg", 0.0f );
	for ( int i = 0; i < 2; i++ )
		LoadHand( i );

	vr::EVRSettingsError e = vr::VRSettingsError_None;
	if ( vr::VRSettings()->GetBool( kSection, "optical_pose_log", &e ) && e == vr::VRSettingsError_None )
	{
		const char *home = std::getenv( "HOME" );
		const std::string path = std::string( home ? home : "/tmp" ) + "/psvr2-optical-pose.txt";
		pose_log_ = std::fopen( path.c_str(), "w" );
		if ( pose_log_ )
			std::fprintf( pose_log_, "# us hand matched rms_px | rig<-model t(mm) q(wxyz) | world<-head pos q | "
			                         "fused imu q | yaw\n" );
		DriverLog( "psvr2: optical pose: log %s %s", path.c_str(), pose_log_ ? "open" : "failed" );
	}
	DriverLog( "psvr2: optical pose: head <- rig offset (%.3f %.3f %.3f) m, tilt %.1f deg", rig_offset_.x,
	           rig_offset_.y, rig_offset_.z, rig_tilt_deg_ );
}

OpticalPose::~OpticalPose()
{
	if ( pose_log_ )
		std::fclose( pose_log_ );
}

void OpticalPose::LoadHand( int hand )
{
	Hand &h = hands_[hand];
	h.remap = GetInt( hand == 0 ? "optical_remap_left" : "optical_remap_right", -1 );
	if ( h.remap < -1 || h.remap >= 24 )
		h.remap = -1;
	// Sony mirrors the left model in x (opticalpose.md §2b.2); whether this
	// model is already mirrored is open, so the left hand learns it too.
	h.mirror = hand == 0 ? GetInt( "optical_mirror_left", -1 ) : 0;
	if ( h.remap < 0 )
		h.mirror = hand == 0 ? -1 : 0;
}

vrmath::Pose OpticalPose::HeadFromRig() const
{
	// Rig: +x left, +y up, +z forward. OpenVR head: +x right, +y up, +z back.
	const vr::HmdQuaternion_t flip = vrmath::AxisAngle( 0, 1, 0, kPi );
	const vr::HmdQuaternion_t tilt = vrmath::AxisAngle( 1, 0, 0, rig_tilt_deg_ * kDeg );
	return { vrmath::Mul( tilt, flip ), rig_offset_ };
}

bool OpticalPose::BuildRig( psvr2_t *dev )
{
	if ( have_rig_ )
		return true;
	static uint8_t blob[0x800];
	const int n = psvr2_read_calibration( dev, "camera_calib0", blob, sizeof( blob ) );
	psvr2_cam_calib cal;
	if ( n <= 0 || psvr2_cam_calib_parse( blob, unsigned( n ), &cal ) != 0 || cal.ncam < PSVR2_SOLVER_CAMS )
		return false;

	Rt rig[PSVR2_SOLVER_CAMS];
	bool known[PSVR2_SOLVER_CAMS] = { true, false, false, false };
	rig[0] = FromRig( kRigVariant2[0] );
	// Extrinsic "a -> b" is b <- a.
	for ( int pass = 0; pass < 3; pass++ )
		for ( unsigned e = 0; e < cal.next; e++ )
		{
			const unsigned a = cal.ext[e].from, b = cal.ext[e].to;
			if ( a >= PSVR2_SOLVER_CAMS || b >= PSVR2_SOLVER_CAMS )
				continue;
			Rt E;
			psvr2_cam_extrinsic_to_rig( &cal.ext[e], E.R, E.t );
			if ( known[a] && !known[b] )
			{
				rig[b] = Mul( rig[a], Inv( E ) );
				known[b] = true;
			}
			else if ( known[b] && !known[a] )
			{
				rig[a] = Mul( rig[b], E );
				known[a] = true;
			}
		}

	for ( int c = 0; c < PSVR2_SOLVER_CAMS; c++ )
	{
		const Rt table = FromRig( kRigVariant2[c] );
		if ( !known[c] )
			rig[c] = table;
		psvr2_rig_camera &cam = cams_[c];
		if ( psvr2_cam_model_from_calib( &cal, unsigned( c ), &cam.model ) != 0 )
			return false;
		std::memcpy( cam.R, rig[c].R, sizeof( cam.R ) );
		std::memcpy( cam.t, rig[c].t, sizeof( cam.t ) );
		cam.active = 1;
		const double d = std::sqrt( ( rig[c].t[0] - table.t[0] ) * ( rig[c].t[0] - table.t[0] ) +
		                            ( rig[c].t[1] - table.t[1] ) * ( rig[c].t[1] - table.t[1] ) +
		                            ( rig[c].t[2] - table.t[2] ) * ( rig[c].t[2] - table.t[2] ) );
		DriverLog( "psvr2: optical pose: cam %d at (%.1f %.1f %.1f) mm%s, %.1f mm / %.1f deg from Sony's table", c,
		           rig[c].t[0], rig[c].t[1], rig[c].t[2], known[c] ? "" : " (table)", d, AngleBetween( rig[c], table ) );
	}
	have_rig_ = true;
	return true;
}

void OpticalPose::Frame( uint64_t frame_us, const psvr2_blob_set blobs[PSVR2_LD_CAMERAS],
                         const int counts[PSVR2_LD_CAMERAS], const bool synced[2] )
{
	if ( !have_rig_ || !hmd_ || !( synced[0] || synced[1] ) )
		return;
	vr::DriverPose_t head;
	if ( !hmd_->PoseAt( frame_us, head ) )
		return;
	const vrmath::Pose world_from_head{ head.qRotation,
		                                vrmath::Add( { head.vecPosition[0], head.vecPosition[1], head.vecPosition[2] },
		                                             vrmath::Rotate( head.qRotation,
		                                                             { head.vecDriverFromHeadTranslation[0],
		                                                               head.vecDriverFromHeadTranslation[1],
		                                                               head.vecDriverFromHeadTranslation[2] } ) ) };
	const vrmath::Pose world_from_rig = vrmath::Compose( world_from_head, HeadFromRig() );

	// Blob lists: drop degenerate/empty and large blobs (room lights; an LED
	// is a few pixels across), keep the smallest PSVR2_SOLVER_BLOBS.
	psvr2_blob_list lists[PSVR2_SOLVER_CAMS];
	for ( int c = 0; c < PSVR2_SOLVER_CAMS; c++ )
	{
		int idx[PSVR2_LD_MAX_BLOBS], n = 0;
		for ( int i = 0; i < counts[c] && i < PSVR2_LD_MAX_BLOBS; i++ )
			if ( !( blobs[c].flags[i] & 0x300 ) && blobs[c].w[i] <= 24 && blobs[c].h[i] <= 24 )
				idx[n++] = i;
		std::sort( idx, idx + n, [&]( int a, int b ) {
			return int( blobs[c].w[a] ) * blobs[c].h[a] < int( blobs[c].w[b] ) * blobs[c].h[b];
		} );
		lists[c].count = std::min( n, PSVR2_SOLVER_BLOBS );
		for ( int k = 0; k < lists[c].count; k++ )
		{
			lists[c].xy[k][0] = blobs[c].x[idx[k]];
			lists[c].xy[k][1] = blobs[c].y[idx[k]];
		}
	}

	for ( int hand = 0; hand < 2; hand++ )
	{
		Hand &h = hands_[hand];
		if ( !synced[hand] )
		{
			h.tracking = false;
			continue;
		}
		h.frames++;
		SolveHand( hand, frame_us, world_from_rig, lists );
	}

	const uint64_t now = NowMicros();
	if ( now - log_at_us_ >= 2000000 )
	{
		log_at_us_ = now;
		for ( int hand = 0; hand < 2; hand++ )
		{
			Hand &h = hands_[hand];
			if ( !h.frames )
				continue;
			DriverLog( "psvr2: optical pose: %s: %s, solved %d/%d frames, %d boots tried, %d fixes, matched %.1f, "
			           "rms %.2f px, remap %d mirror %d",
			           HandName( hand ), h.tracking ? "tracking" : "searching", h.solved, h.frames, h.boots, h.fixes,
			           h.solved ? double( h.matched ) / h.solved : 0.0, h.solved ? h.rms / h.solved : 0.0,
			           h.remap >= 0 ? h.remap : h.cand_remap, h.mirror >= 0 ? h.mirror : h.cand_mirror );
			h.frames = h.solved = h.boots = h.fixes = h.matched = 0;
			h.rms = 0;
		}
	}
}

void OpticalPose::SolveHand( int hand, uint64_t frame_us, const vrmath::Pose &world_from_rig,
                             psvr2_blob_list lists[PSVR2_SOLVER_CAMS] )
{
	Hand &h = hands_[hand];
	Psvr2ControllerDriver *ctrl = controllers_[hand];
	vr::HmdQuaternion_t q_imu;
	double yaw_now;
	if ( !ctrl->ImuOrientation( q_imu, yaw_now ) )
		return;
	const int rev = ctrl->HardwareRevision();
	if ( rev != h.rev )
	{
		h.rev = rev;
		psvr2_led_model_load( rev, &h.model[0] );
		h.model[1] = h.model[0];
		for ( int i = 0; i < h.model[1].count; i++ )
		{
			h.model[1].pos[i][0] = -h.model[1].pos[i][0];
			h.model[1].nrm[i][0] = -h.model[1].nrm[i][0];
		}
		h.tracking = false;
		DriverLog( "psvr2: optical pose: %s: hardware rev %04x, %d LEDs", HandName( hand ), rev, h.model[0].count );
	}

	const Rt rig_from_world = Inv( ToRt( world_from_rig ) );
	psvr2_pose3 out{};
	psvr2_solve_result res{};
	bool ok = false;

	if ( h.tracking )
	{
		// Predict: last optical pose, turned by the IMU's rotation since then
		// (taken about the world's vertical through the heading in use),
		// moved by the optical velocity.
		const double dt = double( frame_us - h.last_us ) * 1e-6;
		const vr::HmdQuaternion_t d = vrmath::Mul( q_imu, vrmath::Conj( h.q_last ) );
		// Heading of the last fix relative to the fused frame.
		double P[9], Pt[9], Rm[9], Rimu[9];
		const int remap = h.remap >= 0 ? h.remap : h.cand_remap;
		std::memcpy( P, kRemap.R[remap], sizeof( P ) );
		MatT( P, Pt );
		vrmath::ToMatrix( h.last.rot, Rm );
		MatMul( Rm, Pt, Rimu );  // world <- IMU at the last fix
		const vr::HmdQuaternion_t world_imu_last = vrmath::FromMatrix( Rimu );
		const vr::HmdQuaternion_t c = vrmath::Mul( world_imu_last, vrmath::Conj( h.q_last ) );
		const vr::HmdQuaternion_t rot =
			vrmath::Mul( vrmath::Mul( vrmath::Mul( c, d ), vrmath::Conj( c ) ), h.last.rot );
		const vrmath::Pose pred{ vrmath::Normalize( rot ),
			                     vrmath::Add( h.last.pos, vrmath::Scale( h.vel, std::min( dt, 0.1 ) ) ) };
		const Rt rp = Mul( rig_from_world, ToRt( pred ) );
		psvr2_pose3 predicted;
		std::memcpy( predicted.R, rp.R, sizeof( rp.R ) );
		std::memcpy( predicted.t, rp.t, sizeof( rp.t ) );
		const int mirror = h.mirror >= 0 ? h.mirror : h.cand_mirror;
		ok = psvr2_led_track( &cfg_, cams_, lists, &h.model[mirror], &predicted, &out, &res ) == 0;
		if ( !ok && ++h.fails >= 6 )
			h.tracking = false;
	}
	if ( !h.tracking )
	{
		ok = Boot( hand, h, world_from_rig, q_imu, yaw_now, lists, out, res );
		if ( ok )
		{
			h.tracking = true;
			h.good = 0;
			h.span = 0;
			h.span_ref = q_imu;
			h.vel = {};
			h.last_us = 0;
		}
	}
	if ( !ok )
		return;
	h.fails = 0;
	Accept( hand, h, frame_us, world_from_rig, out, q_imu, res );
	if ( !h.tracking )
		return;

	// The other hand may not use the blobs this one matched.
	for ( int c = 0; c < PSVR2_SOLVER_CAMS; c++ )
	{
		bool used[PSVR2_SOLVER_BLOBS] = {};
		for ( int l = 0; l < PSVR2_LED_MAX; l++ )
			if ( res.led_blob[c][l] >= 0 && res.led_blob[c][l] < PSVR2_SOLVER_BLOBS )
				used[res.led_blob[c][l]] = true;
		int n = 0;
		for ( int k = 0; k < lists[c].count; k++ )
			if ( !used[k] )
			{
				lists[c].xy[n][0] = lists[c].xy[k][0];
				lists[c].xy[n][1] = lists[c].xy[k][1];
				n++;
			}
		lists[c].count = n;
	}
}

bool OpticalPose::Boot( int hand, Hand &h, const vrmath::Pose &world_from_rig, const vr::HmdQuaternion_t &q_imu,
                        double yaw_now, const psvr2_blob_list lists[PSVR2_SOLVER_CAMS], psvr2_pose3 &out,
                        psvr2_solve_result &res )
{
	// Hypotheses: heading (the controller's current one first when the
	// remap is known, then 12 steps of 30 deg, offset 15 deg every other
	// sweep) x remap x mirror while those are unknown.
	const bool known = h.remap >= 0;
	const int nmirror = known ? 1 : ( hand == 0 ? 2 : 1 );
	const int nremap = known ? 1 : 24;
	const int nyaw = known ? 13 : 12;
	const unsigned total = unsigned( nyaw * nremap * nmirror );

	double Rrw[9];
	const Rt rig_from_world = Inv( ToRt( world_from_rig ) );
	std::memcpy( Rrw, rig_from_world.R, sizeof( Rrw ) );
	double Rimu[9];
	vrmath::ToMatrix( q_imu, Rimu );

	const uint64_t start = NowMicros();
	int tried = 0;
	while ( tried < int( total ) && ( tried == 0 || NowMicros() - start < 3000 ) )
	{
		const unsigned k = h.hyp++;
		tried++;
		const unsigned sweep = k / total, i = k % total;
		const int y = int( i % nyaw ), r = int( ( i / nyaw ) % nremap ), m = int( i / ( nyaw * nremap ) );
		const int remap = known ? h.remap : r;
		// Sony's mirror for the left hand first.
		const int mirror = known ? h.mirror : ( hand == 0 ? 1 - m : 0 );
		double yaw;
		if ( known && y == 0 )
			yaw = yaw_now;
		else
			yaw = ( ( known ? y - 1 : y ) * 30.0 + ( sweep & 1 ? 15.0 : 0.0 ) ) * kDeg;

		double Ry[9], A[9], B[9], prior[9];
		vrmath::ToMatrix( vrmath::AxisAngle( 0, 1, 0, yaw ), Ry );
		MatMul( Ry, Rimu, A );                 // world <- IMU
		MatMul( A, kRemap.R[remap], B );       // world <- model
		MatMul( Rrw, B, prior );               // rig <- model
		h.boots++;
		// The solver accepts a boot from 4 matches; on a recorded frame those
		// were wrong poses (4-9 matches), the right one had 22-30. Own floor.
		if ( psvr2_led_boot_oriented( &cfg_, cams_, lists, &h.model[mirror], prior, &out, &res ) == 0 &&
		     res.matched >= 12 )
		{
			h.cand_remap = remap;
			h.cand_mirror = mirror;
			DriverLog( "psvr2: optical pose: %s: boot ok (heading %.0f deg, remap %d, mirror %d, %d LEDs, %.2f px)",
			           HandName( hand ), yaw / kDeg, remap, mirror, res.matched, res.rms_px );
			return true;
		}
	}
	return false;
}

void OpticalPose::Accept( int hand, Hand &h, uint64_t frame_us, const vrmath::Pose &world_from_rig,
                          const psvr2_pose3 &rig_pose, const vr::HmdQuaternion_t &q_imu, const psvr2_solve_result &res )
{
	Rt rm;
	std::memcpy( rm.R, rig_pose.R, sizeof( rm.R ) );
	std::memcpy( rm.t, rig_pose.t, sizeof( rm.t ) );
	const Rt wm = Mul( ToRt( world_from_rig ), rm );
	const vrmath::Pose world_model = ToPose( wm );

	const int remap = h.remap >= 0 ? h.remap : h.cand_remap;
	double Pt[9], Rimu[9];
	MatT( kRemap.R[remap], Pt );
	MatMul( wm.R, Pt, Rimu );
	const vr::HmdQuaternion_t world_imu = vrmath::FromMatrix( Rimu );
	// Optical world <- IMU against the fused IMU: a correct remap leaves a
	// heading-only difference.
	const double tilt = TiltDeg( vrmath::Mul( world_imu, vrmath::Conj( q_imu ) ) );

	h.solved++;
	h.matched += res.matched;
	h.rms += res.rms_px;

	if ( h.last_us && frame_us > h.last_us )
	{
		const double dt = double( frame_us - h.last_us ) * 1e-6;
		if ( dt > 0.004 && dt < 0.2 )
		{
			const vrmath::Vec3 v = vrmath::Scale( vrmath::Add( world_model.pos, vrmath::Scale( h.last.pos, -1.0 ) ),
			                                      1.0 / dt );
			h.vel = vrmath::Add( vrmath::Scale( h.vel, 0.5 ), vrmath::Scale( v, 0.5 ) );
		}
	}
	h.last = world_model;
	h.last_us = frame_us;
	h.q_last = q_imu;

	if ( pose_log_ )
	{
		if ( !pose_log_until_us_ )
			pose_log_until_us_ = frame_us + 180000000;
		if ( frame_us < pose_log_until_us_ )
		{
			vr::DriverPose_t head;
			if ( hmd_->PoseAt( frame_us, head ) )
			{
				const vr::HmdQuaternion_t rq = vrmath::FromMatrix( rm.R );
				std::fprintf( pose_log_,
				              "%llu %d %d %.3f | %.2f %.2f %.2f %.6f %.6f %.6f %.6f | %.5f %.5f %.5f %.6f %.6f %.6f %.6f | "
				              "%.6f %.6f %.6f %.6f | %.4f\n",
				              (unsigned long long)frame_us, hand, res.matched, res.rms_px, rm.t[0], rm.t[1], rm.t[2], rq.w,
				              rq.x, rq.y, rq.z, head.vecPosition[0], head.vecPosition[1], head.vecPosition[2],
				              head.qRotation.w, head.qRotation.x, head.qRotation.y, head.qRotation.z, q_imu.w, q_imu.x,
				              q_imu.y, q_imu.z, tilt );
			}
		}
	}

	if ( h.remap < 0 )
	{
		// Learning the remap: wrong guesses show as a tilt between the
		// optical and the gravity-aligned IMU orientation once the
		// controller turns.
		if ( tilt > 15.0 )
		{
			h.tracking = false;
			return;
		}
		if ( tilt < 8.0 )
		{
			h.good++;
			h.span = std::max( h.span, vrmath::Angle( vrmath::Mul( q_imu, vrmath::Conj( h.span_ref ) ) ) / kDeg );
		}
		if ( h.good >= 30 && h.span >= 30.0 )
			Confirm( hand, h );
		return;
	}

	// Known remap: hand the controller its IMU's pose.
	if ( tilt > 20.0 )
		return;  // a wrong solve (near-planar ring flip); the IMU disagrees
	const double *v = kImuInModel[hand];
	const vrmath::Vec3 imu_pos = vrmath::Add(
		world_model.pos, vrmath::Rotate( world_model.rot, { v[0] * 1e-3, v[1] * 1e-3, v[2] * 1e-3 } ) );
	if ( controllers_[hand]->OpticalFix( frame_us, imu_pos, world_imu ) )
		h.fixes++;
}

void OpticalPose::Confirm( int hand, Hand &h )
{
	h.remap = h.cand_remap;
	h.mirror = h.cand_mirror;
	vr::VRSettings()->SetInt32( kSection, hand == 0 ? "optical_remap_left" : "optical_remap_right", h.remap );
	if ( hand == 0 )
		vr::VRSettings()->SetInt32( kSection, "optical_mirror_left", h.mirror );
	const double *R = kRemap.R[h.remap];
	DriverLog( "psvr2: optical pose: %s: IMU -> LED model rotation learned: remap %d [%g %g %g / %g %g %g / %g %g %g], "
	           "mirror %d (saved)",
	           HandName( hand ), h.remap, R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8], h.mirror );
}
