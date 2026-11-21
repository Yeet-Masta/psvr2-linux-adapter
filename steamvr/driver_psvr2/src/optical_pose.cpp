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

// IMU <- LED model rotation, entries [0..23]: the 24 proper rotations that
// map axes onto axes ([0] is the identity), searched when the remap is
// unknown. [24] (kSenseMount): the Sense IMU's real mount, which no axis
// permutation can express: the transpose of libpad's mounting matrix
// DAT_180351668 (IMU = M * raw, then a pure translation to the LED model),
// 50.3 deg about x. Measured against Sony's driver (2026-11-18,
// ~/sony-capture-3): 51 deg for both hands once the head <- rig tilt
// (optical_rig_tilt_deg) is fitted too.
constexpr int kSenseMount = 24;
struct Remaps
{
	double R[25][9];
	Remaps()
	{
		const double mount[9] = { 1, 0, 0, 0, 0.639167, -0.769068, 0, 0.769068, 0.639167 };
		std::memcpy( R[kSenseMount], mount, sizeof( mount ) );
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

// See the blob list in OpticalPose::Frame.
constexpr float kLdBlobOffsetX = 4.0f;

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

// Gravity for Sony's pose check: measured in the model frame (from the
// gravity-aligned IMU orientation, world <- model with any heading) and
// expected in the rig frame. OpenVR world: +y up.
void FillGravity( const double world_from_model[9], const double rig_from_world[9], psvr2_imu_prior &p )
{
	for ( int i = 0; i < 3; i++ )
	{
		p.g_model[i] = -world_from_model[3 + i];  // R^T (0, -1, 0)
		p.g_rig[i] = -rig_from_world[3 * i + 1];
	}
	p.gravity_valid = 1;
	p.gravity_age_s = 0;
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
	{
		vr::EVRSettingsError e = vr::VRSettingsError_None;
		const bool sony = vr::VRSettings()->GetBool( kSection, "optical_sony_match", &e );
		cfg_.sony_match = e == vr::VRSettingsError_None ? sony : 1;
		e = vr::VRSettingsError_None;
		const bool states = vr::VRSettings()->GetBool( kSection, "optical_sony_states", &e );
		sony_states_ = e == vr::VRSettingsError_None ? states : true;
		e = vr::VRSettingsError_None;
		const bool edge = vr::VRSettings()->GetBool( kSection, "optical_edge_state", &e );
		cfg_.edge_state = e == vr::VRSettingsError_None ? edge : 1;
	}
	// Defaults fitted against Sony's driver (2026-11-18, ~/sony-capture-3):
	// with them, Sony's poses need < 1 deg and < 5 mm more head <- rig
	// correction, and the right hand matches Sony to 2.7 mm / 1.6 deg (median).
	rig_offset_ = { GetFloat( "optical_rig_offset_x", -0.005f ), GetFloat( "optical_rig_offset_y", -0.009f ),
	                GetFloat( "optical_rig_offset_z", -0.011f ) };
	rig_tilt_deg_ = GetFloat( "optical_rig_tilt_deg", 6.0f );
	for ( int i = 0; i < 2; i++ )
		LoadHand( i );

	vr::EVRSettingsError e = vr::VRSettingsError_None;
	if ( vr::VRSettings()->GetBool( kSection, "optical_pose_log", &e ) && e == vr::VRSettingsError_None )
	{
		const char *home = std::getenv( "HOME" );
		const std::string path = std::string( home ? home : "/tmp" ) + "/psvr2-optical-pose.txt";
		pose_log_ = std::fopen( path.c_str(), "w" );
		if ( pose_log_ )
		{
			// Line-buffered: the log must be readable while SteamVR runs.
			std::setvbuf( pose_log_, nullptr, _IOLBF, 0 );
			std::fprintf( pose_log_, "# us hand matched rms_px | rig<-model t(mm) q(wxyz) | world<-head pos q | "
			                         "fused imu q | tilt_deg\n" );
		}
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
	// Both hands: the measured mount (kSenseMount). The earlier default,
	// remap 7 for the right hand, came from boots biased by their own prior.
	h.remap = GetInt( hand == 0 ? "optical_remap_left" : "optical_remap_right", kSenseMount );
	if ( h.remap < -1 || h.remap > kSenseMount )
		h.remap = -1;
	// Sony always mirrors the left model in x: no model sets its own mirror
	// byte (+0x31e is 0 in all seven), so the descriptor's 'X' applies
	// (opticalpose.md §2b.2). Unmirrored, the left hand booted onto the
	// right controller on hardware.
	h.mirror = hand == 0 ? 1 : 0;
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
	float wt[PSVR2_SOLVER_CAMS][PSVR2_SOLVER_BLOBS];
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
		lists[c].has_angle = 1;
		for ( int k = 0; k < lists[c].count; k++ )
		{
			// Ellipse angle (+ 2 pi; -1 = undefined) for the grazing check.
			lists[c].angle[k] = blobs[c].angle[idx[k]];
			// LD blob x reads 4 px (508 image) left of the calibrated model
			// on every camera; PSVR2Toolkit compensates in Sony's driver by
			// moving cx 8 px left in the 1016 calibration (ir_tracker_hooks.cpp).
			lists[c].xy[k][0] = blobs[c].x[idx[k]] + kLdBlobOffsetX;
			lists[c].xy[k][1] = blobs[c].y[idx[k]];
			wt[c][k] = blobs[c].weight[idx[k]];
		}
	}

	// A tracked hand claims its blobs before a searching one boots on them.
	const int first = !hands_[0].tracking && hands_[1].tracking ? 1 : 0;
	for ( int k = 0; k < 2; k++ )
	{
		const int hand = first ^ k;
		Hand &h = hands_[hand];
		if ( !synced[hand] )
		{
			h.tracking = false;
			continue;
		}
		h.frames++;
		SolveHand( hand, frame_us, world_from_rig, lists, wt );
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
			           "rms %.2f px, remap %d, %d re-boots",
			           HandName( hand ), h.tracking ? "tracking" : "searching", h.solved, h.frames, h.boots, h.fixes,
			           h.solved ? double( h.matched ) / h.solved : 0.0, h.solved ? h.rms / h.solved : 0.0,
			           h.remap >= 0 ? h.remap : h.cand_remap, h.reboots );
			h.frames = h.solved = h.boots = h.fixes = h.matched = h.reboots = 0;
			h.rms = 0;
		}
	}
}

void OpticalPose::SolveHand( int hand, uint64_t frame_us, const vrmath::Pose &world_from_rig,
                             psvr2_blob_list lists[PSVR2_SOLVER_CAMS],
                             float wt[PSVR2_SOLVER_CAMS][PSVR2_SOLVER_BLOBS] )
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
		// Sony's track step (FUN_1800db980): gates scaled by the time since
		// the last fix, then its gravity (20 deg), orientation (30 deg from
		// the prediction) and depth checks (FUN_1800d89f0).
		double Rnow[9], Bm[9];
		vrmath::ToMatrix( q_imu, Rnow );
		MatMul( Rnow, P, Bm );  // world <- model, heading aside
		psvr2_imu_prior prior{};
		FillGravity( Bm, rig_from_world.R, prior );
		prior.orient_valid = 1;
		std::memcpy( prior.R, predicted.R, sizeof( prior.R ) );
		prior.orient_age_s = dt;
		prior.spin_rad_s = dt > 1e-3 ? vrmath::Angle( d ) / dt : 0.0;
		prior.p1p_frames = h.p1p;
		// Sony's state machine (FUN_1800db690): close-range and edge states
		// beside the normal track step.
		ok = ( sony_states_ ? psvr2_led_track_state( &cfg_, cams_, lists, &h.model[h.mirror], &predicted, &prior,
		                                             dt, &h.state, &out, &res )
		                    : psvr2_led_track_ex( &cfg_, cams_, lists, &h.model[h.mirror], &predicted, &prior, dt,
		                                          &out, &res ) ) == 0;
		// Sony (FUN_1800dcf00): count consecutive one-LED tracks.
		h.p1p = ok && res.method == 3 ? h.p1p + 1 : 0;
		// Sony (FUN_1800d5070): the boot also runs while tracking, and its
		// pose replaces a track whose cross-camera support is < 0.5 (or that
		// failed) when it has 2+ more inliers, a score lower by 2, or a lower
		// score and lies nearer the last pose (here: the prediction).
		if ( h.remap >= 0 )
		{
			const double sup = ok ? psvr2_led_support( &cfg_, cams_, lists, &h.model[h.mirror], &out, &res ) : 0.0;
			psvr2_pose3 bo{};
			psvr2_solve_result br{};
			psvr2_imu_prior grav = prior;
			grav.orient_valid = 0;
			if ( sup < 0.5 && Boot( hand, h, frame_us, world_from_rig, q_imu, yaw_now, lists, bo, br, true ) &&
			     psvr2_led_pose_check( &cfg_, cams_, &h.model[h.mirror], &grav, &bo, &br, 0 ) == 0 )
			{
				auto d2 = []( const double *a, const double *b ) {
					return ( a[0] - b[0] ) * ( a[0] - b[0] ) + ( a[1] - b[1] ) * ( a[1] - b[1] ) + ( a[2] - b[2] ) * ( a[2] - b[2] );
				};
				const bool nearer = d2( out.t, predicted.t ) > d2( bo.t, predicted.t );
				if ( !ok || ( nearer && res.score > br.score ) || res.matched + 2 <= br.matched || res.score > br.score + 2 )
				{
					out = bo;
					res = br;
					ok = true;
					h.reboots++;
				}
			}
		}
		if ( !ok && ++h.fails >= 6 )
			h.tracking = false;
	}
	if ( !h.tracking )
	{
		ok = Boot( hand, h, frame_us, world_from_rig, q_imu, yaw_now, lists, out, res );
		if ( ok && h.remap >= 0 )
		{
			// Sony checks gravity on boot results too (FUN_1800d89f0).
			double Rnow[9], Bm[9];
			vrmath::ToMatrix( q_imu, Rnow );
			MatMul( Rnow, kRemap.R[h.remap], Bm );
			psvr2_imu_prior prior{};
			FillGravity( Bm, rig_from_world.R, prior );
			ok = psvr2_led_pose_check( &cfg_, cams_, &h.model[h.mirror], &prior, &out, &res, 0 ) == 0;
		}
		if ( ok )
		{
			h.tracking = true;
			h.good = 0;
			h.span = 0;
			h.span_ref = q_imu;
			h.vel = {};
			h.last_us = 0;
			h.p1p = 0;
			h.state = psvr2_led_boot_state( cams_, &out );
		}
	}
	if ( !ok )
		return;
	h.fails = 0;
	// Sync view: matched blob weight and view angle per camera and LED.
	h.view_us = frame_us;
	for ( int c = 0; c < PSVR2_SOLVER_CAMS; c++ )
		for ( int l = 0; l < PSVR2_LED_MAX; l++ )
		{
			const int k = res.led_blob[c][l];
			double px[2], va = 180.0;
			h.view_w[c][l] = l < h.model[h.mirror].count && k >= 0 && k < lists[c].count ? wt[c][k] : -1.0f;
			if ( h.view_w[c][l] >= 0 )
				psvr2_led_project( &cams_[c], &h.model[h.mirror], &out, l, px, &va, nullptr );
			h.view_deg[c][l] = float( va );
		}
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
				wt[c][n] = wt[c][k];
				n++;
			}
		lists[c].count = n;
	}
}

bool OpticalPose::Boot( int hand, Hand &h, uint64_t frame_us, const vrmath::Pose &world_from_rig, const vr::HmdQuaternion_t &q_imu,
                        double yaw_now, const psvr2_blob_list lists[PSVR2_SOLVER_CAMS], psvr2_pose3 &out,
                        psvr2_solve_result &res, bool quiet )
{
	// Hypotheses: heading (the controller's current one first when the
	// remap is known, then 12 steps of 30 deg, offset 15 deg every other
	// sweep) x remap while it is unknown.
	const bool known = h.remap >= 0;
	const int nremap = known ? 1 : 24;
	const int nyaw = known ? 13 : 12;
	const unsigned total = unsigned( nyaw * nremap );

	double Rrw[9];
	const Rt rig_from_world = Inv( ToRt( world_from_rig ) );
	std::memcpy( Rrw, rig_from_world.R, sizeof( Rrw ) );
	double Rimu[9];
	vrmath::ToMatrix( q_imu, Rimu );

	// Known remap: wrong headings still match 12-15 of the ring's LEDs
	// (hardware, 2026-11-18), right ones 18-30. So try every heading in
	// this frame and keep the best; boot only near the controller's
	// heading while its fixes are recent, else need 18 LEDs.
	const bool locked = known && h.fix_us && frame_us - h.fix_us < 5000000;
	psvr2_pose3 best_out{};
	psvr2_solve_result best_res{};
	double best_yaw = 0;
	bool have_best = false;

	const uint64_t start = NowMicros();
	int tried = 0;
	while ( tried < int( total ) && ( known || tried == 0 || NowMicros() - start < 3000 ) )
	{
		const unsigned k = h.hyp++;
		tried++;
		const unsigned sweep = k / total, i = k % total;
		const int y = int( i % nyaw ), r = int( i / nyaw );
		const int remap = known ? h.remap : r;
		double yaw;
		if ( known && y == 0 )
			yaw = yaw_now;
		else
			yaw = ( ( known ? y - 1 : y ) * 30.0 + ( sweep & 1 ? 15.0 : 0.0 ) ) * kDeg;

		if ( locked && std::fabs( vrmath::WrapPi( yaw - yaw_now ) ) > 30.0 * kDeg )
			continue;

		double Ry[9], A[9], B[9], prior[9];
		vrmath::ToMatrix( vrmath::AxisAngle( 0, 1, 0, yaw ), Ry );
		MatMul( Ry, Rimu, A );                 // world <- IMU
		MatMul( A, kRemap.R[remap], B );       // world <- model
		MatMul( Rrw, B, prior );               // rig <- model
		h.boots++;
		// The solver accepts a boot from 4 matches; on a recorded frame those
		// were wrong poses (4-9 matches), the right one had 22-30. Own floor.
		if ( known )
		{
			psvr2_pose3 o{};
			psvr2_solve_result r{};
			if ( psvr2_led_boot_oriented( &cfg_, cams_, lists, &h.model[h.mirror], prior, &o, &r ) == 0 &&
			     ( !have_best || r.matched > best_res.matched ||
			       ( r.matched == best_res.matched && r.rms_px < best_res.rms_px ) ) )
			{
				best_out = o;
				best_res = r;
				best_yaw = yaw;
				have_best = true;
			}
			continue;
		}
		if ( psvr2_led_boot_oriented( &cfg_, cams_, lists, &h.model[h.mirror], prior, &out, &res ) == 0 &&
		     res.matched >= 12 )
		{
			h.cand_remap = remap;
			DriverLog( "psvr2: optical pose: %s: boot ok (heading %.0f deg, remap %d, %d LEDs, %.2f px)",
			           HandName( hand ), yaw / kDeg, remap, res.matched, res.rms_px );
			return true;
		}
	}
	if ( !have_best || best_res.matched < ( locked ? 12 : 18 ) )
		return false;
	out = best_out;
	res = best_res;
	h.cand_remap = h.remap;
	if ( !quiet )
		DriverLog( "psvr2: optical pose: %s: boot ok (heading %.0f deg%s, %d LEDs, %.2f px)", HandName( hand ),
		           best_yaw / kDeg, locked ? ", near the current heading" : "", res.matched, res.rms_px );
	return true;
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
	// Noise-zone inputs for the controller filter (docs/sony-re/ukf.md 4):
	// the IMU in the head frame, and the distance from the camera rig.
	vrmath::Vec3 head_rel{ 0.0, -0.3, -0.4 };
	vr::DriverPose_t head_at;
	if ( hmd_->PoseAt( frame_us, head_at ) )
	{
		const vrmath::Vec3 hp{ head_at.vecPosition[0], head_at.vecPosition[1], head_at.vecPosition[2] };
		head_rel = vrmath::Rotate( vrmath::Conj( head_at.qRotation ), vrmath::Add( imu_pos, vrmath::Scale( hp, -1.0 ) ) );
	}
	const double cam_dist =
		std::sqrt( rig_pose.t[0] * rig_pose.t[0] + rig_pose.t[1] * rig_pose.t[1] + rig_pose.t[2] * rig_pose.t[2] ) * 1e-3;
	if ( controllers_[hand]->OpticalFix( frame_us, imu_pos, world_imu, head_rel, cam_dist ) )
	{
		h.fixes++;
		h.fix_us = frame_us;
	}
}

void OpticalPose::Confirm( int hand, Hand &h )
{
	h.remap = h.cand_remap;
	vr::VRSettings()->SetInt32( kSection, hand == 0 ? "optical_remap_left" : "optical_remap_right", h.remap );
	const double *R = kRemap.R[h.remap];
	DriverLog( "psvr2: optical pose: %s: IMU -> LED model rotation learned: remap %d [%g %g %g / %g %g %g / %g %g %g], "
	           "(saved)",
	           HandName( hand ), h.remap, R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8] );
}
