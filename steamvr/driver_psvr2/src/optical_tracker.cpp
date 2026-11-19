// SPDX-License-Identifier: GPL-2.0
#include "optical_tracker.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <cstdlib>
#include <poll.h>

#include "controller_device.h"
#include "driverlog.h"

namespace
{
constexpr uint64_t kSec = 1000000;
constexpr uint16_t kVersionPresets = 0x100;  // config pushes need LD version > 0x100

uint64_t NowMicros()
{
	return uint64_t( std::chrono::duration_cast<std::chrono::microseconds>(
				 std::chrono::steady_clock::now().time_since_epoch() )
	                     .count() );
}

uint32_t Rd32( const uint8_t *p )
{
	uint32_t v;
	std::memcpy( &v, p, 4 );
	return v;
}

// Set 0x09 blocks (ledsync.md §2.6): Normal and Many Blob.
void Block09( int mode, uint8_t b[24] )
{
	static const uint8_t normal[24] = { 0xfe, 0, 0, 0, 0xfe, 0, 0, 0, 0x78, 0x30, 0x40, 0x20,
	                                    0x80, 0x3c, 0xa0, 0x04, 0x0a, 0x20, 0x02, 0x02, 0, 0, 0, 0 };
	std::memcpy( b, normal, 24 );
	if ( mode == 1 )
	{
		b[0x08] = 0xb4;
		b[0x09] = 0x20;
		b[0x0e] = 0xb4;
	}
}

// libpad calls this for the PRESCAN base time (ledsync.md §5.4), once per
// PRESCAN command and per 1 s re-sync. Sony returns the frame time as is.
// On hardware (2026-11-18) our headset -> host -> controller mapping put the
// window about 2 ms early: in a 47 s trace the LEDs were lit only at
// re-syncs whose base sat 1.87-2.58 ms (controller clock) after the
// unshifted one, never at 0. Own method: start at the learned offset
// (setting optical_sync_offset_us, default 2000) and, while no LEDs are in
// view, shift each call by one window width (2 ms), alternating sides out to
// ±8 ms, so nine re-syncs cover the whole 16.7 ms frame. While the cameras
// see the LEDs the shift is held, and after 3 s lit it is saved as the new
// offset.
std::atomic<bool> g_sweep_hold{ false };
std::atomic<unsigned> g_shift{ 8 };
std::atomic<int32_t> g_offset_us{ 2000 };
const int8_t kShift[9] = { 0, 1, -1, 2, -2, 3, -3, 4, -4 };

int32_t CurrentShiftUs() { return g_offset_us + int32_t( kShift[g_shift.load() % 9] ) * 2000; }

uint64_t HeadsetTimeCallback()
{
	if ( !g_sweep_hold )
		g_shift = ( g_shift.load() + 1 ) % 9;
	return OpticalTracker::HeadsetFrameMicros() + int64_t( CurrentShiftUs() );
}

// Logical -> physical LED (table at 0x180260aa0): identity for logical
// 1..16 in every Sense model; logical 0 maps to the last LED. The sync LED
// cycles through 1..16 so the model does not matter.
uint8_t SyncLedPhysical( int logical ) { return uint8_t( 1 + logical % 16 ); }
}  // namespace

static void TraceStatus( FILE *f, int hand, Psvr2ControllerDriver *ctrl );

OpticalTracker::OpticalTracker( Psvr2ControllerDriver *const controllers[2] )
{
	vr::EVRSettingsError err = vr::VRSettingsError_None;
	log_blobs_ = vr::VRSettings()->GetBool( "driver_psvr2", "optical_log_blobs", &err ) &&
	             err == vr::VRSettingsError_None;
	err = vr::VRSettingsError_None;
	if ( vr::VRSettings()->GetBool( "driver_psvr2", "optical_trace", &err ) && err == vr::VRSettingsError_None )
	{
		const char *home = std::getenv( "HOME" );
		const std::string path = std::string( home ? home : "/tmp" ) + "/psvr2-ld-trace.txt";
		trace_ = std::fopen( path.c_str(), "w" );
		DriverLog( "psvr2: optical: trace %s %s", path.c_str(), trace_ ? "open" : "failed" );
		if ( trace_ )
			std::fprintf( trace_, "# F us counter dev_us no_det period frame_host_us | cam n: x,y,w,h,weight,id,flags ...\n"
			                      "# C us hand bytes (LED-sync command to the controller)\n"
			                      "# S hand ... (controller LED-sync record after the command)\n" );
	}
	err = vr::VRSettingsError_None;
	led_test_ = vr::VRSettings()->GetBool( "driver_psvr2", "optical_led_test", &err ) &&
	            err == vr::VRSettingsError_None;
	if ( led_test_ )
		DriverLog( "psvr2: optical: LED on/off test mode (no sync)" );
	err = vr::VRSettingsError_None;
	const int32_t off = vr::VRSettings()->GetInt32( "driver_psvr2", "optical_sync_offset_us", &err );
	if ( err == vr::VRSettingsError_None && off > -8400 && off < 8400 )
		g_offset_us = off;
	saved_offset_us_ = g_offset_us;
	g_shift = 8;
	g_sweep_hold = false;
	// Sony's full sequence (BROAD, BG, STABLE) only with optical_sync_broad:
	// on hardware (2026-11-18, firmware 0x0306, preset 2) BROAD lit the sync
	// LED on every third frame, which matches none of the 11-frame codes, so
	// it never locked. PRESCAN lights all LEDs in a 2 ms window that libpad
	// re-centres every second, which is enough for the pose solve.
	err = vr::VRSettingsError_None;
	full_sync_ = vr::VRSettings()->GetBool( "driver_psvr2", "optical_sync_broad", &err ) &&
	             err == vr::VRSettingsError_None;
	DriverLog( "psvr2: optical: sync offset %d us, %s", int( g_offset_us ),
	           full_sync_ ? "full sync (BROAD/BG/STABLE)" : "tracking in PRESCAN" );
	pose_ = std::make_unique<OpticalPose>( controllers[0]->Hmd(), controllers );
	for ( int i = 0; i < 2; i++ )
	{
		controllers_[i] = controllers[i];
		syncers_[i] = std::make_unique<LedSyncer>( [this, i]( const uint8_t *cmd, uint8_t len ) {
			TraceCommand( i, cmd, len );
			controllers_[i]->LedSyncCommand( cmd, len );
			TraceStatus( trace_, i, controllers_[i] );
		} );
		syncers_[i]->HoldPrescan( !full_sync_ );
	}
}

void OpticalTracker::TraceCommand( int hand, const uint8_t *cmd, uint8_t len )
{
	if ( !trace_ )
		return;
	std::fprintf( trace_, "C %llu %d", (unsigned long long)NowMicros(), hand );
	for ( int i = 0; i < len; i++ )
		std::fprintf( trace_, " %02x", cmd[i] );
	std::fputc( '\n', trace_ );
}

// The controller-side record after a command (written once the command is
// applied), so a dropped command or a missing clock fit shows in the trace.
static void TraceStatus( FILE *f, int hand, Psvr2ControllerDriver *ctrl )
{
	SenseHid::LedSyncStatus st;
	if ( !f || !ctrl->LedSyncStatus( st ) )
		return;
	std::fprintf( f, "S %d phase %u seq %u period %u step %d width %d base %d cycle %u clock %d mode %u fiterr %d\n",
	              hand, st.phase, st.seq, st.period_id, st.step, st.width, st.base_time, st.cycle_ns, st.clock_ready,
	              st.sync_mode, st.fit_err_us );
}

void OpticalTracker::TraceFrame( const Header &h, int period )
{
	if ( !trace_ )
		return;
	if ( ++trace_frames_ > 60 * 60 )
	{
		std::fclose( trace_ );
		trace_ = nullptr;
		DriverLog( "psvr2: optical: trace finished" );
		return;
	}
	std::fprintf( trace_, "F %llu %u %u %d %d %llu", (unsigned long long)NowMicros(), h.counter, h.device_us,
	              h.no_detection, period, (unsigned long long)frame_us_.load() );
	for ( int c = 0; c < PSVR2_LD_CAMERAS; c++ )
	{
		std::fprintf( trace_, " | %d %d:", c, blob_count_[c] );
		for ( int i = 0; i < blob_count_[c]; i++ )
			std::fprintf( trace_, " %.1f,%.1f,%d,%d,%.3f,%u,%x", blobs_[c].x[i], blobs_[c].y[i], blobs_[c].w[i],
			              blobs_[c].h[i], blobs_[c].weight[i], blobs_[c].id[i], blobs_[c].flags[i] );
	}
	std::fputc( '\n', trace_ );
	if ( trace_frames_ % 60 == 0 )
		for ( int i = 0; i < 2; i++ )
			TraceStatus( trace_, i, controllers_[i] );
}

OpticalTracker::~OpticalTracker() { Stop(); }

void OpticalTracker::Start()
{
	if ( running_ )
		return;
	dev_ = psvr2_open();
	if ( !dev_ || psvr2_ld_fd( dev_ ) < 0 )
	{
		DriverLog( "psvr2: optical: /dev/psvr2-ld not available (kernel module or udev rules too old?)" );
		if ( dev_ )
			psvr2_close( dev_ );
		dev_ = nullptr;
		return;
	}
	LogCameraCalibration();
	psvr2_sync_headset_time = HeadsetTimeCallback;
	running_ = true;
	thread_ = std::thread( &OpticalTracker::Run, this );
	DriverLog( "psvr2: optical: LED sync started" );
}

// The pose solve needs the camera model; which raw value is which
// coefficient is still open, so log everything once for comparison.
void OpticalTracker::LogCameraCalibration()
{
	static const char *const names[] = { "camera_calib0", "camera_calib1" };
	for ( const char *name : names )
	{
		static uint8_t blob[0x800];
		const int n = psvr2_read_calibration( dev_, name, blob, sizeof( blob ) );
		struct psvr2_cam_calib cal;
		if ( n <= 0 || psvr2_cam_calib_parse( blob, unsigned( n ), &cal ) != 0 )
		{
			DriverLog( "psvr2: optical: %s: %s", name, n <= 0 ? "not received yet" : "malformed" );
			continue;
		}
		DriverLog( "psvr2: optical: %s: model '%s' v%u, %u cameras, %u extrinsics", name, cal.model, cal.version,
		           cal.ncam, cal.next );
		for ( unsigned c = 0; c < cal.ncam; c++ )
		{
			const auto &k = cal.cam[c];
			DriverLog( "psvr2: optical:  cam %u %ux%u K %g %g %g / %g %g %g / %g %g %g", k.id, k.width, k.height, k.K[0],
			           k.K[1], k.K[2], k.K[3], k.K[4], k.K[5], k.K[6], k.K[7], k.K[8] );
			char line[512];
			int o = 0;
			for ( int j = 0; j < PSVR2_CAM_RAW && o < int( sizeof( line ) ) - 24; j++ )
				o += snprintf( line + o, sizeof( line ) - o, " %.9g", k.raw[j] );
			DriverLog( "psvr2: optical:  cam %u raw%s", k.id, line );
			// RESEARCH.md §11 #1: expect s1..s4 ~ 0, small tilt, width >= 508.
			struct psvr2_cam_model m;
			if ( psvr2_cam_model_from_calib( &cal, c, &m ) == 0 )
				DriverLog( "psvr2: optical:  cam %u model fx %.2f fy %.2f cx %.2f cy %.2f (508 px); "
				           "s1..s4 %.3g %.3g %.3g %.3g, tau %.4f %.4f rad%s",
				           k.id, m.fx, m.fy, m.cx, m.cy, k.raw[8], k.raw[9], k.raw[10], k.raw[11], m.tau_x, m.tau_y,
				           k.width < PSVR2_CAM_MODEL_SIZE ? " (width < 508!)" : "" );
		}
		for ( unsigned e = 0; e < cal.next; e++ )
		{
			double R[9], t[3];
			psvr2_cam_extrinsic_to_rig( &cal.ext[e], R, t );
			DriverLog( "psvr2: optical:  extrinsic %u: %u -> %u, t %.2f %.2f %.2f mm", e, cal.ext[e].from,
			           cal.ext[e].to, t[0], t[1], t[2] );
		}
	}
}

void OpticalTracker::Stop()
{
	running_ = false;
	if ( thread_.joinable() )
		thread_.join();
	psvr2_sync_headset_time = nullptr;
	for ( int i = 0; i < 2; i++ )
		syncers_[i]->ForceAllOff();
	if ( trace_ )
		std::fclose( trace_ );
	trace_ = nullptr;
	if ( dev_ )
		psvr2_close( dev_ );
	dev_ = nullptr;
}

void OpticalTracker::Run()
{
	static psvr2_ld_packet pkt;  // ~37 KB; one tracker thread
	const int fd = psvr2_ld_fd( dev_ );
	while ( running_ )
	{
		// Sony waits up to 100 ms for a new LD packet per tracker cycle.
		pollfd pfd = { fd, POLLIN, 0 };
		const int r = poll( &pfd, 1, 100 );
		const uint64_t now = NowMicros();
		if ( r > 0 && psvr2_read_ld( dev_, &pkt, 0 ) == 1 )
		{
			Header h;
			if ( ParseHeader( pkt, h ) )
			{
				UpdateFrameTime( pkt, h );
				UpdateBlobMode( h );
				PushDetectionConfig( pkt, h );
				DecodeBlobs( pkt, h );
				// LEDs in view (>= 3 blobs): hold the PRESCAN time sweep
				// until none were seen for 3 s.
				int total = 0;
				for ( int c = 0; c < PSVR2_LD_CAMERAS; c++ )
					total += blob_count_[c];
				lit_frame_ = total >= 3;
				if ( lit_frame_ )
				{
					if ( now - lit_at_us_ > 200000 )
						lit_since_us_ = now;
					lit_at_us_ = now;
				}
				g_sweep_hold = now - lit_at_us_ < 3 * kSec;
				SaveOffset( now );
				RunSyncers( now );
				SolvePoses( now );
			}
		}
		else if ( r < 0 && errno != EINTR )
			break;
	}
}

// ---- LD packet (ledsync.md §2.8, sony-driver-re.md "LD packet layout") ----

bool OpticalTracker::ParseHeader( const psvr2_ld_packet &pkt, Header &h ) const
{
	if ( pkt.length < PSVR2_LD_PACKET_SIZE || pkt.data[0] != 'L' || pkt.data[1] != 'D' )
		return false;
	const uint8_t *d = pkt.data;
	std::memcpy( &h.version, d + 2, 2 );
	h.device_us = Rd32( d + 0x08 );
	h.cam_flags = Rd32( d + 0x0c );
	h.counter = Rd32( d + 0x14 );
	std::memcpy( &h.echo_a, d + 0x20, 4 );
	std::memcpy( &h.echo_b, d + 0x24, 4 );
	h.no_detection = d[0x35];
	for ( int c = 0; c < PSVR2_LD_CAMERAS; c++ )
	{
		const uint32_t n = Rd32( d + 0x40 + c * 0x2404 );
		h.count[c] = n > PSVR2_LD_MAX_BLOBS ? PSVR2_LD_MAX_BLOBS : n;
	}
	return true;
}

void OpticalTracker::UpdateFrameTime( const psvr2_ld_packet &pkt, const Header &h )
{
	const uint64_t now = NowMicros();
	if ( now - clock_at_us_ > kSec )
	{
		if ( psvr2_get_clock( dev_, &clock_ ) != 0 )
			clock_.valid = 0;
		clock_at_us_ = now;
	}
	uint64_t t = 0;
	if ( clock_.valid )
	{
		const int64_t ns = psvr2_clock_to_host_ns( &clock_, h.device_us );
		if ( ns > 0 )
			t = uint64_t( ns ) / 1000;
	}
	// No clock map yet: Sony uses "now - 6000 µs".
	if ( !t )
		t = pkt.timestamp_ns / 1000 - 6000;
	frame_us_ = t;
}

// ledsync.md §4.3. Extreme-Env modes need tracker settings Sony never sets
// by default, so only Normal <-> Many Blob is implemented.
void OpticalTracker::UpdateBlobMode( const Header &h )
{
	if ( h.version <= kVersionPresets )
		return;
	const bool flags = ( h.cam_flags & 0x3 ) || ( h.cam_flags & 0x300 );
	const bool many = h.count[0] >= 200 || h.count[1] >= 200;
	if ( blob_mode_ == 0 )
	{
		if ( many || flags )
		{
			blob_mode_ = 1;
			quiet_frames_ = 0;
			DriverLog( "psvr2: optical: Many Blob mode" );
		}
		return;
	}
	const bool quiet = !flags && h.count[0] <= 49 && h.count[1] <= 49;
	quiet_frames_ = quiet ? quiet_frames_ + 1 : 0;
	if ( quiet_frames_ >= 480 )
	{
		blob_mode_ = 0;
		DriverLog( "psvr2: optical: Normal blob mode" );
	}
}

void OpticalTracker::PushDetectionConfig( const psvr2_ld_packet &pkt, const Header &h )
{
	if ( h.version <= kVersionPresets )
		return;
	const uint8_t *d = pkt.data;

	// Set 0x09: re-push when the echo differs, never on the frame right
	// after a push (the headset applies it with one frame of delay).
	uint8_t b[24];
	Block09( blob_mode_, b );
	const bool differs = std::memcmp( d + 0x18, b + 0x08, 6 ) != 0 || d[0x1e] != b[0x14] || d[0x1f] != b[0x15] ||
	                     std::memcmp( d + 0x28, b + 0x0e, 6 ) != 0;
	if ( differs && !( pushed_09_ && h.counter == last_09_counter_ + 1 ) )
	{
		if ( psvr2_ctrl_set( dev_, 0x09, 1, b, sizeof( b ) ) == 0 )
		{
			pushed_09_ = true;
			last_09_counter_ = h.counter;
		}
	}

	// Set 0x14: preset values once per preset, when the echo differs.
	const int preset = GlobalPreset();
	if ( !preset || preset == pushed_14_preset_ || h.echo_b == 0.0f )
		return;
	const uint32_t a = preset == 2 ? 85 : 128;
	const uint16_t bw = preset == 2 ? 500 : 1000;
	if ( h.echo_a == float( a ) && h.echo_b == float( bw ) )
	{
		pushed_14_preset_ = preset;
		return;
	}
	uint8_t w[16] = {};
	const uint32_t flags = 0x24800001, applied = 0x100;
	std::memcpy( w, &a, 4 );
	std::memcpy( w + 4, &bw, 2 );
	std::memcpy( w + 8, &flags, 4 );
	std::memcpy( w + 12, &applied, 4 );
	if ( psvr2_ctrl_set( dev_, 0x14, 1, w, sizeof( w ) ) == 0 )
	{
		pushed_14_preset_ = preset;
		DriverLog( "psvr2: optical: LED window set for preset %d", preset );
	}
}

void OpticalTracker::DecodeBlobs( const psvr2_ld_packet &pkt, const Header &h )
{
	const int extended = h.version > 0xff;
	for ( int c = 0; c < PSVR2_LD_CAMERAS; c++ )
	{
		blob_count_[c] = 0;
		if ( h.no_detection )
			continue;
		const uint8_t *rec = pkt.data + 0x40 + c * 0x2404 + 4;
		for ( uint32_t i = 0; i < h.count[c]; i++ )
			psvr2_ld_decode_blob( rec + i * PSVR2_LD_BLOB_SIZE, extended, &blobs_[c], &blob_count_[c], nullptr );
	}
	// Pattern length of the controller now syncing (RESEARCH.md §7.8).
	int period = 0;
	for ( const auto &s : syncers_ )
		if ( s->Current() == LedSyncer::kBroad )
			period = 11;
		else if ( s->Current() == LedSyncer::kBg )
			period = 20;
	for ( int c = 0; c < PSVR2_LD_CAMERAS; c++ )
	{
		float w[PSVR2_LD_MAX_BLOBS];
		for ( int i = 0; i < blob_count_[c]; i++ )
			w[i] = ( blobs_[c].flags[i] & 0x300 ) ? 0.0f : blobs_[c].weight[i];
		finders_[c].Frame( blobs_[c].x, blobs_[c].y, w, blob_count_[c], period );
	}
	TraceFrame( h, period );
	if ( log_blobs_ )
		LogBlobs( h );
}

// Once a second per camera: frames with blobs, the largest count, the
// centroid extents (they show the pixel scale: 0..508 or the sensor size),
// the largest bounding box, and the last frame's first blobs. Used for the
// camera-index and pixel-scale checks in docs/HW-TESTS.md.
void OpticalTracker::LogBlobs( const Header &h )
{
	for ( int c = 0; c < PSVR2_LD_CAMERAS; c++ )
	{
		BlobStats &s = blob_stats_[c];
		s.frames++;
		if ( blob_count_[c] > 0 )
			s.seen++;
		s.max_count = std::max( s.max_count, blob_count_[c] );
		for ( int i = 0; i < blob_count_[c]; i++ )
		{
			s.min_x = std::min( s.min_x, blobs_[c].x[i] );
			s.max_x = std::max( s.max_x, blobs_[c].x[i] );
			s.min_y = std::min( s.min_y, blobs_[c].y[i] );
			s.max_y = std::max( s.max_y, blobs_[c].y[i] );
			s.max_w = std::max( s.max_w, int( blobs_[c].w[i] ) );
			s.max_h = std::max( s.max_h, int( blobs_[c].h[i] ) );
		}
	}
	const uint64_t now = NowMicros();
	if ( now - blob_log_at_us_ < kSec )
		return;
	blob_log_at_us_ = now;
	DriverLog( "psvr2: blobs: LD v%#x cam_flags %#x mode %d", h.version, h.cam_flags, blob_mode_ );
	for ( int c = 0; c < PSVR2_LD_CAMERAS; c++ )
	{
		BlobStats &s = blob_stats_[c];
		char last[160] = "";
		size_t o = 0;
		for ( int i = 0; i < blob_count_[c] && i < 4 && o < sizeof( last ) - 40; i++ )
			o += size_t( std::snprintf( last + o, sizeof( last ) - o, " (%.1f,%.1f %dx%d)", blobs_[c].x[i],
			                            blobs_[c].y[i], blobs_[c].w[i], blobs_[c].h[i] ) );
		float sx = 0, sy = 0;
		char sync[48] = " sync none";
		if ( finders_[c].Where( sx, sy ) )
			std::snprintf( sync, sizeof( sync ), " sync (%.1f,%.1f)", sx, sy );
		if ( s.seen )
			DriverLog( "psvr2: blobs: cam %d seen %d/%d max %d x %.1f..%.1f y %.1f..%.1f box %dx%d%s last%s", c,
			           s.seen, s.frames, s.max_count, s.min_x, s.max_x, s.min_y, s.max_y, s.max_w, s.max_h, sync,
			           last );
		else
			DriverLog( "psvr2: blobs: cam %d seen 0/%d", c, s.frames );
		s = BlobStats{};
	}
}

// ---- LED sync ----------------------------------------------------------

// The first connected controller fixes the preset (RESEARCH.md §7.5). Sony
// reads it from libpad's controller state; its HID origin is still open, so
// libpad's sync mode stands in for it (same timing values). In this Sony
// build every path ends in sync mode 2, so the preset is 2 in practice.
int OpticalTracker::GlobalPreset()
{
	SenseHid::LedSyncStatus st[2];
	bool on[2];
	for ( int i = 0; i < 2; i++ )
		on[i] = controllers_[i]->LedSyncStatus( st[i] ) && st[i].sync_mode != 0;
	if ( preset_owner_ >= 0 && !on[preset_owner_] )
	{
		preset_owner_ = -1;
		preset_ = 0;
		pushed_14_preset_ = 0;  // latch cleared when the owner disconnects
	}
	if ( preset_owner_ < 0 )
		for ( int i = 0; i < 2; i++ )
			if ( on[i] )
			{
				preset_owner_ = i;
				preset_ = st[i].sync_mode;
				break;
			}
	return preset_;
}

// LEDs lit for 3 s: keep the shift that lit them as the new offset.
void OpticalTracker::SaveOffset( uint64_t now_us )
{
	if ( !lit_frame_ || now_us - lit_since_us_ < 3 * kSec )
		return;
	const int32_t cur = CurrentShiftUs();
	if ( cur == saved_offset_us_ )
		return;
	saved_offset_us_ = cur;
	vr::VRSettings()->SetInt32( "driver_psvr2", "optical_sync_offset_us", cur );
	DriverLog( "psvr2: optical: LEDs lit at sync offset %d us, saved", int( cur ) );
}

// Controller poses for the controllers whose LEDs are synced.
void OpticalTracker::SolvePoses( uint64_t now_us )
{
	if ( now_us - rig_try_us_ > kSec )
	{
		rig_try_us_ = now_us;
		pose_->BuildRig( dev_ );
	}
	// Lit: STABLE, or held PRESCAN with LEDs in view.
	auto lit = [&]( int i ) {
		const auto p = syncers_[i]->Current();
		return p == LedSyncer::kStable || ( !full_sync_ && p == LedSyncer::kPrescan && now_us - lit_at_us_ < 500000 );
	};
	const bool synced[2] = { !see_through_ && !led_test_ && lit( 0 ), !see_through_ && !led_test_ && lit( 1 ) };
	pose_->Frame( frame_us_, blobs_, blob_count_, synced );
}

LedObservation OpticalTracker::Observe( int hand, int phase_owner )
{
	LedObservation o;
	o.sync_leds[0] = SyncLedPhysical( sync_led_[hand] );
	// [heuristic] One controller syncs at a time; the other waits in
	// PRESCAN while the owner is in BROAD/BG.
	if ( phase_owner >= 0 && phase_owner != hand )
		return o;
	// Without a pose, the sync LED is the blob that blinks (BlinkFinder).
	// Dark frames report a small weight with count 1, so the history stays
	// valid and the frame reads as dark (RESEARCH.md §7.8).
	bool any = false;
	for ( int c = 0; c < kLedCameras; c++ )
	{
		bool present = false;
		float w = 0;
		any |= blob_count_[c] > 0;
		if ( !finders_[c].Sync( present, w ) )
			continue;
		o.visible |= present;
		o.count[c] = 1;
		o.sync_weight[c] = present ? w : 0.01f;
	}
	// PRESCAN only needs to know the headset sees the LEDs: they do not blink
	// yet, so no site qualifies. A lone stray blob does not count.
	if ( syncers_[hand]->Current() == LedSyncer::kPrescan )
		o.visible = any && lit_frame_;
	return o;
}

void OpticalTracker::RunSyncers( uint64_t now_us )
{
	// See-through camera view: LEDs off for both, restart when it ends
	// (ledsync.md §3.2). Polled once a second like Sony's Get 0x97.
	if ( now_us - see_through_at_us_ > kSec )
	{
		see_through_at_us_ = now_us;
		const bool on = psvr2_get_see_through( dev_ ) > 0;
		if ( see_through_ && !on )
			for ( auto &s : syncers_ )
				s->ForceInit();
		see_through_ = on;
	}
	if ( see_through_ )
	{
		for ( auto &s : syncers_ )
			s->ForceAllOff();
		return;
	}

	if ( led_test_ )
	{
		// Every 2 s alternate LED ALL ON (base time anchored to the headset
		// clock) and LED ALL OFF. Blobs that come and go with this rhythm
		// are the controller's LEDs.
		if ( now_us - led_test_at_us_ >= 2 * kSec )
		{
			led_test_at_us_ = now_us;
			const bool on = !( led_test_step_++ & 1 );
			const uint8_t off_cmd[2] = { 1, LedSyncer::kAllOff };
			for ( int i = 0; i < 2; i++ )
			{
				const bool ok = on ? controllers_[i]->LedTestSweep( 0 ) : controllers_[i]->LedSyncCommand( off_cmd, 2 );
				if ( trace_ )
					std::fprintf( trace_, "T %llu %d %d %d\n", (unsigned long long)now_us, i, on, ok );
				TraceStatus( trace_, i, controllers_[i] );
			}
		}
		return;
	}
	const int preset = GlobalPreset();
	auto owning = []( LedSyncer::Phase p ) { return p == LedSyncer::kBroad || p == LedSyncer::kBg; };
	int owner = -1;
	for ( int i = 0; i < 2; i++ )
		if ( owning( syncers_[i]->Current() ) )
			owner = i;
	for ( int i = 0; i < 2; i++ )
	{
		SenseHid::LedSyncStatus st;
		if ( !controllers_[i]->LedSyncStatus( st ) || !preset )
		{
			syncers_[i]->ForceInit();
			continue;
		}
		// A controller on another preset cannot be tracked with the owner.
		if ( st.sync_mode != preset )
		{
			syncers_[i]->ForceInit();
			continue;
		}
		syncers_[i]->Frame( now_us, preset, Observe( i, owner ) );
		const auto p = syncers_[i]->Current();
		if ( last_phase_[i] == LedSyncer::kBroad && p == LedSyncer::kPrescan )
			sync_led_[i]++;  // that LED never decoded; try the next one
		// A new BROAD round starts from fresh blink history, and claims
		// ownership at once so the other hand cannot enter BROAD on the
		// same frame.
		if ( p == LedSyncer::kBroad && last_phase_[i] != LedSyncer::kBroad )
			for ( auto &f : finders_ )
				f.Reset();
		if ( owning( p ) && owner < 0 )
			owner = i;
		last_phase_[i] = p;
	}
}
