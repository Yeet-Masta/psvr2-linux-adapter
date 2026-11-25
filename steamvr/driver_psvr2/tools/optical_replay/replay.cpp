// SPDX-License-Identifier: GPL-2.0
// optical_replay: run the driver's OpticalPose on a recorded LD trace
// (~/psvr2-ld-trace.txt, setting "optical_trace") without SteamVR.
//
//   optical_replay [-c calib.bin] [-s key=value]... [-q] trace.txt
//
// -c: the headset's camera_calib0 blob (default ~/psvr2-camera-calib0.bin,
//     copy it from /sys/bus/usb/drivers/psvr2/*:1.3/camera_calib0).
// -s: a driver_psvr2 setting, as in steamvr.vrsettings.
// -q: no per-fix lines, summary only.
//
// The trace needs "P" lines (head pose + controller IMU per frame), which
// the driver writes since 2026-11-18. Output: driver log lines on stderr,
// one "FIX" line per optical fix on stdout, then a per-hand summary.
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "controller_device.h"
#include "driverlog.h"
#include "hmd_device_driver.h"
#include "optical_pose.h"

namespace
{
std::map<std::string, std::string> g_settings;
std::string g_calib_path;
bool g_quiet = false;

class Settings : public vr::IVRSettings
{
	const char *Find( const char *key, vr::EVRSettingsError *e )
	{
		auto it = g_settings.find( key );
		if ( e )
			*e = it == g_settings.end() ? vr::VRSettingsError_UnsetSettingHasNoDefault : vr::VRSettingsError_None;
		return it == g_settings.end() ? nullptr : it->second.c_str();
	}

public:
	const char *GetSettingsErrorNameFromEnum( vr::EVRSettingsError ) override { return "replay"; }
	void SetBool( const char *, const char *k, bool v, vr::EVRSettingsError * ) override { g_settings[k] = v ? "true" : "false"; }
	void SetInt32( const char *, const char *k, int32_t v, vr::EVRSettingsError * ) override
	{
		g_settings[k] = std::to_string( v );
		std::fprintf( stderr, "replay: setting %s = %d (not saved)\n", k, v );
	}
	void SetFloat( const char *, const char *k, float v, vr::EVRSettingsError * ) override { g_settings[k] = std::to_string( v ); }
	void SetString( const char *, const char *k, const char *v, vr::EVRSettingsError * ) override { g_settings[k] = v; }
	bool GetBool( const char *, const char *k, vr::EVRSettingsError *e ) override
	{
		const char *v = Find( k, e );
		return v && ( !std::strcmp( v, "true" ) || !std::strcmp( v, "1" ) );
	}
	int32_t GetInt32( const char *, const char *k, vr::EVRSettingsError *e ) override
	{
		const char *v = Find( k, e );
		return v ? std::atoi( v ) : 0;
	}
	float GetFloat( const char *, const char *k, vr::EVRSettingsError *e ) override
	{
		const char *v = Find( k, e );
		return v ? float( std::atof( v ) ) : 0.0f;
	}
	void GetString( const char *, const char *k, char *out, uint32_t len, vr::EVRSettingsError *e ) override
	{
		const char *v = Find( k, e );
		if ( len )
			std::snprintf( out, len, "%s", v ? v : "" );
	}
	void RemoveSection( const char *, vr::EVRSettingsError * ) override {}
	void RemoveKeyInSection( const char *, const char *, vr::EVRSettingsError * ) override {}
};

Settings g_vrsettings;

class Context : public vr::IVRDriverContext
{
public:
	void *GetGenericInterface( const char *version, vr::EVRInitError *e ) override
	{
		if ( e )
			*e = vr::VRInitError_None;
		if ( !std::strcmp( version, vr::IVRSettings_Version ) )
			return &g_vrsettings;
		if ( e )
			*e = vr::VRInitError_Init_InterfaceNotFound;
		return nullptr;
	}
	vr::DriverHandle_t GetDriverHandle() override { return 1; }
};

struct Fix
{
	double t;
	vrmath::Vec3 pos;
	double yaw_off;  // heading of world_from_imu relative to the fused IMU, deg
};
std::vector<Fix> g_fixes[2];
uint64_t g_t0 = 0;

std::vector<std::string> Split( const std::string &s, char sep )
{
	std::vector<std::string> out;
	size_t a = 0;
	for ( ;; )
	{
		const size_t b = s.find( sep, a );
		out.push_back( s.substr( a, b == std::string::npos ? std::string::npos : b - a ) );
		if ( b == std::string::npos )
			return out;
		a = b + 1;
	}
}
}  // namespace

void DriverLog( const char *fmt, ... )
{
	va_list ap;
	va_start( ap, fmt );
	std::vfprintf( stderr, fmt, ap );
	va_end( ap );
	std::fputc( '\n', stderr );
}

void DebugDriverLog( const char *, ... ) {}

extern "C" int psvr2_read_calibration( psvr2_t *, const char *name, void *buf, unsigned int len )
{
	if ( std::strcmp( name, "camera_calib0" ) )
		return -1;
	FILE *f = std::fopen( g_calib_path.c_str(), "rb" );
	if ( !f )
		return -1;
	const size_t n = std::fread( buf, 1, len, f );
	std::fclose( f );
	return int( n );
}

bool Psvr2ControllerDriver::OpticalFix( uint64_t frame_us, const vrmath::Vec3 &pos,
                                        const vr::HmdQuaternion_t &world_from_imu, const vrmath::Vec3 &,
                                        double, bool )
{
	const vr::HmdQuaternion_t c = vrmath::Mul( world_from_imu, vrmath::Conj( q_ ) );
	const double t = double( frame_us - g_t0 ) * 1e-6;
	const double yaw_off = vrmath::Yaw( c ) * 180.0 / M_PI;
	g_fixes[hand].push_back( { t, pos, yaw_off } );
	if ( !g_quiet )
		std::printf( "FIX %.3f %s pos %.3f %.3f %.3f yaw_off %.1f q %.6f %.6f %.6f %.6f\n", t, hand ? "right" : "left",
		             pos.x, pos.y, pos.z, yaw_off, world_from_imu.w, world_from_imu.x, world_from_imu.y,
		             world_from_imu.z );
	return true;
}

int main( int argc, char **argv )
{
	const char *home = std::getenv( "HOME" );
	g_calib_path = std::string( home ? home : "." ) + "/psvr2-camera-calib0.bin";
	const char *trace_path = nullptr;
	for ( int i = 1; i < argc; i++ )
	{
		if ( !std::strcmp( argv[i], "-c" ) && i + 1 < argc )
			g_calib_path = argv[++i];
		else if ( !std::strcmp( argv[i], "-s" ) && i + 1 < argc )
		{
			const std::string kv = argv[++i];
			const size_t eq = kv.find( '=' );
			if ( eq == std::string::npos )
				return std::fprintf( stderr, "bad -s %s\n", kv.c_str() ), 2;
			g_settings[kv.substr( 0, eq )] = kv.substr( eq + 1 );
		}
		else if ( !std::strcmp( argv[i], "-q" ) )
			g_quiet = true;
		else
			trace_path = argv[i];
	}
	if ( !trace_path )
		return std::fprintf( stderr, "usage: %s [-c calib.bin] [-s key=value]... [-q] trace.txt\n", argv[0] ), 2;
	FILE *f = std::fopen( trace_path, "r" );
	if ( !f )
		return std::perror( trace_path ), 1;

	static Context ctx;
	vr::InitServerDriverContext( &ctx );

	Psvr2HmdDriver hmd;
	Psvr2ControllerDriver left, right;
	left.hand = 0;
	right.hand = 1;
	left.hmd_ = right.hmd_ = &hmd;
	Psvr2ControllerDriver *const ctrls[2] = { &left, &right };
	OpticalPose pose( &hmd, ctrls );
	if ( !pose.BuildRig( reinterpret_cast<psvr2_t *>( 1 ) ) )
		return std::fprintf( stderr, "no rig: check %s\n", g_calib_path.c_str() ), 1;

	static psvr2_blob_set blobs[PSVR2_LD_CAMERAS];
	int counts[PSVR2_LD_CAMERAS] = {};
	bool have_frame = false;
	unsigned frames = 0, inputs = 0;
	std::string line;
	char buf[1 << 16];
	while ( std::fgets( buf, sizeof( buf ), f ) )
	{
		line = buf;
		if ( line.size() < 2 )
			continue;
		const auto parts = Split( line, '|' );
		if ( line[0] == 'F' )
		{
			for ( int c = 0; c < PSVR2_LD_CAMERAS; c++ )
				counts[c] = 0;
			for ( size_t p = 1; p < parts.size(); p++ )
			{
				int cam = -1, n = 0, off = 0;
				if ( std::sscanf( parts[p].c_str(), " %d %d:%n", &cam, &n, &off ) < 2 || cam < 0 ||
				     cam >= PSVR2_LD_CAMERAS )
					continue;
				const char *s = parts[p].c_str() + off;
				int k = 0;
				for ( ;; )
				{
					float x, y, wt;
					int w, h, id, used;
					unsigned fl;
					if ( std::sscanf( s, " %f,%f,%d,%d,%f,%d,%x%n", &x, &y, &w, &h, &wt, &id, &fl, &used ) < 7 ||
					     k >= PSVR2_LD_MAX_BLOBS )
						break;
					blobs[cam].x[k] = x;
					blobs[cam].y[k] = y;
					blobs[cam].w[k] = int16_t( w );
					blobs[cam].h[k] = int16_t( h );
					blobs[cam].weight[k] = wt;
					blobs[cam].id[k] = uint8_t( id );
					blobs[cam].flags[k] = uint16_t( fl );
					k++;
					s += used;
				}
				counts[cam] = k;
			}
			have_frame = true;
			frames++;
		}
		else if ( line[0] == 'P' && have_frame && parts.size() >= 4 )
		{
			unsigned long long fu;
			int s0, s1;
			if ( std::sscanf( parts[0].c_str(), "P %llu %d %d", &fu, &s0, &s1 ) != 3 )
				continue;
			if ( !g_t0 )
				g_t0 = fu;
			int hok;
			double p[3], q[4], d[3];
			if ( std::sscanf( parts[1].c_str(), " %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &hok, &p[0], &p[1], &p[2],
			                  &q[0], &q[1], &q[2], &q[3], &d[0], &d[1], &d[2] ) != 11 )
				continue;
			hmd.ok_ = hok;
			for ( int i = 0; i < 3; i++ )
			{
				hmd.pose_.vecPosition[i] = p[i];
				hmd.pose_.vecDriverFromHeadTranslation[i] = d[i];
			}
			hmd.pose_.qRotation = { q[0], q[1], q[2], q[3] };
			for ( int hand = 0; hand < 2; hand++ )
			{
				Psvr2ControllerDriver &c = *ctrls[hand];
				int ok;
				double w, x, y, z, yaw;
				unsigned rev;
				if ( std::sscanf( parts[2 + hand].c_str(), " %d %lf %lf %lf %lf %lf %x", &ok, &w, &x, &y, &z, &yaw, &rev ) ==
				     7 )
				{
					c.ok_ = ok;
					c.q_ = { w, x, y, z };
					c.yaw_ = yaw;
					c.rev_ = int( rev );
				}
			}
			const bool synced[2] = { s0 != 0, s1 != 0 };
			pose.Frame( fu, blobs, counts, synced );
			inputs++;
		}
	}
	std::fclose( f );

	std::printf( "replay: %u LD frames, %u with inputs\n", frames, inputs );
	if ( !inputs )
		std::printf( "replay: no P lines: record a new trace with this driver build\n" );
	for ( int hand = 0; hand < 2; hand++ )
	{
		const auto &v = g_fixes[hand];
		int jumps = 0, pos_jumps = 0;
		for ( size_t i = 1; i < v.size(); i++ )
		{
			double dy = std::fabs( std::remainder( v[i].yaw_off - v[i - 1].yaw_off, 360.0 ) );
			const vrmath::Vec3 dp = vrmath::Add( v[i].pos, vrmath::Scale( v[i - 1].pos, -1.0 ) );
			const double dt = v[i].t - v[i - 1].t;
			if ( dy > 20.0 )
				jumps++;
			if ( dt < 0.1 && std::sqrt( dp.x * dp.x + dp.y * dp.y + dp.z * dp.z ) > 0.15 )
				pos_jumps++;
		}
		std::printf( "%s: %zu fixes, %d heading jumps > 20 deg, %d position jumps > 15 cm within 0.1 s\n",
		             hand ? "right" : "left", v.size(), jumps, pos_jumps );
	}
	return 0;
}
