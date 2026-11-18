// SPDX-License-Identifier: GPL-2.0
// Protocol handling follows Monado's pssense driver (BSL-1.0); see
// sense_protocol.h for attribution.
#include "sense_hid.h"

#include <cerrno>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <linux/hidraw.h>
#include <mutex>
#include <set>
#include <sys/ioctl.h>
#include <unistd.h>

#include "driverlog.h"
#include "openvr_driver.h"

namespace
{
constexpr double kGravity = 9.80665;
constexpr double kPi = 3.14159265358979323846;

int BusOf( const std::string &hid_id )
{
	return static_cast<int>( std::strtol( hid_id.c_str(), nullptr, 16 ) );
}
} // namespace

std::vector<SenseHidInfo> FindSenseControllers()
{
	std::vector<SenseHidInfo> found;
	DIR *d = opendir( "/sys/class/hidraw" );
	if ( !d )
		return found;

	while ( dirent *e = readdir( d ) )
	{
		if ( std::strncmp( e->d_name, "hidraw", 6 ) != 0 )
			continue;
		std::ifstream uevent( std::string( "/sys/class/hidraw/" ) + e->d_name + "/device/uevent" );
		std::string line, hid_id, uniq;
		while ( std::getline( uevent, line ) )
		{
			if ( line.rfind( "HID_ID=", 0 ) == 0 )
				hid_id = line.substr( 7 );
			else if ( line.rfind( "HID_UNIQ=", 0 ) == 0 )
				uniq = line.substr( 9 );
		}
		// HID_ID=BBBB:VVVVVVVV:PPPPPPPP
		unsigned bus = 0, vid = 0, pid = 0;
		if ( std::sscanf( hid_id.c_str(), "%x:%x:%x", &bus, &vid, &pid ) != 3 || vid != sense::kSonyVendorId )
			continue;
		if ( pid != sense::kLeftProductId && pid != sense::kRightProductId )
			continue;

		SenseHidInfo info;
		info.path = std::string( "/dev/" ) + e->d_name;
		info.left = pid == sense::kLeftProductId;
		info.bluetooth = BusOf( hid_id ) == 0x5;
		info.uniq = uniq;
		found.push_back( info );
	}
	closedir( d );
	return found;
}

namespace
{
uint64_t SteadyMicros()
{
	return uint64_t(
		std::chrono::duration_cast<std::chrono::microseconds>( std::chrono::steady_clock::now().time_since_epoch() )
			.count() );
}
}  // namespace

SenseHid::SenseHid( const SenseHidInfo &info ) : info_( info )
{
	psvr2_sync_init( &sync_, info_.bluetooth ? 1 : 0, info_.left ? 1 : 0 );
	// libpad's reset values (ledsync.md 2.2): LED ALL OFF, preset-1 timing.
	led_.phase = sense::kLedPhaseAllOff;
	led_.cycle_ns = 16683350;
	std::memset( led_.led, 0xff, sizeof( led_.led ) );
	led_.width = 1000;
	led_.step = 105;
	led_.half = 1575;
}

SenseHid::~SenseHid()
{
	if ( fd_ >= 0 )
		close( fd_ );
}

bool SenseHid::Open()
{
	fd_ = open( info_.path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC );
	if ( fd_ < 0 )
	{
		// The provider retries every few seconds; say it once per node.
		static std::mutex logged_mutex;
		static std::set<std::string> logged;
		std::lock_guard<std::mutex> lock( logged_mutex );
		if ( logged.insert( info_.path ).second )
			DriverLog( "psvr2: cannot open %s: %s (udev rule installed? see docs/install.md)", info_.path.c_str(),
			           strerror( errno ) );
		return false;
	}
	if ( !ReadCalibration() )
	{
		close( fd_ );
		fd_ = -1;
		return false;
	}
	ReadFirmwareInfo();
	// Sony never sets the polling rate (feature 0x08); keep it opt-in.
	if ( vr::VRSettings()->GetBool( "driver_psvr2", "set_polling_rate" ) && !SetPollingRate() )
		DriverLog( "psvr2: %s: could not set PC polling rate; continuing", info_.path.c_str() );
	return true;
}

void SenseHid::ReadFirmwareInfo()
{
	sense::FirmwareInfoReport r{};
	r.report_id = sense::kFirmwareInfoFeatureReport;
	if ( ioctl( fd_, HIDIOCGFEATURE( sizeof( r ) ), &r ) != static_cast<int>( sizeof( r ) ) )
		return;
	if ( info_.bluetooth && sense::SeededCrc32( sense::kFeatureCrcSeed, &r, sizeof( r ) - 4 ) != r.crc )
		return;
	fw_version_ = r.firmware_version;
	fw_revision_ = r.firmware_revision;
	hw_revision_ = uint16_t( r.hardware_info & 0xffff );
	DriverLog( "psvr2: %s: firmware %08x rev %04x, hardware %04x", info_.path.c_str(), fw_version_, fw_revision_,
	           hw_revision_ );
}

bool SenseHid::ReadCalibration()
{
	sense::CalibrationData cal{};
	auto *raw = reinterpret_cast<uint8_t *>( &cal );
	constexpr size_t kHalf = sizeof( sense::FeatureReport::data );

	for ( int attempt = 0; attempt < 5; attempt++ )
	{
		bool ok = true;
		for ( int part = 0; part < 2 && ok; part++ )
		{
			sense::FeatureReport r{};
			r.report_id = sense::kCalibrationFeatureReport;
			const int n = ioctl( fd_, HIDIOCGFEATURE( sizeof( r ) ), &r );
			if ( n != static_cast<int>( sizeof( r ) ) )
			{
				DriverLog( "psvr2: %s: calibration read failed (%d, %s)", info_.path.c_str(), n, strerror( errno ) );
				return false;
			}
			// CRC is only filled in over Bluetooth.
			if ( info_.bluetooth && sense::SeededCrc32( sense::kFeatureCrcSeed, &r, sizeof( r ) - 4 ) != r.crc )
				ok = false;
			else if ( r.part_id == sense::kCalibrationPart1 )
				std::memcpy( raw, r.data, kHalf );
			else if ( r.part_id == sense::kCalibrationPart2 )
				std::memcpy( raw + kHalf, r.data, kHalf );
			else
				ok = false;
		}
		if ( ok )
			break;
		if ( attempt == 4 )
		{
			DriverLog( "psvr2: %s: calibration CRC kept failing", info_.path.c_str() );
			return false;
		}
	}

	// Reading the report is also what switches the controller to full mode.
	// Use the factory calibration the way Sony's libpad does (the DualSense
	// formula; docs/sony-driver-re.md "IMU calibration"). If it is degenerate,
	// fall back to Monado's fixed scales and let ImuFusion learn gyro bias.
	if ( !ApplyCalibration( cal ) )
	{
		DriverLog( "psvr2: %s: factory IMU calibration unusable, using nominal scales", info_.path.c_str() );
		gyro_bias_ = {};
		accel_bias_ = {};
		const double g = 180.0 / 1024.0 * kPi / 180.0;  // rad/s per LSB
		const double a = kGravity / 4096.0;             // m/s^2 per LSB
		gyro_scale_ = { g, g, g };
		accel_scale_ = { a, a, a };
	}
	return true;
}

bool SenseHid::ApplyCalibration( const sense::CalibrationData &cal )
{
	struct Axis
	{
		int bias, plus, minus;  // gyro
		int aplus, aminus;      // accel
	};
	const Axis axes[3] = {
		{ cal.gyro_bias_x, cal.gyro_plus_x, cal.gyro_minus_x, cal.accel_plus_x, cal.accel_minus_x },
		{ cal.gyro_bias_y, cal.gyro_plus_y, cal.gyro_minus_y, cal.accel_plus_y, cal.accel_minus_y },
		{ cal.gyro_bias_z, cal.gyro_plus_z, cal.gyro_minus_z, cal.accel_plus_z, cal.accel_minus_z },
	};
	// Reference rotation speed in deg/s, as rad/s.
	const double speed = ( cal.gyro_speed_ref1 + cal.gyro_speed_ref2 ) * 0.5 * kPi / 180.0;
	double gb[3], gs[3], ao[3], as[3];
	for ( int i = 0; i < 3; i++ )
	{
		const Axis &x = axes[i];
		const int grange = std::abs( x.plus - x.bias ) + std::abs( x.minus - x.bias );
		const int arange = std::abs( x.aplus ) + std::abs( x.aminus );
		if ( grange == 0 || arange == 0 || !( speed > 0.0 ) )
			return false;
		gb[i] = x.bias;
		gs[i] = speed / ( grange * 0.5 );
		ao[i] = x.aplus - arange * 0.5;
		as[i] = 2.0 / arange * kGravity;  // g per LSB, as m/s^2
	}
	gyro_bias_ = { gb[0], gb[1], gb[2] };
	gyro_scale_ = { gs[0], gs[1], gs[2] };
	accel_bias_ = { ao[0], ao[1], ao[2] };
	accel_scale_ = { as[0], as[1], as[2] };
	return true;
}

bool SenseHid::SetPollingRate()
{
	sense::PollingRateFeatureReport r{};
	r.report_id = sense::kPollingRateFeatureReport;
	r.unk = 0x0e;
	r.rate_1 = 0x000c;
	r.rate_2 = 0x0002;
	r.crc = sense::SeededCrc32( sense::kSetFeatureCrcSeed, &r, sizeof( r ) - 4 );
	return ioctl( fd_, HIDIOCSFEATURE( sizeof( r ) ), &r ) >= 0;
}

int SenseHid::Read( SenseState &out )
{
	uint8_t buf[128];
	const ssize_t n = read( fd_, buf, sizeof( buf ) );
	if ( n < 0 )
		return ( errno == EAGAIN || errno == EINTR ) ? 0 : -1;
	if ( n == 0 )
		return -1;

	if ( buf[0] == sense::kInputReportBluetooth && n == sizeof( sense::BluetoothInputReport ) )
	{
		sense::BluetoothInputReport r;
		std::memcpy( &r, buf, sizeof( r ) );
		if ( sense::SeededCrc32( sense::kInputCrcSeed, &r, sizeof( r ) - 4 ) != r.crc )
			return 0;
		Parse( r.common, out );
		FeedSyncFit( r.common );
		return 1;
	}
	if ( buf[0] == sense::kInputReportUsb && n == sizeof( sense::UsbInputReport ) )
	{
		sense::UsbInputReport r;
		std::memcpy( &r, buf, sizeof( r ) );
		Parse( r.common, out );
		FeedSyncFit( r.common );
		return 1;
	}
	return 0;
}

void SenseHid::Parse( const sense::InputCommon &in, SenseState &s )
{
	const uint8_t *b = in.buttons;

	s.stick_x = ( in.thumbstick_x - 128 ) / 128.0f;
	s.stick_y = ( in.thumbstick_y - 128 ) / -128.0f;
	s.trigger = in.trigger_value / 255.0f;
	s.trigger_proximity = in.trigger_proximity / 255.0f;
	s.grip_proximity = in.squeeze_proximity / 255.0f;

	s.ps_click = b[1] & 0x10;
	s.trigger_touch = b[1] & 0x80;
	s.grip_touch = b[2] & 0x08;
	s.stick_touch = b[2] & 0x04;
	s.face_lower_touch = b[2] & 0x02;
	s.face_upper_touch = b[2] & 0x01;

	if ( info_.left )
	{
		s.menu_click = b[1] & 0x01;          // create
		s.face_lower_click = b[0] & 0x01;    // square
		s.face_upper_click = b[0] & 0x08;    // triangle
		s.grip_click = b[0] & 0x10;
		s.trigger_click = b[0] & 0x40;
		s.stick_click = b[1] & 0x04;
	}
	else
	{
		s.menu_click = b[1] & 0x02;          // options
		s.face_lower_click = b[0] & 0x02;    // cross
		s.face_upper_click = b[0] & 0x04;    // circle
		s.grip_click = b[0] & 0x20;
		s.trigger_click = b[0] & 0x80;
		s.stick_click = b[1] & 0x08;
	}

	// IMU: unwrap the 32-bit tick counter.
	if ( !have_ticks_ )
	{
		last_ticks_ = in.imu_ticks;
		have_ticks_ = true;
	}
	total_ticks_ += uint32_t( in.imu_ticks - last_ticks_ );
	last_ticks_ = in.imu_ticks;
	s.imu_time_s = double( total_ticks_ ) / sense::kImuTicksPerSecond;

	s.gyro = { ( in.gyro[0] - gyro_bias_.x ) * gyro_scale_.x, ( in.gyro[1] - gyro_bias_.y ) * gyro_scale_.y,
	           ( in.gyro[2] - gyro_bias_.z ) * gyro_scale_.z };
	s.accel = { ( in.accel[0] - accel_bias_.x ) * accel_scale_.x, ( in.accel[1] - accel_bias_.y ) * accel_scale_.y,
	            ( in.accel[2] - accel_bias_.z ) * accel_scale_.z };

	// Sony's mapping (docs/sony-re/sense.md 2.4): level 0..9 = 10n+5 %,
	// 10 = 100 %, 11..15 = 0 %; charge state 0..2, anything else an error.
	const uint8_t charge = in.battery_state >> 4, level = in.battery_state & 0xf;
	s.battery_valid = charge <= sense::kFull;
	s.charging = charge == sense::kCharging || charge == sense::kFull;
	s.battery = level <= 9 ? ( level * 10 + 5 ) * 0.01f : level == 10 ? 1.0f : 0.0f;
}

bool SenseHid::Write( const SenseOutput &out )
{
	sense::OutputSettings st{};

	if ( !out.has_pcm && out.rumble_amplitude > 0 )
	{
		st.flag1 |= sense::kFlag1VibrationEnable | out.rumble_mode;
		st.vibration_amplitude = out.rumble_amplitude;
	}
	if ( out.reduction )
	{
		st.flag1 |= sense::kFlag1ReductionValid;
		st.reduction = out.reduction & 7;
	}
	if ( out.trigger_dirty )
	{
		trigger_ = EncodeTriggerEffect( out.trigger );
		st.flag1 |= sense::kFlag1TriggerEnable;
	}
	st.trigger_mode = trigger_.mode;
	std::memcpy( st.trigger_params, trigger_.params.data(), sizeof( st.trigger_params ) );
	if ( led_sync_enabled_ )
	{
		FillLedSync( st, SteadyMicros() );
		// Sony's driver turns on libpad's first output switch once at
		// start and never off, so for firmware revisions above 0x253 every
		// report carries flag2 bit 1 (libpad.md "Sense output report",
		// step 8). Reports without it got no visible LED response
		// (hardware test, 2026-11-18).
		if ( fw_revision_ > 0x253 )
			st.flag2 |= 0x02;
	}
	else
		st.led.phase = sense::kLedPhaseAllOff;

	// Over Bluetooth every report goes through bluetoothd's uhid bridge and
	// shares the radio with the other controller. Sending ~94 reports/s each
	// overruns bluetoothd's socket (EAGAIN) and the link drops within a
	// minute. Sony sends a report when something changes and refreshes it
	// every 100 ms; do the same unless PCM haptics need the full rate.
	const auto now = std::chrono::steady_clock::now();
	if ( info_.bluetooth && !out.has_pcm )
	{
		if ( std::memcmp( &st, &last_bt_settings_, sizeof( st ) ) == 0 &&
		     now - last_bt_write_ < std::chrono::milliseconds( 100 ) )
			return true;
		last_bt_settings_ = st;
		last_bt_write_ = now;
	}

	// Stamped right before the write, as Sony does: the controller echoes it
	// in its input report, which the LED-sync clock fit relies on.
	st.host_timestamp_us = uint32_t(
		std::chrono::duration_cast<std::chrono::microseconds>( std::chrono::steady_clock::now().time_since_epoch() )
			.count() );

	seq_ = ( seq_ + 1 ) & 0xf;

	if ( info_.bluetooth && sony_framing_ && !out.has_pcm )
	{
		sense::SonyBluetoothOutputReport r{};
		r.report_id = sense::kOutputReportBluetooth;
		r.mode = sense::kOutputModeSense;
		r.settings = st;
		r.crc = sense::SeededCrc32( sense::kOutputCrcSeed, &r, sizeof( r ) - 4 );
		LogReport( &r, sizeof( r ), st.led.phase );
		return write( fd_, &r, sizeof( r ) ) == static_cast<ssize_t>( sizeof( r ) );
	}
	if ( info_.bluetooth )
	{
		sense::BluetoothOutputReport r{};
		r.report_id = sense::kOutputReportBluetooth;
		r.seq_no_mode = uint8_t( seq_ << 4 );
		r.tag = sense::kOutputReportTag;
		r.settings = st;
		r.counter = counter_++;
		if ( out.has_pcm )
			std::memcpy( r.pcm, out.pcm, sizeof( r.pcm ) );
		r.crc = sense::SeededCrc32( sense::kOutputCrcSeed, &r, sizeof( r ) - 4 );
		return write( fd_, &r, sizeof( r ) ) == static_cast<ssize_t>( sizeof( r ) );
	}

	sense::UsbOutputReport r{};
	r.report_id = sense::kOutputReportUsb;
	r.settings = st;
	LogReport( &r, sizeof( r ), st.led.phase );
	return write( fd_, &r, sizeof( r ) ) == static_cast<ssize_t>( sizeof( r ) );
}

// ---- LED sync -------------------------------------------------------------

// With LED sync on, log the whole output report once per LED phase change,
// to check the bytes on the wire against Sony's layout.
void SenseHid::LogReport( const void *r, size_t n, uint8_t phase )
{
	if ( !led_sync_enabled_ || phase == logged_phase_ )
		return;
	logged_phase_ = phase;
	char hex[3 * 80 + 1] = {};
	const uint8_t *b = static_cast<const uint8_t *>( r );
	for ( size_t i = 0; i < n && i < 80; i++ )
		std::snprintf( hex + 3 * i, 4, "%02x ", b[i] );
	DriverLog( "psvr2: %s: output phase %u: %s", info_.path.c_str(), phase, hex );
}

void SenseHid::FeedSyncFit( const sense::InputCommon &in )
{
	if ( !led_sync_enabled_ )
		return;
	// The controller echoes the host_timestamp_us of the last output report
	// and adds its own clock; libpad turns pairs of these into the
	// host <-> controller time map (libpad.md, "Clock fit").
	std::lock_guard<std::mutex> lock( led_mutex_ );
	last_rx_us_ = SteadyMicros();
	const uint32_t ticks = in.device_timestamp_ticks ? in.device_timestamp_ticks : in.imu_ticks;
	last_ctrl_us_ = ticks / 3 + ( ticks % 3 == 2 );
	psvr2_sync_update( &sync_, last_rx_us_, in.host_timestamp, in.device_timestamp_ticks, in.imu_ticks,
	                   info_.bluetooth ? 1 : 0 );
}

void SenseHid::LedSyncCommand( const uint8_t *cmd, uint8_t len )
{
	if ( !len || len > 11 )
		return;
	std::lock_guard<std::mutex> lock( led_mutex_ );
	psvr2_ledsync_command( &sync_, &led_, cmd, len );
}

bool SenseHid::LedTestSweep( int32_t offset_us )
{
	std::lock_guard<std::mutex> lock( led_mutex_ );
	if ( !test_anchored_ )
	{
		const uint64_t t = psvr2_sync_headset_time ? psvr2_sync_headset_time() : 0;
		if ( !t || !sync_.fit_off || psvr2_sync_host_to_ctrl( &sync_, int64_t( t ), &test_anchor_ ) )
			return false;
		test_anchored_ = true;
	}
	led_.phase = 6;
	led_.seq++;
	led_.base_time = test_anchor_ + offset_us;
	std::memset( led_.led, 0xff, 4 );
	return true;
}

SenseHid::LedSyncStatus SenseHid::LedSync()
{
	std::lock_guard<std::mutex> lock( led_mutex_ );
	LedSyncStatus s;
	s.phase = led_.phase;
	s.sync_mode = led_.sync_mode;
	s.step = led_.step;
	s.width = led_.width;
	s.clock_ready = sync_.fit_off != 0;
	int32_t c = 0;
	if ( s.clock_ready && !psvr2_sync_host_to_ctrl( &sync_, int64_t( last_rx_us_ ), &c ) )
		s.fit_err_us = int32_t( uint32_t( c ) - last_ctrl_us_ );
	s.seq = led_.seq;
	s.period_id = led_.period_id;
	s.base_time = led_.base_time;
	s.cycle_ns = led_.cycle_ns;
	return s;
}

// Capability probe (ledsync.md 2.2): Set 0x80 "80 13 22 00", then Get 0x81.
// A reply whose first byte is 0x81 means the controller supports the
// tighter sync window (mode 2).
uint8_t SenseHid::ProbeSyncMode()
{
	uint8_t r[64] = { sense::kSyncProbeSetReport, 0x13, 0x22, 0x00 };
	auto crc = [&]( uint8_t seed ) {
		const uint32_t c = sense::SeededCrc32( seed, r, sizeof( r ) - 4 );
		std::memcpy( r + sizeof( r ) - 4, &c, 4 );
	};
	if ( info_.bluetooth )
		crc( sense::kSetFeatureCrcSeed );
	if ( ioctl( fd_, HIDIOCSFEATURE( sizeof( r ) ), r ) < 0 )
		return 0;
	std::memset( r, 0, sizeof( r ) );
	r[0] = sense::kSyncProbeGetReport;
	if ( ioctl( fd_, HIDIOCGFEATURE( sizeof( r ) ), r ) <= 0 )
		return 0;
	return r[0] == sense::kSyncProbeGetReport ? 2 : 0;
}

// Same rules as libpad's output fill (psvr2_sense_output_fill): pick the
// sync mode, refresh the PRESCAN base time, then copy the record.
void SenseHid::FillLedSync( sense::OutputSettings &st, uint64_t now_us )
{
	std::lock_guard<std::mutex> lock( led_mutex_ );
	if ( led_.sync_mode == 0 )
	{
		if ( fw_version_ < 0x1006d || led_.probes > 3 )
			psvr2_ledsync_set_mode( &led_, 2 );
		else
		{
			const uint8_t m = ProbeSyncMode();
			if ( led_.sync_mode != m )
				psvr2_ledsync_set_mode( &led_, m );
			led_.probes++;
		}
	}
	if ( led_.phase && !led_.last_resync )
		led_.last_resync = now_us;
	else
		psvr2_ledsync_resync( &sync_, &led_, now_us );

	// Before the clock fit exists Sony leaves the phase byte at its
	// default for phases 1..4.
	if ( led_.phase == 0 || led_.phase > 4 || sync_.fit_off )
		st.led.phase = led_.phase;
	st.led.sequence_number = led_.seq;
	st.led.period_id = led_.period_id;
	// Sony writes both values times three (controller clock units).
	st.led.cycle_position = uint32_t( led_.base_time ) * 3;
	st.led.cycle_length = led_.cycle_ns * 3;
	if ( led_.phase == 2 || led_.phase == 3 )
		std::memcpy( st.led.led_blink, led_.led, 4 );
	else
		std::memset( st.led.led_blink, 0xff, 4 );
}
