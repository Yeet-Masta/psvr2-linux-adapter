// SPDX-License-Identifier: GPL-2.0
// Protocol handling follows Monado's pssense driver (BSL-1.0); see
// sense_protocol.h for attribution.
#include "sense_hid.h"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <linux/hidraw.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "driverlog.h"

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

SenseHid::SenseHid( const SenseHidInfo &info ) : info_( info ) {}

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
		DriverLog( "psvr2: cannot open %s: %s (udev rule installed?)", info_.path.c_str(), strerror( errno ) );
		return false;
	}
	if ( !ReadCalibration() )
	{
		close( fd_ );
		fd_ = -1;
		return false;
	}
	if ( !SetPollingRate() )
		DriverLog( "psvr2: %s: could not set PC polling rate; continuing", info_.path.c_str() );
	return true;
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

	// Scale/bias derivation as in Monado's parse_pssense_calibration.
	double ref = ( double( cal.gyro_speed_ref1 ) + double( cal.gyro_speed_ref2 ) ) * 0.5 * kPi / 180.0;
	if ( !( ref > 0.0 ) )
		ref = kPi * 3.0;

	gyro_bias_ = { double( cal.gyro_bias_x ), double( cal.gyro_bias_y ), double( cal.gyro_bias_z ) };
	auto span = []( int16_t plus, int16_t minus, double bias ) {
		const double s = ( std::fabs( plus - bias ) + std::fabs( minus - bias ) ) * 0.5;
		return s > 0.0 ? s : 1.0;
	};
	gyro_scale_ = { ref / span( cal.gyro_plus_x, cal.gyro_minus_x, gyro_bias_.x ),
	                ref / span( cal.gyro_plus_y, cal.gyro_minus_y, gyro_bias_.y ),
	                ref / span( cal.gyro_plus_z, cal.gyro_minus_z, gyro_bias_.z ) };

	accel_bias_ = { ( cal.accel_plus_x + cal.accel_minus_x ) * 0.5, ( cal.accel_plus_y + cal.accel_minus_y ) * 0.5,
	                ( cal.accel_plus_z + cal.accel_minus_z ) * 0.5 };
	auto ascale = []( int16_t plus, double bias ) {
		const double d = plus - bias;
		return d != 0.0 ? kGravity / d : 1.0;
	};
	accel_scale_ = { ascale( cal.accel_plus_x, accel_bias_.x ), ascale( cal.accel_plus_y, accel_bias_.y ),
	                 ascale( cal.accel_plus_z, accel_bias_.z ) };
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
		return 1;
	}
	if ( buf[0] == sense::kInputReportUsb && n == sizeof( sense::UsbInputReport ) )
	{
		sense::UsbInputReport r;
		std::memcpy( &r, buf, sizeof( r ) );
		Parse( r.common, out );
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

	const uint8_t charge = in.battery_state >> 4;
	s.battery_valid = charge <= sense::kFull;
	s.charging = charge == sense::kCharging || charge == sense::kFull;
	s.battery = charge == sense::kFull ? 1.0f : std::min( 1.0f, ( in.battery_state & 0xf ) * 0.1f + 0.05f );
}

bool SenseHid::Write( const SenseOutput &out )
{
	sense::OutputSettings st{};

	if ( !out.has_pcm && out.rumble_amplitude > 0 )
	{
		st.flag1 |= sense::kFlag1VibrationEnable | out.rumble_mode;
		st.vibration_amplitude = out.rumble_amplitude;
	}
	if ( out.trigger_dirty )
	{
		trigger_ = EncodeTriggerEffect( out.trigger );
		st.flag1 |= sense::kFlag1TriggerEnable;
	}
	st.trigger_mode = trigger_.mode;
	std::memcpy( st.trigger_params, trigger_.params.data(), sizeof( st.trigger_params ) );
	st.led.phase = sense::kLedPhaseAllOff;
	st.host_timestamp_us = uint32_t(
		std::chrono::duration_cast<std::chrono::microseconds>( std::chrono::steady_clock::now().time_since_epoch() )
			.count() );

	seq_ = ( seq_ + 1 ) & 0xf;

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
	r.seq_no_mode = uint8_t( ( seq_ << 4 ) | 0x2 );
	r.settings = st;
	return write( fd_, &r, sizeof( r ) ) == static_cast<ssize_t>( sizeof( r ) );
}
