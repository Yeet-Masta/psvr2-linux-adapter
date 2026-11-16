// SPDX-License-Identifier: BSL-1.0
// PS VR2 Sense controller HID wire format.
//
// Layouts follow Monado's pssense_protocol.h (Copyright 2023 Collabora, Ltd.,
// 2023 Jarett Millard, 2026 Beyley Cardellio; Boost Software License 1.0,
// https://www.boost.org/LICENSE_1_0.txt), which in turn credits PSVR2 Toolkit
// (MIT) for the PS5-style output report. All multi-byte fields are
// little-endian; this driver only builds for little-endian hosts.
#pragma once

#include <cstdint>

namespace sense
{

constexpr uint16_t kSonyVendorId = 0x054c;
constexpr uint16_t kLeftProductId = 0x0e45;
constexpr uint16_t kRightProductId = 0x0e46;

constexpr uint8_t kInputReportUsb = 0x01;
constexpr uint8_t kInputReportBluetooth = 0x31;
constexpr uint8_t kOutputReportBluetooth = 0x31;
constexpr uint8_t kOutputReportTag = 0x10;
constexpr uint8_t kCalibrationFeatureReport = 0x05;
constexpr uint8_t kPollingRateFeatureReport = 0x08;
constexpr uint8_t kCalibrationPart1 = 0x00;
constexpr uint8_t kCalibrationPart2 = 0x81;

// CRC32 seeds prepended to Bluetooth reports.
constexpr uint8_t kInputCrcSeed = 0xa1;
constexpr uint8_t kOutputCrcSeed = 0xa2;
constexpr uint8_t kFeatureCrcSeed = 0xa3;
constexpr uint8_t kSetFeatureCrcSeed = 0x53;

// PCM haptics: 32 signed 8-bit samples per output report at 3 kHz.
constexpr int kPcmSampleRate = 3000;
constexpr int kPcmSamplesPerReport = 32;

// IMU clock: one tick is a third of a microsecond.
constexpr double kImuTicksPerSecond = 3.0e6;

enum ChargeState : uint8_t
{
	kDischarging = 0x0,
	kCharging = 0x1,
	kFull = 0x2,
};

// output_settings.flag1
constexpr uint8_t kFlag1VibrationEnable = 0x03;  // UNK0 | RUMBLE_EMULATION
constexpr uint8_t kFlag1TriggerEnable = 0x04;
constexpr uint8_t kFlag1VibrationHigh120Hz = 0x00;
constexpr uint8_t kFlag1VibrationLow60Hz = 0x20;
constexpr uint8_t kFlag1VibrationClassic = 0x40;

// LED sync phase used while no camera tracks the LEDs.
constexpr uint8_t kLedPhaseAllOff = 5;

#pragma pack( push, 1 )

struct InputCommon
{
	uint8_t thumbstick_x;
	uint8_t thumbstick_y;
	uint8_t trigger_value;
	uint8_t trigger_proximity;
	uint8_t squeeze_proximity;
	uint8_t unknown1[2];
	uint8_t buttons[3];
	uint8_t unknown2;
	uint32_t seq_no;
	int16_t gyro[3];
	int16_t accel[3];
	uint32_t imu_ticks;
	uint8_t temperature;
	uint8_t unknown3[7];
	uint8_t trigger_feedback_state;
	uint8_t trigger_feedback_mode;
	uint8_t battery_state;  // low nibble level 0..10, high nibble ChargeState
	uint8_t plug_state;
	uint32_t host_timestamp;
	uint32_t device_timestamp_ticks;
	uint8_t unknown4[4];
	uint8_t aes_cmac[8];
};

struct UsbInputReport
{
	uint8_t report_id;
	InputCommon common;
};
static_assert( sizeof( UsbInputReport ) == 64, "USB input report size" );

struct BluetoothInputReport
{
	uint8_t report_id;
	uint8_t bt_header;
	InputCommon common;
	uint8_t unknown5;
	uint8_t crc_failure_count;
	uint8_t padding[7];
	uint32_t crc;
};
static_assert( sizeof( BluetoothInputReport ) == 78, "BT input report size" );

struct LedSettings
{
	uint8_t phase;
	uint8_t sequence_number;
	uint8_t period_id;
	uint32_t cycle_position;
	uint32_t cycle_length;
	uint8_t led_blink[4];
};

struct OutputSettings
{
	uint8_t flag1;
	uint8_t flag2;
	uint8_t vibration_amplitude;  // 0 = off
	uint8_t unk0;
	uint8_t trigger_mode;         // see trigger_effect.h
	uint8_t trigger_params[10];
	uint32_t host_timestamp_us;
	LedSettings led;
	uint8_t reduction;            // low nibble haptics, high nibble trigger; 12.5% steps
	uint8_t status_led_enable;
	uint8_t unk1[2];
};
static_assert( sizeof( OutputSettings ) == 38, "output settings size" );

struct BluetoothOutputReport
{
	uint8_t report_id;
	uint8_t seq_no_mode;  // seq << 4; low bits 0 = PS5 layout
	uint8_t tag;
	OutputSettings settings;
	uint8_t counter;      // must increment for PCM haptics to play
	int8_t pcm[kPcmSamplesPerReport];
	uint32_t crc;
};
static_assert( sizeof( BluetoothOutputReport ) == 78, "BT output report size" );

struct UsbOutputReport
{
	uint8_t seq_no_mode;  // seq << 4 | 0x2
	OutputSettings settings;
};
static_assert( sizeof( UsbOutputReport ) == 39, "USB output report size" );

struct FeatureReport
{
	uint8_t report_id;
	uint8_t part_id;
	uint8_t data[58];
	uint32_t crc;
};
static_assert( sizeof( FeatureReport ) == 64, "feature report size" );

struct CalibrationData
{
	int16_t accel_plus_x;
	uint8_t pad0[4];
	int16_t accel_minus_x;
	uint8_t pad1[6];
	int16_t accel_plus_y;
	uint8_t pad2[4];
	int16_t accel_minus_y;
	uint8_t pad3[6];
	int16_t accel_plus_z;
	uint8_t pad4[4];
	int16_t accel_minus_z;
	int16_t gyro_bias_x;
	int16_t gyro_bias_y;
	int16_t gyro_bias_z;
	int16_t gyro_plus_x;
	uint8_t pad6[4];
	int16_t gyro_minus_x;
	uint8_t pad7[6];
	int16_t gyro_plus_y;
	uint8_t pad8[4];
	int16_t gyro_minus_y;
	uint8_t pad9[6];
	int16_t gyro_plus_z;
	uint8_t pad10[4];
	int16_t gyro_minus_z;
	int16_t gyro_speed_ref1;
	int16_t gyro_speed_ref2;
	uint8_t pad_end[34];
};
static_assert( sizeof( CalibrationData ) == 116, "calibration size" );

struct PollingRateFeatureReport
{
	uint8_t report_id;
	uint8_t unk;
	uint16_t rate_1;
	uint16_t rate_2;
	uint8_t padding[38];
	uint32_t crc;
};
static_assert( sizeof( PollingRateFeatureReport ) == 48, "polling report size" );

#pragma pack( pop )

// Reflected CRC-32 (poly 0xedb88320) as used by DualSense / Sense reports.
inline uint32_t Crc32( uint32_t crc, const uint8_t *p, size_t len )
{
	crc = ~crc;
	while ( len-- )
	{
		crc ^= *p++;
		for ( int i = 0; i < 8; i++ )
			crc = ( crc >> 1 ) ^ ( ( crc & 1 ) ? 0xedb88320u : 0 );
	}
	return ~crc;
}

inline uint32_t SeededCrc32( uint8_t seed, const void *data, size_t len )
{
	return Crc32( Crc32( 0, &seed, 1 ), static_cast<const uint8_t *>( data ), len );
}

} // namespace sense
