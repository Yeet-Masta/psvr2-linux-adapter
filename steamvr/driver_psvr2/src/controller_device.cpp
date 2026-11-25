// SPDX-License-Identifier: GPL-2.0
#include "controller_device.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <poll.h>

#include "driverlog.h"
#include "hand_simulation.h"
#include "hmd_device_driver.h"

namespace
{
using Clock = std::chrono::steady_clock;
constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr auto kOutputPeriod = std::chrono::microseconds( 10667 );  // 32 PCM samples at 3 kHz
constexpr auto kInputTimeout = std::chrono::milliseconds( 1000 );
constexpr auto kRecenterHold = std::chrono::milliseconds( 1000 );
constexpr auto kCalibrateHold = std::chrono::milliseconds( 3000 );
constexpr auto kSkeletonPeriod = std::chrono::milliseconds( 10 );
constexpr auto kWearRecenterWindow = std::chrono::seconds( 8 );
constexpr const char *kSection = "driver_psvr2";

float GetFloatSetting( const char *key, float fallback )
{
	vr::EVRSettingsError err = vr::VRSettingsError_None;
	const float v = vr::VRSettings()->GetFloat( kSection, key, &err );
	return err == vr::VRSettingsError_None ? v : fallback;
}

int GetIntSetting( const char *key, int fallback )
{
	vr::EVRSettingsError err = vr::VRSettingsError_None;
	const int v = vr::VRSettings()->GetInt32( kSection, key, &err );
	return err == vr::VRSettingsError_None ? v : fallback;
}

bool GetBoolSetting( const char *key, bool fallback )
{
	vr::EVRSettingsError err = vr::VRSettingsError_None;
	const bool v = vr::VRSettings()->GetBool( kSection, key, &err );
	return err == vr::VRSettingsError_None ? v : fallback;
}

// default_trigger_effect: off | light | medium | heavy | click | rigid.
TriggerEffect DefaultTriggerEffect()
{
	char name[32] = "off";
	vr::EVRSettingsError err = vr::VRSettingsError_None;
	vr::VRSettings()->GetString( kSection, "default_trigger_effect", name, sizeof( name ), &err );
	TriggerEffect fx;
	const std::string n = err == vr::VRSettingsError_None ? name : "off";
	if ( n == "light" || n == "medium" || n == "heavy" )
	{
		fx.kind = TriggerEffectKind::Feedback;
		fx.start = 1;
		fx.strength = n == "light" ? 2 : n == "medium" ? 4 : 6;
	}
	else if ( n == "click" )
	{
		// A soft wall partway down that gives way, like a two-stage trigger.
		fx.kind = TriggerEffectKind::Weapon;
		fx.start = 4;
		fx.end = 6;
		fx.strength = 5;
	}
	else if ( n == "rigid" )
	{
		fx.kind = TriggerEffectKind::Feedback;
		fx.start = 0;
		fx.strength = 8;
	}
	else if ( n != "off" )
		DriverLog( "psvr2: unknown default_trigger_effect \"%s\", using off", n.c_str() );
	return fx;
}

vr::HmdQuaternion_t RotX( double deg )
{
	return vrmath::AxisAngle( 1, 0, 0, deg * kDeg );
}

// Grip orientation in the IMU frame. The IMU is mounted tilted; Monado's
// pssense driver rotates the fused orientation up 60 degrees about X when it
// runs IMU-only, and so do we. The per-controller calibration (hold the menu
// button for three seconds) corrects whatever is left.
const vr::HmdQuaternion_t kImuToGrip = RotX( 60.0 );

// Arm model, headset-yaw frame: shoulder below/beside the head, elbow hangs
// below the shoulder, forearm follows the controller orientation.
constexpr vrmath::Vec3 kShoulder{ 0.17, -0.28, 0.05 };  // x mirrored for the left hand
constexpr vrmath::Vec3 kUpperArm{ 0.0, -0.25, 0.0 };
constexpr vrmath::Vec3 kForearm{ 0.0, 0.0, -0.30 };

void SetBool( vr::VRInputComponentHandle_t h, bool v )
{
	if ( h )
		vr::VRDriverInput()->UpdateBooleanComponent( h, v, 0 );
}

void SetScalar( vr::VRInputComponentHandle_t h, float v )
{
	if ( h )
		vr::VRDriverInput()->UpdateScalarComponent( h, v, 0 );
}
} // namespace

Psvr2ControllerDriver::Psvr2ControllerDriver( bool left, Psvr2HmdDriver *hmd )
	: left_( left ), hmd_( hmd ), serial_( left ? "PSVR2-SENSE-L" : "PSVR2-SENSE-R" )
{
	LoadSettings();
	LoadCalibration();

	pose_.qWorldFromDriverRotation.w = 1.0;
	pose_.qDriverFromHeadRotation.w = 1.0;
	pose_.qRotation.w = 1.0;
	pose_.result = vr::TrackingResult_Uninitialized;
}

Psvr2ControllerDriver::~Psvr2ControllerDriver()
{
	Deactivate();
}

vr::EVRInitError Psvr2ControllerDriver::Activate( uint32_t unObjectId )
{
	device_index_ = unObjectId;
	container_ = vr::VRProperties()->TrackedDeviceToPropertyContainer( unObjectId );
	auto *props = vr::VRProperties();
	const auto c = container_;

	props->SetStringProperty( c, vr::Prop_ManufacturerName_String, "Sony" );
	props->SetStringProperty( c, vr::Prop_TrackingSystemName_String, "psvr2" );
	props->SetStringProperty( c, vr::Prop_ModelNumber_String,
	                          left_ ? "PS VR2 Sense Controller (L)" : "PS VR2 Sense Controller (R)" );
	props->SetStringProperty( c, vr::Prop_RenderModelName_String,
	                          left_ ? "{psvr2}/rendermodels/psvr2_sense_left" : "{psvr2}/rendermodels/psvr2_sense_right" );
	props->SetStringProperty( c, vr::Prop_InputProfilePath_String, "{psvr2}/input/psvr2_sense_profile.json" );
	props->SetStringProperty( c, vr::Prop_ControllerType_String, "psvr2_sense" );
	props->SetInt32Property( c, vr::Prop_ControllerRoleHint_Int32,
	                         left_ ? vr::TrackedControllerRole_LeftHand : vr::TrackedControllerRole_RightHand );
	props->SetInt32Property( c, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller );
	props->SetBoolProperty( c, vr::Prop_DeviceIsWireless_Bool, true );
	props->SetBoolProperty( c, vr::Prop_DeviceProvidesBatteryStatus_Bool, true );
	props->SetBoolProperty( c, vr::Prop_DeviceCanPowerOff_Bool, false );

	auto *in = vr::VRDriverInput();
	const auto oneSided = vr::VRScalarUnits_NormalizedOneSided;
	const auto twoSided = vr::VRScalarUnits_NormalizedTwoSided;
	in->CreateBooleanComponent( c, "/input/system/click", &system_click_ );
	in->CreateBooleanComponent( c, "/input/application_menu/click", &menu_click_ );
	// Oculus Touch naming (X/Y left, A/B right) so SteamVR can reuse Touch bindings.
	in->CreateBooleanComponent( c, left_ ? "/input/x/click" : "/input/a/click", &lower_click_ );
	in->CreateBooleanComponent( c, left_ ? "/input/x/touch" : "/input/a/touch", &lower_touch_ );
	in->CreateBooleanComponent( c, left_ ? "/input/y/click" : "/input/b/click", &upper_click_ );
	in->CreateBooleanComponent( c, left_ ? "/input/y/touch" : "/input/b/touch", &upper_touch_ );
	in->CreateScalarComponent( c, "/input/trigger/value", &trigger_value_, vr::VRScalarType_Absolute, oneSided );
	in->CreateBooleanComponent( c, "/input/trigger/click", &trigger_click_ );
	in->CreateBooleanComponent( c, "/input/trigger/touch", &trigger_touch_ );
	in->CreateScalarComponent( c, "/input/trigger/proximity", &trigger_prox_, vr::VRScalarType_Absolute, oneSided );
	in->CreateScalarComponent( c, "/input/grip/value", &grip_value_, vr::VRScalarType_Absolute, oneSided );
	in->CreateBooleanComponent( c, "/input/grip/click", &grip_click_ );
	in->CreateBooleanComponent( c, "/input/grip/touch", &grip_touch_ );
	in->CreateScalarComponent( c, "/input/joystick/x", &stick_x_, vr::VRScalarType_Absolute, twoSided );
	in->CreateScalarComponent( c, "/input/joystick/y", &stick_y_, vr::VRScalarType_Absolute, twoSided );
	in->CreateBooleanComponent( c, "/input/joystick/click", &stick_click_ );
	in->CreateBooleanComponent( c, "/input/joystick/touch", &stick_touch_ );
	in->CreateHapticComponent( c, "/output/haptic", &haptic_ );
	// Finger poses from the capacitive sensors. Partial: curls are inferred
	// from touch and proximity, not measured per joint.
	in->CreateSkeletonComponent( c, left_ ? "/input/skeleton/left" : "/input/skeleton/right",
	                             left_ ? "/skeleton/hand/left" : "/skeleton/hand/right", "/pose/raw",
	                             vr::VRSkeletalTracking_Partial, nullptr, 0, &skeleton_ );

	running_ = true;
	io_thread_ = std::thread( &Psvr2ControllerDriver::IoThread, this );
	DriverLog( "psvr2: %s Sense controller registered", left_ ? "left" : "right" );
	return vr::VRInitError_None;
}

void Psvr2ControllerDriver::Deactivate()
{
	if ( running_.exchange( false ) && io_thread_.joinable() )
	{
		io_thread_.join();
		SaveGyroBias();
	}
	device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void Psvr2ControllerDriver::EnterStandby() {}

void *Psvr2ControllerDriver::GetComponent( const char * )
{
	return nullptr;
}

void Psvr2ControllerDriver::DebugRequest( const char *, char *pchResponseBuffer, uint32_t unResponseBufferSize )
{
	if ( unResponseBufferSize )
		pchResponseBuffer[0] = 0;
}

vr::DriverPose_t Psvr2ControllerDriver::GetPose()
{
	std::lock_guard<std::mutex> lock( pose_mutex_ );
	return pose_;
}

void Psvr2ControllerDriver::Attach( const SenseHidInfo &info )
{
	if ( attached_ )
		return;
	auto hid = std::make_unique<SenseHid>( info );
	if ( !hid->Open() )
		return;
	{
		std::lock_guard<std::mutex> lock( hid_mutex_ );
		hid_ = std::move( hid );
		fusion_ = ImuFusion();
		last_imu_time_ = -1.0;
		last_input_ = Clock::now();
		last_pose_time_ = -1.0;
		yaw_anchored_ = false;
		bias_saved_ = false;
		vrmath::Vec3 b;
		const char axes[3] = { 'x', 'y', 'z' };
		double *v[3] = { &b.x, &b.y, &b.z };
		bool have = true;
		for ( int i = 0; i < 3 && have; i++ )
		{
			vr::EVRSettingsError err = vr::VRSettingsError_None;
			*v[i] = vr::VRSettings()->GetFloat( kSection, SettingKey( ( std::string( "gyro_bias_" ) + axes[i] ).c_str() ).c_str(), &err );
			have = err == vr::VRSettingsError_None;
		}
		if ( have )
			fusion_.SetBias( b );
	}
	{
		std::lock_guard<std::mutex> lock( out_mutex_ );
		trigger_dirty_ = true;  // re-apply the current effect on reconnect
	}
	props_dirty_ = true;
	attached_ = true;
	DriverLog( "psvr2: %s Sense controller on %s (%s)", left_ ? "left" : "right", info.path.c_str(),
	           info.bluetooth ? "Bluetooth" : "USB" );
}

void Psvr2ControllerDriver::Disconnected()
{
	SaveGyroBias();
	if ( battery_alerts_ && hmd_ )
		hmd_->RequestAlert();
	{
		std::lock_guard<std::mutex> lock( hid_mutex_ );
		hid_.reset();
	}
	attached_ = false;
	// As Sony: no battery status while the controller is away.
	if ( container_ != vr::k_ulInvalidPropertyContainer )
	{
		vr::VRProperties()->SetBoolProperty( container_, vr::Prop_DeviceProvidesBatteryStatus_Bool, false );
		vr::VRProperties()->SetFloatProperty( container_, vr::Prop_DeviceBatteryPercentage_Float, 0.0f );
		battery_ = -1.0f;
	}
	{
		std::lock_guard<std::mutex> lock( pose_mutex_ );
		pose_.deviceIsConnected = false;
		pose_.poseIsValid = false;
		pose_.result = vr::TrackingResult_Uninitialized;
	}
	if ( device_index_ != vr::k_unTrackedDeviceIndexInvalid )
	{
		const vr::DriverPose_t p = GetPose();
		vr::VRServerDriverHost()->TrackedDevicePoseUpdated( device_index_, p, sizeof( p ) );
	}
	DriverLog( "psvr2: %s Sense controller disconnected", left_ ? "left" : "right" );
}

void Psvr2ControllerDriver::OnHaptic( const vr::VREvent_HapticVibration_t &h )
{
	// Thresholds as Sony's driver applies them (docs/sony-re/controllers.md §B.5.1):
	// a zero amplitude or frequency stops the pulse, but no earlier than
	// 80 ms after it started; events weaker than 0.049 are ignored; every
	// pulse lasts at least 80 ms.
	constexpr auto kMinPulse = std::chrono::milliseconds( 80 );
	std::lock_guard<std::mutex> lock( out_mutex_ );
	const auto now = Clock::now();
	if ( !( h.fAmplitude > 0.0f ) || !( h.fFrequency > 0.0f ) )
	{
		if ( now < haptic_end_ )
			haptic_end_ = std::max( now, haptic_start_ + kMinPulse );
		return;
	}
	if ( h.fAmplitude < 0.049f )
		return;
	haptic_amplitude_ = std::min( h.fAmplitude, 1.0f );
	haptic_frequency_ = h.fFrequency;
	const double secs = std::max( 0.08, double( h.fDurationSeconds ) );
	haptic_start_ = now;
	haptic_end_ = now + std::chrono::duration_cast<Clock::duration>( std::chrono::duration<double>( secs ) );
}

void Psvr2ControllerDriver::SetTriggerEffect( const TriggerEffect *fx )
{
	std::lock_guard<std::mutex> lock( out_mutex_ );
	app_trigger_ = fx != nullptr;
	const TriggerEffect &want = fx ? *fx : default_trigger_;
	if ( want != trigger_ )
	{
		trigger_ = want;
		trigger_dirty_ = true;
	}
}

void Psvr2ControllerDriver::Recenter()
{
	recenter_requested_ = true;
}

void Psvr2ControllerDriver::Buzz( std::initializer_list<double> pattern )
{
	std::lock_guard<std::mutex> lock( out_mutex_ );
	auto t = Clock::now();
	bool on = true;
	for ( double secs : pattern )
	{
		const auto end = t + std::chrono::duration_cast<Clock::duration>( std::chrono::duration<double>( secs ) );
		if ( on )
			buzz_.push_back( { t, end } );
		t = end;
		on = !on;
	}
}

std::string Psvr2ControllerDriver::SettingKey( const char *name ) const
{
	return std::string( left_ ? "left_" : "right_" ) + name;
}

void Psvr2ControllerDriver::LoadSettings()
{
	enhanced_haptics_ = GetBoolSetting( "enhanced_haptics", true );
	prediction_s_ = std::clamp( GetFloatSetting( "controller_prediction_ms", 12.0f ), 0.0f, 50.0f ) / 1000.0;
	recenter_on_wear_ = GetBoolSetting( "recenter_on_wear", true );
	battery_alerts_ = GetBoolSetting( "controller_alerts", true );
	// 0 off, 1 strong (default), 2 medium, 3 weak.
	vibration_strength_ = std::clamp( GetIntSetting( "vibration_strength", 1 ), 0, 3 );
	SenseHid::SetSonyFraming( GetBoolSetting( "sony_bt_framing", true ) );
	SenseHid::SetLedSync( GetBoolSetting( "optical_tracking", false ) );
	ukf_enabled_ = GetBoolSetting( "optical_ukf", true );
	ukf_hold_us_ = uint64_t( std::clamp( GetFloatSetting( "optical_ukf_hold_ms", 300.0f ), 0.0f, 5000.0f ) * 1000.0f );
	ukf_.SetRotSigmaFloor( std::clamp( GetFloatSetting( "optical_ukf_rot_sigma_mm", 1.0f ), 0.01f, 100.0f ) * 1e-3 );
	ukf_.SetPosSigmaFloor( std::clamp( GetFloatSetting( "optical_ukf_pos_sigma_mm", 0.0f ), 0.0f, 100.0f ) * 1e-3 );
	default_trigger_ = DefaultTriggerEffect();
	trigger_ = default_trigger_;
	trigger_dirty_ = true;
}

void Psvr2ControllerDriver::SaveGyroBias()
{
	if ( !fusion_.Settled() )
		return;
	const vrmath::Vec3 &b = fusion_.Bias();
	const double v[3] = { b.x, b.y, b.z };
	const char axes[3] = { 'x', 'y', 'z' };
	for ( int i = 0; i < 3; i++ )
		vr::VRSettings()->SetFloat( kSection, SettingKey( ( std::string( "gyro_bias_" ) + axes[i] ).c_str() ).c_str(),
		                            float( v[i] ) );
}

std::string Psvr2ControllerDriver::CalibrationKey( char axis ) const
{
	return SettingKey( "calibration_" ) + axis;
}

void Psvr2ControllerDriver::LoadCalibration()
{
	double q[4];
	const char axes[4] = { 'w', 'x', 'y', 'z' };
	for ( int i = 0; i < 4; i++ )
	{
		vr::EVRSettingsError err = vr::VRSettingsError_None;
		q[i] = vr::VRSettings()->GetFloat( kSection, CalibrationKey( axes[i] ).c_str(), &err );
		if ( err != vr::VRSettingsError_None )
			return;
	}
	const vr::HmdQuaternion_t c{ q[0], q[1], q[2], q[3] };
	if ( c.w * c.w + c.x * c.x + c.y * c.y + c.z * c.z > 0.5 )
		calibration_ = vrmath::Normalize( c );
}

void Psvr2ControllerDriver::SaveCalibration()
{
	const double q[4] = { calibration_.w, calibration_.x, calibration_.y, calibration_.z };
	const char axes[4] = { 'w', 'x', 'y', 'z' };
	for ( int i = 0; i < 4; i++ )
		vr::VRSettings()->SetFloat( kSection, CalibrationKey( axes[i] ).c_str(), float( q[i] ) );
}

void Psvr2ControllerDriver::UpdateBattery( const SenseState &s )
{
	if ( hmd_ && container_ != vr::k_ulInvalidPropertyContainer && hmd_->UniverseId() != universe_set_ )
	{
		universe_set_ = hmd_->UniverseId();
		vr::VRProperties()->SetUint64Property( container_, vr::Prop_CurrentUniverseId_Uint64, universe_set_ );
	}
	if ( props_dirty_ && container_ != vr::k_ulInvalidPropertyContainer )
	{
		props_dirty_ = false;
		uint16_t fw = 0, hw = 0;
		{
			std::lock_guard<std::mutex> lock( hid_mutex_ );
			if ( hid_ )
			{
				fw = hid_->FirmwareRevision();
				hw = hid_->HardwareRevision();
			}
		}
		char buf[16];
		snprintf( buf, sizeof( buf ), "%04x", fw );
		vr::VRProperties()->SetStringProperty( container_, vr::Prop_TrackingFirmwareVersion_String, fw ? buf : "Unknown" );
		if ( hw )
		{
			snprintf( buf, sizeof( buf ), "%04x", hw );
			vr::VRProperties()->SetStringProperty( container_, vr::Prop_HardwareRevision_String, buf );
		}
		vr::VRProperties()->SetBoolProperty( container_, vr::Prop_DeviceProvidesBatteryStatus_Bool, true );
	}
	if ( !s.battery_valid )
		return;
	if ( std::fabs( s.battery - battery_ ) > 0.01f || s.charging != charging_ )
	{
		battery_ = s.battery;
		charging_ = s.charging;
		vr::VRProperties()->SetFloatProperty( container_, vr::Prop_DeviceBatteryPercentage_Float, battery_ );
		vr::VRProperties()->SetBoolProperty( container_, vr::Prop_DeviceIsCharging_Bool, charging_ );
	}
	// Three short buzzes below 15 %, five below 5 %; re-arm once charged.
	if ( charging_ || battery_ > 0.2f )
		alerted_battery_ = 1.0f;
	else if ( battery_alerts_ && battery_ < 0.05f && alerted_battery_ > 0.05f )
	{
		alerted_battery_ = 0.05f;
		Buzz( { 0.12, 0.1, 0.12, 0.1, 0.12, 0.1, 0.12, 0.1, 0.12 } );
		DriverLog( "psvr2: %s controller battery critical", left_ ? "left" : "right" );
	}
	else if ( battery_alerts_ && battery_ < 0.15f && alerted_battery_ > 0.15f )
	{
		alerted_battery_ = 0.15f;
		Buzz( { 0.15, 0.12, 0.15, 0.12, 0.15 } );
		DriverLog( "psvr2: %s controller battery low", left_ ? "left" : "right" );
	}
}

void Psvr2ControllerDriver::UpdateSkeleton( const SenseState &s )
{
	if ( !skeleton_ )
		return;
	// Curl targets from the capacitive sensors: a resting finger curls
	// partway, a pressed one fully, a lifted one opens up.
	const bool thumb_down = s.face_lower_touch || s.face_upper_touch || s.stick_touch || s.face_lower_click ||
	                        s.face_upper_click || s.stick_click;
	const bool thumb_press = s.face_lower_click || s.face_upper_click || s.stick_click;
	const float target[5] = {
		thumb_press ? 1.0f : thumb_down ? 0.7f : 0.0f,
		s.trigger_touch || s.trigger > 0.0f ? 0.45f + 0.55f * s.trigger : 0.3f * s.trigger_proximity,
		s.grip_click ? 1.0f : s.grip_touch ? 0.85f : 0.8f * s.grip_proximity,
		0.0f,
		0.0f,
	};
	for ( int i = 0; i < 3; i++ )
		curls_[i] += 0.35f * ( target[i] - curls_[i] );
	// One grip sensor covers the last three fingers.
	curls_[3] = curls_[4] = curls_[2];

	const auto now = Clock::now();
	if ( now < next_skeleton_ )
		return;
	next_skeleton_ = now + kSkeletonPeriod;

	static MyHandSimulation sim;
	vr::VRBoneTransform_t bones[eBone_Count];
	const vr::ETrackedControllerRole role = left_ ? vr::TrackedControllerRole_LeftHand : vr::TrackedControllerRole_RightHand;
	sim.ComputeSkeletonTransforms( role, { curls_[0], curls_[1], curls_[2], curls_[3], curls_[4] }, {}, bones );
	vr::VRDriverInput()->UpdateSkeletonComponent( skeleton_, vr::VRSkeletalMotionRange_WithController, bones, eBone_Count );
	// Without the controller in the way the hand can close all the way.
	sim.ComputeSkeletonTransforms( role,
	                               { std::min( 1.0f, curls_[0] * 1.2f ), std::min( 1.0f, curls_[1] * 1.2f ),
	                                 std::min( 1.0f, curls_[2] * 1.2f ), std::min( 1.0f, curls_[3] * 1.2f ),
	                                 std::min( 1.0f, curls_[4] * 1.2f ) },
	                               {}, bones );
	vr::VRDriverInput()->UpdateSkeletonComponent( skeleton_, vr::VRSkeletalMotionRange_WithoutController, bones,
	                                              eBone_Count );
}

void Psvr2ControllerDriver::FillOutput( SenseOutput &out )
{
	std::lock_guard<std::mutex> lock( out_mutex_ );
	out.trigger = trigger_;
	out.trigger_dirty = trigger_dirty_;
	trigger_dirty_ = false;

	const auto now = Clock::now();
	buzz_.erase( std::remove_if( buzz_.begin(), buzz_.end(), [&]( const BuzzSegment &b ) { return b.end <= now; } ),
	             buzz_.end() );
	const bool buzzing =
		std::any_of( buzz_.begin(), buzz_.end(), [&]( const BuzzSegment &b ) { return b.start <= now; } );
	const bool active = buzzing || ( haptic_amplitude_ > 0.0f && now < haptic_end_ );
	out.rumble_amplitude = 0;
	out.has_pcm = false;
	if ( !active )
	{
		pcm_phase_ = 0.0;
		return;
	}
	// Driver alerts win over game haptics. Vibration strength (Sony's
	// setting): off drops game haptics, medium/weak reduce them in 12.5 % steps.
	if ( !buzzing && vibration_strength_ == 0 )
		return;
	const uint8_t reduction = buzzing ? 0 : vibration_strength_ == 2 ? 2 : vibration_strength_ == 3 ? 3 : 0;
	out.reduction = reduction;
	const float amplitude = buzzing ? 0.7f : haptic_amplitude_;
	const float frequency = buzzing ? 160.0f : haptic_frequency_;

	if ( enhanced_haptics_ )
	{
		// Synthesize the requested tone as PCM for the voice-coil actuator.
		const double hz = std::clamp( frequency > 0.0f ? double( frequency ) : 160.0, 20.0, 1000.0 );
		const double step = 2.0 * 3.14159265358979323846 * hz / sense::kPcmSampleRate;
		for ( int i = 0; i < sense::kPcmSamplesPerReport; i++ )
		{
			out.pcm[i] = int8_t( std::lround( 127.0 * amplitude * ( 1.0 - 0.125 * reduction ) * std::sin( pcm_phase_ ) ) );
			pcm_phase_ = std::fmod( pcm_phase_ + step, 2.0 * 3.14159265358979323846 );
		}
		out.has_pcm = true;
	}
	else
	{
		// Sony: amplitude byte ceil(a * 20), frequency ignored, flag1
		// 0x40 (vibration valid) | 0x03 (mode 2).
		out.rumble_amplitude = uint8_t( std::ceil( amplitude * sense::kVibrationMax ) );
		out.rumble_mode = sense::kFlag1VibrationValid;
		(void)frequency;
	}
}

void Psvr2ControllerDriver::FeedUkf( const SenseState &s, double dt )
{
	// IMU sample time on the host clock: the controller clock's steps,
	// pulled slowly toward arrival time minus the Bluetooth latency (the
	// same latency the pose prediction assumes). The optical frames are
	// stamped on the host clock already.
	const uint64_t now_us = uint64_t(
		std::chrono::duration_cast<std::chrono::microseconds>( Clock::now().time_since_epoch() ).count() );
	const double target = double( now_us ) - prediction_s_ * 1e6;
	double est = double( imu_host_us_ ) + dt * 1e6;
	if ( !imu_host_us_ || dt <= 0.0 || std::fabs( est - target ) > 50000.0 )
		est = target;
	else
		est += ( target - est ) * 0.01;
	imu_host_us_ = std::max<uint64_t>( imu_host_us_ + 1, uint64_t( est ) );
	ukf_.SetGyroBiasHint( fusion_.Bias() );
	ukf_.AddImu( imu_host_us_, s.gyro, s.accel );

	Optical o;
	{
		std::lock_guard<std::mutex> lock( optical_mutex_ );
		o = optical_;
	}
	if ( !o.have || o.seq == ukf_seq_used_ )
		return;
	ukf_seq_used_ = o.seq;
	ControllerUkf::Fix f;
	f.us = o.us;
	f.pos = o.pos;
	f.rot = o.rot;
	f.head_rel = o.head_rel;
	f.cam_dist = o.cam_dist;
	const ControllerUkf::Result r = ukf_.AddFix( f );
	if ( r == ControllerUkf::kInitialized )
		DriverLog( "psvr2: %s controller: optical filter (re)started", left_ ? "left" : "right" );
}

void Psvr2ControllerDriver::HandleState( const SenseState &s )
{
	// --- orientation ---
	const double dt = last_imu_time_ < 0.0 ? 0.0 : s.imu_time_s - last_imu_time_;
	last_imu_time_ = s.imu_time_s;
	fusion_.Update( s.accel, s.gyro, dt );
	{
		std::lock_guard<std::mutex> lock( imu_mutex_ );
		imu_q_ = fusion_.Orientation();
		imu_valid_ = fusion_.Initialized();
	}
	if ( ukf_enabled_ )
		FeedUkf( s, dt );

	// --- inputs ---
	SetBool( system_click_, s.ps_click );
	SetBool( menu_click_, s.menu_click );
	SetBool( lower_click_, s.face_lower_click );
	SetBool( lower_touch_, s.face_lower_touch || s.face_lower_click );
	SetBool( upper_click_, s.face_upper_click );
	SetBool( upper_touch_, s.face_upper_touch || s.face_upper_click );
	SetScalar( trigger_value_, s.trigger );
	SetBool( trigger_click_, s.trigger_click || s.trigger > 0.95f );
	SetBool( trigger_touch_, s.trigger_touch || s.trigger > 0.0f );
	SetScalar( trigger_prox_, s.trigger_proximity );
	// The grip is a button; report its capacitive proximity as the analog value.
	SetScalar( grip_value_, s.grip_click ? 1.0f : s.grip_proximity );
	SetBool( grip_click_, s.grip_click );
	SetBool( grip_touch_, s.grip_touch || s.grip_click );
	SetScalar( stick_x_, s.stick_x );
	SetScalar( stick_y_, s.stick_y );
	SetBool( stick_click_, s.stick_click );
	SetBool( stick_touch_, s.stick_touch || s.stick_click );

	// Hold create/options: after one second recenter the heading, after three
	// calibrate the full orientation. A short buzz confirms each step.
	const auto now = Clock::now();
	if ( s.menu_click && !menu_was_down_ )
	{
		menu_down_since_ = now;
		menu_stage_ = 0;
	}
	if ( s.menu_click && menu_was_down_ )
	{
		const auto held = now - menu_down_since_;
		if ( menu_stage_ == 0 && held >= kRecenterHold )
		{
			recenter_requested_ = true;
			Buzz( { 0.06 } );
			menu_stage_ = 1;
		}
		else if ( menu_stage_ == 1 && held >= kCalibrateHold )
		{
			calibrate_requested_ = true;
			Buzz( { 0.3 } );
			menu_stage_ = 2;
		}
	}
	menu_was_down_ = s.menu_click;

	UpdateBattery( s );
	UpdateSkeleton( s );
	if ( !bias_saved_ && fusion_.Settled() )
	{
		SaveGyroBias();
		bias_saved_ = true;
	}

	gyro_world_ = vrmath::Rotate( fusion_.Orientation(), fusion_.Unbiased( s.gyro ) );
}

vr::DriverPose_t Psvr2ControllerDriver::BuildPose()
{
	vr::DriverPose_t p{};
	p.qWorldFromDriverRotation.w = 1.0;
	p.qDriverFromHeadRotation.w = 1.0;
	p.qRotation.w = 1.0;
	p.deviceIsConnected = true;

	const vr::DriverPose_t head = hmd_ ? hmd_->GetPose() : vr::DriverPose_t{};
	if ( !fusion_.Initialized() || !head.poseIsValid )
	{
		p.result = vr::TrackingResult_Running_OutOfRange;
		return p;
	}

	const double head_yaw = vrmath::Yaw( head.qRotation );
	const vr::HmdQuaternion_t mounted = vrmath::Mul( fusion_.Orientation(), kImuToGrip );

	// Calibration: the user points the controller straight ahead and level.
	// Store the rotation that takes the current grip to a level, heading-only
	// orientation; it is a fixed offset in the controller frame.
	if ( calibrate_requested_.exchange( false ) )
	{
		calibration_ = vrmath::Normalize(
			vrmath::Mul( vrmath::Conj( mounted ), vrmath::AxisAngle( 0, 1, 0, vrmath::Yaw( mounted ) ) ) );
		SaveCalibration();
		recenter_requested_ = true;
		DriverLog( "psvr2: %s controller calibrated", left_ ? "left" : "right" );
	}
	const vr::HmdQuaternion_t raw_grip = vrmath::Mul( mounted, calibration_ );

	// The IMU has no heading reference: line it up with the headset on the
	// first pose and whenever recentering is requested.
	// After the headset goes on, recenter the first time the controller is
	// held level and steady in front (within a few seconds).
	const auto now = Clock::now();
	if ( recenter_on_wear_ && hmd_ && hmd_->PutOnCount() != seen_put_on_ )
	{
		seen_put_on_ = hmd_->PutOnCount();
		wear_recenter_until_ = now + kWearRecenterWindow;
	}
	if ( now < wear_recenter_until_ )
	{
		const vrmath::Vec3 fwd = vrmath::Rotate( raw_grip, { 0.0, 0.0, -1.0 } );
		const vrmath::Vec3 &w = smoothed_gyro_world_;
		if ( std::fabs( fwd.y ) < 0.4 && w.x * w.x + w.y * w.y + w.z * w.z < 0.25 )
		{
			recenter_requested_ = true;
			wear_recenter_until_ = {};
		}
	}

	// Optical tracking: a fresh fix sets the heading, and while fixes keep
	// coming the head-based recentering is off.
	Optical o;
	{
		std::lock_guard<std::mutex> lock( optical_mutex_ );
		o = optical_;
	}
	const uint64_t now_us = uint64_t(
		std::chrono::duration_cast<std::chrono::microseconds>( now.time_since_epoch() ).count() );
	const bool optical_recent = o.have && now_us - o.us < 5000000;
	// With the filter running, orientation and position come from it; the
	// heading offset follows it so the arm-model fallback stays continuous.
	const bool ukf_active = ukf_enabled_ && ukf_.Initialized() && now_us < ukf_.LastFixUs() + 5000000;
	if ( ukf_active )
	{
		yaw_offset_ = vrmath::Yaw( vrmath::Mul( ukf_.Orientation(), vrmath::Conj( fusion_.Orientation() ) ) );
		yaw_anchored_ = true;
		optical_seq_used_ = o.seq;
	}
	else if ( o.have && o.seq != optical_seq_used_ )
	{
		const bool first = optical_seq_used_ == 0 || !yaw_anchored_;
		optical_seq_used_ = o.seq;
		// The fix is a few tens of ms old: only use it while the
		// controller turns slowly.
		const vrmath::Vec3 &gw = smoothed_gyro_world_;
		if ( first || gw.x * gw.x + gw.y * gw.y + gw.z * gw.z < 1.0 )
		{
			const vr::HmdQuaternion_t c = vrmath::Mul( o.rot, vrmath::Conj( fusion_.Orientation() ) );
			const vrmath::Vec3 up = vrmath::Rotate( c, { 0.0, 1.0, 0.0 } );
			if ( up.y > std::cos( 8.0 * kDeg ) )  // a heading-only difference
			{
				const double target = vrmath::Yaw( c );
				yaw_offset_ += vrmath::WrapPi( target - yaw_offset_ ) * ( first ? 1.0 : 0.1 );
				yaw_anchored_ = true;
			}
		}
	}
	if ( optical_recent )
	{
		recenter_requested_ = false;
		wear_recenter_until_ = {};
	}
	if ( !yaw_anchored_ || recenter_requested_.exchange( false ) )
	{
		yaw_offset_ = head_yaw - vrmath::Yaw( raw_grip );
		yaw_anchored_ = true;
	}
	yaw_pub_ = yaw_offset_;
	const vr::HmdQuaternion_t yaw_fix = vrmath::AxisAngle( 0, 1, 0, yaw_offset_ );
	const vr::HmdQuaternion_t grip =
		ukf_active ? vrmath::Mul( vrmath::Mul( ukf_.Orientation(), kImuToGrip ), calibration_ ) : vrmath::Mul( yaw_fix, raw_grip );

	// Arm model.
	const vr::HmdQuaternion_t head_yaw_q = vrmath::AxisAngle( 0, 1, 0, head_yaw );
	vrmath::Vec3 shoulder = kShoulder;
	if ( left_ )
		shoulder.x = -shoulder.x;
	const vrmath::Vec3 head_pos{ head.vecPosition[0], head.vecPosition[1], head.vecPosition[2] };
	const vr::HmdQuaternion_t elbow_yaw = vrmath::AxisAngle( 0, 1, 0, vrmath::Yaw( grip ) );
	vrmath::Vec3 hand = vrmath::Add( head_pos, vrmath::Rotate( head_yaw_q, shoulder ) );
	hand = vrmath::Add( hand, vrmath::Rotate( elbow_yaw, kUpperArm ) );
	hand = vrmath::Add( hand, vrmath::Rotate( grip, kForearm ) );

	// Velocities let SteamVR extrapolate over the Bluetooth latency
	// (poseTimeOffset below); smoothed so the prediction does not amplify
	// sensor noise into jitter.
	const vrmath::Vec3 w = vrmath::Rotate( yaw_fix, gyro_world_ );
	smoothed_gyro_world_ = vrmath::Add( vrmath::Scale( smoothed_gyro_world_, 0.6 ), vrmath::Scale( w, 0.4 ) );
	if ( last_pose_time_ >= 0.0 && last_imu_time_ > last_pose_time_ )
	{
		const double dt = last_imu_time_ - last_pose_time_;
		const vrmath::Vec3 v = vrmath::Scale( vrmath::Add( hand, vrmath::Scale( last_hand_, -1.0 ) ), 1.0 / dt );
		hand_velocity_ = vrmath::Add( vrmath::Scale( hand_velocity_, 0.7 ), vrmath::Scale( v, 0.3 ) );
	}
	last_hand_ = hand;
	last_pose_time_ = last_imu_time_;

	p.qRotation = grip;
	vrmath::Vec3 out_pos = hand, out_vel = hand_velocity_;
	double time_offset = -prediction_s_;
	if ( ukf_active )
	{
		// Sony (docs/sony-re/controller-tracking.md §C.5): the filter while fixes are fresh, then the
		// last fix position held; the rotation stays filter-driven.
		if ( now_us < ukf_.LastFixUs() + ukf_hold_us_ )
		{
			out_pos = ukf_.Position();
			out_vel = ukf_.Velocity();
			time_offset = -double( int64_t( now_us - ukf_.TimeUs() ) ) * 1e-6;
		}
		else
		{
			out_pos = ukf_.FixPosition();
			out_vel = {};
		}
	}
	else if ( o.have )
	{
		// Sony (docs/sony-re/controller-tracking.md §C.5): extrapolate the newest optical position
		// for up to 0.3 s, then hold it; the rotation stays IMU-driven.
		const double age = double( int64_t( now_us - o.us ) ) * 1e-6;
		if ( age < 0.3 )
		{
			const double dt = std::clamp( age - prediction_s_, 0.0, 0.1 );
			out_pos = vrmath::Add( o.pos, vrmath::Scale( o.vel, dt ) );
			out_vel = o.vel;
		}
		else
		{
			out_pos = o.pos;
			out_vel = {};
		}
	}
	p.vecPosition[0] = out_pos.x;
	p.vecPosition[1] = out_pos.y;
	p.vecPosition[2] = out_pos.z;
	p.vecVelocity[0] = out_vel.x;
	p.vecVelocity[1] = out_vel.y;
	p.vecVelocity[2] = out_vel.z;
	p.vecAngularVelocity[0] = smoothed_gyro_world_.x;
	p.vecAngularVelocity[1] = smoothed_gyro_world_.y;
	p.vecAngularVelocity[2] = smoothed_gyro_world_.z;
	p.poseTimeOffset = time_offset;
	p.poseIsValid = true;
	p.result = vr::TrackingResult_Running_OK;
	return p;
}

int Psvr2ControllerDriver::HardwareRevision()
{
	std::lock_guard<std::mutex> lock( hid_mutex_ );
	return hid_ ? hid_->HardwareRevision() : 0;
}

bool Psvr2ControllerDriver::ImuOrientation( vr::HmdQuaternion_t &q, double &yaw )
{
	std::lock_guard<std::mutex> lock( imu_mutex_ );
	q = imu_q_;
	yaw = yaw_pub_;
	return imu_valid_ && attached_;
}

bool Psvr2ControllerDriver::OpticalFix( uint64_t frame_us, const vrmath::Vec3 &pos,
                                        const vr::HmdQuaternion_t &world_from_imu, const vrmath::Vec3 &head_rel,
                                        double cam_dist )
{
	std::lock_guard<std::mutex> lock( optical_mutex_ );
	Optical &o = optical_;
	if ( o.have && frame_us <= o.us )
		return false;
	const double dt = o.have ? double( frame_us - o.us ) * 1e-6 : 1e9;
	o.head_rel = head_rel;
	o.cam_dist = cam_dist;
	if ( ukf_enabled_ )
		o.vel = {};  // the filter gates and estimates velocity (HandleState)
	else if ( dt < 0.3 )
	{
		// Innovation gate (Sony's defaults): 5 cm below 1 m/s, else 20 cm.
		// After ten rejects in a row, start over from the measurement.
		const vrmath::Vec3 pred = vrmath::Add( o.pos, vrmath::Scale( o.vel, dt ) );
		const vrmath::Vec3 d = vrmath::Add( pos, vrmath::Scale( pred, -1.0 ) );
		const double dist = std::sqrt( d.x * d.x + d.y * d.y + d.z * d.z );
		const double speed = std::sqrt( o.vel.x * o.vel.x + o.vel.y * o.vel.y + o.vel.z * o.vel.z );
		if ( dist > ( speed < 1.0 ? 0.05 : 0.20 ) && ++o.rejects < 10 )
			return false;
		if ( o.rejects < 10 && dt > 0.004 )
		{
			const vrmath::Vec3 v = vrmath::Scale( vrmath::Add( pos, vrmath::Scale( o.pos, -1.0 ) ), 1.0 / dt );
			o.vel = vrmath::Add( vrmath::Scale( o.vel, 0.5 ), vrmath::Scale( v, 0.5 ) );
		}
		else if ( o.rejects >= 10 )
			o.vel = {};
	}
	else
		o.vel = {};
	o.rejects = 0;
	o.have = true;
	o.us = frame_us;
	o.pos = pos;
	o.rot = world_from_imu;
	o.seq++;
	return true;
}

void Psvr2ControllerDriver::IoThread()
{
	auto next_output = Clock::now();

	while ( running_ )
	{
		if ( !attached_ )
		{
			std::this_thread::sleep_for( std::chrono::milliseconds( 50 ) );
			continue;
		}

		SenseHid *hid;
		{
			std::lock_guard<std::mutex> lock( hid_mutex_ );
			hid = hid_.get();
		}
		if ( !hid )
			continue;

		pollfd pfd{ hid->Fd(), POLLIN, 0 };
		const int to_ms = int( std::max<int64_t>(
			0, std::chrono::duration_cast<std::chrono::milliseconds>( next_output - Clock::now() ).count() ) );
		if ( poll( &pfd, 1, std::min( to_ms, 5 ) ) < 0 && errno != EINTR )
		{
			Disconnected();
			continue;
		}
		if ( pfd.revents & ( POLLERR | POLLHUP | POLLNVAL ) )
		{
			Disconnected();
			continue;
		}

		bool updated = false;
		SenseState state;
		int r;
		while ( ( r = hid->Read( state ) ) > 0 )
		{
			HandleState( state );
			updated = true;
		}
		if ( r < 0 )
		{
			Disconnected();
			continue;
		}

		// BlueZ keeps the hidraw node open across a Bluetooth link drop, so a
		// lost controller just goes quiet. When it reconnects it is back in
		// basic mode until its calibration is read again. Treat a second of
		// silence as a disconnect; the provider then reopens the node, which
		// re-reads the calibration and restores full reports.
		if ( updated )
			last_input_ = Clock::now();
		else if ( Clock::now() - last_input_ > kInputTimeout )
		{
			DriverLog( "psvr2: %s Sense controller sent nothing for %lld ms; reopening", left_ ? "left" : "right",
			           (long long)std::chrono::duration_cast<std::chrono::milliseconds>( kInputTimeout ).count() );
			Disconnected();
			continue;
		}

		if ( updated )
		{
			const vr::DriverPose_t pose = BuildPose();
			{
				std::lock_guard<std::mutex> lock( pose_mutex_ );
				pose_ = pose;
			}
			if ( device_index_ != vr::k_unTrackedDeviceIndexInvalid )
				vr::VRServerDriverHost()->TrackedDevicePoseUpdated( device_index_, pose, sizeof( pose ) );
		}

		if ( Clock::now() >= next_output )
		{
			SenseOutput out;
			FillOutput( out );
			if ( !hid->Write( out ) )
			{
				// Keep the effect pending for the next attempt.
				std::lock_guard<std::mutex> lock( out_mutex_ );
				trigger_dirty_ |= out.trigger_dirty;
			}
			next_output += kOutputPeriod;
			if ( next_output < Clock::now() )
				next_output = Clock::now() + kOutputPeriod;
		}
	}
}

bool Psvr2ControllerDriver::LedSyncCommand( const uint8_t *cmd, uint8_t len )
{
	std::lock_guard<std::mutex> lock( hid_mutex_ );
	if ( !hid_ )
		return false;
	hid_->LedSyncCommand( cmd, len );
	return true;
}

bool Psvr2ControllerDriver::LedTestSweep( int32_t offset_us )
{
	std::lock_guard<std::mutex> lock( hid_mutex_ );
	return hid_ && hid_->LedTestSweep( offset_us );
}

bool Psvr2ControllerDriver::LedSyncStatus( SenseHid::LedSyncStatus &out )
{
	std::lock_guard<std::mutex> lock( hid_mutex_ );
	if ( !hid_ || !attached_ )
		return false;
	out = hid_->LedSync();
	return true;
}
