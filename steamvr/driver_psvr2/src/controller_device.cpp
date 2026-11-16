// SPDX-License-Identifier: GPL-2.0
#include "controller_device.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <poll.h>

#include "driverlog.h"
#include "hmd_device_driver.h"

namespace
{
using Clock = std::chrono::steady_clock;
constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr auto kOutputPeriod = std::chrono::microseconds( 10667 );  // 32 PCM samples at 3 kHz
constexpr auto kRecenterHold = std::chrono::milliseconds( 1000 );
constexpr auto kCalibrateHold = std::chrono::milliseconds( 3000 );

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
	vr::EVRSettingsError err = vr::VRSettingsError_None;
	const bool enhanced = vr::VRSettings()->GetBool( "driver_psvr2", "enhanced_haptics", &err );
	enhanced_haptics_ = err == vr::VRSettingsError_None ? enhanced : true;
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

	running_ = true;
	io_thread_ = std::thread( &Psvr2ControllerDriver::IoThread, this );
	DriverLog( "psvr2: %s Sense controller registered", left_ ? "left" : "right" );
	return vr::VRInitError_None;
}

void Psvr2ControllerDriver::Deactivate()
{
	if ( running_.exchange( false ) && io_thread_.joinable() )
		io_thread_.join();
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
		yaw_anchored_ = false;
	}
	{
		std::lock_guard<std::mutex> lock( out_mutex_ );
		trigger_dirty_ = true;  // re-apply the current effect on reconnect
	}
	attached_ = true;
	DriverLog( "psvr2: %s Sense controller on %s (%s)", left_ ? "left" : "right", info.path.c_str(),
	           info.bluetooth ? "Bluetooth" : "USB" );
}

void Psvr2ControllerDriver::Disconnected()
{
	{
		std::lock_guard<std::mutex> lock( hid_mutex_ );
		hid_.reset();
	}
	attached_ = false;
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
	std::lock_guard<std::mutex> lock( out_mutex_ );
	haptic_amplitude_ = std::clamp( h.fAmplitude, 0.0f, 1.0f );
	haptic_frequency_ = h.fFrequency;
	// A zero duration is a single short pulse.
	const double secs = std::max( 0.011, double( h.fDurationSeconds ) );
	haptic_end_ = Clock::now() + std::chrono::duration_cast<Clock::duration>( std::chrono::duration<double>( secs ) );
}

void Psvr2ControllerDriver::SetTriggerEffect( const TriggerEffect &fx )
{
	std::lock_guard<std::mutex> lock( out_mutex_ );
	if ( fx != trigger_ )
	{
		trigger_ = fx;
		trigger_dirty_ = true;
	}
}

void Psvr2ControllerDriver::Recenter()
{
	recenter_requested_ = true;
}

void Psvr2ControllerDriver::Pulse( double seconds )
{
	std::lock_guard<std::mutex> lock( out_mutex_ );
	haptic_amplitude_ = 0.6f;
	haptic_frequency_ = 160.0f;
	haptic_end_ = Clock::now() + std::chrono::duration_cast<Clock::duration>( std::chrono::duration<double>( seconds ) );
}

std::string Psvr2ControllerDriver::CalibrationKey( char axis ) const
{
	return std::string( left_ ? "left" : "right" ) + "_calibration_" + axis;
}

void Psvr2ControllerDriver::LoadCalibration()
{
	double q[4];
	const char axes[4] = { 'w', 'x', 'y', 'z' };
	for ( int i = 0; i < 4; i++ )
	{
		vr::EVRSettingsError err = vr::VRSettingsError_None;
		q[i] = vr::VRSettings()->GetFloat( "driver_psvr2", CalibrationKey( axes[i] ).c_str(), &err );
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
		vr::VRSettings()->SetFloat( "driver_psvr2", CalibrationKey( axes[i] ).c_str(), float( q[i] ) );
}

void Psvr2ControllerDriver::FillOutput( SenseOutput &out )
{
	std::lock_guard<std::mutex> lock( out_mutex_ );
	out.trigger = trigger_;
	out.trigger_dirty = trigger_dirty_;
	trigger_dirty_ = false;

	const bool active = haptic_amplitude_ > 0.0f && Clock::now() < haptic_end_;
	out.rumble_amplitude = 0;
	out.has_pcm = false;
	if ( !active )
	{
		pcm_phase_ = 0.0;
		return;
	}

	if ( enhanced_haptics_ )
	{
		// Synthesize the requested tone as PCM for the voice-coil actuator.
		const double hz = std::clamp( haptic_frequency_ > 0.0f ? double( haptic_frequency_ ) : 160.0, 20.0, 1000.0 );
		const double step = 2.0 * 3.14159265358979323846 * hz / sense::kPcmSampleRate;
		for ( int i = 0; i < sense::kPcmSamplesPerReport; i++ )
		{
			out.pcm[i] = int8_t( std::lround( 127.0 * haptic_amplitude_ * std::sin( pcm_phase_ ) ) );
			pcm_phase_ = std::fmod( pcm_phase_ + step, 2.0 * 3.14159265358979323846 );
		}
		out.has_pcm = true;
	}
	else
	{
		out.rumble_amplitude = uint8_t( std::lround( haptic_amplitude_ * 255.0f ) );
		out.rumble_mode = haptic_frequency_ > 0.0f && haptic_frequency_ <= 70.0f ? sense::kFlag1VibrationLow60Hz
		                  : haptic_frequency_ >= 110.0f                          ? sense::kFlag1VibrationHigh120Hz
		                                                                         : sense::kFlag1VibrationClassic;
	}
}

void Psvr2ControllerDriver::HandleState( const SenseState &s )
{
	// --- orientation ---
	const double dt = last_imu_time_ < 0.0 ? 0.0 : s.imu_time_s - last_imu_time_;
	last_imu_time_ = s.imu_time_s;
	fusion_.Update( s.accel, s.gyro, dt );

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
			Pulse( 0.06 );
			menu_stage_ = 1;
		}
		else if ( menu_stage_ == 1 && held >= kCalibrateHold )
		{
			calibrate_requested_ = true;
			Pulse( 0.3 );
			menu_stage_ = 2;
		}
	}
	menu_was_down_ = s.menu_click;

	if ( s.battery_valid && ( std::fabs( s.battery - battery_ ) > 0.01f || s.charging != charging_ ) )
	{
		battery_ = s.battery;
		charging_ = s.charging;
		vr::VRProperties()->SetFloatProperty( container_, vr::Prop_DeviceBatteryPercentage_Float, battery_ );
		vr::VRProperties()->SetBoolProperty( container_, vr::Prop_DeviceIsCharging_Bool, charging_ );
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
	if ( !yaw_anchored_ || recenter_requested_.exchange( false ) )
	{
		yaw_offset_ = head_yaw - vrmath::Yaw( raw_grip );
		yaw_anchored_ = true;
	}
	const vr::HmdQuaternion_t yaw_fix = vrmath::AxisAngle( 0, 1, 0, yaw_offset_ );
	const vr::HmdQuaternion_t grip = vrmath::Mul( yaw_fix, raw_grip );

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

	p.qRotation = grip;
	p.vecPosition[0] = hand.x;
	p.vecPosition[1] = hand.y;
	p.vecPosition[2] = hand.z;
	const vrmath::Vec3 w = vrmath::Rotate( yaw_fix, gyro_world_ );
	p.vecAngularVelocity[0] = w.x;
	p.vecAngularVelocity[1] = w.y;
	p.vecAngularVelocity[2] = w.z;
	p.poseIsValid = true;
	p.result = vr::TrackingResult_Running_OK;
	return p;
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
