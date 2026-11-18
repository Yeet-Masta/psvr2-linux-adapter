// SPDX-License-Identifier: GPL-2.0
#include "hmd_device_driver.h"
#include "vr_math.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <poll.h>
#include <string>
#include <thread>

#include "driverlog.h"
#include "libpsvr2.h"

static const char *kSettingsSection = "driver_psvr2";

// PSVR2 panel EDID identity, decoded from the Sony SIE OSS panel EDID (see
// docs/references.md): manufacturer bytes 0x4D 0xD9 = "SNY", product id 0xA205,
// monitor name "SIE  VRH". SteamVR uses these to match and DRM-lease the
// physical connector in direct mode.
static constexpr int32_t kEdidVendorId = 0x4DD9;   // "SNY"
static constexpr int32_t kEdidProductId = 0xA205;

// Sony's names for the board stage in Get 0x81 +0x34.
static std::string BoardStageName( uint32_t stage )
{
	static const char *const names[] = { nullptr, nullptr, "EVA2", "EVA3", "EVT1", "EVT1R",
	                                     "EVT2", "DVT1", "DVT2", "PVT", "MP" };
	if ( stage < sizeof( names ) / sizeof( names[0] ) && names[stage] )
		return names[stage];
	return "stage " + std::to_string( stage );
}

Psvr2HmdDriver::Psvr2HmdDriver()
{
	// Defaults can be overridden in resources/settings/default.vrsettings.
	// IVRSettings::GetString has no default-value arg, so read then fall back.
	char buf[256] = { 0 };
	vr::EVRSettingsError err = vr::VRSettingsError_None;
	vr::VRSettings()->GetString( kSettingsSection, "model_number", buf, sizeof( buf ), &err );
	model_number_ = ( err == vr::VRSettingsError_None && buf[0] ) ? buf : "PSVR2";

	buf[0] = 0;
	err = vr::VRSettingsError_None;
	vr::VRSettings()->GetString( kSettingsSection, "serial_number", buf, sizeof( buf ), &err );
	serial_number_ = ( err == vr::VRSettingsError_None && buf[0] ) ? buf : "PSVR2-0001";

	// The panel runs at 89.91 or 119.88 Hz (its EDID modes). SteamVR's own
	// refresh-rate choice (steamvr/preferredRefreshRate, the Video settings
	// slider) picks the mode it leases, so report that same rate: pacing and
	// pose prediction at 90 Hz on a 120 Hz scanout tears and smears.
	// driver_psvr2/display_frequency is the fallback when SteamVR has none.
	float hz = vr::VRSettings()->GetFloat( vr::k_pch_SteamVR_Section, "preferredRefreshRate" );
	if ( !( hz > 0.0f ) )
		hz = vr::VRSettings()->GetFloat( kSettingsSection, "display_frequency" );
	if ( hz > 0.0f )
		display_frequency_ = hz > 105.0f ? 119.88f : 89.91f;
	// The compositor matches steamvr/preferredRefreshRate against the
	// available-rates list exactly. With {90, 120} a stored 119.88 matched
	// nothing, so it fell back to the first entry and set the display
	// frequency to 90 while scanning out the 119.88 Hz mode: frames paced
	// for 90 Hz on a 120 Hz panel, seen as tearing (vrcompositor.txt,
	// 2026-11-18). List the stored value itself as the entry it is closest to.
	if ( hz > 0.0f && std::fabs( hz - display_frequency_ ) < 1.0f )
		display_frequency_ = hz;
	if ( hz > 105.0f )
		rate_120_ = display_frequency_;
	else if ( hz > 0.0f )
		rate_90_ = display_frequency_;

	dev_ = psvr2_open();
	// Sony offers 120 Hz only on headset software newer than 0x06000000.
	firmware_version_ = psvr2_firmware_version( dev_ );
	supports_120_ = firmware_version_ == 0 || firmware_version_ > 0x06000000;
	if ( !supports_120_ && display_frequency_ > 105.0f )
		display_frequency_ = rate_90_;
	SelectEdid( "init" );
	// The tracker's origin is where it started; without a SteamVR room setup
	// this lifts the view to standing height. 0 (default) leaves it alone.
	standing_height_ = std::clamp( vr::VRSettings()->GetFloat( kSettingsSection, "standing_height_m" ), 0.0f, 2.5f );

	Psvr2DisplayConfig cfg{};
	// Each field falls back to the struct default when the setting is absent (0).
	if ( int32_t v = vr::VRSettings()->GetInt32( kSettingsSection, "window_x" ) )      cfg.window_x = v;
	if ( int32_t v = vr::VRSettings()->GetInt32( kSettingsSection, "window_y" ) )      cfg.window_y = v;
	if ( int32_t v = vr::VRSettings()->GetInt32( kSettingsSection, "window_width" ) )  cfg.window_width = v;
	if ( int32_t v = vr::VRSettings()->GetInt32( kSettingsSection, "window_height" ) ) cfg.window_height = v;
	// Sony recommends 1.7x the per-eye panel size (3400 x 3468), so the lens
	// centre gets about one rendered pixel per panel pixel after distortion.
	cfg.render_width = cfg.window_width / 2 * 17 / 10;
	cfg.render_height = cfg.window_height * 17 / 10;

	// Direct mode is the default; an explicit "direct_mode": false in settings
	// switches to the extended-desktop scaffold.
	vr::EVRSettingsError dm_err = vr::VRSettingsError_None;
	const bool direct_mode = vr::VRSettings()->GetBool( kSettingsSection, "direct_mode", &dm_err );
	cfg.direct_mode = ( dm_err == vr::VRSettingsError_None ) ? direct_mode : true;
	direct_mode_ = cfg.direct_mode;

	if ( psvr2_get_lens_distortion( dev_, cfg.lens ) == 0 )
		DriverLog( "psvr2: using the headset's lens calibration" );
	else
		DriverLog( "psvr2: no lens calibration from the headset; using default lens centres" );

	display_ = std::make_unique<Psvr2DisplayComponent>( cfg );
	pose_source_ = std::make_unique<PoseSource>();

	if ( !pose_source_->HasPose() )
		DriverLog( "psvr2: /dev/psvr2-pose not found — HMD will not track (is the module loaded?)" );
}

Psvr2HmdDriver::~Psvr2HmdDriver()
{
	Deactivate();
	// As Sony on SteamVR quit: tell the headset (Set 0x27), and leave the EDID
	// matching the refresh setting for the next session.
	if ( psvr2_send_quit( dev_ ) != 0 )
		DriverLog( "psvr2: quit command (report 0x27) failed" );
	SelectEdid( "cleanup" );
	psvr2_close( dev_ );
}

// Sony's EDID choice (docs/sony-re/display.md 3.2): expose the 90/120 Hz
// EDID only when SteamVR runs at 120 Hz, the 90 Hz-only one otherwise, and
// give the DisplayPort sink a second to re-enumerate after a switch.
void Psvr2HmdDriver::SelectEdid( const char *why )
{
	const bool want120 = supports_120_ && display_frequency_ > 105.0f;
	const int type = psvr2_get_edid_type( dev_ );
	if ( type < 0 )
		return;
	const int target = want120 ? 0 : 1;
	if ( type == target )
		return;
	if ( psvr2_set_edid_type( dev_, target ) != 0 )
	{
		DriverLog( "psvr2: %s: EDID type %d -> %d failed", why, type, target );
		return;
	}
	DriverLog( "psvr2: %s: EDID type %d -> %d (%s), waiting for the display", why, type, target,
	           want120 ? "90/120 Hz" : "90 Hz" );
	std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
}

vr::EVRInitError Psvr2HmdDriver::Activate( uint32_t unObjectId )
{
	device_index_ = unObjectId;

	vr::PropertyContainerHandle_t c = vr::VRProperties()->TrackedDeviceToPropertyContainer( unObjectId );
	container_ = c;
	vr::VRProperties()->SetStringProperty( c, vr::Prop_ModelNumber_String, model_number_.c_str() );
	vr::VRProperties()->SetStringProperty( c, vr::Prop_ManufacturerName_String, "Sony" );
	vr::VRProperties()->SetBoolProperty( c, vr::Prop_ContainsProximitySensor_Bool, true );
	vr::VRProperties()->SetFloatProperty( c, vr::Prop_IpdUIRangeMinMeters_Float, 0.059f );
	vr::VRProperties()->SetFloatProperty( c, vr::Prop_IpdUIRangeMaxMeters_Float, 0.072f );
	vr::VRProperties()->SetInt32Property( c, vr::Prop_HmdTrackingStyle_Int32, vr::HmdTrackingStyle_InsideOutCameras );
	// Firmware and board, as Sony reports them once the headset is ready.
	if ( firmware_version_ )
	{
		char fw[16];
		snprintf( fw, sizeof( fw ), "%08x", firmware_version_ );
		vr::VRProperties()->SetStringProperty( c, vr::Prop_TrackingFirmwareVersion_String, fw );
	}
	if ( const uint32_t stage = psvr2_board_stage( dev_ ) )
		vr::VRProperties()->SetStringProperty( c, vr::Prop_HardwareRevision_String, BoardStageName( stage ).c_str() );

	// IPD: the headset's dial when the controls are readable, else SteamVR's.
	float ipd = vr::VRSettings()->GetFloat( vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_IPD_Float );
	struct psvr2_controls controls{};
	if ( psvr2_read_controls( dev_, &controls ) == 0 && controls.ipd_mm > 0 )
	{
		ipd_mm_ = controls.ipd_mm;
		ipd = float( ipd_mm_ ) * 0.001f;
	}
	vr::VRProperties()->SetFloatProperty( c, vr::Prop_UserIpdMeters_Float, ipd );
	vr::VRProperties()->SetStringProperty( c, vr::Prop_InputProfilePath_String, "{psvr2}/input/psvr2_hmd_profile.json" );

	// Eye tracking (OpenXR XR_EXT_eye_gaze_interaction through SteamVR).
	if ( psvr2_has_gaze( dev_ ) )
	{
		vr::VRProperties()->SetBoolProperty( c, vr::Prop_SupportsXrEyeGazeInteraction_Bool, true );
		vr::VRDriverInput()->CreateEyeTrackingComponent( c, "/eyetracking", &eye_tracking_ );
	}
	vr::VRDriverInput()->CreateHapticComponent( c, "/output/haptic", &haptic_ );
	vr::VRDriverInput()->CreateBooleanComponent( c, "/proximity", &proximity_ );

	vr::EVRSettingsError cam_err = vr::VRSettingsError_None;
	const bool camera = vr::VRSettings()->GetBool( kSettingsSection, "passthrough_camera", &cam_err );
	if ( cam_err != vr::VRSettingsError_None || camera )
	{
		camera_ = std::make_unique<Psvr2CameraComponent>( [this] { return GetPose(); } );
		Psvr2CameraComponent::SetProperties( c );
	}

	// Required for the compositor to start.
	vr::VRProperties()->SetFloatProperty( c, vr::Prop_DisplayFrequency_Float, display_frequency_ );
	// Let SteamVR offer the panel's modes (it reads them from the EDID), and
	// list them as Sony does.
	vr::VRProperties()->SetBoolProperty( c, vr::Prop_DisplaySupportsMultipleFramerates_Bool, supports_120_ );
	{
		const float rates[2] = { rate_90_, rate_120_ };
		vr::VRProperties()->SetProperty( c, vr::Prop_DisplayAvailableFrameRates_Float_Array, const_cast<float *>( rates ),
		                                 sizeof( float ) * ( supports_120_ ? 2 : 1 ), vr::k_unFloatPropertyTag );
	}
	DriverLog( "psvr2: display refresh %.2f Hz", display_frequency_ );
	vr::VRProperties()->SetFloatProperty( c, vr::Prop_UserHeadToEyeDepthMeters_Float, 0.0f );
	// Panel timing as Sony's driver reports it (docs/sony-re/display.md).
	const bool hz120 = display_frequency_ > 105.0f;
	// Display timing. Like Sony's "debug/vsync to photons" and "debug/photons
	// to vblank" keys, settings in microseconds override both values (a
	// value of -1 or less leaves the default; 0 is a valid override).
	float vsync_to_photons = hz120 ? 0.00102f : 0.00136f;
	float photons_to_vblank = hz120 ? 0.00773f : 0.0103f;
	{
		vr::EVRSettingsError e1 = vr::VRSettingsError_None, e2 = vr::VRSettingsError_None;
		const float v1 = vr::VRSettings()->GetFloat( kSettingsSection, "vsync_to_photons_us", &e1 );
		const float v2 = vr::VRSettings()->GetFloat( kSettingsSection, "photons_to_vblank_us", &e2 );
		if ( e1 == vr::VRSettingsError_None && v1 > -1.0f ) vsync_to_photons = v1 * 1e-6f;
		if ( e2 == vr::VRSettingsError_None && v2 > -1.0f ) photons_to_vblank = v2 * 1e-6f;
	}
	DriverLog( "psvr2: display timing: vsync->photons %.0f us, photons->vblank %.0f us",
	           vsync_to_photons * 1e6f, photons_to_vblank * 1e6f );
	vr::VRProperties()->SetFloatProperty( c, vr::Prop_SecondsFromVsyncToPhotons_Float, vsync_to_photons );
	vr::VRProperties()->SetFloatProperty( c, vr::Prop_SecondsFromPhotonsToVblank_Float, photons_to_vblank );

	// EDID identity lets SteamVR's compositor find the physical connector and
	// (in direct mode) DRM-lease it. In extended mode it's harmless.
	vr::VRProperties()->SetInt32Property( c, vr::Prop_EdidVendorID_Int32, kEdidVendorId );
	vr::VRProperties()->SetInt32Property( c, vr::Prop_EdidProductID_Int32, kEdidProductId );

	// IsOnDesktop must agree with the display component: false => SteamVR
	// acquires the panel directly (direct mode); true => extended desktop.
	vr::VRProperties()->SetBoolProperty( c, vr::Prop_IsOnDesktop_Bool, !direct_mode_ );

	// Hidden area outside the lenses, from this headset's lens calibration.
	for ( int eye = 0; eye < 2; eye++ )
	{
		float v[PSVR2_HIDDEN_AREA_VERTS][2];
		const int n = psvr2_hidden_area( display_->Lens(), eye, v );
		vr::HmdVector2_t mesh[PSVR2_HIDDEN_AREA_VERTS];
		for ( int i = 0; i < n; i++ )
			mesh[i] = { v[i][0], v[i][1] };
		vr::VRProperties()->SetProperty( c, vr::ETrackedDeviceProperty( vr::Prop_DisplayHiddenArea_Binary_Start + eye ),
		                                 mesh, sizeof( vr::HmdVector2_t ) * n, vr::k_unHiddenAreaPropertyTag );
	}
	vr::VRProperties()->SetInt32Property( c, vr::Prop_DistortionMeshResolution_Int32, 80 );

	DriverLog( "psvr2: display mode = %s (EDID %04X:%04X)",
	           direct_mode_ ? "direct (DRM lease)" : "extended desktop",
	           kEdidVendorId, kEdidProductId );

	active_ = true;
	pose_thread_ = std::thread( &Psvr2HmdDriver::PoseThread, this );
	gaze_thread_ = std::thread( &Psvr2HmdDriver::GazeThread, this );
	return vr::VRInitError_None;
}

void Psvr2HmdDriver::Deactivate()
{
	if ( active_.exchange( false ) )
	{
		if ( pose_thread_.joinable() )
			pose_thread_.join();
		if ( gaze_thread_.joinable() )
			gaze_thread_.join();
	}
	if ( camera_ )
		camera_->StopVideoStream();
	SetRumble( 0 );
	device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void Psvr2HmdDriver::EnterStandby() {}

void *Psvr2HmdDriver::GetComponent( const char *pchComponentNameAndVersion )
{
	if ( std::strcmp( pchComponentNameAndVersion, vr::IVRDisplayComponent_Version ) == 0 )
		return display_.get();
	if ( camera_ && std::strcmp( pchComponentNameAndVersion, vr::IVRCameraComponent_Version ) == 0 )
		return static_cast<vr::IVRCameraComponent *>( camera_.get() );
	return nullptr;
}

void Psvr2HmdDriver::DebugRequest( const char *, char *pchResponseBuffer, uint32_t unResponseBufferSize )
{
	if ( unResponseBufferSize >= 1 )
		pchResponseBuffer[0] = 0;
}

vr::DriverPose_t Psvr2HmdDriver::GetPose()
{
	std::lock_guard<std::mutex> lock( pose_mutex_ );
	return last_pose_;
}

bool Psvr2HmdDriver::PoseAt( uint64_t host_us, vr::DriverPose_t &out )
{
	std::lock_guard<std::mutex> lock( pose_mutex_ );
	if ( pose_history_.empty() )
		return false;
	auto it = pose_history_.begin();
	while ( it != pose_history_.end() && it->host_us < host_us )
		++it;
	if ( it == pose_history_.end() || it == pose_history_.begin() )
	{
		// Outside the history: take the nearest end if it is close enough.
		const TimedPose &e = it == pose_history_.end() ? pose_history_.back() : pose_history_.front();
		const int64_t gap = int64_t( host_us ) - int64_t( e.host_us );
		if ( gap > 100000 || gap < -100000 )
			return false;
		out = e.pose;
		return out.poseIsValid;
	}
	const TimedPose &b = *it, &a = *( it - 1 );
	if ( b.host_us - a.host_us > 100000 )
		return false;
	const double t = double( host_us - a.host_us ) / double( b.host_us - a.host_us );
	out = b.pose;
	for ( int i = 0; i < 3; i++ )
		out.vecPosition[i] = a.pose.vecPosition[i] + ( b.pose.vecPosition[i] - a.pose.vecPosition[i] ) * t;
	out.qRotation = vrmath::Nlerp( a.pose.qRotation, b.pose.qRotation, t );
	return a.pose.poseIsValid && b.pose.poseIsValid;
}

Psvr2GazeData Psvr2HmdDriver::GetGaze()
{
	std::lock_guard<std::mutex> lock( gaze_mutex_ );
	return gaze_;
}

void Psvr2HmdDriver::SetRumble( int hz )
{
	hz = std::clamp( hz, 0, 25 );
	if ( hz == rumble_hz_ )
		return;
	if ( psvr2_set_rumble( dev_, hz ) == 0 )
		rumble_hz_ = hz;
}

void Psvr2HmdDriver::OnHaptic( const vr::VREvent_HapticVibration_t &h )
{
	// The motor takes a frequency only; map amplitude onto it (stronger =
	// faster) since games set amplitude far more often than frequency.
	const float amp = std::clamp( h.fAmplitude, 0.0f, 1.0f );
	SetRumble( amp > 0.0f ? std::max( 1, int( std::lround( amp * 25.0f ) ) ) : 0 );
	const double secs = std::max( 0.05, double( h.fDurationSeconds ) );
	rumble_end_ = std::chrono::steady_clock::now() +
	              std::chrono::duration_cast<std::chrono::steady_clock::duration>( std::chrono::duration<double>( secs ) );
}

void Psvr2HmdDriver::RunFrame()
{
	if ( alert_requested_.exchange( false ) )
	{
		SetRumble( 20 );
		rumble_end_ = std::chrono::steady_clock::now() + std::chrono::milliseconds( 400 );
	}
	if ( rumble_hz_ && std::chrono::steady_clock::now() >= rumble_end_ )
		SetRumble( 0 );
}

void Psvr2HmdDriver::PollControls()
{
	struct psvr2_controls c{};
	if ( psvr2_read_controls( dev_, &c ) != 0 )
		return;

	if ( c.ipd_mm > 0 && c.ipd_mm != ipd_mm_ )
	{
		ipd_mm_ = c.ipd_mm;
		vr::VRProperties()->SetFloatProperty( container_, vr::Prop_UserIpdMeters_Float, float( ipd_mm_ ) * 0.001f );
		DriverLog( "psvr2: IPD dial %d mm", ipd_mm_ );
	}
	// See-through (the function button toggles it in the kernel): Sony asks
	// SteamVR to pause the app and draw the controllers meanwhile.
	if ( ++see_through_poll_ >= 10 )
	{
		see_through_poll_ = 0;
		const int st = psvr2_get_see_through( dev_ );
		if ( st >= 0 && st != see_through_ )
		{
			see_through_ = st;
			vr::VRProperties()->SetBoolProperty( container_, vr::Prop_DriverRequestsApplicationPause_Bool, st );
			vr::VRProperties()->SetBoolProperty( container_, vr::Prop_DriverIsDrawingControllers_Bool, st );
			vr::VRProperties()->SetBoolProperty( container_, vr::Prop_DriverRequestsReducedRendering_Bool, st );
			DriverLog( "psvr2: see-through %s", st ? "on" : "off" );
		}
	}
	if ( c.worn != worn_ )
	{
		if ( worn_ == 0 && c.worn )
			put_on_count_++;
		// SteamVR derives user presence (and its standby) from /proximity.
		if ( proximity_ )
			vr::VRDriverInput()->UpdateBooleanComponent( proximity_, c.worn != 0, 0 );
		worn_ = c.worn;
	}
}

// Headset gaze frame -> OpenVR head frame (Monado negates x and z).
static vr::HmdVector3_t GazeToHead( const float v[3], float scale )
{
	return { -v[0] * scale, v[1] * scale, -v[2] * scale };
}

static void CopyEye( const struct psvr2_eye &in, Psvr2EyeData &out )
{
	out.origin_valid = in.gaze_point_valid;
	out.dir_valid = in.gaze_direction_valid;
	out.pupil_valid = in.pupil_diameter_valid;
	out.blink_valid = in.blink_valid;
	std::copy( in.gaze_point_mm, in.gaze_point_mm + 3, out.origin_mm );
	const float n = std::sqrt( in.gaze_direction[0] * in.gaze_direction[0] + in.gaze_direction[1] * in.gaze_direction[1] +
	                           in.gaze_direction[2] * in.gaze_direction[2] );
	for ( int i = 0; i < 3; i++ )
		out.dir[i] = n > 0.0f ? in.gaze_direction[i] / n : 0.0f;
	out.pupil_mm = in.pupil_diameter_mm;
	out.blink = in.blink;
	out.valid = true;
}

void Psvr2HmdDriver::GazeThread()
{
	auto next_controls = std::chrono::steady_clock::now();

	while ( active_ )
	{
		const auto now = std::chrono::steady_clock::now();
		if ( now >= next_controls )
		{
			PollControls();
			next_controls = now + std::chrono::milliseconds( 100 );
		}

		const int fd = psvr2_gaze_fd( dev_ );
		if ( fd < 0 )
		{
			std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
			continue;
		}
		pollfd pfd{ fd, POLLIN, 0 };
		if ( poll( &pfd, 1, 100 ) <= 0 )
			continue;

		struct psvr2_gaze g{};
		bool got = false;
		while ( psvr2_read_gaze( dev_, &g, 0 ) == 1 )
			got = true;
		if ( !got || !g.valid )
			continue;

		Psvr2GazeData d;
		CopyEye( g.left, d.left );
		CopyEye( g.right, d.right );
		d.combined_valid = g.combined.gaze_direction_valid;
		d.origin = g.combined.gaze_point_valid ? GazeToHead( g.combined.gaze_point_mm, 0.001f ) : vr::HmdVector3_t{};
		d.dir = GazeToHead( g.combined.gaze_direction, 1.0f );
		{
			std::lock_guard<std::mutex> lock( gaze_mutex_ );
			gaze_ = d;
		}

		if ( eye_tracking_ )
		{
			vr::VREyeTrackingData_t et{};
			et.bActive = true;
			et.bValid = d.combined_valid;
			et.bTracked = d.combined_valid;
			et.vGazeOrigin = d.origin;
			et.vGazeTarget = { d.origin.v[0] + d.dir.v[0], d.origin.v[1] + d.dir.v[1], d.origin.v[2] + d.dir.v[2] };
			vr::VRDriverInput()->UpdateEyeTrackingComponent( eye_tracking_, &et, 0.0 );
		}
	}
}

// Sony's not-worn throttle (docs/sony-re/display.md 5.5): while the headset
// is off the head, a pose that has not moved is only re-sent for half a
// second, so SteamVR can idle; any motion or state change sends again.
void Psvr2HmdDriver::SubmitPose( const vr::DriverPose_t &pose, bool worn )
{
	const auto now = std::chrono::steady_clock::now();
	bool fresh = worn || !have_ref_ || pose.poseIsValid != ref_pose_.poseIsValid ||
	             pose.deviceIsConnected != ref_pose_.deviceIsConnected || pose.result != ref_pose_.result;
	for ( int i = 0; i < 3 && !fresh; i++ )
		fresh = std::fabs( ref_pose_.vecPosition[i] - pose.vecPosition[i] ) >= 0.005 ||
		        std::fabs( pose.vecAngularVelocity[i] ) >= 0.03;
	if ( fresh )
	{
		ref_pose_ = pose;
		ref_time_ = now;
		have_ref_ = true;
	}
	else if ( now - ref_time_ > std::chrono::microseconds( 500001 ) )
		return;
	if ( device_index_ != vr::k_unTrackedDeviceIndexInvalid )
		vr::VRServerDriverHost()->TrackedDevicePoseUpdated( device_index_, pose, sizeof( pose ) );
}

// Play area for SteamVR, as Sony's driver provides one (docs/sony-re/room.md
// 1, 2.5): a chaperone_info file (version 5) with one universe, published
// through Prop_DriverProvidedChaperonePath and Prop_CurrentUniverseId.
// Without Sony's relocaliser the tracker's origin is wherever the headset
// started, so every session gets a fresh default area: a 24-point circle of
// radius 1.5 m around the first tracked pose, floor 1.5 m below the head
// (or at y = 0 when standing_height_m lifts the view already).
void Psvr2HmdDriver::WriteChaperone( const vr::DriverPose_t &pose )
{
	chaperone_written_ = true;
	vr::EVRSettingsError err = vr::VRSettingsError_None;
	const bool on = vr::VRSettings()->GetBool( kSettingsSection, "provide_chaperone", &err );
	if ( err == vr::VRSettingsError_None && !on )
		return;
	char dir[1024] = { 0 };
	vr::VRProperties()->GetStringProperty( container_, vr::Prop_UserConfigPath_String, dir, sizeof( dir ) );
	if ( !dir[0] )
		return;
	const std::string path = std::string( dir ) + "/psvr2_chaperone_info.vrchap";

	const vr::HmdQuaternion_t &q = pose.qRotation;
	// Heading about +y of the headset's forward (-z) axis.
	const double fx = -2.0 * ( q.x * q.z + q.w * q.y ), fz = -( 1.0 - 2.0 * ( q.x * q.x + q.y * q.y ) );
	const double yaw = std::atan2( -fx, -fz );
	const double hx = pose.vecPosition[0], hy = pose.vecPosition[1], hz = pose.vecPosition[2];
	const double floor_y = standing_height_ > 0.0f ? 0.0 : hy - 1.5;
	const uint64_t universe = uint64_t( time( nullptr ) );

	constexpr int kPoints = 24;
	constexpr double kRadius = 1.5, kWall = 2.43, kPi = 3.14159265358979323846;
	std::string bounds;
	char buf[512];
	for ( int i = 0; i < kPoints; i++ )
	{
		const double a0 = 2.0 * kPi * i / kPoints, a1 = 2.0 * kPi * ( i + 1 ) / kPoints;
		const double x0 = kRadius * std::cos( a0 ), z0 = kRadius * std::sin( a0 );
		const double x1 = kRadius * std::cos( a1 ), z1 = kRadius * std::sin( a1 );
		snprintf( buf, sizeof( buf ), "%s[[%.6f,0,%.6f],[%.6f,%.2f,%.6f],[%.6f,%.2f,%.6f],[%.6f,0,%.6f]]",
		          i ? "," : "", x0, z0, x0, kWall, z0, x1, kWall, z1, x1, z1 );
		bounds += buf;
	}
	FILE *f = fopen( path.c_str(), "w" );
	if ( !f )
	{
		DriverLog( "psvr2: cannot write %s", path.c_str() );
		return;
	}
	const double side = std::sqrt( 2.0 ) * kRadius;
	fprintf( f,
	         "{\n\"jsonid\":\"chaperone_info\",\n\"universes\":[{\n"
	         "\"collision_bounds\":[%s],\n"
	         "\"play_area\":[%.6f,%.6f],\n"
	         "\"seated\":{\"translation\":[%.6f,%.6f,%.6f],\"yaw\":%.6f},\n"
	         "\"standing\":{\"translation\":[%.6f,%.6f,%.6f],\"yaw\":%.6f},\n"
	         "\"universeID\":\"%llu\"\n}],\n\"version\":5\n}\n",
	         bounds.c_str(), side, side, hx, hy, hz, yaw, hx, floor_y, hz, yaw, (unsigned long long)universe );
	fclose( f );

	vr::VRProperties()->SetStringProperty( container_, vr::Prop_DriverProvidedChaperonePath_String, path.c_str() );
	vr::VRProperties()->SetUint64Property( container_, vr::Prop_CurrentUniverseId_Uint64, universe );
	universe_id_ = universe;
	DriverLog( "psvr2: play area: 1.5 m circle at (%.2f, %.2f), universe %llu", hx, hz,
	           (unsigned long long)universe );
}

void Psvr2HmdDriver::PoseThread()
{
	while ( active_ )
	{
		vr::DriverPose_t pose{};
		if ( pose_source_->ReadPose( pose ) )
		{
			pose.vecPosition[1] += standing_height_;
			{
				const uint64_t now_us = uint64_t( std::chrono::duration_cast<std::chrono::microseconds>(
									  std::chrono::steady_clock::now().time_since_epoch() )
				                                      .count() );
				std::lock_guard<std::mutex> lock( pose_mutex_ );
				last_pose_ = pose;
				pose_history_.push_back( { now_us + uint64_t( int64_t( pose.poseTimeOffset * 1e6 ) ), pose } );
				while ( pose_history_.size() > 2 && pose_history_.back().host_us - pose_history_.front().host_us > 500000 )
					pose_history_.pop_front();
			}
			if ( !chaperone_written_ && pose.result == vr::TrackingResult_Running_OK &&
			     container_ != vr::k_ulInvalidPropertyContainer )
				WriteChaperone( pose );
			SubmitPose( pose, worn_ != 0 );
		}
		else
		{
			// No node / timeout: don't spin hot.
			std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
		}
	}
}

// ---------------------------------------------------------------------------
// Display component
// ---------------------------------------------------------------------------

Psvr2DisplayComponent::Psvr2DisplayComponent( const Psvr2DisplayConfig &config )
	: config_( config )
{
}

bool Psvr2DisplayComponent::IsDisplayOnDesktop()
{
	// Direct mode (default): NOT on the desktop, so SteamVR's compositor
	// acquires the panel directly via DRM leasing. Extended mode: on the desktop
	// as an ordinary monitor. See docs/steamvr.md.
	return !config_.direct_mode;
}

bool Psvr2DisplayComponent::IsDisplayRealDisplay()
{
	return true;
}

void Psvr2DisplayComponent::GetRecommendedRenderTargetSize( uint32_t *pnWidth, uint32_t *pnHeight )
{
	*pnWidth = config_.render_width;
	*pnHeight = config_.render_height;
}

void Psvr2DisplayComponent::GetEyeOutputViewport( vr::EVREye eEye, uint32_t *pnX, uint32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight )
{
	*pnY = 0;
	*pnWidth = config_.window_width / 2;
	*pnHeight = config_.window_height;
	*pnX = ( eEye == vr::Eye_Left ) ? 0 : config_.window_width / 2;
}

void Psvr2DisplayComponent::GetProjectionRaw( vr::EVREye eEye, float *pfLeft, float *pfRight, float *pfTop, float *pfBottom )
{
	// Asymmetric per-eye FOV: wider towards the temple than the nose. Sony's
	// tangents unless the settings override the angles.
	constexpr float kDeg = 3.14159265f / 180.0f;
	const float outer = config_.fov_outer_deg > 0 ? std::tan( config_.fov_outer_deg * kDeg ) : 1.84177f;
	const float inner = config_.fov_inner_deg > 0 ? std::tan( config_.fov_inner_deg * kDeg ) : 0.94719f;
	const float vert = config_.fov_vertical_deg > 0 ? std::tan( config_.fov_vertical_deg * kDeg ) : 1.32898f;
	*pfLeft = ( eEye == vr::Eye_Left ) ? -outer : -inner;
	*pfRight = ( eEye == vr::Eye_Left ) ? inner : outer;
	*pfTop = -vert;
	*pfBottom = vert;
}

vr::DistortionCoordinates_t Psvr2DisplayComponent::ComputeDistortion( vr::EVREye eEye, float fU, float fV )
{
	// Sony's lens model with this headset's calibration (libpsvr2).
	float uv[6];
	psvr2_distort( config_.lens, eEye == vr::Eye_Left ? 0 : 1, fU, fV, uv );
	vr::DistortionCoordinates_t c{};
	c.rfRed[0] = uv[0];
	c.rfRed[1] = uv[1];
	c.rfGreen[0] = uv[2];
	c.rfGreen[1] = uv[3];
	c.rfBlue[0] = uv[4];
	c.rfBlue[1] = uv[5];
	return c;
}

void Psvr2DisplayComponent::GetWindowBounds( int32_t *pnX, int32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight )
{
	*pnX = config_.window_x;
	*pnY = config_.window_y;
	*pnWidth = config_.window_width;
	*pnHeight = config_.window_height;
}

bool Psvr2DisplayComponent::ComputeInverseDistortion( vr::HmdVector2_t *, vr::EVREye, uint32_t, float, float )
{
	// Let SteamVR infer the inverse from ComputeDistortion.
	return false;
}
