// SPDX-License-Identifier: GPL-2.0
#include "device_provider.h"

#include "driverlog.h"

#include <cstdlib>
#include <string>

// vrcompositor keeps a 90 Hz frame timing when the wanted mode is the first
// one listed at the HMD's resolution, which the 90/120 Hz EDID's 119.88 Hz
// mode is. The VK_LAYER_PSVR2_mode_order layer (steamvr/vk_mode_order)
// lists the modes in ascending refresh. vrserver forwards VK_INSTANCE_LAYERS
// to the compositor it launches after the drivers load, so add it here.
static void EnableModeOrderLayer()
{
	static const char kLayer[] = "VK_LAYER_PSVR2_mode_order";
	const char *cur = getenv( "VK_INSTANCE_LAYERS" );
	std::string v = cur ? cur : "";
	if ( v.find( kLayer ) != std::string::npos ) return;
	if ( !v.empty() ) v += ':';
	v += kLayer;
	setenv( "VK_INSTANCE_LAYERS", v.c_str(), 1 );
	DriverLog( "psvr2: VK_INSTANCE_LAYERS=%s", v.c_str() );
}

vr::EVRInitError Psvr2DeviceProvider::Init( vr::IVRDriverContext *pDriverContext )
{
	VR_INIT_SERVER_DRIVER_CONTEXT( pDriverContext );
	EnableModeOrderLayer();

	hmd_ = std::make_unique<Psvr2HmdDriver>();

	// Register the headset. The serial number must be unique and stable; we use
	// a fixed string since the adapter exposes a single headset (see roadmap M7:
	// multi-headset /dev naming is future work).
	if ( !vr::VRServerDriverHost()->TrackedDeviceAdded(
		     hmd_->GetSerialNumber().c_str(),
		     vr::TrackedDeviceClass_HMD,
		     hmd_.get() ) )
	{
		DriverLog( "psvr2: failed to add HMD device" );
		return vr::VRInitError_Driver_Unknown;
	}

	controllers_[0] = std::make_unique<Psvr2ControllerDriver>( true, hmd_.get() );
	controllers_[1] = std::make_unique<Psvr2ControllerDriver>( false, hmd_.get() );
	ScanControllers();
	RegisterControllers();

	// LED sync for optical controller tracking (experimental, default off).
	{
		vr::EVRSettingsError oerr = vr::VRSettingsError_None;
		const bool optical = vr::VRSettings()->GetBool( "driver_psvr2", "optical_tracking", &oerr );
		if ( oerr == vr::VRSettingsError_None && optical )
		{
			Psvr2ControllerDriver *const c[2] = { controllers_[0].get(), controllers_[1].get() };
			optical_ = std::make_unique<OpticalTracker>( c );
			optical_->Start();
		}
	}
	scanning_ = true;
	scan_thread_ = std::thread( &Psvr2DeviceProvider::ScanLoop, this );

	vr::EVRSettingsError err = vr::VRSettingsError_None;
	const bool ipc = vr::VRSettings()->GetBool( "driver_psvr2", "toolkit_ipc", &err );
	if ( err != vr::VRSettingsError_None || ipc )
	{
		ipc_ = std::make_unique<ToolkitIpcServer>(
			hmd_.get(), [this]( int hand, const TriggerEffect *fx ) { SetTriggerEffect( hand, fx ); } );
		ipc_->Start();
	}

	return vr::VRInitError_None;
}

void Psvr2DeviceProvider::Cleanup()
{
	// One log line per step: SteamVR aborts a driver that takes > 10 s to
	// clean up, and the last line shows which step hung.
	DriverLog( "psvr2: cleanup: scan thread" );
	scanning_ = false;
	if ( scan_thread_.joinable() )
		scan_thread_.join();
	DriverLog( "psvr2: cleanup: toolkit IPC" );
	ipc_ = nullptr;  // stops client threads before the controllers go
	DriverLog( "psvr2: cleanup: optical tracker" );
	optical_ = nullptr;  // sends LED ALL OFF through the controllers
	DriverLog( "psvr2: cleanup: controllers" );
	controllers_[0] = nullptr;
	controllers_[1] = nullptr;
	DriverLog( "psvr2: cleanup: headset" );
	hmd_ = nullptr;
	DriverLog( "psvr2: cleanup: done" );
}

void Psvr2DeviceProvider::SetTriggerEffect( int hand, const TriggerEffect *fx )
{
	for ( int i = 0; i < 2; i++ )
		if ( hand == 2 || hand == i )
			controllers_[i]->SetTriggerEffect( fx );
}

void Psvr2DeviceProvider::ScanControllers()
{
	if ( controllers_[0]->Attached() && controllers_[1]->Attached() )
		return;

	for ( const SenseHidInfo &info : FindSenseControllers() )
	{
		Psvr2ControllerDriver *c = controllers_[info.left ? 0 : 1].get();
		if ( !c->Attached() )
			c->Attach( info );
	}
}

void Psvr2DeviceProvider::ScanLoop()
{
	while ( scanning_ )
	{
		ScanControllers();
		for ( int i = 0; i < 20 && scanning_; i++ )
			std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
	}
}

void Psvr2DeviceProvider::RegisterControllers()
{
	for ( int i = 0; i < 2; i++ )
	{
		Psvr2ControllerDriver *c = controllers_[i].get();
		if ( c->Attached() && !controller_added_[i] )
			controller_added_[i] = vr::VRServerDriverHost()->TrackedDeviceAdded(
				c->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, c );
	}
}

const char *const *Psvr2DeviceProvider::GetInterfaceVersions()
{
	return vr::k_InterfaceVersions;
}

void Psvr2DeviceProvider::RunFrame()
{
	vr::VREvent_t event{};
	while ( vr::VRServerDriverHost()->PollNextEvent( &event, sizeof( event ) ) )
	{
		if ( event.eventType != vr::VREvent_Input_HapticVibration )
			continue;
		const vr::VREvent_HapticVibration_t &h = event.data.hapticVibration;
		for ( auto &c : controllers_ )
			if ( c->HapticHandle() == h.componentHandle )
				c->OnHaptic( h );
		if ( hmd_->HapticHandle() == h.componentHandle )
			hmd_->OnHaptic( h );
	}

	hmd_->RunFrame();
	RegisterControllers();
}

bool Psvr2DeviceProvider::ShouldBlockStandbyMode()
{
	return false;
}

void Psvr2DeviceProvider::EnterStandby() {}
void Psvr2DeviceProvider::LeaveStandby() {}
