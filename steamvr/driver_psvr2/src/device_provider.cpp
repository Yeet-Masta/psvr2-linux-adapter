// SPDX-License-Identifier: GPL-2.0
#include "device_provider.h"

#include "driverlog.h"

vr::EVRInitError Psvr2DeviceProvider::Init( vr::IVRDriverContext *pDriverContext )
{
	VR_INIT_SERVER_DRIVER_CONTEXT( pDriverContext );

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
	ipc_ = nullptr;  // stops client threads before the controllers go
	controllers_[0] = nullptr;
	controllers_[1] = nullptr;
	hmd_ = nullptr;
}

void Psvr2DeviceProvider::SetTriggerEffect( int hand, const TriggerEffect *fx )
{
	for ( int i = 0; i < 2; i++ )
		if ( hand == 2 || hand == i )
			controllers_[i]->SetTriggerEffect( fx );
}

void Psvr2DeviceProvider::ScanControllers()
{
	next_scan_ = std::chrono::steady_clock::now() + std::chrono::seconds( 2 );
	if ( controllers_[0]->Attached() && controllers_[1]->Attached() )
		return;

	for ( const SenseHidInfo &info : FindSenseControllers() )
	{
		const int i = info.left ? 0 : 1;
		Psvr2ControllerDriver *c = controllers_[i].get();
		if ( c->Attached() )
			continue;
		c->Attach( info );
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
	if ( std::chrono::steady_clock::now() >= next_scan_ )
		ScanControllers();
}

bool Psvr2DeviceProvider::ShouldBlockStandbyMode()
{
	return false;
}

void Psvr2DeviceProvider::EnterStandby() {}
void Psvr2DeviceProvider::LeaveStandby() {}
