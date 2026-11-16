// SPDX-License-Identifier: GPL-2.0
#include "camera_component.h"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <poll.h>
#include <unistd.h>

#include "driverlog.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
// Assumed geometry until the cameras are calibrated: ~100 degree pinhole per
// view, cameras on the front underside, 7 cm apart, tilted down 15 degrees.
constexpr double kFovDeg = 100.0;
constexpr double kBaseline = 0.07;
constexpr double kDown = -0.035, kForward = -0.06, kTiltDeg = -15.0;

double Now()
{
	using namespace std::chrono;
	return duration<double>( steady_clock::now().time_since_epoch() ).count();
}

double Focal()
{
	return ( Psvr2CameraComponent::kViewSize / 2.0 ) / std::tan( kFovDeg * 0.5 * kPi / 180.0 );
}

vr::HmdMatrix34_t CameraToHead( int camera )
{
	const double t = kTiltDeg * kPi / 180.0;
	vr::HmdMatrix34_t m{};
	// Rotation about X by the tilt.
	m.m[0][0] = 1.0f;
	m.m[1][1] = float( std::cos( t ) );
	m.m[1][2] = float( -std::sin( t ) );
	m.m[2][1] = float( std::sin( t ) );
	m.m[2][2] = float( std::cos( t ) );
	m.m[0][3] = float( camera == 0 ? -kBaseline / 2 : kBaseline / 2 );
	m.m[1][3] = float( kDown );
	m.m[2][3] = float( kForward );
	return m;
}

vr::HmdMatrix34_t PoseMatrix( const vr::DriverPose_t &p )
{
	const vr::HmdQuaternion_t &q = p.qRotation;
	vr::HmdMatrix34_t m{};
	m.m[0][0] = float( 1 - 2 * ( q.y * q.y + q.z * q.z ) );
	m.m[0][1] = float( 2 * ( q.x * q.y - q.w * q.z ) );
	m.m[0][2] = float( 2 * ( q.x * q.z + q.w * q.y ) );
	m.m[1][0] = float( 2 * ( q.x * q.y + q.w * q.z ) );
	m.m[1][1] = float( 1 - 2 * ( q.x * q.x + q.z * q.z ) );
	m.m[1][2] = float( 2 * ( q.y * q.z - q.w * q.x ) );
	m.m[2][0] = float( 2 * ( q.x * q.z - q.w * q.y ) );
	m.m[2][1] = float( 2 * ( q.y * q.z + q.w * q.x ) );
	m.m[2][2] = float( 1 - 2 * ( q.x * q.x + q.y * q.y ) );
	for ( int i = 0; i < 3; i++ )
		m.m[i][3] = float( p.vecPosition[i] );
	return m;
}
} // namespace

Psvr2CameraComponent::Psvr2CameraComponent( std::function<vr::DriverPose_t()> pose ) : pose_( std::move( pose ) ) {}

Psvr2CameraComponent::~Psvr2CameraComponent()
{
	StopVideoStream();
}

void Psvr2CameraComponent::SetProperties( vr::PropertyContainerHandle_t c )
{
	auto *props = vr::VRProperties();
	props->SetBoolProperty( c, vr::Prop_HasCamera_Bool, true );
	props->SetBoolProperty( c, vr::Prop_HasCameraComponent_Bool, true );
	props->SetInt32Property( c, vr::Prop_NumCameras_Int32, 2 );
	props->SetInt32Property( c, vr::Prop_CameraFrameLayout_Int32,
	                         vr::EVRTrackedCameraFrameLayout_Stereo | vr::EVRTrackedCameraFrameLayout_HorizontalLayout );
	props->SetInt32Property( c, vr::Prop_CameraStreamFormat_Int32, vr::CVS_FORMAT_RGB24 );
	vr::HmdMatrix34_t both[2] = { CameraToHead( 0 ), CameraToHead( 1 ) };
	props->SetProperty( c, vr::Prop_CameraToHeadTransform_Matrix34, &both[0], sizeof( both[0] ),
	                    vr::k_unHmdMatrix34PropertyTag );
	props->SetProperty( c, vr::Prop_CameraToHeadTransforms_Matrix34_Array, both, sizeof( both ),
	                    vr::k_unHmdMatrix34PropertyTag );
}

std::string Psvr2CameraComponent::FindDevice()
{
	for ( int i = 0; i < 64; i++ )
	{
		std::ifstream f( "/sys/class/video4linux/video" + std::to_string( i ) + "/name" );
		std::string name;
		if ( f && std::getline( f, name ) && name == "psvr2-camera" )
			return "/dev/video" + std::to_string( i );
	}
	return {};
}

bool Psvr2CameraComponent::GetCameraFrameDimensions( vr::ECameraVideoStreamFormat fmt, uint32_t *w, uint32_t *h )
{
	if ( fmt != vr::CVS_FORMAT_RGB24 )
		return false;
	*w = kWidth;
	*h = kHeight;
	return true;
}

bool Psvr2CameraComponent::GetCameraFrameBufferingRequirements( int *queue, uint32_t *size )
{
	*queue = 3;
	*size = kWidth * kHeight * 3;
	return true;
}

bool Psvr2CameraComponent::SetCameraFrameBuffering( int count, void **buffers, uint32_t size )
{
	if ( count <= 0 || !buffers || size < kWidth * kHeight * 3 )
		return false;
	std::lock_guard<std::mutex> lock( mutex_ );
	buffers_.assign( buffers, buffers + count );
	buffer_size_ = size;
	return true;
}

bool Psvr2CameraComponent::SetCameraVideoStreamFormat( vr::ECameraVideoStreamFormat fmt )
{
	return fmt == vr::CVS_FORMAT_RGB24;
}

vr::ECameraVideoStreamFormat Psvr2CameraComponent::GetCameraVideoStreamFormat()
{
	return vr::CVS_FORMAT_RGB24;
}

bool Psvr2CameraComponent::StartVideoStream()
{
	if ( streaming_ )
		return true;
	{
		std::lock_guard<std::mutex> lock( mutex_ );
		if ( buffers_.empty() )
			return false;
		have_frame_ = false;
	}
	start_time_ = Now();
	paused_ = false;
	streaming_ = true;
	thread_ = std::thread( &Psvr2CameraComponent::StreamThread, this );
	return true;
}

void Psvr2CameraComponent::StopVideoStream()
{
	streaming_ = false;
	if ( thread_.joinable() )
		thread_.join();
}

bool Psvr2CameraComponent::IsVideoStreamActive( bool *paused, float *elapsed )
{
	if ( paused )
		*paused = paused_;
	if ( elapsed )
		*elapsed = streaming_ ? float( Now() - start_time_ ) : 0.0f;
	return streaming_;
}

const vr::CameraVideoStreamFrame_t *Psvr2CameraComponent::GetVideoStreamFrame()
{
	std::lock_guard<std::mutex> lock( mutex_ );
	return have_frame_ ? &frame_ : nullptr;
}

void Psvr2CameraComponent::ReleaseVideoStreamFrame( const vr::CameraVideoStreamFrame_t * ) {}

bool Psvr2CameraComponent::SetAutoExposure( bool )
{
	return true;
}

bool Psvr2CameraComponent::PauseVideoStream()
{
	paused_ = true;
	return true;
}

bool Psvr2CameraComponent::ResumeVideoStream()
{
	paused_ = false;
	return true;
}

bool Psvr2CameraComponent::GetCameraDistortion( uint32_t, float u, float v, float *out_u, float *out_v )
{
	*out_u = u;
	*out_v = v;
	return true;
}

bool Psvr2CameraComponent::GetCameraProjection( uint32_t, vr::EVRTrackedCameraFrameType, float znear, float zfar,
                                                vr::HmdMatrix44_t *p )
{
	const float t = float( std::tan( kFovDeg * 0.5 * kPi / 180.0 ) );
	*p = {};
	p->m[0][0] = 1.0f / t;
	p->m[1][1] = 1.0f / t;
	p->m[2][2] = -zfar / ( zfar - znear );
	p->m[2][3] = -zfar * znear / ( zfar - znear );
	p->m[3][2] = -1.0f;
	return true;
}

bool Psvr2CameraComponent::SetFrameRate( int, int )
{
	return true;
}

bool Psvr2CameraComponent::SetCameraVideoSinkCallback( vr::ICameraVideoSinkCallback *callback )
{
	std::lock_guard<std::mutex> lock( mutex_ );
	sink_ = callback;
	return true;
}

bool Psvr2CameraComponent::GetCameraCompatibilityMode( vr::ECameraCompatibilityMode *mode )
{
	*mode = vr::CAMERA_COMPAT_MODE_BULK_DEFAULT;
	return true;
}

bool Psvr2CameraComponent::SetCameraCompatibilityMode( vr::ECameraCompatibilityMode mode )
{
	return mode == vr::CAMERA_COMPAT_MODE_BULK_DEFAULT;
}

bool Psvr2CameraComponent::GetCameraFrameBounds( vr::EVRTrackedCameraFrameType, uint32_t *left, uint32_t *top,
                                                 uint32_t *w, uint32_t *h )
{
	*left = 0;
	*top = 0;
	*w = kWidth;
	*h = kHeight;
	return true;
}

bool Psvr2CameraComponent::GetCameraIntrinsics( uint32_t, vr::EVRTrackedCameraFrameType, vr::HmdVector2_t *focal,
                                                vr::HmdVector2_t *center, vr::EVRDistortionFunctionType *distortion,
                                                double coefficients[vr::k_unMaxDistortionFunctionParameters] )
{
	const float f = float( Focal() );
	*focal = { f, f };
	*center = { kViewSize / 2.0f, kViewSize / 2.0f };
	*distortion = vr::VRDistortionFunctionType_None;
	for ( uint32_t i = 0; i < vr::k_unMaxDistortionFunctionParameters; i++ )
		coefficients[i] = 0.0;
	return true;
}

void Psvr2CameraComponent::StreamThread()
{
	const std::string path = FindDevice();
	const int fd = path.empty() ? -1 : open( path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC );
	if ( fd < 0 )
	{
		DriverLog( "psvr2: camera: cannot open %s (%s)", path.empty() ? "psvr2-camera" : path.c_str(),
		           strerror( errno ) );
		streaming_ = false;
		return;
	}
	DriverLog( "psvr2: camera: streaming from %s", path.c_str() );

	std::vector<uint8_t> grey( kWidth * kHeight );
	uint32_t sequence = 0;
	size_t next = 0;
	while ( streaming_ )
	{
		pollfd pfd{ fd, POLLIN, 0 };
		if ( poll( &pfd, 1, 100 ) <= 0 )
			continue;
		const ssize_t n = read( fd, grey.data(), grey.size() );
		if ( n != ssize_t( grey.size() ) )
		{
			if ( n < 0 && errno != EAGAIN && errno != EINTR )
			{
				DriverLog( "psvr2: camera: read failed (%s)", strerror( errno ) );
				break;
			}
			continue;
		}
		if ( paused_ )
			continue;

		const vr::DriverPose_t head = pose_();
		vr::ICameraVideoSinkCallback *sink;
		{
			std::lock_guard<std::mutex> lock( mutex_ );
			if ( buffers_.empty() )
				continue;
			next = ( next + 1 ) % buffers_.size();
			auto *rgb = static_cast<uint8_t *>( buffers_[next] );
			for ( size_t i = 0; i < grey.size(); i++ )
				rgb[3 * i] = rgb[3 * i + 1] = rgb[3 * i + 2] = grey[i];

			const double now = Now();
			frame_ = {};
			frame_.m_nStreamFormat = vr::CVS_FORMAT_RGB24;
			frame_.m_nWidth = kWidth;
			frame_.m_nHeight = kHeight;
			frame_.m_nImageDataSize = kWidth * kHeight * 3;
			frame_.m_nFrameSequence = sequence++;
			frame_.m_nBufferIndex = uint32_t( next );
			frame_.m_nBufferCount = uint32_t( buffers_.size() );
			frame_.m_flFrameElapsedTime = now - start_time_;
			frame_.m_flFrameCaptureTime_ServerRelative = now;
			frame_.m_RawTrackedDevicePose.mDeviceToAbsoluteTracking = PoseMatrix( head );
			frame_.m_RawTrackedDevicePose.bPoseIsValid = head.poseIsValid;
			frame_.m_RawTrackedDevicePose.bDeviceIsConnected = head.deviceIsConnected;
			frame_.m_RawTrackedDevicePose.eTrackingResult = head.result;
			frame_.m_pImageData = reinterpret_cast<uint64_t>( rgb );
			have_frame_ = true;
			sink = sink_;
		}
		if ( sink )
			sink->OnCameraVideoSinkCallback();
	}
	close( fd );
	streaming_ = false;
}
