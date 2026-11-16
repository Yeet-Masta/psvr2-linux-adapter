// SPDX-License-Identifier: GPL-2.0
#include "toolkit_ipc.h"

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "driverlog.h"
#include "hmd_device_driver.h"

namespace
{
constexpr uint16_t kPort = 3364;
constexpr uint16_t kServerVersion = 3;
constexpr int32_t kMaxPayload = 1024;

enum Command : uint16_t
{
	ClientPing,
	ServerPong,
	ClientRequestHandshake,
	ServerHandshakeResult,
	ClientRequestGazeData,
	ServerGazeDataResult,
	ClientTriggerEffectOff,
	ClientTriggerEffectFeedback,
	ClientTriggerEffectWeapon,
	ClientTriggerEffectVibration,
	ClientTriggerEffectMultiplePositionFeedback,
	ClientTriggerEffectSlopeFeedback,
	ClientTriggerEffectMultiplePositionVibration,
};

enum HandshakeResult : uint8_t
{
	HandshakeFailed,
	HandshakeSuccess,
	HandshakeOutdated,
};

// Natural (unpacked) alignment, as in Toolkit's ipc_protocol.h.
struct Header
{
	uint16_t type;
	int32_t data_len;
};
static_assert( sizeof( Header ) == 8, "IPC header" );

struct HandshakeRequest
{
	uint16_t ipc_version;
	uint32_t process_id;
};

struct HandshakeResponse
{
	uint8_t result;
	uint16_t ipc_version;
};
static_assert( sizeof( HandshakeResponse ) == 4, "IPC handshake response" );

#pragma pack( push, 1 )
struct Vec2
{
	float x, y;
};
struct Vec3
{
	float x, y, z;
};
struct EyeV1
{
	bool origin_valid;
	Vec3 origin_mm;
	bool dir_valid;
	Vec3 dir;
	bool pupil_valid;
	float pupil_mm;
	bool blink_valid;
	bool blink;
};
struct EyeV2
{
	EyeV1 v1;
	bool open_enabled;
	float open;
};
struct EyeV3
{
	bool origin_valid;
	Vec3 origin_mm;
	bool dir_valid;
	Vec3 dir;
	bool pupil_valid;
	float pupil_mm;
	bool pupil_pos_valid;
	Vec2 pupil_pos;
	bool pos_guide_valid;
	Vec2 pos_guide;
	bool blink_valid;
	bool blink;
	bool open_enabled;
	float open;
};
#pragma pack( pop )
static_assert( sizeof( EyeV1 ) == 33 && sizeof( EyeV2 ) == 38 && sizeof( EyeV3 ) == 56, "IPC gaze layout" );

bool SendAll( int fd, const void *buf, size_t len )
{
	const auto *p = static_cast<const uint8_t *>( buf );
	while ( len )
	{
		const ssize_t n = send( fd, p, len, MSG_NOSIGNAL );
		if ( n <= 0 )
		{
			if ( n < 0 && errno == EINTR )
				continue;
			return false;
		}
		p += n;
		len -= size_t( n );
	}
	return true;
}

bool RecvAll( int fd, void *buf, size_t len )
{
	auto *p = static_cast<uint8_t *>( buf );
	while ( len )
	{
		const ssize_t n = recv( fd, p, len, 0 );
		if ( n <= 0 )
		{
			if ( n < 0 && errno == EINTR )
				continue;
			return false;
		}
		p += n;
		len -= size_t( n );
	}
	return true;
}

bool Send( int fd, uint16_t type, const void *data = nullptr, int32_t len = 0 )
{
	uint8_t buf[sizeof( Header ) + kMaxPayload] = {};
	Header h{ type, data ? len : 0 };
	std::memcpy( buf, &h, sizeof( h ) );
	if ( data && len > 0 )
		std::memcpy( buf + sizeof( h ), data, size_t( len ) );
	return SendAll( fd, buf, sizeof( h ) + size_t( h.data_len ) );
}

EyeV3 ToWire( const Psvr2EyeData &e )
{
	EyeV3 w{};
	w.origin_valid = e.valid && e.origin_valid;
	w.origin_mm = { e.origin_mm[0], e.origin_mm[1], e.origin_mm[2] };
	w.dir_valid = e.valid && e.dir_valid;
	w.dir = { e.dir[0], e.dir[1], e.dir[2] };
	w.pupil_valid = e.valid && e.pupil_valid;
	w.pupil_mm = e.pupil_mm;
	w.blink_valid = e.valid && e.blink_valid;
	w.blink = e.blink;
	w.open_enabled = e.valid && e.blink_valid;
	w.open = e.blink ? 0.0f : 1.0f;
	return w;
}

EyeV1 ToV1( const EyeV3 &e )
{
	return { e.origin_valid, e.origin_mm, e.dir_valid, e.dir, e.pupil_valid, e.pupil_mm, e.blink_valid, e.blink };
}

// Toolkit trigger command payloads all start with the hand byte, followed by
// bytes in the order listed in ipc_protocol.h.
bool ParseTrigger( uint16_t type, const uint8_t *d, int32_t len, int &hand, TriggerEffect &fx )
{
	auto need = [&]( int32_t n ) { return len >= n; };
	if ( !need( 1 ) )
		return false;
	hand = d[0];
	switch ( type )
	{
	case ClientTriggerEffectOff:
		fx.kind = TriggerEffectKind::Off;
		return true;
	case ClientTriggerEffectFeedback:
		if ( !need( 3 ) )
			return false;
		fx.kind = TriggerEffectKind::Feedback;
		fx.start = d[1];
		fx.strength = d[2];
		return true;
	case ClientTriggerEffectWeapon:
		if ( !need( 4 ) )
			return false;
		fx.kind = TriggerEffectKind::Weapon;
		fx.start = d[1];
		fx.end = d[2];
		fx.strength = d[3];
		return true;
	case ClientTriggerEffectVibration:
		if ( !need( 4 ) )
			return false;
		fx.kind = TriggerEffectKind::Vibration;
		fx.start = d[1];
		fx.strength = d[2];
		fx.frequency = d[3];
		return true;
	case ClientTriggerEffectMultiplePositionFeedback:
		if ( !need( 11 ) )
			return false;
		fx.kind = TriggerEffectKind::MultiplePositionFeedback;
		std::copy( d + 1, d + 11, fx.zones.begin() );
		return true;
	case ClientTriggerEffectSlopeFeedback:
		if ( !need( 5 ) )
			return false;
		fx.kind = TriggerEffectKind::SlopeFeedback;
		fx.start = d[1];
		fx.end = d[2];
		fx.strength = d[3];
		fx.end_strength = d[4];
		return true;
	case ClientTriggerEffectMultiplePositionVibration:
		if ( !need( 12 ) )
			return false;
		fx.kind = TriggerEffectKind::MultiplePositionVibration;
		fx.frequency = d[1];
		std::copy( d + 2, d + 12, fx.zones.begin() );
		return true;
	}
	return false;
}
} // namespace

ToolkitIpcServer::ToolkitIpcServer( Psvr2HmdDriver *hmd, TriggerSink triggers )
	: hmd_( hmd ), triggers_( std::move( triggers ) )
{
}

ToolkitIpcServer::~ToolkitIpcServer()
{
	Stop();
}

bool ToolkitIpcServer::Start()
{
	listen_fd_ = socket( AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0 );
	if ( listen_fd_ < 0 )
		return false;
	int one = 1;
	setsockopt( listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof( one ) );

	// Loopback only: gaze data stays on this machine.
	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl( INADDR_LOOPBACK );
	addr.sin_port = htons( kPort );
	if ( bind( listen_fd_, reinterpret_cast<sockaddr *>( &addr ), sizeof( addr ) ) < 0 || listen( listen_fd_, 8 ) < 0 )
	{
		DriverLog( "psvr2: Toolkit IPC: cannot listen on 127.0.0.1:%u (%s); another PSVR2 Toolkit running?", kPort,
		           strerror( errno ) );
		close( listen_fd_ );
		listen_fd_ = -1;
		return false;
	}
	running_ = true;
	accept_thread_ = std::thread( &ToolkitIpcServer::AcceptLoop, this );
	DriverLog( "psvr2: PSVR2 Toolkit IPC server on 127.0.0.1:%u", kPort );
	return true;
}

void ToolkitIpcServer::Stop()
{
	if ( !running_.exchange( false ) )
		return;
	shutdown( listen_fd_, SHUT_RDWR );
	close( listen_fd_ );
	if ( accept_thread_.joinable() )
		accept_thread_.join();

	std::vector<std::thread> threads;
	{
		std::lock_guard<std::mutex> lock( clients_mutex_ );
		for ( int fd : client_fds_ )
			shutdown( fd, SHUT_RDWR );
		threads.swap( client_threads_ );
	}
	for ( auto &t : threads )
		if ( t.joinable() )
			t.join();
}

void ToolkitIpcServer::AcceptLoop()
{
	while ( running_ )
	{
		const int fd = accept4( listen_fd_, nullptr, nullptr, SOCK_CLOEXEC );
		if ( fd < 0 )
		{
			if ( errno == EINTR )
				continue;
			break;
		}
		std::lock_guard<std::mutex> lock( clients_mutex_ );
		client_fds_.push_back( fd );
		client_threads_.emplace_back( &ToolkitIpcServer::ClientLoop, this, fd );
	}
}

void ToolkitIpcServer::ClientLoop( int fd )
{
	uint16_t version = 0;  // 0 until the handshake succeeds
	bool set_triggers = false;
	uint8_t payload[kMaxPayload];

	while ( running_ )
	{
		Header h;
		if ( !RecvAll( fd, &h, sizeof( h ) ) || h.data_len < 0 || h.data_len > kMaxPayload ||
		     !RecvAll( fd, payload, size_t( h.data_len ) ) )
			break;
		if ( h.type >= ClientTriggerEffectOff && version )
			set_triggers = true;
		Handle( fd, h.type, payload, h.data_len, version );
	}

	// Don't leave a crashed game's trigger resistance behind.
	if ( set_triggers )
		triggers_( 2, TriggerEffect{} );

	std::lock_guard<std::mutex> lock( clients_mutex_ );
	client_fds_.erase( std::remove( client_fds_.begin(), client_fds_.end(), fd ), client_fds_.end() );
	close( fd );
}

void ToolkitIpcServer::Handle( int fd, uint16_t type, const uint8_t *data, int32_t len, uint16_t &version )
{
	switch ( type )
	{
	case ClientPing:
		if ( version )
			Send( fd, ServerPong );
		return;

	case ClientRequestHandshake:
	{
		HandshakeResponse r{ HandshakeFailed, kServerVersion };
		if ( !version && len >= int32_t( sizeof( HandshakeRequest ) ) )
		{
			HandshakeRequest req;
			std::memcpy( &req, data, sizeof( req ) );
			if ( req.ipc_version > kServerVersion )
				r.result = HandshakeOutdated;
			else if ( req.ipc_version >= 1 )
			{
				version = req.ipc_version;
				r.result = HandshakeSuccess;
				DriverLog( "psvr2: Toolkit IPC client pid %u connected (IPC v%u)", req.process_id, version );
			}
		}
		Send( fd, ServerHandshakeResult, &r, sizeof( r ) );
		return;
	}

	case ClientRequestGazeData:
		if ( version )
			SendGaze( fd, version );
		return;

	default:
	{
		int hand = 2;
		TriggerEffect fx;
		if ( version && ParseTrigger( type, data, len, hand, fx ) && hand >= 0 && hand <= 2 )
			triggers_( hand, fx );
		return;
	}
	}
}

void ToolkitIpcServer::SendGaze( int fd, uint16_t version )
{
	const Psvr2GazeData g = hmd_ ? hmd_->GetGaze() : Psvr2GazeData{};
	const EyeV3 l = ToWire( g.left ), r = ToWire( g.right );

	if ( version == 1 )
	{
		const EyeV1 both[2] = { ToV1( l ), ToV1( r ) };
		Send( fd, ServerGazeDataResult, both, sizeof( both ) );
	}
	else if ( version == 2 )
	{
		const EyeV2 both[2] = { { ToV1( l ), l.open_enabled, l.open }, { ToV1( r ), r.open_enabled, r.open } };
		Send( fd, ServerGazeDataResult, both, sizeof( both ) );
	}
	else
	{
		const EyeV3 both[2] = { l, r };
		Send( fd, ServerGazeDataResult, both, sizeof( both ) );
	}
}
