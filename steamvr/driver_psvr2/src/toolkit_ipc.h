// SPDX-License-Identifier: GPL-2.0
// A server for PSVR2 Toolkit's legacy IPC protocol (TCP 127.0.0.1:3364), so
// software written against PSVR2 Toolkit (face-tracking modules, game mods)
// gets eye tracking and adaptive triggers from this driver too. Wire format:
// projects/psvr2_toolkit_legacy_ipc_server/ipc_protocol.h in
// https://github.com/BnuuySolutions/PSVR2Toolkit (MIT), IPC versions 1..3.
#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "trigger_effect.h"

class Psvr2HmdDriver;

class ToolkitIpcServer
{
public:
	// hand: 0 left, 1 right, 2 both.
	using TriggerSink = std::function<void( int hand, const TriggerEffect &fx )>;

	ToolkitIpcServer( Psvr2HmdDriver *hmd, TriggerSink triggers );
	~ToolkitIpcServer();

	bool Start();
	void Stop();

private:
	void AcceptLoop();
	void ClientLoop( int fd );
	void Handle( int fd, uint16_t type, const uint8_t *data, int32_t len, uint16_t &version );
	void SendGaze( int fd, uint16_t version );

	Psvr2HmdDriver *const hmd_;
	TriggerSink triggers_;
	std::atomic<bool> running_{ false };
	int listen_fd_ = -1;
	std::thread accept_thread_;

	std::mutex clients_mutex_;
	std::vector<int> client_fds_;
	std::vector<std::thread> client_threads_;
};
