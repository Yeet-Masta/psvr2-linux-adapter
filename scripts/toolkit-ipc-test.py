#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Exercise the SteamVR driver's PSVR2 Toolkit IPC server (127.0.0.1:3364).

Run while SteamVR is up with the psvr2 driver:
    scripts/toolkit-ipc-test.py gaze          # print eye tracking for 5 s
    scripts/toolkit-ipc-test.py triggers      # cycle adaptive trigger effects
"""
import socket
import struct
import sys
import time

PORT = 3364
(PING, PONG, HANDSHAKE, HANDSHAKE_RESULT, GAZE, GAZE_RESULT,
 TE_OFF, TE_FEEDBACK, TE_WEAPON, TE_VIBRATION, TE_MULTI_FEEDBACK,
 TE_SLOPE, TE_MULTI_VIBRATION) = range(13)
BOTH = 2
# One eye, IPC v3 (packed): origin valid, origin mm, dir valid, dir, pupil
# valid, pupil mm, pupil pos valid + xy, pos guide valid + xy, blink valid,
# blink, open enabled, open.
EYE_V3 = struct.Struct("<?3f?3f?f?2f?2f???f")


def send(sock, cmd, payload=b""):
    sock.sendall(struct.pack("<HxxI", cmd, len(payload)) + payload)


def recv(sock):
    hdr = b""
    while len(hdr) < 8:
        hdr += sock.recv(8 - len(hdr))
    cmd, n = struct.unpack("<HxxI", hdr)
    data = b""
    while len(data) < n:
        data += sock.recv(n - len(data))
    return cmd, data


def connect():
    sock = socket.create_connection(("127.0.0.1", PORT), timeout=2)
    send(sock, HANDSHAKE, struct.pack("<HxxI", 3, 0))
    cmd, data = recv(sock)
    result, version = struct.unpack("<BxH", data)
    if cmd != HANDSHAKE_RESULT or result != 1:
        sys.exit(f"handshake failed: result {result}, server IPC v{version}")
    print(f"connected, server IPC v{version}")
    return sock


def gaze(sock):
    end = time.time() + 5
    while time.time() < end:
        send(sock, GAZE)
        _, data = recv(sock)
        for name, off in (("L", 0), ("R", EYE_V3.size)):
            e = EYE_V3.unpack_from(data, off)
            print(f"{name}: dir {'%+.2f %+.2f %+.2f' % e[5:8] if e[4] else '   (invalid)    '} "
                  f"pupil {e[9]:.1f} mm blink {e[17]}", end="   ")
        print()
        time.sleep(0.2)


def triggers(sock):
    steps = [
        ("feedback from position 3, strength 8", TE_FEEDBACK, bytes([BOTH, 3, 8])),
        ("weapon 2..6, strength 8", TE_WEAPON, bytes([BOTH, 2, 6, 8])),
        ("vibration from 0, amplitude 8, 40 Hz", TE_VIBRATION, bytes([BOTH, 0, 8, 40])),
        ("slope feedback 0..9, 1..8", TE_SLOPE, bytes([BOTH, 0, 9, 1, 8])),
        ("off", TE_OFF, bytes([BOTH])),
    ]
    for label, cmd, payload in steps:
        print(f"{label}: pull the triggers")
        send(sock, cmd, payload)
        time.sleep(4)


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "gaze"
    s = connect()
    {"gaze": gaze, "triggers": triggers}[mode](s)
