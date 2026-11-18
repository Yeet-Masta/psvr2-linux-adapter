# Topic: Sense controllers (`docs/sony-re/sense.md`) — priority 3

Read `common.md` first. Earlier notes: `docs/sony-re/libpad.md`.

Scope: Sony's libpad (`0x1801b0000–0x1801dffff`) and the controller manager
that uses it (`FUN_18011bae0`, `FUN_18011d0c0`). That covers enumeration
over USB and Bluetooth, open/close, every input report field (buttons,
sticks, triggers, touch/proximity, IMU, timestamps, battery), every output
report byte (vibration modes, PCM haptics, adaptive trigger effects with
their parameter encodings, status LED, LED-sync block), feature reports,
calibration (already decoded: just reference it), the `[HID_SYNC]` clock
fit (`FUN_1801c07a0`, `FUN_1801c6ee0`), the CRC/sequence rules, and
connect/disconnect and firmware version handling.

Deliver: complete input and output report tables with units; trigger
effect encodings; the haptics path (sample rate and buffering); the clock
fit as pseudo-C with constants. Compare against
`steamvr/driver_psvr2/src/sense_protocol.h` and list every mismatch.

Skip: DS4/DualSense-only paths (one line each), global locking, the shared
memory used with the PS VR2 app (one paragraph).
