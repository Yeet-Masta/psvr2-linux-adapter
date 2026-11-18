# Topic: headset (`docs/sony-re/headset.md`) — priority 1

Read `common.md` first. This topic matters most for the kernel module.

Scope: everything the driver does with the headset over USB. That covers
connect, interface/alt-setting selection, init order, every control report
sent and when, the bulk reader threads and their error/restart handling,
status stream decoding, proximity/sleep/power, see-through and black screen,
firmware version checks and update path, disconnect and shutdown.

Entry points:
- USB transport `0x180122000–0x180128fff`: control transfer `FUN_1801283f0`,
  bulk read `FUN_180127d60` and its callers, alt setting `FUN_1801256c0`,
  status `FUN_1801268d0`, firmware `FUN_180123130`.
- Headset state and modes: `FUN_180132890`, `FUN_180131990`, `FUN_180134100`,
  `FUN_180134730`, `FUN_180133960` (`trace.py find 'VrTracker2\] status info'`).
- HMD device activate/deactivate in `0x1801a0000–0x1801affff`
  (`trace.py find 'power|standby|PSVR2Driver'`).

Deliver: the full startup sequence as numbered steps with every report and
payload; a table of every control report (direction, ID, payload, when
sent, who sends it); the status-stream fields and what the driver does on
each change; the power/proximity/see-through state machine; error, timeout
and reconnect behaviour; and the firmware info reports (`0x81`, `0x8f`,
`0xa1`). Mark which parts belong in the kernel module.

Skip: map save/load and play area (topic room), SteamVR properties (topic
display).
