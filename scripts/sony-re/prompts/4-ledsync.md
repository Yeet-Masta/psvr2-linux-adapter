# Topic: LED sync and camera (`docs/sony-re/ledsync.md`) — priority 4

Read `common.md` first. The wire format is already in
`docs/sony-driver-re.md` ("LED sync", "LED detection"). Earlier notes:
`docs/sony-re/irtracker.md`.

Scope: how the headset cameras and controller LEDs are kept in step. That
covers the LedSyncer state machine (`0x180190000+`; `FUN_180194af0`,
`FUN_1801975e0`, `FUN_180198240`); which LEDs each phase lights (table at
`0x180260aa0` and the 0x60-byte side tables before each LED model); how
camera mode (`0x0b`), LED-detection config (`0x14`/`0x09`) and brightness
(`0x12`) are chosen and when they are sent; how the headset frame clock
maps to controller time; the failure and recovery paths; and the "LD"
packet consumption in `FUN_18018aaf0`, up to the point where blobs reach
the tracker.

Deliver: the sync sequence from controller connect to locked tracking, with
every command and the timing; the state machine; the phase/LED tables
(document the layout and read the values with a script, but do not paste
Sony's table into the doc); and the clamps and constants.

Skip: the pose solver itself (topic tracking).
