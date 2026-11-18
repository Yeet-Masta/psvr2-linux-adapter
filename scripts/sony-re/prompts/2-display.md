# Topic: display and SteamVR device (`docs/sony-re/display.md`) — priority 2

Read `common.md` first. Earlier notes: `docs/sony-re/steamvr.md`.

Scope: what Sony reports to SteamVR and how it drives the display. That
covers EDID detection and 90/120 Hz selection (report `0x29`/`0xa9`,
`FUN_1801a4390`); display timing and vsync; render target size; lens
distortion model and its parameters, including where they come from (device
calibration or constants); IPD and eye-to-head transforms; pose reporting
(prediction time, velocity, frame of reference, tracking states); HMD and
controller property values; input components and controller bindings;
the hand skeleton and finger-curl mapping, at summary level.

Entry points: `FUN_1801a5760`, `FUN_1801a8680`, `FUN_18019d1b0`,
`FUN_18019ee30`, `FUN_1801a47c0`, `FUN_1801a4390`, `FUN_1801a02b0`
(debug page; shows which values matter). Use `trace.py find 'Prop_'`.

Deliver: a property table (property, value or formula, source); the
distortion model in pseudo-C with every constant; display-mode selection as
a sequence; the pose-output path with timing; and differences from
`steamvr/driver_psvr2/src`.

Skip: dialogs, localisation, settings UI.
