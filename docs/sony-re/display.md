# Sony PS VR2 driver: display and SteamVR devices

What Sony's SteamVR driver (`driver_playstation_vr2_orig.dll`, PE timestamp
`1741940190`, image base `0x180000000`) reports to SteamVR and how it drives
the display: EDID/refresh selection, timing, render target, lens model, IPD
and eye/head transforms, pose output, HMD and controller properties, input
components and haptics. Written from the decompile in our own words; addresses
are Ghidra VAs. Tags: **[sure]** read in code (or executed under Wine),
**[likely]**, **[guess]**.

Builds on [`steamvr.md`](steamvr.md) (reimplementation pass, difftested) and
[`headset.md`](headset.md) (USB control reports). This file restates what the
Linux driver needs, so it can be read alone, and adds what those notes lacked:
where the lens calibration comes from, the HMD driver-from-head transform
values, pose timing, the controller input-to-component map, battery
properties, haptic event handling, server init order and the driver INI.

## 1. Driver summary

What the Linux stack must do, in priority order:

1. **Refresh / EDID type (userspace, needs kernel ioctl).** Before SteamVR
   leases the display, Get feature `0xa9` (1 byte: 0 = 90/120 Hz EDID,
   1 = 90 Hz-only EDID). If SteamVR wants 120 Hz and the EDID is 90-only, Set
   `0x29` = 0; if SteamVR wants 90 Hz and the EDID is 90/120, Set `0x29` = 1.
   Then wait 1000 ms; the headset re-enumerates its DisplayPort sink. The
   kernel module should expose Get `0xa9` / Set `0x29` (it has neither today).
2. **Lens calibration (kernel reads, userspace uses).** Get `0x8f` (256 bytes)
   at connect, only when the USB link is USB3 (descriptor `bcdUSB ≥ 0x300`).
   It feeds the lens centres and panel rotations (section 5.1), which drive
   `ComputeDistortion` and the hidden-area mesh. Version byte < 4 ⇒ fixed
   defaults.
3. **Display component values (userspace).** Panel 4000 × 2040, viewport per
   eye 2000 × 2040, recommended render target 3400 × 3468, fixed projection
   tangents, vsync-to-photons 1.36 ms / 1.02 ms and photons-to-vblank
   10.3 ms / 7.73 ms at 90 / 120 Hz.
4. **HMD pose (userspace).** Query the tracker at "now", report with
   `poseTimeOffset = (t_query − t_send)` (a few µs, negative), constant
   driver-from-head transform `q = (−0.5, −0.5, 0.5, 0.5)`,
   `t = (−0.09255, 0, 0.02080)` m. Poll every 1 ms while the headset is
   ready, 100 ms otherwise.
5. **IPD dial and proximity (kernel parses status, userspace reports).**
   Status-header byte 5 = IPD in mm → `UserIpdMeters`; byte 1 = proximity →
   boolean `/proximity`.
6. **Controller inputs, battery, haptics (userspace).** Map the per-controller
   input record to the 25 components (section 2.4); battery = level × 0.01;
   haptic events below amplitude 0.049 are ignored, durations are raised to
   at least 80 ms.

Kernel module: control reports `0x8f`, `0xa9`, `0x29`, status stream `0x88`
(IPD/proximity). Everything else (properties, lens model, pose packaging,
input mapping) is userspace in `steamvr/driver_psvr2`.

## 2. Wire and data formats

### 2.1 Control reports used by this topic [sure]

| Report | Dir | Size | When | Meaning | VA |
|--------|-----|------|------|---------|----|
| `0xa9` | Get | 1 | Server init, cleanup, direct-mode change | EDID type: 0 = 90/120 Hz, 1 = 90 Hz only; treated as 0 if the read fails | `FUN_180124970` |
| `0x29` | Set | 1 | Only when the type must change | New EDID type, same encoding. Followed by `Sleep(1000)` | `FUN_180127e80`, `FUN_1801a4390` |
| `0x8f` | Get | 256 | Headset connect (`FUN_1801256c0`), after `0x81`, `0xa1` and the descriptor; skipped if `bcdUSB < 0x300` | Display/lens calibration block (2.2). Kept at manager `+0x4a2c` and copied into the config manager (`FUN_18015e990`, at `+0x70` of its shared block) | `FUN_1801256c0`, `FUN_180124e30` |
| `0xa1` | Get | 4 | Connect | u32 compared against the plugin version at server init: if greater, Sony shows the "update the PC app" dialog and fails Init (returns 200) | `FUN_1801256c0`, `FUN_180125080`, `FUN_1801a2040` |
| `0x81` | Get | 0x54 | Connect | Firmware info; dword +0 = HMD software version (`> 0x06000000` enables 90/120 Hz in SteamVR); +0x34 board stage | `FUN_180124450`, `FUN_180124a00` |
| `0x27` | Set | 2 | `VREvent_QuitAcknowledged` from `vrmonitor.exe`/`steam.exe` | u16 `1000` | `FUN_180125610` |
| `0xa5` | Get | 0x60 | Debug page only | DisplayPort receiver status (layout in main doc) | `FUN_1801248d0`, `FUN_1801a02b0` |

### 2.2 Display calibration block (Get `0x8f`) [sure for used fields]

All little-endian. Only these fields are read by the SteamVR side
(`FUN_1801a1a00`):

| Offset | Type | Meaning |
|--------|------|---------|
| 0x00 | u8 | Version. < 4 ⇒ no calibration, use defaults |
| 0x08 | f32 | Left lens offset x, panel pixels [likely] |
| 0x0c | f32 | Left lens offset y |
| 0x10 | f32 | Right lens offset x |
| 0x14 | f32 | Right lens offset y |
| 0x18 | f32 | Left panel rotation, degrees |
| 0x1c | f32 | Right panel rotation, degrees |
| 0x20 | f32 | Horizontal separation term, pixels [likely] |

Sony re-reads the block from the manager each time Activate runs and skips the
recomputation when it is byte-identical to the last copy.

### 2.3 Status header bytes used here (`0x88`, first 32 bytes) [sure]

| Byte | Use |
|------|-----|
| 1 | Proximity (worn) → `/proximity` boolean, non-zero = true (`FUN_18019ee30`) |
| 5 | IPD dial in mm → `UserIpdMeters = byte · 0.001f`; 0 = not reported, leave the property alone |

### 2.4 Controller input record (`FUN_180118a10`, per side) [sure]

The controller manager hands the SteamVR device a record per controller. Field
offsets (relative to the record, from `FUN_1801a8680`):

| Offset | Type | Meaning |
|--------|------|---------|
| 0x00 | u32 | Button bits |
| 0x04 | u8 | Stick x (0..255, 128 centre) |
| 0x05 | u8 | Stick y (0..255, 0 = up) |
| 0x08 | u8 | Trigger (L2/R2) analog |
| 0x0c | u32 | Touch bits |
| 0x10 | u8 | L1/R1 capacitive sensor (grip proximity) |
| 0x12 | u8 | L2/R2 capacitive sensor (trigger proximity) |
| 0x40 | u8 | Connected |
| 0x5c | u8 | Lost (treated as disconnected for pose) |

Component update, per frame (`FUN_1801a8680`), when the controller is
connected and none of the four pause conditions holds (2.6):

| Component (left / right) | Left source | Right source | Value |
|--------------------------|-------------|--------------|-------|
| `square` / `cross` click | btn bit 15 | btn bit 14 | bool |
| `square` / `cross` touch | touch bit 15 | touch bit 14 | bool |
| `triangle` / `circle` click | btn bit 12 | btn bit 13 | bool |
| `triangle` / `circle` touch | touch bit 12 | touch bit 13 | bool |
| `l1` / `r1` value | btn bit 10 | btn bit 11 | scalar 1.0 or 0.0 (digital) |
| `l1` / `r1` touch | touch bit 10 | touch bit 11 | bool |
| `l1_sensor` / `r1_sensor` value | +0x10 | +0x10 | `byte / 255` |
| `l2` / `r2` value | +0x08 | +0x08 | `byte / 255` |
| `l2` / `r2` touch | touch bit 8 | touch bit 9 | bool |
| `l2_sensor` / `r2_sensor` value | +0x12 | +0x12 | `byte / 255` |
| `left_stick` / `right_stick` click | btn bit 1 | btn bit 2 | bool |
| `…_stick` touch | touch bit 1, **or** stick off centre | touch bit 2, or off centre | bool |
| `…_stick` x | +0x04 | +0x04 | `2·x/255 − 1`, or 0 when centred |
| `…_stick` y | +0x05 | +0x05 | `1 − 2·y/255`, or 0 when centred |
| `create` / `options` click | btn bit 0 | btn bit 3 | bool |
| `left_ps` / `right_ps` click | btn bit 16 | btn bit 16 | bool |
| `ps` touch, `create`/`options` touch | — | — | created but **never updated** (stay false) |

"Centred": `‖(x − c, y − c)‖ ≤ r`, with centre `c` and radius `r` from the
controller manager (`FUN_180118bf0`, per-controller deadzone) [sure]; the
test is only made when the record's connected byte is set. When paused or
disconnected every boolean is sent false and every scalar 0.

Bits are "libpad" bits (see `libpad.md`); left and right use different bit
numbers for the face buttons because the record is the shared DualSense-like
layout.

### 2.5 Haptic event (`VREvent_Input_HapticVibration`, 1700) [sure]

Handled in the controller frame (`FUN_1801a8680`) when the event's component
handle equals the controller's `/output/haptic` handle; passed to
`FUN_18011b4d0(side, duration_s, frequency_hz, amplitude)`:

```c
if (frequency <= 0 || amplitude <= 0) {           // stop request
    if (now_us - start_us < duration_s * 1e6) stop_flag = 1;
} else if ((double)amplitude >= 0.049) {
    duration_s = max(duration_s, 0.08f);
    store(duration_s, frequency, amplitude); stop_flag = 0;
    mode = (mode != 0) + 1;                       // 1 first pulse, 2 after
    start_us = now_us;
}                                                  // amplitude < 0.049: ignored
```

Rendering into the controller's vibration output report belongs to libpad
(`libpad.md`).

### 2.6 Controller pause conditions [sure]

Inputs, pose and skeleton are frozen/zeroed while any of these holds:
1. dialog manager pause (`FUN_180131af0`: see-through/boundary dialog);
2. server-driver flag `FUN_1801a47b0`;
3. headset firmware update needed (`FUN_180125480`);
4. this controller's firmware update needed (`FUN_1801193d0`).

## 3. Sequences

### 3.1 Server init (`ServerDriver::Init`, `FUN_1801a2040`) [sure unless tagged]

1. Get interfaces, log `Activating PlayStation VR2...`.
2. Start config manager, map/chaperone manager, "hmd module" (USB manager,
   `FUN_180125330`), "sense module" (controllers, `FUN_180119310`), tracker
   (`FUN_18018a7e0(2)`), relocaliser and dialogs.
3. **EDID choice** (`FUN_1801a4390`, 3.2), tag `ServerDriver::Init`.
4. **Wait for the headset**: loop until "HMD ready" (`FUN_1801253c0`: board
   stage neither 0 nor `0xffffffff`). Sleep 1000 ms per pass, **no timeout**.
   From the third pass on, log USB2 (`speed code 1..0x2ff`) or "not found".
5. Read driver INI (`FUN_1801a47c0`, section 6.2); log versions.
6. Version gate: Get `0xa1` value (cached) > plugin version ⇒ update dialog,
   Cleanup, return 200 (init failed).
7. Cache `direct_mode/enable` at `+0x8c`.
8. Create HMD device (0x3d0 bytes; panel 4000 × 2040) and add it as
   `playstation_vr2_headset`, class HMD; create and add
   `playstation_vr2_sense_controller_left` / `_right`, class controller.
9. Start the watchdog thread, log `PlayStation VR2 Loaded Successfully`.

### 3.2 EDID / refresh selection (`FUN_1801a4390`) [sure]

1. `rate = steamvr/preferredRefreshRate`. If `|rate−90| > 0.1` and
   `|rate−120| > 0.1`: rate = 90 and write 90 back. Store at
   `ServerDriver+0x88`.
2. `type = Get 0xa9` (0 on failure).
3. If `type == 1` and `|rate−120| ≤ 0.1`: Set `0x29` = 0, Sleep 1000 ms,
   changed = 1.
   Else if `type == 0` and `|rate−90| ≤ 0.1`: Set `0x29` = 1, Sleep 1000 ms,
   changed = 1.
4. Log rate and type; compare `direct_mode/enable` from IVRSettings with the
   value in `steamvr.vrsettings` read by file (`FUN_1801a5310`), log both.

Callers: Init, Cleanup (`FUN_1801a2a40`, so the EDID matches the setting for
the next session), and RunFrame on `VREvent_QuitAcknowledged` (3.4).

### 3.3 HMD Activate (`FUN_18019d1b0`) [sure]

Properties are written in the order of table 6.3; then: register with the
chaperone manager, lens geometry from the `0x8f` block (5.1), hidden-area
meshes (5.3), start the HMD update thread, finally
`DistortionMeshResolution = 80`.

### 3.4 RunFrame display-relevant steps (`FUN_1801a2cb0`) [sure]

- `VREvent_QuitAcknowledged` (703): if the quitting process is
  `vrmonitor.exe` or `steam.exe`, Set `0x27` u16 1000. Then re-read
  `direct_mode/enable`; re-run 3.2. If the setting changed and 3.2 did not
  switch the EDID, request a headset restart (`FUN_180127fa0(…, 1)`, Set
  `0x05`), logged `[DirectMode Check][WA] reboot`.
- Link state change: if the USB speed code is `1..0x2ff`, log the USB2
  warning.

### 3.5 HMD update pass (`FUN_18019ee30`, thread `FUN_1801a19a0`) [sure]

Loop period: 1 ms while the headset is ready, 100 ms otherwise.

1. IPD: if status byte 5 ≠ 0, write `UserIpdMeters = byte·0.001` (every pass).
2. `/proximity` = status byte 1 ≠ 0.
3. See-through dialog pause flag edge: set 2058
   `DriverRequestsApplicationPause`, 2057 `DriverIsDrawingControllers`, 2059
   `DriverRequestsReducedRendering` all true on entry, all false on exit.
4. `t_query = now_us()`; pose (section 5.4).
5. Not-worn throttle (5.5), submit with
   `TrackedDevicePoseUpdated(id, pose, 0x118)`.
6. Once when ready: 1006 `TrackingFirmwareVersion` (`%08x` of software
   version), 1007 `HardwareRevision_String` (board stage name), firmware
   update properties 1014/1015/1016 if needed.
7. Once a second: Windows audio endpoint ids (2300/2301). Linux: set to the
   PipeWire/ALSA node names if SteamVR on Linux uses them [guess].

### 3.6 Controller frame (`FUN_1801a8680`, per side, from RunFrame) [sure]

1. Events: haptic (2.5); `VREvent_SystemReport_Started` (1900) builds a text
   dump of controller link histograms into a string property (debug only).
2. Input components (2.4).
3. Battery properties: disconnected ⇒ 1026 `DeviceProvidesBatteryStatus` =
   false, 1012 `DeviceBatteryPercentage` = 0. Connected ⇒ 1026 = true,
   1012 = `clamp(level·0.01, 0, 1)`, 1011 `DeviceIsCharging` = status ∈ {1, 2}
   (`FUN_180118870` returns status and level byte).
4. On a change of the connected state (flag `+0x171`): 1006
   `TrackingFirmwareVersion` = `%04x` of controller firmware version
   (`FUN_180118900`), or `Unknown` if < 1; 1007 `HardwareRevision_String` =
   controller hardware string (`FUN_180118b10`) if any.
5. Once per connection, if this controller needs a firmware update:
   1015 = true, 1016 = `https://www.playstation.com/psvr2-pc-usage/`,
   1014 = true, and the update dialog.
6. Read `power/turnOffControllersTimeout` (SteamVR setting, int, default 0)
   and pass it to the controller manager (`FUN_18011d570`), which uses it as
   the controller auto-off time [likely].
7. Once a second: if the controller is disconnected, or idle more than 60 s
   (`FUN_180118b70` = µs since last input > 60 000 000), pulse
   `l1/touch` true then false [guess: keeps SteamVR from deciding the
   controller is idle, or forces an input-state refresh].

Controller pose and skeleton are sent from a separate thread (`FUN_1801aa660`:
1 ms loop while either controller is connected, else 100 ms), see 5.6.

## 4. State machines

### 4.1 EDID type

| State (Get `0xa9`) | SteamVR rate | Action |
|--------------------|-------------|--------|
| 1 (90 only) | 120 | Set `0x29`=0, wait 1 s ⇒ state 0 |
| 0 (90/120) | 90 | Set `0x29`=1, wait 1 s ⇒ state 1 |
| any | other | none |

So Sony never leaves a 120-capable EDID visible while running 90 Hz.

### 4.2 HMD pose report

| State | Condition | Report |
|-------|-----------|--------|
| Not ready | `FUN_1801253c0` false | cached last pose, `poseIsValid=1`, `deviceIsConnected=0` |
| Paused | see-through dialog | previous report (if it was connected), velocities zeroed |
| Tracking | ready, not paused | tracker pose, result 200 (1 if headset firmware is newer than the server knows, flag `+0x289`) |

Plus the not-worn throttle (5.5).

### 4.3 Controller pose report

Disconnected (connected byte 0 or lost byte ≠ 0) ⇒ all-zero pose, result 0.
Paused (2.6) ⇒ previous pose, velocities 0; condition 1 also clears
`poseIsValid`. Tracking ⇒ tracker pose with tip transform (5.6).

## 5. Algorithms

### 5.1 Lens geometry from the `0x8f` block (`FUN_1801a1a00`) [sure, difftested]

```c
struct lens { float cx, cy; float cos_r, sin_r; } L, R;

if (blk.version < 4) {
    L = (lens){ -0.0992f, 0, 0, 0 };   // 0xbdcb25aa; rotation (0,0), sic
    R = (lens){ +0.0992f, 0, 0, 0 };
} else {
    // pixel -> normalised lens coordinate; 29.9 um pitch, 32.462 mm half width
    norm(p, shift_mm) = ((p*29.9f + 14.95f) / 1000.0f + shift_mm) / 32.462f;
    L.cx = norm(-blk.lx - blk.sep, -3.22f);
    R.cx = norm(-blk.rx + blk.sep, +3.22f);
    L.cy = norm(-blk.ly, 0.0f);
    R.cy = norm(-blk.ry, 0.0f);
    a = (float)(-blk.lrot_deg * M_PI / 180.0);  L.cos_r = cosf(a); L.sin_r = sinf(a);
    a = (float)(-blk.rrot_deg * M_PI / 180.0);  R.cos_r = cosf(a); R.sin_r = sinf(a);
}
```

All single precision except the degree-to-radian step (double). Divisor
32.462 is the float `0x4201d914`.

### 5.2 Distortion (`ComputeDistortion`, `FUN_18019e860`) [sure, difftested]

Three measured radial tables, 1024 floats each, red/green/blue at
`0x180261180`, `0x180262180`, `0x180263180` (not closed-form; a degree-8 fit
is still 4·10⁻³ off). Per eye `e` with lens `G = L or R`:

```c
const float S = 0.93950f;   // 0x3f7082fd, lens radius scale
const float A = 50.0f/51;   // 0x3f7afafb, aspect (2000 x 2040 panel)
x = ((u+u) - 1) * A * S + G.cx;
y = G.cy + ((v+v) - 1) * S;
t = (y*y + x*x) * 1023;
i = max(0, (int)floorf(t));
for ch in R,G,B:
    if (i + 1 < 1024) k[ch] = f*T[ch][i+1] + (1-f)*T[ch][i];   // f = t - i
    else              k[ch] = (t - 1023) * slope[ch] + end[ch];
// slope R/G/B bits 0x3b6f1e00 0x3b9f5600 0x3bf35000
// end   R/G/B bits 0x3fcbd9db 0x3fd8b389 0x3febbf26
for ch:
    dy = (ch == B) ? (y + 0) * k : (y - 0.00023027f) * k;
    dx = (x + 0) * k;
    U = (G.cos_r*dx - dy*G.sin_r) * 0.35856f + ub[e];   // ub: 0.66038 left, 0.33962 right
    V = (dx*G.sin_r + G.cos_r*dy) * 0.37623f + 0.5f;
```

Constants: `0.35856 = 0x3eb794b5`, `0.37623 = 0x3ec0a0f8`,
`0.66038 = 0x3f290e95`, `0.33962 = 0x3eade2d7`. `ComputeInverseDistortion`
returns false. `DistortionMeshResolution = 80`.

The project's `psvr2_distort` already implements this and is bit-exact.

### 5.3 Hidden-area mesh (`FUN_1801a10c0`) [sure, difftested]

32 rays marched from r = 0.9 in steps of 0.004 through the green table until
`x²+y² > 1.06` or the distorted point leaves the panel; 36 triangles (108
vertices) per eye, left computed from `L`, right = left mirrored
(`x = 1 − x`). Written as 5100/5101 with tag 30. Full detail in
`steamvr.md` "Hidden-area mesh"; the implementation there is exact.

### 5.4 HMD pose (`FUN_18019ee30`, `FUN_180160230`) [sure unless tagged]

```c
t_query = now_us();                     // QPC -> microseconds (FUN_18016b110)
trk = tracker_pose(device 0, t_query);  // INS filter evaluated at t_query
p.vecPosition       = (double)trk.pos;        // world (tracking map) frame, m
p.qRotation         = (double)trk.q;          // (w,x,y,z), world-from-IMU-rig
p.vecVelocity       = (double)trk.vel;        // world frame
p.vecAngularVelocity= (double)trk.angvel;     // rotated into world frame by tracker
p.vecAcceleration = p.vecAngularAcceleration = 0;
p.qWorldFromDriverRotation = (1,0,0,0); p.vecWorldFromDriverTranslation = 0;
p.qDriverFromHeadRotation    = (-0.5, -0.5, 0.5, 0.5);        // (w,x,y,z)
p.vecDriverFromHeadTranslation = (-0.09255, 8e-9, 0.02080);   // m
p.result = headset_fw_newer ? 1 : 200;
p.poseIsValid = 1;  p.deviceIsConnected = ready;
...
t_send = now_us();
p.poseTimeOffset = (float)(t_query - t_send) * 0.001f * 0.001f;   // s, <= 0
```

Prediction: the tracker itself evaluates the filter at `t_query` (it may
extrapolate from the newest IMU sample to that time). SteamVR then predicts
forward from `t_send` minus the tiny offset. There is **no** extra latency
term; the IMU-to-now gap is handled inside the tracker [likely].

The driver-from-head transform is fixed rig geometry, built by the config
manager's defaults (`FUN_180155000` → `FUN_1801556b0`) from: IMU-rig frame at
`(0, −0.02774, −0.07843)` m rotated by Rx(−95°)·Rz(90°), and head frame at
`Rx(−5°)·(0, 0, 0.012)` rotated Rx(−5°). The numbers above were read by
running Sony's initialiser under Wine [sure]. Camera calibration
(`FUN_180156500`) re-runs `FUN_1801556b0` but does not change those inputs,
so the result stays constant [likely]. Quaternion `(−0.5,−0.5,0.5,0.5)` is a
120° axis-permuting rotation; `(0.5,0.5,−0.5,−0.5)` is the same rotation.

### 5.5 HMD not-worn throttle [sure, difftested]

With proximity true, every pass is sent and becomes the reference (time
`+0x3c0` = `t_query`). With proximity false a pass counts as "fresh" if
connected/valid/result differ from the reference, or for some axis
`ref.pos − pos ≥ 0.005` m or `angvel ≥ 0.03` rad/s (signed tests: Sony
quirk). A non-fresh pass is still sent while `t_query − ref_time < 500 001`
µs; after that nothing is sent until motion.

### 5.6 Controller pose (`FUN_1801a7bf0`) [sure, difftested]

Tracker device 2 = left, 1 = right, queried at `now_us()`.
`poseTimeOffset = 0`. `qRotation = q_trk · q_tip`, `q_tip` = 39° about +x.
`vecPosition = p + R(qRotation)·o`, `o = base + grip·0.001`, base
`(±0.01432, −0.007713, −0.10399)` m (x sign per side), grip mm left `(9.3727, −12.2481, −6.0039)`, right
`(−20.0727, −12.2481, −6.0039)`. `vecDriverFromHeadTranslation = −grip·0.001`,
`qDriverFromHeadRotation` identity. Velocities passed through unchanged,
result 200, `poseIsValid` = tracker valid byte.

### 5.7 Hand skeleton (summary) [sure for bones]

Two 31-bone skeletons per hand each frame: with-controller (motion range 0)
and without-controller (range 1). Index curl = `0.7·trigger_prox +
0.3·trigger`; middle/ring/little from `g = 0.2·L1 + 0.8·grip_prox`
blended with index by 0.6/0.4/0.2; thumb shapes from touch bits and the
stick; 50 ms slerp blends on touch changes. Full constants in
`steamvr.md` "Hand skeleton".

## 6. Constants

### 6.1 Display

| Name | Value | Meaning | VA |
|------|-------|---------|----|
| Panel | 4000 × 2040 | both eyes side by side | `FUN_1801a2040` |
| Viewport | left (0,0,2000,2040), right (2000,0,2000,2040) | | `FUN_18019eca0` |
| Render target | 3400 × 3468 | `(W/2)·17/10`, `H·17/10` | `FUN_18019edb0` |
| Projection left | l −1.84177, r +0.94719, t −1.32898, b +1.32898 | tangents; right eye mirrored | `FUN_18019ed60` |
| Window | (0,0,4000,2040), not on desktop, real display | | `FUN_18019edf0` |
| Vsync→photons | 1.36 ms (90 Hz), 1.02 ms (120 Hz) | INI override in µs | `FUN_18019d1b0` |
| Photons→vblank | 10.3 ms (90), 7.73 ms (120) | INI override in µs | `FUN_18019d1b0` |
| 90/120 threshold | `|rate−90| < 0.1` | picks the timing pair | `FUN_18019d1b0` |
| Refresh tolerance | 0.1 Hz | rate snapping | `FUN_1801a4390` |
| EDID settle | 1000 ms | after Set `0x29` | `FUN_1801a4390` |
| 90/120 firmware gate | software version > `0x06000000` | else only 90 Hz | `FUN_18019d1b0` |
| IPD UI range | 0.059–0.072 m | | `FUN_18019d1b0` |
| IPD scale | `byte · 0.001f` | mm → m | `FUN_18019ee30` |
| HMD loop | 1 ms ready / 100 ms not | | `FUN_1801a19a0` |
| Controller loop | 1 ms any connected / 100 ms | | `FUN_1801aa660` |
| Not-worn hold | 500 001 µs | | `FUN_18019ee30` |
| Driver-from-head | q (−0.5,−0.5,0.5,0.5), t (−0.09255, 0, 0.02080) m | | `FUN_1801556b0` |
| Haptic min amplitude | 0.049 | below: ignored | `FUN_18011b4d0` |
| Haptic min duration | 0.08 s | | `FUN_18011b4d0` |
| Battery scale | 0.01 | level byte → 0..1 | `FUN_1801a8680` |
| Stick scale | 255 | `2·b/255 − 1` | `FUN_1801a8680` |
| Idle pulse | 60 s, checked each 1 s | `l1/touch` pulse | `FUN_1801a8680` |

### 6.2 Driver INI (`FUN_1801a47c0`) [sure]

Read with `GetPrivateProfile*` from `major_version.txt` / `version.txt` in
the driver folder. Behaviour-changing keys:

| Section / key | Default | Effect |
|---------------|---------|--------|
| `version/plugin version` | `0x04110006` | compared with Get `0xa1` (init gate) |
| `version/jenkins` | 0 | patches the low BCD digits of the plugin version |
| `debug/no seethrough` | 0 | disables see-through (USB manager flag, `FUN_180127f50`) |
| `debug/sleep action` | 2 | action when the headset sleeps [guess] |
| `debug/sleep action wait` | 0 | delay for it |
| `debug/edid product id` | 0 | overrides `EdidProductID` (default `0xc207`) |
| `debug/vsync to photons` | 0 | µs; used only if either timing key ≥ 1 |
| `debug/photons to vblank` | 0 | µs |
| `debug/force crash sec` | 0 | debug crash |

SteamVR settings read: `steamvr/preferredRefreshRate`, `steamvr/ipd`,
`direct_mode/enable`, `power/turnOffControllersTimeout`.

### 6.3 HMD properties (Activate order) [sure]

| Property | Value | Source |
|----------|-------|--------|
| 1005 ManufacturerName | `SIE` | const |
| 2011 EdidVendorID | `0xd94d` | const |
| 2015 EdidProductID | `0xc207` or INI | const/INI |
| 2110 / 2111 / 2112 (private) | 17 / 4 / 160 | const [guess: SteamVR-internal display hints] |
| 2003 UserIpdMeters | `steamvr/ipd`, then dial | setting, status byte 5 |
| 2080 DisplayAvailableFrameRates | `{90, 120}` if fw > 0x06000000 | `0x81` +0 |
| 2002 DisplayFrequency | 90 (old fw) or chosen rate | 3.2 |
| 2001 SecondsFromVsyncToPhotons | 6.1 | const/INI |
| 2042 SecondsFromPhotonsToVblank | 6.1 | const/INI |
| 2100 / 2101 IpdUIRange | 0.059 / 0.072 | const |
| 2075 HmdTrackingStyle | 3 | const |
| 2007 IsOnDesktop | false | const |
| 1001 ModelNumber | `PS VR2` | const |
| 1025 ContainsProximitySensor | true (+ `/proximity`) | const |
| 1037 InputProfilePath | `{playstation_vr2}/input/playstation_vr2_profile.json` | const |
| 5001–5007, 5009 icons | `headset_status_*` | const |
| 2048 DriverProvidedChaperonePath | `<UserConfigPath>\chaperone_info.vrchap` | |
| 5100 / 5101 hidden area | 5.3 | `0x8f` |
| 2056 DistortionMeshResolution | 80 | const |
| later: 1006, 1007, 1014–1016, 2057–2059, 2300/2301 | 3.5 | |

No serial number, `TrackingSystemName`, `DisplayMCType` or camera
properties are set by Sony.

### 6.4 Controller properties [sure]

Activate (`FUN_1801a5760`): 2075 HmdTrackingStyle 3, 2004 CurrentUniverseId
2, 3002/3003/3004 axis types 2/3/3, 3001 SupportedButtons
`0x0000000700000087`, 1001 `PlayStation VR2 Sense Left/Right`, 1003
`{playstation_vr2}playstation_vr2_sense_left/_right`, 3007 role 1/2, 1037
`{playstation_vr2}/input/playstation_vr2_sense_controller_profile.json`,
icons 5001–5009. Runtime: 1026, 1012, 1011, 1006, 1007, 1014–1016 (3.6).

Components created (order): `/output/haptic`; face buttons click+touch;
create/options click+touch; `l2/touch`, `l2/value`, `l2_sensor/value`,
`l1/touch`, `l1/value`, `l1_sensor/value`; stick click/touch/x/y;
`*_ps/click`, `*_ps/touch`; skeleton `/input/skeleton/<side>`
(`/skeleton/hand/<side>`, base `/pose/raw`, partial tracking).

## 7. Gaps in the current Linux driver

Compared with `kernel/`, `userspace/` and `steamvr/driver_psvr2/src`
(not edited):

| Item | Linux now | Sony |
|------|-----------|------|
| EDID type switch | none: no Get `0xa9` / Set `0x29` anywhere; relies on `DisplaySupportsMultipleFramerates` | switches EDID to match the SteamVR rate, waits 1 s |
| `0x8f` lens block | kernel reads 136 bytes as "Monado layout"; `psvr2_distort` uses it | 256 bytes, fields 2.2; read only on USB3 |
| Render target | `window_width/2 × window_height` (2000 × 2040) | 3400 × 3468 |
| Projection | `tan()` of configurable degrees (inner 0.94896, vert 1.32704) | fixed 0.94719 / 1.32898 / 1.84177 |
| Vsync→photons | 0.011 s; no PhotonsToVblank | 1.36/10.3 ms or 1.02/7.73 ms |
| Frame rates | `DisplaySupportsMultipleFramerates` | `DisplayAvailableFrameRates {90,120}` |
| EDID ids | `0x4DD9` / `0xA205` | `0xd94d` / `0xc207` (endianness of the vendor word differs; confirm which SteamVR-Linux matches on) |
| HMD driver-from-head | identity | q (−0.5,−0.5,0.5,0.5), t (−0.0926, 0, 0.0208) |
| HMD poseTimeOffset | `−clamp(age, 0, 0.05)` | `t_query − t_send` (≈ 0), tracker predicts to now |
| HMD loop | blocks on pose node, 10 ms back-off | 1 ms / 100 ms |
| Not-worn throttle | none | 5.5 |
| IPD | `mm / 1000.0f` | `(float)mm · 0.001f` (1 ulp) |
| IPD UI range, TrackingStyle, DistortionMeshResolution, 2110–2112 | not set | set |
| Hidden-area mesh | not set (no `HiddenArea` in the driver) | 5100/5101 from the lens geometry |
| Controller inputs | own mapping | 2.4 (L1 value is digital; stick 0 inside deadzone; touch on PS/create/options never sent) |
| Haptic thresholds | clamps amplitude only | ignore < 0.049, min 80 ms, stop events |
| Battery | `DeviceProvidesBatteryStatus` true always | false + 0 % while disconnected |
| Controller fw/hw strings | — | 1006 `%04x`, 1007 |
| Quit command | none | Set `0x27` 1000 on SteamVR quit |

## 8. Open questions

- `0xa9`/`0x29` on Linux: does the DP sink really re-enumerate (hotplug) and
  does amdgpu/i915 pick up the new EDID? Needs a capture: Get `0xa9`, Set
  `0x29` 0 → 1, read EDID from `/sys/class/drm/*/edid` before and after.
- `0x8f` bytes beyond 0x24: other users (config manager shares it with the
  PS VR2 app). Capture a block and compare against the 136-byte Monado layout
  the kernel uses today.
- Properties 2110–2112 (17, 4, 160): undocumented SteamVR ids; meaning
  unknown.
- `debug/sleep action` values.
- Idle `l1/touch` pulse purpose.
- Charging status codes 1 and 2 (charging vs full?) need a controller on a
  charger.
- Whether SteamVR on Linux matches `EdidVendorID` as `0xd94d` or `0x4dd9`.

## 9. Function map

| VA | Role | Covered |
|----|------|---------|
| `FUN_1801a2040` | ServerDriver::Init | yes |
| `FUN_1801a2a40` | ServerDriver::Cleanup (re-runs EDID choice) | brief |
| `FUN_1801a2cb0` | RunFrame | display parts |
| `FUN_1801a4390` | EDID / refresh choice | yes |
| `FUN_1801a47c0` | INI config + version log | yes |
| `FUN_18019d1b0` | HMD Activate / properties | yes |
| `FUN_18019ee30` | HMD update pass | yes |
| `FUN_1801a19a0` | HMD update thread loop | yes |
| `FUN_18019e860` | ComputeDistortion | yes |
| `FUN_1801a1a00` | Lens geometry from `0x8f` | yes |
| `FUN_1801a10c0` | Hidden-area mesh | summary |
| `FUN_18019eca0`…`FUN_18019ee20` | Display component getters | yes |
| `FUN_1801256c0` | Headset connect (reads `0x8f`) | `0x8f` part |
| `FUN_180124e30` | Copy `0x8f` block | yes |
| `FUN_18015e990` | Store `0x8f` in config manager | yes |
| `FUN_180155000`, `FUN_1801556b0` | Config defaults, rig transforms | driver-from-head |
| `FUN_180160230` | Tracker pose query | interface only |
| `FUN_1801a5760` | Sense Activate | yes |
| `FUN_1801a8680` | Sense frame: events, inputs, battery, fw props | yes |
| `FUN_18011b4d0` | Haptic request | yes |
| `FUN_1801a7bf0` | Sense pose | summary |
| `FUN_1801aa660` | Sense pose thread loop | yes |
| `FUN_1801a02b0` | Debug page (DP status, stream rates) | skipped |
| `FUN_18016b110` | QPC → µs clock | yes |
| `FUN_18016f350` / `f3e0` / `f470` | Rx / Ry / Rz from degrees | one line |

## Corrections to the main doc

- `0xa1` (main doc: "meaning unknown"): it is compared at server init with
  the plugin version (`version.txt` `plugin version`, default `0x04110006`);
  a larger value makes Sony refuse to load with the update dialog
  (`FUN_1801a2040`). So it is the minimum PC-app version the headset
  firmware requires [likely].
- `0x8f` (main doc: "hardware info/calibration [guess]"): it is the
  display/lens calibration block; offsets 0x00–0x23 used as in 2.2 [sure].
  The gate is the USB descriptor `bcdUSB ≥ 0x300` (USB3), not a report
  version.
