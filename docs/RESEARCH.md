# PS VR2 on Linux: consolidated research (canonical)

This is the **single source of truth** for what is known about the PS VR2
headset, the Sense controllers, and Sony's Windows SteamVR driver
(`driver_playstation_vr2_orig.dll`, PE timestamp 1741940190). It merges and
corrects:

- `docs/sony-driver-re.md` (the first pass);
- the per-topic notes in `docs/sony-re/`;
- `docs/protocol.md`.

**Where this document and an older one disagree, this one is right.** The
older documents stay for depth: tables, function maps, test notes. Every
section here names the detailed source.

## How to edit this document

- Confidence tags: **[sure]** = read in the code or confirmed by a
  differential test; **[likely]** = strong inference; **[guess]** =
  plausible only. An untagged statement in a table inherits the table's
  tag.
- When a new finding changes a fact:
  1. Change it here.
  2. Add a line to §12 (errata), naming the old wrong statement and where it
     appeared.
  3. Change the topic document only if it is still used as the detailed
     reference.
- When hardware testing confirms or refutes a fact, add "(HW yyyy-mm-dd)" to
  its tag.
- Raw values (constants, tables) may be recorded. Sony code may not, not
  even as pseudo-code copied line by line. Describe the behaviour instead.
- Addresses are function VAs in the DLL above. Image base 0x180000000.

## Contents

1. Architecture
2. Headset USB transport
3. Headset data streams
4. Display and the SteamVR HMD device
5. Head tracking
6. Sense controllers
7. LED sync
8. Tracking cameras: model and extrinsics
9. Controller pose solve and fusion
10. Room, play area, relocaliser
11. Open questions
12. Errata (superseded statements)
13. Where the Linux driver stands

---

## 1. Architecture [sure]

- The headset is one USB 3 device, `054c:0cde`, with many interfaces. Sony
  runs **one reader object per interface**. Each one opens its own handle,
  retries every 500 ms while the interface is missing, runs its own connect
  hook, and reads in its own loop (`FUN_180127b70`). One failing interface
  does not stop the others.
- The controllers are plain HID devices: USB or Bluetooth **to the PC**.
  Nothing about them passes through the headset.
- The headset runs SLAM and LED blob detection on its own. The PC gets a
  6DoF head pose (EP `0x83`), IMU samples (EP `0x88`), and LED blobs
  (EP `0x89`). Controller 6DoF is solved **on the PC** from those blobs
  plus the controller IMU.
- **USB 2 is refused.** Every gate checks the device descriptor's
  `bcdUSB ≥ 0x300`. On USB 2 the status loop does nothing and SteamVR init
  fails.

Detail: `sony-re/headset.md` §1–2.1.

## 2. Headset USB transport

### 2.1 Control transfers (`FUN_1801283f0`) [sure]

| | bmRequestType | bRequest | wValue | wIndex |
|---|---|---|---|---|
| Set | `0x42` | 9 | **0** (always) | 0 |
| Get | `0xc2` | 1 | report id | 0 |

The payload starts with an 8-byte header: `u16 report_id, u16 subcmd,
u16 len, u16 0`, followed by the data. `subcmd` is 1 for everything except
Set `0x05`, which uses 2.

### 2.2 Control reports (final list)

| ID | Dir | Len | Meaning | Tag |
|---|---|---|---|---|
| `0x81` | Get | 0x54 | Firmware info. +0x00 = software version, +0x34 = board-stage index (`HardwareRevision_String`), which must be non-zero (retry once after 1 s) | sure |
| `0xa1` | Get | 4 | **Minimum PC-driver version** the headset needs. Sony refuses to load when it exceeds its plugin version (`0x04110006`) | sure |
| `0x8f` | Get | 0x100 | Display/lens calibration block (§4.2). Read only on USB 3 | sure |
| `0xde` | Get | 0x18 | Device clock: µs timestamp at +4 (§3.1) | sure |
| `0x97` | Get | 1 | See-through state, polled once a second | sure |
| `0xa9` | Get | 1 | Current EDID type: 0 = 90/120 Hz, 1 = 90 Hz only | sure |
| `0xa5` | Get | 0x60 | DisplayPort receiver status, after Set `0x25` | sure |
| `0x8a` | Get | 8 | Read on IF9 connect; the value is never stored (Sony bug) | sure |
| `0xa4`, `0xa6` | Get | 4, 0x20 | Controller-tracking related | likely |
| `0x12` | Set | 1 | Panel brightness; init default `0x1f` | sure |
| `0x17` | Set | 2 | `{on, 3}`: see-through on/off | sure |
| `0x22` | Set | 1 | Init, one byte of device state | sure |
| `0x1f` | Set | 256 | Fixed table, sent on IF3 (SLAM) connect | sure (meaning: guess) |
| `0x0b` | Set | 8 | Camera mode `{u32 1, u32 mode}`. Always `0x10`; only VrTrace capture changes it | sure |
| `0x14` | Set | 16 | LED-detection window (§7.6) | sure |
| `0x09` | Set | 24 | Blob-detection thresholds (§7.6) | sure |
| `0x0a` | Set | 8 + 0xd8 | Relocaliser feedback (§10) | sure |
| `0x29` | Set | 1 | Set EDID type (0 / 1, as `0xa9`) | sure |
| `0x25` | Set | 0 | Latch before Get `0xa5` | sure |
| `0x27` | Set | 2 | u16 1000, sent on SteamVR quit | sure |
| `0x05` | Set | 0 | subcmd **2** only: after system resume (default sleep action), and in the direct-mode workaround. Headset restart | sure (meaning: likely) |
| `0x28` | Set | 0 | Only on resume with sleep action 3 | sure |

### 2.3 Init sequence on IF7 (status) connect [sure]

1. Wait 1 s.
2. Get `0x81`. Retry once after 1 s. Fail if +0x34 is 0.
3. Get `0xa1`.
4. Read the device descriptor. The remaining steps run only when
   `bcdUSB ≥ 0x300`.
5. Get `0x8f`.
6. Set `0x17 {0, 3}` (see-through off).
7. Set `0x22`.
8. Select **alternate setting 1 on IF7**.
9. Set `0x12` brightness.

Connect hooks on the other interfaces:

- IF3 sends Set `0x1f`.
- IF8 seeds Set `0x14` / `0x09`.
- IF9 does Get `0x8a`.
- IF10 seeds camera mode `0x0b` = `0x10`.

### 2.4 Bulk IN endpoints [sure]

| EP | Size | Content |
|---|---|---|
| `0x83` | SLAM record (§5.1) | IF3 |
| `0x85` | gaze | not read by Sony |
| `0x87` | 0x200100 | camera images |
| `0x88` | 0x4000 | status header (32 B) + IMU records |
| `0x89` | 0x9050 | `"LD"` LED detection (§7.7) |
| `0x8a` | 0xc8780 | `"RP"` relocaliser frames; **must be drained** |
| `0x8b` | 0x40000 | `"VD"` calibration container (§3.3), on IF10 (thread `CaesarUsbThreadGenData`) |
| `0x8d` | 0x100000 | firmware console log, ANSI text, on IF12 (thread `CaesarUsbThreadLog`) |

### 2.5 Host policies [sure]

- **See-through button:** a rising edge of status byte 2 while the headset
  is worn toggles Set `0x17`. Taking the headset off forces see-through
  off. Get `0x97` is polled once a second.
- **Brightness follow:** when status byte 10 differs from the configured
  brightness, re-send Set `0x12`.
- **Power:**
  - resume: Set `0x05` subcmd 2;
  - SteamVR quit: Set `0x27` (u16 1000).

Detail: `sony-re/headset.md` §2–§5.

## 3. Headset data streams

### 3.1 Device clock (Get `0xde`) [sure]

- Poll every 500 ms. The host stamps the time before and after each call.
- Keep a sample only when RTT < 1000 µs. Its offset is
  `host_before + RTT/2 − device_us`.
- Keep a 100-entry ring, and reset it when the offset jumps by more than
  2000 µs.
- Fit a phase and drift from the ring, with the drift clamped to
  ±50 000 ppb.
- The device clock is a **32-bit µs counter that wraps**. All headset
  timestamps (SLAM, IMU, LD) use it.

### 3.2 Status stream (EP `0x88`) [sure unless tagged]

Each transfer starts with a 32-byte status header, followed by IMU records.

Header bytes used:

| Byte | Meaning |
|---|---|
| 1 | proximity (worn) |
| 2 | function button |
| 5 | IPD dial, mm |
| 10 | panel brightness |

Also in the header: the DisplayPort receiver status (`dprx_status`).

- IMU rate: **2 kHz (500 µs)** [likely]. The rate is accepted as OK in
  (400, 600) µs.
- A 1250 µs group delay is subtracted.

Detail: `sony-re/headset.md` §2.7, `docs/protocol.md`.

### 3.3 Calibration container (EP `0x8b`) [sure]

- Header: `"VD"`, u16 version `0x0200`, u32 size at +4, u32 count at +8.
- Entries start at +0x20: `{u32 type, u32 size, u32 offset}`.

| Type | Size | Content |
|---|---|---|
| 10 | ≤ 0x800 | Tracking-camera calibration: **all 4 cameras and 3 extrinsics** (§8). The only one the IR tracker uses |
| 11 | ≤ 0x800 | Second camera calibration (VrTrace), unused by tracking |
| 12 | | IMU calibration |
| 13 | 0x34 | One extrinsic record: IMU ↔ camera [likely] |

Sony re-sends camera mode `0x10` once a second until types 10 and 12 have
arrived.

### 3.4 Camera calibration blob (types 10/11, `FUN_180161b70`) [sure]

Header:

| Off | Meaning |
|---|---|
| 0x00 | `"SIE"` |
| 0x03 | u8 version ≤ 1 |
| 0x04 | u16 size, which must equal the VD entry size |
| 0x07 | char[8] model. A `':'` followed later by `'d'` means double-precision records (0xe4 bytes); otherwise float records (0x94 bytes) |
| 0x0f | u8 N, the camera count |
| 0x10 | u8 M, the extrinsic count |
| 0x18 | N camera records, then M × 0x34-byte extrinsics |

Camera record (0xe4 layout):

| Off | Meaning |
|---|---|
| +0 | u8 id |
| +4 | u16 width |
| +6 | u16 height |
| +0x08 | 9 f32, K (3×3, row-major) |
| +0x2c | **20 f64**, D0..D19 (§8.1) |

Extrinsic record (0x34 bytes):

| Off | Meaning |
|---|---|
| +0 | u8 from |
| +1 | u8 to |
| +4 | 9 f32, R (row-major) |
| +0x28 | 3 f32, t (metres) |

### 3.5 Other streams

- **SLAM:** §5.1.
- **LD:** §7.7.
- **RP:** §10.
- **Firmware log:** strip the ANSI codes and log it.

## 4. Display and the SteamVR HMD device

Detail: `sony-re/display.md`, `sony-re/steamvr.md` (both difftested).

### 4.1 Panel and timing [sure]

- Panel 4000 × 2040; per-eye viewport 2000 × 2040.
- Recommended render target 3400 × 3468.
- Vsync-to-photons 1.36 / 1.02 ms; photons-to-vblank 10.3 / 7.73 ms (at
  90 / 120 Hz).
- Projection tangents 1.84177 / 0.94719 / 1.32898.
- DistortionMeshResolution 80.
- IPD UI range 0.059–0.072 m. IPD = status byte 5 mm × 0.001.

### 4.2 EDID / refresh (`FUN_1801a4390`) [sure]

- Read Get `0xa9`. If SteamVR wants 120 Hz and the EDID is 90-only, Set
  `0x29` = 0. If it wants 90 Hz and the EDID allows 120, Set `0x29` = 1.
- Then wait 1000 ms: the headset re-enumerates its DisplayPort sink.
- 120 Hz is offered only when the firmware is later than `0x06000000`.

### 4.3 Lens model [sure, difftested]

- The lens centres and panel rotations come from the `0x8f` block. Version
  byte < 4 means fixed defaults.
- Distortion uses three 1024-entry LUTs (R, G, B) at
  `0x180261180 / 0x180262180 / 0x180263180`.
- **Red and green get the −0.00023027 y offset; blue does not.**
- Hidden-area mesh: 32 rays starting at r = 0.9 in steps of 0.004, 36
  triangles, mirrored for the right eye.

### 4.4 HMD pose packaging [sure]

- Query the tracker at "now". `poseTimeOffset = t_query − t_send`.
- Poll every 1 ms while ready, every 100 ms otherwise.
- **Driver-from-head transform** (IMU rig → head):
  - `qDriverFromHeadRotation (w, x, y, z) = (−0.5, −0.5, 0.5, 0.5)`;
  - `vecDriverFromHeadTranslation = (−0.09255, 8e-9, 0.02080)` m, in
    IMU-rig axes.
- `qWorldFromDriver` = identity.
- Not-worn throttle: 5 mm, 0.03 rad/s, 500 001 µs.
- HmdTrackingStyle = 3 (inside-out cameras).

## 5. Head tracking

Detail: `sony-re/tracking.md` §2–§5.2, `notdone.md` §4.

### 5.1 SLAM record (EP `0x83`) [sure]

| Off | Meaning |
|---|---|
| +0x0c | state, **3 = tracking** |
| +0x0d | substate |
| +0x0e | flag |
| +0x10 | position (m) |
| +0x1c | quaternion stored **x, y, z, w** |
| +0x2c | reference pose (wTi) |
| +0x48 | velocity (world) |
| +0x54 | gyro bias |
| +0x60 | accel bias |
| +0x144 | anchor, applied only for record version < `0x110` |
| +0x184 | error variance; sqrt feeds a quality EMA |
| +0x1e0 | SLAM reset counter |
| +0x1e4 | map id (upper 16 bits latch) |

- Valid 6DoF = `state == 3 || flag == 1 || substate == 5`.
- The pose is **world ← IMU rig**. The world is **Y up**, with gravity
  (0, −9.8, 0). No axis permutation is applied before SteamVR.
- The first pose's position is subtracted once (optional).

### 5.2 Mapping to the Linux driver [sure, numeric check]

The Linux remap (quaternion `(q0, −q2, −q1, q3)`, then Rz(+90°), position
`(r2, r1, −r0)`) equals Sony's `q_dfh` folded into the rotation, plus a
world yaw of −90° about Y. It is correct because its "w, x, y, z" naming
and its permutation cancel.

**What it lacks** is Sony's lever arm. In Linux head axes the head origin
is at `(0, +0.0208, +0.09255)` m from the IMU: 2.1 cm up and 9.3 cm back.
Report it as `vecDriverFromHeadTranslation`.

### 5.3 Host INS (Sony) [sure unless tagged]

Sony keeps two strapdown integrators over the 2 kHz IMU:

- a VIO filter, reset to every valid SLAM sample;
- an output filter, pulled toward the VIO filter by an adaptive blender.

Constants:

| Constant | Value |
|---|---|
| SLAM gap counted as a drop | 24 ms |
| No valid SLAM for this long → 3DoF | 500 ms |
| IMU stall → reset | 100 ms |
| Shock | 35.5 m/s² per axis; fast blend for 3 s |
| Quality EMA | 0.04; bad 5.0; exit 2.5; enter 2.0; n > 30 |
| Gyro-bias sanity | norm ≤ 1.0 |
| Accel-bias sanity | norm ≤ 1.7 |

A minimal port: take the latest SLAM pose, velocity and biases; integrate
newer IMU samples; predict to display time.

## 6. Sense controllers

Detail: `sony-re/sense.md`, `sony-re/libpad.md`.

### 6.1 Identification and features [sure]

- VID `054c`; PID `0e45` (left), `0e46` (right). `0ce7` is also a VR
  controller; its hand comes from feature `0x20`.
- Transport by input length: 64 bytes = USB, 78 bytes = Bluetooth.
- **Feature `0x20`** (64 bytes):
  - +0x18 hardware info; its low 16 bits are the **hardware rev**, which
    selects the LED model (§9.1);
  - +0x1c **main firmware version** (u32);
  - +0x2c firmware revision;
  - +0x30 bit 0, hand.
- **Feature `0x05`**: IMU calibration in two parts, DualSense-style math.
- IMU ticks run at 3 MHz.
- Sony never sends feature `0x08` (polling rate): the setter has no
  callers.

### 6.2 Output report [sure]

- Bluetooth: `31 a2` + body + zero padding + CRC32 (seed `0xa2`) over 74
  bytes. **No tag byte, no PCM block.**
- Body (OutputSettings, 38 bytes):

  | Off | Meaning |
  |---|---|
  | 0 | flag1 |
  | 1 | flag2 |
  | 2 | amplitude |
  | 4 | trigger mode |
  | 5..14 | trigger parameters |
  | 0x0f | host µs timestamp |
  | 0x13 | phase |
  | 0x14 | seq |
  | 0x15 | period id |
  | 0x16 | base_time × 3 |
  | 0x1a | cycle × 3 |
  | 0x1e..0x21 | LED[0..3] |
  | 0x22 | reduction |

- Stamp the host µs right before the write. The controller echoes it,
  which drives the clock fit (§6.4).
- Vibration: amplitude = ceil(a × 20) with flag1 `0x40 | 0x03`. Events
  below 0.049 are ignored; the minimum pulse is 80 ms; re-send every
  100 ms. The pad vibration mode is always 2.
- Reduction byte (flag1 `0x10`): high nibble trigger, low nibble haptics,
  12.5 % steps.
- Adaptive triggers: libpad can encode them, but Sony's SteamVR driver
  never calls them (optional on Linux).

### 6.3 Battery [sure]

Level 0..9 → `10n + 5` %, 10 → 100 %, 11..15 → 0 %.

### 6.4 Host ↔ controller clock fit (libpad `[HID_SYNC]`) [sure]

- Every input report feeds the fit with (host receive time, echoed host
  stamp, controller ticks).
- The output: `host_to_ctrl(t) = (drift·ref + (t − fit_off)·1e9) / (drift + 1e9)`.
- It is needed only for LED sync.
- Implemented bit-exact in `userspace/lib/sie/libpad/psvr2_hidsync.c`.

## 7. LED sync

Detail: `sony-re/ledsync.md` (read it together with the corrections in
§12), `notdone.md` §3.

### 7.1 Purpose

The PC makes each controller pulse its LEDs while the headset's camera
shutters are open. It does this by putting a phase, an LED set, a time
offset and a frame period into every controller output report.

### 7.2 Phases [sure]

| # | Name | period_id (preset 1 / 2) | Report LEDs |
|---|---|---|---|
| 0 | INIT | – | – |
| 1 | PRESCAN | 40 / 40 | ff |
| 2 | BROAD | 50 / 42 | sync LEDs |
| 3 | BG | 30 / 30 | sync LEDs |
| 4 | STABLE | 12 / 12 | ff |
| 5 | LED ALL OFF | – | ff |
| 6 | LED ALL ON | – | ff (not used) |

### 7.3 Commands to libpad [sure]

| Cmd | Len | Bytes | Effect |
|---|---|---|---|
| 1 | 7 / 11 / 2 | `01 phase period led0..3 [u32 value]` | Set phase. PRESCAN: base_time from the newest LD frame time; with len 11 the value sets the frame cycle. BROAD/BG/STABLE: value = offset, clamped ±15·step. Phases 5/6: len 2 |
| 2 | 5 | `02 led0..3` | LEDs only |
| 3 | 5 | `03 f32 slope` | `cycle := clamp((1 + slope)·cycle)` |
| 4 | 5 | `04 s32 offset` | base_time = clamp(offset) |
| 5 | 9 | `05 f32 slope s32 offset` | 3 + 4 |
| 7 | | `07 sub arg` | sub 1 = LEDs off; sub 2 = full reset; sub 3 = stats |

- Every accepted command increments seq and zeroes base_time first.
- PRESCAN base time: `t_ctrl(host(LD frame time)) − period·step/2 − width/2`.
  It is refreshed every ≥ 1 s, and seq is bumped each time.
- The tracker sends len 7 in PRESCAN while the retry count is < 2,
  otherwise len 11 with 16 683 350.

### 7.4 Sync mode (libpad, `FUN_1801cbeb0`, `FUN_1801c1820`) [sure]

- The block runs only while the mode is 0.
- If feature `0x20` +0x1c (firmware) < `0x1006d`, or the probe has been
  tried more than 3 times: **mode 2**.
- Otherwise: Set `0x80` `80 13 22 00…` (64 bytes), then Get `0x81`. Reply
  byte 0 = `0x81` → mode 2, else 0. Count the try.
- Mode 2 = width 500, step 50. Mode 1 (1000/105) is **never set** in this
  build. The reset values, used while the mode is 0, are 1000/105/1575.

**Steady state is always mode 2.**

### 7.5 Presets [sure]

The table is at `0x18026035c`. Preset 1 and preset 2 values:

| | Preset 1 | Preset 2 |
|---|---|---|
| step (µs) | 105 | 50 |
| PRESCAN window (µs) | 4200 | 2000 |
| BROAD window (µs) | 5250 | 2100 |
| BG window (µs) | 3150 | 1500 |
| STABLE window (µs) | 1260 | 600 |
| `0x14` A | 128 | 85 |
| `0x14` B | 1000 | 500 |

- period_id = window / step.
- The first connected controller fixes the global preset. A controller on
  another preset stays in INIT ("SHOULD BE UPDATE CONTROLLER FW").
- The preset itself is pad +0x1298, copied from byte +0x6f of libpad's
  controller state sample. Its HID origin is open (§11).

### 7.6 Headset detection config [sure]

**Set `0x14`** (16 bytes):

| Off | Meaning |
|---|---|
| +0 | u32 A |
| +4 | u16 B |
| +6 | u16 0 |
| +8 | u32 `0x24800001` |
| +0xc | u32 0 at startup, `0x100` once a preset is applied |

- Startup values: `{0x55, 500, 0x24800001, 0}`.
- Re-send the preset values when the LD echo (+0x20 / +0x24, both floats)
  differs, the B echo is non-zero, and the LD version is > 0x100. Send once
  per preset.

**Set `0x09`** (24 bytes):

| Off | Normal | Many Blob |
|---|---|---|
| 0x00 | `fe 00 00 00` | same |
| 0x04 | `fe 00 00 00` | same |
| 0x08 | `78` | `b4` |
| 0x09 | `30` | `20` |
| 0x0a..0x0d | `40 20 80 3c` | same |
| 0x0e | `a0` | `b4` |
| 0x0f | `04` | same |
| 0x10..0x13 | `0a 20 02 02` | same |
| 0x14..0x17 | 0 | 0 |

- The LD echo locations: 0x18..0x1f and 0x28..0x2d.
- Re-push when the echo differs, but never on the frame right after a
  push (LD counter = last + 1).

**Blob mode:**

- Normal → Many Blob when camera 0 or 1 reports ≥ 200 blobs or sets its
  flag bits.
- Back to Normal after 480 quiet frames: both counts ≤ 49 and no flags.
- Extreme-Env modes 2–4 can be enabled only by an external process
  writing Sony's shared-memory settings. They are not used in practice.

### 7.7 LD packet (EP `0x89`, 0x9050 bytes) [sure]

| Off | Meaning |
|---|---|
| 0x00 | `"LD"` |
| 0x02 | u16 version (≥ 0x82: the +0x14 counter is valid) |
| 0x08 | u32 device µs of the frame. With no clock map, Sony uses host now − 6000 µs |
| 0x0c | four 2-bit camera flags (bits 0–1, 8–9, 16–17, 24–25) |
| 0x14 | u32 frame counter |
| 0x18..0x1f, 0x28..0x2d | `0x09` echo |
| 0x20, 0x24 | f32 echoes of the `0x14` A and B |
| 0x35 | u8, non-zero = no detection this frame |
| 0x40 + n·0x2404 | camera n: u32 count (≤ 256), then 36-byte blob records |

- The blob record decode (`FUN_180184900`) is in `sony-driver-re.md`.
  Implemented in `sie/irtracker/psvr2_ld.c`, difftested.
- Sony waits up to 100 ms per tracker cycle for a new packet.

### 7.8 Per-camera history and states (`FUN_180194f70`) [sure]

- Window L: PRESCAN 1, **BROAD 11**, **BG 20**, STABLE 1.
- Per frame k the tracker keeps w[k], the sync-LED blob weight, and
  cnt[k], the controller's blob count in that camera.

```
for k from newest back: if w[k] ≈ 0 → history short (status 4) and stop
max/min over frames with cnt > 0
max ≈ 0 or min = FLT_MAX → status 9 ("NO MAX MIN")
max − min < 0.1         → status 10 ("NO DIF MAX MIN")
thr = (max + min)/2;  state[k] = (cnt[k] > 0 && w[k] > thr) ? 2 (lit) : 0
```

### 7.9 BROAD [sure]

- Blink codes, 11 frames: frames 0–4 off, frames 5–7 on, frames 8–10 the
  3-bit **Gray code** of col/2, frame 8 = MSB. Only even columns 0..12 are
  valid.
- A code matches when every frame agrees: lit ↔ 1, dark ↔ 0.
- Across cameras, take the slot closest to the previous one (within 32).
- Accept when the last **3 decodes that reached the test** agree. This
  counts decodes, not frames.
- On accept:
  - `slot − 4 > 4` is a GRID SKIP; 6 of them reset BROAD.
  - Otherwise, if slot ≠ 4, send offset `(slot − 4)·5·step/2` µs; the
    accumulated slot sum goes into the BROAD drift fit.
- Leave BROAD:
  - slot 4 held ≥ 3 s (13 s on retries) → BG;
  - no code for 10 s → PRESCAN, retry + 1.

### 7.10 BG [sure]

- Entry value:
  - from BROAD: `21·step − BG/2` (630 / 300 µs);
  - from STABLE: `STABLE/2 − BG/2`.
- **Marker check** over the 20-frame window: frames 0, 1, 2, 3 must be
  lit, dark, dark, lit, and frames 17, 18, 19 must be lit, dark, lit.
  Frames 4..16 are not checked; they carry the sweep. A mismatch is camera
  status 5.
- **Estimator:**
  - `floor = max/2`;
  - for k = 4..16, `d = max(0, floor − w[k])`;
  - `x = Σk·d / Σd`.

  So x is the centre of the **dark** gap, in steps. Target x = 10.
- **Selection:** among the cameras and the combined slot 4, take the x
  closest to the last accepted x.
- **Skip:** if the fit has ≥ 2 runs and is fitted, and |x − 10| > 1, drop
  the sample. Allowed up to 3 times in a row.
- **Accept:**
  - push `x + acc` into the BG/STABLE fit;
  - if x is outside [9, 11]: `acc −= (10 − x)` and send cmd 4 with
    `(10 − x)·step` µs;
  - if `|(10 − x)·step| ≥ 40`, reset the fit first.
- **Lock (→ STABLE):** the drift fit has run twice, |slope| < 2e-6, and BG
  has been good for > 15 s (3 s when re-entering within 300 s of STABLE).
- **Bad** for ≥ 13 s → PRESCAN.

### 7.11 STABLE [sure]

- Value = `BG/2 − STABLE/2 + step·(10 − x)`. LEDs ff (the controller keeps
  its last pattern).
- Every 100 s while visible, drop back to BG.
- Invisible for ≥ 15 s → PRESCAN.

### 7.12 Drift fit [sure]

- Runs of equal values are merged (within DBL_EPSILON).
- With ≥ 4 runs, fit a least-squares slope over all runs except the first.
  The fit latches until it is reset.
- t is in µs since the first sample [likely host clock].
- The slope is sent **raw** (steps/µs or slots/µs, unscaled) in cmd 3, or
  cmd 5 with an offset. A positive slope lengthens the cycle.
- Sent whenever |slope| > DBL_EPSILON. A send resets the retry counter.

### 7.13 Forced transitions [sure]

| Condition | Result |
|---|---|
| Controller not connected, preset mismatch, > 5 s since its last pass, or connection id changed | INIT |
| Tracker disabled | LED ALL OFF |
| See-through on | LED ALL OFF for both; INIT when it ends |
| Extreme-Env mode 3 | PRESCAN |
| Phase 0, 5 or 6 at the end of the checks | PRESCAN |

### 7.14 Sync LED choice (`FUN_1801999f0`) [sure]

- Candidates: eligible LEDs with an associated blob and `0 < metric·0.5 ≤ 45`,
  where the metric is the LED-normal ↔ view-ray angle in degrees.
- Keep N, the sync-LED count (default 1), with the smallest metric. Take
  the camera with the most kept LEDs.
- A success less than 1 s ago keeps the current LED.
- LED bytes are **physical** indices. For logical LED k: k ≥ 1 → k;
  0 → the LED count (20 for rev 1, 17 otherwise). Every LED is eligible.

## 8. Tracking cameras: model and extrinsics

Detail: `notdone.md` §1.

### 8.1 Calibration to model [sure for the data flow; likely for the OpenCV names]

- D0..D13 are OpenCV's `(k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4, τx, τy)`.
- D14..D16 extend the numerator with r⁸, r¹⁰, r¹²; D17..D19 extend the
  denominator.
- s1..s4 are stored but **not used**.
- K: fx = K[0], fy = K[4], cx = K[2], cy = K[5]. Skew is dropped.
- Sony rescales to a **508 × 508** model image with `s = 508/width` (and
  the same for y):
  - `fx′ = s·fx`;
  - `cx′ = (cx + 0.5)·s − 0.5` (pixel-centre convention).
- Live path: VD type 10 → `FUN_180182e80` (26-float record) →
  `FUN_180045420` + `FUN_18009d4d0` → a 0x1c0-byte model per camera.

### 8.2 Projection (`FUN_180045650`) [sure]

Equivalent OpenCV form: rotate the camera-frame point 180° about the
optical axis, `(X, Y, Z)_cv = (−X, −Y, Z)`. Then:

```
x = X/Z, y = Y/Z, r2 = x² + y²     (reject r2 > 200 or Z ≤ 1e-4)
rad = (1 + k1 r2 + k2 r2² + k3 r2³ + D14 r2⁴ + D15 r2⁵ + D16 r2⁶)
    / (1 + k4 r2 + k5 r2² + k6 r2³ + D17 r2⁴ + D18 r2⁵ + D19 r2⁶)
xd = x·rad + 2 p1 x y + p2 (r2 + 2x²)
yd = y·rad + p1 (r2 + 2y²) + 2 p2 x y
(xd, yd) ← tilt(τx, τy)            (OpenCV computeTiltProjectionMatrix)
u = cx′ + fx′·xd,  v = cy′ + fy′·yd
```

Sony's own form stores p1, p2, τx and τy negated and computes
`u = cx′ − fx′·xd` on the unrotated point. The two forms are identical.
The projection Jacobian is `FUN_180103580` (it uses the derivative lanes).

### 8.3 Extrinsics [sure unless tagged]

- Calibration R, t (metres) become rig 4×4 matrices in **mm**:
  `R′ = G R G`, `t′ = 1000·G t`, with `G = diag(−1, −1, 1)`. This is the
  same 180° rotation as §8.2.
- Each camera is placed in the rig frame by composing its extrinsic with
  camera 0's built-in rig pose.
- An extrinsic recorded as from 2 → to 0 is inverted on load. So the
  expected direction is "0 → k". §8.4 gives the measured mapping.
- Built-in rig translations, variant 0 (mm):

  | Cam | Translation |
  |---|---|
  | 0 | (+40.0, −41.68, 93.58) |
  | 1 | (−40.0, −41.68, 93.58) |
  | 2 | (+61.5, 22.97, 97.03) |
  | 3 | (−61.5, 22.97, 97.03) |

  0/1 are the inner pair 80 mm apart; 2/3 are the outer pair.

### 8.4 Measured on hardware (one headset, 2026-11-18) [sure]

- Calibration model `pv:tpxd` v1, 4 cameras at **1016 × 1016**, so
  s = 508/1016 = 0.5 exactly. fx ≈ 380–383 px → 190–191 px in the model.
- D8..D11 (s1..s4) = 0. τx, τy within ±0.003 rad.
- The denominator terms k4, k5, k6 and D17..D19 are identical on all four
  cameras (a shared lens design); the rest differ per camera.
- Three extrinsics: 0→1, 2→3, 2→0 (OpenCV frame, metres).
- **Direction:** extrinsic `a → b` maps points as **X_b = R·X_a + t**.
  Only this reading gives a mirror-symmetric rig (residual 0.2 mm); the
  inverse reading is off by ~90 mm.
- The rig from them, in camera 0's OpenCV frame (mm):

  | Cam | Centre | Optical axis |
  |---|---|---|
  | 0 | (0, 0, 0) | (0, 0, 1) |
  | 1 | (76.1, 1.1, −20.2) | (0.50, 0.01, 0.87) |
  | 2 | (−35.3, −58.5, −30.2) | (−0.31, −0.61, 0.73) |
  | 3 | (93.4, −56.5, −64.6) | (0.65, −0.59, 0.47) |

  The 0–1 baseline is 78.8 mm (built-in table: 80); 2–3 is 133.2 mm
  (table: 123). The built-in table is likely a different variant.
- **Physical position (cover test, wearer's view):** cam 0 bottom-left,
  cam 1 bottom-right, cam 2 top-left, cam 3 top-right. 2/3 sit higher and
  look up and outward; 0/1 look forward, toed out 15° each.
- **Blob coordinates** are already in the 508 × 508 model image
  (centroids span 0..507).
- A lit room gives **50–80 blobs per camera per frame** in Normal blob
  mode at the startup `0x14` window {85, 500}, many of them large
  (up to ~60 × 60 px boxes).

## 9. Controller pose solve and fusion

Detail: `notdone.md` §2, `tracking.md` §5.3–5.6, `ctrlboot.md`,
`ctrltracker.md`.

### 9.1 LED models [sure]

| Hardware rev (feature `0x20` +0x18) | Model address |
|---|---|
| 1 | `0x180257280` |
| 2 | `0x180258680` |
| 3 | `0x180259a80` |
| 4 | `0x18025ae80` |
| 5 | `0x18025c280` |
| 6 | `0x18025d680` |
| other | `0x18025ea80` |

- The 0x54-byte boot config sits 0x60 before each model.
- Model header:
  - word 0 = LED count (20 for rev 1, 17 otherwise);
  - word 4 = LED cone, **170°** for rev 1–3 and **180°** for rev 4–7.
- Model data: positions in **mm in the controller frame**, plus normals.
  `scripts/sony-re/extract_led_models.py` reads them.
- `ImuPoseIndex = 2·rev + 1 + hand` (3..14; default 15/16) selects a
  constant IMU → controller 4×4.

### 9.2 Boot config (0x54 block, names from `ConfigFile0.trk` keys) [sure]

The values are the same for all models:

| Key | Value |
|---|---|
| NUMFRAMESUNTILBOOT | 2 |
| INLIERTHRESHOLD | 2 px |
| POTHOLETHRESHOLD | 1 (compared with px²) |
| SCALEDPOTHOLE | 1 |
| SCALEDINLIER | 0 |
| SCALEDSEARCHRADIUS | 1 |
| SEARCHRADIUS | 3 px |
| MININLIERSFORBOOT | 7 |
| …MULTICAMBOOT | 5 |
| …STRICTGRAVITYBOOT | 5 |
| …MULTICAMSTRICTGRAVITYBOOT | 4 |
| …ORIENTATIONBOOT | 4 |
| RANSACMAXSAMPLES | 200 |
| …BOOTINTRACK-MAXSAMPLES | 50 |
| RANSACRINGBOOTRATIO | 0.5 |
| USEGRAVITYBOOT | 1 |
| MINSIZEDOT09 | 240 |
| MINSIZEDOT07 | 220 |

### 9.3 Pipeline [sure unless tagged]

1. Per camera:
   - map the blobs through the camera model;
   - drop the blobs outside the active window.

   Every controller then sees **all** blobs of the camera. Nothing
   partitions blobs between controllers; LED-sync timing and the
   pose-driven association keep the controllers apart.
2. **Association:** projection-driven.
   1. Visibility: an LED is visible when it projects inside the image
      **and** its view angle (normal ↔ LED-to-camera direction, degrees) is
      below the cone. The limit is 170°/180°: 174° in the multi-camera
      accept mode, ×0.8 in mode 3.
   2. When the depth is ≥ 120 mm, cull an LED that lies within 3 px (box)
      of another and more than 10 mm deeper.
   3. Sort the remaining LEDs by view angle, face-on first.
3. **Gated match:**
   - for each LED in order, take up to 7 nearest blobs within R;
   - one blob may serve only one LED;
   - drop outliers whose distance is > f × the best's and > 3 px:
     f = 1.5, or min(25, 900/z_mm) under 150 mm.
4. **Boot** (single camera):
   - RANSAC over 3 LEDs × 3 blobs (P3P, up to 4 poses): first up to 100
     samples from 3-LED windows of the model's LED rings against blob
     chains, then an LED and two of its 4 nearest against a blob and two
     of its 4 nearest; MSAC score in that camera, 2 px (6 px close);
     every hypothesis passes the pose check (`notdone.md` §2.19);
   - fixed count: 200 cold or 50 while tracking (the adaptive
     `n = log(1−p)/log(1−w⁶)` bound is off for this object);
   - the gravity prior comes from the IMU quaternion when USEGRAVITYBOOT is
     set.
5. **Boot checks:**
   - pothole `T = min(1/d_m, 10)·1`;
   - refine with gate 1.5·T;
   - accept with ≥ 7 inliers; or ≥ 5 if the gravity check passes (3°,
     or 6° under 150 mm); or ≥ 4 if the orientation check passes (15°,
     or 30° under 150 mm; prior < 10 s old; distance < 500 mm);
   - drop candidates whose score is > 0.34.
6. **Multi-camera boot** (when single-camera boot fails):
   - the rotation comes from the IMU prior;
   - solve the translation from 2 blob↔LED rays over the fixed LED-pair
     table;
   - score across all cameras;
   - accept when m ≥ 4 and m = v, or after a multi-camera LM refine when
     m > 5 and e < 0.1.
7. **Refine:**
   - weighted Levenberg–Marquardt over 6 DoF, residual = reprojection
     error in pixels;
   - **Huber, c = 1.0** on scale `1.483·median|r|` (none with exactly 3
     LEDs);
   - IRLS, μ0 = 1e4, ν = 2, Nielsen update, 250 iterations at most;
   - rotation updated through the exp map.
8. **Score:** MSAC, `Σ (d² − T)` over d² ≤ T, divided by T (−n is
   perfect).
9. **Close-range states** (mm):

   | From | Condition | To |
   |---|---|---|
   | 0 | close, d < 80 | 2 |
   | 0 | close, 80 ≤ d < 130 | 1 |
   | 1 | d > 170 | 0 |
   | 2 | d > 120 | 1 |
   | 0..3 | not valid | 4 (lost) |

   From state 4, a re-boot gives 2 when d < 100, 1 when d < 150, else 0.

### 9.4 UKF [sure unless tagged]

- Error state of 15: position, velocity, attitude, gyro bias, accel bias.
- P0: 0.0025, 0.04, (1°, 5°, 1°)². Yaw is the loose axis.
- Delayed optical measurements: apply at their own timestamp and replay
  the IMU (up to 31 states).
- Stationary damping: velocity × 0.99 when |a − g| < 1.96 m/s².
- Divergence guard: reset the velocity and biases. A full reset follows a
  BadState.
- The process noise depends on angular rate (`ctrltracker.md`). Several
  limits come from config and were not read (§11).

## 10. Room, play area, relocaliser

Detail: `sony-re/room.md`, `sony-re/reloc.md`.

- **Play area** [sure]:
  - set property 2048 = `<UserConfigPath>/…vrchap` and 2004 = its
    `universeID`;
  - file format: chaperone_info JSON, version 5;
  - default area: a 24-point circle of radius 1.5 m around the headset.
- **RP frames** (EP `0x8a`) [sure]:
  - `"RP"`, size `0xc8780`, 8 u32 command slots at +0x08;
  - 162 features per camera pair: u16 keypoints and 128-byte
    descriptors;
  - a 640 × 640 image, a pose prior, and up to 2 anchors.
- **Set `0x0a`** [sure]:
  - header `0a 00 <u16 subcmd> <u16 0xd8> 00 00`, then 0xd8 bytes;
  - subcmd 4 = start (u32 1); 3 = tuning, once; 1 = localiser result,
    per RP frame;
  - sends are paced at ≥ 20 ms, with 5 retries.
- Without a relocaliser, only **persistence** is lost: the map and the
  play area across sessions. In-session tracking needs nothing from the
  PC. Sony saves `sceMapDb.bin` and `sceBoundaryMeta.bin` on quit, every
  60 s while worn, and after edits.

## 11. Open questions

| # | Question | Next step |
|---|---|---|
| 1–3 | Camera size, extrinsic direction, camera index: answered, §8.4 | – |
| 4 | HID origin of the controller "preset" (libpad state +0x6f) | Trace libpad input parse |
| 5 | Time unit of the drift-fit t; does BROAD shift slot −1 into its 3-history? | `FUN_180196ff0` caller, `FUN_180198240` |
| 6 | Candidate score inside the gated match: answered (`sony-re/notdone.md` §2.11). Open: LM stop tolerances; the 0.25 option | `FUN_1800f7970` |
| 7 | Unit of the multi-camera boot error e | `FUN_1800e49b0` |
| 8 | Cross-controller cleanup | `FUN_1800a5e30` → `FUN_1800ad2b0` |
| 9 | UKF config limits (`+0x798..+0x7a0`, `+0x784`, `+0x864`) and the exact process noise | `ctrltracker.md`, config reader |
| 10 | `FUN_1800e1c90` "20° cone" (`cos 0.349`) vs the 170°/180° view-angle limit: two different tests? | Read both call sites |
| 11 | Meaning of the Set `0x1f` table and the `0x14` A value | – |
| 12 | Hand-1 descriptor byte `0x58`: answered, it is the char `'X'`; the left model is mirrored in x (`sony-re/opticalpose.md` §2b.2) | – |
| 13 | Per-frame tracking acceptance: answered. ≥ 3 matches, then gravity 20°, orientation 30°, depth 5–1500 mm (`sony-re/notdone.md` §2.16). Our stricter 3/4-facing / 1.5 px rule stays on top (`own_accept`) | – |
| 14 | Blob → sync-LED assignment: answered. There is none without a pose: `FUN_1801999f0` fills node `+0x24` from the pose tracker's LED → blob map, else 0 (`sony-re/notdone.md` §3.1). BlinkFinder stays as our fallback for a hand without a pose | – |

## 12. Errata (superseded statements)

| Where | Old statement | Correct |
|---|---|---|
| sony-driver-re.md | `0xa1` meaning unknown | Minimum PC-driver version (§2.2) |
| sony-driver-re.md | `0x8f` gate "version ≥ 0x300 at +2" | USB descriptor `bcdUSB ≥ 0x300` |
| sony-driver-re.md | Alt setting on a "control interface" | Alt setting 1 on **IF7** |
| sony-driver-re.md | Thread names of `0x8b` / `0x8d` | Swapped (§2.4) |
| sony-driver-re.md, protocol.md | Set wValue = report id | Set wValue = 0 |
| sony-driver-re.md | SLAM +0x0c = 3 means lost | 3 = tracking |
| sony-driver-re.md | Camera record holds 19 doubles | 20 doubles (D0..D19) |
| sony-driver-re.md | LED-sync cmd table (cmd 5 "sync loss", cmd 7 "modes") | §7.3 |
| sony-driver-re.md | `0x14` defaults are the active values | Startup only; the presets apply later (§7.6) |
| sony-driver-re.md | LD +0x20/+0x24 "exposure/sync phase" | Float echoes of `0x14` A/B |
| sony-driver-re.md | Output +0x14 written "when a sync request is pending" | When phase ∉ 1..4 or the clock fit has a reference |
| ledsync.md §2.2 | Mode 1 for firmware < 0x1006d or a failed probe | **Mode 2**; mode 1 is never set |
| ledsync.md §5.3 | BG estimator uses samples above the floor; skip is relative to the last x | Below max/2; skip is relative to 10 |
| ledsync.md §5.5 | Units [guess] | Slope raw, steps or slots per µs |
| ledsync.md §2.9 | BG pattern = 32 frame bytes | 4 s32 pairs: frames 0–3 and 17–19 |
| tracking.md §5.3 | Extrinsics "z axis flipped" | 180° about z: t′ = 1000·(−tx, −ty, tz) |
| tracking.md §5.3 | P1PFromFailedUndistort = 1-point fix after an undistort failure | It is the **lost** state; no undistort iteration |
| tracking.md §8.1 | SLAM quaternion order open | x, y, z, w; the Linux remap is right |
| tracking.md / earlier guesses | 0x54 tuning block names | `ConfigFile0.trk` keys (§9.2) |
| tracking.md §6 | LED cone one constant | Per model, 170° or 180° |
| tracking.md, ctrlboot.md | RANSAC 1000 iterations | 200 cold, 50 while tracking |
| tracking.md, RESEARCH.md §9 | Boot RANSAC over 6-point minimal sets, adaptive count | 3 LEDs × 3 blobs (P3P), ring stage first; adaptive flag 0 (`notdone.md` §2.19) |
| earlier guesses | Gravity/orientation boot limits from config | Fixed 3°/15° (6°/30° close) |
| DEV-NOTES.md | Camera-model builder `FUN_1800c8950/8a40`, 0x180-byte struct | `FUN_180045420` + `FUN_18009d4d0`; 0x1c0 bytes; those two are RANSAC settings |
| libpad.md | Pad +6 "connection kind" | Vibration mode (always 2) |
| libpad.md | Polling rate sent | Never sent (setter has no callers) |

## 13. Where the Linux driver stands

See `docs/DEV-NOTES.md` (dev branch) for what is implemented, what is
tested, and how to test.
