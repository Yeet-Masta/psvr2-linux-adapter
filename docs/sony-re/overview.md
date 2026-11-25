# Sony driver reverse engineering: overview

Interoperability notes on Sony's Windows SteamVR driver for the PS VR2. Own words, no Sony code. Tags: **[sure]** read from the code or confirmed by a differential test, **[likely]** strong inference, **[guess]** plausible only. Topic files: [headset.md](headset.md), [controllers.md](controllers.md), [controller-tracking.md](controller-tracking.md), [steamvr-and-display.md](steamvr-and-display.md), [room.md](room.md), [vrtrace.md](vrtrace.md), [function-catalogue.md](function-catalogue.md).

Parts:

- [Part A: Consolidated research](#part-a-consolidated-research)
- [Part B: Driver architecture and code map](#part-b-driver-architecture-and-code-map)
- [Part C: Function triage](#part-c-function-triage)

---

## Part A: Consolidated research

This is the **single source of truth** for what is known about the PS VR2
headset, the Sense controllers, and Sony's Windows SteamVR driver
(`driver_playstation_vr2_orig.dll`, PE timestamp 1741940190). It merges and
corrects:

- part B (the first pass);
- the per-topic notes in `docs/sony-re/`;
- `hardware-protocol.md` part B.

**Where this document and an older one disagree, this one is right.** The
older documents stay for depth: tables, function maps, test notes. Every
section here names the detailed source.

### How to edit this document

- Confidence tags: **[sure]** = read in the code or confirmed by a
  differential test; **[likely]** = strong inference; **[guess]** =
  plausible only. An untagged statement in a table inherits the table's
  tag.
- When a new finding changes a fact:
  1. Change it here.
  2. Add a line to §A.12 (errata), naming the old wrong statement and where it
     appeared.
  3. Change the topic document only if it is still used as the detailed
     reference.
- When hardware testing confirms or refutes a fact, add "(HW yyyy-mm-dd)" to
  its tag.
- Raw values (constants, tables) may be recorded. Sony code may not, not
  even as pseudo-code copied line by line. Describe the behaviour instead.
- Addresses are function VAs in the DLL above. Image base 0x180000000.

### Contents

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

### A.1 Architecture [sure]

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

Detail: `headset.md` §A.1–2.1.

### A.2 Headset USB transport

#### A.2.1 Control transfers (`FUN_1801283f0`) [sure]

| | bmRequestType | bRequest | wValue | wIndex |
|---|---|---|---|---|
| Set | `0x42` | 9 | **0** (always) | 0 |
| Get | `0xc2` | 1 | report id | 0 |

The payload starts with an 8-byte header: `u16 report_id, u16 subcmd,
u16 len, u16 0`, followed by the data. `subcmd` is 1 for everything except
Set `0x05`, which uses 2.

#### A.2.2 Control reports (final list)

| ID | Dir | Len | Meaning | Tag |
|---|---|---|---|---|
| `0x81` | Get | 0x54 | Firmware info. +0x00 = software version, +0x34 = board-stage index (`HardwareRevision_String`), which must be non-zero (retry once after 1 s) | sure |
| `0xa1` | Get | 4 | **Minimum PC-driver version** the headset needs. Sony refuses to load when it exceeds its plugin version (`0x04110006`) | sure |
| `0x8f` | Get | 0x100 | Display/lens calibration block (§A.4.2). Read only on USB 3 | sure |
| `0xde` | Get | 0x18 | Device clock: µs timestamp at +4 (§A.3.1) | sure |
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
| `0x14` | Set | 16 | LED-detection window (§A.7.6) | sure |
| `0x09` | Set | 24 | Blob-detection thresholds (§A.7.6) | sure |
| `0x0a` | Set | 8 + 0xd8 | Relocaliser feedback (§A.10) | sure |
| `0x29` | Set | 1 | Set EDID type (0 / 1, as `0xa9`) | sure |
| `0x25` | Set | 0 | Latch before Get `0xa5` | sure |
| `0x27` | Set | 2 | u16 1000, sent on SteamVR quit | sure |
| `0x05` | Set | 0 | subcmd **2** only: after system resume (default sleep action), and in the direct-mode workaround. Headset restart | sure (meaning: likely) |
| `0x28` | Set | 0 | Only on resume with sleep action 3 | sure |

#### A.2.3 Init sequence on IF7 (status) connect [sure]

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

#### A.2.4 Bulk IN endpoints [sure]

| EP | Size | Content |
|---|---|---|
| `0x83` | SLAM record (§A.5.1) | IF3 |
| `0x85` | gaze | not read by Sony |
| `0x87` | 0x200100 | camera images |
| `0x88` | 0x4000 | status header (32 B) + IMU records |
| `0x89` | 0x9050 | `"LD"` LED detection (§A.7.7) |
| `0x8a` | 0xc8780 | `"RP"` relocaliser frames; **must be drained** |
| `0x8b` | 0x40000 | `"VD"` calibration container (§A.3.3), on IF10 (thread `CaesarUsbThreadGenData`) |
| `0x8d` | 0x100000 | firmware console log, ANSI text, on IF12 (thread `CaesarUsbThreadLog`) |

#### A.2.5 Host policies [sure]

- **See-through button:** a rising edge of status byte 2 while the headset
  is worn toggles Set `0x17`. Taking the headset off forces see-through
  off. Get `0x97` is polled once a second.
- **Brightness follow:** when status byte 10 differs from the configured
  brightness, re-send Set `0x12`.
- **Power:**
  - resume: Set `0x05` subcmd 2;
  - SteamVR quit: Set `0x27` (u16 1000).

Detail: `headset.md` §A.2–§A.5.

### A.3 Headset data streams

#### A.3.1 Device clock (Get `0xde`) [sure]

- Poll every 500 ms. The host stamps the time before and after each call.
- Keep a sample only when RTT < 1000 µs. Its offset is
  `host_before + RTT/2 − device_us`.
- Keep a 100-entry ring, and reset it when the offset jumps by more than
  2000 µs.
- Fit a phase and drift from the ring, with the drift clamped to
  ±50 000 ppb.
- The device clock is a **32-bit µs counter that wraps**. All headset
  timestamps (SLAM, IMU, LD) use it.

#### A.3.2 Status stream (EP `0x88`) [sure unless tagged]

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

Detail: `headset.md` §A.2.7, `hardware-protocol.md` part B.

#### A.3.3 Calibration container (EP `0x8b`) [sure]

- Header: `"VD"`, u16 version `0x0200`, u32 size at +4, u32 count at +8.
- Entries start at +0x20: `{u32 type, u32 size, u32 offset}`.

| Type | Size | Content |
|---|---|---|
| 10 | ≤ 0x800 | Tracking-camera calibration: **all 4 cameras and 3 extrinsics** (§A.8). The only one the IR tracker uses |
| 11 | ≤ 0x800 | Second camera calibration (VrTrace), unused by tracking |
| 12 | | IMU calibration |
| 13 | 0x34 | `imu_set_calib.bin`: starts with a quaternion (x, y, z, w) that rotates config pose A (IMU ↔ head/rig, `controller-tracking.md` §C.2.4–2.5); +0x28..+0x30 are used negated by the head INS [sure] |

Sony re-sends camera mode `0x10` once a second until types 10 and 12 have
arrived.

#### A.3.4 Camera calibration blob (types 10/11, `FUN_180161b70`) [sure]

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
| +0x2c | **20 f64**, D0..D19 (§A.8.1) |

Extrinsic record (0x34 bytes):

| Off | Meaning |
|---|---|
| +0 | u8 from |
| +1 | u8 to |
| +4 | 9 f32, R (row-major) |
| +0x28 | 3 f32, t (metres) |

#### A.3.5 Other streams

- **SLAM:** §A.5.1.
- **LD:** §A.7.7.
- **RP:** §A.10.
- **Firmware log:** strip the ANSI codes and log it.

### A.4 Display and the SteamVR HMD device

Detail: `steamvr-and-display.md` part B, `steamvr-and-display.md` part A (both difftested).

#### A.4.1 Panel and timing [sure]

- Panel 4000 × 2040; per-eye viewport 2000 × 2040.
- Recommended render target 3400 × 3468.
- Vsync-to-photons 1.36 / 1.02 ms; photons-to-vblank 10.3 / 7.73 ms (at
  90 / 120 Hz).
- Projection tangents 1.84177 / 0.94719 / 1.32898.
- DistortionMeshResolution 80.
- IPD UI range 0.059–0.072 m. IPD = status byte 5 mm × 0.001.

#### A.4.2 EDID / refresh (`FUN_1801a4390`) [sure]

- Read Get `0xa9`. If SteamVR wants 120 Hz and the EDID is 90-only, Set
  `0x29` = 0. If it wants 90 Hz and the EDID allows 120, Set `0x29` = 1.
- Then wait 1000 ms: the headset re-enumerates its DisplayPort sink.
- 120 Hz is offered only when the firmware is later than `0x06000000`.

#### A.4.3 Lens model [sure, difftested]

- The lens centres and panel rotations come from the `0x8f` block. Version
  byte < 4 means fixed defaults.
- Distortion uses three 1024-entry LUTs (R, G, B) at
  `0x180261180 / 0x180262180 / 0x180263180`.
- **Red and green get the −0.00023027 y offset; blue does not.**
- Hidden-area mesh: 32 rays starting at r = 0.9 in steps of 0.004, 36
  triangles, mirrored for the right eye.

#### A.4.4 HMD pose packaging [sure]

- Query the tracker at "now". `poseTimeOffset = t_query − t_send`.
- Poll every 1 ms while ready, every 100 ms otherwise.
- **Driver-from-head transform** (IMU rig → head):
  - `qDriverFromHeadRotation (w, x, y, z) = (−0.5, −0.5, 0.5, 0.5)`;
  - `vecDriverFromHeadTranslation = (−0.09255, 8e-9, 0.02080)` m, in
    IMU-rig axes.
- `qWorldFromDriver` = identity.
- Not-worn throttle: 5 mm, 0.03 rad/s, 500 001 µs.
- HmdTrackingStyle = 3 (inside-out cameras).

### A.5 Head tracking

Detail: `controller-tracking.md` §A.2–§A.5.2, `controller-tracking.md` §B.4.

#### A.5.1 SLAM record (EP `0x83`) [sure]

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

#### A.5.2 Mapping to the Linux driver [sure, numeric check]

The Linux remap (quaternion `(q0, −q2, −q1, q3)`, then Rz(+90°), position
`(r2, r1, −r0)`) equals Sony's `q_dfh` folded into the rotation, plus a
world yaw of −90° about Y. It is correct because its "w, x, y, z" naming
and its permutation cancel.

**What it lacks** is Sony's lever arm. In Linux head axes the head origin
is at `(0, +0.0208, +0.09255)` m from the IMU: 2.1 cm up and 9.3 cm back.
Report it as `vecDriverFromHeadTranslation`.

#### A.5.3 Host INS (Sony) [sure unless tagged]

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

### A.6 Sense controllers

Detail: `controllers.md` part B, `controllers.md` part A.

#### A.6.1 Identification and features [sure]

- VID `054c`; PID `0e45` (left), `0e46` (right). `0ce7` is also a VR
  controller; its hand comes from feature `0x20`.
- Transport by input length: 64 bytes = USB, 78 bytes = Bluetooth.
- **Feature `0x20`** (64 bytes):
  - +0x18 hardware info; its low 16 bits are the **hardware rev**, which
    selects the LED model (§A.9.1);
  - +0x1c **main firmware version** (u32);
  - +0x2c firmware revision;
  - +0x30 bit 0, hand.
- **Feature `0x05`**: IMU calibration in two parts, DualSense-style math.
- IMU ticks run at 3 MHz.
- Sony never sends feature `0x08` (polling rate): the setter has no
  callers.

#### A.6.2 Output report [sure]

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
  which drives the clock fit (§A.6.4).
- Vibration: amplitude = ceil(a × 20) with flag1 `0x40 | 0x03`. Events
  below 0.049 are ignored; the minimum pulse is 80 ms; re-send every
  100 ms. The pad vibration mode is always 2.
- Reduction byte (flag1 `0x10`): high nibble trigger, low nibble haptics,
  12.5 % steps.
- Adaptive triggers: libpad can encode them, but Sony's SteamVR driver
  never calls them (optional on Linux).

#### A.6.3 Battery [sure]

Level 0..9 → `10n + 5` %, 10 → 100 %, 11..15 → 0 %.

#### A.6.4 Host ↔ controller clock fit (libpad `[HID_SYNC]`) [sure]

- Every input report feeds the fit with (host receive time, echoed host
  stamp, controller ticks).
- The output: `host_to_ctrl(t) = (drift·ref + (t − fit_off)·1e9) / (drift + 1e9)`.
- It is needed only for LED sync.
- Implemented bit-exact in `userspace/lib/sie/libpad/psvr2_hidsync.c`.

### A.7 LED sync

Detail: `controllers.md` part C (read it together with the corrections in
§A.12), `controller-tracking.md` §B.3.

#### A.7.1 Purpose

The PC makes each controller pulse its LEDs while the headset's camera
shutters are open. It does this by putting a phase, an LED set, a time
offset and a frame period into every controller output report.

#### A.7.2 Phases [sure]

| # | Name | period_id (preset 1 / 2) | Report LEDs |
|---|---|---|---|
| 0 | INIT | – | – |
| 1 | PRESCAN | 40 / 40 | ff |
| 2 | BROAD | 50 / 42 | sync LEDs |
| 3 | BG | 30 / 30 | sync LEDs |
| 4 | STABLE | 12 / 12 | ff |
| 5 | LED ALL OFF | – | ff |
| 6 | LED ALL ON | – | ff (not used) |

#### A.7.3 Commands to libpad [sure]

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

#### A.7.4 Sync mode (libpad, `FUN_1801cbeb0`, `FUN_1801c1820`) [sure]

- The block runs only while the mode is 0.
- If feature `0x20` +0x1c (firmware) < `0x1006d`, or the probe has been
  tried more than 3 times: **mode 2**.
- Otherwise: Set `0x80` `80 13 22 00…` (64 bytes), then Get `0x81`. Reply
  byte 0 = `0x81` → mode 2, else 0. Count the try.
- Mode 2 = width 500, step 50. Mode 1 (1000/105) is **never set** in this
  build. The reset values, used while the mode is 0, are 1000/105/1575.

**Steady state is always mode 2.**

#### A.7.5 Presets [sure]

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
  controller state sample. Its HID origin is open (§A.11).

#### A.7.6 Headset detection config [sure]

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

#### A.7.7 LD packet (EP `0x89`, 0x9050 bytes) [sure]

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

- The blob record decode (`FUN_180184900`) is in part B.
  Implemented in `sie/irtracker/psvr2_ld.c`, difftested.
- Sony waits up to 100 ms per tracker cycle for a new packet.

#### A.7.8 Per-camera history and states (`FUN_180194f70`) [sure]

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

#### A.7.9 BROAD [sure]

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

#### A.7.10 BG [sure]

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

#### A.7.11 STABLE [sure]

- Value = `BG/2 − STABLE/2 + step·(10 − x)`. LEDs ff (the controller keeps
  its last pattern).
- Every 100 s while visible, drop back to BG.
- Invisible for ≥ 15 s → PRESCAN.

#### A.7.12 Drift fit [sure]

- Runs of equal values are merged (within DBL_EPSILON).
- With ≥ 4 runs, fit a least-squares slope over all runs except the first.
  The fit latches until it is reset.
- t is in µs since the first sample, LD frame time on the host clock
  [sure] (`controller-tracking.md` §B.3.5).
- The slope is sent **raw** in cmd 3, or cmd 5 with an offset. BROAD: the
  fit holds µs, so the slope is dimensionless. BG/STABLE: steps/µs. A
  positive slope lengthens the cycle.
- Sent whenever |slope| > DBL_EPSILON. A send resets the retry counter.

#### A.7.13 Forced transitions [sure]

| Condition | Result |
|---|---|
| Controller not connected, preset mismatch, > 5 s since its last pass, or connection id changed | INIT |
| Tracker disabled | LED ALL OFF |
| See-through on | LED ALL OFF for both; INIT when it ends |
| Extreme-Env mode 3 | PRESCAN |
| Phase 0, 5 or 6 at the end of the checks | PRESCAN |

#### A.7.14 Sync LED choice (`FUN_1801999f0`) [sure]

- Candidates: eligible LEDs with an associated blob and `0 < metric·0.5 ≤ 45`,
  where the metric is the LED-normal ↔ view-ray angle in degrees.
- Keep N, the sync-LED count (default 1), with the smallest metric. Take
  the camera with the most kept LEDs.
- A success less than 1 s ago keeps the current LED.
- LED bytes are **physical** indices. For logical LED k: k ≥ 1 → k;
  0 → the LED count (20 for rev 1, 17 otherwise). Every LED is eligible.

### A.8 Tracking cameras: model and extrinsics

Detail: `controller-tracking.md` §B.1.

#### A.8.1 Calibration to model [sure for the data flow; likely for the OpenCV names]

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

#### A.8.2 Projection (`FUN_180045650`) [sure]

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

#### A.8.3 Extrinsics [sure unless tagged]

- Calibration R, t (metres) become rig 4×4 matrices in **mm**:
  `R′ = G R G`, `t′ = 1000·G t`, with `G = diag(−1, −1, 1)`. This is the
  same 180° rotation as §A.8.2.
- Each camera is placed in the rig frame by composing its extrinsic with
  camera 0's built-in rig pose. The rig frame is **not** camera 0's frame:
  camera 0 sits at its built-in pose, cameras 1–3 at that pose ∘ extrinsic.
- The solver's head chain is `world_from_X = INS · A⁻¹ · B`, with A and B
  from `controller-tracking.md` §C.2.4.
- An extrinsic recorded as from 2 → to 0 is inverted on load. So the
  expected direction is "0 → k". §A.8.4 gives the measured mapping.
- The built-in table has three variants with full rotations
  (`controller-tracking.md` §C.2.1); the measured headset's 15° toe-out matches
  variant 1/2. Translations, variant 0 (mm):

  | Cam | Translation |
  |---|---|
  | 0 | (+40.0, −41.68, 93.58) |
  | 1 | (−40.0, −41.68, 93.58) |
  | 2 | (+61.5, 22.97, 97.03) |
  | 3 | (−61.5, 22.97, 97.03) |

  0/1 are the inner pair 80 mm apart; 2/3 are the outer pair.

#### A.8.4 Measured on hardware (one headset, 2026-11-18) [sure]

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

### A.9 Controller pose solve and fusion

Detail: `controller-tracking.md` §B.2, `controller-tracking.md` §A.5.3–5.6, `controller-tracking.md` part D,
`controller-tracking.md` part H.

#### A.9.1 LED models [sure]

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
  constant IMU → controller 4×4, but the call always passes the pad vector,
  so the stored transform is a pure translation by pad `+0x48`
  (R (−20.0727, −12.2481, −6.0039) mm, L (9.3727, …) mm). The tables apply
  only to a NULL vector (`controller-tracking.md` §C.2b.1).

#### A.9.2 Boot config (0x54 block, names from `ConfigFile0.trk` keys) [sure]

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

#### A.9.3 Pipeline [sure unless tagged]

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
     every hypothesis passes the pose check (`controller-tracking.md` §B.2.19);
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

#### A.9.4 UKF [sure unless tagged]

- Error state of 15: position, velocity, attitude, gyro bias, accel bias.
- P0: 0.0025, 0.04, (1°, 5°, 1°)². Yaw is the loose axis.
- Delayed optical measurements: apply at their own timestamp and replay
  the IMU (up to 31 states).
- Stationary damping: velocity × 0.99 when |a − g| < 1.96 m/s².
- Divergence guard: reset the velocity and biases. A full reset follows a
  BadState.
- Optical measurement = full pose. If it is 0.3 s stale, the position is
  held, rotation comes from the UKF and the status is 3; `poseIsValid`
  stays 1 (`controller-tracking.md` part C).
- The process noise depends on angular rate (`controller-tracking.md` part H). Several
  limits come from config and were not read (§A.11).

### A.10 Room, play area, relocaliser

Detail: `room.md` part A, `room.md` part B.

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

### A.11 Open questions

| # | Question | Next step |
|---|---|---|
| 1–3 | Camera size, extrinsic direction, camera index: answered, §A.8.4 | – |
| 4 | HID origin of the controller "preset" (libpad state +0x6f) | Trace libpad input parse |
| 5 | Time unit of the drift-fit t; does BROAD shift slot −1 into its 3-history? | `FUN_180196ff0` caller, `FUN_180198240` |
| 6 | Candidate score inside the gated match: answered (`controller-tracking.md` §B.2.11). Open: LM stop tolerances; the 0.25 option | `FUN_1800f7970` |
| 7 | Unit of the multi-camera boot error e | `FUN_1800e49b0` |
| 8 | Cross-controller cleanup | `FUN_1800a5e30` → `FUN_1800ad2b0` |
| 9 | UKF config limits (`+0x798..+0x7a0`, `+0x784`, `+0x864`) and the exact process noise | `controller-tracking.md` part H, config reader |
| 10 | `FUN_1800e1c90` "20° cone" (`cos 0.349`) vs the 170°/180° view-angle limit: two different tests? | Read both call sites |
| 11 | Meaning of the Set `0x1f` table and the `0x14` A value | – |
| 12 | Hand-1 descriptor byte `0x58`: answered, it is the char `'X'`; the left model is mirrored in x (`controller-tracking.md` §C.2b.2) | – |
| 13 | Per-frame tracking acceptance: answered. ≥ 3 matches, then gravity 20°, orientation 30°, depth 5–1500 mm (`controller-tracking.md` §B.2.16). Our stricter 3/4-facing / 1.5 px rule stays on top (`own_accept`) | – |
| 14 | Blob → sync-LED assignment: answered. There is none without a pose: `FUN_1801999f0` fills node `+0x24` from the pose tracker's LED → blob map, else 0 (`controller-tracking.md` §B.3.1). BlinkFinder stays as our fallback for a hand without a pose | – |

### A.12 Errata (superseded statements)

Every statement below has been corrected in place; this table is the only
record of what changed. "Where" names the original file, now a part of a
merged file.

| Where | Old statement | Correct |
|---|---|---|
| part B | `0xa1` meaning unknown | Minimum PC-driver version (§A.2.2) |
| part B | `0x8f` gate "version ≥ 0x300 at +2" | USB descriptor `bcdUSB ≥ 0x300` |
| part B | Alt setting on a "control interface" | Alt setting 1 on **IF7** |
| part B | Thread names of `0x8b` / `0x8d` | Swapped (§A.2.4) |
| part B, hardware-protocol.md part B | Set wValue = report id | Set wValue = 0 |
| part B | SLAM +0x0c = 3 means lost | 3 = tracking |
| part B | Camera record holds 19 doubles | 20 doubles (D0..D19) |
| part B | LED-sync cmd table (cmd 5 "sync loss", cmd 7 "modes") | §A.7.3 |
| part B | `0x14` defaults are the active values | Startup only; the presets apply later (§A.7.6) |
| part B | LD +0x20/+0x24 "exposure/sync phase" | Float echoes of `0x14` A/B |
| part B | Output +0x14 written "when a sync request is pending" | When phase ∉ 1..4 or the clock fit has a reference |
| controllers.md §C.2.2 | Mode 1 for firmware < 0x1006d or a failed probe | **Mode 2**; mode 1 is never set |
| controllers.md §C.5.3 | BG estimator uses samples above the floor; skip is relative to the last x | Below max/2; skip is relative to 10 |
| controllers.md §C.5.5 | Units [guess] | Slope raw, steps or slots per µs |
| controllers.md §C.2.9 | BG pattern = 32 frame bytes | 4 s32 pairs: frames 0–3 and 17–19 |
| controller-tracking.md §A.5.3 | Extrinsics "z axis flipped" | 180° about z: t′ = 1000·(−tx, −ty, tz) |
| controller-tracking.md §A.5.3 | P1PFromFailedUndistort = 1-point fix after an undistort failure | It is the **lost** state; no undistort iteration |
| controller-tracking.md §A.8.1 | SLAM quaternion order open | x, y, z, w; the Linux remap is right |
| controller-tracking.md part A / earlier guesses | 0x54 tuning block names | `ConfigFile0.trk` keys (§A.9.2) |
| controller-tracking.md §A.6 | LED cone one constant | Per model, 170° or 180° |
| controller-tracking.md part A, controller-tracking.md part D | RANSAC 1000 iterations | 200 cold, 50 while tracking |
| controller-tracking.md part A, part A §A.9 | Boot RANSAC over 6-point minimal sets, adaptive count | 3 LEDs × 3 blobs (P3P), ring stage first; adaptive flag 0 (`controller-tracking.md` §B.2.19) |
| earlier guesses | Gravity/orientation boot limits from config | Fixed 3°/15° (6°/30° close) |
| development.md part A | Camera-model builder `FUN_1800c8950/8a40`, 0x180-byte struct | `FUN_180045420` + `FUN_18009d4d0`; 0x1c0 bytes; those two are RANSAC settings |
| controllers.md part A | Pad +6 "connection kind" | Vibration mode (always 2) |
| controllers.md part A | Polling rate sent | Never sent (setter has no callers) |
| part B | `0x8f` "hardware info/calibration [guess]" | Display/lens calibration block, offsets 0x00–0x23 used |
| part B | Alt setting after the firmware info, then `0x22`, `0x12` | Last step but one of init: after `0x17` and `0x22`, before `0x12` |
| part B | `0x81` layout open | +0x00 software version, +0x34 board-stage index |
| part B | `0x8a` Get "vtable only" | Issued on IF9 connect; value never stored |
| part B | `0x05` / `0x28` senders | `0x05` only with subcmd 2 (resume, direct-mode workaround); `0x28` only on resume with sleep action 3 |
| part B | `0x0b` "re-sent when the config changes" | Default `0x10`; only VrTrace capture changes it |
| part B | `0x0a` row [likely] | [sure]: always 0xd8, subcmd 1/3/4, 20 ms pacing (room.md §A.2.2) |
| part B | Bulk `0x8a` payload | Also 8 u32 command slots at +0x08 (room.md §A.2.1) |
| part B | SLAM +0x2c/+0x48/+0x54 "likely" | `wTi` reference pose, `wVi` velocity, gyro bias, accel bias (+0x60) |
| part B | SLAM +0x144 always applied | Only for record versions < `0x110` |
| part B | SLAM +0x184 / +0x1e0 "copied" | Error variance; SLAM reset counter; +0x1e4 map id |
| part B | Headset IMU 1 kHz | 2 kHz (500 µs) [likely] |
| part B | LED cmd 1 = phase + 4 LED bytes | `[01, phase, period_id, led0..3, u32]`, len 11 / 7 / 2; physical LED indices |
| part B | LED cmd 3 sent above a drift threshold | Threshold is DBL_EPSILON: any non-zero slope |
| part B | LED cmd 6 | Ignored by libpad |
| part B | Sync commands need tracker state 4 | LedSyncer context state 4 (ready), hand 0 = right, 1 = left |
| part B, controllers.md part B | Feature `0x20` for DualSense only; BT output framing | Also VR controllers: hand bit +0x30, revision +0x18, firmware +0x1c; BT output `31 a2 …` + CRC, no tag |
| part B | Code map `0x180120000–0x18013ffff` partly | Map save scheduling, play area, chaperone covered (room.md part A) |
| controllers.md part A | Firmware gate "below `0x1006d`" source | Feature `0x20` +0x1c (device slot +0x1d0) |
| controller-tracking.md §A.5.6 | IMU → controller translation in mm | Metres; table used only without a pad vector |
| controller-tracking.md §A.5.3 | Method enum from names | Values 0 none … 8 kMultiCamP2PR (`controller-tracking.md` §B.1.8) |
| controller-tracking.md §A.5.3 | Optical axis "z flipped" in prose | `FUN_1800b02a0` also uses the 180° rotation |
| controller-tracking.md §B.2.5 | RANSAC cap 1000 | Default only; boot sets 200 cold, 50 in tracking |
| controller-tracking.md §B.2.8 | LED visibility as a half-angle test | View angle compared directly with 170°/180° (174° in association mode 1) |
| controller-tracking.md part A | Robust kernel unspecified | Huber, c = 1.0 on a 1.483·median scale; none with exactly 3 LEDs (`controller-tracking.md` §B.2.10) |
| development.md part A | Model resolution unstated | 508 × 508; builder called from `FUN_180049240` on data from `FUN_180182e80` |
| part A §A.3.3 | Type 13 = IMU ↔ camera [likely] | `imu_set_calib.bin`, quaternion rotating config pose A |
| part A §A.8.3 | Rig frame = camera 0's frame; variant 0 only | Not camera 0's frame; three variants with rotations |
| part A §A.5 / §A.8 | Head chain missing | `world_from_X = INS · A⁻¹ · B` |
| part A §A.9.1 | `ImuPoseIndex` table always used | Pure translation by pad `+0x48`; table only for a NULL vector |
| part A §A.9.4 | Stale optical pose unspecified | 0.3 s stale: position held, rotation from UKF, status 3 |
| function-catalogue.md part F | `0x1801bf850` returns the libpad version word | It writes the host µs clock (`0x1801c8210`) through its pointer argument |
| controller-tracking.md §G.2 and controller-tracking.md §G.3 | Process callback `0x1800638d0`; `h(x)` = slot +0x18 `0x18005fad0` | `0x1800638d0` is the work-area allocator; process + measurement are one per-sigma-point function `0x18006f020`, `h` is slot +0x48 `0x180060030` (controller-tracking.md §G.2.1) |

### A.13 Where the Linux driver stands

See `development.md` part A (dev branch) for what is implemented, what is
tested, and how to test.

---

## Part B: Driver architecture and code map

Interoperability notes on Sony's Windows SteamVR driver
(`PlayStation VR2 App/SteamVR_Plug-In/bin/win64/driver_playstation_vr2.dll`;
PSVR2 Toolkit renames it `driver_playstation_vr2_orig.dll`). The goal is to
document what the hardware does so this project can implement it
independently. **No Sony code is copied here or committed.** The decompiled
output stays on the researcher's machine. Reproduce it with
[`scripts/sony-re/decompile.sh`](../scripts/sony-re/decompile.sh)
(Ghidra 12.2 headless, about 5 minutes).

Function names such as `FUN_1801283f0` are Ghidra's addresses in the build
analysed: 3 575 976 bytes, PE timestamp `1741940190` (2025-03-14), image base
`0x180000000`, sole export `HmdDriverFactory`. They change with each build.
Confidence tags: **[sure]** read directly from code, **[likely]** inferred
from strings or call context, **[guess]** untested hypothesis.

### Architecture

The DLL links Sony's PS5 tracking stack (`SceVrTracker2`), ported to PC. The
modules below come from RTTI and log strings.

| Area | Classes / strings | Role |
|------|-------------------|------|
| USB | `CaesarManager`, `CaesarUsbThread{Image,ImuStatus,Leddet,SlamTracking,RelocPre,GenData,Log}` | One reader thread per headset bulk endpoint (WinUSB) |
| Controller tracking | `IRTrackerThread`, `OIFusion::OpticalMarkerData`, `DotMarkerData`, `RnD::PoseCostMultiCam`, `RnD::Robustifier{Huber,Tukey,Clip}`, `UkfManager`, `LedSyncer`, `controller_tracking.bin` | 6DoF Sense tracking: LED blobs from the headset, then a multi-camera pose solve, then UKF fusion with the controller IMU |
| Map / play area | `Relocalizer`, `RelocalizerThread`, `InsManager`, chaperone / play-area / "safety notice" strings | Room map save/load, relocalisation, boundary |
| Hands | `HandSkeletonWithController`, `HandSkeletonWithoutController` | Finger skeleton from Sense touch/grip inputs |
| Calibration | `selfcalibration{1,2}.bin`, `vrt_camera_calib_vrtrace{0,1}.bin`, `imu_calib.bin` | Camera self-calibration, factory camera/IMU calibration |
| Controllers | `scePad*`, `SHARE_VRT2_WIN_LIBPAD_*` named mutex/events | Sony libpad (DS4/DualSense/Sense HID), shared with the PS VR2 app |
| Debug | `VrTrace*` | Capture/replay of every stream to `.bin` files |

**Eye tracking is not in Sony's driver [sure].** It never reads endpoint
`0x85` and never sends control report `0x0c` (the only `WinUsb_ReadPipe` and
`WinUsb_ControlTransfer` call sites are listed below). PSVR2 Toolkit adds gaze
itself, and this project already decodes it (see
[hardware-protocol.md part B](../hardware-protocol.md#part-b-usb-protocol)). The DLL holds no gaze model,
gaze calibration or foveation logic to recover.

### Headset USB transport

#### Control transfers (`FUN_1801283f0`) [sure]

These are vendor transfers on ep0, carrying the `sie_ctrl_pkt` from
[hardware-protocol.md part B](../hardware-protocol.md#part-b-usb-protocol):

| | bmRequestType | bRequest | wValue | wIndex | wLength |
|-|---------------|----------|--------|--------|---------|
| Set | `0x42` | `0x09` | `0` | `0` | `len + 8` |
| Get | `0xc2` | `0x01` | report_id | `0` | `len + 8` |

The payload header is `u8 report_id, u8 0, u16 subcmd, u16 len, u16 0`, then
`len` data bytes. The buffer is 512 bytes. Sony's Set uses `wValue = 0`; the
`report_id` lives only in the payload.

Sony sets **alternate setting 1 on IF7** (the status/IMU interface, through
its own WinUSB handle) as the last step but one of init: after `0x17` and
`0x22`, before `0x12` (`FUN_1801256c0`).

#### Bulk IN endpoints (`FUN_180127d60` callers) [sure]

| EP | Read size | Thread (from `[USB] fps imu/img/trk/led/rel` log) | Notes |
|----|-----------|-------------------------------|-------|
| `0x83` | 0x200 | SlamTracking ("trk") | `SLA` pose record, already supported |
| `0x87` | 0x200100 | Image ("img") | Camera frames, already partly supported |
| `0x88` | 0x4000 | ImuStatus ("imu") | Status + IMU, already supported |
| `0x89` | 0x9050 | Leddet ("led") | **LED detection results**: magic `"LD"` at offset 0; see below |
| `0x8a` | 0xc8780 | RelocPre ("rel") | Magic `"RP"`, u32 size at +4 must be 0xc8780 [sure]; relocaliser keyframe/feature data for the PC-side relocaliser [likely]. 8 u32 command slots at +0x08 let the headset drive the host relocaliser [sure] |
| `0x8b` | 0x40000 | GenData (`CaesarUsbThreadGenData`, IF10) | **Calibration container `"VD"`**, see below. The same thread re-sends camera mode `0x0b` once a second until camera calib (type 10) and IMU calib (type 12) have arrived |
| `0x8d` | 0x100000 | Log (`CaesarUsbThreadLog`, IF12) | **Headset firmware console log**: newline-separated text with ANSI colour codes, which Sony strips before logging it [sure]. Useful for debugging |

`0x85` (gaze) is not read.

#### Control reports

`Get` = device to host. Len = payload bytes.

| ID | Dir | Len | subcmd | Meaning | Where |
|----|-----|-----|--------|---------|-------|
| `0x81` | Get | 0x54 | 1 | Firmware info. Init fails ("Failed to read PS VR2 firmware info") if the dword at +0x34 is 0; retried once after 1 s. +0x00 = software version, +0x34 = board-stage index (`HardwareRevision_String`) **[sure]** | `FUN_180124450` |
| `0xa1` | Get | 4 | 1 | **Minimum PC-driver version** the headset firmware requires. Compared at server init with the plugin version (`version.txt` `plugin version`, default `0x04110006`); if it is larger, Sony refuses to load and shows the update dialog (`FUN_1801a2040`) **[likely]**. This is the check behind flag `+0x289` | `FUN_1801256c0` |
| `0x8f` | Get | 0x100 | 1 | 256-byte block handed to the config manager; read only if the USB device descriptor's `bcdUSB` is ≥ 0x300 (USB3, `FUN_180124510`) **[sure]**; display/lens calibration block, offsets 0x00–0x23 used **[sure]** | `FUN_1801256c0` |
| `0xde` | Get | 0x18 | 1 | **Clock sync**: device timestamp (µs) at +4. Host stamps before/after; samples with RTT < 1000 µs are kept; offset = host_before + RTT/2 − device_ts, averaged over a 100-entry ring; reset on a jump > 2000 µs **[sure]** | `FUN_180123fa0` |
| `0x97` | Get | 1 | 1 | See-through state, polled every second (see the status stream) **[likely]** | `FUN_1801268d0` |
| `0xa4` | Get | 4 | 1 | Controller-tracking related dword **[likely]** | `FUN_180124ee0`, caller `FUN_18011bae0` |
| `0xa6` | Get | 0x20 | 1 | 8 dwords, controller-tracking related **[likely]** | `FUN_180124c00` |
| `0xa5` | Get | 0x60 | 1 | **DisplayPort receiver status**, read after Set `0x25` (latch) **[sure]**; layout below | `FUN_1801248d0` |
| `0xa9` | Get | 1 | 1 | **Current EDID type**: 0 = 90/120 Hz, 1 = 90 Hz only **[sure]** | `FUN_180124970` |
| `0x8a` | Get | 8 | 1 | Issued on IF9 connect, but the value is never stored (return-value check bug) **[sure]** | `FUN_180125a50` |
| `0x12` | Set | 1 | 1 | Panel brightness; Sony's init default is `0x1f` **[sure]** | `FUN_180123f60` |
| `0x17` | Set | 2 | 1 | `{on, 3}`: see-through toggle (see the status stream) **[likely]**; init sends 0 | `FUN_180124580` |
| `0x22` | Set | 1 | 1 | Init, a byte from device state **[sure]** | `FUN_1801256c0` |
| `0x25` | Set | 0 | 1 | Request that precedes Get `0xa5` **[sure]** | `FUN_1801248d0` |
| `0x29` | Set | 1 | 1 | **Set EDID type**: 0 = 90/120 Hz, 1 = 90 Hz only. Sony picks it from SteamVR's `preferredRefreshRate` **[sure]** | `FUN_180127e80`, `FUN_1801a4390` |
| `0x27` | Set | 2 | 1 | u16 `1000`, sent from `RunFrame`'s direct-mode check; a timeout in ms **[guess]** | `FUN_180125610` |
| `0x05` | Set | 0 | n+1 | `subcmd = flag + 1`; only ever sent with subcmd 2 (flag 1), from the resume handler (default sleep action) and the direct-mode workaround **[sure]** | `FUN_180127fa0` |
| `0x28` | Set | 0 | 1 | Sent only on resume with sleep action 3 (`FUN_1801a3c00`) **[sure]**, meaning unknown | `FUN_180127e40` |
| `0x0b` | Set | 8 | 1 | Camera mode `{1, mode}`, default `0x10`; only VrTrace capture changes it. Re-sent at most once per second **[sure]** | `FUN_180125cc0` |
| `0x14` | Set | 0x10 | 1 | **LED-detection config**: startup values `0x55`, `500`, `0x24800000 \| 1`; the active values come later from the preset (128/1000 or 85/500) and dword 4 becomes 0x100 **[sure]**; field meaning **[guess]**: threshold, exposure, flags | `FUN_180126b80` |
| `0x09` | Set | 0x18 | 1 | Second LED-detection/sync config block (24 bytes) from the config manager **[sure]** | `FUN_180126b80` |
| `0x1f` | Set | 0x100 | 1 | 256-byte block built from constants (`0x28`, `10`, `5`, 16-byte tables) **[sure]**, meaning unknown | `FUN_180125ab0` |
| `0x0a` | Set | 0xd8 | n | Relocaliser ("setRelocPreOutput"): length always 0xd8; `subcmd` selects the payload (1 = localiser result, 3 = parameter block, 4 = start); sent through `FUN_1801adba0` with 20 ms pacing **[sure]** | `FUN_180127fe0`, `FUN_1801adba0` |

Not seen from the host: `0x08` (haptics in hardware-protocol.md part B) and `0x0c` (gaze);
both are toolkit/Monado territory.

#### SLAM record (`0x83`, `FUN_1801270c0` → `FUN_18017a5d0`)

Sony checks `"SL"` (bytes 0..1) and that the high byte of the **u16 version
at +2** is `0x01`. hardware-protocol.md part B's `"SLA"` + `const1 = 0x01` is this version
read as text: firmware sends `0x0141`. Fields by version (offsets in the
512-byte record; [sure] for the version gates, [likely] for the meanings):

| Offset | Type | Use in Sony's driver |
|--------|------|----------------------|
| 0x08 | u32 | Device timestamp (µs) |
| 0x0c | u8 | Tracking state; **3 = tracking** (valid 6DoF). Valid = `state == 3 \|\| +0x0e == 1 \|\| +0x0d == 5` (`FUN_18017a5d0`) [sure] |
| 0x0d | u8 | Sub-state; 5 = special handling |
| 0x0e | u8 | Flag; 1 = special handling |
| 0x0f | u8 | Present from version 0x120 |
| 0x10 | float[3] + float[4] | Head pose: position (m), then the quaternion at +0x1c, which Sony consumes as **x, y, z, w** [sure]. The Linux remap in hardware-protocol.md part B is right (`controller-tracking.md` §B.4.2) |
| 0x2c | float[3] + float[4] | Second pose (`wTi` reference) [sure]; combined with 0x144 for the play-area frame [likely] |
| 0x48 | float[3] | Linear velocity (`wVi`) [sure] |
| 0x54 | float[6] | Gyro bias (+0x54) and accel bias (+0x60) [sure: log strings in `FUN_180180200`] |
| 0x144 | float[3] + float[4] | Map/anchor transform applied to the poses above [likely]; applied only for record versions < `0x110` [sure] |
| 0x160..0x168 | float ×3 | Compared by magnitude, version ≥ 0x150 |
| 0x184, 0x188 | u32 | Copied per frame; +0x184 is an error variance (sqrt → quality EMA) [sure] |
| 0x1c8 | i32 | Map slot 0..2, version ≥ 0x120 (out of range means 0) |
| 0x1e0 | u8 | SLAM reset counter, version ≥ 0x131; +0x1e4 is the map id (upper 16 bits latch) [sure] |
| 0x1e8 | float | Defaults to 0.08 when the version is older than 0x134 |
| 0x1ec | u16, 0x1ee u8, 0x1ef u8 | Version ≥ 0x134 / 0x135 / 0x141 |

The driver latches the timestamp for latency stats like the LD stream.

#### Status stream (`0x88`, `FUN_1801268d0`)

Sony copies the first 32 bytes (the status header) every transfer and then
parses the IMU records (`FUN_180127620`). Additions to hardware-protocol.md part B:

- Header byte 10 = **current panel brightness**. If it differs from the
  configured brightness, Sony re-sends Set `0x12` [sure].
- Header byte 1 (worn) and byte 2 (function button) drive Set **`0x17`**
  `{on, 3}`. On a button press while worn, Sony toggles it, using the state
  from Get **`0x97`** (1 byte, polled every second) [sure]. This is the
  function-button **see-through (passthrough) toggle** [likely, matches the
  "nonvr seeThrough off" log].
- **PC-support check:** three Get `0xde` reads; if every device timestamp is
  0, the firmware does not support PC mode [sure].
- Clock sync (`0xde`) repeats at a configured interval (`+0x224`, µs) [sure].
- USB3 gate: if the device descriptor's `bcdUSB` is below 0x300
  (`FUN_180124510`), none of this runs [sure].
- The headset IMU runs at **2 kHz** (500 µs) [likely: rate check and
  averager config].

#### Image stream (`0x87`, `FUN_180125e00`) [sure unless tagged]

Each transfer starts with a 256-byte header; pixels start at +0x100.

| Offset | Type | Meaning |
|--------|------|---------|
| 0x00 | char[2] | `"VI"` |
| 0x08 | u32 | Frame id / timestamp; matches parts of one frame together |
| 0x10 | u16 | Format |
| 0x12 | u16 | Bytes per output row |
| 0x14 | u16 | Rows (per view) |
| 0x16 | u16 | Source row pitch |
| 0x18 | u16 | Source rows per output row (views interleaved) |

Format 0: rows of two views are interleaved; Sony de-interleaves them (view A
rows, then view B rows) into a 3-slot ring buffer. Format 3: a continuation
part of the frame in the current slot (same id at +0x08). Sony appends it after
the four-quadrant part and relabels the slot format `0x0d` with params `1, 4`.
This is how the large interleaved modes arrive in several transfers [likely].

#### Calibration stream (`0x8b`, `FUN_180127250`) [sure]

A transfer holds one or more `"VD"` records:

| Offset | Type | Meaning |
|--------|------|---------|
| 0x00 | char[2] | `"VD"` |
| 0x02 | u16 | `0x0200` (version; anything else is rejected) |
| 0x04 | u32 | Record size (next record starts here) |
| 0x08 | u16 | Entry count |
| 0x20 + 12·i | u32 type, u32 size, u32 offset | Entry table; offset is from the record start |

| Type | Size | Content (Sony's dump file name) |
|------|------|-------------------------------|
| 10 | ≤ 0x800 | Camera calibration 0 (`vrt_camera_calib_vrtrace0.bin`); parsed by `FUN_180156500`, also fed to the config manager |
| 11 | ≤ 0x800 | Camera calibration 1 (`vrt_camera_calib_vrtrace1.bin`) |
| 12 | 0x78 | Headset IMU calibration (`imu_calib.bin`) |
| 13 | 0x34 | IMU "set" calibration (`imu_set_calib.bin`) |
| 0x11 | — | Stored at +0x414cc, sent to config (`FUN_18015eb70`) |
| 0x12 | — | Stored at +0x412cc, sent to config (`FUN_18015ea40`) |
| 0x13 | — | Stored at +0x41ccc |
| 0x14 | — | Stored at +0x42d50 |

##### Camera calibration blob (types 10/11, `FUN_180161b70`) [sure unless tagged]

| Offset | Type | Meaning |
|--------|------|---------|
| 0x00 | char[3] | `"SIE"` |
| 0x03 | u8 | Version, ≤ 1 |
| 0x04 | u16 | Blob size; must equal the VD entry size |
| 0x06 | u8 | Kept by Sony; meaning unknown |
| 0x07 | char[8] | Model string. If it contains `':'` followed later by `'d'`, camera records are the 0xe4-byte layout, else 0x94 |
| 0x0f | u8 | N = camera count |
| 0x10 | u8 | M = extrinsic count |
| 0x18 | N camera records | |
| 0x18 + N·rec | M × 0x34 extrinsics | |

Camera record (0xe4 layout): `u8 id` at +0, `u16 width` at +4, `u16 height` at +6,
9 floats at +8 (3×3, likely the intrinsic matrix K), then **20** doubles at +0x2c: OpenCV (k1, k2, p1, p2, k3, k4, k5, k6,
s1..s4, τx, τy) plus 6 extra rational terms [sure] (`controller-tracking.md` §B.1.3). The 0x94 layout is the float-only
variant.

Extrinsic record (0x34 bytes): `u8 from`, `u8 to`, 2 bytes padding, 3×3 float
rotation, float[3] translation. It links camera ↔ camera/IMU frames [likely].

The kernel module can get factory calibration by reading EP `0x8b` after
setting a camera mode.

#### DisplayPort receiver status (Get `0xa5`, printed by `FUN_1801a02b0`)

Offsets in the 96-byte reply. Field names come from Sony's debug-page format
strings [sure]; exact widths beyond the first byte are [likely].

| Offset | Field |
|--------|-------|
| 0x00 | u8 DPStatus (link state, same as the status header's `dprx_status`) |
| 0x04 | float link rate, Gbps/lane; followed by lane count, 4 lane-status bytes, HDCP auth, HDCP encrypt |
| 0x10 | Video input: width, height, fps (float), 3 more values |
| 0x1c | u8 Source |
| 0x20 | Two counters, `Det` and `VBID` dwords |
| 0x30 | PATH, M (dwords) |
| 0x38 | Audio infoframe: u8 channels, u8 bits, u8 code, u32 sample rate (Hz·k) at +0x3c |
| 0x40 | Audio channel status: u8 code, u8 channels, u8 bits, u32 sample rate at +0x44 |
| 0x50 | PLL frequency (u32) + M/N + 2 values |
| 0x59 | Frame-tracking status, distance, two counters |

Useful for diagnosing DSC/link problems on Linux without Sony tools.

#### LED detection (`0x89`) [sure]

The headset itself finds the Sense controller LEDs in its camera images and
streams blob results; the PC never needs raw camera frames for controller
tracking. `FUN_180126b80` loops through these steps:

1. When the config changes, send Set `0x14` (16 bytes) and Set `0x09` (24 bytes).
2. Read EP `0x89` (0x9050 bytes). Accept only `"LD"` at bytes 0..1.
3. Copy the packet under the device lock and bump a u16 sequence counter.
4. Signal the IR tracker. The dword at packet +8 is compared with the device clock for latency stats **[likely]**.

`FUN_18018aaf0` (the IR tracker, about 1100 decompiled lines) consumes it with
`LedSyncer` (log: `phase1:%hs, seq_id:%d, period_id:%d, base_time:%u,
1frame_cycle:%u, led[0..3]:0x%x`), which syncs controller LED blink phase to the
camera exposures.

##### `"LD"` packet layout (from `FUN_18018aaf0`) [sure unless tagged]

| Offset | Type | Meaning |
|--------|------|---------|
| 0x00 | char[2] | `"LD"` |
| 0x02 | u16 | Format version; code branches on `< 0x82` vs `>= 0x82` (newer firmware uses the +0x14 counter for drop detection) |
| 0x08 | u32 | Device timestamp (µs); used to look up the matching IMU/SLAM time (`FUN_180124a80`) |
| 0x14 | u32 | Frame counter (version >= 0x82) |
| 0x20, 0x24 | u32 ×2 | Copied per frame [likely: exposure/sync phase] |
| 0x35 | u8 | Non-zero = no valid detection this frame |
| 0x40 + n·0x2404, n = 0..3 | u32 count + 0x2400 bytes | **Four camera blocks**; each starts with the blob count. 0x40 + 4·0x2404 = 0x9050 |

Each camera block: `u32 count` (clamped to 256) then up to **256 blob
records of 36 bytes**. Header byte 0x18 matters when version > 0x100 (a value
> 0xb3 sets a flag). Blob record (`FUN_180184900`):

| Offset | Type | Meaning |
|--------|------|---------|
| 0x00 | u8 | Kind: 1 or 2 = moment blob; 5 = pre-solved blob (only when version > 0xff) |
| 0x01 | u8 | Upper flag bits (OR-ed with 0x100/0x200/0x400 internally); kind 5 stores byte 0x02 as an id |
| 0x04, 0x06 | s16 | x min, x max (pixels) |
| 0x08, 0x0a | s16 | y min, y max |
| 0x0c | u32 | m00: total intensity (0 = empty blob) |
| 0x10, 0x14 | u32 | m10, m01: first moments relative to (x min, y min) |
| 0x18 | u32 | m11 |
| 0x1c, 0x20 | u32 | m20, m02 |

Centroid: `x = xmin + m10/m00`, `y = ymin + m01/m00`. Covariance:
`cxx = m20/m00 − cx²`, `cxy = m11/m00 − cx·cy`, `cyy = m02/m00 − cy²` (cx, cy
relative). Sony takes the eigenvalues of that 2×2 matrix as the ellipse axes;
blobs one pixel wide or tall skip it. Kind 5 carries m10/m00 and m01/m00
without the bbox offset, and an area ratio `(w·h)/m00`.

##### LED sync (`LedSyncer`, libpad `FUN_1801c0340` / `FUN_1801c1880`) [sure unless tagged]

Each Sense controller keeps an LED-sync state block (libpad pad context
+0x33e0) that libpad serialises into the controller's output report:

| Offset | Type | Field (names from Sony's log format) |
|--------|------|--------------------------------------|
| 0x00 | u8 | phase (mode) |
| 0x01 | u8 | seq_id |
| 0x02 | u8 | period_id |
| 0x04 | u32 | base_time (µs, in controller clock) |
| 0x08 | u32 | 1frame_cycle (camera frame period) |
| 0x0c..0x0f | u8 ×4 | led[0..3] pattern bytes |

The tracker sends commands to it (first byte = command):
`1` set phase/period/led pattern and recompute base_time from the headset↔controller
clock offset (`base = clock − period·cycle/2 − width/2`); `2` set the 4 LED
bytes; `3` scale 1frame_cycle by `(1 + f)` (drift correction); `4` set
base_time; `5` drift plus offset; `6` ignored; `7` sub-commands (table below).

**Output-report placement** (`FUN_1801cbeb0`) [sure]. Body offset N here is
`OutputSettings` offset N−1 in `sense_protocol.h`: Sony's fields land exactly
on Monado's `LedSettings` (`cycle_position` = base_time×3, `cycle_length` =
frame period×3). What was missing is the values Sony puts there:

| Body offset | Value |
|-------------|-------|
| +0x01 | flag bits: `|1` and `|2` select which feature groups are valid |
| +0x02 | more flag bits (`|2`, `|0x10`), firmware ≥ 0x254 only |
| +0x10 | u32 LE, a timestamp from libpad's clock |
| +0x14 | phase (written only when phase ∉ 1..4 or the controller clock fit has a reference time) |
| +0x15 | seq_id |
| +0x16 | period_id |
| +0x17 | u32 LE base_time × 3 (µs → 1/3-µs controller ticks) |
| +0x1b | u32 LE 1frame_cycle × 3 |
| +0x1f..0x22 | led[0..3] when phase is 2 or 3, else `ff ff ff ff` |

On firmware ≥ 0x1006d (and up to 4 tries) libpad probes a capability: Set
feature `0x80` with `80 13 22 00`, then Get feature `0x81`. A reply that starts
with `0x81` selects sync mode 2, any other reply mode 0 [sure]. Below firmware
0x1006d (feature `0x20` +0x1c), or once more than 3 probes have run, libpad
sets mode 2 without probing. Mode 1 is never set in this build, so the steady
state is mode 2 [sure] (`controller-tracking.md` §B.3.7).

Sync commands the tracker sends (`FUN_180194af0`, `FUN_18019b*`), first byte
= command [sure], meaning [likely]:

| Cmd | Payload | Sent when |
|-----|---------|-----------|
| 1 | `[01, phase, period_id, led0..3, u32 value]`, len 11 (len 7 for PRESCAN without a frame-cycle reset, len 2 for phases 5/6). LED bytes are physical LED indices from the map at `0x180260aa0`, row `3·model + hand` (0xff = LED off) | Starting or restarting sync with a given LED set |
| 3 | f32 slope | Frame-period correction whenever the fitted slope is non-zero (threshold DBL_EPSILON) |
| 4 | u32 base time | Phase alignment |
| 5 | f32 slope, s32 offset (9 bytes) | Drift plus offset |
| 6 | 2 bytes (value, count) | Rate-limited to 1 per second; ignored by libpad |
| 7 | 2 bytes (sub, value) | sub 1 = LED off, 2 = full reset, 3 = perf stats |

Commands go only to hands 0 (right) and 1 (left), and only while the
LedSyncer context state is 4 (ready).

Sync value limits and clock model (libpad `FUN_1801c08a0..FUN_1801c09e0`) [sure]:

- `period_id` is clamped to ≤ 60.
- `1frame_cycle` is clamped to **15 683 350 … 17 683 350**, centred on
  16 683 350: one camera frame at 59.94 Hz in **nanoseconds**. Out-of-range
  requests are counted as errors.
- `base_time` is clamped to ±(the state's half-width field at +0x24).
- The host→controller time conversion is linear with drift:
  `ctrl = (drift·offset + (t − t0)·1e9) / (drift + 1e9)`, drift in ppb. t0,
  drift and offset are kept per controller, fitted from the timestamps of the
  controller's input reports (the `[HID_SYNC]` / `motionTSDiff` logic). With
  the fit disabled it is a plain offset.

Sync-failure status enum shown by Sony's UI: 0 NONE, 1 WAITING OFFSET,
2 ALL NOTHING, 3 NOT DETECT 2LED, 4 GRID SKIP, 5 NO HIT, 6 NO SYNC MARKER,
7 NO 3MATCH, 8 NO MAX MIN, 9 NO DIF MAX MIN, 10 BT DISCONNECTED, 11 FRAME
DROP, 12 NOT YET DATA. These suggest the method: blink a sync marker, find the
max/min brightness frames to get the phase, then match 3 LEDs [likely].

Together with the LED models and calibration below, this is enough to build
6DoF controller tracking with no image processing on the PC: only pose
solving and IMU fusion.

#### Sense LED models (`.rdata`, used by `FUN_18018aaf0`) [sure]

Seven models, 0x1400 bytes apart, starting at VA `0x180257280`. Each model:

| Offset | Type | Meaning |
|--------|------|---------|
| 0x00 | i32 ×6 | LED count (20 for model 1, 17 for the rest), 0, 2, 1, 170 or 180, 48; fields 2..5 unknown |
| 0x18 | float[32][3] | LED positions, **millimetres**, controller frame |
| 0x198 | float[32][3] | LED emission normals (unit vectors, checked) |

32 slots correspond to the 32 bits of `led[0..3]`. The tracker picks a model by
a controller hardware/revision id (switch cases 1..4, default) and hand, with
internal ids 3/4, 5/6, 7/8, … and 0xf/0x10 for the default. Models 3..7 share
most LEDs and differ in a few (later hardware revisions) [likely]. A second
table 0x60 bytes before each model (`0x180257220`, …) is passed alongside;
contents not decoded.

[`scripts/sony-re/extract_led_models.py`](../scripts/sony-re/extract_led_models.py)
prints them as JSON from the user's own DLL. The numbers are deliberately not
committed.

### Sense controllers (libpad HID)

The PID families are **[sure]**:

| Predicate | PIDs (VID `0x054c`) |
|-----------|---------------------|
| VR controller | `0x0ce7`, `0x0e45` (Sense L), `0x0e46` (Sense R) |
| DualSense | `0x0ce6`, `0x0df2` (Edge) |
| DS4 | `0x05c4`, `0x09cc` |

#### IMU calibration: feature report `0x05` [sure]

Sony reads it in a loop (`FUN_1801ce8e0`): `GetFeature(0x05, 64 bytes)` until
the high bit of byte 1 is set. Byte 1 & 0x7f is the part index. Part 0 holds
29 little-endian `s16` values from byte 2 (`c[0..28]`). Part 1 holds 13 values
from byte 2 (`c[29..41]`) (`FUN_1801d2a90`). The existing
`sense::CalibrationData` struct places every field Sony uses at the right
offset. Sony's conversion (`FUN_1801d54e0`) runs per axis:

```
gyro bias     bx,by,bz        = c[18], c[19], c[20]
gyro samples  x: c[21] c[24]   y: c[28] c[31]   z: c[35] c[38]
gyro range_a  = |s1 - b| + |s2 - b|
gyro speed    = (c[39] + c[40]) / 2           (deg/s)
gyro scale_a  = speed * pi/180 / (range_a / 2) (rad/s per LSB)
gyro rad/s    = (raw - b) * scale_a

accel plus/minus  x: c[0] c[3]   y: c[7] c[10]   z: c[14] c[17]
accel range_a = |plus| + |minus|
accel offset  = plus - range_a / 2
accel scale_a = 2 / range_a                    (g per LSB)
accel g       = (raw - offset) * scale_a
```

This matches the DualSense math in Linux `hid-playstation`. A calibration with
any zero gyro range is rejected (`0x809200ff`). The struct also stores `1/3`
and `3`, consistent with the IMU tick of 1/3 µs. `SenseHid::ReadCalibration`
currently ignores this data and uses Monado's fixed scales. Switching to it
should give per-unit gyro/accel scales and a factory gyro bias. Validate it on
hardware against the fixed scales first.

Output over Bluetooth is `31 a2 <body[1..]>` + CRC (seed `0xa2`), with no tag
byte [sure] (`controllers.md` §B.2.3).

#### Other controller feature reports [sure]

| ID | Len | Used for |
|----|-----|----------|
| `0x20` | 64 | Firmware/version info (DualSense and VR families). For VR controllers also the hand bit (+0x30 bit 0), the hardware revision (+0x18) and the main firmware version (+0x1c, u32) |
| `0x05` | 64 | IMU calibration (above) |
| `0x12`, `0xa3`, `0x02`, `0xe1`, `0xe3`, `0xe5` | 16, 49, 37, 49, 5, — | DS4 path (MAC/pairing, firmware, DS4 calibration with the same formula, dongle) |

### Reimplementation and equivalence testing

Our implementations live in `userspace/lib/sie/`. Each is written from the
behaviour described in this document, not from Sony's code structure.
[`scripts/sony-re/difftest/run.sh`](../scripts/sony-re/difftest/run.sh) loads
Sony's DLL under Wine, calls the original function at its address with random
inputs, and compares the output bit for bit with ours. The code must be built
with `-ffp-contract=off` (no FMA fusion), because Sony's build does not fuse.

| Function (Sony VA) | Ours | Inputs tested | Result |
|--------------------|------|---------------|--------|
| Sense IMU calibration (`0x1801d54e0`) | `psvr2_sense_cal_compute` | 200 000 random | bit-exact |
| LD blob decode (`0x180184900`) | `psvr2_ld_decode_blob` | 300 000 random | bit-exact (`atanf` included) |

### Code map and coverage

The DLL is linked by module, so address ranges map to subsystems (64 KB bins,
anchored by log strings). 5175 functions; about 4800 have no strings.

| VA range | Subsystem | Status |
|----------|-----------|--------|
| `0x18000xxxx–0x1803ffff` | VR dialogs, localisation strings, boundary meta (`sceBoundaryMeta.bin`) | UI only; skipped |
| `0x180040000–0x18004ffff` | MSVC CRT / STL | Library; skipped |
| `0x180050000–0x18006ffff` | Controller tracker core: `pad_tracker`, `UkfManager`, `LIT::searchOffsetByTimestamp`, Kalman steps | Identified; algorithm not decoded |
| `0x180070000–0x18009ffff` | Dense linear algebra (BLAS/LAPACK-style "Lower/Transpose") | Library; skipped |
| `0x1800a0000–0x1800fffff` | Tracker boot / multi-camera pose search (`ComputeBootStats`, `ConnectedComponents`, `ConfigFile0.trk`) | Identified; algorithm not decoded |
| `0x180100000–0x18011ffff` | VrTrace capture/replay | Debug only |
| `0x180120000–0x18013ffff` | Map save/load, play area, universes, SteamVR glue | Map save scheduling, play-area state and chaperone writing decoded (`room.md` part A); rest partly |
| `0x180140000–0x18015ffff` | jsoncpp + config manager | Library + glue |
| `0x180160000–0x18016ffff` | Geometry math (SVD, Rodrigues), threads/mutexes | Library |
| `0x180170000–0x18018ffff` | Headset INS (IMU integration), IR tracker thread (`FUN_18018aaf0`) | Inputs decoded |
| `0x180190000–0x18019ffff` | LedSyncer, controller tracking glue | Commands decoded |
| `0x1801a0000–0x1801affff` | SteamVR devices (HMD, Sense), properties, debug page | Partly |
| `0x1801b0000–0x1801dffff` | libpad (HID I/O, `HID_SYNC`, calibration, output reports) | Calibration + sync decoded |
| `0x1801e0000–0x18023ffff` | Relocaliser / SLAM backend (keyframes, pose graph, `map_id`) | Identified; not decoded |

The device protocol (every endpoint, every control report, the calibration
blobs, LED models, LED-sync state) is mapped above. What remains is
**algorithms**: the multi-camera LED pose solver, the UKF fusing it with
controller IMU, and the relocaliser. These are better reimplemented with open
methods (P3P/RANSAC + LM refinement over the LED model, an EKF/UKF) than
transcribed. Sony's code shows the structure to aim for: robust (Huber/Tukey)
reprojection cost over all cameras, and a boot search scored over many frames.

### Next targets (priority order)

1. **Hardware check** of the decoded formats: dump EP `0x89` ("LD"), `0x8b`
   ("VD"/"SIE" calibration) and `0x8d` (firmware log) from the kernel module
   after setting a camera mode, and confirm the layouts above.
2. **LED table at `0x180260aa0`** and the 0x60-byte side table before each
   LED model: which LEDs each sync phase lights.
3. **Report `0x81` / `0x8f` / `0xa1` layouts** (firmware/hardware info).
4. **Controller tracker algorithm** (`0x180050000–0x1800fffff`): boot search
   and scoring parameters, UKF tuning. Reimplement; don't transcribe.
5. **Relocaliser** (`0x1801e0000–0x18023ffff`) and the `0x8a` "RP" payload:
   persistent room map.
6. **Report `0x29`** in the SteamVR driver: 120 Hz.

---

## Part C: Function triage

Every function in [`scripts/sony-re/inventory.tsv`](../../scripts/sony-re/inventory.tsv)
(5175 functions, Sony `driver_playstation_vr2_orig.dll`, PE timestamp
1741940190) is given one verdict in
[`scripts/sony-re/triage.tsv`](../../scripts/sony-re/triage.tsv):

| Verdict | Meaning | Functions | Decompiled lines |
|---------|---------|----------:|-----------------:|
| **core** | Behaviour the Linux driver needs: already ported, or still to port. | 2396 | ~204 k |
| **ref** | Worth reading to understand the device or Sony's design, or an optional feature (relocaliser). Nothing to port now. | 1025 | ~96 k |
| **fluff** | Library code, Windows glue, Sony's companion-app UI, VrTrace debug capture, initialisers. Skip. | 1754 | ~83 k |

Columns of `triage.tsv`: `va`, `lines` (Ghidra line count from the
inventory), `subsystem` (inventory label), `verdict`, `area` (what part of
the driver it belongs to), `state` (the per-subsystem status sheet value:
`bit-exact`, `spec-only`, `tolerance`, `skip:*`, `todo`), `docs` (which
`docs/*.md` already name it), `reason` (one line, own words).

### C.1 How the verdicts were made

The inventory's `subsystem` column is by address range, and several ranges
are mislabelled (section 5). So the verdicts come from IDA (9.5, the
`bin2` database) instead:

1. **Features per function** (`scripts/sony-re/triage/q_features.py`):
   referenced strings, imports, callers, callees (including function
   pointers taken), data references and the owning RTTI vtable.
2. **Seeds**, strongest first:
   - manual verdicts from reading the decompile
     (`triage/overrides.tsv`, about 60 entries and ranges);
   - the status sheets: `skip:*` is fluff; done entries
     (`bit-exact`/`spec-only`/`tolerance`) are core, except in
     `trace.tsv`, where only the room/chaperone glue is core and the
     VrTrace format parsers are ref;
   - the RTTI class of the vtable a function sits in
     (`triage/classes.tsv`, e.g. `CaesarUsbThread*`, `LedSyncer`,
     `UkfManager` core; `VrTrace*`, `Json::*`, `Framework::*` fluff;
     `Relocalizer*` ref);
   - library ranges and jsoncpp/STL error strings: fluff;
   - device I/O imports (`WinUsb_*`, `HidD_*`, `ReadFile`/`WriteFile`,
     `DeviceIoControl`): core.
3. **Propagation down the call graph**: a helper takes the best verdict
   of its callers (core > ref > fluff). Library, initialiser and OS-wrapper
   code does not pass its verdict on, since it calls everything.
4. **Leftovers**: a wrapper of core/ref code inherits upwards; a function
   between two neighbours of one verdict in the same object file (MSVC
   lays functions out per `.obj`) takes theirs; vtable-only methods in the
   tracker ranges are core, in the relocaliser ref; tiny stubs and STL
   instantiations are fluff. The last 50 or so were read by hand.

Regenerate (needs the local IDA database and decompile, which stay out of
the repo): run `q_features.py` through `ida_run.py` to get
`features.jsonl`, then `classify.py > triage.tsv`. Paths at the top of
`classify.py` point at the local workspace.

Limits: the verdict is per function, the area label is the first area that
reached it, so a shared helper can carry the "wrong" area. A core verdict
says the code is needed, not that the Linux driver lacks it; section 3 says
what is still missing.

### C.2 By area

| Area | Verdict | Functions | Lines | Done in `userspace/lib/sie` | What it is | Linux driver today |
|------|---------|----------:|------:|----:|-----------|-------------------|
| ctrlboot | core | 800 | 74 k | 42 | Controller boot: multi-camera pose search, RANSAC/P3P, LM pose solve, robust losses, background model, connected components | Ported in large parts (`optical_pose.cpp`, `sie/ctrlboot`): RANSAC boot, P3P, sampler, MSAC, few-LED mode |
| tracking | core | 322 | 32 k | 0 | IR tracker thread stages, UKF manager, TrackerManager, INS feed, occlusion queries | UKF in simulation (`controller_ukf.cpp`); IR-thread stages in section 3.2 not described |
| sense | core | 175 | 15 k | 0 | Controller manager over libpad: open/close, per-frame poll, input, vibration mode | `sense_hid.cpp`, `controller_device.cpp` |
| headset-usb | core | 205 | 9 k | 0 | `CaesarManager` and the per-interface `CaesarUsbThread*` readers: enumeration, alt settings, control reports, init order, restart | Kernel module: init sequence, `0xde` clock map, firmware gate, see-through, quit/restart, per-interface recovery |
| libpad | core | 142 | 14 k | 73 | HID transport, `HID_SYNC`, calibration, output reports, clock fit | `sense_hid.cpp`, `sie/libpad` |
| steamvr | core | 200 | 14 k | 38 | `ServerDriver`, `HmdDevice`, `SenseDevice`, display, lens, hidden area, skeleton, properties | `driver_psvr2` (see `steamvr-and-display.md` part A "Project driver vs Sony") |
| irtracker | core | 104 | 7 k | 41 | Headset INS helpers (quaternion, filters) | `psvr2_ins.c` (strapdown + mode-1 blender) |
| room | core | 105 | 6 k | 14 | Map-state singleton, universe/chaperone file, default play area | Own chaperone file in `hmd_device_driver.cpp` (`provide_chaperone`) |
| io | core | 30 | 5 k | 0 | Low-level HID/USB I/O wrappers (libpad device threads, overlapped reads) | Replaced by hidraw / usbfs |
| ctrltracker | core | 317 | 29 k | 36 | UKF kernels, process noise, quaternion maths; the UKF predict / correct / replay code and its double-precision matrix kernels at `0x180070950`–`0x18009cfff` (first mislabelled as a library, see `function-catalogue.md` part C) | `sie/ctrltracker`, `controller_ukf.cpp` |
| ledsync | core | 4 | 0.1 k | 0 | Few helpers; the LedSyncer itself is under tracking/steamvr | `led_syncer.cpp` |
| reloc | ref | 874 | 84 k | 0 | Relocaliser: keyframes, pose graph, map DB, RP frame unpack | Not done (optional, `development.md` part A); RP data exposed on `/dev/psvr2-rp` |
| config | ref | 35 | 2 k | 0 | `ConfigManager`: `config.ini`, change notifications | Settings in `steamvr.vrsettings` instead |
| ipc | ref | 12 | 1 k | 0 | `ShareManager`: `SHARE_VRT2_*` shared memory with Sony's apps | `toolkit_ipc.cpp` (own protocol) |
| status | ref | 5 | 0.7 k | 0 | Status-info dispatcher (mode, proximity, thermal warning) to UI prompts | Field meanings only |
| firmware | ref | 6 | 0.7 k | 0 | `HMD2_FIRMWARE.CUP` version check beside the app | Not needed |
| trace-format | ref | 9 | 0.8 k | 9 | VrTrace parsers that document wire formats (image header V0200, `ST` frame TOC) | Formats in `vrtrace.md` §4 |
| lib | fluff | 1092 | 43 k | – | CRT, STL, jsoncpp, dense linear algebra, Eigen, OpenCV-derived SVD/Rodrigues, thunks, CRT startup | Use libc / Eigen / OpenCV |
| debug | fluff | 365 | 21 k | 316 | VrTrace capture tool, IR recording writer, debug status text | – |
| trace | fluff | 105 | 9 k | – | Remaining VrTrace helpers | – |
| app | fluff | 74 | 6 k | – | `VrDialogManager` (dialogs, desktop/see-through guides, safety notice, locale), companion-app launch and Authenticode checks | – |
| init | fluff | 81 | 4 k | – | Static initialisers (`0x180001000–0x180008160`) | – |

### C.3 What is still to implement or describe

#### C.3.1 Known gaps, checked against the code (2026-11-21)

Most gaps listed in the older docs are closed (init sequence, `0xde` clock
sync, firmware gate, see-through, quit/restart, EDID switch, hidden area,
driver-from-head, chaperone file, INS, LED sync, UKF). What is still open:

| Item | Sony functions | Verdict | Notes |
|------|----------------|---------|-------|
| Relocaliser + Set `0x0a` | thread `0x1801ac4e0`, start `0x1801abf90`, RP unpack `0x1801ab410`, run/update/optimise `0x1801df010` / `0x1801df630` / `0x1801ded40`, map load `0x1801ac1f0`, backend `0x1801e0000–0x180238880` | ref | Only needed for a play area that survives restarts. 873 functions, the largest optional block. |
| Map persistence (`sceBoundaryMeta.bin`, map id) | `0x18011fd30`, `0x180120b40`, `0x180120c80`, `0x180120f20`, `0x1801214a0` | core / ref | Tied to the relocaliser; the own chaperone file covers the non-persistent case. |
| INS mode-0 adaptive blender, stillness / shock switches, SLAM good-frame count | blender `0x180181400` and its callers (controller-tracking.md §A.3.1) | core | `psvr2_ins.h` says only mode 1 is done; mode-0 gains are unknown. |
| Extreme-environment blob modes 2–4 | ledsync / ctrlboot (`controller-tracking.md` part B) | core | Need tracker settings that are 0 in a normal install. |
| Hardware checks | UKF, two-controller sync, BROAD divider | core | Implemented, not yet confirmed on hardware (`development.md` part A). |
| Background model (static IR lights) | `RnD::Background`, `0x1800b40e0` and stages (`controller-tracking.md` part E) | core | Not in the driver. Sony scores every blob against a 3D map of static lights, but the controller solve never reads the score (controller-tracking.md §E.5) [likely]: optional. |

#### C.3.2 Core functions no doc describes yet

143 core functions of 150+ lines (49 k lines) are not named in any
`docs/*.md` and have no status entry beyond `todo`. Many in ctrlboot are
probably covered by recent ports without being named; the rest are the
places to read next. Full list: filter `triage.tsv` on `verdict=core`,
empty `docs`, `lines>=150`. By area: ctrlboot 79, tracking 20, libpad 13,
sense 10, irtracker 6, steamvr 4, io 4, ctrltracker 3, room 3,
headset-usb 1. The largest:

| Function | Lines | Called from | Guess |
|----------|------:|-------------|-------|
| `0x1801b37b0` | 2525 | IR thread `0x18018aaf0` | IR-thread stage, float min/max heavy: blob or LED-model processing [guess] |
| `0x1801b1280` | 1619 | IR thread | same family [guess] |
| `0x1801910f0` | 950 | IR thread | same family [guess] |
| `0x1801b81c0` | 587 | IR thread | same family [guess] |
| `0x1801ba190` | 637 | libpad `0x1801bf060` | libpad-side tracking helper [guess] |
| `0x1801c58d0` | 723 | libpad `0x1801bf550` | controller report processing [guess] |
| `0x18005c7f0`, `0x18005c130`, `0x1800641d0` | 529, 457, 441 | UKF snapshot code `0x18005e5f0–0x18005e6e0` | UKF snapshot save/restore [likely] |
| `0x1801780e0` | 506 | `0x1801792a0` | INS status / version log ("%dDoF") [likely] |
| `0x18006f020`, `0x180068ee0`, `0x18006dbd0` | 594, 443, 429 | ctrltracker | UKF steps not yet in `controller-tracking.md` part G |
| `0x1800b43d0`, `0x1800b3090`, `0x1800b53c0` | 816–818 | ctrlboot `0x1800b40e0` | **background model** stages (static IR lights), now in `controller-tracking.md` part E |
| `0x1800b8b00`, `0x1800a88f0` | 780–822 | ctrlboot `0x1800a37f0` | `0x1800a88f0`: close-range fusion path of `0x1800a37f0` (`function-catalogue.md` part A) |
| `0x1800ea4c0` | 411 | ctrlboot | connected components (blob labelling) |

Read the four IR-thread stages first: they run per LD frame between the
blob packet and the pose solve, which is the part the Linux optical
tracker most likely still approximates.

### C.4 What is fluff, and why

- **Libraries (1092 in area `lib`).** MSVC CRT/STL, jsoncpp, an f2c CLAPACK
  (`0x180083700–0x18008d3ff`, `function-catalogue.md` part C), Eigen integrators,
  OpenCV-derived SVD / Jacobi / Rodrigues / fisheye projection, import
  thunks, CRT startup and exception funclets. Linux uses its own libraries.
  The camera model those projection helpers implement is already written
  up in `controller-tracking.md` §B.1.
- **VrTrace (about 440).** Sony's capture tool. On PC nothing feeds its
  rings (`vrtrace.md` §1). Only the format notes are kept (ref).
- **Companion-app UI (74).** `VrDialogManager`: dialogs, desktop guide,
  see-through guide, safety notice records, locale matching, launching and
  Authenticode-checking `PlayStationVR2App.exe`, `psvr2_dialog.exe`,
  `psvr2_overlay.exe`. The status-info dispatcher and tracking-status
  handler in this range are kept as ref, because they show what the
  status fields mean.
- **Static initialisers (81).** Global constructors, std::string
  constants, localised installer messages. A few seed tunables (for
  example the `ImuRingBuffer` defaults); those values are in the
  subsystem docs.
- **Debug output.** IR recording file writer (`0x1801adf50–0x1801ae260`),
  the debug status text (`0x180185ef0`).

### C.5 Inventory label corrections (applied to `inventory.tsv` 2026-11-21)

| Range | Inventory says | Actually |
|-------|----------------|----------|
| `0x180001000–0x180008160` | `ui` | static initialisers |
| `0x180043f30–0x1800464ff` | `lib:crt` | RnD vector / pose math (`ctrl-boot`) |
| `0x180046500–0x18004d8ff` | `lib:crt` | controller-tracker frontend: init, config file, per-frame core `0x18004b0a0`, stats, overlay (`tracking`) |
| `0x180100140–0x180109fb0` | `debug:trace` | ctrlboot solver and robust losses |
| `0x18011b3c0–0x18011d6f0` | `debug:trace` | controller manager (`sense`) |
| `0x18011e000–0x18011ffff` | `debug:trace` | map state, chaperone, default play area (`room`) |
| `0x18012d900–0x1801354ff` | `map+steamvr` | `VrDialogManager` (companion-app UI) |
| `0x180154870–0x18015ffa0` | `lib:json+config` | `ConfigManager` / `ShareManager` glue, not jsoncpp |
| `0x1801d6230–0x1801dfb00` | `libpad` | relocaliser kernels and object API |
| `0x180238f00–0x18023ffff` | `reloc` | CRT startup, import thunks, unwind funclets |

The IDA database knows 9166 functions; the 5175 in the inventory are
Ghidra's boundaries. 19 inventory entries have no IDA function at
that address (IDA merged them into a neighbour); they take the verdict of
their range.
