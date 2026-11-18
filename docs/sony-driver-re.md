# Sony PSVR2 SteamVR driver: reverse-engineering notes

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

## Architecture

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
[protocol.md](protocol.md#if5-eye--gaze-tracking)). The DLL holds no gaze model,
gaze calibration or foveation logic to recover.

## Headset USB transport

### Control transfers (`FUN_1801283f0`) [sure]

These are vendor transfers on ep0, carrying the `sie_ctrl_pkt` from
[protocol.md](protocol.md):

| | bmRequestType | bRequest | wValue | wIndex | wLength |
|-|---------------|----------|--------|--------|---------|
| Set | `0x42` | `0x09` | `0` | `0` | `len + 8` |
| Get | `0xc2` | `0x01` | report_id | `0` | `len + 8` |

The payload header is `u8 report_id, u8 0, u16 subcmd, u16 len, u16 0`, then
`len` data bytes. The buffer is 512 bytes. Sony's Set uses `wValue = 0`; the
`report_id` lives only in the payload.

Sony switches the control interface to **alternate setting 1** after reading
the firmware info, then sends `0x22` and `0x12` (`FUN_1801256c0`).

### Bulk IN endpoints (`FUN_180127d60` callers) [sure]

| EP | Read size | Thread (from `[USB] fps imu/img/trk/led/rel` log) | Notes |
|----|-----------|-------------------------------|-------|
| `0x83` | 0x200 | SlamTracking ("trk") | `SLA` pose record, already supported |
| `0x87` | 0x200100 | Image ("img") | Camera frames, already partly supported |
| `0x88` | 0x4000 | ImuStatus ("imu") | Status + IMU, already supported |
| `0x89` | 0x9050 | Leddet ("led") | **LED detection results**: magic `"LD"` at offset 0; see below |
| `0x8a` | 0xc8780 | RelocPre ("rel") | Magic `"RP"`, u32 size at +4 must be 0xc8780 [sure]; relocaliser keyframe/feature data for the PC-side relocaliser [likely] |
| `0x8b` | 0x40000 | ("Log" thread) | **Calibration container `"VD"`**, see below. The same thread re-sends camera mode `0x0b` once a second until camera calib (type 10) and IMU calib (type 12) have arrived |
| `0x8d` | 0x100000 | GenData | **Headset firmware console log**: newline-separated text with ANSI colour codes, which Sony strips before logging it [sure]. Useful for debugging |

`0x85` (gaze) is not read.

### Control reports

`Get` = device to host. Len = payload bytes.

| ID | Dir | Len | subcmd | Meaning | Where |
|----|-----|-----|--------|---------|-------|
| `0x81` | Get | 0x54 | 1 | Firmware info. Init fails ("Failed to read PS VR2 firmware info") if the dword at +0x34 is 0; retried once after 1 s **[sure]** | `FUN_180124450` |
| `0xa1` | Get | 4 | 1 | Dword cached at init **[sure]**, meaning unknown | `FUN_1801256c0` |
| `0x8f` | Get | 0x100 | 1 | 256-byte block handed to the config manager; read only if the version at +2 of an earlier report is ≥ 0x300 **[sure]**; hardware info/calibration **[guess]** | `FUN_1801256c0` |
| `0xde` | Get | 0x18 | 1 | **Clock sync**: device timestamp (µs) at +4. Host stamps before/after; samples with RTT < 1000 µs are kept; offset = host_before + RTT/2 − device_ts, averaged over a 100-entry ring; reset on a jump > 2000 µs **[sure]** | `FUN_180123fa0` |
| `0x97` | Get | 1 | 1 | See-through state, polled every second (see the status stream) **[likely]** | `FUN_1801268d0` |
| `0xa4` | Get | 4 | 1 | Controller-tracking related dword **[likely]** | `FUN_180124ee0`, caller `FUN_18011bae0` |
| `0xa6` | Get | 0x20 | 1 | 8 dwords, controller-tracking related **[likely]** | `FUN_180124c00` |
| `0xa5` | Get | 0x60 | 1 | **DisplayPort receiver status**, read after Set `0x25` (latch) **[sure]**; layout below | `FUN_1801248d0` |
| `0xa9` | Get | 1 | 1 | **Current EDID type**: 0 = 90/120 Hz, 1 = 90 Hz only **[sure]** | `FUN_180124970` |
| `0x8a` | Get | 8 | 1 | Dword cached; not called directly (vtable) | `FUN_180125a50` |
| `0x12` | Set | 1 | 1 | Panel brightness; Sony's init default is `0x1f` **[sure]** | `FUN_180123f60` |
| `0x17` | Set | 2 | 1 | `{on, 3}`: see-through toggle (see the status stream) **[likely]**; init sends 0 | `FUN_180124580` |
| `0x22` | Set | 1 | 1 | Init, a byte from device state **[sure]** | `FUN_1801256c0` |
| `0x25` | Set | 0 | 1 | Request that precedes Get `0xa5` **[sure]** | `FUN_1801248d0` |
| `0x29` | Set | 1 | 1 | **Set EDID type**: 0 = 90/120 Hz, 1 = 90 Hz only. Sony picks it from SteamVR's `preferredRefreshRate` **[sure]** | `FUN_180127e80`, `FUN_1801a4390` |
| `0x27` | Set | 2 | 1 | u16 `1000`, sent from `RunFrame`'s direct-mode check; a timeout in ms **[guess]** | `FUN_180125610` |
| `0x05` | Set | 0 | n+1 | `subcmd = flag + 1`, from `RunFrame` and `FUN_1801a3c00` (display/session state) **[likely]** | `FUN_180127fa0` |
| `0x28` | Set | 0 | 1 | From `FUN_1801a3c00` **[sure]**, meaning unknown | `FUN_180127e40` |
| `0x0b` | Set | 8 | 1 | Camera mode `{1, mode}`, re-sent when the config changes and at most once per second **[sure]** | `FUN_180125cc0` |
| `0x14` | Set | 0x10 | 1 | **LED-detection config**: three dwords `0x55`, `500`, `0x24800000 \| 1` (Sony's defaults) **[sure]**; field meaning **[guess]**: threshold, exposure, flags | `FUN_180126b80` |
| `0x09` | Set | 0x18 | 1 | Second LED-detection/sync config block (24 bytes) from the config manager **[sure]** | `FUN_180126b80` |
| `0x1f` | Set | 0x100 | 1 | 256-byte block built from constants (`0x28`, `10`, `5`, 16-byte tables) **[sure]**, meaning unknown | `FUN_180125ab0` |
| `0x0a` | Set | 0xd8 | n | Relocaliser pre-output parameters ("setRelocPreOutput") **[likely]** | `FUN_180127fe0`, `FUN_1801adba0` |

Not seen from the host: `0x08` (haptics in protocol.md) and `0x0c` (gaze);
both are toolkit/Monado territory.

### SLAM record (`0x83`, `FUN_1801270c0` → `FUN_18017a5d0`)

Sony checks `"SL"` (bytes 0..1) and that the high byte of the **u16 version
at +2** is `0x01`. protocol.md's `"SLA"` + `const1 = 0x01` is this version
read as text: firmware sends `0x0141`. Fields by version (offsets in the
512-byte record; [sure] for the version gates, [likely] for the meanings):

| Offset | Type | Use in Sony's driver |
|--------|------|----------------------|
| 0x08 | u32 | Device timestamp (µs) |
| 0x0c | u8 | Tracking state; 3 = special handling (lost/relocalising [likely]) |
| 0x0d | u8 | Sub-state; 5 = special handling |
| 0x0e | u8 | Flag; 1 = special handling |
| 0x0f | u8 | Present from version 0x120 |
| 0x10 | float[3] + float[4] | Head pose (position m, quaternion w-first), as in protocol.md |
| 0x2c | float[3] + float[4] | Second pose; combined with 0x144 for the play-area frame [likely] |
| 0x48 | float[3] | Vector, likely linear velocity |
| 0x54 | float[6] | Two vectors, likely angular velocity and acceleration |
| 0x144 | float[3] + float[4] | Map/anchor transform applied to the poses above [likely] |
| 0x160..0x168 | float ×3 | Compared by magnitude, version ≥ 0x150 |
| 0x184, 0x188 | u32 | Copied per frame |
| 0x1c8 | i32 | Map slot 0..2, version ≥ 0x120 (out of range means 0) |
| 0x1e0 | u8 | Version ≥ 0x131 |
| 0x1e8 | float | Defaults to 0.08 when the version is older than 0x134 |
| 0x1ec | u16, 0x1ee u8, 0x1ef u8 | Version ≥ 0x134 / 0x135 / 0x141 |

The driver latches the timestamp for latency stats like the LD stream.

### Status stream (`0x88`, `FUN_1801268d0`)

Sony copies the first 32 bytes (the status header) every transfer and then
parses the IMU records (`FUN_180127620`). Additions to protocol.md:

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
- Version gate: below version 0x300 (Get from `FUN_180124510`) none of this
  runs [sure].

### Image stream (`0x87`, `FUN_180125e00`) [sure unless tagged]

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

### Calibration stream (`0x8b`, `FUN_180127250`) [sure]

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

#### Camera calibration blob (types 10/11, `FUN_180161b70`) [sure unless tagged]

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
9 floats at +8 (3×3, likely the intrinsic matrix K), then 19 doubles at +0x2c
(distortion model coefficients [likely]). The 0x94 layout is the float-only
variant.

Extrinsic record (0x34 bytes): `u8 from`, `u8 to`, 2 bytes padding, 3×3 float
rotation, float[3] translation. It links camera ↔ camera/IMU frames [likely].

The kernel module can get factory calibration by reading EP `0x8b` after
setting a camera mode.

### DisplayPort receiver status (Get `0xa5`, printed by `FUN_1801a02b0`)

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

### LED detection (`0x89`) [sure]

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

#### `"LD"` packet layout (from `FUN_18018aaf0`) [sure unless tagged]

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

#### LED sync (`LedSyncer`, libpad `FUN_1801c0340` / `FUN_1801c1880`) [sure unless tagged]

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
base_time; `5`… (more cases, not decoded yet).

**Output-report placement** (`FUN_1801cbeb0`) [sure]. Body offset N here is
`OutputSettings` offset N−1 in `sense_protocol.h`: Sony's fields land exactly
on Monado's `LedSettings` (`cycle_position` = base_time×3, `cycle_length` =
frame period×3). What was missing is the values Sony puts there:

| Body offset | Value |
|-------------|-------|
| +0x01 | flag bits: `|1` and `|2` select which feature groups are valid |
| +0x02 | more flag bits (`|2`, `|0x10`), firmware ≥ 0x254 only |
| +0x10 | u32 LE, a timestamp from libpad's clock |
| +0x14 | phase (written only when phase ∉ 1..4 or a sync request is pending) |
| +0x15 | seq_id |
| +0x16 | period_id |
| +0x17 | u32 LE base_time × 3 (µs → 1/3-µs controller ticks) |
| +0x1b | u32 LE 1frame_cycle × 3 |
| +0x1f..0x22 | led[0..3] when phase is 2 or 3, else `ff ff ff ff` |

On firmware ≥ 0x1006d (and up to 4 tries) libpad probes a capability: Set
feature `0x80` with `80 13 22 00`, then Get feature `0x81`. A reply that starts
with `0x81` selects sync mode 2 [sure], meaning unknown.

Sync commands the tracker sends (`FUN_180194af0`, `FUN_18019b*`), first byte
= command [sure], meaning [likely]:

| Cmd | Payload | Sent when |
|-----|---------|-----------|
| 1 | phase, then 4 LED bytes looked up per (model, hand, slot) in a table at `0x180260aa0` (0xff = LED off) | Starting or restarting sync with a given LED set |
| 3 | float drift | Frame-period correction when the measured drift exceeds a threshold |
| 4 | u32 base time | Phase alignment |
| 5 | 1 byte | Sync-loss handling |
| 6 | 2 bytes (value, count) | Rate-limited to 1 per second |
| 7 | 2 bytes (mode 1..3, value) | Mode change |

Commands go only to tracked controllers 0 and 1, and only while the tracker
state is 4.

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

### Sense LED models (`.rdata`, used by `FUN_18018aaf0`) [sure]

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

## Sense controllers (libpad HID)

The PID families are **[sure]**:

| Predicate | PIDs (VID `0x054c`) |
|-----------|---------------------|
| VR controller | `0x0ce7`, `0x0e45` (Sense L), `0x0e46` (Sense R) |
| DualSense | `0x0ce6`, `0x0df2` (Edge) |
| DS4 | `0x05c4`, `0x09cc` |

### IMU calibration: feature report `0x05` [sure]

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

### Other controller feature reports [sure]

| ID | Len | Used for |
|----|-----|----------|
| `0x20` | 64 | Firmware/version info (DualSense and VR families) |
| `0x05` | 64 | IMU calibration (above) |
| `0x12`, `0xa3`, `0x02`, `0xe1`, `0xe3`, `0xe5` | 16, 49, 37, 49, 5, — | DS4 path (MAC/pairing, firmware, DS4 calibration with the same formula, dongle) |

## Reimplementation and equivalence testing

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

## Code map and coverage

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
| `0x180120000–0x18013ffff` | Map save/load, play area, universes, SteamVR glue | Partly |
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

## Next targets (priority order)

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
