# Tracking: headset INS and Sense controller tracking

Sony `driver_playstation_vr2_orig.dll`, PE timestamp 1741940190. Algorithm
level description for an open reimplementation. Earlier bit-exact notes:
[irtracker.md](irtracker.md) (INS helpers), [ctrltracker.md](ctrltracker.md)
(UKF kernels), [ctrlboot.md](ctrlboot.md) (robustifiers, RANSAC). This file
does not repeat their kernel-level detail.

Tags: **[sure]** read in code, **[likely]** strong inference, **[guess]**.

## 1. Driver summary

Everything here is **userspace**. The kernel module only has to deliver the
raw bulk transfers of EP `0x83` (SLAM), `0x88` (status + IMU), `0x89` (LED
blobs) with their device timestamps intact.

Priority order for the Linux stack:

1. **Headset pose = on-device SLAM, IMU-propagated.** The headset already
   computes 6DoF (EP `0x83`). Sony's host "INS" does not run a full EKF over
   SLAM: it keeps two strapdown integrators over the 2 kHz IMU, one *reset to
   every valid SLAM sample* ("VIO" filter) and one *output* filter that is
   pulled toward the VIO filter by an adaptive complementary blender. A Linux
   driver gets most of the value from: take the latest SLAM pose, velocity and
   biases, integrate IMU samples newer than the SLAM timestamp, then predict
   to display time. §5.1.
2. **6DoF / 3DoF state machine** driven by the SLAM quality value at record
   `+0x184` and the SLAM reset counter at `+0x1e0`. §4.1.
3. **Controller tracking** from LED blobs (EP `0x89`): model-based pose solve
   plus UKF with controller IMU. §5.3–5.6.

## 2. Wire and data formats

### 2.1 SLAM record (EP `0x83`), as consumed by the INS [sure unless tagged]

Ingest is `FUN_18017a5d0` (called from `FUN_18017b100` ← `FUN_1801270c0`).
The record is copied into one of **three slots** chosen by the map slot at
`+0x1c8` (version ≥ `0x120`; out of 0..2 → 0). The INS then reads it as the
following sample (offsets are the 512-byte record):

| Offset | Type | Meaning (INS view) |
|--------|------|--------------------|
| 0x02 | u16 | Version; high byte must be `0x01` |
| 0x08 | u32 | Device timestamp, µs |
| 0x0c | u8 | Tracking state. **3 = SLAM tracking (6DoF valid)** |
| 0x0d | u8 | Sub-state. 5 = valid as well; then `+0x188` is used |
| 0x0e | u8 | Flag; 1 = valid as well |
| 0x0f | u8 | Counted per slot when non-zero (ver ≥ 0x120, else forced 0) |
| 0x10 | float[3] | `wTi` position, m (world ← IMU) |
| 0x1c | float[4] | `wTi` rotation, **x, y, z, w** (identity reset writes 1.0 at index 3; see corrections) |
| 0x2c | float[3] + float[4] | Second pose (same layout); stored as the filter's previous / reference pose [likely] |
| 0x48 | float[3] | `wVi` velocity, m/s |
| 0x54 | float[3] | **Gyro bias**, rad/s (reset if any NaN or norm > 1.0) |
| 0x60 | float[3] | **Accel bias**, m/s² (reset if any NaN or norm > 1.7) |
| 0x144 | float[3] + float[4] | Anchor (map) pose. Versions < `0x110` only: both poses are re-expressed as `anchor ∘ pose` on ingest; newer firmware already sends world poses |
| 0x160..0x168 | float ×3 | Per-axis running max of |value| (ver ≥ 0x150), else −10000 [stats] |
| 0x184 | float | **SLAM error variance**; `sqrt` feeds the quality monitor (§4.1) |
| 0x188 | float | Used when sub-state = 5; else −1 is substituted |
| 0x1c8 | i32 | Map slot 0..2 |
| 0x1e0 | u8 | **SLAM reset counter**; a change = "Detect SLAM Reset" (ver ≥ 0x131) |
| 0x1e1 | u8 | Map valid (ver ≥ 0x133) |
| 0x1e4 | u32 | **Map id**; upper 16 bits change = "Map Latch" (ver ≥ 0x132; −1 = none) |
| 0x1e8 | float | Default 0.08 for ver < 0x134 |
| 0x1ee | u8 | Copied to INS `+0x44` |

Ingest also (`FUN_18017a5d0`):

- The **first** pose after init is latched as origin; afterwards only the
  *position* is reported relative to it (`pos -= first_pos`), rotation is not
  touched [sure].
- "valid" = `state == 3 || flag0e == 1 || substate == 5` [sure].
- Map latch: when `(map_id & 0xffff0000)` changes, the INS "delta pose
  latched" flag (`INS+0x1a0`) is cleared so the next valid sample re-latches
  the SLAM→INS offset; logs `Map Latch` [sure].
- Map valid and INS in 6DoF → one-shot `ResetLocalCoordinate` [sure].
- Records more than **24 000 µs** apart are counted as SLAM drops [sure].

### 2.2 Headset IMU samples (EP `0x88` body) [sure]

`FUN_180173fe0` → `FUN_180174410`. 24-byte records:

| Offset | Type | Meaning |
|--------|------|---------|
| 0x00 | s16[3] | Accelerometer raw |
| 0x06 | s16[3] | Gyro raw |
| 0x0c | u32 | Device timestamp, µs |
| 0x14 | u16 | Short timestamp, µs; consecutive samples differ by **500** |

Calibrated: `a = A·raw_a + b_a`, `w = G·raw_w + b_w` (3×3 + bias, built from
the IMU calibration records via `FUN_180124700` / `FUN_180124780`, rebuilt
when the calibration generation changes). Stored timestamp is
`ts − 1250 µs` (fixed group delay). Ring of 300 samples. Drop detection:
`Δts16 / 500 ≠ 1` → log and suppress rate estimation for 300 samples. Rate
estimate = mean Δt over the ring, accepted only in (400, 600) µs. Stillness
averager is configured with rate **2000 Hz** (`FUN_180175290`) → IMU is
**2 kHz**.

The INS internal sample is 7 floats: accel[3], gyro[3], time (µs) [sure].

## 3. Sequences

### 3.1 SLAM sample arrival (`FUN_18017cdb0`) [sure]

1. If the INS has no time base yet: take the newest IMU time; give up if it
   is more than 20 ms from the SLAM time; else set `t_ins = t_imu`.
2. **Mode** (`INS+0x180`, 0 normal / 1 force 6DoF / 2 IMU only). Mode 2 skips
   to step 6.
3. Update the SLAM quality monitor (§4.1) with `+0x184`, `+0x188`, valid.
4. Fetch IMU samples in `(t_slam, t_ins]`. If the sample is valid (or mode
   1): **reset the VIO filter state to the SLAM state** (pose, velocity, gyro
   and accel bias, anchor; `FUN_18017b850` → `FUN_180180200` with NaN/norm
   checks) and integrate those IMU samples forward to `t_ins`
   (`FUN_18017f8e0`). First time (or after a map latch): latch the offset
   between output and SLAM frames, reset both blenders, log "INS latched Slam
   delta pose". Otherwise, if any accel axis exceeded **35.5 m/s²** in those
   samples, record a shock time.
5. If valid and (quality good or map slot > 0): enter 6DoF.
6. With no SLAM sample (IMU tick path, `INS+0x170 ≥ 1`): fetch IMU samples
   in `[t_ins, t_ins + 128 ms]`; feed gyro into the stillness averager; in
   3DoF copy its gyro-bias result into the output filter; propagate both
   filters (`FUN_18017b5a0`); update the stationary counter
   (`still_us += n·500` while `|gyro|,|accel|` are within the parameter block
   limits, else 0; stationary = > 1.5 s); tilt (gravity) correction
   `FUN_180171400` using the averaged accel when not in 6DoF; then in 6DoF
   blend output toward VIO (`FUN_18017ca00`). If the output pose fails its
   sanity check (NaN, `|q|² ∉ [0.9, 1.1]`): log "filtered pose is corrupted"
   and copy the VIO filter into it. With no IMU for > 100 ms, `t_ins = now`
   and drop to 3DoF.
7. 6DoF exit: mode 0 and no valid SLAM for **500 ms**; SLAM reset counter
   changed; or quality bad (§4.1). Mode 1 forces 6DoF.

## 4. State machines

### 4.1 6DoF / 3DoF (INS `+0x178`) and the SLAM quality monitor

Monitor (`FUN_180181260`, object at `INS+0x568`) per SLAM record, if the
previous record is ≤ 100 ms old:

```
holdoff_us -= dt_us                       (if > 0)
if (rec188 > 0) good_frames++
if (valid && var > 0):
    s = min(sqrt(var), 10.0)
    n_samples++
    err = err + (s - err) * 0.04          // EMA
    if (err > 5.0 && holdoff_us <= 0) { bad_events++; holdoff_us = 3000 }
    if (s   > 2.5) n_samples  = 0
    if (err > 2.5) good_frames = 0
good() = n_samples > 30 && holdoff_us < 1 && err < 2.0 && good_frames > 1
bad()  = err > 2.5
```

| From | Event | To |
|------|-------|----|
| 3DoF | valid SLAM && (`good()` or map slot > 0) | 6DoF (count a 6DoF entry) |
| any | mode 1 | 6DoF |
| 6DoF | mode 0 and 500 ms since last valid SLAM | 3DoF |
| 6DoF | SLAM reset counter (`+0x1e0`) changed | 3DoF |
| 6DoF | `bad()` | 3DoF |
| any | no IMU 100 ms | 3DoF |

In 3DoF the output filter's position is held (taken from the latched
offset), rotation comes from IMU + gravity tilt correction.

## 5. Algorithms

### 5.1 INS strapdown propagation (`FUN_18017f8e0`) [sure]

State per filter (struct of pointers): pose `wTi` (p, q xyzw), velocity v,
gyro bias b_g, accel bias b_a, gravity g = **(0, −9.8, 0)** (world **Y up**,
`FUN_180175290`), previous accel and gyro sample.

```
T = dt * n                                   // dt = sample period (s)
dq = identity; Sa = 0; SSa = 0
for i in 0..n-1:
    w = gyro[i] - b_g
    // rotation increment from (w_prev, w): midpoint rotation vector plus
    // coning term (w_prev x w) * dt^2 / 24; small-angle series below
    // |theta|^2 < 2.29e-5 (FUN_18017f2c0)
    dq = dq ⊗ dq_i
    a = R(dq) * (accel[i] - b_a)             // body→start frame
    Sa  += a
    SSa += a * (n - i)                       // for the double integral
q' = normalize(q ⊗ dq)                       // FUN_1801805a0
v' = v + R(q)*Sa*dt + g*T
p' = p + v*T + R(q)*SSa*dt^2 + 0.5*g*T^2
```

### 5.2 Output blender (`FUN_180181700`, two instances at INS `+0x3b0`, `+0x470`)

Pulls the output filter toward the VIO filter. Mode (`blender[0]`, set by
`FUN_180175690`, mod 3):

- **2**: snap to target.
- **1** (`FUN_180181a50`): if every position axis differs by ≤ `pos_gate`
  and the rotation differs by ≤ `rot_gate`, blend with `α = min(4·dt, 1)`
  (lerp + slerp); else snap.
- **0** (`FUN_180181dd0`, adaptive): error `e = log(out⁻¹·target)` as 6
  values (position, rotation vector). Smooth `ē += (e − ē)·0.05` (reset
  value; instance `+0x470` uses 0.01). Per axis gain from
  `max(|ē| − deadband, 0)` plus velocity-dependent terms (×5×4), floored at a
  per-axis minimum gain, then gain EMA (factor 1.0 by default). Any `|e_i|`
  above its snap threshold → snap. Correction `out ← out ⊗ exp(gain ⊙ e·dt)`.
  When **stationary** use mode 1 blending; after a **shock** (< 3 s) use mode
  1 with α scaled ×4.

Per-axis parameters (6 axes: position x, y, z, then rotation x, y, z) are
set when the blender is bound to its storage (`FUN_180181400`) [sure]:

| Field | Default | Role in mode 0 |
|-------|---------|----------------|
| `+0x70..+0x78` | 0.0005 | minimum gain, position axes |
| `+0x7c..+0x84` | 1e-6 | minimum gain, rotation axes |
| `+0x88..+0x9c` | 0 | deadband (all axes; `FUN_180182960` sets them again on `+0x470`) |
| `+0xa0..+0xa8` | 0.5 m | snap threshold, position (also mode 1's position gate) |
| `+0xac..+0xb4` | 0.1 rad | snap threshold, rotation (also mode 1's rotation gate) |

**Which mode runs on PC** [sure]: binding sets the mode word to 0, and the
only mode setter (`FUN_180175690`, through `FUN_18017af40`) has no caller
in the DLL. So the output blender at `+0x3b0` runs **mode 0** (adaptive);
`+0x470` is switched to mode 1 by `FUN_18017c680`.

Mode 0 per call, with `dt` and the angular rate ω passed by the INS step
[sure for the formulas; the exact frames of the two error logs are
likely]:

```
A      = (0, 0, 0, |ω_x|·dt, |ω_y|·dt, |ω_z|·dt)      // per-axis motion
ē      = ē + (e_raw − ē)·0.05                           // error EMA (+0x68; 0.01 on +0x470)
e      = log(error pose built from ē), 6 values
snap   = any |e_i| > snap_i
r_i    = max(0, |e_i| − deadband_i)
target gains:
  pos x: max(min_0, A_0) + 20·A_4 + 20·A_5
  pos y: max(min_1, A_1) + 20·A_5 + 20·A_3
  pos z: max(min_2, A_2) + 20·A_3 + 20·A_4
  rot i: max(min_i, 4·A_i)
g_i    = g_i + (target_i − g_i)·k                        // k = +0x60 = 1.0 → g = target
c_i    = sign(e'_i) · r_i · g_i                          // e' a second error log, same pose pair
if snap or any c_i is NaN: output = target, c = 0
else: output = output ⊗ exp(c), quaternion renormalised
```

So corrections happen only while the head turns: position is pulled in at
20 × the rotation swept about the two other axes per frame, rotation at
4 × the rotation swept about that axis, plus a tiny floor (0.05 % of the
error per call for position). A still head keeps its small SLAM offsets
until it moves, which hides the corrections. Stationary and shock cases
fall back to the mode-1 blend (shock: α from 4·dt).

Biases and anchor are copied VIO → output after each blend.

Blender reset (`FUN_1801828f0`): `+0x58` = 0.1, gain EMA `+0x60` = 1.0,
error EMA `+0x68` = 0.05; the 7-float last-target buffer is set to NaN.

### 5.3 Controller tracking pipeline (overview)

Entry: IR tracker thread `FUN_18018aaf0`, once per EP `0x89` "LD" packet.

| Stage | Function | Input | Output |
|-------|----------|-------|--------|
| 1. Blob decode | `FUN_180184900` | LD packet, 4 cameras × ≤ 256 blobs | centroid, 2×2 covariance, kind (main doc) |
| 2. LED sync | `FUN_180194af0`, libpad `LedSyncer` | blob counts per frame | LED blink phase, which LED subset is lit (`led[0..3]` masks) |
| 3. Model select | inline in `FUN_18018aaf0` (switch on hardware rev 1..6, default) | controller rev, hand | LED model table + IMU-pose table, `ImuPoseIndex` = 2·rev + 1 + hand (3..14), default 15/16 |
| 4. IMU pose | `FUN_18005f1b0` (`UkfManager::setImuPose`) | ImuPoseIndex | IMU→controller 4×4 per device (§5.6) |
| 5. Per-frame core | `FUN_18004b0a0` | blobs, head pose at LD timestamp, camera extrinsics (`FUN_1800cbd70`, table at `+0x10a40`) | per-device candidate poses (boot or tracking) |
| 6. Boot / recovery | `FUN_1800a56b0` → `FUN_1800d4b90` → `FUN_1800d5070` | blobs + model | pose hypotheses (RANSAC over 6-point minimal sets, P3P-style closed form, `ctrlboot.md`) |
| 7. Refinement | `RnD::PoseCostMultiCam` + Huber/Tukey (`ctrlboot.md`) | hypothesis, LED↔blob matches in all cameras | refined 6DoF pose + inlier count |
| 8. Fusion | `FUN_180067b50` → `FUN_180065b70` ("kalmanFilterStep") | controller IMU (from libpad HID path `FUN_18011bae0` → `FUN_18005e6e0`) and optical poses (`FUN_18005ec00`) | filtered pose, velocity, biases |
| 9. Stats | `FUN_18004cac0`, `FUN_1800c7f00` (`ComputeBootStats`), `FUN_1800a2f70` | — | logging only (skip) |

Frames: LED models are **mm in controller frame**; camera calibration
translation is converted m → mm with the z axis flipped
(`FUN_1800b0130`, ctrlboot.md). So the optical solver works in **mm,
camera-style frame**; the UKF works in **m, Y-up world** (gravity
9.80665 m/s², accel samples in g, `FUN_18006e5a0`) [likely].

Tracking quality states shown by the debug overlay (`FUN_1800d5fc0`):
`No state`, `Normal`, `Obscured`, `Close`, `Very Close`; per-frame method
enum `kNoSpecialState`, `kCrossBlobPnPR`, `kMultiCamNLR`,
`kMultiCamP2PR`, `kP1PFromFailedUndistort`; modes `GLOBAL CLOSE TRACKING`,
`CLOSE BOOT` [sure: strings]. Distance class (`FUN_1800db5f0`): `|p| < 100`
→ Very Close, `< 150` → Close, else normal (units mm from the camera; the
overlay prints cm) [likely]. LED visibility test (`FUN_1800e1c90`): an LED
is usable only if its rotated normal faces the camera within **20°** of the
cone limit (`cos(0.349)`) [sure].

Method meaning [guess from names]: *CrossBlobPnPR* = PnP across blobs of one
camera then refine; *MultiCamNLR* = nonlinear multi-camera refinement from
the predicted pose (normal tracking); *MultiCamP2PR* = 2-point from
prediction (attitude from IMU, solve position) then refine;
*P1PFromFailedUndistort* = 1-point position fix when undistortion fails.

### 5.4 Robust cost (summary)

Residual per matched LED = reprojection error (pixels) in each camera,
weighted by IRLS weights from the robustifier (`ctrlboot.md` §Robust loss).
Huber for refinement, Tukey/Clip for inlier counting [likely]. RANSAC bound
`n = log(1−p)/log(1−w^6)`, inlier ratio capped at 0.9 [sure, `FUN_1800c8b50`].

### 5.5 UKF (`UkfManager`, `FUN_180065b70`) [sure unless tagged]

- **Error state 15**: five 3-blocks at 0, 3, 6, 9, 12 (packed upper
  triangle, `ctrltracker.md`): position, velocity, attitude, gyro bias, accel
  bias [likely order; blocks 0, 9, 12 are reset together in `FUN_18006e5a0`].
- **Initial covariance** on (re)initialisation: position `0.0025` (σ 5 cm),
  velocity `0.04` (σ 0.2 m/s), attitude `(3.05e-4, 7.62e-3, 3.05e-4)` =
  (1°)², (5°)², (1°)² — **yaw (about Y) is the loose axis**.
- **Process noise**: `FUN_180062880` / `FUN_180062950`
  (`ctrltracker.md`), angular-rate dependent; config at UKF `+0x6a0`.
- **History**: states in a ring (0x1c8 bytes each, time µs at `+0x108`),
  measurements in a ring (0x1e0 each). An optical measurement older than the
  newest state is applied at its own timestamp and the IMU is **replayed**
  forward (up to 31 states, `FUN_18007add0`) — delayed-measurement handling.
  `dt = Δt_us · 1e-6`.
- **Stationary pseudo-measurement** (`FUN_18006e5a0`): if the mean of two
  rotated accel samples minus gravity is under **1.96 m/s²** (`|Δa|² <
  3.8468`), velocity is damped ×**0.99** per step (flag-gated).
- **Divergence guard** (`FUN_180067b50`): if `|v|` > limit or any gyro/accel
  bias component exceeds its limit, zero velocity, restore default biases,
  reset those covariance blocks. `FUN_180065130` detects a bad state → full
  reset (`FUN_180063f50`), log "`[UKF] %p: BadState`".
- After an update the next optical deadline is jittered:
  `next = t + uniform(0, 0.01)·1e6` µs (`FUN_180062bc0`) [sure; purpose
  unknown].

### 5.6 IMU → controller transform (`FUN_18005f1b0`) [sure]

Per `ImuPoseIndex` a constant 4×4 (column-major, metres) is **inverted** and
stored per device (`+0x808`, device stride `0x2218`). Index 0 = identity.
Example (index 1): rotation columns ≈ (−0.276, −0.197, 0.941),
(0.736, 0.587, 0.338), (−0.619, 0.785, −0.017), translation
(14.5, 12.9, −8.4) mm. Indices ≥ 3 can take a per-device matrix argument
(calibration) instead of the table and also set limits ±33.16 and 250.0 at
`+0x6c8`/`+0x6cc`/`+0x7d0` [meaning unknown]. Read the tables with the
section reader; do not commit them.

## 6. Constants

| Name | Value | Meaning | VA |
|------|-------|---------|----|
| SLAM gap drop | 24 000 µs | count as SLAM drop | `FUN_18017a5d0` |
| INS time-base window | 20 000 µs | SLAM vs IMU time accepted for init | `FUN_18017cdb0` |
| IMU fetch window | 128 000 µs (`0x1f400`) | IMU span per tick | `FUN_18017cdb0` |
| IMU stall | 100 000 µs | reset time base, drop 6DoF | `FUN_18017cdb0` |
| 6DoF timeout | 500 000 µs | no valid SLAM → 3DoF | `FUN_18017cdb0` |
| Shock threshold | 35.5 m/s² per axis | record shock time | `0x180253f9c` |
| Shock window | 3 s | blender uses fast mode | `FUN_18017cdb0` |
| Stationary time | 1.5 s (`n·500` µs > 1 500 000) | stationary flag | `FUN_18017cdb0` |
| IMU period | 500 µs (2 kHz), rate ok in (400, 600) | | `FUN_180174410` |
| IMU group delay | 1250 µs subtracted | | `FUN_180174410` |
| Gravity (INS) | (0, −9.8, 0) | world Y up | `FUN_180175290` |
| Gyro-bias sanity | norm > 1.0 → 0 | | `FUN_180180200` |
| Accel-bias sanity | norm > 1.7 → 0 | | `FUN_180180200` |
| Quaternion sanity | ‖q‖² ∈ [0.9, 1.1] | | `FUN_18017bac0` |
| Stillness averager | var 3.0e-4, rate 2000 | gyro bias when still | `FUN_180175290` |
| Quality EMA | 0.04, clamp 10, bad 5.0, exit 2.5, enter 2.0, n > 30 | §4.1 | `FUN_180181260` |
| Blender mode-1 gain | α = min(4·dt, 1) | | `FUN_180181a50` |
| Blender error EMA | 0.05 (`+0x3b0`), 0.01 (`+0x470`) | | `FUN_1801828f0`, `FUN_180175290` |
| UKF P0 | 0.0025 / 0.04 / (1°,5°,1°)² | | `FUN_180065b70` |
| UKF stationary | 1.96 m/s², damp 0.99 | | `FUN_18006e5a0` |
| LED cone | 20° | visibility | `FUN_1800e1c90` |
| Close / very close | 150 / 100 (mm) | | `FUN_1800db5f0` |
| RANSAC | minimal 6, w ≤ 0.9 | | `FUN_1800c8b50` |

## 7. Gaps in the current Linux driver

- **Headset INS: ported** (`userspace/lib/psvr2_ins.c`, kernels bit-exact
  in `sie/irtracker/psvr2_ins_core.c`, irtracker.md "INS core kernels").
  `pose_source.cpp` reads every IMU sample from the IIO buffer, resets the
  VIO filter to each valid SLAM sample (pose, velocity, biases) and
  re-integrates the IMU after it, blends the output filter (mode 1),
  holds position in 3DoF, and reports the pose predicted to *now* with the
  gyro rate (`poseTimeOffset` 0). Setting `headset_ins`. Differences from
  Sony, all marked [ours] in `psvr2_ins.h`: frame check from data (below),
  relaxed 6DoF entry, plain tilt correction in 3DoF, mode-1 blender only,
  snap on 3DoF→6DoF. Synthetic test (`test_ins.c`): pose error at now
  0.008° / 0.06 mm vs 0.17° / 0.6 mm for SLAM + finite-difference
  extrapolation. Not yet run on hardware.
- Still missing on the headset path: Sony's mode-0 adaptive blender (the
  one PC runs on the output filter; gains now in §5.2), the stillness / shock switches (§3.1 step
  6), tilt correction `FUN_180171400`, the SLAM `+0x188` good-frame count.
- Validity: Sony treats `state == 3 || flag0e == 1 || substate == 5` as
  valid; check what the Linux code gates on.
- Controllers (`controller_device.cpp`, `imu_fusion.h`) are IMU-only 3DoF
  with a complementary filter and a fixed `RotX(60°)` IMU→grip. Missing: the
  whole optical pipeline (§5.3), per-model IMU→controller transform (§5.6),
  UKF with delayed measurements.
- `userspace/lib/sie/*` holds bit-exact ports of helper kernels only; no
  pipeline wiring.

## 8. Open questions

1. **SLAM quaternion order.** Sony's INS reads `+0x1c` as x, y, z, w (its
   identity reset writes 1.0 at index 3, `FUN_180180200`; rotation increment
   `FUN_18017f2c0` stores w last). The Linux driver reads w, x, y, z and says
   this was confirmed on hardware. Both cannot be right for the raw record:
   capture a record with the headset level and still and check which
   component is ≈ ±1. **The INS no longer depends on it**: `psvr2_ins.c`
   fits the gyro integral between SLAM samples against the SLAM rotation
   for 4 quaternion orders × 24 IMU axis maps and uses the winner (orders
   that equal `q ⊗ k` tie with a matching axis map and give the same
   output; the identity map is preferred). Its output goes back in the
   reported order, so `RemapPose` is unchanged. The driver log line
   `headset INS running (quaternion order N, ...)` answers the question on
   hardware: order 0 = Sony's x,y,z,w, order 1 = the uapi's w,x,y,z.
2. ~~Blender per-axis deadband / min gain / snap thresholds~~ answered
   (2026-11-21): set by the bind function `FUN_180181400`, see §5.2.
3. Meaning of SLAM `+0x188` (used only when sub-state 5) and `+0x0f`.
4. UKF limits at UKF `+0x798..+0x7a0` (velocity / bias divergence limits),
   `+0x784`, `+0x864`: values come from config; not read.
5. LED↔blob matching (`FUN_1800d5070`, `FUN_1800d8b60`, `FUN_1800e2a00`)
   and `RnD::PoseCostMultiCam` residual details were not decoded in this
   pass.
6. Controller UKF state order inside the 15-vector is [likely]; confirm in
   `FUN_18006fe90` (measurement update) and `FUN_18006d0b0`.

Hardware captures wanted: EP `0x83` records while (a) still and level,
(b) covering the cameras (state ≠ 3), (c) after a SLAM reset; EP `0x88` to
confirm 2 kHz and the 500 µs u16 step.

## Corrections to the main doc

- SLAM `+0x0c`: **3 means tracking (valid 6DoF)**, not lost. Valid =
  `state == 3 || +0x0e == 1 || +0x0d == 5` (`FUN_18017a5d0`) [sure].
- SLAM `+0x1c` quaternion: Sony consumes it as **x, y, z, w** [sure for
  Sony's code; see open question 1].
- `+0x2c` is the second pose (`wTi` reference), `+0x48` velocity (`wVi`),
  `+0x54` gyro bias, `+0x60` accel bias (log strings in `FUN_180180200`)
  [sure].
- `+0x144` anchor is applied only for record versions < `0x110` [sure].
- `+0x184` is an error variance (sqrt → quality EMA), `+0x1e0` the SLAM
  reset counter, `+0x1e4` the map id (upper 16 bits latch) [sure].
- Headset IMU runs at **2 kHz** (500 µs), not 1 kHz [likely: rate check and
  averager config].

## 9. Function map

| VA | Role | Covered |
|----|------|---------|
| `FUN_18017b100` | SLAM record entry (propagate, then ingest) | yes |
| `FUN_18017a5d0` | SLAM record ingest, versions, map latch | yes |
| `FUN_18017cdb0` | INS step (SLAM or IMU tick) | yes |
| `FUN_18017b850` | Reset VIO filter to SLAM + integrate | yes |
| `FUN_180180200` | Set filter state with sanity resets | yes |
| `FUN_18017f8e0` | Strapdown propagation | yes |
| `FUN_18017f2c0` | Rotation increment with coning term | yes |
| `FUN_18017b5a0` | Propagate output + VIO filters | yes |
| `FUN_18017ca00` | Blend output → VIO | yes |
| `FUN_180181700` | Blender dispatch (mode 0/1/2) | yes |
| `FUN_180181a50` | Gated lerp/slerp blend | yes |
| `FUN_180181dd0` | Adaptive gain blend | partly |
| `FUN_1801828f0` | Blender reset | yes |
| `FUN_180181260` / `FUN_1801813b0` / `FUN_180181360` | SLAM quality monitor, good, bad | yes |
| `FUN_180175290` | INS init / reset defaults | yes |
| `FUN_180173fe0` / `FUN_180174410` | IMU calibration + decode | yes |
| `FUN_18017b2a0`, `FUN_18017b520`, `FUN_18017de70` | latched SLAM→output offset helpers | named only |
| `FUN_18005f1b0` | `UkfManager::setImuPose` | yes |
| `FUN_180067b50` | UKF measurement queue, replay, divergence guard | partly |
| `FUN_180065b70` | `kalmanFilterStep` | partly |
| `FUN_18006e5a0` | Accel / stationary update | partly |
| `FUN_18004b0a0` | Controller per-frame core | stages only |
| `FUN_1800a56b0`, `FUN_1800d5070` | Boot / pose search | stages only |
| `FUN_1800c7f00` | `ComputeBootStats` (logging) | skipped |
| `FUN_1800d5fc0` | Debug overlay (state names) | strings only |
