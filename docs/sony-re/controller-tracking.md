# Controller tracking

Optical Sense controller tracking in Sony's driver: camera model, boot and solve, background model, post-solve stages, UKF fusion and the pose handed to SteamVR.

Parts:

- [Part A: Tracking overview](#part-a-tracking-overview)
- [Part B: Camera model, boot and solve details](#part-b-camera-model-boot-and-solve-details)
- [Part C: Optical pose into SteamVR](#part-c-optical-pose-into-steamvr)
- [Part D: Boot kernels](#part-d-boot-kernels)
- [Part E: Background model](#part-e-background-model)
- [Part F: Occluded latch and arm model](#part-f-occluded-latch-and-arm-model)
- [Part G: Controller UKF](#part-g-controller-ukf)
- [Part H: UKF kernels](#part-h-ukf-kernels)

---

## Part A: Tracking overview

Sony `driver_playstation_vr2_orig.dll`, PE timestamp 1741940190. Algorithm
level description for an open reimplementation. Earlier bit-exact notes:
[headset.md part B](headset.md#part-b-headset-ins-and-ir-tracker-thread) (INS helpers), [part H](#part-h-ukf-kernels)
(UKF kernels), [part D](#part-d-boot-kernels) (robustifiers, RANSAC). This file
does not repeat their kernel-level detail.

Tags: **[sure]** read in code, **[likely]** strong inference, **[guess]**.

### A.1 Driver summary

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
   to display time. §A.5.1.
2. **6DoF / 3DoF state machine** driven by the SLAM quality value at record
   `+0x184` and the SLAM reset counter at `+0x1e0`. §A.4.1.
3. **Controller tracking** from LED blobs (EP `0x89`): model-based pose solve
   plus UKF with controller IMU. §A.5.3–5.6.

### A.2 Wire and data formats

#### A.2.1 SLAM record (EP `0x83`), as consumed by the INS [sure unless tagged]

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
| 0x184 | float | **SLAM error variance**; `sqrt` feeds the quality monitor (§A.4.1) |
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

#### A.2.2 Headset IMU samples (EP `0x88` body) [sure]

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

### A.3 Sequences

#### A.3.1 SLAM sample arrival (`FUN_18017cdb0`) [sure]

1. If the INS has no time base yet: take the newest IMU time; give up if it
   is more than 20 ms from the SLAM time; else set `t_ins = t_imu`.
2. **Mode** (`INS+0x180`, 0 normal / 1 force 6DoF / 2 IMU only). Mode 2 skips
   to step 6.
3. Update the SLAM quality monitor (§A.4.1) with `+0x184`, `+0x188`, valid.
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
   changed; or quality bad (§A.4.1). Mode 1 forces 6DoF.

### A.4 State machines

#### A.4.1 6DoF / 3DoF (INS `+0x178`) and the SLAM quality monitor

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

### A.5 Algorithms

#### A.5.1 INS strapdown propagation (`FUN_18017f8e0`) [sure]

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

#### A.5.2 Output blender (`FUN_180181700`, two instances at INS `+0x3b0`, `+0x470`)

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

#### A.5.3 Controller tracking pipeline (overview)

Entry: IR tracker thread `FUN_18018aaf0`, once per EP `0x89` "LD" packet.

| Stage | Function | Input | Output |
|-------|----------|-------|--------|
| 1. Blob decode | `FUN_180184900` | LD packet, 4 cameras × ≤ 256 blobs | centroid, 2×2 covariance, kind (overview.md part B) |
| 2. LED sync | `FUN_180194af0`, libpad `LedSyncer` | blob counts per frame | LED blink phase, which LED subset is lit (`led[0..3]` masks) |
| 3. Model select | inline in `FUN_18018aaf0` (switch on hardware rev 1..6, default) | controller rev, hand | LED model table + IMU-pose table, `ImuPoseIndex` = 2·rev + 1 + hand (3..14), default 15/16 |
| 4. IMU pose | `FUN_18005f1b0` (`UkfManager::setImuPose`) | ImuPoseIndex | IMU→controller 4×4 per device (§A.5.6) |
| 5. Per-frame core | `FUN_18004b0a0` | blobs, head pose at LD timestamp, camera extrinsics (`FUN_1800cbd70`, table at `+0x10a40`) | per-device candidate poses (boot or tracking) |
| 6. Boot / recovery | `FUN_1800a56b0` → `FUN_1800d4b90` → `FUN_1800d5070` | blobs + model | pose hypotheses (RANSAC over 3 LEDs × 3 blobs (P3P), ring stage first; 200 iterations cold, 50 while tracking; adaptive flag 0; part B §B.2.19) |
| 7. Refinement | `RnD::PoseCostMultiCam` + Huber/Tukey (part D) | hypothesis, LED↔blob matches in all cameras | refined 6DoF pose + inlier count |
| 8. Fusion | `FUN_180067b50` → `FUN_180065b70` ("kalmanFilterStep") | controller IMU (from libpad HID path `FUN_18011bae0` → `FUN_18005e6e0`) and optical poses (`FUN_18005ec00`) | filtered pose, velocity, biases |
| 9. Stats | `FUN_18004cac0`, `FUN_1800c7f00` (`ComputeBootStats`), `FUN_1800a2f70` | — | logging only (skip) |

Frames: LED models are **mm in controller frame**; camera calibration
translation is converted m → mm by a 180° rotation about the optical axis:
`t′ = 1000·(−tx, −ty, tz)`, R conjugated by diag(−1, −1, 1) (`FUN_1800b0130`,
`FUN_1800b02a0`, part D). So the optical solver works in **mm,
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
*P1PFromFailedUndistort* is the **lost** state; there is no undistort
iteration behind it [sure]. The enum at tracker `+0x4c94` is 0 none, 1 kNLR,
2 P2P, 3 kP1P, 4 lost ("kP1PFromFailedUndistort"), 5 kCrossBlobPnPR,
7 kMultiCamNLR, 8 kMultiCamP2PR [sure] (part B §B.1.8).

#### A.5.4 Robust cost (summary)

Residual per matched LED = reprojection error (pixels) in each camera,
weighted by IRLS weights from the robustifier (part D §Robust loss).
Huber for refinement, Tukey/Clip for inlier counting [likely]. RANSAC bound
`n = log(1−p)/log(1−w^6)`, inlier ratio capped at 0.9 [sure, `FUN_1800c8b50`].

#### A.5.5 UKF (`UkfManager`, `FUN_180065b70`) [sure unless tagged]

- **Error state 15**: five 3-blocks at 0, 3, 6, 9, 12 (packed upper
  triangle, part H): position, velocity, attitude, gyro bias, accel
  bias [likely order; blocks 0, 9, 12 are reset together in `FUN_18006e5a0`].
- **Initial covariance** on (re)initialisation: position `0.0025` (σ 5 cm),
  velocity `0.04` (σ 0.2 m/s), attitude `(3.05e-4, 7.62e-3, 3.05e-4)` =
  (1°)², (5°)², (1°)² — **yaw (about Y) is the loose axis**.
- **Process noise**: `FUN_180062880` / `FUN_180062950`
  (part H), angular-rate dependent; config at UKF `+0x6a0`.
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

#### A.5.6 IMU → controller transform (`FUN_18005f1b0`) [sure]

Per `ImuPoseIndex` a constant 4×4 (column-major, metres) is **inverted** and
stored per device (`+0x808`, device stride `0x2218`). Index 0 = identity.
Example (index 1): rotation columns ≈ (−0.276, −0.197, 0.941),
(0.736, 0.587, 0.338), (−0.619, 0.785, −0.017), translation
(0.0145, 0.0129, −0.0084) m. The table is used only when no pad vector is
passed; Sony always passes one, so the stored transform is a pure
translation by pad `+0x48` (part C §C.2b.1). Indices ≥ 3 can take a per-device matrix argument
(calibration) instead of the table and also set limits ±33.16 and 250.0 at
`+0x6c8`/`+0x6cc`/`+0x7d0` [meaning unknown]. Read the tables with the
section reader; do not commit them.

#### A.5.7 Stillness test and tilt correction (`FUN_18017cdb0` IMU tick, `FUN_180171400`) [sure unless tagged]

Each IMU tick (step 6 of §A.3.1) takes statistics over the whole IMU ring
(`FUN_180173830`, 300 samples once wrapped, at least 5): per-axis mean and
variance of the accelerometer and of the gyro.

**Stillness.** `FUN_1801732f0` passes when
`|var_accel|² < 3·0.05²` and `|var_gyro|² < 3·0.005²` (parameter block
+0x00 and +0x08). While it passes, `still_us += 500·n` (n = samples this
tick); otherwise `still_us = 0`. Stationary = `still_us > 1 500 000`
(core `+0x564` / `+0x560`).

**Tilt correction.** Runs when its enable byte (behind core `+0x190`) is
set and the ring statistics are valid. Inputs: mean accel `a` (IMU frame),
`va` = mean of the three accel variances, `vg` = mean of the three gyro
variances, the output filter's orientation `q` (x, y, z, w), and
`T = n·dt` (time covered by this tick). State: reference gravity `g`
(world, `(0, −9.8, 0)`), a running world-frame estimate `m` of the
measured specific force with spread `σ`, and a "large tilt" flag.

```
f   = q ⊗ a ⊗ q*                           // accel in world axes
d   = | |a| − |g| |
w   = max(1 − s(2d − 2) + s(−2d − 2), 1e-8)  // s(x) = 1/(1+e^−x); w ≈ 1 when |a| ≈ |g|
wA  = 0.05 / σ                              // weight of the old estimate
wB  = w · (T / (max(va,1e-8) / 0.05)) / (max(vg,1e-8) / 0.005)
// weighted merge of two Gaussians (FUN_1801711f0)
m'  = (wA·m + wB·f) / (wA + wB)
v'  = (wA·(|m|² + σ²) + wB·(|f|² + max(va,1e-8)²)) / (wA + wB) − |m'|²
if v' > 0: m = m', σ = sqrt(v')            // else keep the old estimate
if σ or m is NaN: m = f, σ = 0.1; stop

n   = (m × g) / (|m|·|g|)
θ   = asin(min(|n|, 1)); if m·g > 0: θ = π − θ   // angle between m and −g
tilt_out = |θ|
flag = 1 if |θ| > 20°; flag = 0 if |θ| < 0.1° (hysteresis)
k   = T / 0.05
if flag == 0: k = k · (θ in degrees)²       // small errors are pulled in slowly
if k > 0:
    k = min(k, 1)
    r = rotation by −θ·k about n/|n|        // quaternion (sin(−θk/2)·n̂, cos(−θk/2))
    q = r ⊗ q                               // Hamilton, premultiplied (world axes)
    m = r · m                               // keep the estimate consistent
    applied_out = k·θ
else applied_out = 0
```

The corrected `q` is written back to the output filter **only in 3DoF**
(INS `+0x178` byte = 0); in 6DoF the estimate still runs but the
orientation comes from the blender (§A.5.2). With a large error (flag
set) the time constant is 50 ms. Below 20° the gain is multiplied by the
square of the error in degrees, so a 1° error is corrected at rate
`T/0.05` and a 0.1° error 100 × slower.

### A.6 Constants

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
| Quality EMA | 0.04, clamp 10, bad 5.0, exit 2.5, enter 2.0, n > 30 | §A.4.1 | `FUN_180181260` |
| Blender mode-1 gain | α = min(4·dt, 1) | | `FUN_180181a50` |
| Blender error EMA | 0.05 (`+0x3b0`), 0.01 (`+0x470`) | | `FUN_1801828f0`, `FUN_180175290` |
| UKF P0 | 0.0025 / 0.04 / (1°,5°,1°)² | | `FUN_180065b70` |
| UKF stationary | 1.96 m/s², damp 0.99 | | `FUN_18006e5a0` |
| LED visibility margin | 20° (`cos 0.349`) | `FUN_1800e1c90` test | `FUN_1800e1c90` |
| LED cone | 170° (rev 1–3) / 180° (rev 4–7) | per model, model word 4 | part B §B.2.4 |
| Close / very close | 150 / 100 (mm) | | `FUN_1800db5f0` |
| RANSAC | P3P samples; 200 cold / 50 while tracking; adaptive bound off | the 6-point adaptive bound exists but its flag is 0 | `FUN_1800c8b50`, `FUN_1800d8b60` |

### A.7 Gaps in the current Linux driver

- **Headset INS: ported** (`userspace/lib/psvr2_ins.c`, kernels bit-exact
  in `sie/irtracker/psvr2_ins_core.c`, headset.md part B "INS core kernels").
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
  one PC runs on the output filter; gains now in §A.5.2), the stillness / shock switches (§A.3.1 step
  6), tilt correction `FUN_180171400` (now in §A.5.7), the SLAM `+0x188` good-frame count.
- Validity: Sony treats `state == 3 || flag0e == 1 || substate == 5` as
  valid; check what the Linux code gates on.
- Controllers (`controller_device.cpp`, `imu_fusion.h`) are IMU-only 3DoF
  with a complementary filter and a fixed `RotX(60°)` IMU→grip. Missing: the
  whole optical pipeline (§A.5.3), per-model IMU→controller transform (§A.5.6),
  UKF with delayed measurements.
- `userspace/lib/sie/*` holds bit-exact ports of helper kernels only; no
  pipeline wiring.

### A.8 Open questions

1. ~~SLAM quaternion order~~ answered: the record is **x, y, z, w** (Sony's
   identity reset writes 1.0 at index 3, `FUN_180180200`; rotation increment
   `FUN_18017f2c0` stores w last). The Linux driver's remap equals Sony's
   `q_dfh` plus a world yaw, so both observations were right
   (part B §B.4.2). The Linux driver lacks
   `vecDriverFromHeadTranslation`; use (0, 0.0208, 0.09255) m in head axes.
   **The INS no longer depends on it**: `psvr2_ins.c`
   fits the gyro integral between SLAM samples against the SLAM rotation
   for 4 quaternion orders × 24 IMU axis maps and uses the winner (orders
   that equal `q ⊗ k` tie with a matching axis map and give the same
   output; the identity map is preferred). Its output goes back in the
   reported order, so `RemapPose` is unchanged. The driver log line
   `headset INS running (quaternion order N, ...)` answers the question on
   hardware: order 0 = Sony's x,y,z,w, order 1 = the uapi's w,x,y,z.
2. ~~Blender per-axis deadband / min gain / snap thresholds~~ answered
   (2026-11-21): set by the bind function `FUN_180181400`, see §A.5.2.
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

### A.9 Function map

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

---

## Part B: Camera model, boot and solve details

Scope: the items in `development.md` part A "Not done, and why". Each section
gives only the unknown that blocked the item. Facts already in
part A, `controllers.md` part C, part D, part H,
`headset.md` part B and `overview.md` part B are not repeated.

Tags: **[sure]** read in code, **[likely]** strong inference, **[guess]**.
Addresses are function VAs in `driver_playstation_vr2_orig.dll`
(PE timestamp 1741940190).

---

### B.1 Raw camera calibration to camera model

#### B.1.1 Summary

The model is **OpenCV's 14-coefficient camera model plus six more rational
terms (degree 12 in r)**, with these differences:

- the thin-prism terms s1..s4 are stored but **not used** by the
  projection;
- the camera frame is the calibration frame **rotated 180° about the
  optical axis**, in mm;
- intrinsics are **rescaled to 508 × 508**.

Concretely: fx, fy, cx, cy are K; D0..D13 are OpenCV's
(k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4, τx, τy); D14..D19 extend
the numerator and denominator to r¹⁰ and r¹². [likely for the names; the
data flow is sure]

The user's hints were partly off:

- The builder is **not** `FUN_1800c8950` or `FUN_1800c8a40`. Those set up a
  RANSAC settings block that holds a Mersenne Twister.
- The struct is **0x1c0 bytes**, not 0x180. A camera model copy that holds
  0x180 bytes keeps only the part the forward projection needs.

#### B.1.2 Live data path [sure]

| Step | Function | What happens |
|------|----------|--------------|
| 1 | `FUN_180127250` | VD entry type 10 → copied to headset context `+0x40220` (0x800 bytes), counter `+0x43dd4` ++. Type 11 → `+0x40a20` (not used by the IR tracker) |
| 2 | `FUN_18018aaf0` (IR thread start, and again whenever the counter changes, checked every 64th LD packet) | `FUN_1801247d0` copies the 0x800-byte type-10 blob, then calls `FUN_180182e80(blob, 0x800, cfg)` |
| 3 | `FUN_180182e80` | `FUN_180161b70` parses the SIE blob into doubles (layout in overview.md part B). Converts each camera to a 26-float record and the first 3 extrinsics to float R/t. Writes them into the tracker config `cfg`. |
| 4 | `FUN_180049240` (tracker init; `cfg` is `param_1`) | Per camera `i < cfg+0x14` (= 4): `FUN_180045420` builds the model; `FUN_18009d4d0` derives the polynomial lanes and tilt matrices; the 0x1c0-byte model is copied to tracker global `+0x10340 + i·0x1c0` |
| 5 | `FUN_180049240` → `FUN_1800b02a0` (because `cfg+0x1ec` = 1) | Extrinsics → 4×4 mm matrices at `DAT_1805f32e0 + i·0x40`, copied to tracker global `+0x10a40 + i·0x40` |

The config-file path (`FUN_18004a340`, `ConfigFile0.trk`) is debug only.
`cfg+0x1ec` = 0 selects the built-in default extrinsics (`FUN_1800af650`)
instead of step 5.

Tracker config values set by `FUN_18018aaf0` before step 3 [sure]:

| `cfg` offset | Value | Meaning |
|---|---|---|
| +0x14 | 4 | camera count |
| +0x18 | 2 | [unknown] |
| +0x1c | 0x1fc = **508** | model width (the static default `DAT_180351764` is 640) |
| +0x20 | 0x1fc = **508** | model height (static default `DAT_180351768` is 480) |
| +0x24 | 1 | "calibration present" (set by `FUN_180182e80`) |
| +0x28 + i·0x6c | camera record i (below) | |
| +0x1d8 | 2 | (set by `FUN_180182e80`) |
| +0x1ec | 1 | "use calibrated extrinsics" |
| +0x1f0 / +0x214 | R0 (9 f32, row-major) / t0 (3 f32, metres) | extrinsic 0 |
| +0x21c / +0x240 | R1 / t1 | extrinsic 1 |
| +0x250 / +0x274 | R2 / t2 | extrinsic 2 (inverted on load if its record says from = 2, to = 0) |

#### B.1.3 Step 3: raw doubles → 26-float camera record (`FUN_180182e80`) [sure]

Here `K[0..8]` is the parsed 3×3 (row-major) and `D[0..19]` are the 20
doubles at blob record `+0x2c`. Record i lives at `cfg + 0x28 + i·0x6c`.
The fields in order are:

| Rec off | Value |
|---|---|
| +0x00 | width (from blob, as float) |
| +0x04 | height |
| +0x08 | fx = K[0] |
| +0x0c | fy = K[4] |
| +0x10 | cx = K[2] |
| +0x14 | cy = K[5] |
| +0x18..+0x44 | D0, D1, D4, D5, D6, D7, D14, D15, D16, D17, D18, D19 |
| +0x48..+0x54 | D2, D3, D12, D13 |
| +0x58..+0x64 | D8, D9, D10, D11 |
| +0x68 | u8 flag = 1 (valid) |

K[1] (skew), K[3], K[6..8] are dropped. Only cameras 0..3 are copied.

#### B.1.4 Step 4: model struct (`FUN_180045420`, `FUN_18009d4d0`) [sure unless tagged]

Scale factors: `sx = 508 / width`, `sy = 508 / height`.

| Struct off | Value | Role |
|---|---|---|
| +0x00 | sx·fx | fx′ (stage 2 "scale" x) |
| +0x04 | sy·fy | fy′ |
| +0x08 | (cx + 0.5)·sx − 0.5 | cx′ (stage 2 "offset" x; pixel-centre convention) |
| +0x0c | (cy + 0.5)·sy − 0.5 | cy′ |
| +0x10 / +0x14 | sx·width / sy·height (= 508, 508) | model image size; also copied to `DAT_18035176c` / `DAT_180351770` as ints |
| +0x18, +0x1c, +0x20 | D0, D1, D4 | numerator k1, k2, k3 (r², r⁴, r⁶) |
| +0x24, +0x28, +0x2c | D5, D6, D7 | denominator k4, k5, k6 |
| +0x30, +0x34, +0x38 | D14, D15, D16 | numerator r⁸, r¹⁰, r¹² |
| +0x3c, +0x40, +0x44 | D17, D18, D19 | denominator r⁸, r¹⁰, r¹² |
| +0x48 | −D2 | p1 (negated) |
| +0x4c | −D3 | p2 (negated) |
| +0x50 | −D12 | τx, tilt angle (rad), negated |
| +0x54 | −D13 | τy, negated |
| +0x58..+0x64 | D8..D11 | s1..s4, **not read by the projection** `FUN_180045650` |
| +0x68 / +0x6c | sin, cos of (+0x50) | |
| +0x70 / +0x74 | sin, cos of (+0x54) | |
| +0x78..+0x84 | sin², cos² of the same | |
| +0x90..+0xbc | 3 columns × 4 f32 | tilt matrix T (column-major, 4th lane unused) |
| +0xc0..+0xef | 3 columns × 4 f32 | T⁻¹ (`FUN_18009d310` + `FUN_180044f60`) [likely: inverse] |
| +0xf0 + 0x10·j, j = 0..5 | 4 lanes | polynomial lanes, below |
| +0x150 + 0x10·j, j = 0..5 | 4 lanes | derivative lanes, below |
| +0x1b0 | i32 | camera index |

Polynomial lanes. Write `n = (D0, D1, D4, D14, D15, D16)` for the numerator
and `d = (D5, D6, D7, D17, D18, D19)` for the denominator. Then for
j = 0..5:

| Offset | lane 0 | lane 1 | lane 2 | lane 3 |
|---|---|---|---|---|
| +0xf0 + 0x10·j | d[j] | 2(j+1)·d[j] | (2j+3)·n[j] | n[j] |
| +0x150 + 0x10·j | d[j] | (j+1)·d[j] | (j+1)·n[j] | n[j] |

The constants 2..13 come from `0x180319910`..`0x180319970`.

Each lane is evaluated as `1 + Σ_j lane[j] · (r²)^(j+1)`. So:

- lane 3 of block `+0xf0` = N(r) = 1 + k1 r² + k2 r⁴ + k3 r⁶ + n3 r⁸ + n4 r¹⁰ + n5 r¹²;
- lane 0 of block `+0xf0` = Dn(r), the same with d;
- lane 1 of block `+0xf0` = 1 + r·Dn′(r);
- lane 2 of block `+0xf0` = (r·N)′.

Only lanes 0 and 3 of block `+0xf0` feed the forward projection.

- **Lanes 1 and 2, and the block at `+0x150`**, are read by
  `FUN_180103580`. That function also evaluates the forward lanes and takes
  a 4×4 pose, so it is the projection **Jacobian** used by the refiners
  [likely]. Other readers of these offsets: `FUN_1801052f0` (same family),
  and `FUN_180065760` [unchecked].
- Nothing found uses them for an undistort.

#### B.1.5 Forward projection, in full [sure]

Callers: `FUN_180046330` (applies a column-major 4×4, then calls
`FUN_180046240`) and `FUN_180046240` (camera-frame point → pixel).

```c
// P = (X, Y, Z): point in Sony camera frame, mm
bool project(model *m, vec3 P, vec2 *uv) {
    if (Z == 0) return false;
    float x = X / Z, y = Y / Z;              // +Z is forward
    float r2 = x*x + y*y;
    if (r2 > 200.0f) return false;           // 0x1803199dc
    float N  = 1 + Σ n[j] r2^(j+1);          // lane 3
    float Dn = 1 + Σ d[j] r2^(j+1);          // lane 0
    float rad = N / Dn;
    // m->p1 = -D2, m->p2 = -D3 (stored negated)
    float xd = x*rad + 2*m->p1*x*y + m->p2*(r2 + 2*x*x);
    float yd = y*rad + 2*m->p2*x*y + m->p1*(r2 + 2*y*y);
    vec3 v = T * (xd, yd, 1);                 // tilt, +0x90
    xd = v.x / v.z;  yd = v.y / v.z;
    uv->u = m->cx_ - m->fx_ * xd;            // stage 2: offset − scale·p
    uv->v = m->cy_ - m->fy_ * yd;
    return Z > 1e-4f;                        // 0x180319730: in front
}
```

The constant at `DAT_180319dc0` is 2.0, and `DAT_180319d60` is 1.0
(4 lanes).

T is OpenCV's `computeTiltProjectionMatrix` built from the stored
(negated) angles τx′ = +0x50 and τy′ = +0x54:

```c
R = Ry(τy′) · Rx(τx′)
T = [[R22, 0, -R02], [0, R22, -R12], [0, 0, 1]] · R
```

Same structure as OpenCV [likely; the angle signs follow OpenCV's
`matRotX` and `matRotY`].

**Equivalence to OpenCV [likely, derived].** Take
(X_cv, Y_cv, Z_cv) = (−X, −Y, Z), a 180° rotation about Z. Then Sony's
x, y = −x_cv, −y_cv. The negated p1, p2, τx, τy cancel every sign flip,
and the minus in stage 2 cancels the last one:

```
u = cx′ + fx′·xd_cv(x_cv, y_cv)
```

where `xd_cv` is OpenCV's distortion with `(k1..k6, p1 = D2, p2 = D3,
τx = D12, τy = D13)`, s = 0, and the two extra rational orders.

So: **feed OpenCV the raw D values and the pixel-centre-rescaled K, but
rotate points 180° about the optical axis first.** Or use the raw
calibration frame directly. The rotation only matters because Sony's
extrinsics (§B.1.7) live in the rotated frame.

#### B.1.6 Worked example

No real blob is available: the dev module was not loaded, and no
`/sys/bus/usb/devices/*/camera_calib0` exists. Symbolically, for a blob
camera record with width W = 1016 [guess] and K, D:

| Struct field | = |
|---|---|
| +0x00 fx′ | (508/W)·K[0] |
| +0x08 cx′ | (K[2] + 0.5)·508/W − 0.5 |
| +0x18 / +0x1c / +0x20 | D[0] / D[1] / D[4] |
| +0x24 / +0x28 / +0x2c | D[5] / D[6] / D[7] |
| +0x30..+0x38 | D[14] / D[15] / D[16] |
| +0x3c..+0x44 | D[17] / D[18] / D[19] |
| +0x48 / +0x4c | −D[2] / −D[3] |
| +0x50 / +0x54 | −D[12] / −D[13] |

**Check on hardware**, with the dev tracker's raw log line:

- D8..D11 should be ≈ 0 if the model is OpenCV-like.
- D12, D13 should be small angles (|τ| < 0.05 rad).
- The blob's W, H should be ≥ 508.

#### B.1.7 Extrinsics (0x34-byte records) [sure unless tagged]

`FUN_1800b02a0`, and its helper `FUN_1800b0130`, convert each calibration
R (row-major, metres) to a 4×4 column-major float matrix:

```
R′ = G·R·G,  t′ = 1000·G·t,  with G = diag(−1, −1, +1)
```

The constants are `0x180319a70` = −1000 and `0x180319a04` = +1000; the
sign masks flip R[0][2], R[1][2], R[2][0], R[2][1].

So the **units are mm**. The frame change is the same 180° rotation about
Z as in §B.1.5. part A says "z axis flipped": that is equivalent for
R, but wrong for t, where x and y flip and z does not.

`FUN_1800b02a0` then composes each extrinsic with camera 0's built-in rig
pose (`FUN_1800af650(…, 0)`) to place every camera in the **rig frame**
(the LED solver's "HMD" frame). The tracker stores these at global
`+0x10a40 + 0x40·cam`, and the per-frame core passes them to the solver
(`FUN_1800cbd70`) [likely].

Direction:

- `FUN_180182e80` normalises extrinsic 2 by inverting it when its record
  says from = 2, to = 0. So the expected convention is "from = 0, to = k":
  it maps points in camera 0's frame into camera k's frame, or the reverse
  [likely; not decidable without a real blob].
- Check with the blob: a pure x-offset of ≈ ±0.08 m or ±0.135 m between
  cameras matches the defaults below.

VD type 13 (`imu_set_calib`, 0x34 bytes) is exactly **one extrinsic
record**: the IMU ↔ camera link [likely].

Built-in default rig poses (`FUN_1800af650`; translation column, mm, in the
rig frame; three hardware variants chosen by `param_2`) [sure for values]:

| Cam | variant 0 (default) | variant 1 | variant 2 |
|---|---|---|---|
| 0 | (+40.0, −41.68, 93.58) | (+38.19, −41.62, 99.44) | (+40.0, −39.07, 79.40) |
| 1 | (−40.0, −41.68, 93.58) | (−38.19, −41.62, 99.44) | (−40.0, −39.07, 79.40) |
| 2 | (+61.50, 22.97, 97.03) | (+65.98, 30.71, 106.24) | (+67.5, 32.14, 80.11) |
| 3 | (−61.50, 22.97, 97.03) | (−65.98, 30.71, 106.24) | (−67.5, 32.14, 80.11) |

Tables at `0x180319b40`, `0x180319b80`, `0x180319b00`.

**Camera index ↔ physical camera:**

- Cameras 0 and 1 are the inner pair, 80 mm apart. Cameras 2 and 3 are the
  outer pair, about 123–135 mm apart, on the opposite y side.
- Even indices are on +x, odd on −x.
- In the rig frame (+z forward from the face) [likely], with y up [guess]:
  0/1 are the lower pair and 2/3 the upper outer pair.
- Confirm on hardware: cover one camera and watch which LD camera index
  loses its blobs.
- LedSync uses cameras 0 and 1 for its "Many Blob" test (controllers.md §C.4.3).

#### B.1.8 "Undistort" and `kP1PFromFailedUndistort` [sure unless tagged]

There is **no iterative undistort** behind that name. Method enum at tracker
`+0x4c94` (overlay `FUN_1800d5fc0`): 0 none, 1 kNLR, 2 P2P, 3 kP1P,
4 kP1PFromFailedUndistort, 5 kCrossBlobPnPR, 7 kMultiCamNLR, 8 kMultiCamP2PR.

Close-range state machine, `FUN_1800db690`:

- d = |pose translation| in mm, from the camera/rig origin.
- "valid" = solver output `+0x4ac`.
- "close" = tracker flag `+0x3360`.

| State | valid and … | → |
|---|---|---|
| 0 | close, d < 80 | 2 |
| 0 | close, 80 ≤ d < 130 | 1 |
| 1 | d > 170 | 0 |
| 1 | close, d < 80 | 2 |
| 2 | d > 120 | 1 |
| 3 | close, d < 130 | 1 |
| 3 | not close or d ≥ 130, and count `+0x49c` ≥ 6 | 0 |
| 0 | `FUN_1800db420(…) < 4` (fewer than 4 usable LEDs) [likely] | 3 |
| 0..3 | **not valid** | **4** |

- State 4 is left only by a successful re-boot in `FUN_1800d5070`:
  d < 100 → 2, d < 150 → 1, else → 0.
- State-specific solvers: 0 `FUN_1800db980`, 1 `FUN_1800dbda0`,
  2 `FUN_1800dc210`, 3 `FUN_1800dc680`.
- Thresholds are at `0x180319998` (80), `0x1803199a0` (100),
  `0x1803199a8` (120), `0x1803199ac` (130), `0x1803199c0` (150) and
  `0x1803199c8` (170).

So "P1P from failed undistort" is the **lost** state. No inverse
distortion is needed to build a first tracker. A blob → ray conversion
can use any iterative inverse of §B.1.5; Sony's was not found [open].

#### B.1.9 Still unknown (item 1)

- Extrinsic direction (from → to): needs one real blob.
- Which of extrinsics 0 and 1 links which cameras.
- Width/height in the blob (whether 508/W is exact).
- Physical camera names: needs the cover test.
- Next functions to read: `FUN_1800b02a0` lines after the first 4 calls
  (composition order), and `FUN_1800cbd70` (how the per-frame core uses
  `+0x10a40`).

---
### B.2 Controller pose solve: the gaps

Item 2 is closed. The chain is: blob ROI crop (§B.2.14) → boot config
(§B.2.4) → single-camera boot (§B.2.9) or orientation-prior multi-camera boot
(§B.2.13) → association (§B.2.8) → gated match (§B.2.11) → weighted LM with a
robust kernel (§B.2.10) → MSAC score (§B.2.12). Only small leftovers remain
(§B.2.15).

#### B.2.1 Per-frame blob/LED tables shared by the solver and LedSync [sure]

The IR thread's frame struct (`lStack_16a50` in `FUN_18018aaf0`, 0x9d28
bytes) holds one block per controller (stride **0x5a44**). Each controller
block holds one sub-block per camera (stride **0x1688**, 4 cameras, plus
0x24 bytes).

Read by `FUN_1801999f0`, `FUN_1801957d0` and `FUN_18004cac0`:

| Camera sub-block off | Type | Meaning |
|---|---|---|
| +0x30 | u16 | blob count for this controller in this camera (ctrl-block +0x30 is checked per controller) |
| +0x34 | u8 | camera used for this controller |
| +0x38 + 0x14·b | 0x14 bytes | blob b: … +0x08 f32 weight (blob size/brightness), +0x10 s16 state (1 = associated) [likely] |
| +0x1438 + 2·L | s16 | **LED L → blob index** in this camera, −1 = none |
| +0x15bc + 4·L | f32 | per-LED view metric (below) |

So association output is a **per camera LED → blob map**. Each LED maps
to at most one blob, so it is unique per LED. Whether one blob can serve
two LEDs was not checked.

#### B.2.2 LED visibility metric (+0x15bc, the "≈45") [sure for the use, guess for the unit]

`FUN_1801999f0` uses `m = metric·0.5`:

- an LED is a sync candidate when `m ≠ 0` (epsilon test against
  DBL_EPSILON) and `m ≤ 45.0` (`0x18026114c`, and the per-slot bests
  `0x42340000` = 45.0);
- smaller is better.

The writer of `+0x15bc` was not found (it is filled through a block copy,
not by a literal offset). The solver's own visibility test is in
part A: `FUN_1800e1c90`, normal within `cos(0.349)` of the cone,
[sure].

Resolved by §B.2.8 [likely]: the metric is the LED-normal ↔ view-ray angle
in degrees (0 = face-on), so the LedSync gate `metric·0.5 ≤ 45` keeps LEDs
within 90° of facing the camera. The direct writer of `+0x15bc` (a block
copy from the per-pose `+0x51c` array) was not pinned to one instruction.

#### B.2.3 Hardware type → model (`FUN_18018aaf0`) [sure]

- rev = libpad `FUN_1801bc660` = feature `0x20` +0x18 low 16 bits.
  The table is in `controllers.md` §B.3.2. Stored at manager pad `+0x3c`
  (`FUN_18011bae0`), read by `FUN_180118b00`.
- It is **not** in the 0xf0-byte info block: info `+0x3c` is pad `+0x12c4`,
  a statistics histogram.

| rev | LED model (tuning, model) | ImuPoseIndex (hand 0 / 1) |
|---|---|---|
| 1 | `0x180257220`, `0x180257280` | 3 / 4 |
| 2 | `0x180258620`, `0x180258680` | 5 / 6 |
| 3 | `0x180259a20`, `0x180259a80` | 7 / 8 |
| 4 | `0x18025ae20`, `0x18025ae80` | 9 / 10 |
| 5 | `0x18025c220`, `0x18025c280` | 11 / 12 |
| 6 | `0x18025d620`, `0x18025d680` | 13 / 14 |
| other (8–11, errors) | `0x18025ea20`, `0x18025ea80` | 15 / 16 |

The descriptor handed to the tracker is
`{…, tuning ptr, size 0x54, model ptr, size 0x139c}`. Hand 1 also sets byte
0x58 in the descriptor.

The "preset" (info `+0x10`) is pad `+0x1298`. `FUN_18011bae0` copies it
from byte `+0x6f` of libpad's controller state sample. The HID origin of
that byte was not traced.

#### B.2.4 Tuning block = boot config (`ConfigFile0.trk` keys) [sure]

The 0x54-byte block is the controller **boot config**. `FUN_18004a340`
copies it, with a word permutation, into a 700-byte per-controller config
(`0x1805f0720 + ctrl·0x2bc`) when the device descriptor is used. Without a
descriptor the same config is read from a text file `ConfigFile0.trk`
(`KEY = value` lines, `#` comments) by `FUN_1800a6940`, whose key strings
give the names. `FUN_1800d3360` then copies it into the boot object
(`+0x610…+0x6a4`). All 7 models carry the same values.

| Tuning word | Value | Config off | Key | Boot obj off |
|---|---|---|---|---|
| 0 | 2 | +0x04 | NUMFRAMESUNTILBOOT | +0x674 |
| 1 | 2 | +0x08 | INLIERTHRESHOLD (px) | +0x634 (as float) |
| 2 | 1 | +0x0c | POTHOLETHRESHOLD | +0x64c (as float) |
| 12 | 1 | +0x10 | SCALEDPOTHOLE-THRESHOLD (bool) | +0x648 |
| 13 | 0 | +0x14 | SCALEDINLIER-THRESHOLD (bool) | +0x649 |
| 11 | 1 | +0x18 | SCALEDSEARCH-RADIUS (bool) | +0x64a |
| 3 | 3 | +0x1c | SEARCHRADIUS (px) | +0x65c (as float) |
| 4 | 7 | +0x20 | MININLIERSFORBOOT | +0x610 |
| 5 | 5 | +0x24 | MININLIERS-FOR-MULTICAMBOOT | +0x614 |
| 6 | 5 | +0x28 | MININLIERS-FOR-STRICTGRAVITYBOOT | +0x618 |
| 7 | 4 | +0x2c | MININLIERS-FOR-MULTICAMSTRICTGRAVITYBOOT | +0x61c |
| 8 | 4 | +0x30 | MININLIERS-FOR-ORIENTATIONBOOT | +0x620 |
| 14 | 200 | +0x34 | RANSACMAXSAMPLES | +0x63c |
| 15 | 50 | +0x38 | RANSAC-BOOTINTRACK-MAXSAMPLES | +0x640 |
| 16 | 0.5 | +0x3c | RANSACRINGBOOTRATIO | +0x690 |
| 17 | 1 | +0x40 | USEGRAVITYBOOT (bool) | +0x694 |
| 19 | 240.0 | +0x44 | MINSIZEDOT09 | +0x6a0 |
| 20 | 220.0 | +0x48 | MINSIZEDOT07 | +0x6a4 |
| 9, 10, 18 | 0.9, 0.2, 0 | — | not copied | — |

- `MODELNAME` (string) sits at config `+0x4c`; the descriptor path does not
  set it.
- `FUN_1800d3360` also copies a block from `self+0x610` onto itself
  (call-site disassembly at `0x18004a8b3`: `r11 = obj+0x610`), so it is a
  no-op [sure].
- The px units of INLIERTHRESHOLD / SEARCHRADIUS are [likely]: they are
  compared to reprojection distances. "SCALED" means multiplied by
  1/distance(m), see §B.2.9.
- The "0.9" in word 9 is not the RANSAC cap; the cap is the separate
  constant in §B.2.5. Previous [guess] names for words 0–20 are withdrawn.

**Model header** (the 0x139c block): word 0 = LED count (20 for rev 1,
17 for rev 2–7), word 4 = LED cone angle in degrees (**170** for rev 1–3,
**180** for rev 4–7), copied to boot obj `+0x660` via `0x1805f4054`
(`FUN_1800d3ae0`) [sure]. Word 5 = 48 [unknown], copied to `0x1805f4058`.

#### B.2.5 RANSAC count (`FUN_1800c8b50`, settings from `FUN_1800c8950`) [sure]

```c
max_iter = 1000;                      // settings +0x00 and +0x04 default 1000; boot overrides: 200 cold, 50 in tracking (§B.2.4)
if (settings.adaptive) {              // +0x0c
    w = MIN(inlier_ratio, 0.9f);      // +0x1c, cap 0x180319818
    n = (int)(log(1 - p) / log(1 - pow(w, 6.0)));   // p = +0x14 confidence
    iterations = MIN(iterations, n);
}
```

The sample size is **6** (`0x1803198d0`). Settings `+0x08` = 12 [unknown].
The block also holds a Mersenne Twister seeded with **0x29a** (666):
`FUN_1800c8950`.

#### B.2.6 Close-range / lost state machine

See §B.1.8 (`FUN_1800db690`). Distances are in mm:

- 80 / 120 / 130 / 170 for entering and leaving "close" modes;
- re-boot classes 100 / 150.

#### B.2.7 UKF

Filter structure, measurement model and noise: [part G](#part-g-controller-ukf)
(2026-11-20). Before that, part A §A.5.5 and part H.

- The limits at UKF `+0x798..+0x7a0`, `+0x784` and `+0x864` come from
  config and were not read.
- The controller IMU feed path is in part A §A.5.3 step 8.

#### B.2.8 Association: project, cull, sort (`FUN_1800dd5d0` → `FUN_1800daff0`) [sure unless tagged]

Association is projection-driven, not nearest-neighbour from blobs.
For one candidate pose:

1. **Visibility per LED** (`FUN_180103090`, max 24 LEDs, bitmask at pose
   `+0x68`). The LED's view direction is the unit vector from the LED to
   the camera; the angle is acos(normal · view) in **degrees** (0 = LED
   faces the camera), stored at pose `+0x51c + 4·L`. The LED is visible
   when it projects inside the image (`FUN_180046240`) **and** the angle is
   **< the limit**. There is no halving: the limit is compared directly with
   this angle. Limit per association mode (`param_3` of `FUN_1800daff0`):

   | Mode | Limit | Source |
   |---|---|---|
   | 0 (boot, refine) | model cone, 170° (rev 1–3) or 180° (rev 4–7) | boot obj `+0x660` |
   | 1 (multi-camera accept), 2 | **174°** | `0x1803199cc`; flag `0x1803517c2` = 1 in this build |
   | 3 | cone × 0.80 | `0x180319810` |

   The ×0.85 for mode 1 (`0x180319814`) exists but is overridden by the
   flag. Mode 2 caller not traced.
2. **Self-occlusion cull**, only when the pose depth (translation z) is
   ≥ **120 mm** (`0x1803199a8`): LED i is dropped when another visible LED
   j projects within **3 px** in both x and y (`0x18031a500`, a box, not a
   circle) and i is more than **10** (mm [likely]) deeper than j
   (`0x18031a520`).
3. **Order**: the visible LEDs are sorted by the view angle at `+0x51c`,
   smallest first (most face-on first). The order is at boot obj `+0x1958`.

The view angle stored at `+0x51c` is in degrees. The LedSync metric at
`+0x15bc` (§B.2.2) is this angle [likely], so LedSync's `metric·0.5 ≤ 45`
means "LED faces the camera within 90°".

#### B.2.9 Boot (`FUN_1800d8b60`) [sure unless tagged]

Called from `FUN_1800d5070` per camera. `param_5` = "boot while tracking".

```c
if (n_blobs < MININLIERS_FOR_ORIENTATIONBOOT /*4*/) fail;
ransac.max = in_track ? BOOTINTRACK_MAXSAMPLES /*50*/ : RANSACMAXSAMPLES /*200*/;
R0 = (USEGRAVITYBOOT && imu_quat(+0x3380) != 0) ? R(imu_quat) : I;   // mode 7 / 6
ransac(blobs, model, R0);                      // FUN_1800c3aa0, FUN_1800d0950
if (inliers <= 3) fail;
associate(pose, mode 0);                       // §B.2.8
count_inliers(pose);                           // FUN_1800e0690
pothole = POTHOLETHRESHOLD;
if (SCALEDPOTHOLE)
    pothole = MIN(pothole / (|t_mm| * 0.001), pothole * 10);   // 1/metres, cap 10x
r = (inliers < 5) ? refine(pose)                               // FUN_1800d9de0
                  : refine_gated(pose, pothole * 1.5);          // FUN_1800d9d50
if (inliers < MININLIERSFORBOOT /*7*/) {
    if (inliers >= MULTICAMSTRICTGRAVITY /*4*/) keep as multi-cam candidate (+0x46a0);
    ok_g = gravity_ok(pose);       // FUN_1800e1f50
    ok_o = orientation_ok(pose);   // FUN_1800e20e0
    accept only if (ok_g && inliers >= STRICTGRAVITY /*5*/)
                || (ok_o && inliers >= ORIENTATIONBOOT /*4*/);
}
```

- The RANSAC itself (sampler, P3P, score, setup): §B.2.19.
- **Gravity check** (`FUN_1800e1f50`): angle in degrees (`·180/π`)
  between measured gravity (`+0x3310`, valid flag `+0x3334`, set by
  `FUN_1800d4b90`'s IMU path) and the model's gravity under the candidate
  rotation, scaled per axis by `+0x3300`. Limit 3° (`+0x624`), or 6°
  (`+0x62c`) when the pose is closer than **150 mm**.
- **Orientation check** (`FUN_1800e20e0`): angle to the IMU orientation
  prior (`+0x33c0`, valid flag `+0x3360`). Limit 15° (`+0x628`), or 30°
  (`+0x630`) closer than 150 mm. Only used when both prior ages `+0x35e0`/`+0x35e4`
  are < **10** (`0x180319954`; init 99999.9, incremented by the boot entry's
  dt argument; unit [guess] seconds). Also fails at distance ≥ **500 mm**
  (`0x1803199f8`) [likely].
- The four limits are constants written by the boot-object constructor
  (code at `0x1800d23e3`): gravity **3°** (`+0x624`), orientation **15°**
  (`+0x628`), gravity close-range **6°** (`+0x62c`), orientation
  close-range **30°** (`+0x630`). Config cannot change them.
- `FUN_1800d5070` then drops candidates whose score (`+0x1a90`) exceeds
  **0.34** (`0x1803517b0`) and sets the "close" flag at 100/150 mm
  (`0x1803199a0`/`0x1803199c0`), see §B.2.6.

#### B.2.10 Refine (`FUN_1800d9de0`) [sure unless tagged]

Steps:

1. `FUN_1800e0130` prepares the residual set; the pose is inverted
   (camera ← model).
2. Weighted Levenberg–Marquardt solve, `FUN_1800f7970`
   (RTTI: `RnD::WeightedLMSolver<float, RnD::PoseCost, 6>`; the multi-camera
   variant uses `RnD::PoseCostMultiCam`). Details below.
3. Re-associate (`FUN_1800ddb30` → §B.2.8, then `FUN_1800ddb90`) with the
   SCALED pothole gate; re-score (`FUN_1800e0690`, §B.2.12).
4. Fail code 3 if fewer than 3 matched LEDs; fail code 4 if `FUN_1800d89f0`
   rejects the pose.

`FUN_1800d9d50` wraps this: first the gated match (`FUN_1800df5b0`, §B.2.11)
with radius `pothole·1.5`; fewer than 3 matches → fail code 2.

**Residual** (`FUN_180105ed0`, `RnD::PoseCost`), 8 LEDs per AVX batch:

- transform each matched model LED by the pose, perspective-divide, apply
  the full camera distortion (`FUN_1800c08a0`, the model of §B.1.5);
- residual = projected pixel − blob centre, two scalars (x, y) per LED,
  in **pixels**; `|r|` per LED kept at cost `+0xb0` for the robustifier;
- an LED's residual is forced to 0 when its camera-frame depth is 0 or its
  normalised radius² x²+y² exceeds **200** (`0x18031a560`).

**Cost** (`FUN_1800fc710`): Σ wᵢ·rᵢ² over all residual scalars.

**Robust weights** (`RnD::Robustifier`, chosen by the solver's last
argument; objects at `0x180351938…0x180351960`, vtable slot 0 is the weight
function). `k = c · s`, `s` = robust scale:

| Index | Object | c | Weight w(|r|) | Function |
|---|---|---|---|---|
| 0 | `0x180351948` | — | 1 (plain least squares) | `FUN_180109f20` |
| 1 | `0x180351960` | 1.0 | 1 if \|r\| ≤ k, else k/\|r\| (Huber) | `FUN_180109fb0` |
| 2 | `0x180351938` | 1.345 | 1 if \|r\| ≤ k, else 0 (hard cut) | `FUN_18010a210` |
| 3 | `0x180351950` | 4.6851 | (1 − (r/k)²)² if \|r\| < k, else 0 (Tukey) | `FUN_18010a320` |

- The boot refine (`FUN_1800d9de0`) and `FUN_1800dd2f0` pass
  `inliers ≠ 3`: **exactly 3 LEDs → no kernel, otherwise Huber, c = 1.0**.
  The multi-camera call (`FUN_1800e2a00` at `0x1800e3a3e`) passes a
  register value; its kernel is [unknown].
- Scale `s` (`FUN_1801086c0` → `FUN_1801046e0`): **1.483 × median |r|**
  (`0x180319870`, the MAD factor) [likely], floored at cost `+0x18`; with
  flag `+0x1c` it can only shrink across iterations. Kernel weights for
  outer values (index 1/3 "else" branches) are [likely] as in the table.
- Weights are recomputed after every residual evaluation (IRLS).

**LM loop** (`FUN_1800f9a80`, init `FUN_1800f9860`):

```c
if (n_residuals < 6 && !allow_underdetermined) return status 7;
mu = 1e4;  nu = 2;            // 0x461c4000 (two fields); 0x1803198a8
for (it = 0; it < max_iter /*250*/; it++) {
    if (cost <= cost_tol) return 1;
    solve (JᵀWJ + mu·D) dx = -JᵀWr          // 6x6, solver object vtable +0x10
    for (try = 0; try < max_tries /*100*/ [likely]; try++) {
        x' = x ⊕ dx;   // rotation: exp-map of dx[0..2] (FUN_180104e30, eps 0.001); translation added
        c' = cost(x');  if (!finite(c')) abort;
        rho = (cost - c') / predicted_decrease;
        if (gain_small) return 5;
        if (|dx|² <= eps·|x|²) return 4;        // eps 1e-8 [likely]
        if (c' < cost) { accept; mu *= max(1/3, 1 - (2·rho - 1)^3); nu = 2; break; }
        mu *= nu;  nu *= 2;                     // Nielsen update
    }
    if (tries exhausted) return 3 or 6;
}
return 2;   // max iterations
```

Constants: 1/3 `0x1803197d8`, exponent 3.0 `0x1803198b8` (double), 2.0
`0x1803198a8`. The 0.25 passed in the option block (`0x1803197c4`) is
[unknown]: candidate the minimum robust scale `+0x18`.

#### B.2.11 Gated match (`FUN_1800df5b0`) [sure unless tagged]

Arguments: gate radius R (pixels) and an optional second radius
(default FLT_MAX); both are squared before use.

1. Reset per LED: best distance² 9999.9 at pose `+0x2b8 + 16·L`, blob
   `+0x43c + 4·L` = −1. `FUN_1800c0420` builds a spatial index of the blobs.
2. Walk the visible LEDs in view-angle order (§B.2.8). For each, take blobs
   nearest-first (`FUN_1800f0ee0`, returns distance²) — at most **7**
   candidates, stop at distance² > R². Score each candidate with
   `FUN_1800f0a50` (via `FUN_1800f1ce0`) and keep the lowest score whose
   support count is at least `need` (`need` = 2 when depth < 150 mm and
   flags `+0x4ad`, `+0x4ae` are set) [likely for "support"].
3. `FUN_1800f17d0` records the match.
4. `FUN_1800f1340` cleans up, called three times (modes 0, 1, 2):
   - **One blob, two LEDs**: both LEDs are compared on the rank short at
     `+0xe` of their match record and on distance; the loser is unmatched,
     or both when neither dominates.
   - **Outlier cut**: the reference LED is the matched LED with the highest
     rank and smallest distance product. Any other LED is unmatched when
     its pixel distance > f × the reference's **and** > **3 px**
     (`0x180319910`). f = **1.5** (`0x180319874`) at depth ≥ 150 mm,
     else MIN(25, 1.5·4·150/z_mm) (`0x180319928` = 4, `0x18031997c` = 25).
5. Returns the match count.

Details (2026-11-20, `FUN_1800df5b0`, `FUN_1800f0a50`, `FUN_1800f0cc0`,
`FUN_1800f1ce0`, `FUN_1800f17d0`, `FUN_1800f1340`) [sure unless tagged]:

- Arguments: support radius S (4th, xmm3) and gate R (5th). Track step:
  first call S = SEARCHRADIUS·scale, R = 3·S·(dt / 16.67 ms); second call
  S = R = 1.5·POTHOLE·scale (§B.2.16).
- `need` = 2, or 3 when the pose's flag `+0x4ad` is set; back to 2 when
  `+0x4ae` is also set and the depth is < 150 mm. `+0x4ad` is the flag
  `FUN_1800d89f0` gets as "close" [likely].
- **Support score** of candidate blob b for LED i (`FUN_1800f0a50`):
  k = clamp(visible − 2, 2, 5). Walk the other visible LEDs j by
  predicted-pixel distance to i, nearest first, at most 7. For each, q =
  b + (p_j − p_i); m = smallest squared distance from q to a blob other
  than b, starting at S² (`FUN_1800f1ce0`). m < S² counts as found: add m;
  stop at k found. Missing ones add (k − found)·S²/2. Score = sum / found
  (the sum when none). Lower is better; the found count is the rank.
- **Pass 1**: every visible LED in view-angle order; candidates = up to 7
  nearest blobs within R (blobs other LEDs took are not excluded). Keep
  the lowest score with rank ≥ need. Record (`FUN_1800f17d0`): blob, pixel
  distance |p − blob| (`+0x2bc`), score (`+0x2b8`), rank (`+0x2c6`).
  Cleanup.
- **Pass 2**: anchors = up to 5 matched LEDs in order of rising distance.
  Every visible unmatched LED re-scores up to 7 nearest blobs not taken by
  a matched LED, the support coming from the anchors first, then from the
  nearest other LEDs (`FUN_1800f0cc0`, `FUN_1800f0720`). Cleanup.
- **Pass 3**: mean pixel distance of the matched LEDs; each visible
  unmatched LED takes its nearest blob not taken before this pass when the
  squared distance is < (1.5·mean)². Taken blobs are not updated within
  the pass. Cleanup.
- **Cleanup** (`FUN_1800f1340`, the same in all three calls): for two
  matched LEDs i, j on one blob: when rank_j ≤ rank_i and j is no better
  in score and distance, j loses; else when rank_j < rank_i, or i is better
  in score or distance, both lose; else i loses. Then the outlier cut of
  step 4, the reference being the matched LED with rank ≥ the best so far
  and the smallest distance·score.
- Our port: `match_sony` in `userspace/lib/psvr2_led_solver.c` (setting
  `optical_sony_match`, default on).

#### B.2.12 Pose score (`FUN_1800e0690`) [sure]

Not a count: an **MSAC** score. For each matched LED, d = pixel distance
projection↔blob (stored at pose `+0x2bc + 16·L`, −1 when unmatched).
With T = pothole threshold (§B.2.9, scaled by 1/distance):
score += d² − T when d² ≤ T, else 0. Pose `+0x4a0` = score / T, so it runs
from −n (perfect) to 0. So POTHOLETHRESHOLD is compared with a **squared**
pixel distance [likely px²].

#### B.2.13 Multi-camera boot (`FUN_1800e2a00`) [sure unless tagged]

Runs when the single-camera boot found nothing (flag `0x1803517c1` = 1).
It is an **orientation-prior boot**: rotation comes from the IMU, only
position is searched.

1. Needs a valid IMU orientation prior (`+0x3360`); otherwise returns −1.
2. Collect this camera's blobs (`FUN_1800e2330`, radius 5.0 `0x180319938`,
   params 4.0 / 3.0). With more than 10 blobs, the first 10 slots are a
   random partial shuffle (RNG `+0x3600`, the Mersenne Twister of §B.2.5).
3. For blob pairs × **LED pairs** from the table at `0x180319380` (22
   entries, read as (t[i], t[i^1])): (3,4), (5,6), (7,8), (8,9), (9,10),
   (10,11), (11,12), (12,13), (13,14), (0,16), (16,15), each in both orders.
   Skip a pair unless both LED normals, rotated by the prior, face the
   camera. The table indexes the model arrays directly for every model
   (no per-model remap; `FUN_1800d3ae0` loads LEDs in stored order), so
   on the 20-LED rev 1 the pairs (0,16), (16,15) are not the back trio
   (0, 18, 19) as on the 17-LED models [sure]. `FUN_1800d1240` solves the translation from the two
   blob↔LED rays with the rotation fixed (2-point). Candidates go to the
   list at `+0x3fd8` (0x30-byte entries).
4. Score each candidate in **all** cameras (`FUN_1800e49b0`): matches m,
   error e, visible v. Keep the best (≥ 3 matches in the own camera,
   m ≥ 4, lowest e).
5. **Accept at once** when m ≥ 4 and m = v; or (flag `0x1803517d2` = 1)
   m > 4, m = v and e < **0.2** (`0x1803197c0`); or m > 8, v − m < 3 and
   e < 0.2.
6. Otherwise refine the best candidate with the multi-camera LM
   (`RnD::PoseCostMultiCam`, §B.2.10; flag `0x1803517d9` = 1), re-score,
   and accept when m > 5 and e < **0.1** (`0x180319794`).
7. On accept: build the pose (`FUN_1800d1aa0`), check it with
   `FUN_1800d95a0` (ratio 0.7, `0x18031980c`), associate in mode 1 (174°
   cone), then `FUN_1800db5f0`, `FUN_1800ce970`.

The unit of e is [unknown]; 0.1/0.2 suggest a normalised score, not pixels.

#### B.2.14 Blobs per camera and the two controllers [sure unless tagged]

There is **no split of blobs between controllers** before solving
(`FUN_18004b0a0`):

1. Per camera, the blob list (stride 0x168 in the IR frame) is mapped
   through the camera model (`FUN_1800a5310` → `FUN_18009eb50`, model at
   `+0x10340 + cam·0x1c0`) [likely: undistort to ideal pixels].
2. `FUN_1800a0480` copies it to that camera's manager
   (`DAT_180616720[cam] + 0x18160`).
3. `FUN_1800a4fb0` drops blobs outside the camera's active window: origin
   `+0x18084`/`+0x18088` (or `+0x1808c`/`+0x18090` in the alternate
   mode), size from the boot object's `+0x6c0`/`+0x6c4` plus 0.5.
4. `FUN_1800a56b0` then runs each controller's boot/track
   (objects of 0x4ce0 bytes) on **all** remaining blobs of that camera.

So the two controllers are kept apart by LED-sync timing (each lights in
its own frames, `controllers.md` part C) and by the pose-driven association, not by
a blob partition [likely]. `FUN_1800a5e30` runs after all controllers:
it is no conflict resolution. It builds (|t|², index) per controller (the
tracked pose `+0x1b20`, or the predicted pose when only predicted, flag
`+0x9200`) and sorts it (`FUN_1800ad2b0`, std::sort) into the order at
`+0x15c44`: the next frame processes the nearest controller first. With
no blob claiming the order only matters for shared per-camera state.
Our driver lets a tracked hand claim its blobs first (own choice).

`FUN_1800d4b90` (before each controller's `FUN_1800d5070`): per LED, the
blob whose `link` word (camera blob store `+0x88`, not the tracker list: LED in byte 0, controller id
`+0x604` in byte 1) names it, into `+0x1988`. Nothing in the tracker reads
`+0x1988` (only init/free and an overlay) [likely: debug].

Blob record arrays (per camera manager): `+0x50` x,y; `+0x70`, `+0x90`,
`+0xf0`, `+0x110` f32 [unknown]; `+0xb0` flag byte; `+0xd0` weight = 1.0;
`+0x130` u16; `+0x150` original index.

#### B.2.16 Track step and pose check (`FUN_1800d5070`, `FUN_1800db690`, `FUN_1800db980`, `FUN_1800d89f0`, `FUN_1800e1d50`) [sure unless tagged]

- `FUN_1800d5070` per camera and frame: when the controller is tracked,
  `FUN_1800db690` runs the state's track solver (state 0:
  `FUN_1800db980`). The boot (§B.2.9) can still run; a boot result replaces
  the track result when the track's cross-camera support
  (`FUN_1800e27a0`) is below **0.5** (`0x1803517ac`) and the boot has 2+
  more inliers, a score lower by 2, or lies nearer the last pose [likely
  for the exact combination]. Boot candidates with score > 0.34 are
  dropped. The status string is "Boot", "T" or "-".
- State 0 track (`FUN_1800db980`): predict (`FUN_1800da320`); blob set by
  depth (thresholds 100/200/300 mm, compared squared); associate mode 0;
  gated match + refine (`FUN_1800d9d50`) with S = SEARCHRADIUS·scale and
  R = 3·S·dt / (1/60 s); gated match with S = R = 1.5·POTHOLE·scale,
  < 3 matches → fail (code 8); `FUN_1800dcf00` (the few-LED solve, §B.2.17,
  else refine); pose check.
- **Pose check** (`FUN_1800d89f0`), in this order:
  1. gravity: when the prior flag `+0x3360` is set and its age `+0x35e0`
     is < **10** s: the measured gravity (`+0x3310`) rotated by the pose,
     dotted with the expected one (`+0x3300`), must be > cos **20°**
     (`0x180319880` = 0.349 rad), else code 2;
  2. orientation (`FUN_1800e1d50`, limit **30°** `0x18031998c`): angle
     between the pose rotation and the IMU orientation prior (`+0x33c0`);
     skipped when either prior age (`+0x35e0`, `+0x35e4`) is ≥ 5 s or the
     rate `+0x33d0` is ≥ π rad/s; else code 1;
  3. depth: model LED 0 in the camera frame must be ≤ **1500** mm
     (`+0x68c`) and, unless the "close" argument is set, ≥ **5** mm
     (`+0x688`); both written by `FUN_1800d2360`; else code 3.
- Our port: `psvr2_led_track_ex` and `psvr2_led_pose_check`; the driver
  passes gravity from the fused IMU orientation and the prediction as the
  orientation prior. Our joint 4-camera solve replaces Sony's per-camera
  trackers. The cross-camera support is `psvr2_led_support`; the driver
  runs the track/boot arbitration on it (`OpticalPose::SolveHand`).

#### B.2.17 Few-LED solve (`FUN_1800dcf00`, `FUN_1800e0d20`, `FUN_1800d1240`) [sure unless tagged]

- `FUN_1800dcf00` enters the few-LED mode (`+0x4c98` = 1, counter
  `+0x4c9c` = 0) when the match count (`+0x49c`) is ≤ **3**, the IMU
  orientation flag (`a1[822]`) is set and the pose is < **700** mm away;
  else it refines (`FUN_1800dd2f0`). The mode **sticks**: every later
  frame solves with the IMU rotation (pairs over all matched LEDs) until
  the counter, raised on frames with ≥ **5** matches (never reset in the
  mode), passes **15**, or the IMU flag goes. With 1 match and the consecutive-P1P counter
  (`+0x5dc`) > **5** it gives up. Success: method `+0x5e0` = 2 (P2P,
  counter reset) or 3 (P1P, counter + 1; at 5+ the pose z is replaced from
  `a1[529]` [unclear what that vector is]).
- `FUN_1800e0d20`: rotation = the IMU quaternion `a1[828]`. LEDs used:
  the tracker's matched set (`+0x48` bit mask, `+0x43c` blob indices),
  minus blobs whose box (centre ± size·π/2) leaves the image or whose
  normalised position is NaN or out of range.
  - 1 LED (only when the caller's flag is set, it always is): t = the
    point on the blob ray (x, y, 1) nearest R·p + t_pred, minus R·p. No
    distance gate.
  - 2+ LEDs: for each pair (a, b) whose normalised positions differ by
    more than √0.001, D = R(p_b − p_a); 1/z_b = −(w·u)/|u|² with
    u = D.xy − D.z·r_a, w = r_a − r_b; z_a = z_b − D.z;
    t₁ = z_a(r_a, 1) − R p_a. `FUN_1800d1240` is the same with a and b
    swapped (t₂). Each tᵢ within **100** mm of the prediction adds to
    both LEDs' sums and counts. Pair cost |t₁ − t₂|² / |p_a − p_b| when
    both count, else **9999**, added to both LEDs. Costs floored at
    **0.01**; result = Σ (S/cost_l / n_l / W) · sum_l with S = Σ cost,
    W = Σ S/cost_l over LEDs with n_l > 0. None counted → fail.
- Sony's gated match (§B.2.11) needs 2 supporting neighbours per LED, so 1
  or 2 isolated blobs never reach this solve in state 0; it acts when a
  cleanup or pass 3 leaves ≤ 3 matches, on a patch of 3 neighbouring LEDs,
  or in states 1–3 via the nearest-blob fallback (§B.2.18).
- State 0 order (`FUN_1800db980`): `FUN_1800d9d50` (gated match + refine
  when ≥ 3; else it clears the valid flag, code 7, and the step goes
  on from the prediction), then the pothole-gate match, < 3 → fail
  (code 8), then `FUN_1800dcf00`. So state 0 never solves 1–2 LEDs.
- Our port: `few_led_solve` via `few_or_refine` (`cfg.few_led`, default
  on; the sticky mode and P1P count in `struct psvr2_track_mem`, kept by
  the caller of `psvr2_led_track_state`; `psvr2_led_track_ex` has no
  memory and decides per frame), pairs pooled over all cameras (Sony: per camera); rays
  only in different cameras use the least-squares intersection, gated at
  100 mm. Border drop: blobs carry the LD major semi-axis as the size
  (tracker list `+0x28` ← store `+0x48`; the same field is the major
  in the `+0x48` major/minor ratio test of `FUN_1800e2330`) [likely].
- `FUN_1800e2330` (the per-frame blob filter, by camera distance of the
  prediction: ≤ 100 mm {8.5, flag 0}, ≤ 200 {8.5, 1}, ≤ 300 {3.5, 1},
  else {3.0, 0}; tables `0x180318b88/b60/bd8/bb8`): keeps a blob when its
  weight (store `+0xe8`) is ≥ 3 (2 when the store byte `+0x160` is set,
  or the major ≥ 32/π), store flag byte `+0xa8` bit 2 clear, bit 1 clear
  when the table flag is set, and with bit 0 set major/minor ≤ the table
  ratio. Not ported: the flag byte's source bits were not traced (it is
  not the LD `flags` word as is) [open].
- Test (`test_led_solver`, occlusion block): a patch of 1–3 neighbouring
  LEDs in the middle of one camera, IMU 1° off, through
  `psvr2_led_track_state`. 3-LED patches within 40 mm: Sony match
  34–59 %, our matcher (`LED_SONY=0`) 80–91 %; ≤ 1 pose off by more.
  1–2 LEDs there stay in state 0 and fail, as in Sony. Then all LEDs
  back for 17 frames: P2P for 15, the full refine on the 16th, in
  92–100 % of runs. Two LEDs ~6 px apart at 600 mm with 0.3 px noise give
  20–35 mm depth error from the geometry alone.

#### B.2.18 Close-range and edge states (`FUN_1800db690`, `FUN_1800dbda0`, `FUN_1800dc210`, `FUN_1800dc680`, `FUN_1800db420`, `FUN_1800ddb90`, `FUN_1800f1950`, `FUN_1800e4ec0`) [sure unless tagged]

Corrects §B.1.8: "close" there is the IMU orientation flag `a1[822]` (as
in §B.2.17), d = |translation| in the camera frame (controller origin),
the match count in state 3 is `+0x49c` and must be > **5** to leave.

- State 0 enters state 3 before solving when `FUN_1800db420` ≤ **3**: LEDs
  visible at association mode 3 (cone × 0.8) whose prediction lies
  ≥ **20** px inside the image rectangle and within **1.2391838·fx** of
  the principal point (camera model `+0x6b0` fx, `+0x6b8` cx, cy).
- States 1–3 (one solver each, same shape): association mode 2 (state 1,
  174°), 0 (state 2, model cone) or 1 (state 3, 174°); one gated match
  (§B.2.11) with gate = support radius = min(SEARCHRADIUS·scale,
  10·SEARCHRADIUS) (`+0x65c`, scaled when `+0x64a`; 10 at
  `0x180319954`), no dt scaling, no refine before it. Under 3 matches:
  association mode 0 and `FUN_1800ddb90`: each LED takes its nearest blob
  within the same gate (`FUN_1800f32c0`) and remembers the second
  (`FUN_1800f35c0`); `FUN_1800f1950` then splits shared blobs: a
  squared-distance gap ≥ **400** drops the farther LED; else, when
  neither LED was matched in the previous frame (`+0x2128` mask, flag
  `+0x258c`, per-LED bit `+0x58`), the later one (in the view-angle
  order) drops; else, both with a second blob, the one whose second is
  nearer moves to it; else the one not matched last frame drops (both
  were: the farther).
- Then `FUN_1800e4ec0` (flag `0x1803517c3` = 1): a matched LED with view
  angle (`+0x51c`) ≥ **140°**, ≤ **300** mm from the camera and a blob
  angle ≥ 0 (blob list `+0x88`) is dropped when the blob ellipse axis is
  more than **25°** from perpendicular to the projected LED normal (camera
  Jacobian `FUN_180103580`). 0 left → fail.
- Then `FUN_1800dcf00` (few-LED, §B.2.17, else `FUN_1800dd2f0`: LM refine
  with ≥ 3 matches, method 1). No pose check (`FUN_1800d89f0`) in these
  states; the 25° orientation flag test after it is as in state 0.
- Our port: `psvr2_led_track_state` (`struct psvr2_track_mem` per hand
  in the driver, `HandState::mem`, reset by `psvr2_led_track_mem_boot`
  after a boot or a boot that replaces the track), our joint solve: d =
  nearest camera that matched, the inner count summed over cameras; last
  frame's matched LEDs per camera for the shared-blob split. Not
  modelled: Sony's state 4
  (re-boot at once) — the driver keeps its 6-miss tolerance and the next
  call runs as state 0. Driver settings `optical_sony_states`,
  `optical_edge_state` (both default true); blobs carry the LD ellipse
  angle for the grazing check.
- Test (`test_led_solver`, `run_close`): controller 90–170 mm in front of
  a camera: 65–85 % right (5 mm / 4°) vs 15–43 % for the normal step,
  ≤ 1 gross. Controller near an image corner, one camera: the edge state
  is right 57–68 % vs 81–84 % for the normal step, 3–7 % gross vs
  2–5 %; blobs cut by the border are not simulated.

#### B.2.19 RANSAC boot (`FUN_1800c2960`, `FUN_1800c4940`, `FUN_1800c2080`, `FUN_1800c52f0`, `FUN_1800c58f0`, `FUN_1800c5ac0`, `FUN_1800c62c0`, `FUN_1800c6df0`, `FUN_1800c3320`, `FUN_1800c18a0`) [sure unless tagged]

The RANSAC object is boot obj `+0x870`. `FUN_1800d3360` sets it up with
`FUN_1800c18a0` (decompiling it crashes IDA 9.5; read from the
disassembly):

| Off | Value | Source |
|---|---|---|
| `+0x00`, `+0x04` | iteration cap | RANSACMAXSAMPLES (`+0x63c`); `FUN_1800d8b60` then sets 50/200 (§B.2.9) |
| `+0x08` | 12 | [unknown] |
| `+0x0c` | adaptive flag = **0** | so §B.2.5's bound never runs here |
| `+0x10` | RANSACRINGBOOTRATIO 0.5 | `+0x690` |
| `+0x14` | confidence 0.99 | |
| `+0x18`, `+0x1c` | `+0x638` | [unknown] |
| `+0x20` | inlier threshold, px | INLIERTHRESHOLD (`+0x634`); × min(1/d(m), 10) when SCALEDINLIER (`+0x649`, 0 in all models) |
| `+0x24`, `+0x28`, `+0x2c` | **6.0**, **200.0**, **3.0** | close threshold, distance, blob size (step 5) |
| `+0xf7c` | ring-stage cap = (int)(`+0x00` × `+0x10`) = **100** | computed before the boot-in-track cap |

`FUN_1800d8b60` reseeds the Mersenne Twister at `+0x1f8` with `+0x60c`
= **0** on every boot, so a boot is deterministic per blob set. Mode
`+0xf10` is **7** with an IMU quaternion (`+0x3380`, USEGRAVITYBOOT), else
**6**; its rotation is at `+0xf20`. The hypothesis callback `+0xf08` is
`RnD::FullBootConstraints` (vtable `0x1803191e0`, boot obj `+0x4ca0`):
slot 1 = `FUN_1800d89f0(pose, close = 1) == 0`, the pose check of §B.2.16.

Setup (`FUN_1800c3aa0`):

- Valid blobs (`FUN_1800c3c90`): those that undistort; failures store
  100000.
- Blob neighbours (`FUN_1800c62c0`): the min(4, valid − 1) nearest valid
  blobs in pixels (mode 3 instead takes one per 90° direction quadrant).
- LED neighbours (`FUN_1800c6df0`): the 4 nearest LEDs in the model, all
  LEDs.
- Sampled LEDs (`FUN_1800c3320`): all LEDs in modes 6 and 7; the cut to
  LEDs within 95° of the camera under the mode rotation applies to mode 4
  only.
- LED rings: model block (§B.2.4) words **198** = ring count, **199 + k** =
  length of ring k, **231 + 32·k** = its LED indices (rev 1: 6 rings, rev
  2–6: 9, other: 10; the first ring repeats its first 3 LEDs at the end,
  so it closes). `FUN_1800d3ae0` copies them to boot obj `+0x19c0`
  (total length `+0x19e0`); `FUN_1800d8b60` points `+0xf70`/`+0xf78` at
  them.

Loop (`FUN_1800c2960`), until `+0x1f0` (samples) reaches the cap:

1. **Sample** (`FUN_1800c4940`, modes 6/7). Draws are
   `(int)(r·2⁻³²·n)` (`0x18031a2e0`).
   - *Ring stage*, while fewer than `+0xf7c` ring samples were taken:
     without a current blob chain, up to min(4·blobs, 20) tries of a blob
     and two of its neighbours; `FUN_1800c52f0` orders them as a chain
     (end, middle, end) through the vertex where the path turns by < 90°,
     ends so that `d_in.y·d_out.x < d_in.x·d_out.y`, and needs a fourth
     neighbour of the last blob continuing it (same turn, dot ≥ 0); chains
     already tried are skipped. Then the next 3-LED window of the rings
     (window i of a ring of n: i ≤ n − 4, then the next ring) that passes
     `FUN_1800c5ac0(…, 0)` is the sample. Windows used up: the chain is
     dropped and this sample comes from the next stage.
   - *Random stage*: an LED and two of its neighbours, a blob and two of
     its neighbours, all distinct (`FUN_1800c61d0`); `FUN_1800c58f0` swaps
     LEDs 1 and 2 when the blob triangle's turn (`d1.y·d2.x < d1.x·d2.y`)
     and the LED triangle's (`((L1−L0)×(L2−L1))·(n0+n1+n2) < 0`)
     disagree, then in mode 7 `FUN_1800c5ac0(…, 1)`; retried until one
     passes. A blob with a known LED (`+0x1980` map) replaces the drawn
     blob.
   - `FUN_1800c5ac0` (mode 7 only, else pass): the image vector blob 0 →
     blob 2 (a3 = 0) or → middle of blobs 1, 2 (a3 = 1) within 90° of the
     LED vector, both in an image frame aligned to gravity (`+0x1000`);
     when the mean ray is within 30° of gravity the LED centroid must face
     the camera instead.
2. The 3 blobs must be valid and their `+0x3504` value ≤ `+0x1070`
   [unknown field], else a fail counter (limit `+0x106c`).
3. P3P (`FUN_1800f43e0`) on the three rays → up to 4 poses.
4. Per pose: each sampled LED faces the camera (camera-frame normal · unit
   view ray < **0.08**) and lies within **95°** of the optical axis
   (`cosf(1.6580629)`); then the callback (pose check).
5. **Score** (`FUN_1800c2080`): threshold `+0x20`, or `+0x24` (6 px) when
   |t| < `+0x28` (200 mm), the mean `+0x3496` value of the 3 sampled
   blobs > `+0x2c` (3; blob size [likely]) and `+0xf14` = 0. Every LED
   (≤ 24) is projected with the full camera model; it counts when its
   depth ≠ 0 and normalised r² ≤ 200 (no facing test). Each LED takes its
   nearest blob with d² < thr²; a blob takes at most **2** LEDs, in LED
   order. cost = Σ min(d², thr²) / LED count.
6. Keep the pose with more inliers, or as many and a lower cost (the last
   3 bests are kept at `+0x30/+0x70/+0xb0`; `FUN_1800c7f00` uses them only
   when `+0x664` = 1, model word 5, which is 48).

Afterwards `FUN_1800d8b60` needs ≥ **4** inliers (`+0xa94`), then §B.2.9.

**Our port**: `psvr2_led_boot_ransac(cfg, cams, blobs, model, prior,
in_track, …)` — mode 7 when `prior->orient_valid`, else mode 6; every
hypothesis goes through `psvr2_led_pose_check(…, close = 1)`. Per camera:
RANSAC, our refine (`psvr2_led_track`, all cameras), the §B.2.9 acceptance
(`boot_accept`: ≥ MININLIERSFORBOOT, or gravity ≤ 3°/6° with ≥ 5, or
orientation ≤ 15°/30° nearer than 500 mm with ≥ 4); the candidate with
the most matches (then lowest rms) wins. Model rings come from
`scripts/sony-re/extract_led_header.py`. Ours, not Sony's:

- P3P by Grunert's quartic (Haralick et al. 1994) with numeric roots;
- the mode 7 direction test uses the full prior rotation, not image
  gravity (no near-gravity branch);
- blob size = LD major semi-axis;
- with `own_accept`, a candidate needs cross-camera support ≥ **0.75**
  (`psvr2_led_support`): without it, wrong boots (mostly a ring turned by
  one LED, ~20°, 11–31 matches at ~1 px) reached 1.5 % (orientation
  prior) and 10 % (no IMU) in the test; they score 0.23–0.78, right ones
  0.69–0.95;
- not ported: the `+0x3504` blob gate, the known-LED map, the early stop
  `FUN_1800c7cc0` (at `+0x1060` inliers).

Driver: `optical_ransac_boot` (default true) tries it first when the
remap is known — gravity only (mode 6, no heading), plus the current
heading (mode 7) while fixes are recent — with the sweep's 12/18-LED
floor; the heading sweep stays the fallback.

Test (`test_led_solver`, 200 poses per model, same blobs as the boot
test): orientation ~3° and gravity ~1° off (mode 7): **178–196** right,
0 wrong; gravity only (mode 6, the driver's case): **154–183** right,
0 wrong; no IMU (`LED_RANSAC=0`): 138–170 right, ≤ 1 wrong. Ring stage
off, rev 3: 139 and 90 right (the ring stage is most of the hit rate);
with the chain turn sign flipped: 158 and 91 (so the sign above holds in
our pixel frame). Left-hand model (`LED_MIRROR=1`,
`psvr2_led_model_mirror_x`: x negated, rings reversed as Sony does,
part C §C.2b.2), rev 3: 186 and 170 right, 0 wrong; mirrored
without reversing the rings: 159 and 124. About 6 ms per call (4
cameras, WSL). Our oriented
boot (`psvr2_led_boot_oriented`) stays better when the heading is known:
198–200 right.

#### B.2.15 Leftovers (item 2)

These no longer block a port:

| Gap | Where |
|---|---|
| Candidate score inside the gated match | answered, §B.2.11 |
| Track-vs-boot arbitration, cross-camera support | answered, §B.2.16 |
| Exact LM stop thresholds (cost tol, gain tol) | `FUN_1800f9a80` fields `+0x1c`, `+0x24`; set from the option block in `FUN_1800f7970` |
| The 0.25 in the option block | `FUN_1800f7970` (candidate: minimum robust scale `+0x18`) |
| Unit of the multi-camera error e | `FUN_1800e49b0` |
| Cross-controller cleanup | `FUN_1800a5e30` → `FUN_1800ad2b0` |
| Model word 5 (= 48) | consumer of `0x1805f4058` |

---

### B.3 LED-sync details

Phase enum (names at `0x180268d20`, 12-byte stride) [sure]: 0 INIT,
1 PRESCAN, 2 BROAD, 3 BG, 4 STABLE, 5 LED ALL OFF, 6 LED ALL ON.

#### B.3.1 Per-camera history window (`FUN_180194f70`) [sure]

Window length L by phase: PRESCAN 1, **BROAD 11**, **BG 20**, STABLE 1
(the same switch appears in `FUN_1801954f0`, `FUN_1801957d0` and
`FUN_180194f70`).

For one (controller, camera), it walks the newest L frame records
backwards. Each record is a 0x104-byte node in the ring at
`*(0x180362900 + (ctrl·5 + cam)·8)`; head at `+0x14`, link to the older
node via the short at `+10`.

Per frame k (0 = oldest, L−1 = newest) it takes:

- `w[k]`: the sync-LED blob weight. It is the float at
  `node + 0x24 + (ctrl·0x1f + cam)·4`, saved to
  `0x180362b04 + ctrl·0x1e8 + cam·0x50 + 4k`.
- `cnt[k]`: the blob count at `node + 0x10 + (ctrl·0x1f + cam)·4`.

```c
max = 0; min = FLT_MAX; short = false;
for k = L-1 down to 0:
    if (w[k] ≈ 0) { short = true; break; }      // history shorter than L
    if (cnt[k] > 0) { max = MAX(max, w[k]); min = MIN(min, w[k]); }
if (max ≈ 0 || min ≈ FLT_MAX) return 9;          // "NO MAX MIN"
if (max - min < 0.1f) return 10;                 // "NO DIF MAX MIN" (0x180319794)
if (short) return 4;
thr = (max + min) / 2;                            // stored at 0x180362c94
state[k] = (cnt[k] > 0 && w[k] > thr) ? 2 : 0;    // ctx +0x10 + 4k
return 1;
```

So **2 = "lit"**: the blob is brighter than the midpoint of the window's
range.

**Where w and cnt come from** (`FUN_1801999f0`, 2026-11-20) [sure]: the
node is a copy of a per-frame global block (`FUN_18019c500` copies
`0x1803629c0..`), whose w is `0x1803629d8 + (ctrl·31 + cam)·4` and cnt
`0x1803629c4 + …`. `FUN_1801999f0` writes them from the **pose
tracker's** per-camera LED → blob map (IR frame struct `+0x1438`, §B.2.1):
w = mean weight of the blobs matched to the sync LEDs whose blob state is
1 and whose view metric·0.5 ≤ 45, cnt = their number. When the controller
has no tracked pose in this frame (block `+0x10` clear) both are 0, so the
window is short (status 4). There is no pose-free detector: a dark sync
LED must still show as a dim matched blob. Sync LEDs are logical (model)
indices; the chosen camera and LEDs are kept while a candidate was found
less than 1 s ago (`0x180362e90`). **0 = "dark or absent"**. 1 is never written here.

Other status codes, stored per camera at `0x180362ac8`:

| Code | Set by | Meaning |
|---|---|---|
| 2 | `FUN_1801957d0` | camera not in use for this controller |
| 0xb | `FUN_1801957d0` | fewer than L frames in history |
| 5 | `FUN_1801957d0` | BG marker mismatch (below) |

#### B.3.2 BG check pattern `0x180260430` [sure]

The pattern is compared as raw memory against the state array
(`ctx + 0x10`, s32 per frame) in `FUN_1801957d0` and `FUN_180195a70`:

| Bytes | Compared with | Frames | Required states |
|---|---|---|---|
| 0x180260430..437 | ctx +0x10..+0x17 | 0, 1 | lit, dark |
| 0x180260438..43f | ctx +0x18..+0x1f | 2, 3 | dark, lit |
| 0x180260440..447 | ctx +0x54..+0x5b | 17, 18 | lit, dark |
| 0x180260448..44b | ctx +0x5c..+0x5f | 19 | lit |

The 32 bytes are four s32 pairs, not 32 frames. The "2 at byte 0, 12, 16
and 24" are frames 0, 3, 17 and 19 = lit.

- Frames 4..16 are **not checked**. They carry the brightness sweep.
- The marker is a framing signature that the controller emits at the start
  (lit, dark, dark, lit) and end (lit, dark, lit) of its 20-frame BG
  pattern [likely: meaning; sure: positions].
- Mismatch → camera status 5.

#### B.3.3 BG estimator (`FUN_1801954f0`) [sure]

Called as `FUN_1801954f0(ctx, max, 0)` from `FUN_1801957d0`, so
**a = window max, b = 0**, and floor = max/2.

```c
floor = (a + b) * 0.5f;                 // = max/2
num = den = 0;
for k = 4 .. L-4:                       // BG: k = 4..16 (13 frames)
    d = MAX(0, floor - w[k]);           // how far BELOW half-max
    num += k * d;  den += d;
if (num != 0 && den != 0) x[cam] = num / den;   // 0x180362af0[ctrl·0x7a + cam]
```

- The weights favour **dark** frames. x is the centre of the dark gap
  inside the sweep, in frame (= step) units. The exposure centre is
  x = 10 (`0x180319954`).
- `controllers.md` §C.5.3 says "samples above the floor": wrong. It is below.

#### B.3.4 BG accept and correction (`FUN_1801975e0`) [sure unless tagged]

1. Over the 4 cameras plus slot 4, keep only valid x (≠ −1). Pick the one
   closest to the last accepted x (`0x180362b00 + ctrl·0x1e8`). That
   camera index goes to `0x180362ac4`.
2. **Skip rule.** If the fit object already has ≥ 2 runs, a fit exists,
   and **|x − 10| > 1**, the sample is dropped. This is allowed up to 3
   times in a row (`0x180363110`). `controllers.md` §C.5.3 says "jumps by > 1 from the last
   accepted value": wrong. The reference is the centre 10.
3. Accept: `last_x = x`; `phase = x + acc` (`0x180362f38`, where `acc` is
   `0x180362ee0`). `phase` is what `FUN_180196ff0` pushes into the
   BG/STABLE fit.
4. Correction:

   ```c
   if (9 <= x && x <= 11) return;            // deadband ±1 step (0x18031994c, 0x180319968)
   acc -= (10 - x);
   send cmd 4, s32 = (int)((10 - x) * step_ns / 1000)   // = (10 - x)·step µs
   ```

   `step_ns = DAT_18036304c = step · 1000`. `step` = `DAT_180350ce4`, 105
   or 50 from the preset table `0x18026035c`.

   - In the first-STABLE path (flag `0x180362f24` set), the same command
     is sent.
   - Before that: if `|(10 − x)·step| ≥ 40` (`0x180350d08`), the BG/STABLE
     fit is reset (`FUN_180199650`).
   - Only for camera slot 4 (the combined result) and hand < 2.

#### B.3.5 Drift fit: values, units, scaling [sure unless tagged]

| Fit | Object | Value pushed | Value unit | Time unit |
|---|---|---|---|---|
| BROAD | `0x180362ee8` | `*puVar12` in `FUN_180198240`: the accumulated slot offset (`0x180362f10` += slot − 4) [likely] | BROAD slots | µs since first sample (host clock) [likely] |
| BG / STABLE | `0x180362ef8` | `x + acc` (`0x180362f38`) | BG steps | same |

- **No scaling** before the command: `FUN_18019bc00` reads the fit's slope
  (fit +0x10) and sends it raw as f32 in cmd 3, or cmd 5 with an s32
  offset.
- The slope's unit is therefore steps/µs (or slots/µs).
- libpad applies `cycle := (1 + slope)·cycle` (`controllers.md` §C.5.5). So
  Sony **under-corrects by a factor of `step`** (105 or 50) per round,
  compared with a dimensionless rate. The loop converges only because it
  repeats [likely]. Copy this behaviour as-is to match Sony.
- BROAD offset for the record (`FUN_180198240`):
  `(int)((slot−4)·step_ns·5/1000)/2` = (slot−4)·5·step/2 µs, matching `controllers.md` §C.5.2.

**Sign** [sure for the arithmetic, likely for the physics]:

- A positive slope makes `cycle` longer.
- A positive slope means the measured phase (x + acc) grows over time:
  the dark gap moves to later frames.

#### B.3.6 BROAD "3 consecutive" [likely]

- `FUN_180198240` keeps the last 3 decoded slots per controller
  (`0x1803628c0`, shift register).
- It shifts in the new slot only on the code path that reaches the accept
  test, after a decode attempt that produced a slot.
- Agreement of all 3 entries sets the accepted flag `0x180362f08`.
- So the count is **per decode attempt that reached the test**, not per
  frame. A frame whose decode failed earlier (status 3 etc.) does not
  shift.
- Whether slot = −1 is shifted in was not checked.
- GRID SKIP is `slot − 4 > 4` (only the high side can exceed it; slot ≤ 12).
  After 6 of them, BROAD resets: the fit is reset (`FUN_1801997c0`),
  `0x180362f10` = 0.

#### B.3.7 Sync mode selection (`controllers.md` §C.2.2) [sure]

`FUN_1801cbeb0` runs its block only while the pad's sync mode
(`+0x33e0 + 0x18`) is 0:

```c
fw = feature_0x20[+0x1c]   // u32 "main firmware version" (device record +0x1d0, FUN_1801d3490)
if (fw < 0x1006d || probe_tries > 3)          // tries at pad +0x3408
    set_sync_mode(2);                          // FUN_1801c1820
else {
    Set 0x80 "80 13 22 00…"; Get 0x81;
    mode = (reply[0] == 0x81) ? 2 : 0;
    if (mode != current) set_sync_mode(mode);
    probe_tries++;
}
```

`FUN_1801c1820` tables:

| Mode | Width | Step |
|---|---|---|
| 1 | 1000 (`0x180268dc0`) | 105 (`0x180268dc8`) |
| 2 | 500 (`0x180268dc4`) | 50 (`0x180268dcc`) |

Mode 0 keeps the reset values 1000/105.

So:

- `local_b4` is feature `0x20` **+0x1c** (main firmware version; `controllers.md` part B
  §B.2.7).
- **Every path ends in mode 2.** Old firmware gets mode 2 at once. A
  failing probe leaves mode 0 for up to 4 attempts, then mode 2.
- `FUN_1801c1820` has no other caller, so **mode 1 is never set** in this
  build. The 1000/105 timing is used only while mode = 0 (before or
  between probes).
- The `controllers.md` part C table row "Mode 1: firmware < 0x1006d or probe failed 4
  times" is wrong; it is mode 2.
- The dev driver's "libpad sync mode 1/2 stands in for preset" should
  expect mode 2, timing 500/50, in steady state.

#### B.3.8 Still unknown (item 3)

- The exact time unit of the fit's t: host µs is [likely]. Read the caller
  of `FUN_180196ff0` for `param_2`.
- Whether `FUN_180198240` shifts slot = −1 into the 3-history (lines before
  the shift).
- What `*puVar12` points to in `FUN_180198240` for the BROAD fit.

---
### B.4 Sony's driver-from-head transform

#### B.4.1 What Sony does [sure unless tagged]

The values are already in `steamvr-and-display.md` §B.5.4: `FUN_18019ee30`, built by
`FUN_180155000` → `FUN_1801556b0`. Restated as a pipeline:

1. EP `0x83` record → INS (`FUN_18017a5d0`). The pose is **wTi: world ←
   IMU rig**.
   - Position: `+0x10`, m; the first pose's position is subtracted once.
   - Rotation: `+0x1c`, read as **x, y, z, w**.
   - The INS world is **Y up**: gravity (0, −9.8, 0), `FUN_180175290`. No
     axis permutation anywhere between the record and SteamVR.
2. INS output at `t_query`:
   - `vecPosition = p` (m, world)
   - `qRotation = q` (world ← IMU rig)
   - `qWorldFromDriver = identity`
3. Constant IMU-rig → head:
   - `qDriverFromHeadRotation = (w,x,y,z) = (−0.5, −0.5, 0.5, 0.5)`
   - `vecDriverFromHeadTranslation = (−0.09255, 8e-9, 0.02080)` m, in IMU-rig
     axes.
4. SteamVR composes `world←head = (p, q) ∘ (t_dfh, q_dfh)`:
   - head origin = `p + R(q)·t_dfh`
   - head rotation = `q ⊗ q_dfh`

So **the SLAM pose is the IMU rig**, not the display centre or between the
eyes. The head frame is a fixed 120° axis permutation plus a 9.5 cm lever
arm away from it.

`R(q_dfh)` maps head axes into IMU-rig axes:

| | head +x | head +y | head +z |
|---|---|---|---|
| IMU rig | −y | +z | −x |

- Head right = IMU −y.
- Head up = IMU +z.
- Head back = IMU −x, so IMU +x points forward.

#### B.4.2 Check against `steamvr/driver_psvr2/src/pose_source.cpp` [sure: numeric check]

The Linux driver:

- reads the record quaternion as (w, x, y, z);
- maps it to `(q0, −q2, −q1, q3)`;
- left-multiplies Rz(+90°);
- maps position `(r2, r1, −r0)`;
- sets driver-from-head to identity.

With the record stored **x, y, z, w** (Sony's reading), this is exactly:

```
R_linux = M · R(q_slam) · R(q_dfh),   M = [[0,0,1],[0,1,0],[−1,0,0]]   (world yaw −90° about Y)
p_linux = M · p_slam
```

This was checked on random quaternions: the max error is 0. Reading the
record as w, x, y, z instead gives errors > 1. So:

- **Rotation:** the Linux remap plus roll correction is Sony's
  `q_dfh`, folded into `qRotation`, plus a harmless world yaw M. The
  open question in part A §A.8.1 is settled: the record is x, y, z, w,
  and the Linux code is right only because its "w, x, y, z" naming and its
  permutation cancel.
- **Translation: missing.** Sony puts the head origin at
  `p + R(q)·t_dfh`. Linux reports the IMU position.

  In the Linux head frame (`qDriverFromHead` = identity), the equivalent
  offset is `R(q_dfh)ᵀ·t_dfh = (0.0, +0.0208, +0.0926)` m: head origin
  2.1 cm above and 9.3 cm behind the IMU, OpenVR axes.

  Fix: set `vecDriverFromHeadTranslation = {0, 0.0208, 0.09255}`, keep
  `qDriverFromHeadRotation` identity. Without it, rotating the head moves
  the eye points on an arc about 9.5 cm off. This is visible as swimming
  on yaw and pitch [likely].
- The world yaw M only changes which way "forward" points at start.
  Sony's world has the same yaw as the raw SLAM map.

#### B.4.3 Still unknown (item 4)

Nothing blocks the fix. Sony's origin handling (subtract the first
position; rotation untouched) is optional.

---
### B.5 Relocaliser (low priority)

This is already covered: `room.md` §A.1, §A.2.2–2.3 and §B.3, plus `room.md` part B.
No new unknowns block it. In short:

- **Set `0x0a` payload:** header `0a 00 <u16 subcmd> <u16 0xd8> 00 00`,
  then always 0xd8 bytes (`FUN_180127fe0`, `FUN_1801283f0`).

  | Subcmd | Content | When |
  |---|---|---|
  | 4 | u32 1 | restart |
  | 3 | fixed tuning words | once after the first frame |
  | 1 | localiser result | every processed RP frame |

  The subcmd 1 result layout is `room.md` §A.2.3. Sends are paced at
  ≥ 20 ms with 5 retries (`FUN_1801adba0`).
- **RP packets (EP `0x8a`, 0xc8780 bytes):**
  - header: `"RP"`, u32 size at +4; the first 0x2c bytes are unused by the
    PC;
  - body: 162 features per camera pair (u16 keypoints, 128-byte
    descriptors, per-feature float blocks), a 640×640 8-bit image, a pose
    prior, and up to 2 anchors (`room.md` part B table).
- **Without it:**
  - only the **persistent map** is lost: re-anchoring a saved play area
    across sessions, and map recovery against a stored map;
  - SLAM tracking, the INS and loss recovery inside a session run on the
    headset;
  - the RP endpoint must still be drained (kernel already does).
  - [sure for the data flow; likely that in-session recovery does not
    need the PC]

---

### B.6 Extreme-Env blob modes (low priority)

**Nothing in a normal install sets them** [sure].

- Bytes `+0x33` / `+0x34` are `DAT_18035ba23` / `DAT_18035ba24` in the
  0x400-byte tracker-settings block `DAT_18035b9f0`.
- `FUN_18018a7e0` zero-fills that block and sets only a few defaults:
  `+0x01..+0x03` = 1, `+0x2e` = 1 (sync-LED count), `+0x36` = 1, plus geometry
  from `FUN_18005d160`.
- It then compares the block with the config manager's shared-memory copy
  (`FUN_18015cfd0`: shared memory `+0x8098`, mutex at
  config-manager `+9000`). If they differ, it writes the defaults there
  (`FUN_18015f110`).
- `FUN_18011b3e0` re-reads the shared copy and adopts it when it changes
  (`FUN_180192ae0`).
- So only an external process that writes that shared-memory block (a
  Sony tool or the PS VR2 app) can enable Extreme-Env. There is no
  SteamVR setting, registry key or file for it in this DLL.
- The same block holds the LED-sync phase hold (`+0x05`, `controllers.md` part C
  §B.3.2), a debug knob that pins controllers to one phase. Extreme-Env
  mode 3 is the only code in the DLL that writes it.

What an opt-in port would need (none of it is in the driver yet):

1. A setting standing in for `+0x34` (force) and `+0x33` (auto entry from
   Many Blob, and which of modes 2/3/4).
2. The two extra `0x09` blocks from `controllers.md` §C.2.6 (`Block09` in
   `optical_tracker.cpp` has only Normal and Many Blob).
3. The entry and exit rules of `controllers.md` §C.4.3: 5 flagged packets in a
   row to enter from Many Blob, 180 flag-free packets to leave.
4. In mode 3, hold every controller in PRESCAN (`LedSyncer::HoldPrescan`)
   and release it on the way out.

What blocks 0x08/0x0e = `0xff` (mode 4) and block 0x15 = 1/2 do inside the
headset is not known [guess: harder blob rejection for bright, cluttered
rooms]. Without hardware tests, Normal ↔ Many Blob is the safer default.

---

### B.2.20 Headset INS (done, not on hardware)

Ported Sony's host INS for the headset (part A §A.3.1, §A.5.1, §A.5.2):
bit-exact kernels `psvr2_ins_core.c` (quat_mul `0x1801805a0`, rotinc
`0x18017f2c0`, strapdown `0x18017f8e0`, predict `0x18017f500`, pose_ok
`0x18017bac0`, blend mode 1 `0x180181a50`, IMU calibration matrix
`0x1801733f0`; difftest `t_irtracker_inscore.c`, 0 mismatches), the
`FUN_18017cdb0` sequence in `psvr2_ins.c`, a buffered IIO reader in
libpsvr2 (`psvr2_imu_stream_*`), and the driver path in `pose_source.cpp`
(setting `headset_ins`). Test `test_ins.c` (part of `make check`).

Left: mode-0 adaptive blender (gains unknown), stillness and shock
switches (now §A.5.2 and §A.5.7), Sony's 3DoF tilt correction `FUN_180171400` (§A.5.7), the `+0x188`
good-frame gate; the VD type 12 layout is read from code only (the
driver rejects it if its diagonal is not within 20 % of the IIO scale).

---

## Part C: Optical pose into SteamVR

Sony `driver_playstation_vr2_orig.dll`, PE timestamp 1741940190. Addresses
are VAs, image base 0x180000000. Tags: **[sure]** read in code (or called
under Wine), **[likely]**, **[guess]**. Written from `FUN_1800af650`,
`FUN_1800b02a0`, `FUN_1800cbd70`, `FUN_180173fe0`, `FUN_18005ec00`,
`FUN_180189c70`, `FUN_180160230`, `FUN_18018aaf0` (IR thread) and the VD
dispatcher at `0x180170xxx` (`case 0xd`, decomp line ~171011).

Status: partial. Done: Q1 rig table, calibration composition and the
head chain `INS · A⁻¹ · B` with numbers (§C.2), type 13 layout and route
(§C.2.3–2.5), Q2 IMU offset and hand-1 mirror (§C.2b), the optical
measurement content, its innovation gate and the UKF config defaults
(§C.4), and the output/timeout path (§C.5). Open (§C.8): whether X is exactly
the rig frame (needs one HW check), which config entries are the
measurement noise, the solver's per-frame acceptance rule,
two-controller association, LD time offset.

### C.1 Driver summary

What the Linux driver must do, in order (all userspace):

1. Build the camera rig **from Sony's built-in table, not from camera 0's
   frame**. Camera 0 keeps its built-in pose; cameras 1–3 are camera 0's
   built-in pose composed with the calibrated extrinsics (§C.2). The rig
   frame is a headset-fixed frame in mm: +x = wearer's left, +y = up,
   +z = forward [likely, from the camera positions and the HW cover test].
2. The optical solve outputs a controller pose (position + quaternion)
   that the IR thread hands to the controller UKF as a full-pose
   measurement (§C.4).
3. Report the UKF pose, but **hold the last optical position** when no
   optical update has arrived for **0.3 s**; keep the rotation from the UKF.
   Report a status code 9 (optical fresh), 3 (position held) or 6
   (position from the fallback model), and `poseIsValid = 0` only when the
   tracker has no state at all (§C.5).
4. Apply the SteamVR tip/grip transform (already in `steamvr-and-display.md` part A
   "Controller pose").

Nothing here belongs in the kernel.

### C.2 Camera rig (Q1)

#### C.2.1 Built-in rig table (`FUN_1800af650(out, variant, cam)`) [sure, called under Wine]

Output: a column-major 4×4 float, `X_rig = R·X_cam + t`, mm. `X_cam` is in
Sony's camera frame (the OpenCV frame rotated 180° about z, overview.md §A.8.2),
so its +z is the optical axis and R's third column is the optical axis in
the rig frame. Rotations are built from fixed angles 35°, 15°, 10°, 20°,
30° (`0x180319a58/0x180319808` = ∓0.61087, `0x1803197d0` = 0.26180,
`0x1803197bc` = 0.17453, `0x1803197ec` = 0.34907, `0x180319804` = 0.52360).
Translations: `0x180319b40` (variant 0), `0x180319b80` (1), `0x180319b00` (2).

R given as columns c0 | c1 | c2; t in mm:

| Var | Cam | c0 | c1 | c2 (optical axis) | t |
|---|---|---|---|---|---|
| 0 | 0 | (1, 0, 0) | (0, 0.93969, 0.34202) | (0, −0.34202, 0.93969) | (40.0, −41.683, 93.582) |
| 0 | 1 | (1, 0, 0) | (0, 0.93969, 0.34202) | (0, −0.34202, 0.93969) | (−40.0, −41.683, 93.582) |
| 0 | 2 | (0.86603, −0.25, −0.43301) | (0, 0.86603, −0.5) | (0.5, 0.43301, 0.75) | (61.498, 22.971, 97.027) |
| 0 | 3 | (0.86603, 0.25, 0.43301) | (0, 0.86603, −0.5) | (−0.5, 0.43301, 0.75) | (−61.498, 22.971, 97.027) |
| 1 | 0 | (0.96593, 0.14845, −0.21201) | (0, 0.81915, 0.57358) | (0.25882, −0.55403, 0.79124) | (38.1872, −41.62, 99.4412) |
| 1 | 1 | (0.96593, −0.14845, 0.21201) | (0, 0.81915, 0.57358) | (−0.25882, −0.55403, 0.79124) | (−38.1872, −41.62, 99.4412) |
| 1 | 2 | (0.67101, −0.67564, −0.30538) | (0.46985, 0.70609, −0.52979) | (0.57358, 0.21201, 0.79124) | (65.9827, 30.7139, 106.2393) |
| 1 | 3 | (0.67101, 0.67564, 0.30538) | (−0.46985, 0.70609, −0.52979) | (−0.57358, 0.21201, 0.79124) | (−65.9827, 30.7139, 106.2393) |
| 2 | 0 | (0.96593, 0.14845, −0.21201) | (0, 0.81915, 0.57358) | (0.25882, −0.55403, 0.79124) | (40.0, −39.0685, 79.3971) |
| 2 | 1 | (0.96593, −0.14845, 0.21201) | (0, 0.81915, 0.57358) | (−0.25882, −0.55403, 0.79124) | (−40.0, −39.0685, 79.3971) |
| 2 | 2 | (0.70941, 0.49374, −0.50295) | (−0.49673, 0.85651, 0.14019) | (0.5, 0.15038, 0.85287) | (67.5, 32.1406, 80.106) |
| 2 | 3 | (0.70941, −0.49374, 0.50295) | (0.49673, 0.85651, 0.14019) | (−0.5, 0.15038, 0.85287) | (−67.5, 32.1406, 80.106) |

Note the sign of c2's y: in Sony's camera frame (x, y flipped vs OpenCV)
the optical axis of the lower cameras 0/1 tilts *down* by 20° (var 0) or
35° (var 1/2) in rig y once the 180° flip is undone [likely; check with
§C.8.4 HW data: cams 0/1 look forward, 2/3 up and out]. Variants 1/2 also toe
cams 0/1 out by 15° (c2.x = ±0.259) — this matches the HW measurement
"0/1 toed out 15° each" (overview.md §A.8.4), so **the measured headset is
variant 1 or 2, not 0** [likely]. Baselines: var 1 cam 0–1 76.4 mm,
2–3 132.0 mm; var 2 80.0 / 135.0 mm; measured 78.8 / 133.2 mm.

The variant is the tracker field `param_1[0x3b]` passed by
`FUN_180049240` (decomp ~9107). Its source was not traced (§C.8).

#### C.2.2 Calibration on top of the table (`FUN_180049240`, `FUN_1800b02a0`) [sure for the flow]

- If the tracker has no calibration (`+0x1ec` = 0): all four cameras use
  the built-in table for the variant.
- With calibration (VD type 10, the 3 extrinsics): `FUN_1800b02a0` first
  fills all four from the table, then takes **camera 0's built-in pose**
  and composes each converted extrinsic (`R′ = G R G`, `t′ = 1000·G t`,
  `G = diag(−1,−1,1)`, `0x180319a70` = −1000, `0x180319a04` = 1000) with
  it to place cameras 1–3. Camera 0 keeps its built-in pose.
- The four matrices are copied to the camera object at
  `DAT_1805f6cb0 + 0x10a40 + 0x40·cam`. `FUN_1800cbd70(obj, out, cam,
  invert)` returns that matrix (invert = 0) or its rigid inverse
  `[Rᵀ | −Rᵀt]` (invert = 1). The per-frame core `FUN_18004b0a0` uses the
  inverse (rig → camera) to project LEDs.

So the answer to "is camera 0's built-in pose used on top of the
calibration": **yes**. The calibration supplies only camera-to-camera
extrinsics; the rig placement of camera 0 always comes from the table.
The rig frame is therefore not camera 0's frame. overview.md §A.8.3 now says so (errata in
overview.md §A.12).

#### C.2.3 VD type 13 (`imu_set_calib.bin`, 0x34 bytes) [sure for the routing]

- Dispatcher `case 0xd`: copied to headset object `+0x41298`, version
  counter `+0x43dda`, and pushed to the shared-memory block for the PS VR2
  app (`FUN_18015e910`, ShareManager `+1000..+0x418`).
- Read back by `FUN_180124780` (getter) for three users:
  - `FUN_1801132a0`: VrTrace dump (skip);
  - `FUN_1801617c0`: on a new version, sends words +0x0c, +0x00, +0x04,
    +0x08 to the config manager `FUN_180157800` (meaning not traced);
  - `FUN_180173fe0`: the **headset INS** calibration. It combines type 12
    (`FUN_180124700`) with type 13 and builds two 4×4s
    (`DAT_180350b50`, `DAT_180350b90`). From type 13 it uses only the
    three floats at **+0x28..+0x30, negated**, as the translation of the
    second matrix [sure]; the rotation comes from type 12.
- The IR tracker does not read `+0x41298` directly, but type 13's
  quaternion reaches it through config pose A (§C.2.5). So type 13 **is**
  on the controller chain, as an IMU ↔ head/rig rotation, not as a
  camera extrinsic.

#### C.2.4 Head pose handed to the solver (`FUN_18018aaf0` decomp ~244285) [sure for the chain, called under Wine]

Per LD frame the IR thread takes the head INS pose at the frame time
(`FUN_180179bb0`, world ← IMU, m) and composes, with 4×4 column-major
matrices:

```
world_from_X = M(INS pose) · M(A)⁻¹ · M(B)
A = config +0x48c, B = config +0x66c   (config manager FUN_180156470)
```

then converts back to a pose (`FUN_18016e910`) and passes it with the
blobs to the per-frame core `FUN_18004b0a0`. The HMD path in
`FUN_180160230` builds the same `INS · A⁻¹ · B` chain (via
`FUN_1801613a0`/`FUN_18011e190`) for the headset's reported pose.

Helpers (all [sure, Wine]): pose struct = pos f32[3], quaternion
(w, x, y, z), scale f32[3]; `FUN_18016dc50` = pose → `[R|t]`;
`FUN_18016dda0` = pose → inverse `[Rᵀ|−Rᵀt]`; `FUN_18016bd20(a, out, b)`
= `out = a·b`; `FUN_18016f350/f3e0/f470` = rotation about x/y/z, degrees.

Defaults (`FUN_180155000` → `FUN_1801556b0`; no JSON override found):

| Pose | Position (m) | Rotation | Notes |
|---|---|---|---|
| A (`+0x48c`) | (0, −0.027741, −0.078431) | q (w,x,y,z) = (0.477714, −0.521334, 0.521334, 0.477714) | raw R = RotX(−95°)·RotZ(90°) at cfg `+0x118`, **pre-multiplied by the VD type 13 quaternion** (cfg `+0x9c0`, identity by default) |
| A⁻¹ | (−0.08055, 0, 0.02080) | columns (0, −1, 0), (−0.08716, 0, 0.99619), (−0.99619, 0, −0.08716) | = the HMD driver-from-head rotation with an extra **5° tilt about x** |
| B (`+0x66c`) | (0, 0, 0.09) | identity | raw cfg `+0x1cc` |
| `+0x694` (HMD `vecDriverFromHead…`) | (−0.09255, 0, 0.0208) | (−0.5, −0.5, 0.5, 0.5) | for comparison; from cfg `+0x13c − +0x10c` |

So the X frame's origin is at `A⁻¹·(0, 0, 0.09)` = **(−0.17021, 0,
0.01296) m in IMU axes**, and its axes are the head axes tilted 5° about
x. Whether X is exactly the camera rig frame of §C.2.1 (in metres; the rig
is in mm) is [likely] but not read in `FUN_18004b0a0`. Two facts argue
for it: the solver needs world ← rig, and B's 0.09 m matches the cameras'
rig z (≈ 80–106 mm). One fact needs explaining: X's +z maps to IMU −x,
the same direction as the HMD head's +z. A hardware check settles it
(§C.8).

When the controller has no prediction, the IR thread seeds its prior
position as **(0, −0.2, 0.3) m in X** (`0xbe4ccccd`, `0x3e99999a`).

#### C.2.5 VD type 13 layout (detail of §C.2.3) [sure for the first 16 bytes]

`FUN_1801617c0` forwards words `+0x0c, +0x00, +0x04, +0x08` to
`FUN_180157800`, which stores them as the quaternion (w, x, y, z) at
config `+0x9c0` and recomputes the config (`FUN_1801556b0`): so **type 13
starts with a quaternion x, y, z, w** (not the u8 from/to + 3×3 layout of
an extrinsic record). That quaternion rotates pose A, i.e. it is a
per-unit **IMU ↔ head/rig alignment**, and it reaches the controller
solve through A. Its translation-like floats at `+0x28..+0x30` are used
(negated) only by the head INS (`FUN_180173fe0`).

### 2b. LED model ↔ controller IMU, and the hand mirror (Q2)

#### 2b.1 IMU offset (`FUN_18005f1b0`, IR thread decomp ~243800) [sure]

- The IR thread calls `UkfManager::setImuPose(ukf, hand, ImuPoseIndex,
  &v)` with `v` **always non-null**: `v` = pad `+0x48` (3 floats, mm),
  read by `FUN_180118b50`. It logs (XOR-obfuscated string) "Initialize
  VrController UKF: dev, model, …" with v.
- Pad `+0x48` is set on connect in `FUN_18011bae0` from the pad side
  (`+0x34`: 0 = right "R", else left "L"):
  - right: **(−20.0727, −12.2481, −6.0039) mm**;
  - left: **(+9.3727, −12.2481, −6.0039) mm**.
  These are the same numbers as the SteamVR "grip" offset (`steamvr-and-display.md` part A).
- With `v` non-null, every index ≥ 3 builds `M = [I | −0.001·v]` (metres,
  `0x180319a54` = −0.001; identity rotation) and stores **M⁻¹ =
  [I | +0.001·v]** at UKF device `+0x808` (stride 0x2218). So the UKF's
  IMU ↔ LED-model link is a **pure translation of v (mm) with no
  rotation**; the per-index tables below are not used in this build's
  normal flow. Which way the translation applies (IMU in model frame, or
  model origin in IMU frame) [likely: v = IMU position in the LED-model
  frame, mm, since the model is right-handed and the left pad's x is
  mirrored around −5.35 mm].
- Index ≥ 3 also sets UKF `+0x6c8/+0x6cc` = ∓33.1613 (rev 1, index 3–8)
  or ∓66.3225 (index 9–16) and `+0x7d0` = 250.0 [meaning open].
- Tables used only when `v` is NULL (column-major, translation in m),
  for reference [sure for values]:

  | Index | c0 | c1 | c2 | t (m) |
  |---|---|---|---|---|
  | 1 | (−0.27575, −0.19665, 0.94090) | (0.73570, 0.58679, 0.33826) | (−0.61863, 0.78550, −0.01713) | (0.014455, 0.012869, −0.008378) |
  | 2 | (0.27575, 0.19665, 0.94090) | (0.73570, 0.58679, −0.33826) | (−0.61863, 0.78550, 0.01713) | (−0.015395, 0.013208, −0.008331) |
  | 3 | (1,0,0) | (0, 0.79864, 0.60181) | (0, −0.60181, 0.79864) | (0.02891, 0.01097, 0.00084) |
  | 4 | same R | | | (−0.02891, 0.01097, 0.00084) |
  | 5 | (1,0,0) | (0, 0.63917, 0.76906) | (0, −0.76906, 0.63917) | (0.021404, 0.005154, 0.010678) |
  | 6 | same R | | | (−0.016129, 0.005154, 0.010678) |
  | 7 | (1,0,0) | (0, 0.63917, 0.76907) | (0, −0.76907, 0.63917) | (0.021523, −0.004072, 0.011409) |
  | 8 | same R | | | (−0.022710, −0.002764, 0.012496) |
  | 9–16 | I | | | (0, 0, 0) |

  (Tables `0x18031a020/0a0c0/a040/a060/a080` for c0,c1 and
  `0x18031a300..a420` for c2,t. Index 3–8 = rev 1–3; the rotations are
  37.0° and 50.3° about x.) Each is inverted before storing.

#### 2b.2 Hand-1 descriptor byte = mirror axis 'X' (`FUN_18018aaf0`, `FUN_1800d3360`, `FUN_1800d3ae0`) [sure]

- The "byte 0x58" is a **value**, ASCII `'X'`, written into the low byte
  of descriptor word 1 when hand == 1 (left). Hand 0 leaves it as is.
- `FUN_18004a9d0` passes it to `FUN_1800d3360`, which stores it at boot
  object `+0x1978`.
- `FUN_1800d3ae0` (model load): the model's own byte at model `+0x31e`
  wins unless it is 0 or `'0'`; then the descriptor byte is used. For
  every LED: `'X'`/`'x'` negates x of the position (`+0x18f8`) and the
  normal (`+0x1918`); `'Y'`/`'y'`/`'1'` negates y; `'Z'`/`'z'` negates z.
  (`FUN_1800e2a00` transforms `+0x18f8` entries as points and dots
  `+0x1918` entries with view rays, so `+0x18f8` = positions.)
  After mirroring it **reverses the order of every LED index list** at
  `+0x19c8` (winding/ring order flips with a mirror).
- So: hand 0 = right uses the model as stored; hand 1 = left uses it
  mirrored in x. Hand index = SteamVR device − 1 (device 1 right, 2 left).

### C.3 Wire and data formats

#### C.3.1 Per-controller result block (IR frame struct, stride 0x5a44) [sure]

The IR thread copies its 0xb4f8-byte frame struct to tracker `+0x38c50`
every LD frame (`FUN_18018aaf0`, decomp ~245700). Per controller block:

| Off | Type | Meaning |
|---|---|---|
| +0x08 | u64 | frame time (µs, host clock) of the LD frame |
| +0x10 | u8 | optical pose valid this frame |
| +0x12 | u16 | flags; bit 3 (`8`) = no tracker output [likely]; bit 2 (`4`) set by the reader when the optical pose is stale |
| +0x14 | f32[3] | optical position |
| +0x20 | f32[4] | optical orientation (x, y, z, w) |
| +0x15a0 + 0x1688·cam | f32[3] | per-camera vector passed to the UKF measurement (meaning open) |

On a valid frame the tracker caches the time at `+0x44148[hand]`, the
position at `+0x44158 + 12·hand` and the quaternion at `+0x44170 +
16·hand` (stored w, x, y, z).

#### C.3.2 Tracker output (`FUN_180189c70(tracker, hand, t, out)`, 0xc0 bytes) [sure]

| Off | Type | Meaning |
|---|---|---|
| +0x00 | u64 | time of the UKF state used |
| +0x08 | u64 | time of the last valid optical frame |
| +0x10 | u64 | UKF `+0x3b0` (per device; meaning open) |
| +0x18 | s32 | hand |
| +0x1c | f32[3] | position |
| +0x28 | f32[4] | orientation x, y, z, w (w = 1 on reset) |
| +0x38 | s32 | 1 = tracker has a state, 2 = state lost, 0 = no output |
| +0x3c, +0x40 | s32 | tracking status: 9, 6, 3 or 0 (§C.5) |
| +0x44..+0x4c | f32[3] | velocity (copied only on a fresh optical pose) |
| +0x50.. | | angular velocity, etc. |
| +0x68 | u8 | 6 = UKF valid, 7 = UKF valid and fresh optical, 0 = none |

`FUN_180160230` (device 1 = right, 2 = left; hand = device − 1) turns this
into the 0x50-byte pose `steamvr-and-display.md` part A documents: `valid (+0x4c) =
(+0x3c ≠ 0) && headset ready (FUN_1801253c0)`.

### C.4 Fusion: optical measurement (Q3) [partial]

`FUN_18005ec00(ukf, hand, t_frame, frame, …, rot_free, …)`, called from
the IR thread per controller per LD frame:

1. Builds a 0xb0-byte `DotMarkerData` record: valid byte (+0x10),
   position (+0x14), and the quaternion (+0x20) **normalised** — unless the
   `rot_free` flag (7th argument) is set, then no rotation is taken
   [sure]. Also four per-camera vectors and 32 bytes of UKF state
   `+0x08..+0x28`.
2. If the device's filter is not initialised and the measurement is valid,
   the filter is initialised from it (`FUN_180068620`); on success state
   `+0x2238 = 1` [sure].
3. Once initialised, the record goes into the measurement ring
   (`+0x1f60 + 0x2218·hand`, 0xb0 per entry) and the queue is drained
   through `FUN_180067b50` (the step / replay function, overview.md §A.9.4).
4. Remembers `rot_free` at `+0x4458 + hand`.

So Sony feeds the UKF a **full 6-DoF pose** (position + rotation) in the
solver's output frame, stamped with the LD frame time; it is not an LED
pixel measurement [sure].

#### C.4.1 Innovation gate (`FUN_180065b70` "kalmanFilterStep") [sure for the test, likely for reject]

Before the update (`FUN_180065760`, then `FUN_18006fe90`), the step
predicts the position at the measurement time and computes
`d = |p_meas − p_pred|` (m) and the speed `|v|`:

```
if |v| < 1.0 m/s (cfg +0x784):  skip the update when d > 0.05 m (cfg +0x778)
else:                            skip the update when d > 0.20 m (cfg +0x780)
```

#### C.4.2 UKF config defaults (`FUN_1800598a0`, block at UKF `+0x6a0`) [sure for values]

Names are [guess] unless the use is known (process noise uses per
part H).

| Off | Value | Meaning |
|---|---|---|
| +0x6a0/+0x6a4 | ∓43.6332 | ±2500 °/s in rad/s [guess: gyro range] |
| +0x6a8/+0x6ac | ∓6.0 | [guess: bias/accel limit] |
| +0x6b0..+0x6b8 | 3.0e-4 ×3 | process noise base, block 1 |
| +0x6c0..+0x6c8 | 8.0e-6 ×3 | process noise base, block 2 (+0x6c8/+0x6cc then overwritten by setImuPose: ∓33.16 / ∓66.32) |
| +0x6d0, +0x6d4, +0x6d8 | 2.5e-5, 6.25e-6, 3.04617 | rate noise own, cross, fixed |
| +0x6dc, +0x6e0, +0x6e4, +0x6e8 | 1e-4, 2.5e-5, 6.25e-6, 6.25 | second block extra, own, cross, fixed |
| +0x6ec..+0x6f4 | 1.9038590e-7 ×3 | [guess] |
| +0x744..+0x74c | 1.0e-6 ×3 | [guess] |
| +0x754..+0x760 | 3.0462e-10, 3.0462e-6, 7.6154e-9, 1.0e-8 | [guess: optical measurement variances; 3.046e-6 rad² = (0.1°)², 1e-8 m² = (0.1 mm)²] |
| +0x764, +0x768 | 0.05236 (3°), 0.01745 (1°) | [guess] |
| +0x76c | 10 (int) | |
| +0x770 | 1.5 | |
| +0x778 / +0x780 / +0x784 | 0.05 m / 0.20 m / 1.0 m/s | optical gate (§C.4.1) [sure] |
| +0x77c | 0.0333 | |
| +0x788, +0x78c | 0.01, 1.0 | |
| +0x790 | 100 (int) | |
| +0x794..+0x7a8 | 0.8, 32.0, 0.34907 (20°), 2.0, 0.02, 177.78 | (+0x798..+0x7a0 are overview.md §A.11 q9 "limits") |
| +0x7b0 / +0x7b8 / +0x7c0 | −1 (u64) / 14000 (u64) / 7 | |
| +0x7c8, +0x7cc, +0x7d4 | 0.5236 (30°), 0.045, 0.2618 (15°) | |

### C.5 Output, staleness and not-tracking (Q3, last part) [sure]

Defaults set by the tracker constructor `FUN_1801852e0`: stale limit
`+0x441f8` = 0 → **0.3 s**; mode `+0x4d378` = 1; `+0x4d26c` = 1;
`+0x4d270` = −1. The setters (`FUN_1801929f0`, `FUN_180192a50`,
`FUN_180193990`) have no callers, so these are fixed.

`FUN_180189c70(hand, t)`:

```
t += global offset DAT_18035ba14            (0 unless set; FUN_18018a7e0 clears it)
stale_limit = 0.3 s   (+0x441f8: 0 → 0.3 s, <0 → never, >0 → value µs)
state = UKF predicted to t                  (FUN_18005df40 → FUN_18005dab0)
if state valid:
    age = state.time − last_optical_time     (s, ÷1e6)
    fresh = age < 0.3
    if age ≥ stale_limit:
        position = last cached position      (held; or the fallback model
                                              FUN_1801b99a0 when +0x441d8[hand] is set)
        rotation = UKF rotation
        flags |= 4
    else:
        position, rotation = UKF; update the cache; velocity copied
    status = (fallback-model position) ? 6 : (fresh && !held) ? 9 : 3
else (no UKF state):
    position = cached; rotation = cached if 0.9 ≤ |q|² ≤ 1.1
    if last optical < 0.3 s ago → same status rule
    else → +0x38 = 2, status 0  (poseIsValid = 0)
if flags bit 3: +0x38 = 0, status 0           (poseIsValid = 0)
```

Constants: 1000.0 (`0x180319a04`, µs→s twice), 0.3 (`0x1803197d4`),
FLT_MAX (`0x180319a50`), 0.9/1.1 (`0x180319818`/`0x180319830`), 2.0 s
(`0x1803198a8`, recent-event window on `+0x4f468`).

In SteamVR (`FUN_1801a7bf0`, `steamvr-and-display.md` part A): `eTrackingResult` is always
200 (Running_OK) while connected and not paused; `poseIsValid` is the
valid byte above. There is **no not-tracking timeout to
`TrackingResult_Running_OutOfRange`**: after 0.3 s without optics the
position freezes at the last optical value and the rotation keeps coming
from the IMU-driven UKF, valid, indefinitely [sure for this path]. Only a
lost filter state (or flags bit 3) clears `poseIsValid`.

Headset path (device 0) of `FUN_180160230`, for comparison: status 9 when
the INS is 6-DoF (`FUN_1801752b0`), else 3.

### C.6 Constants

| Name | Value | Source |
|---|---|---|
| Optical stale limit | 0.3 s | `0x1803197d4`, `FUN_180189c70` |
| Quaternion sanity | 0.9 ≤ \|q\|² ≤ 1.1 | `FUN_180189c70` |
| Rig angles | 35°, 15°, 10°, 20°, 30° | `FUN_1800af650` |
| Rig translations | §C.2.1 | `0x180319b00/b40/b80` |
| Extrinsic scale | ±1000 (m → mm) | `0x180319a70`, `0x180319a04` |
| Grip offset | left (9.3727, −12.2481, −6.0039) mm, right (−20.0727, …) | `0x1802538ac/b0`, `FUN_180160230` |

### C.7 Gaps in the Linux driver

- `psvr2_led_solver.c` assumes the rig frame is camera 0's frame. Sony's
  rig frame puts camera 0 at its built-in pose (§C.2.2). Any rig → head
  transform taken from Sony must be applied to Sony's rig frame, so build
  the rig the same way.
- No 0.3 s position hold, and no status distinction (9/3) in the
  controller pose.

### C.8 Open questions

1. Is X (§C.2.4) exactly the §C.2.1 rig frame, and where does
   `FUN_18004b0a0` scale m → mm and flip axes? **Hardware check:** hold a
   controller still, solve it in the rig frame (mm), map with
   `INS · A⁻¹ · B` (rig/1000, then try identity vs 180° about y), and see
   which one keeps the world position fixed while the head turns.
2. Variant source: who sets tracker `param_1[0x3b]` (`FUN_180049240`).
3. Whether the UKF state (and so the tracker output) is the IMU or the
   LED-model origin, i.e. which side of the v translation (§C.2b.1) the
   SteamVR grip offset undoes (`FUN_18006c340` lever arm, measurement
   model `FUN_18005fc20`).
4. Model byte `+0x31e` values per rev (is any model pre-mirrored?).
5. Which config entries are the optical measurement noise: read the
   sigma-point update `FUN_18006fe90` → `FUN_18006c760` (gate is done,
   §C.4.1).
6. Two controllers: `FUN_1800a5e30` → `FUN_1800ad2b0`.
7. LD `+0x08` → frame time offset; `DAT_18035ba14`.
8. ~~Status 6's fallback~~: part F §F.3.6 (an 80° view-cone clamp; the
   enable byte `+0x441d8[hand]` is never set on PC).

### C.9 Function map

| VA | Role | Covered |
|---|---|---|
| `FUN_1800af650` | built-in camera rig pose (variant, cam) | yes (Wine) |
| `FUN_1800b02a0` | rig from calibration extrinsics + cam 0 built-in | flow |
| `FUN_1800cbd70` | get rig matrix or its rigid inverse | yes |
| `FUN_180049240` | tracker setup; selects table vs calibration | partly |
| `FUN_180173fe0` | headset INS calibration from VD 12 + 13 | partly |
| `FUN_180124780` | getter for VD type 13 | yes |
| `FUN_1801617c0` | forwards VD 13 to config | skim |
| `FUN_18005ec00` | optical pose → UKF measurement queue | yes |
| `FUN_180189c70` | tracker pose output, staleness, status | yes |
| `FUN_180160230` | per-device pose for SteamVR | controller path |
| `FUN_1801852e0` | IR tracker ctor (defaults) | fields used here |

---

## Part D: Boot kernels

Behavioural spec for the controller-tracker boot stage of Sony's PSVR2 SteamVR
driver (build PE timestamp 1741940190, VA range `0x1800a0000–0x1800fffff`,
plus the `RnD::` helper classes it uses). Written in our own words from
analysis; our implementation is `userspace/lib/sie/ctrlboot/`, checked by
`scripts/sony-re/difftest/t_ctrlboot_*.c`. Per-function status:
`scripts/sony-re/status/ctrlboot.tsv`.

Conventions: `|x|` is the IEEE absolute value (sign bit cleared, NaN stays
NaN). All comparisons are ordered (false when either side is NaN). Float is
binary32, no fused multiply-add.

### Common vector type

Many `RnD::` methods take a dense vector object. Only two fields matter: a
pointer to the float elements at +0x30 and an element count (64-bit) at
+0x38. Below, "vector" means that (data, n) pair.

### Robust loss functions (`RnD::Robustifier*`)

Each robustifier is an object with a vtable and one float parameter `c` at
+8 (the cut-off, in units of the noise scale). The vtable slots are:

| Slot | Meaning |
|------|---------|
| 0 | `weights(out, r, s)`: per-element IRLS weight from residual `r[i]`, scale `s` |
| 1 | `weights_sq(out, e, t)`: per-element weight from a non-negative error `e[i]` against a threshold `t` (no `c`) |
| 2 | `sqrt_weights(out, r, s)`: square root of slot 0 |
| 3 | `param()`: returns `c` (Null returns 1.0) |
| 4 | `kind()`: 0 Null, 1 Huber, 2 Clip, 3 Tukey |
| 5 | destructor |
| 6, 7 | Huber only: `rho(x)` and `rho(x)/x²` |

`out` has the same length as the input; the loop runs over `out`'s count.
Let `k = s·c` (computed per element as `s * c`), `a = |r[i]|`.

**Null** (vtable `0x1803196c8`): every slot writes 1.0.

**Huber** (vtable `0x180319680`):
- slot 0: `a > k` → `k / a`, else 1.0.
- slot 1: `e > t` → `sqrt(t / e)`, else 1.0 (sign of `e` is kept).
- slot 2: `a > k` → `sqrt(k / a)`, else 1.0.
- slot 6, `rho(x)`: `a = |x|`; `a < c` → `a²`, else `(a + a − c)·c` (evaluated
  as `((a + a) - c) * c`).
- slot 7: `|x| < 0.001` → 2.0, else `rho(x) / (x·x)` (rho through the vtable).

**Clip** (vtable `0x180319648`):
- slots 0 and 2: `k < a` → 0.0, else 1.0.
- slot 1: `t < e` → 0.0, else 1.0.

**Tukey** (vtable `0x180319610`):
- slot 0: `a < k` → `u = a / k`, `(1 − u·u)²`; **else 1.0** (Sony's slot 0
  gives full weight to outliers; slot 2 gives them zero. Reproduced as is).
- slot 1: `e < t` → `(1 − e/t)²`, else 0.0.
- slot 2: `a < k` → `sqrt((1 − u·u)²)` with `u = a / k`, else 0.0.

Square roots are the correctly-rounded `sqrtf`.

### Random generator (`RnD::MersenneTwister`, `RnD::Random<MT, uint>`)

Object: vtable, then 624 × u32 state at +8, then a u32 read index at +0x9c8.
The algorithm is the standard MT19937 (Matsumoto–Nishimura reference
constants: init multiplier 1812433253, twist matrix `0x9908b0df`, tempering
shifts 11, 7/`0x9d2c5680`, 15/`0xefc60000`, 18). Two details differ from
`std::mt19937` and change the output stream:

- **Seeding** (`0x18009f3e0`) fills the state the reference way and then
  **twists immediately**, setting the index to 0. Outputs start at the
  tempered first word of the twisted state.
- **Reading** (`0x18009f530`, `block(out, step)`) always tempers the eight
  words `state[index .. index+7]` into `out`, then adds `step` to the index.
  If `index + 8 > 624` afterwards, the index resets to 0 and the state is
  twisted. So single draws (step 1) use only words 0..616 of each generation;
  words 617..623 are never output. Step 6 is used to draw six at once.

Derived draws (all take one step-1 word `u`):

| VA | Result |
|----|--------|
| `0x18009f630`, `0x1800ac930` | `u` |
| `0x1800ac850` | `(float)u * 2^-32` (float multiply; can round to 1.0) |
| `0x1800ac8a0` | `(double)u * 2^-32` |
| `0x1800abe10` | `uniform(lo, hi)`: `(float)((double)u * 2^-32 * (double)(hi - lo) + (double)lo)`, `hi - lo` in float |

Seeds seen at call sites: `0x14`, `0x29a`, `0x2fd78`, and config-supplied
values.

### Fused multiply-add

Unlike the libpad/irtracker code, this module **does use FMA**: matrix ×
vector is `mul` for the first column then `vfmadd231ps` for the others
(976 FMA instructions in `0x1800a0000–0x1800fffff`). Ghidra prints a fused
`a*b + c` the same as an unfused one, so every function's fusion pattern is
taken from the disassembly. Our code is built with `-ffp-contract=off` and
calls `fmaf` exactly where the reference fuses.

### Math helpers (`psvr2_cb_math.c`)

Matrices are 4 columns × 4 floats; column 3 is the translation.

| VA | Behaviour |
|----|-----------|
| `0x1800cbd70` | `get(obj, out, i, invert)`: matrix `i` of a table at obj+0x10a40 (64 bytes each). `invert`=0 copies. Otherwise rigid inverse: column `j` = row `j` of the 3×3 with lane 3 = `c2.w`; translation column = `R^T·(−t)` computed per lane as `fma(c2', −tz, fma(c1', −ty, c0'·(−tx)))` (lane 3 gets the same formula from `c2.w`). Returns `out` |
| `0x1800c05f0` | `rotate(m, out, v)`: `out = fma(v.z, c2, fma(v.y, c1, v.x·c0))`, 4 lanes |
| `0x1800e0a60` | depth of `v = **(obj+0x18f8)` in pose `m`: if byte m+0x4ac is 0 returns −1; else lane 2 of `fma(v.z,c2, fma(v.y,c1, fma(v.x,c0,c3)))` |
| `0x1800c0220` | `out = fma(b, a, −c)` per lane of 2-vectors (args out, a, b, c) |
| `0x1800c04c0` | 2-D length `sqrt(y² + x²)` |
| `0x1800c0520` | 2-D normalise: `k = 1/sqrt(y² + x²)`, `out = (k·x, k·y)` (no zero guard) |
| `0x1800c5290` | `a.y·b.x < a.x·b.y` (b is counter-clockwise of a) |
| `0x1800c7200` | Stat finalise on {sum, sumsq, std, −, valid, int count}: count 0 → valid = 0. Else mean = sum/n (stored), var = sumsq/n − mean² (stored), std = sqrt(var) only if var ≥ 0 |
| `0x1800c8b50` | RANSAC bound on {+4 int max, +0xc u8 adaptive, +0x14 confidence p, +0x1c inlier ratio w}: if adaptive, `w' = w < 0.9 ? w : 0.9`, `n = trunc(log(1−p) / log(1 − w'^6))` in double (64-bit truncation, low 32 bits), `max = min(max, n)`. **Minimal sample = 6 points** |
| `0x1800cc4b0` | `[[a b][c d]]`: `t = −d − a`, `disc = t² − (a·d − b·c)·4`; if `disc > 1e-6`: `l = (sqrt(disc) − t)·0.5 − d`; if `l ≠ 0` writes `c / l`, returns 1; else 0 |
| `0x1800d0750` | `1 − (clamp(sqrt(x), lo, hi) − lo)/(hi − lo)`, clamp as SSE `max(lo, s)` then `min(hi, ·)` (`lo > s ? lo : s`, `hi < s ? hi : s`) |
| `0x1800db5f0` | `n = sqrt((y² + x²) + z²)` of the vec at arg+0x30; obj+0x4c94 = 2 if `100 > n`, else 1 if `150 > n`, else 0 |
| `0x1800e1c90` | LED facing: returns 1 if obj+0x3360 is 0 or `!(10 > obj+0x35e0)`. Else `r = rotate(m, axis@0x3310) ⊙ scale@0x3300`, `d = (r.y + r.x) + r.z`; returns 0 iff `cos(0.3490658503988659) ≥ d` (double; 20°) |
| `0x1800e80c0` | `out[k] = fma(m.c3[k], w.z, fma(m.c2[k], w.y, fma(m.c1[k], w.x, 0))) · s`, k = 0, 1, written at out+0x10; `w` at arg4+0x10 |
| `0x1800e9e80` | 2-vector + (0.5, 0.5) |
| `0x1800bff30` | `(int)d[0], (int)d[1]` (truncation) |
| `0x1800d0480`, `0x1800d04b0` | `clock()` elapsed seconds since a stored start (ms / 1000) |
| `0x1800d0c40` | constructor: two (0, −1, 0, 0) vectors, (0,0,0,1), byte +0x34 = 0, float +0x30 = cosf(0.34906557) |

### Complex and 2×2 helpers (`psvr2_cb_cplx.c`)

Complex numbers are float pairs (re, im). These serve a polynomial root
solver (cube roots, square roots and division of complex values are the
pieces of a closed-form cubic/quartic solve) [likely: P3P-style minimal
pose from 3 LEDs].

| VA | Behaviour |
|----|-----------|
| `0x1800f4100` | `a / b`: `den = fma(b.im, b.im, b.re²)`; re = `(a.re·b.re + a.im·b.im)/den`, im = `(a.im·b.re − a.re·b.im)/den` (products rounded, then add/sub) |
| `0x1800f41e0` | Principal cube root: `θ = atan2f(im, re)/3`, `r = (float)sqrt(im² + re²)` in double, `m = cbrtf(r)`, result `(cosf θ·m, sinf θ·m)` |
| `0x1800f42a0` | Principal square root in double: `|z| = sqrt(fma(im, im, re²))`; re = `sqrt((|z| + re)/2)`, im = `sqrt((|z| − re)/2)`, negated when im < 0 (−0 counts as positive); rounded to float |
| `0x1800f5d30` | `(cosf a · r, sinf a · r)` (args out, &r, &a) |
| `0x1800b1b20` | 2×2 inverse of (a, b, c, d) (column order): `det = fma(−b, c, d·a)`; `r0 = rcpps(det)`; `r1 = fma(−(r0·det), r0, r0 + r0)`; for adj = (d, −b, −c, a): `y = adj·r1`, out = `fma(−(r1·det), y, y + y)`. No singular check. Uses the CPU reciprocal estimate |
| `0x1800b1aa0` | `out = x·c0 + y·c1` for a 2×2 (args m, out, v) |
| `0x1800f3220` | 2-D normalise with `rsqrtps` + two Newton steps: `s = y² + x²`, `r ← fma(−(r·s), r, 3)·(r·0.5)` twice, out = `r·v` |
| `0x1800b2ba0` | outer product `(b.x·a.x, b.x·a.y, b.y·a.x, b.y·a.y)` |
| `0x1800b2c20` | 3×4 block `[b.x·v, b.y·v, 0]` |
| `0x1800b1e90`, `0x1800b1f30` | 12-float `a += b`, `a *= s` |
| `0x1800b1690` | 2-vector `a += b` |
| `0x1800b0130` | pose from row-major R (9 floats) and t (metres): columns `(R0, R3, −R6, 0)`, `(R1, R4, −R7, 0)`, `(−R2, −R5, R8, 0)`, `(−1000·t0, −1000·t1, 1000·t2, 0)`: transpose with the z axis flipped, metres → mm |

Tolerance: functions calling `cosf/sinf/cbrtf/atan2f` match to ≤ 2 ulp
(the reference uses the Windows UCRT; our build uses another libm). Every
other function in this table is bit-exact. `rcpps`/`rsqrtps` estimates
differ between CPU vendors, so those two match only on the same CPU (as in
the test, which runs both in one process).

---

## Part E: Background model

Sony's controller tracker keeps a 3D map of IR light sources that do not
move (lamps, reflections, other headsets' LEDs at rest) and gives every
blob of every camera a "background" score, so the controller solver can
ignore blobs that are part of the room. The Linux driver has nothing like
it. No other doc covered it; `overview.md` part C called its stages "boot search
stages".

Tags as elsewhere: `[sure]` read from the code, `[likely]`, `[guess]`.

### E.1 Summary for the driver

- Three stereo **views**, each a pair of cameras: (0, 1), (0, 2), (1, 3)
  [sure].
- Each view keeps a list of 3D **landmarks** in world coordinates
  (320-byte records), built by triangulating blobs that both cameras of
  the view see and that are not near the headset [sure for the flow,
  likely for the details].
- Every frame each landmark is projected into both cameras with the
  current head pose, matched to the nearest blob, and its per-camera hit
  and miss counters updated [likely].
- A landmark is dropped after **40** missed frames in a camera, or **300**
  once that camera has seen it at least **10** times [sure].
- Output: per camera, a float per blob (0 = not background; otherwise the
  matched landmark's score) [sure]. The controller solver never reads it
  (§E.5) [likely], so the model is optional for a port.

### E.2 Objects and calls [sure]

| Item | Value |
|------|-------|
| Interface | `RnD::IBackground` (vtable `0x180319060`; slots 1–4 pure) |
| Used on PC | `RnD::Background` (vtable `0x1803190e0`), 0x490 bytes, 3 views × 0x180 (ctor in `0x180049240`, view ctor `0x1800b6d40`, view dtor `0x1800b3f50`) |
| Alternative | `RnD::Background2` (vtable `0x180319090`), 0x2a90 bytes, 4 per-camera models × 0xaa0. Chosen only when BSS byte `0x1805f05d8` is non-zero; nothing writes that byte, so it is dead on PC [likely] |
| Owner | global `Block` (Background), `DAT_180616700` (Background2); freed in `0x18004ab50` |

| Slot | Background | Background2 | Role |
|------|------------|-------------|------|
| 0 | `0x1800b3ea0` | `0x1800b6ed0` | destructor |
| 1 | `0x1800b2190` (returns 3) | `0x1800b2100` (returns 4) | number of views / cameras |
| 2 | `0x1800b3fb0` | `0x1800b7000` | update one view (frame id, view index, frame data, blobs); Background runs only when `frame +0x10334` ≥ 2 [likely: the number of cameras in the frame] |
| 3 | `0x1800b3fe0` | `0x1800b82f0` | write the per-blob scores of one camera |
| 4 | `0x1800b6450` | `0x1800b9f00` | debug overlay (text `lmcount: %d`, drawing helpers `0x1800bdd00`, `0x1800be0c0`, `0x1800bedc0`, `0x1800bf660`) |

Per frame, in the controller per-frame core `0x18004b0a0`: slot 2 for
views 0..2, then slot 3 for every camera, then the blob lists go to the
camera managers (`0x1800a0480`) and the controller boot/track runs
(`0x1800a56b0`). The multi-camera path (`0x180048690`) does the same on
worker threads (`BackgroundThreadParams`).

View records (0x180 bytes): `+0` view id, `+4` camera A, `+8` camera B,
`+0x10` / `+0x50` world-from-camera poses of A and B (head pose ·
extrinsic, frame `+0x10a40 + 0x40·cam`), `+0x90..+0x128` per-camera blob
counts and scratch, `+0x130` landmark list (RnD::Vector of 320-byte
records).

### E.3 One view update (`0x1800b40e0`) [sure unless tagged]

1. World-from-camera poses for cameras A and B; blob counts.
2. `0x1800b53c0` for A, then for B (§E.3.1).
3. `0x1800b3090` for A, then for B, with factor 0.99 (§E.3.2).
4. `0x1800b43d0`: new landmarks from the pair (§E.3.3).
5. Prune: for each landmark and each of its two cameras, misses
   (`+0xa0` for A, `+0x130` for B) over the limit drop it. The limit is
   300 when that camera's hit count (`+0x24` / `+0xb4`) is ≥ 10, else 40.

#### E.3.1 Projection and ray test (`0x1800b53c0`) [likely]

- Each landmark is projected into the camera (head pose · extrinsic,
  distortion through `0x180045650`). Landmarks behind the camera, at zero
  depth or far outside the view (a 2D magnitude test against 200,
  units not checked) are skipped. The previous predicted pixel moves to `+0x30`, the new one
  goes to `+0x28`; the per-frame flags `+0x20..+0x23` and the blob link
  `+0x84` (−1) are reset.
- Each blob's viewing ray is tested against the landmarks: a landmark
  closer than 100 mm to the ray (squared distance < 10 000 mm²) and in
  front of the camera sets that blob's bit in the view's near-landmark
  bitsets (`+0x90`, `+0xe0`).

#### E.3.2 Match and score (`0x1800b3090`) [likely]

Matches each projected landmark to a blob within a pixel radius
(2500 px², i.e. 50 px) using the camera's focal length (constants
185.18, 1107.07, 1110.07, 0.0027 in this function) and updates its hit
and miss counters and its score (`+0x88` for A, `+0x118` for B), with the
0.99 factor as decay [guess for the exact rule].

#### E.3.3 New landmarks (`0x1800b43d0`) [likely]

For blobs of camera A and B that matched no landmark: pair them when the
epipolar error is small (|y| ≤ 0.02 in normalised coordinates),
triangulate (`0x1800b24b0`), and reject points within 600 mm of the
reference (squared distance < 360 000), or within 2 m when a flag is
clear (< 4 000 000) [guess: keep controllers in hand out of the map].
Accepted points become landmarks with an initial value 20.0 per camera
(`0x1800bb900` appends).

### E.4 Function map

| Function | Lines | Role |
|----------|-------|------|
| `0x1800b40e0` | 135 | one view update (§E.3) [sure] |
| `0x1800b53c0` | 816 | projection and ray test (§E.3.1) [likely] |
| `0x1800b3090` | 816 | match and score (§E.3.2) [likely] |
| `0x1800b43d0` | 818 | new landmarks (§E.3.3) [likely] |
| `0x1800b24b0` | 477 | two-view triangulation used by §E.3.3 [likely] |
| `0x1800b2e20` | – | helper of `0x1800b3090` |
| `0x1800bb900`, `0x1800bbbe0` | – | landmark list append / bitset grow |
| `0x1800b3fe0` | 64 | slot 3: zero the camera's score array, then write each landmark's score at its blob index (camera 0 → view 0 side A, 1 → view 0 side B, 2 → view 1 side B, 3 → view 2 side B) [sure] |
| `0x1800b6450`, `0x1800b9f00` | 470, 451 | debug overlays (slot 4) [sure] |
| `0x1800b7040` | 145 | Background2 per-camera update → `0x1800b7350`, `0x1800b85b0`, `0x1800b9740` (dead on PC) [likely] |
| `0x1800b8b00` | 822 | Background2 counterpart of `0x1800b3090` (same arguments, shares `0x1800b2e20`; no direct caller) [likely] |
| `0x1800b82f0` | 39 | Background2 slot 3: scores from 288-byte spot records [sure] |
| `0x18004c9c0` | 22 | debug: draws the active model through slot 4 [sure] |

### E.5 Where the score goes [likely]

Traced 2026-11-25. The per-blob score array (`+0xc8` of the frame's blob
store) is copied into each camera manager's store (`0x1800a0480`,
manager `+0x18160`). The blob collector `0x1800e2330` (called by the
boot and tracking paths `0x1800d5070`, `0x1800db980`, `0x1800dbda0`,
`0x1800dc210`, `0x1800dc680`, `0x1800dcb00`, `0x1800e2a00`) copies it
into its filtered list (array `+0x68`) next to position, size and id,
and `0x1800e2a00` keeps it in step when it shuffles that list. Its blob
filter tests the blob **flags** (`+0xa8`: bits 0–2, set by the LD decode,
not by the background model), the size and the radius, never the score.
No function in `0x180090000–0x180110000` reads the score as a number; the
only reader is the debug overlay (slot 4). So on PC the background model
is computed every frame but **does not change the controller solve**,
and a port can leave it out without changing Sony's result.

Still open (only matters for the overlay): the exact matching and
scoring rule in `0x1800b3090`.

---

## Part F: Occluded latch and arm model

Sony `driver_playstation_vr2_orig.dll`, PE timestamp 1741940190, image base
`0x180000000`. Written from the decompile (IDA 9.5) in our own words; no
Sony code is reproduced. Tags: **[sure]** read in code, **[likely]**,
**[guess]**. These functions sat unnamed in the triage
([overview.md part C](overview.md#part-c-function-triage) §C.3.2) as the four largest undocumented core
functions.

After the optical solve (`0x180189c70`) the IR tracker thread
(`0x18018aaf0`) runs two more stages on every LD frame, both enabled by
default on PC:

1. the **occluded-controller latch** (`0x1801b81c0`, flag `irt+0x4d26c`):
   when the farther controller loses its LEDs behind the nearer one, its
   position is carried along with the nearer controller;
2. the **arm model** (object at `irt+0x4d380`, functions
   `0x1801b0120–0x1801b9d70`, mode `irt+0x4d378`): a shoulder–elbow–wrist
   model driven by the head pose and the controller orientation. It feeds
   the controller UKF a pseudo position measurement when the optical fix
   is stale.

`irt` is the IR tracker object (`qword_180355510`). Frames: the latch works
in the **camera-rig frame with +z forward** (the head pose composed with
the head-from-rig transform, then turned 180° about y, `0x1801b7e40`).

### F.1 Driver summary

What a Linux driver gains, in priority order (all userspace):

1. **Keep a controller tracked through short occlusion by the other
   controller.** Two-handed grips and crossed hands hide the far
   controller's LEDs behind the near one. Sony keeps the far controller's
   position rigidly attached to the near one, as seen from the cameras,
   for as long as the near one stays tracked, and blends back over at most
   0.5 s when the far one is seen again (§F.2).
2. **A real arm model as the no-optics fallback.** Sony's fallback
   position is not "hold the last position": an arm model gives a
   plausible hand position from the head pose and the controller
   orientation, and it is fed to the UKF as a measurement once the optical
   fix is older than 0.3 s (§F.3).

The Linux driver has neither: `controller_device.cpp` uses its own fixed
arm offset when there is no optical fix, and nothing handles occlusion by
the other controller.

### F.2 Occluded-controller latch (`0x1801b81c0`) [sure unless tagged]

#### F.2.1 Call and inputs

Called once per LD frame, after the optical solve, while `irt+0x4d26c` is
set (default 1; setter `0x180192a50`, no caller found in the DLL):

- `latch` object at `irt+0x4d288`;
- an `OcclusionQueriesVrTracker` object (vtable methods: head pose at a
  time, head-from-rig transform);
- frame time `t` (host µs);
- the per-hand optical result of this frame: valid byte, position,
  orientation, for both hands;
- the per-hand "last optical position" slots (`irt+0x44158`) and "last
  optical fix time" slots (`irt+0x44148`);
- `0.3` (s): a hand counts as **optically tracked** when its last fix is
  less than 0.3 s old.

It asks the controller UKF (`0x18005df40`) for both hands' states at `t`
and at `t + 100 ms`, and expresses the positions in the rig frame.

Return value: the latched hand (0 or 1), −1 when idle, −2 when a latch was
just dropped. The IR thread stores it at `irt+0x4d270`. For a latched hand
it restores that hand's "valid" byte in the stored frame and stamps
`irt+0x4d278 + 8·hand` with `t`. The optical solve reads the latched
position and that time stamp back as its prediction hint for the next
association (`0x180189c70`, globals `0x18035be90` / `0x18035bed8`).

#### F.2.2 Roles

- **carrier** (`latch+0x40`): the nearer controller (smaller rig-frame z);
- **latched** (`latch+0x44`): the farther one, the one that is hidden.
- If both z are equal, or either is ≤ 0 (behind the cameras), no latch.

#### F.2.3 Entry (state 0 → 1)

All must hold:

- at least one hand has a UKF state;
- the carrier is optically tracked (fix < 0.3 s), the latched hand is not;
- the latched hand's last fix is less than 0.6 s old;
- the latched hand is less than 1 m from the rig origin;
- the latched hand has no bit `0x20` in its UKF status word (`+0x90`)
  [meaning of the bit: guess, "no IMU"];
- the two controllers **overlap as seen from the cameras**. Each
  controller is treated as a sphere of radius 0.0875 m; let `ρc`, `ρl` be
  their angular radii at their distances and `θ` the angle between the two
  view rays. Overlap means `θ ≤ min(20°, |ρc − ρl| + 5°)`, and the view
  rays must be less than 90° apart (positive dot product);
- timing: either less than 0.3 s since the last release or drop
  (`latch+0xe8`), or at least 0.3 s since the latched hand's last fix
  (the second is already implied by "not optically tracked").

On entry the latch records, for `t` and `t + 100 ms`, the transform from
the carrier's view-ray frame to the latched controller's view-ray frame.
A view-ray frame (`0x1801b91d0`) sits at the controller position with its
z axis along the ray from the rig origin. So the latch keeps the hidden
controller where it was **relative to the line of sight to the carrier**
(angular offset and depth difference), not relative to the carrier's own
orientation.

#### F.2.4 States

| State | Meaning | Output for the latched hand | Leaves when |
|------:|---------|-----------------------------|-------------|
| 0 | idle | nothing (returns −1) | entry conditions (§F.2.3) → 1 |
| 1 | latched | position = carrier view-ray frame · recorded transform; orientation = the hand's own UKF orientation at `t` (and at `t + 100 ms` for the prediction slot). Written into the frame result as if it were an optical fix. | carrier has no optical result this frame, carrier ≥ 1 m away, or status bit `0x20` on the latched hand → 3; latched hand has its own optical result this frame → 2 |
| 2 | release | each frame the held position moves toward the UKF position by the fraction `(t − t_release) / 0.5 s` (so it reaches it within 0.5 s) | 0.5 s elapsed, or held position more than 0.15 m from the UKF position → 0 |
| 3 | dropped | hold the last latched position, valid byte cleared (returns −2) | latched hand gets a fix again → 0 |

Positions handed downstream are interpolated between the `t` and
`t + 100 ms` slots by `0x1801b7da0`: `p(τ) = p_t + (p_{t+0.1} − p_t)·(τ − t)/0.1 s`.

#### F.2.5 Constants

| Value | Meaning |
|------:|---------|
| 0.3 s | "optically tracked" window; also the re-latch timing gate |
| 0.6 s | longest gap since the latched hand's last fix |
| 1 m | largest rig-frame distance of either controller |
| 0.0875 m | controller radius used for the overlap test |
| 20°, 5° | overlap cap and margin |
| 0.5 s, 0.15 m | release blend length and abort distance |
| 100 ms | look-ahead of the second recorded transform |

### F.3 Arm model (`irt+0x4d380`)

#### F.3.1 Frames, hands, inputs [likely for the frame, sure for the numbers]

- Works in the tracker world frame with the **full head pose** `H`: the
  INS head pose composed with two config-manager transforms (IR thread,
  `0x18018aaf0`, around the `0x180156470` calls). Shoulders rotate with
  the whole head, including pitch and roll [sure].
- Axes are OpenVR-like: +x right, +y up, +z back. Evidence: the elbow hint
  moves to +z (behind) for a low hand and to −z (in front) for a raised
  hand, and the wrist sits at +z behind the grip [likely].
- **Hand 0 is right, hand 1 is left** (hand 0 uses the +x tables), matching
  the mirrored left model in part C [likely].
- Quaternions in these functions are **w first**.
- Per hand and frame the IR thread passes the UKF-predicted controller pose
  (position + orientation) and whether this LD frame had an optical result.

#### F.3.2 Body constants (`0x180351430`–`0x180351640`) [sure]

| Item | Value (hand 0 / hand 1) |
|------|-------------------------|
| Shoulder offset from the head origin, head frame | (+0.175 / −0.175, −0.275, 0) plus neck (0, 0.0897, z) with z = −0.0978, except in mode 1 where z = 0. On PC: (±0.175, −0.185, 0) |
| Upper arm, forearm | 0.28 m, 0.27 m (`0x1801afc40` arguments; table entries `+0xe0/+0x150`, `+0xfc/+0x16c`) |
| Grip to wrist | 0.08 m along the controller's +z (table entry `+0x00/+0x1c` holds (0, 0, −0.08), applied inverted) |
| Shoulder joint frame | 90° about ±y with offset (±0.0882, −0.0359, 0.0123) (`+0x134` / `+0xc4`) |
| Longest reach | 1.0 m (straight arm beyond it) |
| Forearm cone | the elbow must lie within 75° of the wrist's backward axis (`0x1801afc40`) |
| Elbow hint ("pole") on PC | (±0.4, −0.6, z_h) in the head frame. With `d = clamp(hand_y − head_y, −0.4, 0)` and `f = (d + 0.4) / 0.4`: `z_h = (1 − f)·0.4 − f·0.3`, so +0.4 (behind) for a hand at least 0.4 m below the head, −0.3 (in front) at head height and above |
| Elbow hint, other mode | (±0.3, −0.6, 0.15) (`0x1803515e8/f8`), not used on PC |
| Swing–twist axis | the bone's z axis (`0x1803515b8` = (0, 0, 1)) |
| Twist limit | ±60° (`0x1801b1280`) |
| Hand mask | `0x1803515c4` = 3 (both hands take the PC branch) |

#### F.3.3 Mode and the dead reference-pose path [sure]

`irt+0x4d378` is 1 from init and nothing changes it. `0x1801b9d70` copies
it to `0x180597460` each frame. In mode 1 it also checks two stored
reference poses (`0x180597470`, one per hand) against the current head pose
(view axes within 90°, up axes within 60°, controller within half the
head's distance) and writes the result to `0x180597464`. The functions
that would capture those references (`0x1801b9640`, `0x1801b9880`) have no
caller in the DLL, so on PC the check always fails (`0x180597464` = 0) and
the "other mode" branches of `0x1801b37b0` / `0x1801b1280` never run.
`0x1801b37b0` overwrites the slot with the current head pose every frame,
so the slowly following "torso" orientation in `0x1801b1280` (2 % slerp
toward the head when the view direction changes by more than 0.5°) ends up
equal to the head orientation on PC.

#### F.3.4 Per frame, per hand (IR thread) [sure unless tagged]

Only for a hand whose UKF state is valid (otherwise `0x180185bd0` resets
that hand's arm state):

1. **Input smoothing** (`0x1801b08b0`, mode 1 only): keeps the UKF state
   0.1 s ahead and, when the optical fix comes back after a gap, blends
   the stored position into the new one over a configured number of
   frames (fields `+0x2e0..+0x2f4` per hand, stride 0xe0) [likely for the
   purpose].
2. **IK check** (`0x1801b37b0`, PC branch): wrist = grip + R·(0, 0, 0.08);
   shoulder from §F.3.2; two-bone IK with the hint (`0x1801b0120`: law of
   cosines, elbow bent toward the hint's component perpendicular to the
   shoulder→wrist line), then the 75° forearm cone (`0x1801afc40`). The
   elbow it finds is not kept. The function records the controller and
   head orientations, resets the torso angle, and returns whether the
   wrist is reachable. While a hand has no optical result it latches
   "arm raised" (`+0x222 + hand`) = wrist above shoulder − 0.1 m.
3. **Out-of-view test** (`0x1801b9560`, only if the hand moved
   (|v| ≥ 0.0125) within the last 1.5 s, or a debug flag is set): true
   when the predicted controller is **at least 80° off the view axis**
   (head −z pitched down 5°, `0x1801b93f0`) **and within 1.0 m of its
   shoulder**.
4. **Pseudo measurement** (`0x1801b1280`), when the out-of-view test is
   true **or** the last optical fix is older than the stale limit
   (default 0.3 s; tracker setting `irt+0x442f8` in µs: > 0 sets it,
   < 0 disables it):
   - forward kinematics from the shoulder through shoulder, elbow and
     wrist joint rotations (state `+0x23c/+0x25c/+0x27c` per hand,
     stride 0x10), driven by the controller orientation. The wrist joint
     is slerped 75 % toward the needed rotation, twists are limited to
     ±60° about the bone axis (swing–twist, `0x1801afaf0`), and the elbow
     solution (up or down) follows the "arm raised" flag, flipped when the
     hand moves more than 1.5 cm (joint limits in §F.3.5);
   - output per hand (stride 0x50 from the object start): +0x10 elbow,
     +0x1c wrist, +0x28 **hand position**;
   - the return value is an angle in radians; the IR thread accepts it if
     it is ≤ 20, which every angle meets (looks like a degrees/radians
     slip) [sure for the code];
   - the hand position goes to the UKF as a **position-only**
     measurement (`0x18005ec00`, rot_free = 1, float argument 1000 at
     record +0x34). The mode argument is 1 while the model agrees with the
     UKF prediction within 0.1 m and the model is valid, else 0; per
     part G §G.4 that selects σ = 0.005 m or σ = 0.05 m.
5. Statistics: distance moved since the last frame, binned at
   0.05/0.1/0.2/0.3/0.4 m into counters (`irt+0x4f4a4`, `irt+0x4f4b0`),
   split by fixes older than 2 s [likely: diagnostics only].

#### F.3.5 Joint limits of the FK chain [sure unless tagged]

Each joint function takes the parent bone's world pose and the child pose
the chain wants, expresses the child rotation in the parent frame
(`q_local = conj(q_parent) ⊗ q_child`), splits it into a **twist** about
the bone axis z (`0x1801afaf0`) and the remaining **swing**, clamps the
angles (`0x1801b0f50` axis–angle, `0x18016f2d0` rebuild), and composes
the result back onto the parent. If nothing was clamped the child pose is
returned unchanged. Hand 1 mirrors the sign of the hinge axis or offset.

| Joint | Function | Parent frame | Twist about z | Swing |
|---|---|---|---|---|
| Shoulder | `0x1801aefb0` | parent rotated by 45° about y (−45° for hand 1) | clamped to ±90° | clamped to ±100 **rad**, so never limited (another degrees/radians slip) |
| Elbow | `0x1801aeaf0` | parent | clamped to ±100° | only its component about **+y** (−y for hand 1) is kept, clamped to 0..160°; the rest of the swing is dropped (a hinge) |
| Wrist | `0x1801af610` | parent | removed (clamped to 0) | clamped to ±75° |

So the forearm can twist ±100° at the elbow, the wrist bends up to 75° in
any direction but cannot twist, and the elbow bends one way only.

#### F.3.6 Pose-output helpers (`FUN_180189c70`) [sure unless tagged]

- `0x1801b11e0` (arm-model object `irt+0x4d380`): when the hand's
  model flag `+0x29c + hand` is set, return the model hand position at
  time t, linear between the two slots `+0x2a0 + 12·hand` and
  `+0x2b8 + 12·hand`, with `(t − slot time) / 100 000 µs` as the weight
  (the slot time is at `+0x2d0 + 8·hand`).
- `0x1801b99a0` (status 6, "OcclusionQueries"): takes the head pose at t
  (`OcclusionQueriesVrTracker` vtable `+0x10`) composed with the config
  transform (`+0x08`), pitches the view axis 5° down, and if the held
  controller position lies **more than 80° off the view axis**
  (`0x1801b93f0`), rotates it about the eye point onto the 80° cone. The
  caller uses it only when `irt+0x441d8 + hand` is set. That byte is
  zeroed by the IR-tracker constructor and nothing in the DLL writes it,
  so on PC status 6 does not occur [likely].

### F.4 Function map

| Function | Role |
|----------|------|
| `0x1801b81c0` | occluded-controller latch (§F.2) |
| `0x1801b7e40` | both hands' UKF states into the rig frame (head pose at `t`, head-from-rig, 180° about y) |
| `0x1801b91d0` | view-ray frame of a point: rotation taking +z to the ray, translation along it |
| `0x1801b7da0` | latched position at a time, linear between the `t` and `t + 100 ms` slots |
| `0x1801b7d90` | `OcclusionQueries` destructor (vtable reset) |
| `0x180192a50` | set latch enable `irt+0x4d26c` (under the IR lock) |
| `0x1801852e0` | IR tracker init: latch enable 1, latched hand −1, arm-model mode 1, arm-model object init `0x180184fa0` |
| `0x180184fa0` | arm-model object init (zeroes, identity joints) |
| `0x180185bd0` | reset one hand's arm state (no UKF state) |
| `0x1801b9d70` | per-frame mode copy and reference-pose check (§F.3.3) |
| `0x1801b9640`, `0x1801b9880` | reference-pose capture; no callers (dead on PC) |
| `0x1801b08b0` | input smoothing / re-entry blend (§F.3.4 step 1) |
| `0x1801b37b0` | IK check and orientation bookkeeping (§F.3.4 step 2); 1500 lines of other-mode code unused on PC |
| `0x1801b1280` | forward-kinematics pseudo measurement (§F.3.4 step 4) |
| `0x1801b9560`, `0x1801b93f0` | out-of-view test: ≥ 80° from the view axis pitched 5° down, ≤ 1 m from the shoulder |
| `0x1801b0500` | returns the reach limit 1.0 |
| `0x1801b0510`, `0x1801b05c0` | shoulder point: head pose · (shoulder offset + neck offset) |
| `0x1801b0600` | shoulder pose (point + joint-frame rotation) |
| `0x1801b0120` | two-bone IK with pole hint and reach clamp |
| `0x1801afc40` | two-bone IK plus 75° forearm cone |
| `0x1801afaf0` | swing–twist: twist of a quaternion about an axis |
| `0x1801b0f50` | quaternion → axis–angle, angle in (−π, π] |
| `0x180170dc0` | slerp (shortest path; lerp when cos > 0.9999) |
| `0x1801aefb0`, `0x1801aeaf0`, `0x1801af610` | shoulder, elbow, wrist joint limits (§F.3.5) |
| `0x1801b0dc0`, `0x1801b0e70`, `0x1801b10b0` | other-mode helpers of `0x1801b37b0` (unused on PC) |
| `0x1801b11e0`, `0x1801b99a0` | model hand position at t; status-6 view-cone clamp (§F.3.6) |

---

## Part G: Controller UKF

Interoperability notes on how Sony's driver fuses the optical controller pose
with the controller IMU. Own words, no Sony code. Addresses are in the build
listed in [overview.md part B](overview.md#part-b-driver-architecture-and-code-map) (IDA `sub_X` = Ghidra
`FUN_X`). Tags: **[sure]** read from code, **[likely]** from context,
**[guess]** untested.

Earlier notes: [part A](#part-a-tracking-overview) §A.5.5 (P0, history, stationary
damping, divergence guard), [part H](#part-h-ukf-kernels) (process noise,
packed covariance, quaternion kernels), [part C](#part-c-optical-pose-into-steamvr) §C.4
(optical feed, innovation gate, config block). This file adds the filter
structure, the measurement model and its noise.

### G.1 Objects [sure]

- `UkfManager` is a singleton (0x4460 bytes, vtable `0x180318c80`) holding two
  per-hand filters (0x2218 bytes each, from `+0x28`) and a 32-byte optical
  noise block at `+0x08` (§G.4). It is set up by `0x18005d5d0`.
- Each per-hand filter has a state ring (0x1c8-byte entries, one per IMU
  sample) and a measurement ring (0x1e0-byte entries).
- A state entry holds, among others [likely for names, sure for offsets]:
  - `+0x00` angular rate and `+0x10` acceleration of that IMU sample (the
    process-noise inputs, part H);
  - `+0x40` position, `+0x50` velocity (float3, m, m/s);
  - `+0x60` orientation as **double** quaternion x, y, z, w;
  - `+0xc0`, `+0xd0` gyro and accel bias (float3);
  - `+0x108` time (µs).
- The covariance is the 15×15 packed upper triangle of part H.
- An optical measurement is a `DotMarkerData` object (vtable `0x180318c90`,
  base class `OIFusion::OpticalMarkerData`). The filter only talks to it
  through virtual calls (§G.3), so other measurement types can plug in.

### G.2 Filter type [sure unless tagged]

An **error-state, augmented, symmetric sigma-point filter**:

- error state: 15 (position, velocity, attitude, gyro bias, accel bias; block
  order as part A §A.5.5);
- augmented with **6 IMU noise** terms (gyro, accel) and the **m measurement
  noise** terms: n = 15 + 6 + m (`0x1800942f0`, set up by `0x180092630` and
  `0x180091930`);
- **2n + 1 sigma points**: the central point plus `±sqrt(n)` along the
  columns of the Cholesky factor. The unscented transform with α = 1,
  β = 2, κ = 0 (n + λ is computed as `n + 0 − n + n`, so λ = 0): the
  central point has mean weight 0 and covariance weight **2**, the others
  `1/(2n)` for both. So the mean is the cubature mean, but the predicted
  covariances get an extra `2·(y₀ − ŷ)(y₀ − ŷ)ᵀ` term
  (`function-catalogue.md` §B.1; an earlier version of this file said
  there was no central point).

One filter step (`0x180065b70`, "kalmanFilterStep") covers the interval from
the newest filter state to the measurement time in one go:

1. The IMU samples inside the interval are reduced by `0x180082e90` to one
   **time-weighted mean** angular rate and acceleration (bias-corrected with
   the state's biases) plus the mean of their per-sample process-noise
   variances (`0x180062880`, `0x180062950`). Up to 31 samples; if the span
   reaches the config time (`+0x2168` in the manager) the step is split.
2. Each sigma point is propagated with that constant mean rate and
   acceleration over the interval and mapped to a predicted measurement in
   the same call (`0x18006f020`, §G.2.1); the averaged variances are the 6
   IMU noise terms of the augmentation.
3. The propagated points go through the measurement's `h(x)` (§G.3); mean,
   innovation covariance and cross covariance give the gain; the correction
   is applied to the nominal state (`0x18006c760`, attitude by the rotation
   vector, `0x18006c1b0` writes the covariance back).
4. If a flag on the filter is set, the accel-bias block of the covariance is
   rewritten from the state after the update.
5. The stationary damping (`0x18006e5a0`, part A §A.5.5) is called
   only when the IMU history reached the split time in step 1, that is on
   long intervals without measurements [sure for the call site].
6. Return codes of the step: 0 ok, −1 numeric failure, −2 a specific
   solver failure. Return codes of the solver `0x1800942f0`: 0 ok, −1 a callback failed,
   −3 the augmented covariance is not positive definite, −4 `DPPTRF` of the
   innovation covariance reported a bad argument. The fixed-size transforms
   return −2 or −3 for a failed Cholesky.

Delayed measurements: a measurement older than the newest state is applied at
its own time and the IMU states after it are replayed (`0x18007add0`,
part A §A.5.5). The first valid measurement initialises the filter
(`0x180068620`) with P0 from part A §A.5.5.

#### G.2.1 Sigma-point function (`0x18006f020`) [sure unless tagged]

The solver calls one function per sigma point that does both the process
step and the measurement prediction. `0x1800638d0` (passed to the solver
setup) is only its work-area allocator: it rounds a bump pointer up to an
alignment.

Sigma vector (doubles, converted to float), n = 21 + m:

| Index | Content |
|---|---|
| 0..2 | δp, position (m) |
| 3..5 | δv, velocity (m/s) |
| 6..8 | δθ, attitude error (generalised Rodrigues parameters, see below) |
| 9..11 | gyro bias error |
| 12..14 | accel bias error |
| 15..17 | accel noise |
| 18..20 | gyro noise |
| 21.. | the m measurement-noise terms |

The nominal record holds the window means `f̄` (accel, g units) at
`+0x00`, `ω̄` (gyro, rad/s) at `+0x18` and the attitude `q̂` (x, y, z, w) at
`+0x30`, all doubles. The context holds `dt`, the IMU → controller
rotation `M` and lever arm `l` (§A.5.6) and the measurement object.

```
// attitude error → quaternion: GRP with a = 1, f = 4 (USQUE form)
s   = |δθ|²
δq.w = (16 − s) / (16 + s)
δq.v = δθ · (1 + δq.w) / 4
q    = δq ⊗ q̂

ω    = ω̄ − δb_g + n_g
f    = f̄ − δb_a + n_a                          // g units
q'   = integrate(q, ω, dt)                     // 0x18006bcd0
v'   = δv + (R(q)·f − (0, 1, 0)) · 9.80665 · dt   // world Y up
r    = R(q')·l                                 // lever arm in world axes
ωw   = R(q')·ω
pose.R = R(q')·M                               // 0x1800621a0
pose.t = δp + (δv + v')/2 · dt + r
v_pt   = v' + ωw × r                           // passed along, unused by h
y      = h(pose) + n_meas                      // vtable +0x48
```

`h` (`0x180060030`, `DotMarkerData` slot `+0x48`): fails if the record is
not valid. `y[0..2] = pose.t`; unless position-only,
`y[3..11] = pose.R · (0.1, 0, 0)`, `pose.R · (0, 0.1, 0)`,
`pose.R · (0, 0, 0.1)`: the three 10 cm tips **relative to the
position** (not added to it). This matches the measured scalars of §G.4.

The position and velocity in the sigma vector are therefore offsets from
the nominal state's frame of this step (the nominal record carries no
position or velocity); `0x18006c760` adds the corrected error back to the
nominal state [likely for the frame bookkeeping, sure for the formulas
above].

### G.3 Measurement interface (`DotMarkerData` vtable) [sure]

| Slot | VA | Use |
|---|---|---|
| +0x08 | `0x18005f910` | position (and zero velocity) if valid |
| +0x10 | `0x18005f950` | measured rotation as a 3×3 matrix (none if position-only) |
| +0x18 | `0x18005fad0` | fills two diagonal 3×3 blocks, 1e-4 and 1e-6 [likely: a default covariance; the 12-scalar path does not use it] |
| +0x20 | `0x18005fc00` | dimension: 0 invalid, 3 position-only, 12 full |
| +0x28 | `0x18005fc20` | scalar measurements and their variances (§G.4) |
| +0x30, +0x38 | | 2-D and 3-D blocks: none (return 0) |
| +0x40 | | returns 1 |
| +0x48 | `0x180060030` | predicted measurement `h(pose)` (§G.2.1) |
| +0x50 | | timestamp (`+0x88`, µs) |
| +0x58 | | byte `+0x30` |
| +0x60 | | the vector at `+0x38` (§G.4) |

Record layout written by `0x18005ec00` (0xb0 bytes):

| Off | Content |
|---|---|
| +0x08 | position (float3, m) |
| +0x18 | orientation quaternion x, y, z, w (normalised by the feed) |
| +0x28 | valid |
| +0x29 | position-only ("rot_free" argument) |
| +0x2c | int mode argument (1 selects the tight position-only noise) |
| +0x30 | flag (set unless position-only and per-hand flag `+0x4458`) |
| +0x34 | float argument |
| +0x38 | float3 argument: the controller position in the head-relative frame used by §G.4 [likely] |
| +0x48, +0x58, +0x68, +0x78 | one float3 per camera from the tracker frame (stride 0x1688), norms in mm |
| +0x88 | timestamp (µs) |
| +0x90 | copy of the manager's 32-byte noise block |

### G.4 Measurement content and noise (`0x18005fc20`) [sure unless tagged]

**Content.** 12 scalars for a full pose, 3 for position-only:

- 3: the position;
- 9: the measured rotation applied to three **0.1 m axis vectors**
  (table at `0x180351880`: (0.1,0,0), (0,0.1,0), (0,0,0.1)), i.e. the tips
  of a 10 cm body frame. Rotation error enters the filter as metres of tip
  displacement, so position and rotation residuals share one unit.

**Noise.** Each scalar gets an independent variance σ².

- Position-only: σ = 0.005 m if the mode argument is 1, else 0.05 m.
- Full pose: σ_pos and σ_rot come from `0x1800602d0`, a test against an
  **ellipsoid around the head** with semi-axes (0.30, 0.20, 0.25) m:
  - inside: (σ_rot, σ_pos) = (B_rot, B_pos); outside: (A_rot, A_pos);
  - within ±0.04 m of the ellipsoid surface (measured along the ray from the
    centre) the two are blended linearly in variance:
    `σ² = w·B² + (1 − w)·A²`, `w = (r_surface + 0.04 − r) / 0.08`;
  - the noise block holds A = (0.0, 0.0005) and B = (0.0, 0.0025) as
    (rot, pos) [sure for the values; their order inside the pair is
    likely]. So inside the close zone the position is trusted 5× less.
  - If any per-camera vector is longer than 1.1 m (1100 mm), σ_pos is
    replaced by 0.01 m.
  - σ_rot = 0 from this block [sure for the value]: the rotation tips are
    taken as exact apart from the filter's own state uncertainty. How the
    solver tolerates a zero variance (the augmented noise column is then
    zero) is not checked; a port should floor it.

The innovation gate before the update is in part C §C.4.1 (5 cm
below 1 m/s, else 20 cm).

### G.5 Still open

- ~~Sigma-point process model and `h(x)`~~: done, §G.2.1
  (`0x18006f020`, `0x180060030`).
- Differences of `steamvr/driver_psvr2/src/controller_ukf.cpp` from
  §G.2.1 (read 2026-11-25): it propagates the covariance with a linear
  `Φ P Φᵀ + Q` step and applies attitude errors on the right
  (`q ⊗ exp(δθ)`); Sony pushes every sigma point through the IMU step
  (`0x18006f020`) and applies the error on the left with GRP
  (`δq ⊗ q̂`, a = 1, f = 4), with the IMU noise as augmented sigma
  dimensions (accel noise before gyro noise).
- The UKF solver internals are now in `function-catalogue.md` §B.1:
  packed Cholesky without jitter, inverse of S by `DPPTRF` + `DPPTRI`.
  Open: when `DPPTRF` finds S not positive definite (positive info) the
  code continues with the half-factored matrix instead of failing; whether
  that ever happens in practice is unchecked.
- The manager's split time `+0x2168` and the "use accel bias flag" (`+0x38`
  of the state).
- Config limits listed in `overview.md` §A.11 q9.

---

## Part H: UKF kernels

Sony VA ranges `0x180049240–0x18004cfff` and `0x180050000–0x18006ffff` of
`driver_playstation_vr2_orig.dll` (PE timestamp 1741940190). This file
describes behaviour in our own words; the implementation in
`userspace/lib/sie/ctrltracker/` is written from it, and
`scripts/sony-re/difftest/t_ctrltracker_*.c` compares it with Sony's code
under Wine. Per-function status: `scripts/sony-re/status/ctrltracker.tsv`.

Conventions:

- Floats are IEEE single. Sony's build uses AVX but never fuses
  multiply-add, so ours must be built with `-ffp-contract=off`.
- Sums are evaluated left to right as written, `a + b + c = (a + b) + c`.
  The order of the two factors of a product does not matter (exact).
  Doubling (`2a`, written as `a + a`) is exact, so `(2a)·b = 2(a·b)`.
- Quaternions are stored `x, y, z, w` (w last).
- Matrices are column-major, each column 16 bytes (4 floats), like glm.
  `a[r][c]` is the element at row r, column c = `m[4c + r]`.

### Small vector / quaternion kernels

#### Rotate a vector by a quaternion (`0x180058720`)

`out = rot(q, v)`, out/q/v pointers, returns out. Computed as two Hamilton
products, `p = q ⊗ (v, 0)` then `out = (p ⊗ q̄).xyz`, but with p's scalar
part kept with the opposite sign (`d` below), so no negation is needed:

```
px = (vx·w + vz·y) − vy·z
py = (vx·z + vy·w) − vz·x
pz = (vy·x + vz·w) − vx·y
d  = (vx·x + vy·y) + vz·z
out.x = ((d·x + px·w) − py·z) + pz·y
out.y = ((d·y + py·w) − pz·x) + px·z
out.z = ((d·z + pz·w) − px·y) + py·x
```

The same sequence is inlined in several bigger functions (e.g. the
measurement model at `0x18005fc20`).

#### Quaternion length / normalise (`0x180058900`, `0x180058970`)

Length: `sqrt(((x² + y²) + z²) + w²)`. Normalise: `s = 1 / length`, then
each component times `s` (no zero check: a zero quaternion gives inf/NaN).

#### Quaternion to rotation matrix (`0x180058a10`)

Writes three 16-byte columns; only lanes 0..2 are defined (lane 3 is left
as whatever was in a register; callers ignore it). With `X = x+x`,
`Y = y+y`, `Z = z+z`:

```
col0 = ((1 − Y·y) − Z·z,  Z·w + Y·x,        Z·x − Y·w)
col1 = (Y·x − Z·w,        (1 − X·x) − Z·z,  Z·y + X·w)
col2 = (Y·w + Z·x,        Z·y − X·w,        (1 − X·x) − Y·y)
```

#### Matrix times vector (`0x180058e40`, `0x180058f30`)

4×4 times vec4: `out[r] = ((v1·a[r][1] + v0·a[r][0]) + v2·a[r][2]) + v3·a[r][3]`
for r = 0..3. 4×3 (three 16-byte columns) times vec3: the same without the
last term. Arguments: (matrix, out, vector); returns out.

#### 4×4 inverse (`0x180059000`)

Cofactor expansion using 2×2 minors; arguments (out, in). With `a = in`:

```
m17 = a22·a30 − a32·a20     m18 = a23·a31 − a33·a21
m19 = a22·a10 − a12·a20     m20 = a23·a11 − a13·a21
m21 = a13·a31 − a33·a11     m22 = a12·a30 − a32·a10
c0 = (a12·m18 − a32·m20) − a22·m21
c1 = (a13·m17 − a33·m19) − a23·m22
c2 = (a20·m21 + a30·m20) − a10·m18
c3 = (a21·m22 + a31·m19) − a11·m17
s  = 1 / (((c1·a01 + c0·a00) + c2·a02) + c3·a03)
n27 = a02·a10 − a00·a12     n28 = a03·a11 − a01·a13
n29 = a02·a30 − a00·a32     n30 = a03·a31 − a01·a33
n31 = a02·a20 − a00·a22     n32 = a03·a21 − a01·a23
out col0 = (c0, c1, c2, c3)·s
out col1 = ((a22·n30 − a32·n32) − a02·m18, (a23·n29 − a33·n31) − a03·m17,
            (a30·n32 − a20·n30) + a00·m18, (a31·n31 − a21·n29) + a01·m17)·s
out col2 = ((a32·n28 − a12·n30) + a02·m21, (a33·n27 − a13·n29) + a03·m22,
            (a10·n30 − a30·n28) − a00·m21, (a11·n29 − a31·n27) − a01·m22)·s
out col3 = ((a12·n32 − a22·n28) + a02·m20, (a13·n31 − a23·n27) + a03·m19,
            (a20·n28 − a10·n32) − a00·m20, (a21·n27 − a11·n31) − a01·m19)·s
```

No singularity check (a singular matrix gives inf/NaN).

#### libm

Sony imports `sinf`, `cosf`, `atan2f`, `acosf`, `asinf`, `atanf`, `tanf`,
`expf`, `logf`, `sin`, `cos`, `acos`, `log`, `pow`, `sqrt(f)` from the UCRT.
The difftests route our calls to the same UCRT entry points
(`ct_difftest.h`), so "bit-exact" means bit-exact given the same libm. With
mingw's libm, `sinf`/`cosf` differ by at most 1 ulp (1 % of random angles);
glibc may differ similarly.

#### More kernels

- **vec3 add in place** (`0x180061be0`, (a, b)): `a[i] = b[i] + a[i]`.
- **vec3 length** (`0x180061c60`): `sqrt((x² + y²) + z²)`.
- **cross product** (`0x180061cc0`, (out, a, b)): `a × b`, each component
  `a_j·b_k − a_k·b_j`.
- **quaternion product** (`0x180061fa0`, (p, out, q)): Hamilton `p ⊗ q`:
  `x = ((px·qw + pw·qx) + py·qz) − pz·qy`, `y = ((py·qw + pw·qy) + pz·qx) −
  px·qz`, `z = ((pz·qw + pw·qz) + px·qy) − py·qx`,
  `w = ((pw·qw − px·qx) − py·qy) − pz·qz`.
- **yaw quaternion** (`0x1800620b0`, (out, angle)): `(0, sin(a·0.5), 0,
  cos(a·0.5))`.
- **3×3 product** (`0x1800621a0`, (A, out, B)): column c of the result is
  `A·B[c]`. Column 0 sums as `(B0.y·A.col1 + B0.x·A.col0) + B0.z·A.col2`;
  columns 1 and 2 as `(Bc.x·A.col0 + Bc.y·A.col1) + Bc.z·A.col2`.
- **rotation about Y** (`0x1800623d0`, (out, angle)): columns `(c, 0, −s)`,
  `(0, 1, 0)`, `(s, 0, c)`, `s = sin a`, `c = cos a`.
- **azimuth difference** (`0x1800627f0`, (out_a, out_b, a, b) → float):
  `ta = atan2(a.z, a.x)`, `tb = atan2(b.z, b.x)`, `d = ta − tb`; if
  `d > π` add `−2π`, else if `d < −π` add `2π` (π and 2π as floats).
  Stores ta, tb, returns d. Used for yaw alignment between two frames.
- **uniform random** (`0x180062bc0`, (state[2], lo, hi) → float):
  Marsaglia multiply-with-carry: `s0 = 36969·(s0 & 0xffff) + (s0 >> 16)`,
  `s1 = 18000·(s1 & 0xffff) + (s1 >> 16)`, `u = s1 + (s0 << 16)` (u32),
  `t = float((double(u) + 1) · 2.328306435454494e-10)` (that is
  `1/(2³² + 2)`, so t is strictly inside (0, 1)), result `t·(hi − lo) + lo`.

### Process noise (`0x180062880`, `0x180062950`)

Per-axis variance of a 3-vector state block, driven by the current rate
vector v (angular velocity for the first, the vector at +0x10 of the state
for the second). For axis i with the other two axes j < k:

```
var_i = ((v_i²·own + v_j²·cross) + v_k²·cross) [+ extra] + base_i
```

or `fixed_value` if the axis's "fixed" flag is set. Config offsets
(third argument): first function own +0x30, cross +0x34, fixed value +0x38,
base +0x10..0x18, no extra; flags at state +0x30..0x32. Second function own
+0x40, cross +0x44, extra +0x3c, fixed value +0x48, base +0x20..0x28; v at
state +0x10, flags at state +0x33..0x35. Arguments (state, out, config).

#### Batch 3 kernels

- **vec3 subtract / scale in place** (`0x18006af00`, `0x18006af50`):
  `a[i] = a[i] − b[i]`; `a[i] = s·a[i]`.
- **shortest-arc quaternion** (`0x18006b030`, (out, a, b), a and b unit):
  `d = ((a·b summed x, y, z) + 1)`, `s = sqrt(d + d)`, `xyz = (a × b)·(1/s)`,
  `w = s·0.5`. No guard for opposite vectors (d = 0 gives inf/NaN).
- **double quaternion normalise / product** (`0x18006b250`, `0x18006b310`):
  as the float versions, in double.
- **sphere-cap quaternion** (`0x18006b4a0`, (out, v, c, r) floats;
  `0x18006b5c0` the same with double c, r and double output):
  `n = |v|²` (float), `t = sqrt((1 − c²)·n + r²)`,
  `w = (t·r − n·c) / (r² + n)`, `k = w + c`, `xyz = (v_i·(1/r))·k`,
  output w = `w`. The double version computes n in float, then t and w in
  double, rounds `k` and `1/r` to float and evaluates `(v_i·(1/r))·k` in
  float before widening.
- **duplicate**: `0x18006b700` is byte-for-byte the same routine as
  `0x1800627f0` (azimuth difference).

#### Rotation matrix to quaternion (`0x180061d40`, (out, m))

m is three 4-float columns. Ghidra's decompile of the flag logic is wrong
here; this is from the disassembly.

1. `neg = 0 > (m11 + m00) + m22` (NaN: false).
2. Three flags, each "negate one diagonal element" (all false unless neg):
   - `fz = neg ∧ ¬(m22 > m00 ∧ m22 > m11)` negates m22 and m01;
   - `fx = neg ∧ (m11 > m00 ∨ m22 > m00)` negates m00 and m12;
   - `fy = neg ∧ ¬(m11 > m00 ∧ ¬(m22 > m11))` negates m11 and m20.
   So with a negative trace every diagonal element but the largest is
   negated.
3. `t = ((m11 + m00) + m22) + 1` and `h = 0.5 / sqrt(t)` with the negated
   values; `d_yz = m21 − m12`, `d_zx = m02 − m20`, `d_xy = m10 − m01` (the
   subtrahends possibly negated).
4. Lanes `p = fz ? (t, d_zx, d_xy, d_yz) : (d_yz, d_xy, d_zx, t)`; output
   `fx ? (p2, p0, p3, p1) : (p0, p2, p1, p3)`, each times h. This yields
   the usual "largest component first" formulas: w-, x-, y- or z-largest.

#### Batch 4 kernels

- **relative yaw** (`0x18006b790`, (p, q) → float): `r = p ⊗ q̄` (q̄ =
  conjugate, xyz negated); `u = rot(r, (0,1,0))`; `c = shortest-arc(u →
  (0,1,0))` removes the tilt; `g = c ⊗ r`; `e = rot(g, (1,0,0))`; return the
  azimuth difference of `(1,0,0)` and `e` (that is `atan2(0,1) −
  atan2(e.z, e.x)`, wrapped). All steps are the kernels above evaluated with
  literal 0/1 components (products by 0 are kept: they matter for inf/NaN
  and signed zeros).
- **exponential map** (`0x18006bb50`, (out, v)): `len = |v|`, `half =
  len·0.5`. If `len < sqrt(FLT_EPSILON)` (`sqrt(1.1920929e-7)`, computed
  once at first use) then `k = (len·len)/48 + 0.5`, else `k = sin(half) /
  len`. `xyz = v·k`, `w = cos(half)`. (The series has the wrong sign — the
  true one is `0.5 − len²/48` — but below the threshold the term is under
  half an ulp of 0.5, so it never changes the result.)
- **attitude integration** (`0x18006bcd0`, (out, q, w, dt); q/out double,
  w/dt float): `v = dt·w` (float), `e = exp(v)` as above (float), widen e
  to double, `out = normalize(q ⊗ e)` in double.
- **reference setters** (`0x18006bf80`, `0x18006bfd0`, (obj, ref)): copy the
  4-float ref to obj+0xc0 (resp. +0xd0) and store `obj[0..2] − ref` (resp.
  `obj+0x10` vector minus ref) at obj+0xe0 (resp. +0xf0); lane 3 undefined.

### Packed covariance (15 × 15)

The filter state has 15 error components in five 3-blocks (block base
b = 0, 3, 6, 9, 12). Its covariance is stored as the packed upper triangle,
row-major, 120 floats: element (i, j), i ≤ j, lives at
`((29 − i)·i)/2 + j`.

- **reset block** (`0x18006c020`, (P, b)): for rows i = b..b+2 (only
  i < 15) clear the row from the diagonal to the end; for every earlier row
  r < b clear the three entries in columns b..b+2; then put 1.0 on the three
  diagonal entries.
- **set block** (`0x18006c1b0`, (P, b, M)): reset as above, then store the
  upper triangle of the 3×3 matrix M (three 4-float columns): (b+r, b+c) =
  `M[r][c]` for r ≤ c, where `M[r][c]` is lane c of column r (M is used as
  symmetric).
- **get block** (`0x18006c100`, (out, P, b)): read the six upper entries of
  the diagonal block and mirror them; lane 3 of each output column is not
  written.

### Lever-arm transfer (`0x18006c340`)

Arguments (out_pos, out_vel, out_rot, pos, vel, R, ω, B, lever). With
`l = R·lever` (4×3 product order as `0x180058f30`):
`out_pos = l + pos`,
`out_vel = (l.z·ω.y − l.y·ω.z, l.x·ω.z − l.z·ω.x, l.y·ω.x − l.x·ω.y) + vel`
(that is `ω × l + vel`, each sum done as `(cross term) + vel_i`),
`out_rot = R·B` via the 3×3 product `0x1800621a0` (48 bytes copied, lane 3
undefined). Moves a pose and its velocity from the body origin to a point
fixed on the body (e.g. IMU to controller origin).
