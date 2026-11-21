# Optical controller pose into SteamVR (Sony driver)

Sony `driver_playstation_vr2_orig.dll`, PE timestamp 1741940190. Addresses
are VAs, image base 0x180000000. Tags: **[sure]** read in code (or called
under Wine), **[likely]**, **[guess]**. Written from `FUN_1800af650`,
`FUN_1800b02a0`, `FUN_1800cbd70`, `FUN_180173fe0`, `FUN_18005ec00`,
`FUN_180189c70`, `FUN_180160230`, `FUN_18018aaf0` (IR thread) and the VD
dispatcher at `0x180170xxx` (`case 0xd`, decomp line ~171011).

Status: partial. Done: Q1 rig table, calibration composition and the
head chain `INS · A⁻¹ · B` with numbers (§2), type 13 layout and route
(§2.3–2.5), Q2 IMU offset and hand-1 mirror (§2b), the optical
measurement content, its innovation gate and the UKF config defaults
(§4), and the output/timeout path (§5). Open (§8): whether X is exactly
the rig frame (needs one HW check), which config entries are the
measurement noise, the solver's per-frame acceptance rule,
two-controller association, LD time offset.

## 1. Driver summary

What the Linux driver must do, in order (all userspace):

1. Build the camera rig **from Sony's built-in table, not from camera 0's
   frame**. Camera 0 keeps its built-in pose; cameras 1–3 are camera 0's
   built-in pose composed with the calibrated extrinsics (§2). The rig
   frame is a headset-fixed frame in mm: +x = wearer's left, +y = up,
   +z = forward [likely, from the camera positions and the HW cover test].
2. The optical solve outputs a controller pose (position + quaternion)
   that the IR thread hands to the controller UKF as a full-pose
   measurement (§4).
3. Report the UKF pose, but **hold the last optical position** when no
   optical update has arrived for **0.3 s**; keep the rotation from the UKF.
   Report a status code 9 (optical fresh), 3 (position held) or 6
   (position from the fallback model), and `poseIsValid = 0` only when the
   tracker has no state at all (§5).
4. Apply the SteamVR tip/grip transform (already in `steamvr.md`
   "Controller pose").

Nothing here belongs in the kernel.

## 2. Camera rig (Q1)

### 2.1 Built-in rig table (`FUN_1800af650(out, variant, cam)`) [sure, called under Wine]

Output: a column-major 4×4 float, `X_rig = R·X_cam + t`, mm. `X_cam` is in
Sony's camera frame (the OpenCV frame rotated 180° about z, RESEARCH §8.2),
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
§8.4 HW data: cams 0/1 look forward, 2/3 up and out]. Variants 1/2 also toe
cams 0/1 out by 15° (c2.x = ±0.259) — this matches the HW measurement
"0/1 toed out 15° each" (RESEARCH §8.4), so **the measured headset is
variant 1 or 2, not 0** [likely]. Baselines: var 1 cam 0–1 76.4 mm,
2–3 132.0 mm; var 2 80.0 / 135.0 mm; measured 78.8 / 133.2 mm.

The variant is the tracker field `param_1[0x3b]` passed by
`FUN_180049240` (decomp ~9107). Its source was not traced (§8).

### 2.2 Calibration on top of the table (`FUN_180049240`, `FUN_1800b02a0`) [sure for the flow]

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
The rig frame is therefore not camera 0's frame. Correct RESEARCH §8.3 /
§9 wording (§9 below).

### 2.3 VD type 13 (`imu_set_calib.bin`, 0x34 bytes) [sure for the routing]

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
  quaternion reaches it through config pose A (§2.5). So type 13 **is**
  on the controller chain, as an IMU ↔ head/rig rotation, not as a
  camera extrinsic.

### 2.4 Head pose handed to the solver (`FUN_18018aaf0` decomp ~244285) [sure for the chain, called under Wine]

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
x. Whether X is exactly the camera rig frame of §2.1 (in metres; the rig
is in mm) is [likely] but not read in `FUN_18004b0a0`. Two facts argue
for it: the solver needs world ← rig, and B's 0.09 m matches the cameras'
rig z (≈ 80–106 mm). One fact needs explaining: X's +z maps to IMU −x,
the same direction as the HMD head's +z. A hardware check settles it
(§8).

When the controller has no prediction, the IR thread seeds its prior
position as **(0, −0.2, 0.3) m in X** (`0xbe4ccccd`, `0x3e99999a`).

### 2.5 VD type 13 layout (correction of §2.3) [sure for the first 16 bytes]

`FUN_1801617c0` forwards words `+0x0c, +0x00, +0x04, +0x08` to
`FUN_180157800`, which stores them as the quaternion (w, x, y, z) at
config `+0x9c0` and recomputes the config (`FUN_1801556b0`): so **type 13
starts with a quaternion x, y, z, w** (not the u8 from/to + 3×3 layout of
an extrinsic record). That quaternion rotates pose A, i.e. it is a
per-unit **IMU ↔ head/rig alignment**, and it reaches the controller
solve through A. Its translation-like floats at `+0x28..+0x30` are used
(negated) only by the head INS (`FUN_180173fe0`).

## 2b. LED model ↔ controller IMU, and the hand mirror (Q2)

### 2b.1 IMU offset (`FUN_18005f1b0`, IR thread decomp ~243800) [sure]

- The IR thread calls `UkfManager::setImuPose(ukf, hand, ImuPoseIndex,
  &v)` with `v` **always non-null**: `v` = pad `+0x48` (3 floats, mm),
  read by `FUN_180118b50`. It logs (XOR-obfuscated string) "Initialize
  VrController UKF: dev, model, …" with v.
- Pad `+0x48` is set on connect in `FUN_18011bae0` from the pad side
  (`+0x34`: 0 = right "R", else left "L"):
  - right: **(−20.0727, −12.2481, −6.0039) mm**;
  - left: **(+9.3727, −12.2481, −6.0039) mm**.
  These are the same numbers as the SteamVR "grip" offset (`steamvr.md`).
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

### 2b.2 Hand-1 descriptor byte = mirror axis 'X' (`FUN_18018aaf0`, `FUN_1800d3360`, `FUN_1800d3ae0`) [sure]

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

## 3. Wire and data formats

### 3.1 Per-controller result block (IR frame struct, stride 0x5a44) [sure]

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

### 3.2 Tracker output (`FUN_180189c70(tracker, hand, t, out)`, 0xc0 bytes) [sure]

| Off | Type | Meaning |
|---|---|---|
| +0x00 | u64 | time of the UKF state used |
| +0x08 | u64 | time of the last valid optical frame |
| +0x10 | u64 | UKF `+0x3b0` (per device; meaning open) |
| +0x18 | s32 | hand |
| +0x1c | f32[3] | position |
| +0x28 | f32[4] | orientation x, y, z, w (w = 1 on reset) |
| +0x38 | s32 | 1 = tracker has a state, 2 = state lost, 0 = no output |
| +0x3c, +0x40 | s32 | tracking status: 9, 6, 3 or 0 (§5) |
| +0x44..+0x4c | f32[3] | velocity (copied only on a fresh optical pose) |
| +0x50.. | | angular velocity, etc. |
| +0x68 | u8 | 6 = UKF valid, 7 = UKF valid and fresh optical, 0 = none |

`FUN_180160230` (device 1 = right, 2 = left; hand = device − 1) turns this
into the 0x50-byte pose `steamvr.md` documents: `valid (+0x4c) =
(+0x3c ≠ 0) && headset ready (FUN_1801253c0)`.

## 4. Fusion: optical measurement (Q3) [partial]

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
   through `FUN_180067b50` (the step / replay function, RESEARCH §9.4).
4. Remembers `rot_free` at `+0x4458 + hand`.

So Sony feeds the UKF a **full 6-DoF pose** (position + rotation) in the
solver's output frame, stamped with the LD frame time; it is not an LED
pixel measurement [sure].

### 4.1 Innovation gate (`FUN_180065b70` "kalmanFilterStep") [sure for the test, likely for reject]

Before the update (`FUN_180065760`, then `FUN_18006fe90`), the step
predicts the position at the measurement time and computes
`d = |p_meas − p_pred|` (m) and the speed `|v|`:

```
if |v| < 1.0 m/s (cfg +0x784):  skip the update when d > 0.05 m (cfg +0x778)
else:                            skip the update when d > 0.20 m (cfg +0x780)
```

### 4.2 UKF config defaults (`FUN_1800598a0`, block at UKF `+0x6a0`) [sure for values]

Names are [guess] unless the use is known (process noise uses per
`ctrltracker.md`).

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
| +0x778 / +0x780 / +0x784 | 0.05 m / 0.20 m / 1.0 m/s | optical gate (§4.1) [sure] |
| +0x77c | 0.0333 | |
| +0x788, +0x78c | 0.01, 1.0 | |
| +0x790 | 100 (int) | |
| +0x794..+0x7a8 | 0.8, 32.0, 0.34907 (20°), 2.0, 0.02, 177.78 | (+0x798..+0x7a0 are RESEARCH §11 q9 "limits") |
| +0x7b0 / +0x7b8 / +0x7c0 | −1 (u64) / 14000 (u64) / 7 | |
| +0x7c8, +0x7cc, +0x7d4 | 0.5236 (30°), 0.045, 0.2618 (15°) | |

## 5. Output, staleness and not-tracking (Q3, last part) [sure]

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

In SteamVR (`FUN_1801a7bf0`, `steamvr.md`): `eTrackingResult` is always
200 (Running_OK) while connected and not paused; `poseIsValid` is the
valid byte above. There is **no not-tracking timeout to
`TrackingResult_Running_OutOfRange`**: after 0.3 s without optics the
position freezes at the last optical value and the rotation keeps coming
from the IMU-driven UKF, valid, indefinitely [sure for this path]. Only a
lost filter state (or flags bit 3) clears `poseIsValid`.

Headset path (device 0) of `FUN_180160230`, for comparison: status 9 when
the INS is 6-DoF (`FUN_1801752b0`), else 3.

## 6. Constants

| Name | Value | Source |
|---|---|---|
| Optical stale limit | 0.3 s | `0x1803197d4`, `FUN_180189c70` |
| Quaternion sanity | 0.9 ≤ \|q\|² ≤ 1.1 | `FUN_180189c70` |
| Rig angles | 35°, 15°, 10°, 20°, 30° | `FUN_1800af650` |
| Rig translations | §2.1 | `0x180319b00/b40/b80` |
| Extrinsic scale | ±1000 (m → mm) | `0x180319a70`, `0x180319a04` |
| Grip offset | left (9.3727, −12.2481, −6.0039) mm, right (−20.0727, …) | `0x1802538ac/b0`, `FUN_180160230` |

## 7. Gaps in the Linux driver

- `psvr2_led_solver.c` assumes the rig frame is camera 0's frame. Sony's
  rig frame puts camera 0 at its built-in pose (§2.2). Any rig → head
  transform taken from Sony must be applied to Sony's rig frame, so build
  the rig the same way.
- No 0.3 s position hold, and no status distinction (9/3) in the
  controller pose.

## 8. Open questions

1. Is X (§2.4) exactly the §2.1 rig frame, and where does
   `FUN_18004b0a0` scale m → mm and flip axes? **Hardware check:** hold a
   controller still, solve it in the rig frame (mm), map with
   `INS · A⁻¹ · B` (rig/1000, then try identity vs 180° about y), and see
   which one keeps the world position fixed while the head turns.
2. Variant source: who sets tracker `param_1[0x3b]` (`FUN_180049240`).
3. Whether the UKF state (and so the tracker output) is the IMU or the
   LED-model origin, i.e. which side of the v translation (§2b.1) the
   SteamVR grip offset undoes (`FUN_18006c340` lever arm, measurement
   model `FUN_18005fc20`).
4. Model byte `+0x31e` values per rev (is any model pre-mirrored?).
5. Which config entries are the optical measurement noise: read the
   sigma-point update `FUN_18006fe90` → `FUN_18006c760` (gate is done,
   §4.1).
6. Two controllers: `FUN_1800a5e30` → `FUN_1800ad2b0`.
7. LD `+0x08` → frame time offset; `DAT_18035ba14`.
8. Meaning of status 6's fallback (`FUN_1801b99a0`, "OcclusionQueries")
   and `+0x441d8[hand]`.

## 9. Corrections to RESEARCH.md

Change, and add to §12:

| Where | Old | Correct |
|---|---|---|
| §3.3 | Type 13 = "IMU ↔ camera [likely]", 0x34 extrinsic layout | `imu_set_calib.bin`; starts with a quaternion (x, y, z, w) that rotates config pose A (IMU ↔ head/rig, opticalpose.md §2.4–2.5); +0x28..+0x30 used negated by the head INS |
| §5 / §8 | — | Add the solver's head chain `world_from_X = INS · A⁻¹ · B` with A, B from opticalpose.md §2.4 |
| §8.3 | "Each camera is placed … composing its extrinsic with camera 0's built-in rig pose" (correct) but §9 / task text "camera rig frame (camera 0's frame)" | The rig frame is **not** camera 0's frame: camera 0 sits at its built-in pose (§2.1 table), cameras 1–3 = that pose ∘ extrinsic |
| §8.3 | Built-in table "variant 0" only, translations only | Three variants with full rotations (opticalpose.md §2.1); the HW headset's 15° toe-out matches variant 1/2 |
| §9.4 / steamvr | — | Add: optical measurement = full pose; 0.3 s stale → position held, rotation from UKF, status 3; `poseIsValid` stays 1 |
| §9.1 | `ImuPoseIndex` selects a constant IMU → controller 4×4 | The call always passes the pad vector, so the stored transform is a pure translation by pad `+0x48` (R (−20.0727, −12.2481, −6.0039) / L (9.3727, …) mm); the tables apply only to a NULL vector (opticalpose.md §2b.1) |
| §11 q12 | Hand-1 byte `0x58` | Answered: the value `'X'`; left hand mirrors LED positions and normals in x and reverses index lists, unless the model's byte `+0x31e` says otherwise (§2b.2) |
| tracking.md §5.6 | "Example (index 1) … translation (14.5, 12.9, −8.4) mm" | Table translations are in **metres** (0.0145 m); only used without a pad vector |

## 10. Function map

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
