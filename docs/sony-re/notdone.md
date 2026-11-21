# Open gaps in the dev driver: the missing unknowns

Scope: the items in `docs/DEV-NOTES.md` "Not done, and why". Each section
gives only the unknown that blocked the item. Facts already in
`tracking.md`, `ledsync.md`, `ctrlboot.md`, `ctrltracker.md`,
`irtracker.md` and `docs/sony-driver-re.md` are not repeated.

Tags: **[sure]** read in code, **[likely]** strong inference, **[guess]**.
Addresses are function VAs in `driver_playstation_vr2_orig.dll`
(PE timestamp 1741940190).

---

## 1. Raw camera calibration to camera model

### 1.1 Summary

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

### 1.2 Live data path [sure]

| Step | Function | What happens |
|------|----------|--------------|
| 1 | `FUN_180127250` | VD entry type 10 → copied to headset context `+0x40220` (0x800 bytes), counter `+0x43dd4` ++. Type 11 → `+0x40a20` (not used by the IR tracker) |
| 2 | `FUN_18018aaf0` (IR thread start, and again whenever the counter changes, checked every 64th LD packet) | `FUN_1801247d0` copies the 0x800-byte type-10 blob, then calls `FUN_180182e80(blob, 0x800, cfg)` |
| 3 | `FUN_180182e80` | `FUN_180161b70` parses the SIE blob into doubles (layout in the main doc). Converts each camera to a 26-float record and the first 3 extrinsics to float R/t. Writes them into the tracker config `cfg`. |
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

### 1.3 Step 3: raw doubles → 26-float camera record (`FUN_180182e80`) [sure]

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

### 1.4 Step 4: model struct (`FUN_180045420`, `FUN_18009d4d0`) [sure unless tagged]

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

### 1.5 Forward projection, in full [sure]

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
extrinsics (§1.7) live in the rotated frame.

### 1.6 Worked example

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

### 1.7 Extrinsics (0x34-byte records) [sure unless tagged]

`FUN_1800b02a0`, and its helper `FUN_1800b0130`, convert each calibration
R (row-major, metres) to a 4×4 column-major float matrix:

```
R′ = G·R·G,  t′ = 1000·G·t,  with G = diag(−1, −1, +1)
```

The constants are `0x180319a70` = −1000 and `0x180319a04` = +1000; the
sign masks flip R[0][2], R[1][2], R[2][0], R[2][1].

So the **units are mm**. The frame change is the same 180° rotation about
Z as in §1.5. `tracking.md` says "z axis flipped": that is equivalent for
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
- LedSync uses cameras 0 and 1 for its "Many Blob" test (ledsync.md §4.3).

### 1.8 "Undistort" and `kP1PFromFailedUndistort` [sure unless tagged]

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
can use any iterative inverse of §1.5; Sony's was not found [open].

### 1.9 Still unknown (item 1)

- Extrinsic direction (from → to): needs one real blob.
- Which of extrinsics 0 and 1 links which cameras.
- Width/height in the blob (whether 508/W is exact).
- Physical camera names: needs the cover test.
- Next functions to read: `FUN_1800b02a0` lines after the first 4 calls
  (composition order), and `FUN_1800cbd70` (how the per-frame core uses
  `+0x10a40`).

---
## 2. Controller pose solve: the gaps

Item 2 is closed. The chain is: blob ROI crop (§2.14) → boot config
(§2.4) → single-camera boot (§2.9) or orientation-prior multi-camera boot
(§2.13) → association (§2.8) → gated match (§2.11) → weighted LM with a
robust kernel (§2.10) → MSAC score (§2.12). Only small leftovers remain
(§2.15).

### 2.1 Per-frame blob/LED tables shared by the solver and LedSync [sure]

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

### 2.2 LED visibility metric (+0x15bc, the "≈45") [sure for the use, guess for the unit]

`FUN_1801999f0` uses `m = metric·0.5`:

- an LED is a sync candidate when `m ≠ 0` (epsilon test against
  DBL_EPSILON) and `m ≤ 45.0` (`0x18026114c`, and the per-slot bests
  `0x42340000` = 45.0);
- smaller is better.

The writer of `+0x15bc` was not found (it is filled through a block copy,
not by a literal offset). The solver's own visibility test is in
`tracking.md`: `FUN_1800e1c90`, normal within `cos(0.349)` of the cone,
[sure].

Resolved by §2.8 [likely]: the metric is the LED-normal ↔ view-ray angle
in degrees (0 = face-on), so the LedSync gate `metric·0.5 ≤ 45` keeps LEDs
within 90° of facing the camera. The direct writer of `+0x15bc` (a block
copy from the per-pose `+0x51c` array) was not pinned to one instruction.

### 2.3 Hardware type → model (`FUN_18018aaf0`) [sure]

- rev = libpad `FUN_1801bc660` = feature `0x20` +0x18 low 16 bits.
  The table is in `sense.md` §3.2. Stored at manager pad `+0x3c`
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

### 2.4 Tuning block = boot config (`ConfigFile0.trk` keys) [sure]

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
  1/distance(m), see §2.9.
- The "0.9" in word 9 is not the RANSAC cap; the cap is the separate
  constant in §2.5. Previous [guess] names for words 0–20 are withdrawn.

**Model header** (the 0x139c block): word 0 = LED count (20 for rev 1,
17 for rev 2–7), word 4 = LED cone angle in degrees (**170** for rev 1–3,
**180** for rev 4–7), copied to boot obj `+0x660` via `0x1805f4054`
(`FUN_1800d3ae0`) [sure]. Word 5 = 48 [unknown], copied to `0x1805f4058`.

### 2.5 RANSAC count (`FUN_1800c8b50`, settings from `FUN_1800c8950`) [sure]

```c
max_iter = 1000;                      // settings +0x00 and +0x04 default 1000
if (settings.adaptive) {              // +0x0c
    w = MIN(inlier_ratio, 0.9f);      // +0x1c, cap 0x180319818
    n = (int)(log(1 - p) / log(1 - pow(w, 6.0)));   // p = +0x14 confidence
    iterations = MIN(iterations, n);
}
```

The sample size is **6** (`0x1803198d0`). Settings `+0x08` = 12 [unknown].
The block also holds a Mersenne Twister seeded with **0x29a** (666):
`FUN_1800c8950`.

### 2.6 Close-range / lost state machine

See §1.8 (`FUN_1800db690`). Distances are in mm:

- 80 / 120 / 130 / 170 for entering and leaving "close" modes;
- re-boot classes 100 / 150.

### 2.7 UKF

Filter structure, measurement model and noise: [ukf.md](ukf.md)
(2026-11-20). Before that, `tracking.md` §5.5 and `ctrltracker.md`.

- The limits at UKF `+0x798..+0x7a0`, `+0x784` and `+0x864` come from
  config and were not read.
- The controller IMU feed path is in `tracking.md` §5.3 step 8.

### 2.8 Association: project, cull, sort (`FUN_1800dd5d0` → `FUN_1800daff0`) [sure unless tagged]

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
`+0x15bc` (§2.2) is this angle [likely], so LedSync's `metric·0.5 ≤ 45`
means "LED faces the camera within 90°".

### 2.9 Boot (`FUN_1800d8b60`) [sure unless tagged]

Called from `FUN_1800d5070` per camera. `param_5` = "boot while tracking".

```c
if (n_blobs < MININLIERS_FOR_ORIENTATIONBOOT /*4*/) fail;
ransac.max = in_track ? BOOTINTRACK_MAXSAMPLES /*50*/ : RANSACMAXSAMPLES /*200*/;
R0 = (USEGRAVITYBOOT && imu_quat(+0x3380) != 0) ? R(imu_quat) : I;   // mode 7 / 6
ransac(blobs, model, R0);                      // FUN_1800c3aa0, FUN_1800d0950
if (inliers <= 3) fail;
associate(pose, mode 0);                       // §2.8
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

- The RANSAC itself (sampler, P3P, score, setup): §2.19.
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
  (`0x1803199a0`/`0x1803199c0`), see §2.6.

### 2.10 Refine (`FUN_1800d9de0`) [sure unless tagged]

Steps:

1. `FUN_1800e0130` prepares the residual set; the pose is inverted
   (camera ← model).
2. Weighted Levenberg–Marquardt solve, `FUN_1800f7970`
   (RTTI: `RnD::WeightedLMSolver<float, RnD::PoseCost, 6>`; the multi-camera
   variant uses `RnD::PoseCostMultiCam`). Details below.
3. Re-associate (`FUN_1800ddb30` → §2.8, then `FUN_1800ddb90`) with the
   SCALED pothole gate; re-score (`FUN_1800e0690`, §2.12).
4. Fail code 3 if fewer than 3 matched LEDs; fail code 4 if `FUN_1800d89f0`
   rejects the pose.

`FUN_1800d9d50` wraps this: first the gated match (`FUN_1800df5b0`, §2.11)
with radius `pothole·1.5`; fewer than 3 matches → fail code 2.

**Residual** (`FUN_180105ed0`, `RnD::PoseCost`), 8 LEDs per AVX batch:

- transform each matched model LED by the pose, perspective-divide, apply
  the full camera distortion (`FUN_1800c08a0`, the model of §1.5);
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

### 2.11 Gated match (`FUN_1800df5b0`) [sure unless tagged]

Arguments: gate radius R (pixels) and an optional second radius
(default FLT_MAX); both are squared before use.

1. Reset per LED: best distance² 9999.9 at pose `+0x2b8 + 16·L`, blob
   `+0x43c + 4·L` = −1. `FUN_1800c0420` builds a spatial index of the blobs.
2. Walk the visible LEDs in view-angle order (§2.8). For each, take blobs
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
  S = R = 1.5·POTHOLE·scale (§2.16).
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

### 2.12 Pose score (`FUN_1800e0690`) [sure]

Not a count: an **MSAC** score. For each matched LED, d = pixel distance
projection↔blob (stored at pose `+0x2bc + 16·L`, −1 when unmatched).
With T = pothole threshold (§2.9, scaled by 1/distance):
score += d² − T when d² ≤ T, else 0. Pose `+0x4a0` = score / T, so it runs
from −n (perfect) to 0. So POTHOLETHRESHOLD is compared with a **squared**
pixel distance [likely px²].

### 2.13 Multi-camera boot (`FUN_1800e2a00`) [sure unless tagged]

Runs when the single-camera boot found nothing (flag `0x1803517c1` = 1).
It is an **orientation-prior boot**: rotation comes from the IMU, only
position is searched.

1. Needs a valid IMU orientation prior (`+0x3360`); otherwise returns −1.
2. Collect this camera's blobs (`FUN_1800e2330`, radius 5.0 `0x180319938`,
   params 4.0 / 3.0). With more than 10 blobs, the first 10 slots are a
   random partial shuffle (RNG `+0x3600`, the Mersenne Twister of §2.5).
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
   (`RnD::PoseCostMultiCam`, §2.10; flag `0x1803517d9` = 1), re-score,
   and accept when m > 5 and e < **0.1** (`0x180319794`).
7. On accept: build the pose (`FUN_1800d1aa0`), check it with
   `FUN_1800d95a0` (ratio 0.7, `0x18031980c`), associate in mode 1 (174°
   cone), then `FUN_1800db5f0`, `FUN_1800ce970`.

The unit of e is [unknown]; 0.1/0.2 suggest a normalised score, not pixels.

### 2.14 Blobs per camera and the two controllers [sure unless tagged]

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
its own frames, `ledsync.md`) and by the pose-driven association, not by
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

### 2.16 Track step and pose check (`FUN_1800d5070`, `FUN_1800db690`, `FUN_1800db980`, `FUN_1800d89f0`, `FUN_1800e1d50`) [sure unless tagged]

- `FUN_1800d5070` per camera and frame: when the controller is tracked,
  `FUN_1800db690` runs the state's track solver (state 0:
  `FUN_1800db980`). The boot (§2.9) can still run; a boot result replaces
  the track result when the track's cross-camera support
  (`FUN_1800e27a0`) is below **0.5** (`0x1803517ac`) and the boot has 2+
  more inliers, a score lower by 2, or lies nearer the last pose [likely
  for the exact combination]. Boot candidates with score > 0.34 are
  dropped. The status string is "Boot", "T" or "-".
- State 0 track (`FUN_1800db980`): predict (`FUN_1800da320`); blob set by
  depth (thresholds 100/200/300 mm, compared squared); associate mode 0;
  gated match + refine (`FUN_1800d9d50`) with S = SEARCHRADIUS·scale and
  R = 3·S·dt / (1/60 s); gated match with S = R = 1.5·POTHOLE·scale,
  < 3 matches → fail (code 8); `FUN_1800dcf00` (the few-LED solve, §2.17,
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

### 2.17 Few-LED solve (`FUN_1800dcf00`, `FUN_1800e0d20`, `FUN_1800d1240`) [sure unless tagged]

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
- Sony's gated match (§2.11) needs 2 supporting neighbours per LED, so 1
  or 2 isolated blobs never reach this solve in state 0; it acts when a
  cleanup or pass 3 leaves ≤ 3 matches, on a patch of 3 neighbouring LEDs,
  or in states 1–3 via the nearest-blob fallback (§2.18).
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

### 2.18 Close-range and edge states (`FUN_1800db690`, `FUN_1800dbda0`, `FUN_1800dc210`, `FUN_1800dc680`, `FUN_1800db420`, `FUN_1800ddb90`, `FUN_1800f1950`, `FUN_1800e4ec0`) [sure unless tagged]

Corrects §1.8: "close" there is the IMU orientation flag `a1[822]` (as
in §2.17), d = |translation| in the camera frame (controller origin),
the match count in state 3 is `+0x49c` and must be > **5** to leave.

- State 0 enters state 3 before solving when `FUN_1800db420` ≤ **3**: LEDs
  visible at association mode 3 (cone × 0.8) whose prediction lies
  ≥ **20** px inside the image rectangle and within **1.2391838·fx** of
  the principal point (camera model `+0x6b0` fx, `+0x6b8` cx, cy).
- States 1–3 (one solver each, same shape): association mode 2 (state 1,
  174°), 0 (state 2, model cone) or 1 (state 3, 174°); one gated match
  (§2.11) with gate = support radius = min(SEARCHRADIUS·scale,
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
- Then `FUN_1800dcf00` (few-LED, §2.17, else `FUN_1800dd2f0`: LM refine
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

### 2.19 RANSAC boot (`FUN_1800c2960`, `FUN_1800c4940`, `FUN_1800c2080`, `FUN_1800c52f0`, `FUN_1800c58f0`, `FUN_1800c5ac0`, `FUN_1800c62c0`, `FUN_1800c6df0`, `FUN_1800c3320`, `FUN_1800c18a0`) [sure unless tagged]

The RANSAC object is boot obj `+0x870`. `FUN_1800d3360` sets it up with
`FUN_1800c18a0` (decompiling it crashes IDA 9.5; read from the
disassembly):

| Off | Value | Source |
|---|---|---|
| `+0x00`, `+0x04` | iteration cap | RANSACMAXSAMPLES (`+0x63c`); `FUN_1800d8b60` then sets 50/200 (§2.9) |
| `+0x08` | 12 | [unknown] |
| `+0x0c` | adaptive flag = **0** | so §2.5's bound never runs here |
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
slot 1 = `FUN_1800d89f0(pose, close = 1) == 0`, the pose check of §2.16.

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
- LED rings: model block (§2.4) words **198** = ring count, **199 + k** =
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

Afterwards `FUN_1800d8b60` needs ≥ **4** inliers (`+0xa94`), then §2.9.

**Our port**: `psvr2_led_boot_ransac(cfg, cams, blobs, model, prior,
in_track, …)` — mode 7 when `prior->orient_valid`, else mode 6; every
hypothesis goes through `psvr2_led_pose_check(…, close = 1)`. Per camera:
RANSAC, our refine (`psvr2_led_track`, all cameras), the §2.9 acceptance
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
opticalpose.md §2b.2), rev 3: 186 and 170 right, 0 wrong; mirrored
without reversing the rings: 159 and 124. About 6 ms per call (4
cameras, WSL). Our oriented
boot (`psvr2_led_boot_oriented`) stays better when the heading is known:
198–200 right.

### 2.15 Leftovers (item 2)

These no longer block a port:

| Gap | Where |
|---|---|
| Candidate score inside the gated match | answered, §2.11 |
| Track-vs-boot arbitration, cross-camera support | answered, §2.16 |
| Exact LM stop thresholds (cost tol, gain tol) | `FUN_1800f9a80` fields `+0x1c`, `+0x24`; set from the option block in `FUN_1800f7970` |
| The 0.25 in the option block | `FUN_1800f7970` (candidate: minimum robust scale `+0x18`) |
| Unit of the multi-camera error e | `FUN_1800e49b0` |
| Cross-controller cleanup | `FUN_1800a5e30` → `FUN_1800ad2b0` |
| Model word 5 (= 48) | consumer of `0x1805f4058` |

---

## 3. LED-sync details

Phase enum (names at `0x180268d20`, 12-byte stride) [sure]: 0 INIT,
1 PRESCAN, 2 BROAD, 3 BG, 4 STABLE, 5 LED ALL OFF, 6 LED ALL ON.

### 3.1 Per-camera history window (`FUN_180194f70`) [sure]

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
tracker's** per-camera LED → blob map (IR frame struct `+0x1438`, §2.1):
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

### 3.2 BG check pattern `0x180260430` [sure]

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

### 3.3 BG estimator (`FUN_1801954f0`) [sure]

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
- `ledsync.md` §5.3 says "samples above the floor": wrong. It is below.

### 3.4 BG accept and correction (`FUN_1801975e0`) [sure unless tagged]

1. Over the 4 cameras plus slot 4, keep only valid x (≠ −1). Pick the one
   closest to the last accepted x (`0x180362b00 + ctrl·0x1e8`). That
   camera index goes to `0x180362ac4`.
2. **Skip rule.** If the fit object already has ≥ 2 runs, a fit exists,
   and **|x − 10| > 1**, the sample is dropped. This is allowed up to 3
   times in a row (`0x180363110`). §5.3 says "jumps by > 1 from the last
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

### 3.5 Drift fit: values, units, scaling [sure unless tagged]

| Fit | Object | Value pushed | Value unit | Time unit |
|---|---|---|---|---|
| BROAD | `0x180362ee8` | `*puVar12` in `FUN_180198240`: the accumulated slot offset (`0x180362f10` += slot − 4) [likely] | BROAD slots | µs since first sample (host clock) [likely] |
| BG / STABLE | `0x180362ef8` | `x + acc` (`0x180362f38`) | BG steps | same |

- **No scaling** before the command: `FUN_18019bc00` reads the fit's slope
  (fit +0x10) and sends it raw as f32 in cmd 3, or cmd 5 with an s32
  offset.
- The slope's unit is therefore steps/µs (or slots/µs).
- libpad applies `cycle := (1 + slope)·cycle` (`ledsync.md` §5.5). So
  Sony **under-corrects by a factor of `step`** (105 or 50) per round,
  compared with a dimensionless rate. The loop converges only because it
  repeats [likely]. Copy this behaviour as-is to match Sony.
- BROAD offset for the record (`FUN_180198240`):
  `(int)((slot−4)·step_ns·5/1000)/2` = (slot−4)·5·step/2 µs, matching §5.2.

**Sign** [sure for the arithmetic, likely for the physics]:

- A positive slope makes `cycle` longer.
- A positive slope means the measured phase (x + acc) grows over time:
  the dark gap moves to later frames.

### 3.6 BROAD "3 consecutive" [likely]

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

### 3.7 Sync mode: correction to `ledsync.md` §2.2 [sure]

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

- `local_b4` is feature `0x20` **+0x1c** (main firmware version; `sense.md`
  §2.7).
- **Every path ends in mode 2.** Old firmware gets mode 2 at once. A
  failing probe leaves mode 0 for up to 4 attempts, then mode 2.
- `FUN_1801c1820` has no other caller, so **mode 1 is never set** in this
  build. The 1000/105 timing is used only while mode = 0 (before or
  between probes).
- The `ledsync.md` table row "Mode 1: firmware < 0x1006d or probe failed 4
  times" is wrong; it is mode 2.
- The dev driver's "libpad sync mode 1/2 stands in for preset" should
  expect mode 2, timing 500/50, in steady state.

### 3.8 Still unknown (item 3)

- The exact time unit of the fit's t: host µs is [likely]. Read the caller
  of `FUN_180196ff0` for `param_2`.
- Whether `FUN_180198240` shifts slot = −1 into the 3-history (lines before
  the shift).
- What `*puVar12` points to in `FUN_180198240` for the BROAD fit.

---
## 4. Sony's driver-from-head transform

### 4.1 What Sony does [sure unless tagged]

The values are already in `display.md` §5.4: `FUN_18019ee30`, built by
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

### 4.2 Check against `steamvr/driver_psvr2/src/pose_source.cpp` [sure: numeric check]

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
  open question in `tracking.md` §8.1 is settled: the record is x, y, z, w,
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

### 4.3 Still unknown (item 4)

Nothing blocks the fix. Sony's origin handling (subtract the first
position; rotation untouched) is optional.

---
## 5. Relocaliser (low priority)

This is already covered: `room.md` §1, §2.2–2.3 and §3, plus `reloc.md`.
No new unknowns block it. In short:

- **Set `0x0a` payload:** header `0a 00 <u16 subcmd> <u16 0xd8> 00 00`,
  then always 0xd8 bytes (`FUN_180127fe0`, `FUN_1801283f0`).

  | Subcmd | Content | When |
  |---|---|---|
  | 4 | u32 1 | restart |
  | 3 | fixed tuning words | once after the first frame |
  | 1 | localiser result | every processed RP frame |

  The subcmd 1 result layout is `room.md` §2.3. Sends are paced at
  ≥ 20 ms with 5 retries (`FUN_1801adba0`).
- **RP packets (EP `0x8a`, 0xc8780 bytes):**
  - header: `"RP"`, u32 size at +4; the first 0x2c bytes are unused by the
    PC;
  - body: 162 features per camera pair (u16 keypoints, 128-byte
    descriptors, per-feature float blocks), a 640×640 8-bit image, a pose
    prior, and up to 2 anchors (`reloc.md` table).
- **Without it:**
  - only the **persistent map** is lost: re-anchoring a saved play area
    across sessions, and map recovery against a stored map;
  - SLAM tracking, the INS and loss recovery inside a session run on the
    headset;
  - the RP endpoint must still be drained (kernel already does).
  - [sure for the data flow; likely that in-session recovery does not
    need the PC]

---

## 6. Extreme-Env blob modes (low priority)

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
- The same block holds the LED-sync phase hold (`+0x05`, `ledsync.md`
  §3.2), a debug knob that pins controllers to one phase. Extreme-Env
  mode 3 is the only code in the DLL that writes it.

What an opt-in port would need (none of it is in the driver yet):

1. A setting standing in for `+0x34` (force) and `+0x33` (auto entry from
   Many Blob, and which of modes 2/3/4).
2. The two extra `0x09` blocks from `ledsync.md` §2.6 (`Block09` in
   `optical_tracker.cpp` has only Normal and Many Blob).
3. The entry and exit rules of `ledsync.md` §4.3: 5 flagged packets in a
   row to enter from Many Blob, 180 flag-free packets to leave.
4. In mode 3, hold every controller in PRESCAN (`LedSyncer::HoldPrescan`)
   and release it on the way out.

What blocks 0x08/0x0e = `0xff` (mode 4) and block 0x15 = 1/2 do inside the
headset is not known [guess: harder blob rejection for bright, cluttered
rooms]. Without hardware tests, Normal ↔ Many Blob is the safer default.

---

## Corrections to the existing docs

1. **`tracking.md` §5.3 "Frames"**: the extrinsic conversion is
   `t′ = 1000·(−tx, −ty, +tz)` with R conjugated by diag(−1, −1, 1). That
   is a 180° rotation about the optical axis, not a z flip. The R pattern
   is the same, but the translation shows the difference (`FUN_1800b0130`,
   `FUN_1800b02a0`).
2. **`tracking.md` §8.1** (SLAM quaternion order): settled.
   - The record is x, y, z, w.
   - The Linux driver's remap equals Sony's `q_dfh` plus a world yaw
     (§4.2). Both observations were right.
   - The Linux driver lacks `vecDriverFromHeadTranslation`; use
     (0, 0.0208, 0.09255) m in head axes.
3. **`DEV-NOTES.md` "Raw camera calibration"**:
   - the builder is `FUN_180045420` + `FUN_18009d4d0`, called from
     `FUN_180049240`, on data converted by `FUN_180182e80`;
   - the struct is 0x1c0 bytes;
   - the model resolution is 508 × 508;
   - `FUN_1800c8950` / `FUN_1800c8a40` are RANSAC settings, not the camera
     model.
4. **Main doc, camera record**: the 0xe4 layout has **20** doubles at
   +0x2c (not 19). Their meaning is OpenCV
   (k1, k2, p1, p2, k3, k4, k5, k6, s1..s4, τx, τy) + 6 extra rational
   terms (§1.3). VD type 13 (0x34 bytes) is one extrinsic record. The IR
   tracker uses only type 10, which holds all 4 cameras and 3 extrinsics.
5. **`ledsync.md` §2.2**:
   - firmware < 0x1006d or probe tries > 3 → **mode 2** (500/50), not
     mode 1;
   - mode 1 is never set in this build;
   - steady state is always mode 2;
   - `local_b4` = feature `0x20` +0x1c (§3.7).
6. **`ledsync.md` §5.3**:
   - the estimator weights frames **below** half the window max:
     `floor = (max + 0)/2`, not "above the floor";
   - the skip rule compares x with the centre 10, not with the last
     accepted value;
   - the BG window is 20 frames, and the estimator uses frames 4..16
     (§3.3–3.4).
7. **`ledsync.md` §5.5 units**: the slope is sent unscaled in steps/µs
   (BG/STABLE) or slots/µs (BROAD). The cmd 4 offset is `(10 − x)·step` µs
   (§3.5).
8. **`ledsync.md` "BG check pattern"**: it is 4 s32 pairs compared with
   frames 0–3 and 17–19 (lit, dark, dark, lit / lit, dark, lit), not 32
   frame bytes (§3.2).
9. **`tracking.md` §5.3 method names**: the enum at tracker `+0x4c94` is
   0 none, 1 kNLR, 2 P2P, 3 kP1P, 4 lost ("kP1PFromFailedUndistort"),
   5 kCrossBlobPnPR, 7 kMultiCamNLR, 8 kMultiCamP2PR. There is no undistort
   iteration behind 4 (§1.8).
10. **RANSAC iteration cap during boot**: the 1000 default in §2.5 is
    overridden per boot by `FUN_1800d8b60` (settings `+0x00` = boot obj
    `+0x870`): **200** for a cold boot, **50** for a boot while tracking
    (RANSACMAXSAMPLES / RANSAC-BOOTINTRACK-MAXSAMPLES, §2.4).
11. **Tuning block names**: the earlier [guess] roles for the 0x54 block
    are replaced by the `ConfigFile0.trk` key names in §2.4. A
    `ConfigFile0.trk` placed in the driver's working directory overrides
    them when no descriptor is given [likely].
12. **LED cone**: rev 1–3 controllers use a 170° LED cone, rev 4–7 use
    180° (model word 4, §2.4). Any port should take it per model, not as
    one constant.
13. **LED visibility angle (§2.8)**: not a half-angle test. The view angle
    (0° = face-on) is compared directly with 170°/180°, so almost every LED
    not facing straight away counts as visible. Association mode 1 uses 174°.
14. **Gravity/orientation boot limits**: fixed constants 3°/15°, and 6°/30°
    under 150 mm (§2.9), not config values.
15. **Robust kernel**: Huber with c = 1.0 on a 1.483·median scale, except
    with exactly 3 LEDs, where no kernel is used (§2.10).

## 2.20 Headset INS (done, not on hardware)

Ported Sony's host INS for the headset (tracking.md §3.1, §5.1, §5.2):
bit-exact kernels `psvr2_ins_core.c` (quat_mul `0x1801805a0`, rotinc
`0x18017f2c0`, strapdown `0x18017f8e0`, predict `0x18017f500`, pose_ok
`0x18017bac0`, blend mode 1 `0x180181a50`, IMU calibration matrix
`0x1801733f0`; difftest `t_irtracker_inscore.c`, 0 mismatches), the
`FUN_18017cdb0` sequence in `psvr2_ins.c`, a buffered IIO reader in
libpsvr2 (`psvr2_imu_stream_*`), and the driver path in `pose_source.cpp`
(setting `headset_ins`). Test `test_ins.c` (part of `make check`).

Left: mode-0 adaptive blender (gains unknown), stillness and shock
switches, Sony's 3DoF tilt correction `FUN_180171400`, the `+0x188`
good-frame gate; the VD type 12 layout is read from code only (the
driver rejects it if its diagonal is not within 20 % of the IIO scale).
