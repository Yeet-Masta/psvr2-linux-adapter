# Function catalogue: ctrl-boot (`0x1800a0000`–`0x18010ffff`)

One row per function that no other doc describes. Roles are in our own
words; tags as elsewhere (`[sure]` read from the code, `[likely]`,
`[guess]`). Sizes are decompiled lines. The pipeline itself is in
`notdone.md` §2 and `tracking.md` §5.3; this file only fills the gaps.

Object names used below:

- **tracker** = `RnD::HmdTrackerIntf` (`0x1800a12d0` ctor). Per-controller
  blocks of 0x1d00 bytes from `+0x20`; controller count at `+0x15c40`;
  processing order at `+0x15c44`.
- **camera manager** = `DAT_180616720[cam]`; its controllers are 0x4ce0-byte
  objects at `*(mgr + 0x10)`.
- **n_cams** = `DAT_1805e69b0`. Several offsets differ when it is < 2.
- **RnD::Vector** = `{allocator*, begin, end, cap}`; the allocator's vtable
  slot 1 allocates, slot 3 frees.

## Boilerplate groups [sure]

| Group | Functions | What they are |
|-------|-----------|---------------|
| XOR string decoders | `0x1800aa0b0`–`0x1800aa950` (19) | Inline decoders for obfuscated log strings: XOR a stack buffer with fixed 64-bit keys. No logic. |
| RnD::Vector grow | `0x1800aab40` (8-byte items), `0x1800aad20` (16-byte), `0x1800ab430` (reserve n) and more (`shapes.py` label `vec_grow`) | push-back growth through the allocator |
| RnD::Vector free | `0x1800aacf0`, `0x1800ab0a0`, `0x1800ab540` and more (`vec_free`) | free storage, zero begin/end/cap |

## Tracker object

| Function | Lines | Role |
|----------|-------|------|
| `0x1800a12d0` | 74 | `HmdTrackerIntf` ctor: zeroes 4 per-camera blob stores (0x1d00 each) in three banks (`+0x20`, `+0x7420`, `+0xe820`), builds the camera models (`0x1800a1000`, `0x1800a16e0`) [sure] |
| `0x1800a1510` | 35 | `HmdTrackerIntf` dtor (frees its buffers) [sure] |
| `0x1800a1990` | 9 | construct 4 controller objects of 0x4ce0 (`0x1800d2360` ctor, `0x1800d30d0` dtor) [sure] |
| `0x1800a1bb0` | 9 | forward to controller `a2`'s update (`0x1800d3360`) [sure] |
| `0x1800a1bf0` | 21 | reset controller `a2`: clear flag `+0x601`, reinitialise its pose state (`0x1800d0d30` into `+0x1af0`) and copy it to the previous-frame slot `+0x20e0` (`0x1800a0bf0`), clear `+0x1f94`, reset IMU prior (`0x1800a0ab0`), zero its 0x1d00 blob store [sure] |
| `0x1800a1ce0`, `0x1800a1d60` | 19, 33 | debug overlay draw for one controller (`0x1800d6e50`, `0x1800d7410`) [sure] |
| `0x1800a0ab0` | 77 | reset the IMU-orientation prior of a controller: `+0x3358` = 0, `+0x3360..+0x351f` = 0, `+0x35f0` = 1e5 (≈ "never") [sure] |
| `0x1800a0bf0` | 126 | copy-assign of the 0x5f0-byte per-controller **pose-state** block (nested copy `0x18009fa80`, then fields up to +0x570). Used for the previous-frame slot (`+0x1af0` → `+0x20e0`) and for the per-camera snapshots in `out +0xa430` [sure] |
| `0x1800a1270` | 27 | thunk into the camera model mapping `0x18009eb50` [sure] |
| `0x1800a09a0` | 37 | blob list (struct of 11 arrays): remove item `i` by moving the last item into it [sure] |
| `0x1800a32b0` | 281 | per-frame output gather (see below) |

### `0x1800a32b0`: output gather and "near" hysteresis [sure unless tagged]

For each controller `i`:

1. Best camera `cam = tracker[+0x1809c + 4i]` goes to `out +0x8964 + 4i`. If
   it is not −1, the 0x108-byte block `+0x34b0..+0x35b7` of that camera's
   controller `i` is copied to `out +0x8980 + 0x110·i`; the same block of
   every camera goes to `out +0x9320 + 0x440·i + 0x110·cam`.
2. If the controller has a pose this frame (flag byte at block
   `+0x7728`, or `+0x328` when n_cams < 2), the pose (built by
   `0x180045d50`) is pushed into a 4-entry ring (64-byte entries; count and
   head at the out block `+0x4c0/+0x4c4`). Without a pose the ring resets.
3. "Near" flag `out +0xa420 + i`, hysteresis on |t| of the tracked pose
   (mm): set when valid and |t| ≤ **280**, cleared when invalid or
   |t| > **330**. Read by `0x18004a340`, `0x18004a9d0`, `0x1800d5fc0`
   (overlay) and `0x1800dde10` [sure for readers; purpose: close-range
   handling, likely].

## Debug overlay (fluff) [sure]

Draws into an RGB debug image; none of it affects tracking. Marked fluff
in `triage.tsv`.

| Function | Lines | Role |
|----------|-------|------|
| `0x1800bdd00` | 76 | 8×8 text at (x, y), clipped to the image |
| `0x1800bdec0` | 99 | Bresenham line |
| `0x1800be0c0` | 110 | line / cross with colour |
| `0x1800be3e0` | 97 | rectangle |
| `0x1800be530` | 675 | ellipse outline from line segments |
| `0x1800bedc0` | 683 | covariance ellipse of a blob |
| `0x1800bf660`, `0x1800bf750` | 38, 216 | circle, filled marker |
| `0x1800d5c40` | 101 | coloured marker |
| `0x1800d5fc0` | 750 | tracking-state overlay (`notdone.md`) |
| `0x1800d6e50`, `0x1800d7410` | 320, 699 | one controller: LED projections, pose, blobs |
| `0x1800d7da0` | 162 | pose axes |
| `0x1800d8890`, `0x1800a1e80` | 63, 66 | integer and controller-id labels |
| `0x1800a1ce0`, `0x1800a1d60` | 19, 33 | per-controller overlay thunks |
| `0x18004c590`, `0x18004c600`, `0x18004c9c0`, `0x1800474f0` | 14–660 | overlay entry points in the tracking frontend |

## Multi-camera frame step (`0x180048690`, tracking frontend) [sure unless tagged]

Called by the per-frame core `0x18004b0a0`. Order of work:

1. Timestamps for the stage log (`0x1800480c0`, read by `0x180048190`).
2. Per camera: blobs through the camera model (`0x1800a5310`).
3. Per tracked device: next-frame pose prediction (`0x1800ceaa0`, below)
   into `out +0x9200 + i` (valid), `+0x9210 + 0x40·i` (pose),
   `+0x9250 + 4i` (jitter).
4. Per camera, on worker threads (`0x180054410` starts, `0x180053da0`
   waits; 4 × 0x60-byte parameter blocks): each camera's own
   boot/track. Then the blob lists go to the camera managers
   (`0x1800a0480`) and the controller order is re-sorted (`0x1800a5e30`).
5. A second worker round, per-device timing, `0x1800a2c70`.
6. Cross-camera agreement filter `0x1800a45f0` (skipped when debug byte
   `0x1805f6cad` is set), blob lists copied again.
7. Unless debug byte `0x1805f6cac` is set: each controller's config is
   snapshotted (`0x1800a0bf0`) and a third worker round runs.
8. Multi-camera fusion / cross boot `0x1800a37f0` (cameras 0..3), output
   gather `0x1800a32b0`, stage log `0x180048190`.

### Next-frame pose prediction (`0x1800ceaa0`) [sure]

Input: the 4-entry history of output poses that `0x1800a32b0` keeps per
device (64-byte entries, count `+0x100`, head `+0x104`). Needs ≥ 3
entries, else returns 0. With `t0, t1, t2` the newest three positions and
`R0, R1` the newest two rotations:

```
v0 = t0 − t1,  v1 = t1 − t2
t_pred = t0 + 0.9 · (0.7·v0 + 0.3·v1)
jitter = |v1 − v0|                       // second difference, mm
dq     = quat(R0 · R1⁻¹)
q_pred = slerp(q0, dq ⊗ q0, 0.3)         // lerp + normalise when cos ≥ 0.999
```

So the next frame starts from a damped constant-velocity guess: 90 % of a
smoothed per-frame step in position, 30 % of the last rotation step. The
predicted pose is what `0x1800a5e30` sorts by when a controller has no
tracked pose (flag `+0x9200`, `notdone.md` §2.14).

## Cross-camera agreement filter (`0x1800a45f0`) [sure]

For each controller without the "near" flag (`out +0xa420`): collect the
pose each camera found (`+0x1af0` block of the camera's controller object,
"has pose" byte `+0x1f9c`), mapped into camera 0's frame with the camera
extrinsics (`tracker +0x18150 + 0x1c0·cam`, `0x1800451e0`). For every pair
of cameras with a pose:

- `d²` = squared distance of the two positions (mm²);
- `a` = sum over the three axes of the angle between the two rotated unit
  axes, in degrees.

Let `d²_min` (floored at 144, i.e. 12 mm) and `a_min` (floored at 7°) be
the best pair values. A pair agrees when `d² < 25 · d²_min` (distance
within 5× the best) **and** `a < 5 · a_min`. Both cameras of an agreeing
pair are kept. If any pair agrees, every other camera's "has pose" byte is
cleared: its pose is treated as an outlier.

## Other large functions in `0x1800a0000`–`0x1800affff`

| Function | Lines | Role |
|----------|-------|------|
| `0x1800a6190` | 249 | publish one controller's solve into the tracker's output block (stride 0x1d00): status, flag bytes `+0x1f9c..+0x1f9e`, pose (rotation as matrix via `0x180044bc0`), the inlier LED → blob list (24-byte items at `+0x398`, count `+0x390`, LED map `+0x1b94`; LEDs without a blob get −1), a second pose at `+0x1c40` and 0xc0 bytes of covariance-like data [sure for the copy; meaning of the second pose: likely the predicted pose] |
| `0x1800a7c50` | 413 | text parser for a debug LED-model override (header `%d %d %d`, LED count ≤ 64, lines `M=%f %f %f`, `N=%f %f %f`, `RING=`); can add random noise (Mersenne Twister seed 20) but the factor is 0. Only reached from the `ConfigFile0.trk` debug path `0x18004a340` [sure] |
| `0x1800acfb0`, `0x1800add40`, `0x1800aeff0`, `0x1800ae860` | 136, 156, 36, 33 | `std::sort` on (float key, payload) pairs, descending: introsort loop, partition with median-of-9, median-of-3, heap sift [sure] |
| `0x1800ae2b0`, `0x1800af120`, `0x1800aeb30` | 156, 36, 33 | the ascending instance used by `0x1800ad2b0` (`notdone.md` §2.14) [sure] |
| `0x1800af280`–`0x1800af5b0` | 36–49 | elementary rotation matrices about x / y / z from an angle (scalar or vector lane), sine and cosine from `0x18009e2a0` [sure] |
| `0x18009e170`, `0x18009e2a0` | 83, 115 | SIMD `acos` and SIMD `sincos` [sure] |
| `0x180044bc0` | 120 | 3×3 rotation → quaternion [sure] |
| `0x1800ac160`, `0x1800ac9b0`, `0x1800ad610` | 51–68 | RnD::Vector of 16-byte items: resize with fill, reserve [sure] |

## Multi-camera fusion (`0x1800a37f0`) [sure unless tagged]

Runs once per frame after the per-camera solves, per controller:

1. **Best camera.** Among cameras whose controller object has a pose
   (`+0x1f9c` set, `+0x1f9e` clear), prefer one that passes the step check
   below and is in state 3 (`+0x20d0`); ties go to the larger counter
   `+0x3354`. With a single camera that one wins. Result in
   `tracker +0x1809c + 4·ctrl` (−1 when none).
2. **Step check** (`0x1800a2530`): the camera's pose against its previous
   frame's state (`+0x20e0`, see `0x1800a0bf0`): quaternion dot >
   cos 25° (rotation step < 50°) and position step < 25 mm, and, when the
   IMU-orientation prior is valid (`tracker +0x3360`), the same angle test
   against that prior.
3. **Pair learning.** For every other camera with a pose (enable bytes
   `tracker +0x18070 + ctrl`, `+0x18070 + 4·cam`): push the relative
   transform between the two cameras' poses of this controller into a
   per-pair estimator (`0x1800a2220` → `0x1800b0cc0`, below).
4. **Fusion.** With setting `0x1803517b7` = 1 (it is 1 in this build) and a
   converged pair estimate, the other camera's pose is mapped into the
   best camera's frame and blended with weight `n_b / (n_a + n_b)` by
   inlier counts (`0x1800a28f0`: slerp + lerp). In "near" mode
   (`out +0xa420`, setting `0x1803517b4` = 1) the close-range path
   `0x1800a88f0` runs instead, with the camera extrinsics from
   `0x1800cbd70`.
5. The result (or, with no camera, the previous pose `+0x33c0`) is stored
   by `0x1800a2140` in the tracker block (`+0x7770` position, rotation as
   matrix and quaternion).

### Relative-pose estimators [sure]

Two online estimates of camera-to-camera transforms exist, both learned
from controllers seen by two cameras at once:

- **Per controller and camera pair** (`tracker +0x15c60 + 0x1f0·(ctrl +
  4·cam)`, ctor `0x1800a1000`): `0x1800a2220` pushes the newest relative
  transform into a 6-deep history (count `+0x180`). `0x1800b0cc0` needs at
  least 5 samples (`+0x1dc`), finds the sample with the most neighbours
  within 10 mm (50 mm once converged; `+0x1d4`/`+0x1d8`), needs at least
  3 of them (`+0x1e0`), averages them (mean position, sign-aligned
  quaternion mean), and either sets the estimate (`+0x190` rotation,
  `+0x1c0` position) or blends it toward the new mean once converged
  (flag `+0x1d0`). Used by step 4 above.
- **Rig-wide table** (`rig +0x5b0`, 0x70 per pair, `rig` =
  `DAT_1805f6cb0`, built by `0x1800cb670`): seeded from the factory
  extrinsics by `0x1800b1340` at init. `0x1800a2c70` adds a sample for
  each camera pair where both cameras track the same controller well
  (pose, state not 2 or 3, more than 4 inliers `+0x1f8c`, depth ≥ 250 mm),
  and `0x1800b0980` folds it in: position EMA with α = 0.01 for the first
  20 samples, then 0.005; rotation slerped by the same α. Only the
  cross-camera agreement filter (`0x1800a45f0`) reads this table.

Settings in this build (data at `0x1803517b4..b7`): all 1. Camera image
size `0x180351764/68` = 640 × 480.

| Function | Lines | Role |
|----------|-------|------|
| `0x1800a2140` | 55 | store the fused pose of a controller in the tracker block [sure] |
| `0x1800a2220` | 125 | relative transform of two cameras' poses → per-pair history, then `0x1800b0cc0` [sure] |
| `0x1800a23f0` | 84 | per camera with a clean pose: set the pair-enable byte (`tracker +0x18070 + ctrl + 4·cam`) and store that camera's pose mapped into the rig frame (camera transforms at `out +0x10a50 + 0x40·cam`) in `tracker +0x17cb0 + 0x40·ctrl + 0x100·cam`, the input of `0x1800a2220` [sure] |
| `0x1800a2530` | 173 | step check (step 2 above) [sure] |
| `0x1800a27e0` | 58 | best-camera choice on its own (rule of step 1), result to `tracker +0x1809c` and the caller [sure] |
| `0x1800a28f0` | 243 | weighted blend of two poses (slerp rotation, lerp position) [sure] |
| `0x1800a2c70` | 129 | rig-table sampling (above) [sure] |
| `0x1800b0980` | 112 | rig-table EMA (above) [sure] |
| `0x1800b0cc0` | 517 | per-pair consensus estimator (above) [sure] |
| `0x1800a1000` | 40 | per-pair estimator ctor: 6 history slots, thresholds 10 / 50, min 5 samples, min support 3 [sure] |
