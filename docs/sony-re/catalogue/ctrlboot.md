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

## Raw-image blob extraction (unreachable on PC) [sure for reachability]

A complete PC-side blob detector for raw camera images exists, but nothing
calls it in this DLL: its only entry `0x18004ad80` has no caller and no
data reference, and the DLL exports only `HmdDriverFactory`. The headset
does the detection and sends `LD` blob packets instead (`ledsync.md`).
Marked `ref` in `triage.tsv`.

- `0x18004ad80`: copies a 14-byte detection parameter block, then queues
  one `BlobThreadParams` job per camera (`0x180054410`) and waits
  (`0x180053da0`).
- Worker thread `0x1800540e0` (vtable `WorkerThread`): job type 0 →
  `0x1800ccad0` (image 640 × 480 from `0x18035176c/70` into a 0x2404-byte
  blob list, the same size as one camera's block in an `LD` packet);
  type 1 → controller boot/track `0x1800a56b0` (modes 1 and 2); type 2 →
  cross boot `0x1800a5ac0`; type 3 → background view update (slot 2).
  Types 1–3 are used (`0x180048690`); type 0 never is.
- `0x1800ccad0` (707 lines): threshold, collect up to 256 candidate peaks
  above a level (parameter byte 1), grow each into a region
  (`0x1800cc1b0`, `0x1800cc570`), label components (`0x1800edac0`, 777
  lines), keep regions with 2..`param[12]` components, then centroid and
  shape (`0x1800eb0e0`) [likely for the stage roles].
- `0x1800ea4c0` (411 lines): a second connected-components entry with its
  own error text ("ConnectedComponents returned ..."), helpers
  `0x1800eae50`, `0x1800eb360`, `0x1800ec020`, `0x1800ed8d0`,
  `0x1800f2700`, `0x1800f23a0` [likely].
- Other helpers: `0x1800e9a90`, `0x1800e97e0`, `0x1800ce140`,
  `0x1800ed6f0`, `0x1800ed830`, thunks `0x1800a4f40`, `0x1800cca50`,
  `0x1800ebfc0`.

## Cross boot (worker job type 2) [sure unless tagged]

Per camera manager, after that camera's own boot/track (`0x1800a5ac0`):
controllers in distance order (`0x1800a5e30`), and for each:

1. `0x1800dde10`: try to start this camera's controller from another
   camera's pose (the snapshots in `out +0xa430`, see `0x180048690`
   step 7). Skipped when the controller already has a pose that passes the
   pose check `0x1800e1d50` with score `+0x1f94` > 0.5
   (`0x1803517ac`), and in "near" mode when more than 2 cameras already
   have it. Candidate: the other camera with the best snapshot score
   (`+0x4a4`), taken when it beats this camera's score by 0.1 or this
   camera has ≤ 3 inliers; its pose is mapped into this camera.
   - Depth > 150 mm: refine from the transferred pose (`0x1800dea50`:
     association `0x1800dd5d0`, gated match `0x1800df5b0`, refine
     `0x1800d9de0`, accept test `0x1800e2330`). On success the result
     becomes the pose state (`0x1800a0bf0`), its distance class is set
     (`0x1800db5f0`) and `0x1800e4df0` updates the history.
   - Depth ≤ 150 mm: close-range solve with few LEDs (`0x1800dee40`:
     `0x1800d89f0`, `0x1800ddb90`, few-LED solve `0x1800e0d20`,
     `0x1800e27a0`, `0x1800e2330`).
2. If the controller now has a pose: `0x1800a8770` post-processing.
   Otherwise, with setting `0x1803517b6` = 1 (this build) and a predicted
   pose (`out +0x9200`), the predicted pose is mapped into this camera
   (extrinsic `frame +0x10a40 + 0x40·cam`, camera index at manager
   `+0x182c8`) and handed to `0x1800d9800` as a tracking seed.

| Function | Lines | Role |
|----------|-------|------|
| `0x1800a5ac0` | 120 | cross-boot entry (above) |
| `0x1800dde10` | 619 | cross boot of one controller (above) |
| `0x1800dea50` | 227 | refine from a transferred pose |
| `0x1800dee40` | 396 | close-range transferred-pose solve |
| `0x1800e53f0` | 465 | at the start of each boot (`0x1800d8b60`): projects the model centre (`+0x19a0/+0x19b0` box) under the previous pose and the IMU prior (`+0x33c0`, `+0x3380`) into the image (`0x180046240`), result at `+0x1870` [guess: an image-space hint for the boot] |
| `0x1800efd70` | 434 | per LED: project into the camera (z ≥ 0, camera model `0x180045650`), find the nearest blob, mean distance; start value 9999 [likely] |
| `0x1800d1c80` | 254 | RnD::Vector of 8-byte items: resize with fill [sure] |
| `0x1800e3e80` | 477 | copy-construct a blob store (11 RnD::Vectors, as `0x1800a0480`) [sure] |

## Solver classes: vtable slots [sure for the map, likely for slot roles]

The refine and multi-camera solves (`notdone.md` §2.10, entry
`0x1800f7970`) use these classes. Robustifier slot meanings are in
`ctrlboot.md`; here are their addresses.

| Class (vtable) | Slots |
|----------------|-------|
| `RobustifierHuber` (`0x180319680`) | 0 `0x180109fb0`, 1 `0x18010a090`, 2 `0x18010a140`, 3 `0x180109db0`, 4 `0x180109dc0`, 5 `0x180109de0`, 6 `0x18010a010`, 7 `0x18010a040` |
| `RobustifierClip` (`0x180319648`) | 0 `0x18010a210`, 1 `0x18010a270`, 2 `0x18010a2c0`, 3 `0x180109e40`, 4 `0x180109e50`, 5 `0x180109e70` |
| `RobustifierTukey` (`0x180319610`) | 0 `0x18010a320`, 1 `0x18010a380`, 2 `0x18010a3d0`, 3 `0x180109ec0`, 4 `0x180109ed0`, 5 `0x180109ef0` |
| `Robustifier` base (`0x1803195d8`) | 0–4 pure, 5 `0x180109d00` (destructor) |
| `WeightedLMSolver<PoseCost,6>` (`0x180319518`) | 0 `0x1800f8f80` copy-assign (parameter scales, packed normal-equation buffers), 1 `0x1800f94f0` cost: residuals through the cost object, weighted sum, 2 `0x1800f9600` clear both flags `+0x210/+0x211`, 3 `0x1800f9610` clear `+0x211`, 4 `0x1800f9810` copy the parameter vector out (`0x180101a40`), 5 `0x1800fbcd0` destructor (`0x1800f7c70` frees the buffers), 6 `0x1800f9620` one iteration: robust weights (`0x180105ed0`), Jacobian and normal equations (`0x1800f69a0`, `0x1800f6b30`, `0x1800f6c90`, `0x1800fd060`–`0x1800fd3a0`, `0x180106610`) |
| `WeightedLMSolver<PoseCostMultiCam,6>` (`0x180319470`) | same pattern: 0 `0x1800fa7d0`, 1 `0x1800fad40`, 2 `0x1800fae50`, 3 `0x1800fae60`, 4 `0x1800fb040`, 5 `0x1800fbd30`, 6 `0x1800fae70` |
| `LMSolver<PoseCost,6>` (`0x180319558`) | 0–3 pure, 4 `0x1800f8420` (returns 0), 5 `0x1800f84a0` destructor |
| `LMSolver<PoseCostMultiCam,6>` (`0x1803194d0`) | 0–3 pure, 4 `0x1800f83f0`, 5 `0x1800f8470` |
| `PoseCost` (`0x180319590`), `PoseCostMultiCam` (`0x1803194c0`) | destructors `0x1800f73b0` (`0x1800f71d0`), `0x1800f7660` (`0x1800f7480`) |
| `LeastSquaresCost<…>` (`0x1803195a0`, `0x1803194b0`) | destructors `0x1800f8500`, `0x1800f84d0` |
| `FullBootConstraints` (`0x1803191e0`) | 0 `0x1800e5a90`, 1 `0x1800e5ae0` |
| `SatisfyConstraintsAbstract` (`0x1803193e8`) | 0 `0x1800d0820`, 1 pure |
| `Spots` (`0x180319410`) | 0 `0x1800ea090` (destructor) |

Other solver helpers:

| Function | Lines | Role |
|----------|-------|------|
| `0x1800f5df0` | 391 | real roots of a quartic for the P3P step (`0x1800f43e0`), via the cubic `0x1800f41e0`; tolerances 1e-5, 1e5 [likely] |
| `0x1800f7e20` | 199 | multi-camera LM driver called from `0x1800cfd90`: builds the cost (`0x1800f8260`), runs the solver (`0x1800fb090`, `0x1800fb2b0`, `0x1800fa4a0`, `0x180107280`), constant 0.4 [likely] |
| `0x1800fb2b0` | 465 | solver iteration loop with a `pow`-based step rule [likely] |
| `0x1800f8c50`, `0x1800fa4a0` | 130 | resize of solver work buffers (`0x1800fe9e0`) [likely] |
| `0x1800fbe40`, `0x1800fdd90` | 146, 167 | recursive helpers (sort / partition) [guess] |
| `0x1800f2cc0` | 149 | 32 points on a circle (step 2π/32) for a caller with no caller (`0x1800e9f30`) [likely: debug] |
| `0x180105a60` | 181 | builds the `LeastSquaresCost<PoseCost,float,6>` object (vtable `0x1803195a0`) for the LM driver `0x1800f7970`: copies the point and model references, residual count = 2 × points (u and v per LED) [sure] |
| `0x180107700` | 227 | multi-camera residuals: turns the 6-vector (rotation vector at `+0x10`, translation at `+0x1c`) into a pose with the exp-map `0x180104e30` (eps 0.001), composes it with the rig transform at `cost +0x50`, then for each LED composes with that LED's camera transform and projects with the pinhole model `0x180046240`; residuals into `cost +0xc0`, then the robust scale `0x180108920` [sure for the flow] |
| `0x180107bf0` | 226 | same as `0x180107700` but projects through the full camera model with distortion (`0x180103580`) [sure for the flow] |
| `0x180108920` | 71 | residual scale after a residual pass: median of the absolute residuals through `0x1801046e0` (the 1.483 × median rule of `notdone.md` §2.10) [likely] |
| `0x180106d70`, `0x180108100` | 145 | residual statistics of a cost object: RMS, max, min of the residuals and mean weight (residuals `+0xb0`, weights `+0x100`; the second reads `+0xc0`/`+0x110`). No caller in this build [sure] |
| `0x180109670` | 179 | partial sort (introselect) of a float array, the core of the median used by `0x1801046e0` (via `0x1801090f0` / `0x1801094a0`); `0x180109a70` (99) is its partition step [likely] |

## Boot statistics and RANSAC settings [sure unless tagged]

| Function | Lines | Role |
|----------|-------|------|
| `0x1800c7f00` | 176 | end-of-boot statistics (log tag `ComputeBootStats`), called by the boot step `0x1800d8b60`: over the list of boot candidates (80-byte entries at `+0x1048`/`+0x1050`) it takes mean, SD, min and max of the score (`+0x44`) and of the inlier count (`+0x40`); feeds each candidate pose to `0x1800c7470`, then `0x1800c78c0`; stores a boot ratio (candidates / attempts, scaled) at `+0x1064` and the spread at `+0x1068`, logs both lines, then empties the list. Diagnostic only: nothing reads `+0x1064` for a decision [likely] |
| `0x1800c7470` | 261 | add a pose (3 × 3 rotation + translation) to the statistics accumulator unless an equal one is already in it [sure] |
| `0x1800c78c0` | 231 | finish the accumulator: mean resultant length of the rotation axes (norm of each averaged column, mean of the three; 1.0 = all poses agree), pose count and the largest angle to the mean (`maxPoseErrorDeg`) [likely] |
| `0x1800c7280` | 93 | accumulator ctor [sure] |
| `0x1800c0e10` | 194 | RANSAC settings block, default ctor (inside the controller ctor `0x1800d2360`): max iterations 1000 / 1000, `+0x08` = 12, adaptive flag 1, three identity pose slots, Mersenne Twister seeded 666, sub-block `0x1800c07e0` [sure] |
| `0x1800c1310` | 251 | the same block from explicit parameters (thresholds as floats, a 16-byte header and the seed from the caller's struct), with float defaults 6.0 (sample size), 200.0 and 3.0. No caller in this build [sure] |
| `0x1800e6450`, `0x1800e7120`, `0x1800e8270`, `0x1800e8e00` | 124–217 | copy / resize / insert for a vector whose items are 32-byte `RnD::Vector<float>` objects (allocate through the allocator, deep-copy each inner vector) [likely] |

Close-range path helpers (`0x1800a88f0`):

| Function | Lines | Role |
|----------|-------|------|
| `0x1800cf7d0` | 311 | for both cameras of the pair: map each 2D observation through that camera's model (`0x18009eb50`) to a normalised ray [likely] |
| `0x1800cf540` | 208 | rotation helper: matrix from a quaternion (`0x180044bc0`) with SIMD `acos`/`sincos` (`0x18009e170`, `0x18009e2a0`), i.e. a slerp-style blend between two orientations [guess] |

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
