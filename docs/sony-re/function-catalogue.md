# Function catalogue

Per-function notes for functions that the topic files do not describe. Parts D and E are generated (`scripts/sony-re/triage/trivial.py`, `libcat.py`); only their first column counts as documentation.

Parts:

- [Part A: ctrl-boot](#part-a-ctrl-boot)
- [Part B: ctrl-tracker UKF internals](#part-b-ctrl-tracker-ukf-internals)
- [Part C: CLAPACK](#part-c-clapack)
- [Part D: Trivial functions](#part-d-trivial-functions)
- [Part E: Library and runtime code](#part-e-library-and-runtime-code)

---

## Part A: ctrl-boot

One row per function that no other doc describes. Roles are in our own
words; tags as elsewhere (`[sure]` read from the code, `[likely]`,
`[guess]`). Sizes are decompiled lines. The pipeline itself is in
`controller-tracking.md` §B.2 and `controller-tracking.md` §A.5.3; this file only fills the gaps.

Object names used below:

- **tracker** = `RnD::HmdTrackerIntf` (`0x1800a12d0` ctor). Per-controller
  blocks of 0x1d00 bytes from `+0x20`; controller count at `+0x15c40`;
  processing order at `+0x15c44`.
- **camera manager** = `DAT_180616720[cam]`; its controllers are 0x4ce0-byte
  objects at `*(mgr + 0x10)`.
- **n_cams** = `DAT_1805e69b0`. Several offsets differ when it is < 2.
- **RnD::Vector** = `{allocator*, begin, end, cap}`; the allocator's vtable
  slot 1 allocates, slot 3 frees.

### Boilerplate groups [sure]

| Group | Functions | What they are |
|-------|-----------|---------------|
| XOR string decoders | `0x1800aa0b0`–`0x1800aa950` (19) | Inline decoders for obfuscated log strings: XOR a stack buffer with fixed 64-bit keys. No logic. |
| RnD::Vector grow | `0x1800aab40` (8-byte items), `0x1800aad20` (16-byte), `0x1800ab430` (reserve n) and more (`shapes.py` label `vec_grow`) | push-back growth through the allocator |
| RnD::Vector free | `0x1800aacf0`, `0x1800ab0a0`, `0x1800ab540` and more (`vec_free`) | free storage, zero begin/end/cap |

### Tracker object

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

#### `0x1800a32b0`: output gather and "near" hysteresis [sure unless tagged]

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

### Debug overlay (fluff) [sure]

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
| `0x1800d5fc0` | 750 | tracking-state overlay (`controller-tracking.md` part B) |
| `0x1800d6e50`, `0x1800d7410` | 320, 699 | one controller: LED projections, pose, blobs |
| `0x1800d7da0` | 162 | pose axes |
| `0x1800d8890`, `0x1800a1e80` | 63, 66 | integer and controller-id labels |
| `0x1800a1ce0`, `0x1800a1d60` | 19, 33 | per-controller overlay thunks |
| `0x18004c590`, `0x18004c600`, `0x18004c9c0`, `0x1800474f0` | 14–660 | overlay entry points in the tracking frontend |

### Raw-image blob extraction (unreachable on PC) [sure for reachability]

A complete PC-side blob detector for raw camera images exists, but nothing
calls it in this DLL: its only entry `0x18004ad80` has no caller and no
data reference, and the DLL exports only `HmdDriverFactory`. The headset
does the detection and sends `LD` blob packets instead (`controllers.md` part C).
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

### Cross boot (worker job type 2) [sure unless tagged]

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

### Solver classes: vtable slots [sure for the map, likely for slot roles]

The refine and multi-camera solves (`controller-tracking.md` §B.2.10, entry
`0x1800f7970`) use these classes. Robustifier slot meanings are in
part A; here are their addresses.

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
| `0x180108920` | 71 | residual scale after a residual pass: median of the absolute residuals through `0x1801046e0` (the 1.483 × median rule of `controller-tracking.md` §B.2.10) [likely] |
| `0x180106d70`, `0x180108100` | 145 | residual statistics of a cost object: RMS, max, min of the residuals and mean weight (residuals `+0xb0`, weights `+0x100`; the second reads `+0xc0`/`+0x110`). No caller in this build [sure] |
| `0x180109670` | 179 | partial sort (introselect) of a float array, the core of the median used by `0x1801046e0` (via `0x1801090f0` / `0x1801094a0`); `0x180109a70` (99) is its partition step [likely] |

### Boot statistics and RANSAC settings [sure unless tagged]

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

### Multi-camera frame step (`0x180048690`, tracking frontend) [sure unless tagged]

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

#### Next-frame pose prediction (`0x1800ceaa0`) [sure]

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
tracked pose (flag `+0x9200`, `controller-tracking.md` §B.2.14).

### Cross-camera agreement filter (`0x1800a45f0`) [sure]

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

### Other large functions in `0x1800a0000`–`0x1800affff`

| Function | Lines | Role |
|----------|-------|------|
| `0x1800a6190` | 249 | publish one controller's solve into the tracker's output block (stride 0x1d00): status, flag bytes `+0x1f9c..+0x1f9e`, pose (rotation as matrix via `0x180044bc0`), the inlier LED → blob list (24-byte items at `+0x398`, count `+0x390`, LED map `+0x1b94`; LEDs without a blob get −1), a second pose at `+0x1c40` and 0xc0 bytes of covariance-like data [sure for the copy; meaning of the second pose: likely the predicted pose] |
| `0x1800a7c50` | 413 | text parser for a debug LED-model override (header `%d %d %d`, LED count ≤ 64, lines `M=%f %f %f`, `N=%f %f %f`, `RING=`); can add random noise (Mersenne Twister seed 20) but the factor is 0. Only reached from the `ConfigFile0.trk` debug path `0x18004a340` [sure] |
| `0x1800acfb0`, `0x1800add40`, `0x1800aeff0`, `0x1800ae860` | 136, 156, 36, 33 | `std::sort` on (float key, payload) pairs, descending: introsort loop, partition with median-of-9, median-of-3, heap sift [sure] |
| `0x1800ae2b0`, `0x1800af120`, `0x1800aeb30` | 156, 36, 33 | the ascending instance used by `0x1800ad2b0` (`controller-tracking.md` §B.2.14) [sure] |
| `0x1800af280`–`0x1800af5b0` | 36–49 | elementary rotation matrices about x / y / z from an angle (scalar or vector lane), sine and cosine from `0x18009e2a0` [sure] |
| `0x18009e170`, `0x18009e2a0` | 83, 115 | SIMD `acos` and SIMD `sincos` [sure] |
| `0x180044bc0` | 120 | 3×3 rotation → quaternion [sure] |
| `0x1800ac160`, `0x1800ac9b0`, `0x1800ad610` | 51–68 | RnD::Vector of 16-byte items: resize with fill, reserve [sure] |

### Multi-camera fusion (`0x1800a37f0`) [sure unless tagged]

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

#### Relative-pose estimators [sure]

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

---

## Part B: ctrl-tracker UKF internals

These functions were first labelled as a linear-algebra library
(part C explains the mix-up). They are the controller UKF of
`controller-tracking.md` part G: the generic sigma-point solver, fixed-size unscented transforms,
their double-precision matrix kernels, the per-sample IMU path and the
filter (re)initialisation. Own words; tags as elsewhere. Sizes are
decompiled lines.

Matrices are small Sony C++ objects: `{double* data, int rows, int cols}`
(dense) or `{double* data, int n}` (packed upper triangle, n(n+1)/2
entries, index of (i, j) with i ≤ j is `j + i·(2n − i − 1)/2`). They are
allocated through an allocator struct of two callbacks (alloc, free)
[sure].

### B.1 Generic solver `0x1800942f0` (858 lines) [sure unless tagged]

Called from the measurement update `0x18006fe90`. Inputs: state mean x
(15), its packed covariance P (15), the noise covariance Q (packed,
size k), the measurement z, a callback that maps an augmented sigma point
to a predicted measurement (`0x18006f020`), and a workspace.

1. Augmented mean = [x; 0_k] (`0x18008ef30`) and augmented covariance =
   blockdiag(P, Q) (`0x18008efe0`, twice), n = 15 + k.
2. Weights for α = 1, β = 2, κ = 0: λ = 0, so the mean weight of the
   central point is 0, its covariance weight is 0 + 2 = **2.0**, and every
   other point has weight 1/(2n). The spread is √n.
3. Cholesky factor L of the augmented covariance (`0x18008f530`). Failure
   → return **−3**.
4. Propagate the **central point** through the callback, then the 2n points
   x ± √n·L(:, j) (`0x18008f740` / `0x18008fb20` add or subtract the
   scaled column). Any callback failure → return **−1**.
5. Predicted measurement ŷ = Σ wᵢ yᵢ (`0x18008fd30`, `0x18008f740`).
6. Innovation covariance S = 2.0·(y₀ − ŷ)(y₀ − ŷ)ᵀ + Σ wᵢ (yᵢ − ŷ)(yᵢ − ŷ)ᵀ
   (`0x18008f950`, `0x18008ffe0`, `0x1800902d0`, `0x180090640`), cross
   covariance Pxy accumulated the same way (`0x18008f2b0` clears it,
   `0x180090430` adds). The central point enters the covariances with
   weight 2 but not the mean.
7. S is copied to packed form and inverted with `DPPTRF` + `DPPTRI`
   (part C). If `DPPTRF` reports a negative info → return
   **−4**. If it reports a positive info (S not positive definite) the
   code skips `DPPTRI` and carries on with the half-factored matrix
   [sure for the branch; a port should treat it as a failure].
8. K = Pxy·S⁻¹ (`0x180090b50`), innovation ν = z − ŷ (`0x18008f950`),
   x ← x + K·ν (`0x18008f740` on the 15 state rows),
   P ← P − K·S·Kᵀ (`0x180091010`), stored back packed (`0x180090850`).
   Return 0.

This corrects `controller-tracking.md` §G.2, which said there is no central point.

#### Solver kernels [sure for the operation, from the element loops]

| Function | Lines | Operation |
|----------|-------|-----------|
| `0x18008ee30` | 11 | allocate a dense rows × cols matrix through the allocator |
| `0x18008ee80` | 10 | allocate a packed n × n matrix (n(n+1)/2 doubles) |
| `0x18008eed0`, `0x18008ef00` | 7, 9 | free a matrix through the allocator, zero its size |
| `0x18008ef30` | 60 | copy a vector into the head of a longer one (rest zero) |
| `0x18008efe0` | 59 | copy a packed block into a larger packed matrix at a diagonal offset |
| `0x18008f1c0`, `0x18008f2b0` | 46 | fill a vector / a matrix with one value |
| `0x18008f3b0` | 68 | zero n doubles (no caller) |
| `0x18008f530` | 104 | Cholesky factor of a packed matrix ('Lower', through `DSCAL`/`DSPR`); nonzero on failure |
| `0x18008f740` | 89 | b += a (vectors) |
| `0x18008fb20` | 89 | b = a − b |
| `0x18008f950` | 83 | c = a − b |
| `0x18008fd30` | 66 | c = s·a |
| `0x18008fe90` | 83 | scaled outer product into a dense matrix (no caller) |
| `0x18008ffe0` | 75 | outer product d·dᵀ into a packed matrix |
| `0x1800902d0` | 65 | scale a packed matrix |
| `0x180090430`, `0x180090640` | 87, 90 | add one matrix into another (dense / packed) |
| `0x180090850` | 78 | P ← P − X, packed |
| `0x180090a00` | 66 | invert a packed SPD matrix (`DPPTRF` + `DPPTRI`), −2 on failure (no caller) |
| `0x180090b50` | 127 | dense × packed-symmetric product (the gain K = Pxy·S⁻¹) |
| `0x180090e90` | 58 | matrix × vector |
| `0x180091010` | 243 | P − K·S·Kᵀ into a packed result |

#### Work-matrix setup [likely]

| Function | Lines | Role |
|----------|-------|------|
| `0x180092020` | 105 | allocate the solver workspace (sigma-point, mean and covariance matrices) for a given state and noise size; callers are the filter ctor and reset paths (`0x18005d970`, `0x180063d30`, `0x18006d890`) |
| `0x1800928f0` | 55 | free that workspace |
| `0x180092300` | 75 | allocate 6-sized blocks (one packed 6 × 6, 21 doubles) and zero them |
| `0x180092560`, `0x180092630` | 29, 36 | build the IMU noise block (`0x180091700`) / the measurement noise block (`0x180091930`, `controller-tracking.md` §G.2) |
| `0x180091700`, `0x180091930` | 100, 194 | allocate and fill those blocks |
| `0x180092a60`, `0x180096bb0`, `0x180096c80` | 20–39 | wrappers over the above with no caller |
| `0x180091580`, `0x180091d70`, `0x180091eb0` | 51–62 | further block allocators with no caller |
| `0x180092b00`, `0x1800936e0` | 501, 513 | two more copies of the generic solver with a different callback signature; only `.pdata` refers to them, so no code reaches them in this build [sure for reachability] |

### B.2 Fixed-size unscented transforms [sure for sizes and callers]

Besides the generic solver there are hand-sized copies of the same
algorithm (same weights; spread √n with n fixed). Each takes a mean, a
packed covariance, a noise block and a callback, and returns −2 or −3
when the Cholesky step fails.

| Function | Lines | n | Callback | Called from |
|----------|-------|---|----------|-------------|
| `0x180072a00` | 313 | 21 (15 + 6) | `0x18006e520` / `0x18006e540` (both forward to `0x18006dbd0`) | stationary damping `0x18006e5a0` |
| `0x180073110` | 179 | 12 (6 + 6) | `0x18006e560` / `0x18006e580` | stationary damping `0x18006e5a0` |
| `0x180073560` | 327 | 6 | `0x180071670` | `0x1800719a0` |
| `0x180073c10` | 425 | 18 | `0x180070d10` → `0x180070950` | measurement step `0x180070d50` |
| `0x180074540` | 320 | 12 | `0x180070d30` → `0x180070950` | measurement step `0x180070d50` |

`0x1800729b0`–`0x1800729f0` are 6-line thunks into these five.

Per-size kernels (one copy per size; the role follows from the
signature and the call position) [likely]:

| Role | n = 21 | n = 12 | n = 6 | n = 18 | n = 12 (`0x180074540`) |
|------|--------|--------|-------|--------|---------|
| copy a block into the augmented covariance | `0x180074c60`* | `0x180075f30` | | `0x180077130`*, `0x180077280`* | `0x180075f30` |
| packed Cholesky ('Lower') | `0x180074f10`* | `0x180076090` | `0x180076b00`* | `0x1800773a0`* | `0x180076090` |
| sigma point x ± s·L(:, j) | `0x180075090`, `0x180075300`* | `0x180076210`, `0x180076480`* | `0x180076c70`, `0x180076ee0`* | `0x180077520`, `0x180077790`* | `0x180076210` |
| weighted mean (add a scaled vector) | `0x180075660` | `0x1800767a0` | `0x1800767a0` | `0x180077ac0` | `0x1800767a0` |
| weighted outer-product accumulate | `0x180075840` | `0x180076980` | | `0x180077cd0` | `0x180076980` |
| cross covariance accumulate | `0x1800754d0` | `0x180076650` | `0x180076650` | `0x180077960`, `0x180077d90`* | `0x180076650`, `0x180078550`* |
| invert S (`DPPTRF` + `DPPTRI`) | | | | `0x180077eb0` | `0x180078630`* |
| gain, state and covariance update | | | | `0x180077f30`, `0x180078080`, `0x180078190` | `0x1800786c0`, `0x180078860`, `0x180078940` |

\* no caller in this build (copies left after inlining).

The unreachable tilt filter of §B.4 (`0x18007dc60`) has its own two
transforms, `0x180097990` (158 lines) and `0x180097d90` (245), with the
same kernel set [likely]: block copy `0x180098370`, `0x1800984c0`;
Cholesky `0x1800985e0`; sigma point `0x180098760` (`0x1800989d0`* the
subtracting copy); mean `0x180098c40`; outer-product accumulate
`0x180098e10`; cross covariance `0x180098ba0`; update `0x180098f40`,
`0x180098ff0`, `0x180099040`; and a callback `0x180097340` that rotates a
vector by the sigma point's attitude (`0x180058720`). `0x180097110` and
`0x1800975f0` are the two correction calls of `0x18007dc60` into them.

### B.3 Measurement step and its callbacks [sure unless tagged]

| Function | Lines | Role |
|----------|-------|------|
| `0x180070d50` | 211 | measurement step called by `kalmanFilterStep` (`0x180065b70`): picks the full-pose path (n = 18, `0x180073c10`) or the position-only path (n = 12, `0x180074540`) by the byte `+0x1a8` of its first argument; converts the state with `0x18006c4c0` (full) or `0x18006ce80` (position-only) first and writes the covariance back with `0x18006c1b0`, the state with `0x18006c760` |
| `0x180070950` | 159 | callback of both paths: maps one sigma point to the predicted measurement, rotating the three 0.1 m axis tips by the point's attitude (`0x180058720`) [likely: the `h(x)` of `controller-tracking.md` §G.4] |
| `0x180070d10`, `0x180070d30` | 9 | adapters to `0x180070950` for the two sizes |
| `0x1800713b0` | 79 | compose two rotations through the 3×3 product (`0x1800621a0`) [likely] |
| `0x180071670` | 125 | callback of the n = 6 transform: quaternion product (`0x180061fa0`) then `0x1800713b0` [likely] |
| `0x1800719a0` | 683 | pre-update pass from `0x180065760`: predicted pose for the innovation gate (`controller-tracking.md` §C.4.1); uses the relative yaw `0x18006b790` and builds a half-angle rotation (sin, cos), then runs the n = 6 transform on the rotation [likely] |

### B.4 IMU sample path and filter set-up [likely unless tagged]

| Function | Lines | Role |
|----------|-------|------|
| `0x180079e10` | 104 | find the ring entry for a timestamp (log tag `LIT::searchOffsetByTimestamp`); returns 0, −1 or −2 [sure for the codes] |
| `0x18007a1f0` | 204 | reset of the per-hand filter object from `0x1800638f0`: clears counters, stores 32 at `+0x308` and two 0.01 constants [sure for the stores] |
| `0x18007c080` | 660 | per-IMU-sample processing called from the sample feed `0x1800641d0`: updates the state ring, may replay (`0x18007add0`), and runs the still / gravity checks `0x180099400`, `0x1800998f0` and `0x18009a1a0` |
| `0x180099400` | 196 | still test: accelerometer magnitude within a tolerance of 1 g and small rotation, with a slow (0.05) running correction [guess for the use] |
| `0x1800998f0` | 430 | fit of accelerometer samples against gravity (0, 1, 0) in g: residual sums per axis [guess for the use] |
| `0x18009a1a0` | 445 | collects samples with µs timestamps (× 1e-6) and calls the fit `0x18007bbd0` with a 0.1 threshold, then `0x18009b7c0` [guess: online accelerometer calibration] |
| `0x18007a9a0` | 217 | push one IMU sample into the ring (from `0x1800641d0`, `0x180068ee0`) through `0x18005a570`, `0x18005a940`, `0x180079750` |
| `0x180081bb0` | 289 | initialise a state from the first valid measurement (from `0x180068620`, `controller-tracking.md` §G.2): copies the record (`0x180064f40`), attitude from `0x1800813d0` with a 0.999 threshold |
| `0x1800821a0` | 191 | extrapolate a state to a later time: p + v·dt + ½·a·dt², time += dt·10⁶ µs (from `0x180068620`) [sure for the formula] |
| `0x18007e8f0` | 74 | wrapper over `0x180079e10`, `0x180081bb0` and `0x1800821a0`; no caller |
| `0x18007cde0` | 259 | a second copy of the sample feed (log "drop sample due to large gap"); no caller [sure for reachability] |
| `0x18007dc60` | 418 | "correct(tilt)" step of an IMU-only tilt filter (logs "correct(tilt): num of sensor data is larger than internal buffer" and "(i) %p: BadState"); runs its own sigma-point update (`0x180097110` → `0x180097990`, `0x1800975f0` → `0x180097d90`) — **no caller in this build**, so the PC driver never runs a tilt-only correction [sure for reachability] |

### B.5 Helpers of the IMU path and the filter object

| Function | Lines | Role |
|----------|-------|------|
| `0x18007a6a0` | 52 | free the per-hand filter's buffers through the allocator; −1 if the "initialised" byte `+0x448` is clear [sure] |
| `0x18007a860` | 82 | under the filter mutex (`+0x3a8`) copy two 128-byte blocks between the caller's buffer and the filter at `+0x48` (caller `0x180063f50`) [sure for the copy; direction not checked] |
| `0x18007d930` | 28 | NaN check of a state entry (attitude doubles `+0x60..+0x78`, then the float fields up to `+0xb0`): true if any is NaN, the "BadState" test [sure] |
| `0x18007db00` | 56 | bias limit check: if any gyro-bias (`+0xd0`) or accel-bias (`+0xc0`) component exceeds the config limits (`+0x148`, `+0x144`) the bias is reset; no caller [likely] |
| `0x18007bf50` | 59 | per-axis test \|a − b\| > threshold for a float3; no caller [sure] |
| `0x18007eb70` | 126 | add one IMU sample to running sums kept as fixed-point integers (scale 2²³), after rotating it by the current attitude (`0x180061fa0`); from `0x18007c080` [likely] |
| `0x18007f950` | 25 | mean of those sums once the window is full (sum × 2⁻²³ / count, plus an offset) [sure] |
| `0x18007f370` | 99 | the same for an accumulated 3 × 3 rotation sum, then a 3 × 3 product (`0x1800621a0`) [likely] |
| `0x18007f180` | 39 | 3 × 3 rotation matrix from a quaternion (1 − 2(y² + z²) form), written to the slot chosen by the count at `+0x78` [likely] |
| `0x18007ef30`, `0x18007f740` | 129, 115 | callers of the window means from `kalmanFilterStep` (`0x180065b70`) [sure for the calls; use not read] |
| `0x180079a80` | 108 | attitude from the gravity direction: compares the accelerometer vector with 9.80665 m/s² and builds the tilt quaternion (used by the replay `0x18007add0`) [likely] |
| `0x180079370` | 70 | tilt rotation that turns the measured gravity vector onto +y (cross product with (0, 1, 0)); no caller [likely] |
| `0x180079880` | 96 | copy a sample's pose into a ring entry and form differences to the previous entry; no caller [guess] |
| `0x1800795a0`, `0x180081750`, `0x180082740`, `0x180095f90` | 55, 54, 54, 54 | exp map rotation vector → quaternion, with the Taylor form below √ε (ε = 2⁻²³) computed once into a static; double / float variants [sure] |
| `0x1800818d0`, `0x180096110` | 74, 81 | integrate an attitude by ω·dt with that exp map (sin/cos of the half angle, series `0.5 − θ²/48` below the threshold); float / double [sure] |
| `0x1800828c0` | 40 | q ⊗ exp(ω·dt) then renormalise (`0x180082740`, `0x180061fa0`) [sure] |
| `0x180082980`, `0x180082c90` | 58, 13 | closed-form integral of a cubic time polynomial (coefficients with the 12, 6, 4 terms of a Hermite form) and its difference between two times; used by the IMU interval reduction `0x180082e90` [likely] |
| `0x180082ce0` | 67 | builds the cubic's coefficients from a sample pair, then integrates it (`0x180082740`, `0x180082980`) [likely] |
| `0x1800812c0` | 22 | linear interpolation of a float3 [sure] |
| `0x180081600` | 39 | p + dt·v for a float3, optionally also the second vector [sure] |
| `0x180081280` | 16 | divide a float3 by a scalar [sure] |
| `0x1800791f0` | 34 | per-component square root of a float3 (σ from variances) [sure] |
| `0x180099320`, `0x1800993c0` | 27, 17 | normalise a float3; scalar square root [sure] |
| `0x18009abf0` | 30 | determinant of a 3 × 3 matrix (triple product) [sure] |
| `0x18009a930` | 33 | monotonic time in ns from `QueryPerformanceCounter` (first call caches a start value) [sure] |
| `0x1800790d0` | 32 | `memcpy_s` [sure] |
| `0x1800792e0`, `0x1800804b0`, `0x180080530`, `0x1800805e0`, `0x180080660`, `0x180080700`, `0x1800807b0`, `0x180080850`, `0x180080900` | 14–16 | allocate one ring or array of the filter object through the allocator (fixed sizes 128 × 60 B, 128 × 456 B (the 0x1c8 state entries), 32 × 64 B, or n × 1, 8, 16, 24, 96 B) [sure] |
| `0x180092750` | 60 | free the noise blocks of §B.1 [sure] |
| `0x18007faf0`, `0x18007fc90`, `0x18007fe60`, `0x18007ff50`, `0x1800801c0`, `0x180080310` | 16–24 | in-place XOR decoders for obfuscated log strings (shared key table); no caller [sure] |
| `0x180074db0` | 65 | copy a 6 × 6 packed block into a packed 21 × 21 matrix at offset 15 (the IMU noise into the augmented covariance of the n = 21 transform); no caller [sure] |
| `0x180098eb0` | 26 | 3-wide cross-covariance accumulate (tilt filter); no caller [sure] |

#### Still / calibration path from `0x18007c080` [guess unless tagged]

| Function | Lines | Role |
|----------|-------|------|
| `0x180099860` | 25 | still test wrapper: threshold 6·√\|v\|, then `0x180099400` with 0.1 [sure for the call] |
| `0x180095760`, `0x180095b00` | 156, 44 | sample acceptance tests with 100.0 and 1.5 limits; the wrapper adds a √ / 4.0 check |
| `0x180095c30` | 106 | windowed consistency check of the collected samples |
| `0x180095e50` | 38 | double-precision normalise and scale of a float3 |
| `0x1800963c0`, `0x180096570`, `0x180096740`, `0x180096d90` | 44–162 | rotation conversions for the tilt filter: matrix → quaternion (Shepperd form with 0.25 / 4.0 / 16.0 terms) [likely], composed with the gravity attitude `0x180079a80` and the exp map `0x180096110` |
| `0x18009aa00`, `0x18009aca0` | 70, 49 | normal-equation accumulators for a 4-parameter fit |
| `0x18009b420`, `0x18009b7c0`, `0x18009bf60`, `0x18009c400` | 163–302 | 4-parameter least-squares fits solved with the 4 × 4 inverse `0x180059000` (centre and radius of a sphere through accelerometer samples, i.e. an accelerometer offset calibration); only `0x18009b7c0` is reached (from `0x18009a1a0`) |
| `0x18009aea0`, `0x18009b080` | 78, 119 | per-axis acceptance flags for that fit (fabs tests); no caller |

### B.6 `UkfManager` object, snapshot I/O and helpers (`0x180050000`–`0x18006ffff`)

#### Manager and filter object [sure unless tagged]

| Function | Lines | Role |
|----------|-------|------|
| `0x180061950`, `0x18005d180`, `0x18005d2d0` | 15–19 | create the `UkfManager` singleton (0x4460 bytes; two per-hand filters of 0x2218 bytes, ctor `0x18005d440`, dtor `0x18005d4b0`) and a 1.5 MB (0x180000) arena |
| `0x180061920` | 16 | destroy the singleton |
| `0x18005d0e0` | 25 | bump allocator in that arena with alignment; logs "Memory allocated for pad_tracker is not enough" when the 1.5 MB is used up |
| `0x18005d220` | 35 | reset both filters (`0x180063d30`) |
| `0x18005d4b0`, `0x18005d530` | 19, 15 | per-hand filter dtor: measurement buffer (`Array<VisionBufferElement<DotMarkerData>,4>`, 0xb0-byte entries) and five mutexes (`+0x3a8`, `+0x3f8`, `+0xee0`, `+0xf30`, `+0xf80`) |
| `0x18005cff0` | 12 | destroy two of those mutexes |
| `0x18005c0d0` | 19 | reset a filter's working state: clear counters, zero the pose block (`0x18005a570`) and the covariance (`0x180060a40`) |
| `0x18006d6e0` | 45 | allocate the filter's work matrices (15-vector, packed 15 × 15 double, ...) and the noise blocks (`0x180092300`, `0x180092560`, `0x180092630`) |
| `0x18005dff0` | 73 | start a filter from an external pose: `0x18005dab0`, then the first-measurement init `0x180068620` [likely] |
| `0x18005e240` | 72 | convert one state entry to the exported pose record: time `+0x108`, position, velocity, attitude (double → float), rates [sure] |
| `0x18005e340` | 106 | under the manager mutex, copy up to n newest state entries, newest first, as 184-byte exported records (`0x18005e240`) [likely: the history query] |
| `0x18005e590` | 21 | read one field of a filter under its mutex `+0xf58` |
| `0x18005e9c0`, `0x18005eac0` | 89, 97 | copy the three 128-byte records at `+0x6c8` out, then reset (`0x180063f50`) [likely] |
| `0x18005f8b0` | 22 | store position and quaternion into a measurement record |
| `0x18005f8f0` | 10 | set one flag byte in both per-hand filters (`+0x7f8`) |
| `0x18005b770`, `0x18005b790` | 10, 51 | filter config defaults: two 32-byte constant blocks, 0.001 and 0.1 (as raw floats), 60000 (µs) [sure for the values] |

#### Snapshot save / restore [sure]

A filter can be written to and read back from a flat buffer. The buffer
starts with the magic `UKF` plus a zero byte and must be at least 2 MB
(`0x18005e670`, which logs "snapshot header is invalid magic." otherwise);
`0x18005c130` writes, `0x18005c7f0` reads.

| Function | Lines | Role |
|----------|-------|------|
| `0x18005e670` | 17 | check the magic and size, then restore |
| `0x180057f30`, `0x1800580b0` | 18, 21 | write the two top-level blocks |
| `0x180057fa0`, `0x180058130` | 58, 80 | read them |
| `0x180057050`, `0x180057560` | 77, 138 | write composite blocks (several vectors and matrices) |
| `0x1800571e0`, `0x180057850` | 210, 371 | read composite blocks |
| `0x180056b60`, `0x180056df0`, `0x180056f20` | 39 | write a vector (count, then 8-byte items) |
| `0x180056c10`, `0x180056ea0`, `0x180056fd0` | 28 | read a vector |
| `0x180056c90`, `0x180056d70` | 40, 22 | write / read a dense matrix (rows, cols, items) |
| `0x18005aae0`, `0x18005b010` | 282, 366 | write / read the state ring (456-byte entries) |
| `0x180059d90`, `0x180059f00` | 84, 81 | write / read a ring of 96-byte entries |
| `0x18005a160`, `0x18005a280` | 72, 66 | write / read a ring of 24-byte entries |
| `0x180060910`, `0x1800608a0` | 31, 30 | write / read the measurement ring (480-byte entries) |
| `0x180060ba0`, `0x180060b30` | 31, 30 | the same for 60-byte entries |
| `0x180060cb0`, `0x180060c40` | 31, 30 | the same for 456-byte entries (a second state ring) |
| `0x180060e00`, `0x180060d90` | 33, 30 | the same for 16-byte entries |
| `0x180060f10`, `0x180060ea0` | 31, 30 | the same for 24-byte entries |
| `0x180061020`, `0x180060fb0` | 33, 30 | the same for 64-byte entries |
| `0x180061130`, `0x1800610c0` | 31, 30 | the same for 96-byte entries |
| `0x180056a10`, `0x1800619d0` | 32 | `memcpy_s` copies used by these |

#### State, covariance and ring helpers [sure unless tagged]

| Function | Lines | Role |
|----------|-------|------|
| `0x180060a40` | 24 | reset a packed 15 × 15 float covariance (480 bytes) to the identity |
| `0x1800634b0` | 25 | build a diagonal packed covariance from five float3 variances (position, velocity, attitude, gyro bias, accel bias) |
| `0x180065390` | 85 | P0 for a new filter: attitude (1°, 5°, 1°)², velocity 0.04, position 0.0025, through `0x1800634b0` (values of `controller-tracking.md` §A.5.5) |
| `0x180063220`, `0x180063310`, `0x1800633c0` | 46, 23, 22 | clear / read / write a 3 × 3 block of the packed 15 × 15 covariance (row offset by the 29 − i rule) |
| `0x18006ace0` | 56 | fill a packed double matrix with one value |
| `0x18006abf0`, `0x18006ac30`, `0x18006ac80`, `0x18006acb0` | 13, 13, 10, 10 | allocate a double vector / packed matrix; free them |
| `0x18006a000`, `0x180063890` | 14, 13 | allocate a 128-entry ring of 480-byte measurements; a 128 KB block |
| `0x180062710`, `0x18006a240` | 48, 45 | copy one state ring into another when the sizes match |
| `0x180062c30` | 53 | sanity check of the newest state entry: any position component beyond 10000 → reset [likely] |
| `0x180062d60`, `0x1800675e0` | 104, 44 | find a ring entry by timestamp (log tag `LIT::searchOffsetByTimestamp`); index lookup with −1 when out of range |
| `0x1800654f0` | 134 | start a state entry from another one: copy time and attitude, zero the velocity and biases; no caller [likely] |
| `0x180067410` | 67 | divergence test as in `controller-tracking.md` §A.5.5 (\|v\| over the limit at config `+0x798`, gyro or accel bias over `+0x7a0` / `+0x79c`) followed by a covariance reset; no caller (the live copy is inside `0x180067b50`) |
| `0x1800676a0` | 169 | build a measurement-time snapshot of the newest state and measurement (`0x180067900`, slot `+0x50` of the measurement); no caller |
| `0x180059cb0`, `0x180059d20`, `0x18005a060`, `0x18005a0c0`, `0x18005a110` | 13–22 | state-entry ctors: zero fields, default standard deviations 0.01 |
| `0x180059b60`, `0x180059bc0`, `0x18005a390`, `0x18005a3e0`, `0x18005a430`, `0x18005a800`, `0x18005a860` | 7–30 | pose / vector record ctors (zero position, unit axis, cleared flags) |
| `0x180058c60`, `0x180058cd0` | 11 | 3 × 3 identity; diagonal 3 × 3 from a float3 |
| `0x180058bc0` | 4 | read element (r, c) of a 4-wide matrix |
| `0x180062490`, `0x180062540` | 22, 20 | 3 × 3 transpose between row and column layouts |
| `0x180062610` | 9 | apply a rotation to the four columns of a 4 × 4 pose (`0x180058e40`) |
| `0x180063140` | 32 | 3 × 3 product then repack (`0x1800621a0`) |
| `0x180063580` | 72 | combine several float3 inputs with three 3 × 3 matrices into one result; no caller [guess] |
| `0x18005f150`, `0x180061900` | 3, 2 | `DotMarkerData` vtable stores (ctor / dtor) |
| `0x180060030` | 62 | `DotMarkerData` slot: the measurement mapped through a 4 × 4 transform (0 if invalid) |
| `0x180060570` | 16 | `DotMarkerData` slot `+0x60`: the vector at `+0x38` if valid (`controller-tracking.md` §G.3) |
| `0x1800617a0`, `0x180060590`, `0x180060690`, `0x1800612f0`, `0x180061370`, `0x180061880` | 9–62 | the 4-entry measurement buffer (`VisionBufferElement`, 0xb0 bytes): shift entries down, insert, pop and count |
| `0x180069a20`, `0x180069b50`, `0x180069d20`, `0x180069e10`, `0x18006a070` | 16–33 | in-place XOR decoders for obfuscated log strings; no caller |

#### Generic code inside this range (library / debug) [sure]

| Function | Lines | Role |
|----------|-------|------|
| `0x180050370`, `0x1800505a0`, `0x180050880`, `0x1800509c0`, `0x180050b60` | 70–80 | `std::string` / `std::wstring` grow and append |
| `0x180050d10`, `0x180052760`, `0x1800527e0`, `0x180053020`, `0x180053070`, `0x180053620`, `0x1800537b0` | 7–19 | `std::map` sentinel and node allocation, recursive node free |
| `0x180052670` | 20 | 32-byte aligned `operator new` |
| `0x180052e80` | 45 | narrow string → `std::wstring` through the filesystem code page (`__std_fs_code_page`, `__std_fs_convert_narrow_to_wide`) |
| `0x180056ae0` | 15 | lock a mutex, throw on error |
| `0x180053840`, `0x180053870`, `0x180053890`, `0x1800538d0`, `0x180053970`, `0x180053a20`, `0x180054710` | 12–29 | `std::thread` / `std::mutex` / `std::condition_variable` wrappers (join, lock, timed wait, move) |
| `0x180054870`, `0x180054a90` | 58, 27 | condition wait with a timeout in ms (clamped to 10 days) |
| `0x180053b00`, `0x180053bd0`, `0x180053cd0`, `0x180053d10`, `0x180053d40` | 13–28 | `IrFramework::Thread` dtor and stop/join; `IrFramework::Event` ctor, dtor and signal |
| `0x180053e70`, `0x180053f30`, `0x1800544d0`, `0x180054490` | 20–24 | `WorkerThread` ctor, dtor, signal, stop |
| `0x180053b20`, `0x180053fc0`, `0x180054570`, `0x180054690`, `0x1800547c0`, `0x1800549e0`, `0x180054bd0`, `0x180054c20` | 11–40 | start a thread through `_beginthreadex` with a heap-allocated (object, entry) pair, the shared thread entry, and the stop of a worker in the global table at `0x1805f65e0` (stride 408) |
| `0x1800542c0` | 53 | worker job dispatch by job type (0–3), e.g. the cross boot `0x1800a5ac0` (part A) |
| `0x180050010`, `0x180050f30`, `0x180051060`, `0x180051190` | 21–63 | destructors of the rig object `0x1805f6cb0` (element arrays, `RnD::Vector` members) |
| `0x180054ec0`, `0x1800558e0` | 313, 291 | draw a text string into an 8-bit image with a 16 × 16-glyph bitmap font (glyph index from the character, size from the last argument); no caller — debug overlay |

---

## Part C: CLAPACK

The DLL links a small f2c-translated CLAPACK (single precision, the
3.2-era version whose `SLAMCH` still computes the machine constants
through `SLAMC1`–`SLAMC5`). The 28 functions below are the whole block.
Each one is matched to its reference routine by the evidence named in the
row: the argument count and order of the reference signature (all
arguments by pointer), the routine name passed to the tuning call
(`ILAENV`), the option strings (`'Left'`, `'Lower'`, `'No transpose'`,
`'Unit'`, ...) and the call graph.

No `XERBLA` is left in the binary (the error calls were dropped), and
link-time optimisation inlined some leaf routines into their callers, so a few copies have no
caller (`ncall 0`) and some argument lists lost a constant argument.

On Linux none of this is ported: any LAPACK (or Eigen) gives the same
results.

### Who calls it [sure]

Only two Sony functions call into the block, both in the ctrl-boot LM
solver (part A, "Solver classes"):

- `0x1800fce20` (45 lines), called from `0x1800f8f80` / `0x1800fa7d0`
  (slot 0 of the two `WeightedLMSolver` vtables): factorises the packed symmetric
  normal-equation matrix with `SSPTRF` and solves with `SSPTRS`. So the
  6 × 6 LM step is a Bunch–Kaufman solve on packed storage, not a
  Cholesky.
- `0x1800fd810` (68 lines), from `0x1800fa430` / `0x1800fbc60`: inverts a
  matrix with `SGETRF` + `SGETRI` (LU, then inverse from the factors)
  [likely: the covariance of the final pose].

### Routines

| Function | Lines | Routine | Evidence |
|----------|-------|---------|----------|
| `0x180087b30` | 193 | `SGETRF` (blocked LU with partial pivoting) [sure] | routine name 'SGETRF'; calls ILAENV, SGETF2, SLASWP, STRSM ('Left','Lower','No transpose','Unit'), SGEMM |
| `0x18008c920` | 242 | `SGETRI` (inverse from the LU factors) [sure] | routine name 'SGETRI'; calls STRTRI, SGEMV, SGEMM, STRSM ('Right','Lower','Unit'), SSWAP |
| `0x18008cfa0` | 190 | `STRTRI` (triangular inverse, blocked) [sure] | routine name 'STRTRI'; options 'Upper','Non-unit','Left','Right'; calls STRTI2, STRMM, STRSM |
| `0x180087980` | 90 | `STRTI2` (triangular inverse, unblocked) [likely] | called only by STRTRI; calls STRMV and SSCAL |
| `0x18008b060` | 190 | `SGETF2` (unblocked LU) [sure] | called by SGETRF; calls ISAMAX, SSWAP, SSCAL, SGER and SLAMCH (via `SLAMC2`); no self-call, so not the recursive `SGETRF2` |
| `0x180086ed0` | 309 | `SSPTRF` (Bunch–Kaufman factorisation, packed symmetric) [sure] | 5 arguments (uplo, n, ap, ipiv, info); constant α = (1 + √17) / 8; packed index n(n−1)/2 + 1; calls ISAMAX, SSWAP, SSCAL |
| `0x1800865a0` | 144 | `SSPTRS` (solve with the SSPTRF factors) [sure] | 8 arguments (uplo, n, nrhs, ap, ipiv, b, ldb, info); packed index n(n+1)/2 + 1; option 'Transpose'; calls SGEMV, SGER, SSWAP, SSCAL |
| `0x180086d70` | 67 | `SSPR` (packed symmetric rank-1 update) [likely] | 6 arguments (uplo, n, alpha, x, incx, ap); skips zero x; no caller (inlined into SSPTRF) |
| `0x18008b550` | 217 | `SGEMM` [sure] | 13 arguments in reference order; called by SGETRF and SGETRI |
| `0x180083a30` | 608 | `STRSM` [sure] | 11 arguments (side, uplo, transa, diag, m, n, alpha, a, lda, b, ldb); the one routine common to SGETRF, SGETRI and STRTRI |
| `0x18008d3f0` | 200 | `STRMM` [likely] | same 11-argument shape as STRSM; called by STRTRI |
| `0x180085530` | 540 | `SGEMV` [sure] | 11 arguments (trans, m, n, alpha, a, lda, x, incx, beta, y, incy); called by SGETRI and SSPTRS |
| `0x180087720` | 186 | `STRMV` [likely] | 7 arguments (uplo, trans, diag, n, a, lda, x); called by STRTI2 |
| `0x180085fb0` | 134 | `SGER` (rank-1 update) [sure] | 9 arguments (m, n, alpha, x, incx, y, incy, a, lda) |
| `0x1800862e0` | 135 | `SSWAP` [sure] | 5 arguments (n, x, incx, y, incy) |
| `0x180086ba0` | 120 | `SSCAL` [sure] | 4 arguments (n, alpha, x, incx) |
| `0x1800852d0` | 111 | `ISAMAX` [sure] | 3 arguments (n, x, incx), returns an index |
| `0x180084b30` | 144 | `SLASWP` (row interchanges) [likely] | called by SGETRF after the panel; unrolled loop over the pivot list; the constant `incx` argument was folded away |
| `0x180087ff0` | 2608 | `ILAENV` (block-size tuning) [sure] | compares the routine name pieces 'GE', 'PO', 'SY', 'TRF', 'TRI', 'QRF', 'BRD', 'TRD', ...; IEEECK and IPARMQ inlined |
| `0x180084f60` | 95 | `IPARMQ` [likely] | ispec range 12–16 test, `nh < 30` rule; no caller (inlined into ILAENV) |
| `0x180085170` | 41 | `IEEECK` [sure] | tests that ±1/0 give ±infinity and that NaN compares unequal |
| `0x18008bb30` | 82 | `SLAMCH` (machine constants) [sure] | first call fills cached eps, base, rounding, emin, emax through SLAMC2; no caller (inlined) |
| `0x18008bcd0` | 65 | `SLAMC1` [sure] | the doubling loop that finds the radix and rounding mode |
| `0x18008be80` | 254 | `SLAMC2` [sure] | calls SLAMC4 and SLAMC5; reached from SGETF2's inlined SLAMCH |
| `0x18008c460` | 82 | `SLAMC4` [likely] | underflow search on one float |
| `0x18008c640` | 149 | `SLAMC5` [likely] | computes emax and rmax |
| `0x180083880` | 59 | f2c `s_copy` [sure] | Fortran string assignment (dest, src, dest length, src length); called by ILAENV |
| `0x180083700` | 76 | f2c `s_cat` [likely] | Fortran string concatenation (pointer list, length list, overlap check); no caller |

### Double-precision subset (`0x18008d910`–`0x18008ee2f`)

A second, smaller f2c block in double precision serves the controller
UKF (part B): packed Cholesky, inverse from the
factor, and the BLAS routines they need. Same evidence as above.

| Function | Lines | Routine | Evidence |
|----------|-------|---------|----------|
| `0x18008d910` | 61 | `DPPTRF` (Cholesky of a packed SPD matrix) [sure] | 4 arguments (uplo, n, ap, info); info = −2 for n < 0; square root of each pivot; calls DSCAL and DSPR; option 'Lower' |
| `0x18008daa0` | 123 | `DPPTRI` (inverse from the DPPTRF factor) [sure] | 4 arguments; calls DTPTRI (inlined), DDOT, DSCAL, DSPR, DTPMV; options 'Lower', 'No transpose', 'Non-unit', 'Transpose' |
| `0x18008dd70` | 75 | `DTPTRI` (packed triangular inverse) [likely] | options 'Non-unit', 'No transpose', 'Lower'; calls DSCAL and DTPMV; no caller (inlined into DPPTRI) |
| `0x18008df50` | 99 | `DDOT` [sure] | returns a double dot product |
| `0x18008e0e0` | 98 | `DSCAL` [sure] | scales a vector (constant increment folded away) |
| `0x18008e270` | 195 | `DSPR` (packed symmetric rank-1 update) [sure] | 6 arguments (uplo, n, alpha, x, incx, ap) |
| `0x18008e5b0` | 381 | `DTPMV` (packed triangular matrix × vector) [likely] | 6 arguments (uplo, trans, diag, n, ap, x) |
| `0x18008ec20` | 80 | `DTPSV` or a second `DTPMV` copy [guess] | same 6-argument shape; no caller |

### The rest of the old `lib:linalg` range

The triage first labelled all of `0x180070950`–`0x18009ffe0` as a
library. Only the block above is one. The rest is Sony code and is now
labelled core (`overview.md` §C.5):

- `0x180070950`–`0x180082e90` and `0x18008ee30`–`0x18009cff0`: the
  controller UKF (`controller-tracking.md` part G, part B) with its
  double-precision matrix kernels, its replay path (`0x18007add0`,
  log `WARNING--->backwards!`), the tilt correction (`0x18007dc60`), the
  IMU interval reduction (`0x180082e90`) and the solver `0x1800942f0`.
  → ctrl-tracker.
- `0x18009d100`–`0x18009ffe0`: RnD math used by ctrl-boot (Mersenne
  Twister `0x18009f3e0`, SIMD `acos`/`sincos`, camera model
  `0x18009eb50`). → ctrl-boot.

---

## Part D: Trivial functions

<!-- gen:trivial:start -->
Generated by `scripts/sony-re/triage/trivial.py` from the body shape of
each function (rules in `shapes.py`). Every row is a leaf or near-leaf
helper: a constant return, a field read, a forwarder, a compiler array
helper, an `RnD::Vector` grow/free, a destructor, a constructor that
only zeroes fields, a record copy or an obfuscated-string decoder. The
role column is in our own words and is `[sure]` for the shape; what the
caller does with the value is in the caller's own entry.

793 functions.

### app:dialog (3)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x18012dd60` | fluff | thunk | forwards to `0x18012ddb0` | none (vtable/callback) |
| `0x18012ddb0` | fluff | dtor | destructor (frees itself when the flag argument is set) | `0x18012dd60` |
| `0x18012de50` | fluff | dtor | destructor of `CreateProcessThread` (frees itself when the flag argument is set) | none (vtable/callback) |

### config (9)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x180154fd0` | ref | dtor | destructor of `ConfigManager` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x1801562f0` | core | copy | copies a record field by field (about 33 fields) | `0x180127250` |
| `0x180157ef0` | fluff | dtor | destructor of `ShareManager` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180158198` | fluff | thunk | forwards to `0x1801581b0` | none (vtable/callback) |
| `0x1801581f0` | ref | dtor | destructor of `ShareManager` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18015c670` | fluff | const_ret | wrapper around `WaitForSingleObject` | none (vtable/callback) |
| `0x18015e230` | core | const_ret | wrapper around `ReleaseMutex` | `0x1801185a0`, `0x18011b700` +1 |
| `0x18015e340` | core | const_ret | wrapper around `SetEvent` | none (vtable/callback) |
| `0x18015e480` | core | const_ret | wrapper around `ReleaseMutex` | none (vtable/callback) |

### ctrl-boot (134)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x18009db00` | fluff | const_ret | returns a constant | `0x18009db10`, `0x18009db70` +1 |
| `0x18009db10` | core | setter | wrapper around `_local_stdio_scanf_options` | none (vtable/callback) |
| `0x18009e100` | core | thunk | forwards to `0x18009e170` | none (vtable/callback) |
| `0x18009e120` | core | thunk | forwards to `0x18009e2a0` | none (vtable/callback) |
| `0x18009e430` | core | thunk | forwards to `0x18009e2a0` | none (vtable/callback) |
| `0x18009f500` | core | thunk | forwards to `0x18009f530` with extra constant arguments | none (vtable/callback) |
| `0x18009f770` | ref | dtor | destructor (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18009fd80` | core | zero_init | constructor/reset: sets about 44 fields to zero or defaults | `0x1800474f0`, `0x18004a9d0` +14 |
| `0x1800a1130` | core | eh_ctor | constructs an array of 6 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x1800a1170` | core | eh_ctor | constructs an array of 6 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x1800a1670` | core | zero_init | constructor/reset: sets about 7 fields to zero or defaults | none (vtable/callback) |
| `0x1800a19d0` | core | dtor | destructor of `RnD::HmdTrackerIntf` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x1800aa100` | core | xordec | decodes an obfuscated log string (XOR with fixed 64-bit keys) | none (vtable/callback) |
| `0x1800aa150` | core | xordec | decodes an obfuscated log string (XOR with fixed 64-bit keys) | none (vtable/callback) |
| `0x1800aa1a0` | core | xordec | decodes an obfuscated log string (XOR with fixed 64-bit keys) | none (vtable/callback) |
| `0x1800aa1f0` | core | xordec | decodes an obfuscated log string (XOR with fixed 64-bit keys) | none (vtable/callback) |
| `0x1800aa2d0` | core | xordec | decodes an obfuscated log string (XOR with fixed 64-bit keys) | none (vtable/callback) |
| `0x1800aa3b0` | core | xordec | decodes an obfuscated log string (XOR with fixed 64-bit keys) | none (vtable/callback) |
| `0x1800aa480` | core | xordec | decodes an obfuscated log string (XOR with fixed 64-bit keys) | none (vtable/callback) |
| `0x1800aa670` | core | xordec | decodes an obfuscated log string (XOR with fixed 64-bit keys) | none (vtable/callback) |
| `0x1800aa900` | core | xordec | decodes an obfuscated log string (XOR with fixed 64-bit keys) | none (vtable/callback) |
| `0x1800aae90` | core | vec_grow | RnD::Vector grow/insert for 4-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ab0d0` | core | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ab240` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800ab270` | core | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ab3e0` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800ac090` | fluff | dtor | destructor (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x1800ac0c0` | core | eh_ctor | destroys an array of 4 × 0x4ce0-byte elements (element function `0x1800d30d0`) | none (vtable/callback) |
| `0x1800ac0f0` | core | eh_ctor | destroys an array of 4 × 0x4ce0-byte elements (element function `0x1800d30d0`) | none (vtable/callback) |
| `0x1800ac2a0` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800ac320` | core | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ac460` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800ac4a0` | core | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ac5e0` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800ac740` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800acb10` | core | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800acc90` | core | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800acde0` | core | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800acf30` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800ad730` | core | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ad840` | core | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ad970` | core | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ae670` | core | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800aedd0` | core | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800b0210` | core | eh_ctor | constructs an array of 4 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x1800b1550` | core | const_ret | wrapper around `fdclass` | none (vtable/callback) |
| `0x1800b2060` | core | dtor | destructor of `RnD::IBackground` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x1800b2d90` | core | setter | reads one lane of a SIMD vector selected by argument 2 | none (vtable/callback) |
| `0x1800b3f00` | core | eh_ctor | destroys an array at +0x2 of 3 × 0x180-byte elements (element function `0x1800b3f50`) | none (vtable/callback) |
| `0x1800b6e80` | core | eh_ctor | constructs an array at +0x2 of 4 × 0xaa0-byte elements (element function `0x1800ba770`) | none (vtable/callback) |
| `0x1800b6f30` | core | eh_ctor | destroys an array at +0x2 of 4 × 0xaa0-byte elements (element function `0x1800b6f70`) | none (vtable/callback) |
| `0x1800b6f70` | core | dtor | destructor (frees itself when the flag argument is set) | `0x180049240`, `0x18004fe10` +3 |
| `0x1800ba770` | core | zero_init | constructor/reset of `RnD::MersenneTwister`: sets about 23 fields to zero or defaults | `0x180049240`, `0x18004fe10` +1 |
| `0x1800bb680` | core | vec_grow | RnD::Vector grow/insert for 4-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800bc490` | core | copy | copies a record field by field (about 30 fields) | `0x1800b9740`, `0x1800baf90` +9 |
| `0x1800bffd0` | core | const_ret | returns a constant | none (vtable/callback) |
| `0x1800c0100` | core | const_ret | bitwise select: bits of argument 2 where the mask (argument 3) is set, else argument 1 | none (vtable/callback) |
| `0x1800c01f0` | core | setter | copies two packed floats with both signs flipped | none (vtable/callback) |
| `0x1800c03b0` | core | thunk | forwards to `0x18009f530` with extra constant arguments | none (vtable/callback) |
| `0x1800c03f0` | core | thunk | forwards to `0x18009f530` with extra constant arguments | none (vtable/callback) |
| `0x1800c3250` | core | eh_ctor | constructs an array of 4 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x1800c3290` | core | eh_ctor | constructs an array of 4 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x1800c32d0` | core | eh_ctor | destroys an array of 4 × 0x40-byte elements (no-op element function) | `0x18023d0f0` |
| `0x1800c32f0` | core | eh_ctor | destroys an array of 4 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x1800c8dd0` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800c8e10` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800c9020` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800c9050` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800c9200` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800c9240` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800c9630` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800c9660` | core | vec_grow | RnD::Vector grow/insert for 64-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800c9b10` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800c9cb0` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800c9d60` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800c9ea0` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800c9ef0` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ca0e0` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800ca1b0` | core | vec_grow | RnD::Vector grow/insert for 64-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ca340` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800ca4c0` | core | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | none (vtable/callback) |
| `0x1800ca660` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ca870` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ca9d0` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800cac40` | core | vec_grow | RnD::Vector grow/insert for 64-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800caf10` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800cb090` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800cb200` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800cb3d0` | core | vec_grow | RnD::Vector grow/insert for 64-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800cba30` | core | eh_ctor | constructs an array of 16 × 0x5f0-byte elements (no-op element function) | none (vtable/callback) |
| `0x1800ce0f0` | ref | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | `0x18023d3a0`, `0x18023d3b0` +4 |
| `0x1800ce260` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ce3c0` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800ce490` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ce610` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ce750` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ce920` | core | eh_ctor | constructs an array of 4 × 0x40-byte elements (no-op element function) | `0x1800cb670` |
| `0x1800d0440` | core | setter | stores `clock()` into its argument | none (vtable/callback) |
| `0x1800d1130` | core | thunk | forwards to `0x180103090` | none (vtable/callback) |
| `0x1800e5fd0` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800e6c70` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | `0x18023d5a0` |
| `0x1800e6cb0` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800e7030` | core | const_ret | fills a 256-byte table with one byte value | none (vtable/callback) |
| `0x1800e77e0` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800e7980` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800e7cb0` | core | vec_grow | RnD::Vector grow/insert for 4-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800e7e90` | core | vec_grow | RnD::Vector grow/insert for 2-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800e87f0` | core | vec_grow | RnD::Vector grow/insert for 2-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800e8990` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800e9190` | core | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800e95e0` | core | vec_grow | RnD::Vector grow/insert for 4-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ec320` | ref | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | `0x18023d760` |
| `0x1800ec630` | ref | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ec7b0` | ref | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ec9d0` | ref | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ecc30` | ref | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ecdd0` | ref | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800ed200` | ref | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ed320` | ref | vec_grow | RnD::Vector grow/insert for 16-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800eed90` | core | vec_grow | RnD::Vector grow/insert for 4-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800eeed0` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | `0x18023d7e0` |
| `0x1800eef00` | core | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ef030` | core | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ef2d0` | core | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800ef400` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800ef4b0` | core | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800fd540` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800fd5a0` | core | setter | wrapper around `_isnan` | none (vtable/callback) |
| `0x1800fd9b0` | core | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x1800ff320` | core | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | none (vtable/callback) |
| `0x1800ff340` | core | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | none (vtable/callback) |
| `0x180109d30` | core | const_ret | returns the constant 1.0 | none (vtable/callback) |
| `0x180109d40` | core | const_ret | returns the constant 0 | none (vtable/callback) |
| `0x180109d60` | core | dtor | destructor of `RnD::Robustifier` (frees itself when the flag argument is set) | none (vtable/callback) |

### ctrl-tracker (64)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x180051380` | core | eh_ctor | destroys an array at +0x700 of 64 × 0x1f0-byte elements (no-op element function) | `0x18023d1c0` |
| `0x1800513d0` | core | eh_ctor | destroys an array of 4 × 0x40-byte elements (no-op element function) | `0x18004ab50`, `0x180050010` +7 |
| `0x1800513f0` | core | eh_ctor | destroys an array of 16 × 0x5f0-byte elements (no-op element function) | `0x18023d2e0` |
| `0x180051430` | core | eh_ctor | destroys an array of 6 × 0x40-byte elements (no-op element function) | `0x18004ab50`, `0x180050010` +9 |
| `0x180051a70` | ref | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x180051ba0` | ref | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x180051c10` | ref | vec_grow | RnD::Vector grow/insert for 4-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x180051d20` | ref | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x180051d60` | ref | vec_grow | RnD::Vector grow/insert for 4-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x180051e70` | ref | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x180051eb0` | ref | vec_grow | RnD::Vector grow/insert for fixed-size elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x180051fa0` | ref | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x180051fe0` | ref | vec_grow | RnD::Vector grow/insert for 2-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800520e0` | ref | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x180052860` | core | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800529d0` | core | vec_grow | RnD::Vector grow/insert for 4-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x180052b20` | core | vec_grow | RnD::Vector grow/insert for 4-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x180052c70` | core | vec_grow | RnD::Vector grow/insert for fixed-size elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x180052d90` | core | vec_grow | RnD::Vector grow/insert for 2-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800530f0` | core | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x180053200` | core | vec_grow | RnD::Vector grow/insert for 4-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x180053300` | core | vec_grow | RnD::Vector grow/insert for 4-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x180053400` | core | vec_grow | RnD::Vector grow/insert for fixed-size elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x1800534e0` | core | vec_grow | RnD::Vector grow/insert for 2-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x180053820` | core | setter | stores `_Xtime_get_ticks()` into its argument | none (vtable/callback) |
| `0x180053940` | core | const_ret | wrapper around `_Mtx_init_in_situ` | none (vtable/callback) |
| `0x1800539b0` | core | const_ret | wrapper around `_Mtx_init_in_situ` | none (vtable/callback) |
| `0x1800539d0` | core | const_ret | wrapper around `_Cnd_init_in_situ` | none (vtable/callback) |
| `0x180053c70` | core | const_ret | calls virtual slot 1 of its object | `0x180049240`, `0x180053b20` +2 |
| `0x180053c90` | fluff | dtor | destructor of `IrFramework::Thread` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180053e20` | fluff | dtor | destructor of `IrFramework::Event` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180054660` | core | thunk | forwards to `0x180054410` on an entry of a global table | none (vtable/callback) |
| `0x180056ac0` | core | const_ret | wrapper around `_Mtx_init_in_situ` | none (vtable/callback) |
| `0x180056b00` | core | const_ret | wrapper around `_Mtx_unlock` | none (vtable/callback) |
| `0x180059770` | core | thunk | forwards to `0x180059790` | none (vtable/callback) |
| `0x180059880` | core | thunk | forwards to `0x1800598a0` | none (vtable/callback) |
| `0x180059c20` | core | zero_init | constructor/reset: sets about 27 fields to zero or defaults | none (vtable/callback) |
| `0x18005b740` | core | dtor | destructor of `OIFusion::OpticalMarkerData` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18005b860` | core | zero_init | constructor/reset: sets about 88 fields to zero or defaults | `0x18005d440` |
| `0x18005bb00` | core | zero_init | constructor/reset: sets about 119 fields to zero or defaults | `0x18005b860` |
| `0x18005bf30` | core | zero_init | constructor/reset: sets about 39 fields to zero or defaults | `0x18005b860`, `0x18005bb00` +1 |
| `0x18005d360` | core | eh_ctor | constructs an array at +0x5 of 2 × 0x2218-byte elements (element function `0x18005d440`) | none (vtable/callback) |
| `0x18005d3d0` | core | dtor | destructor of `UkfManager` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18005d580` | core | eh_ctor | destroys an array at +0x5 of 2 × 0x2218-byte elements (element function `0x18005d4b0`) | none (vtable/callback) |
| `0x18005f180` | core | dtor | destructor of `OIFusion::OpticalMarkerData` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180060000` | core | const_ret | returns the constant 0 | none (vtable/callback) |
| `0x180060010` | core | const_ret | returns the constant 0 | none (vtable/callback) |
| `0x180060020` | core | const_ret | returns the constant 1 | none (vtable/callback) |
| `0x180060280` | core | getter | reads the 8-byte field at +0x88 of its object | none (vtable/callback) |
| `0x180060290` | core | getter | reads the byte field at +0x30 of its object | none (vtable/callback) |
| `0x1800607f0` | core | eh_ctor | destroys an array at +0x2 of 4 × 0xb0-byte elements (element function `0x180061570`) | none (vtable/callback) |
| `0x180060820` | core | eh_ctor | constructs an array at +0x2 of 4 × 0xb0-byte elements (element function `0x180061900`) | none (vtable/callback) |
| `0x180060870` | core | eh_ctor | destroys an array at +0x2 of 4 × 0xb0-byte elements (element function `0x180061570`) | none (vtable/callback) |
| `0x180060af0` | core | thunk | forwards to `0x180060a40` | none (vtable/callback) |
| `0x1800611e0` | core | eh_ctor | destroys an array at +0x2 of 4 × 0xb0-byte elements (element function `0x180061570`) | none (vtable/callback) |
| `0x180061490` | core | eh_ctor | constructs an array at +0x2 of 4 × 0xb0-byte elements (element function `0x180061900`) | none (vtable/callback) |
| `0x180061ab0` | core | getter | identity: returns argument 1 | none (vtable/callback) |
| `0x180063480` | core | thunk | forwards to `0x180063310` with extra constant arguments | none (vtable/callback) |
| `0x180069950` | core | xordec | decodes an obfuscated log string (XOR with fixed 64-bit keys) | none (vtable/callback) |
| `0x18006ae80` | core | setter | reads one lane of a SIMD vector selected by argument 2 | none (vtable/callback) |
| `0x18006aec0` | core | setter | reads one lane of a SIMD vector selected by argument 2 | none (vtable/callback) |
| `0x18007fa20` | core | xordec | decodes an obfuscated log string (XOR with fixed 64-bit keys) | none (vtable/callback) |
| `0x1800800f0` | core | xordec | decodes an obfuscated log string (XOR with fixed 64-bit keys) | none (vtable/callback) |
| `0x180097100` | core | thunk | forwards to `0x180096d90` with extra constant arguments | `0x180097990` |

### debug:trace (69)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x18010a760` | fluff | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | `0x18010b540`, `0x18010c390` |
| `0x18010af10` | fluff | eh_ctor | destroys an array at +0xab8 of 3 × 0x34-byte elements (no-op element function) | `0x18023dc40` |
| `0x18010c350` | fluff | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | none (vtable/callback) |
| `0x18010c370` | fluff | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | none (vtable/callback) |
| `0x18010c5f0` | fluff | thunk | forwards to `0x18010c610` | none (vtable/callback) |
| `0x18010c600` | fluff | thunk | forwards to `0x18010c610` | none (vtable/callback) |
| `0x18010cd60` | fluff | const_ret | returns a constant string "basic_Info.bin" | none (vtable/callback) |
| `0x18010cd70` | fluff | const_ret | returns a constant string "vrt_camera_selfcalib_vrtrace.bin" | none (vtable/callback) |
| `0x18010cd80` | fluff | getter | returns the address of the sub-object at +0x50 | none (vtable/callback) |
| `0x18010cd90` | fluff | const_ret | returns a constant string "controller_tracking.bin" | none (vtable/callback) |
| `0x18010cda0` | fluff | const_ret | returns a constant string "packed_image.bin" | none (vtable/callback) |
| `0x18010cdb0` | fluff | const_ret | returns a constant string "imu.bin" | none (vtable/callback) |
| `0x18010cdc0` | fluff | const_ret | returns a constant string "ins.bin" | none (vtable/callback) |
| `0x18010cde0` | fluff | const_ret | returns a constant string "reloc_context.bin" | none (vtable/callback) |
| `0x18010cdf0` | fluff | const_ret | returns a constant string "reloc_snapshot.bin" | none (vtable/callback) |
| `0x18010ce00` | fluff | const_ret | returns a constant string "selfcalibration.bin" | none (vtable/callback) |
| `0x18010ce10` | fluff | const_ret | returns a constant string "db_000.dat" | none (vtable/callback) |
| `0x18010d430` | fluff | const_ret | returns the constant 1 | none (vtable/callback) |
| `0x18010d4d0` | fluff | setter | clears the 4-byte field at +0x40 and returns true | none (vtable/callback) |
| `0x18010e370` | fluff | thunk | forwards to `0x18016b850` | `0x18023dc50`, `0x18023dc60` +11 |
| `0x18010e3e0` | fluff | dtor | destructor (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18010e430` | fluff | dtor | destructor of `VrTraceCore` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18010e7b0` | fluff | thunk | wrapper around `nullsub_7` | none (vtable/callback) |
| `0x18010f1f0` | fluff | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | `0x1801123f0`, `0x1801135a0` +11 |
| `0x18010f200` | fluff | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | `0x1801135a0` |
| `0x18010f220` | fluff | getter | reads the byte field at +0xa of its object | `0x1801149e0` |
| `0x18010f230` | fluff | getter | reads the byte field at +0x9 of its object | `0x1801123f0`, `0x180112dd0` +10 |
| `0x18010f240` | fluff | getter | reads the byte field at +0x8 of its object | `0x1801123f0`, `0x1801137a0` +9 |
| `0x18010f270` | fluff | thunk | forwards to `0x18010ec10` | none (vtable/callback) |
| `0x1801100e0` | fluff | zero_init | constructor/reset of `VrTraceDumper`: sets about 10 fields to zero or defaults | `0x18010abb0`, `0x18010abf0` +10 |
| `0x180110120` | fluff | zero_init | constructor/reset: sets about 4 fields to zero or defaults | `0x18010e2d0` |
| `0x1801103c0` | fluff | const_ret | returns the constant 0 | none (vtable/callback) |
| `0x1801103d0` | fluff | const_ret | returns the constant 0 | none (vtable/callback) |
| `0x1801107e0` | fluff | const_ret | returns the constant 1 | none (vtable/callback) |
| `0x1801107f0` | fluff | const_ret | returns the constant 0 | none (vtable/callback) |
| `0x180110800` | fluff | const_ret | returns the constant 1 | none (vtable/callback) |
| `0x180111f50` | fluff | dtor | destructor of `VrTraceManager` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x1801127f0` | fluff | const_ret | returns the constant 0 | none (vtable/callback) |
| `0x180112db0` | fluff | thunk | forwards to `0x18010e7d0` | `0x180116770` |
| `0x180113f10` | fluff | zero_init | constructor/reset: sets about 27 fields to zero or defaults | `0x1801129c0`, `0x180112d70` |
| `0x180113fe0` | fluff | const_ret | returns the constant 1 | none (vtable/callback) |
| `0x180114000` | fluff | thunk | forwards to `0x180112880` | none (vtable/callback) |
| `0x180114010` | fluff | const_ret | returns the constant 1 | none (vtable/callback) |
| `0x180114020` | fluff | const_ret | returns the constant 0 | none (vtable/callback) |
| `0x1801150e0` | fluff | thunk | forwards to `0x18010f460` | `0x180116230` |
| `0x1801150f0` | fluff | thunk | forwards to `0x18010f510` | `0x1801167b0` |
| `0x180115170` | fluff | const_ret | wrapper around `memset` | `0x180111400` |
| `0x180115820` | fluff | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | none (vtable/callback) |
| `0x180115900` | fluff | dtor | destructor (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180115950` | fluff | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x1801159e0` | fluff | getter | returns the address of the sub-object at +0x8 | none (vtable/callback) |
| `0x1801159f0` | fluff | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x180115a00` | fluff | const_ret | std::function plumbing: returns the lambda's RTTI type descriptor | none (vtable/callback) |
| `0x180116760` | fluff | const_ret | returns the constant 1 | `0x18010eae0` |
| `0x180116a50` | fluff | const_ret | returns the constant 0 | `0x18010e860`, `0x18010ec10` |
| `0x180116a80` | fluff | const_ret | returns the constant 0 | `0x18010b540`, `0x18010c390` +1 |
| `0x180116a90` | fluff | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | `0x18010e4b0` |
| `0x180117390` | fluff | const_ret | zeroes the byte range [first, last) | none (vtable/callback) |
| `0x1801173b0` | fluff | const_ret | zeroes the byte range [first, last) | none (vtable/callback) |
| `0x180118148` | fluff | thunk | forwards to `0x1801181a0` | none (vtable/callback) |
| `0x1801181e0` | fluff | dtor | destructor of `AstonContext` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180118230` | fluff | dtor | destructor of `AstonManager` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180118260` | fluff | dtor | destructor of `AstonContext::ScopedLock` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x1801182b0` | fluff | dtor | destructor of `std::exception` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180118580` | core | thunk | forwards to `0x180119400` | none (vtable/callback) |
| `0x1801188f0` | core | const_ret | reads a field of the entry selected by argument 2 in a pointer table | `0x18018aaf0` |
| `0x180118910` | core | const_ret | reads a field of the entry selected by argument 2 in a pointer table | `0x180189b70`, `0x18018aaf0` |
| `0x1801193b0` | core | const_ret | reads a field of the entry selected by argument 2 in a pointer table | `0x180185ef0`, `0x18018aad0` +5 |
| `0x1801193c0` | core | const_ret | reads a field of the entry selected by argument 2 in a pointer table | `0x18018aaf0` |

### import/thunk (106)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x1800970f0` | fluff | thunk | forwards to `0x180096d90` with extra constant arguments | `0x180097990` |
| `0x18013b4f0` | fluff | dtor | destructor of `Json::CharReaderBuilder` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18013b780` | fluff | dtor | destructor of `Json::StreamWriterBuilder` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180185ae0` | fluff | const_ret | wrapper around `malloc` | `0x180049240`, `0x18004a340` +12 |
| `0x1801aa7a0` | fluff | const_ret | wrapper around `malloc_0` | `0x1801debe0`, `0x1801ec600` +37 |
| `0x1801aa7d0` | fluff | const_ret | wrapper around `read` | `0x1802312c0`, `0x180232c30` +3 |
| `0x1801aa7e0` | fluff | const_ret | wrapper around `write` | `0x18022dff0`, `0x18022ee60` |
| `0x1801addbe` | fluff | const_ret | wrapper around `CreateToolhelp32Snapshot` | `0x180131690`, `0x180131c90` +1 |
| `0x1801addc4` | fluff | const_ret | wrapper around `Process32First` | `0x180131690`, `0x180131c90` +1 |
| `0x1801addca` | fluff | const_ret | wrapper around `Process32Next` | `0x180131690`, `0x180131c90` +1 |
| `0x1801ade5a` | fluff | const_ret | wrapper around `K32GetModuleBaseNameA` | `0x1801a2cb0` |
| `0x1801ba157` | fluff | const_ret | wrapper around `WinUsb_Initialize` | `0x180122ac0` |
| `0x1801ba15d` | fluff | const_ret | wrapper around `WinUsb_Free` | `0x180122a70`, `0x180127b70` |
| `0x1801ba169` | fluff | const_ret | wrapper around `WinUsb_SetCurrentAlternateSetting` | `0x1801256c0` |
| `0x1801ba16f` | fluff | const_ret | wrapper around `WinUsb_GetCurrentAlternateSetting` | `0x1801256c0` |
| `0x1801ba187` | fluff | const_ret | wrapper around `WinUsb_AbortPipe` | `0x180122120`, `0x1801221a0` +19 |
| `0x180238f7e` | fluff | const_ret | wrapper around `Xtime_get_ticks` | `0x180115400`, `0x180115490` +5 |
| `0x180238f84` | fluff | const_ret | wrapper around `Query_perf_counter` | `0x180115350`, `0x180115490` +3 |
| `0x180238f8a` | fluff | const_ret | wrapper around `Query_perf_frequency` | `0x180115350`, `0x180115490` +3 |
| `0x1802390ea` | fluff | const_ret | wrapper around `_Getgloballocale` | `0x1801175e0`, `0x18012cf10` |
| `0x18023913e` | fluff | const_ret | wrapper around `xsgetn` | none (vtable/callback) |
| `0x180239144` | fluff | const_ret | wrapper around `xsputn` | none (vtable/callback) |
| `0x180239174` | fluff | const_ret | wrapper around `_Fiopen` | `0x1801176b0`, `0x18015c6a0` |
| `0x18023917a` | fluff | const_ret | wrapper around `showmanyc` | none (vtable/callback) |
| `0x18023918c` | fluff | const_ret | wrapper around `Mtx_lock` | `0x18011e510`, `0x18011e650` +43 |
| `0x180239192` | fluff | const_ret | wrapper around `Mtx_unlock` | `0x18011e510`, `0x18011e650` +44 |
| `0x1802391aa` | fluff | const_ret | wrapper around `uncaught_exception` | `0x18012c0f0`, `0x18012cbe0` +2 |
| `0x1802391b6` | fluff | const_ret | wrapper around `Thrd_detach` | `0x180131d90`, `0x180132340` +2 |
| `0x180239216` | fluff | const_ret | wrapper around `setbuf` | none (vtable/callback) |
| `0x18023921c` | fluff | const_ret | wrapper around `sync` | none (vtable/callback) |
| `0x180239222` | fluff | const_ret | wrapper around `uflow` | none (vtable/callback) |
| `0x18023926a` | fluff | const_ret | wrapper around `setw` | `0x18014ee00`, `0x18014f750` |
| `0x180239270` | fluff | const_ret | wrapper around `Thrd_join` | `0x180159770`, `0x18016b540` |
| `0x180239276` | fluff | const_ret | wrapper around `Thrd_id` | `0x180159770`, `0x18016b540` |
| `0x18023929a` | fluff | const_ret | wrapper around `Cnd_wait` | `0x18017c170`, `0x18017c290` +3 |
| `0x1802392a0` | fluff | const_ret | wrapper around `Cnd_broadcast` | `0x18017bb80`, `0x18017c170` +6 |
| `0x1802392c4` | fluff | thunk | forwards to `0x18023a23c` | none (vtable/callback) |
| `0x18023a864` | fluff | const_ret | wrapper around `LocalFree` | `0x180042c80`, `0x180042d40` |
| `0x18023ad98` | fluff | const_ret | wrapper around `_GSHandlerCheckCommon` | none (vtable/callback) |
| `0x18023b3d8` | fluff | const_ret | registers an exit handler (CRT onexit) | `0x180001000`, `0x180001070` +134 |
| `0x18023b3f0` | fluff | const_ret | wrapper around `new` | `0x18011e360`, `0x1801350e0` +8 |
| `0x18023c140` | fluff | const_ret | returns the address of global `0x180351740` | `0x18023b050`, `0x18023b08c` +4 |
| `0x18023c340` | fluff | const_ret | wrapper around `WinVerifyTrust` | `0x1801350e0` |
| `0x18023c34c` | fluff | const_ret | wrapper around `WTHelperGetProvCertFromChain` | `0x1801350e0` |
| `0x18023c352` | fluff | const_ret | wrapper around `WTHelperProvDataFromStateData` | `0x1801350e0` |
| `0x18023c3b8` | fluff | const_ret | wrapper around `BCryptCloseAlgorithmProvider` | `0x180246420` |
| `0x18023c3c4` | fluff | const_ret | wrapper around `BCryptHashData` | `0x180232e50` |
| `0x18023c3ca` | fluff | const_ret | wrapper around `BCryptFinishHash` | `0x180232e50` |
| `0x18023c3d0` | fluff | const_ret | wrapper around `BCryptDestroyHash` | `0x180232e50` |
| `0x18023c3dc` | fluff | const_ret | wrapper around `_CxxFrameHandler4` | `0x18023ae14` |
| `0x18023c3e2` | fluff | const_ret | wrapper around `memcpy` | `0x180041e50`, `0x180042fc0` +220 |
| `0x18023c3e8` | fluff | const_ret | wrapper around `memset` | `0x180001120`, `0x180007660` +264 |
| `0x18023c3ee` | fluff | const_ret | wrapper around `purecall` | none (vtable/callback) |
| `0x18023c3f4` | fluff | const_ret | wrapper around `memmove` | `0x18004e8f0`, `0x18004eb60` +64 |
| `0x18023c3fa` | fluff | const_ret | wrapper around `memcmp` | `0x180115820`, `0x180115e60` +42 |
| `0x18023c400` | fluff | const_ret | wrapper around `_std_exception_copy` | `0x180117ab0`, `0x180138320` +2 |
| `0x18023c406` | fluff | const_ret | wrapper around `_std_exception_destroy` | `0x180118060`, `0x1801182b0` +1 |
| `0x18023c412` | fluff | const_ret | wrapper around `strstr` | `0x18009dbd0`, `0x1800a6940` +3 |
| `0x18023c418` | fluff | const_ret | wrapper around `memchr` | `0x18012e8b0` |
| `0x18023c41e` | fluff | const_ret | wrapper around `strchr` | `0x1801457d0` |
| `0x18023c424` | fluff | const_ret | wrapper around `wcsstr` | `0x1801a17a0` |
| `0x18023c430` | fluff | const_ret | wrapper around `_current_exception` | `0x18023acf4`, `0x18023bef0` +1 |
| `0x18023c436` | fluff | const_ret | wrapper around `_current_exception_context` | `0x18023acf4`, `0x18023bef0` +1 |
| `0x18023c43c` | fluff | const_ret | wrapper around `_std_type_info_destroy_list` | `0x18023c298` |
| `0x18023c4f6` | fluff | const_ret | wrapper around `ceilf` | `0x18011d580` |
| `0x18023c4fc` | fluff | const_ret | wrapper around `sqrt` | `0x1800013a0`, `0x1800602d0` +43 |
| `0x18023c526` | fluff | const_ret | wrapper around `acosf` | `0x180057040`, `0x1800602d0` +30 |
| `0x18023c52c` | fluff | const_ret | wrapper around `cosf` | `0x1800422d0`, `0x180045420` +52 |
| `0x18023c532` | fluff | const_ret | wrapper around `sinf` | `0x1800422e0`, `0x180045420` +53 |
| `0x18023c538` | fluff | const_ret | wrapper around `sqrtf` | `0x180045c40`, `0x180045cb0` +155 |
| `0x18023c592` | fluff | const_ret | wrapper around `malloc_1` | `0x18023af68` |
| `0x18023c598` | fluff | const_ret | wrapper around `dclass` | `0x180148b40`, `0x1801499a0` +4 |
| `0x18023c5a4` | fluff | const_ret | wrapper around `tan` | `0x180166200`, `0x1801664d0` +4 |
| `0x18023c5b0` | fluff | const_ret | wrapper around `acos` | `0x180168a00`, `0x180183db0` |
| `0x18023c5b6` | fluff | const_ret | wrapper around `cos` | `0x1800795a0`, `0x1800d89f0` +8 |
| `0x18023c5bc` | fluff | const_ret | wrapper around `pow` | `0x1800c8b50`, `0x1800f9a80` +3 |
| `0x18023c5c2` | fluff | const_ret | wrapper around `sin` | `0x1800795a0`, `0x180166dd0` +2 |
| `0x18023c5ce` | fluff | const_ret | wrapper around `atan2f` | `0x1800627f0`, `0x180068ee0` +17 |
| `0x18023c5d4` | fluff | const_ret | wrapper around `fdclass` | `0x1800b1550`, `0x1800b43d0` +7 |
| `0x18023c5da` | fluff | const_ret | wrapper around `asinf` | `0x180171400` |
| `0x18023c5e0` | fluff | const_ret | wrapper around `expf` | `0x180171400` |
| `0x18023c5e6` | fluff | const_ret | wrapper around `round` | `0x180174410` |
| `0x18023c604` | fluff | const_ret | wrapper around `atanf` | `0x1800cc570`, `0x180184900` |
| `0x18023c610` | fluff | const_ret | wrapper around `floorf` | `0x18019e860`, `0x1801a10c0` |
| `0x18023c646` | fluff | const_ret | wrapper around `tanf` | `0x1800d0120`, `0x1800d8150` +1 |
| `0x18023c664` | fluff | const_ret | wrapper around `fabs` | `0x1801d68b0`, `0x1801d7140` +20 |
| `0x18023c69a` | fluff | const_ret | wrapper around `callnewh` | `0x18023af68` |
| `0x18023c6a0` | fluff | const_ret | wrapper around `seh_filter_dll` | `0x18023b118` |
| `0x18023c6a6` | fluff | const_ret | wrapper around `configure_narrow_argv` | `0x18023afac`, `0x18023b08c` |
| `0x18023c6ac` | fluff | const_ret | wrapper around `initialize_narrow_environment` | `0x18023b00c`, `0x18023b08c` |
| `0x18023c6b2` | fluff | const_ret | wrapper around `initialize_onexit_table` | `0x18023b208` |
| `0x18023c6b8` | fluff | const_ret | wrapper around `register_onexit_function` | `0x18023b37c`, `0x18023b3b8` |
| `0x18023c6be` | fluff | const_ret | wrapper around `execute_onexit_table` | `0x18023b178` |
| `0x18023c6c4` | fluff | const_ret | wrapper around `crt_atexit` | `0x18023b37c` |
| `0x18023c6dc` | fluff | const_ret | wrapper around `initterm_e` | `0x18023b804` |
| `0x18023c87a` | fluff | const_ret | wrapper around `HidP_GetCaps` | `0x1801ca7e0`, `0x1801cb4c0` +8 |
| `0x18023c886` | fluff | const_ret | wrapper around `HidD_GetAttributes` | `0x1801cfe40` |
| `0x18023c892` | fluff | const_ret | wrapper around `HidD_GetPreparsedData` | `0x1801ca7e0`, `0x1801cb4c0` +8 |
| `0x18023c898` | fluff | const_ret | wrapper around `HidD_FreePreparsedData` | `0x1801ca7e0`, `0x1801cb4c0` +8 |
| `0x18023c89e` | fluff | const_ret | wrapper around `HidD_GetFeature` | `0x1801caf90`, `0x1801cb4c0` +4 |
| `0x18023c8a4` | fluff | const_ret | wrapper around `HidD_SetFeature` | `0x1801ca7e0`, `0x1801cb7d0` +1 |
| `0x18023c8aa` | fluff | const_ret | wrapper around `HidD_GetManufacturerString` | `0x1801cfe40` |
| `0x18023c8b0` | fluff | const_ret | wrapper around `HidD_GetProductString` | `0x1801cfe40` |
| `0x18023c8b6` | fluff | const_ret | wrapper around `HidD_GetSerialNumberString` | `0x1801cfe40` |
| `0x18023c8eb` | fluff | const_ret | wrapper around `log` | `0x180084f60`, `0x180087ff0` +1 |
| `0x18023c910` | fluff | const_ret | returns a constant | `0x180238fbc`, `0x180238ff4` +16 |

### init (11)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x1800010e0` | fluff | eh_ctor | constructs an array of 4 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x180001360` | fluff | eh_ctor | constructs an array of 4 × 0x198-byte elements (element function `0x180053e70`) | none (vtable/callback) |
| `0x180002a20` | fluff | thunk | wrapper around `atexit` | none (vtable/callback) |
| `0x180002c90` | fluff | thunk | wrapper around `atexit` | none (vtable/callback) |
| `0x180007760` | fluff | const_ret | wrapper around `Mtx_init_in_situ` | none (vtable/callback) |
| `0x180007980` | fluff | eh_ctor | constructs an array of 2 × 0x170-byte elements (element function `0x180185690`) | none (vtable/callback) |
| `0x1800079c0` | fluff | thunk | wrapper around `atexit` | none (vtable/callback) |
| `0x1800079f0` | fluff | eh_ctor | constructs an array of 2 × 0x20-byte elements (element function `0x18004e070`) | none (vtable/callback) |
| `0x180007f80` | fluff | thunk | wrapper around `atexit` | none (vtable/callback) |
| `0x180007fb0` | fluff | thunk | wrapper around `atexit` | none (vtable/callback) |
| `0x180007fe0` | fluff | thunk | wrapper around `atexit` | none (vtable/callback) |

### ins+irtracker (34)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x180173810` | core | thunk | forwards to `0x18016b850` | `0x180242f50` |
| `0x180173eb0` | core | const_ret | returns the address of global `0x180354fe0` | `0x180175760`, `0x1801806a0` |
| `0x180173f00` | core | setter | stores argument 1 into global `0x180354fe0` | `0x180123fa0` |
| `0x180173f10` | core | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | `0x180179980` |
| `0x180173f40` | core | const_ret | returns the address of global `0x18034e12c` | `0x18017b030`, `0x18017b100` +2 |
| `0x180173fa0` | core | thunk | forwards to `0x180173830` | `0x1801780e0`, `0x18017cdb0` |
| `0x180173fd0` | ref | const_ret | returns the constant 1 | `0x180177930`, `0x18017a2f0` |
| `0x1801743b0` | core | thunk | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | `0x18017b1a0` |
| `0x1801743d0` | core | thunk | forwards to `0x1801748a0` | `0x18017cdb0`, `0x18017eae0` |
| `0x180174400` | core | thunk | forwards to `0x18016b850` | `0x18017b1a0` |
| `0x180175740` | core | const_ret | returns the constant 0 | none (vtable/callback) |
| `0x180175750` | core | thunk | forwards to `0x180173ec0` | none (vtable/callback) |
| `0x180175760` | core | thunk | forwards to `0x180173eb0` | none (vtable/callback) |
| `0x180177760` | core | eh_ctor | destroys an array at +0x8 of 64 × 0x44-byte elements (no-op element function) | `0x180243060` |
| `0x1801778d0` | core | dtor | destructor of `InsManager` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x1801780a0` | core | thunk | forwards to `0x180177d70` | none (vtable/callback) |
| `0x180179c10` | core | thunk | forwards to `0x180179980` | `0x180128030`, `0x18018aaf0` |
| `0x180179c40` | core | const_ret | calls virtual slot 1 of the member object at +0x18 | `0x18018aaf0` |
| `0x18017a500` | core | const_ret | returns a constant | `0x18012b860`, `0x1801ac4e0` |
| `0x18017a590` | ref | const_ret | returns a constant | `0x1801ac4e0`, `0x1801adc80` |
| `0x18017a5a0` | fluff | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | `0x180134800` |
| `0x18017b460` | core | copy | copies a record field by field (about 18 fields) | `0x18017b5a0` |
| `0x18017f200` | fluff | dtor | destructor (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18017f230` | core | dtor | destructor (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180180110` | core | setter | wrapper around `isnan` | `0x180180200` |
| `0x180180ee0` | core | zero_init | constructor/reset: sets about 70 fields to zero or defaults | none (vtable/callback) |
| `0x180181380` | core | const_ret | returns a constant | `0x1801780e0` |
| `0x180181540` | core | setter | wrapper around `isnan` | `0x180181dd0` |
| `0x1801815e0` | core | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | `0x180181700` |
| `0x180181630` | core | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | `0x180181dd0` |
| `0x180185790` | core | dtor | destructor of `IRTrackerThread` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180185920` | core | dtor | destructor of `IRTrackerThread` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180185a00` | fluff | dtor | destructor of `IrCaptureMutex` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180189c30` | core | const_ret | returns the address of global `0x18035ba1d` | `0x1801999f0` |

### ledsync (5)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x180193d90` | core | dtor | destructor of `LedSyncer` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180196810` | core | const_ret | reads global `0x180350d98` | `0x180190300` |
| `0x180196fd0` | core | setter | two-statement leaf (one store or load, then return) | `0x180190300` |
| `0x180199940` | core | zero_init | constructor/reset: sets about 11 fields to zero or defaults | `0x1801910f0` |
| `0x18019d160` | core | dtor | destructor of `UpdateThread` (frees itself when the flag argument is set) | none (vtable/callback) |

### lib:crt (79)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x180041cc0` | fluff | const_ret | returns a constant | `0x180041cd0`, `0x180041d20` +12 |
| `0x180041cd0` | fluff | setter | wrapper around `_local_stdio_printf_options` | none (vtable/callback) |
| `0x180041e50` | fluff | const_ret | wrapper around `memcpy` | none (vtable/callback) |
| `0x180042020` | fluff | dtor | destructor of `std::exception` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180042090` | fluff | dtor | destructor of `std::exception` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180042130` | fluff | dtor | destructor of `std::exception` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x1800423c0` | fluff | dtor | destructor of `std::exception` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x1800424e0` | fluff | const_ret | returns argument 2 unchanged | none (vtable/callback) |
| `0x1800425c0` | fluff | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | none (vtable/callback) |
| `0x180042980` | fluff | dtor | destructor of `std::exception` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x1800429f0` | fluff | dtor | destructor of `std::system_error` (frees itself when the flag argument is set) | `0x180042b50`, `0x180043050` |
| `0x180042ae0` | fluff | dtor | destructor of `std::exception` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180042c90` | fluff | const_ret | returns a constant string "generic" | none (vtable/callback) |
| `0x180042d30` | fluff | const_ret | returns a constant string "system" | none (vtable/callback) |
| `0x180043770` | fluff | dtor | destructor of `std::exception` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18004e270` | fluff | getter | identity: returns argument 1 | `0x18023cc80` |
| `0x18004edc0` | fluff | const_ret | wrapper around `memcpy` | none (vtable/callback) |
| `0x18004edf0` | fluff | const_ret | wrapper around `memcpy` | none (vtable/callback) |
| `0x18004f160` | fluff | const_ret | wrapper around `memset` | none (vtable/callback) |
| `0x18004f190` | fluff | const_ret | wrapper around `memmove` | none (vtable/callback) |
| `0x18004f620` | fluff | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | `0x18023d030`, `0x18023d040` +8 |
| `0x18004f650` | fluff | vec_grow | RnD::Vector grow/insert for 8-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x18004f7f0` | fluff | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | `0x18023d480`, `0x18023d6c0` +1 |
| `0x18004f820` | fluff | vec_grow | RnD::Vector grow/insert for 4-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x18004f950` | fluff | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | `0x18023d060`, `0x18023d0e0` +6 |
| `0x18004f980` | fluff | vec_grow | RnD::Vector grow/insert for 4-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x18004faa0` | fluff | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x18004fad0` | fluff | vec_grow | RnD::Vector grow/insert for fixed-size elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x18004fbd0` | fluff | vec_free | RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity | none (vtable/callback) |
| `0x18004fc00` | fluff | vec_grow | RnD::Vector grow/insert for 2-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
| `0x18004fe10` | fluff | eh_ctor | constructs an array at +0x2 of 4 × 0xaa0-byte elements (element function `0x1800ba770`) | none (vtable/callback) |
| `0x18023af3c` | fluff | dtor | destructor of `type_info` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18023afac` | fluff | setter | wrapper around `get_startup_argv_mode` | none (vtable/callback) |
| `0x18023b0c0` | fluff | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | `0x18023b804` |
| `0x18023b1a8` | fluff | const_ret | wrapper around `_acrt_uninitialize_critical` | `0x18023b91c`, `0x1802459e2` |
| `0x18023c138` | fluff | const_ret | returns a constant | `0x18023afac`, `0x18023b08c` |
| `0x18023c298` | fluff | const_ret | wrapper around `_std_type_info_destroy_list` | `0x18023b91c` |
| `0x18023c2c0` | fluff | const_ret | returns a constant | `0x18023b804` |
| `0x18023c8d4` | fluff | const_ret | wrapper around `_acrt_initialize` | `0x18023b1bc` |
| `0x18023c8d8` | fluff | const_ret | wrapper around `_acrt_thread_attach` | `0x18023b0d8` |
| `0x18023c8dc` | fluff | const_ret | wrapper around `_vcrt_thread_detach` | `0x18023b0d8`, `0x18023b100` |
| `0x18023c8e0` | fluff | const_ret | wrapper around `_vcrt_uninitialize` | `0x18023b1bc`, `0x18023b350` |
| `0x18023c8e4` | fluff | const_ret | wrapper around `_acrt_uninitialize_critical` | `0x18023b1a8` |
| `0x18023c8e8` | fluff | const_ret | returns the constant 0 | `0x18023b178` |
| `0x18023cb10` | fluff | eh_ctor | destroys an array at +0x1f0 of 4 × 0x60-byte elements (no-op element function) | none (vtable/callback) |
| `0x18023cd80` | fluff | eh_ctor | destroys an array at +0x48 of 2 × 0x2218-byte elements (element function `0x18005d4b0`) | none (vtable/callback) |
| `0x18023cdb0` | fluff | eh_ctor | destroys an array at +0x40 of 2 × 0x2218-byte elements (element function `0x18005d4b0`) | none (vtable/callback) |
| `0x18023cde0` | fluff | eh_ctor | destroys an array at +0x40 of 2 × 0x2218-byte elements (element function `0x18005d4b0`) | none (vtable/callback) |
| `0x18023ce60` | fluff | eh_ctor | destroys an array at +0x40 of 2 × 0x2218-byte elements (element function `0x18005d4b0`) | none (vtable/callback) |
| `0x18023ce90` | fluff | eh_ctor | destroys an array at +0x60 of 16 × 0x1f0-byte elements (element function `0x180051430`) | none (vtable/callback) |
| `0x18023ced0` | fluff | eh_ctor | destroys an array at +0x60 of 4 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x18023cf10` | fluff | eh_ctor | destroys an array at +0x60 of 16 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x18023cf70` | fluff | eh_ctor | destroys an array at +0x710 of 4 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x18023cfc0` | fluff | eh_ctor | destroys an array at +0x210 of 4 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x18023cff0` | fluff | eh_ctor | destroys an array at +0x3b0 of 4 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x18023d070` | fluff | eh_ctor | destroys an array at +0x40 of 2 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x18023d130` | fluff | eh_ctor | destroys an array at +0x4b0 of 4 × 0x20-byte elements (element function `0x1800ab540`) | none (vtable/callback) |
| `0x18023d190` | fluff | eh_ctor | destroys an array at +0x70 of 4 × 0x168-byte elements (element function `0x180051190`) | none (vtable/callback) |
| `0x18023d1e0` | fluff | eh_ctor | destroys an array at +0x70 of 4 × 0x110-byte elements (element function `0x1800513d0`) | none (vtable/callback) |
| `0x18023d220` | fluff | eh_ctor | destroys an array at +0x70 of 4 × 0x110-byte elements (element function `0x1800513d0`) | none (vtable/callback) |
| `0x18023d260` | fluff | eh_ctor | destroys an array at +0x70 of 4 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x18023d2a0` | fluff | eh_ctor | destroys an array at +0x70 of 16 × 0x110-byte elements (element function `0x1800513d0`) | none (vtable/callback) |
| `0x18023d300` | fluff | eh_ctor | destroys an array at +0x78 of 16 × 0x70-byte elements (no-op element function) | none (vtable/callback) |
| `0x18023d330` | fluff | eh_ctor | destroys an array at +0x40 of 16 × 0x70-byte elements (no-op element function) | none (vtable/callback) |
| `0x18023d3c0` | fluff | eh_ctor | destroys an array at +0x100 of 3 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x18023d3f0` | fluff | eh_ctor | destroys an array at +0x60 of 2 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x18023d5f0` | fluff | eh_ctor | destroys an array at +0x180 of 3 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x18023d620` | fluff | eh_ctor | destroys an array at +0xe70 of 4 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x18023d660` | fluff | eh_ctor | destroys an array at +0x1b00 of 4 × 0xc0-byte elements (element function `0x1800d2fd0`) | none (vtable/callback) |
| `0x18023dbc0` | fluff | eh_ctor | destroys an array at +0x15f8 of 3 × 0x34-byte elements (no-op element function) | none (vtable/callback) |
| `0x18023dc00` | fluff | eh_ctor | destroys an array at +0xc58 of 3 × 0x34-byte elements (no-op element function) | none (vtable/callback) |
| `0x18023df30` | fluff | eh_ctor | destroys an array at +0x70 of 2 × 0x50-byte elements (element function `0x180111a30`) | none (vtable/callback) |
| `0x18023e220` | fluff | const_ret | sets the fail state of a std::ios stream | none (vtable/callback) |
| `0x18023ed00` | fluff | const_ret | sets the fail state of a std::ios stream | none (vtable/callback) |
| `0x18023ed60` | fluff | const_ret | sets the fail state of a std::ios stream | none (vtable/callback) |
| `0x18023edc0` | fluff | const_ret | sets the fail state of a std::ios stream | none (vtable/callback) |
| `0x18023f0c0` | fluff | eh_ctor | destroys an array at +0x30 of 32 × 0x40-byte elements (element function `0x18012d6d0`) | none (vtable/callback) |
| `0x18023f4e0` | fluff | eh_ctor | destroys an array at +0x30 of 32 × 0x40-byte elements (element function `0x18012d6d0`) | none (vtable/callback) |
| `0x18023f900` | fluff | eh_ctor | destroys an array at +0x30 of 32 × 0x40-byte elements (element function `0x18012d6d0`) | none (vtable/callback) |

### lib:json+config (7)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x180141770` | fluff | thunk | forwards to `0x180142010` | none (vtable/callback) |
| `0x1801449e0` | fluff | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | `0x1801a5310` |
| `0x180144da0` | fluff | thunk | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | none (vtable/callback) |
| `0x180145620` | fluff | thunk | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | none (vtable/callback) |
| `0x180150090` | fluff | thunk | forwards to `0x180150240` | none (vtable/callback) |
| `0x1801500b0` | fluff | thunk | forwards to `0x180150390` | none (vtable/callback) |
| `0x1801500d0` | fluff | thunk | forwards to `0x1801500f0` | none (vtable/callback) |

### lib:misc (25)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x180045a00` | fluff | dtor | destructor of `RnD::Allocator` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180045a50` | fluff | dtor | destructor of `RnD::Allocator` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180045ab0` | fluff | const_ret | wrapper around `malloc` | none (vtable/callback) |
| `0x180045b50` | fluff | const_ret | calls virtual slot 3 of its object | none (vtable/callback) |
| `0x180240210` | fluff | eh_ctor | destroys an array at +0xe10 of 37 × 0x60-byte elements (element function `0x18012d660`) | none (vtable/callback) |
| `0x180240930` | fluff | thunk | forwards to `0x180134e60` | none (vtable/callback) |
| `0x180240980` | fluff | const_ret | sets the fail state of a std::ios stream | none (vtable/callback) |
| `0x180242fb0` | fluff | eh_ctor | destroys an array at +0x50 of 3 × 0x258-byte elements (element function `0x180177630`) | none (vtable/callback) |
| `0x180242fe0` | fluff | eh_ctor | destroys an array at +0x50 of 3 × 0x258-byte elements (element function `0x180177630`) | none (vtable/callback) |
| `0x180243020` | fluff | eh_ctor | destroys an array at +0x50 of 4 × 0x258-byte elements (element function `0x180177630`) | none (vtable/callback) |
| `0x1802431e0` | fluff | thunk | forwards to `0x180185910` | none (vtable/callback) |
| `0x180243d80` | fluff | eh_ctor | destroys an array at +0x40 of 12 × 0x34-byte elements (no-op element function) | none (vtable/callback) |
| `0x180244110` | fluff | const_ret | returns the constant 0 | none (vtable/callback) |
| `0x180244240` | fluff | eh_ctor | destroys an array at +0x40 of 4 × 0x80-byte elements (element function `0x1801f7ca0`) | none (vtable/callback) |
| `0x180244320` | fluff | eh_ctor | destroys an array at +0x124 of 12 × 0x34-byte elements (no-op element function) | none (vtable/callback) |
| `0x180244400` | fluff | eh_ctor | destroys an array at +0x104 of 12 × 0x34-byte elements (no-op element function) | none (vtable/callback) |
| `0x180244450` | fluff | eh_ctor | destroys an array at +0x144 of 12 × 0x34-byte elements (no-op element function) | none (vtable/callback) |
| `0x1802444c0` | fluff | eh_ctor | destroys an array at +0xf4 of 12 × 0x34-byte elements (no-op element function) | none (vtable/callback) |
| `0x180244530` | fluff | eh_ctor | destroys an array at +0x7f0 of 4 × 0xc4-byte elements (no-op element function) | none (vtable/callback) |
| `0x180244560` | fluff | eh_ctor | destroys an array at +0x6a0 of 4 × 0x54-byte elements (no-op element function) | none (vtable/callback) |
| `0x18024599a` | fluff | const_ret | returns a constant | none (vtable/callback) |
| `0x1802459b2` | fluff | const_ret | wrapper around `_scrt_release_startup_lock` | none (vtable/callback) |
| `0x1802459c9` | fluff | const_ret | wrapper around `_scrt_release_startup_lock` | none (vtable/callback) |
| `0x1802459e2` | fluff | const_ret | wrapper around `_scrt_dllmain_uninitialize_critical` | none (vtable/callback) |
| `0x180245a40` | fluff | eh_ctor | destroys an array of 4 × 0x40-byte elements (no-op element function) | `0x1800010e0` |

### libpad (20)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x1801bc440` | core | const_ret | returns the address of global `0x1805974c0` | `0x1801cce70`, `0x1801ce8e0` +1 |
| `0x1801c1500` | core | const_ret | returns a constant | `0x1801c6ee0` |
| `0x1801c6910` | fluff | const_ret | returns a constant | `0x1801cce70`, `0x1801d1d10` +5 |
| `0x1801c6920` | fluff | const_ret | returns a constant | `0x1801c1880`, `0x1801cccf0` +3 |
| `0x1801c83a0` | core | const_ret | wrapper around `timeGetTime` | none (vtable/callback) |
| `0x1801c90a0` | core | const_ret | wrapper around `memset` | `0x1801cc690` |
| `0x1801c90c0` | fluff | setter | wrapper around `HIWORD` | `0x1801cb6d0`, `0x1801cba20` +3 |
| `0x1801c9470` | core | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | `0x1801bb330`, `0x1801bbc50` +9 |
| `0x1801c9500` | core | const_ret | returns a constant | `0x1801bb330`, `0x1801bd2d0` +12 |
| `0x1801c9590` | core | const_ret | returns a constant | `0x1801ca540`, `0x1801cfe40` |
| `0x1801c95c0` | core | const_ret | returns a constant | `0x1801bb330`, `0x1801bd2d0` +11 |
| `0x1801c96c0` | fluff | const_ret | returns a constant | `0x1801bde70` |
| `0x1801caa30` | fluff | const_ret | reads global `0x1805e3498` | `0x1801c58d0`, `0x1801c6250` +1 |
| `0x1801cafc0` | fluff | const_ret | reads global `0x1805e349d` | `0x1801c6250`, `0x1801c8570` |
| `0x1801cafd0` | fluff | const_ret | reads global `0x1805b157b` | `0x1801c8570` |
| `0x1801cafe0` | fluff | const_ret | reads global `0x1805b1571` | `0x1801bb650`, `0x1801bb9d0` +5 |
| `0x1801caff0` | fluff | const_ret | reads global `0x1805b1572` | `0x1801c3020` |
| `0x1801cb000` | fluff | const_ret | reads global `0x1805b1573` | `0x1801bb650`, `0x1801bb9d0` +3 |
| `0x1801cb4b0` | core | const_ret | reads global `0x1805e349c` | `0x1801c8570` |
| `0x1801ce210` | core | copy | copies a record field by field (about 54 fields) | `0x1801cce70`, `0x1801cfe40` |

### map+steamvr (64)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x180121f30` | core | zero_init | constructor/reset of `CaesarUsbThread`: sets about 31 fields to zero or defaults | `0x180121af0` |
| `0x1801225a0` | core | dtor | destructor of `CaesarUsbThread` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x1801225f0` | core | dtor | destructor of `CaesarUsbThreadGenData` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180122690` | core | dtor | destructor of `CaesarUsbThreadImage` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180122730` | core | dtor | destructor of `CaesarUsbThreadImuStatus` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x1801227d0` | core | dtor | destructor of `CaesarUsbThreadLeddet` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180122880` | ref | dtor | destructor of `CaesarUsbThreadLog` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180122920` | ref | dtor | destructor of `CaesarUsbThreadRelocPre` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x1801229d0` | core | dtor | destructor of `CaesarUsbThreadSlamTracking` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180124cd0` | core | const_ret | returns the constant 0 | none (vtable/callback) |
| `0x180124ce0` | core | const_ret | returns the constant 10 | none (vtable/callback) |
| `0x180124cf0` | core | const_ret | returns the constant 6 | none (vtable/callback) |
| `0x180124d00` | core | const_ret | returns the constant 7 | none (vtable/callback) |
| `0x180124d10` | core | const_ret | returns the constant 8 | none (vtable/callback) |
| `0x180124d20` | ref | const_ret | returns the constant 12 | none (vtable/callback) |
| `0x180124d30` | ref | const_ret | returns the constant 9 | none (vtable/callback) |
| `0x180124d40` | core | const_ret | returns the constant 3 | none (vtable/callback) |
| `0x180124f60` | core | const_ret | returns the constant 0 | none (vtable/callback) |
| `0x180124f70` | core | const_ret | returns the constant -117 | none (vtable/callback) |
| `0x180124f80` | core | const_ret | returns the constant -121 | none (vtable/callback) |
| `0x180124f90` | core | const_ret | returns the constant -120 | none (vtable/callback) |
| `0x180124fa0` | core | const_ret | returns the constant -119 | none (vtable/callback) |
| `0x180124fb0` | ref | const_ret | returns the constant -115 | none (vtable/callback) |
| `0x180124fc0` | ref | const_ret | returns the constant -118 | none (vtable/callback) |
| `0x180124fd0` | core | const_ret | returns the constant -125 | none (vtable/callback) |
| `0x180125390` | core | thunk | forwards to `0x18016b5f0` | `0x180118920` |
| `0x1801253b0` | core | thunk | forwards to `0x18016b850` | `0x180118920` |
| `0x180125660` | ref | const_ret | returns the constant 0 | none (vtable/callback) |
| `0x180125cb0` | core | const_ret | returns the constant 0 | none (vtable/callback) |
| `0x180129370` | core | const_ret | calls virtual slot 1 of its object | none (vtable/callback) |
| `0x18012ced0` | fluff | setter | wrapper around `new` | none (vtable/callback) |
| `0x18012cef0` | fluff | setter | wrapper around `new` | none (vtable/callback) |
| `0x18012d540` | fluff | dtor | destructor of `std::stringbuf` (frees itself when the flag argument is set) | `0x18012d600`, `0x18012dd00` +15 |
| `0x1801359b0` | fluff | const_ret | returns argument 1 unchanged | none (vtable/callback) |
| `0x180135ba0` | fluff | thunk | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | none (vtable/callback) |
| `0x180135e60` | core | thunk | forwards to `0x180137580` | none (vtable/callback) |
| `0x180137a70` | core | const_ret | zeroes the byte range [first, last) | none (vtable/callback) |
| `0x180137a90` | core | const_ret | zeroes the byte range [first, last) | none (vtable/callback) |
| `0x180137ab0` | core | const_ret | zeroes the byte range [first, last) | none (vtable/callback) |
| `0x1801392e0` | fluff | setter | two-statement leaf (one store or load, then return) | `0x180119400` |
| `0x180139b60` | fluff | zero_init | constructor/reset of `Json::CharReaderBuilder`: sets about 4 fields to zero or defaults | `0x180240ee0` |
| `0x180139c80` | fluff | dtor | destructor of `Json::Exception` (frees itself when the flag argument is set) | `0x180139d20`, `0x180139e80` +3 |
| `0x180139ea0` | fluff | zero_init | constructor/reset of `Json::StreamWriterBuilder`: sets about 4 fields to zero or defaults | `0x180240f20` |
| `0x18013a080` | fluff | setter | two-statement leaf (one store or load, then return) | `0x18011df70` |
| `0x18013a6b0` | core | thunk | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | none (vtable/callback) |
| `0x18013b100` | core | thunk | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | none (vtable/callback) |
| `0x18013b130` | core | thunk | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | none (vtable/callback) |
| `0x18013b280` | fluff | thunk | forwards to `0x18013b340` | none (vtable/callback) |
| `0x18013b28c` | fluff | thunk | forwards to `0x18013b3e0` | none (vtable/callback) |
| `0x18013b340` | fluff | dtor | destructor of `std::istringstream` (frees itself when the flag argument is set) | `0x18013b280` |
| `0x18013b3e0` | fluff | dtor | destructor of `std::ostringstream` (frees itself when the flag argument is set) | `0x18013b28c` |
| `0x18013b4c0` | fluff | dtor | destructor of `Json::CharReader` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18013b580` | fluff | dtor | destructor of `Json::CharReader::Factory` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18013b5b0` | fluff | dtor | destructor of `Json::StreamWriter::Factory` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18013b5e0` | fluff | dtor | destructor of `Json::FastWriter` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18013b6c0` | fluff | dtor | destructor of `Json::CharReader` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18013b750` | fluff | dtor | destructor of `Json::StreamWriter` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18013b7d0` | fluff | dtor | destructor of `Json::Writer` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18013b940` | fluff | thunk | forwards to `0x180136040` | `0x180240ac0`, `0x180240b80` |
| `0x18013b960` | fluff | thunk | forwards to `0x1801360d0` | `0x180240b00`, `0x180240bc0` |
| `0x18013bdd0` | core | getter | identity: returns argument 1 | `0x1802409e0` |
| `0x18013bde0` | core | getter | identity: returns argument 1 | `0x180241050` |
| `0x18013df60` | core | setter | two-statement leaf (one store or load, then return) | none (vtable/callback) |
| `0x18013dfa0` | core | setter | two-statement leaf (one store or load, then return) | none (vtable/callback) |

### math+threads (21)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x180160070` | core | zero_init | constructor/reset: sets about 13 fields to zero or defaults | none (vtable/callback) |
| `0x1801601e0` | core | dtor | destructor of `TrackerManager` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x1801661a0` | fluff | thunk | forwards to `0x180164ef0` | none (vtable/callback) |
| `0x1801661d0` | fluff | thunk | forwards to `0x180165830` | `0x1801633e0`, `0x180163a10` |
| `0x18016b350` | fluff | setter | wrapper around `CloseHandle` | `0x180111940`, `0x180111970` +36 |
| `0x18016b370` | fluff | setter | wrapper around `CloseHandle` | `0x18010e3c0`, `0x18010e430` +62 |
| `0x18016b3b0` | fluff | dtor | destructor of `Framework::Thread` (frees itself when the flag argument is set) | `0x18010e390`, `0x18010e3e0` +35 |
| `0x18016b3f0` | fluff | dtor | destructor of `Framework::Event` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18016b440` | fluff | dtor | destructor of `Framework::Mutex` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18016b490` | fluff | dtor | destructor of `Framework::Semaphore` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18016b4e0` | fluff | dtor | destructor of `Framework::Thread` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18016b5f0` | fluff | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | `0x180007660`, `0x18010af50` +153 |
| `0x18016b610` | fluff | const_ret | calls virtual slot 1 of its object | `0x18016b660` |
| `0x18016b650` | fluff | const_ret | wrapper around `ResetEvent` | `0x180128590`, `0x180128600` |
| `0x18016b790` | fluff | const_ret | wrapper around `SetEvent` | `0x18010b030`, `0x18010b0b0` +44 |
| `0x18016b830` | fluff | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | `0x1801120c0`, `0x1801123f0` +9 |
| `0x18016b850` | fluff | const_ret | wrapper around `ReleaseMutex` | `0x180007660`, `0x18010af50` +157 |
| `0x18016b8c0` | fluff | getter | identity: returns argument 1 | `0x180156500`, `0x1801b91d0` |
| `0x18016b8d0` | fluff | copy | copies a record field by field (about 17 fields) | `0x180156500`, `0x18018aaf0` +3 |
| `0x18016b940` | fluff | getter | identity: returns argument 1 | `0x180154be0`, `0x180155000` +4 |
| `0x18016b990` | fluff | copy | copies a record field by field (about 17 fields) | `0x1801556b0`, `0x180156500` +2 |

### reloc (82)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x1801dc490` | ref | copy | copies a record field by field (about 51 fields) | `0x1801df630` |
| `0x1801defa0` | ref | zero_init | constructor/reset: sets about 24 fields to zero or defaults | none (vtable/callback) |
| `0x1801df900` | ref | thunk | forwards to `0x1801e2060` | none (vtable/callback) |
| `0x1801e5250` | ref | copy | copies a record field by field (about 10 fields) | `0x1801e39c0` |
| `0x1801ec130` | ref | copy | copies a record field by field (about 50 fields) | `0x1801ec460`, `0x1801ed9f0` +12 |
| `0x1801ee1e0` | ref | thunk | forwards to `0x180226a80` | none (vtable/callback) |
| `0x1801ee790` | ref | thunk | forwards to `0x1802264f0` | `0x1801de0d0`, `0x1801ded90` |
| `0x1801eeab0` | ref | copy | copies a record field by field (about 4 fields) | `0x1801de180` |
| `0x1801eeaf0` | ref | copy | copies a record field by field (about 24 fields) | `0x1801de2a0` |
| `0x1801f3ce0` | ref | zero_init | constructor/reset: sets about 25 fields to zero or defaults | `0x1801ec600` |
| `0x1801f5530` | ref | zero_init | constructor/reset: sets about 12 fields to zero or defaults | `0x1801ec600` |
| `0x1801f6e40` | ref | zero_init | constructor/reset: sets about 16 fields to zero or defaults | `0x1801ec600` |
| `0x1801f7af0` | ref | zero_init | constructor/reset: sets about 23 fields to zero or defaults | `0x1801ec600` |
| `0x1801f7c50` | ref | eh_ctor | destroys an array at +0x9f4 of 12 × 0xc4-byte elements (no-op element function) | `0x1802442f0`, `0x1802443f0` +2 |
| `0x1801ffbc0` | ref | thunk | forwards to `0x1801ff950` | none (vtable/callback) |
| `0x180200440` | ref | thunk | forwards to `0x1802000f0` | none (vtable/callback) |
| `0x1802101b0` | ref | copy | copies a record field by field (about 13 fields) | `0x18020c040` |
| `0x1802103b0` | ref | copy | copies a record field by field (about 7 fields) | `0x18021d4d0` |
| `0x180210580` | ref | copy | copies a record field by field (about 13 fields) | `0x18020c040` |
| `0x180224a80` | ref | dtor | destructor (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180224ab0` | ref | dtor | destructor (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180224ae0` | ref | dtor | destructor (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180224b10` | ref | dtor | destructor (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180224b40` | ref | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | none (vtable/callback) |
| `0x180224bf0` | ref | dtor | destructor (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180224c20` | ref | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | none (vtable/callback) |
| `0x180224d30` | ref | dtor | destructor (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180224d60` | ref | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | none (vtable/callback) |
| `0x180224e10` | ref | dtor | destructor (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180224e40` | ref | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | none (vtable/callback) |
| `0x180224fa0` | ref | dtor | destructor (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180225130` | ref | copy | copies a record field by field (about 55 fields) | `0x180224ea0`, `0x180226c40` +2 |
| `0x180225310` | ref | dtor | destructor (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180225340` | ref | dtor | destructor (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180225b70` | ref | setter | two-statement leaf (one store or load, then return) | `0x1801ec600` |
| `0x180225b90` | ref | setter | two-statement leaf (one store or load, then return) | `0x1801ec600` |
| `0x180225ba0` | ref | setter | two-statement leaf (one store or load, then return) | `0x1801ec600` |
| `0x180225bb0` | ref | setter | two-statement leaf (one store or load, then return) | `0x1801ec600` |
| `0x180226030` | ref | const_ret | returns the constant 0 | `0x1801edd00` |
| `0x180226050` | ref | const_ret | returns the constant 0 | `0x1801edd00` |
| `0x180226060` | ref | const_ret | returns the constant 0 | `0x1801edd00` |
| `0x180226070` | ref | const_ret | returns the constant 0 | `0x1801edd00` |
| `0x180226080` | ref | thunk | forwards to `0x18022f400` | `0x1801fc140`, `0x1801fd070` |
| `0x180226200` | ref | thunk | forwards to `0x18022fe30` | `0x1801f2470`, `0x1801fa720` |
| `0x1802262c0` | ref | thunk | forwards to `0x18022fe80` | `0x1801ed0b0`, `0x1801edec0` +6 |
| `0x180226310` | ref | thunk | forwards to `0x18022fed0` | `0x1801fa910` |
| `0x1802264e0` | ref | thunk | forwards to `0x180230170` | `0x1801ed0b0`, `0x1801efb90` |
| `0x1802264f0` | ref | thunk | forwards to `0x1802301f0` | `0x1801ed0b0`, `0x1801ee790` |
| `0x180226500` | ref | thunk | forwards to `0x180230250` | `0x1801ed0b0`, `0x1801efb90` +1 |
| `0x180226510` | ref | thunk | forwards to `0x180230370` | `0x1801ed0b0` |
| `0x180226520` | ref | thunk | forwards to `0x1802303f0` | `0x1801ed0b0`, `0x1801ed3d0` +10 |
| `0x180226550` | ref | thunk | forwards to `0x1802303f0` | `0x1801ed0b0`, `0x1801f0b80` |
| `0x180226570` | ref | thunk | forwards to `0x1802304f0` | `0x1801f2470` |
| `0x180226ad0` | ref | thunk | forwards to `0x1802308c0` | `0x1801edf80` |
| `0x180226f50` | ref | thunk | forwards to `0x180232220` | `0x1801efb90` |
| `0x180226f70` | ref | thunk | forwards to `0x180232220` | `0x1801fade0` |
| `0x1802276d0` | ref | const_ret | predicate: tests its argument or one field against a constant range or bit mask (no side effects) | none (vtable/callback) |
| `0x180228040` | ref | const_ret | returns a constant | `0x180008160` |
| `0x180228050` | ref | zero_init | constructor/reset: sets about 9 fields to zero or defaults | `0x1801f7080`, `0x1801f8750` |
| `0x180228c70` | ref | thunk | forwards to `0x180228cc0` | none (vtable/callback) |
| `0x180228f90` | ref | getter | identity: returns argument 1 | `0x1801f8750` |
| `0x180228fd0` | ref | const_ret | returns the constant 0 | `0x1801f8750` |
| `0x18022b360` | ref | getter | identity: returns argument 1 | `0x1801f8750` |
| `0x18022c4e0` | ref | setter | two-statement leaf (one store or load, then return) | `0x180225ad0` |
| `0x18022c550` | ref | thunk | forwards to `0x180238520` | `0x180225c40` |
| `0x18022c780` | ref | thunk | forwards to `0x180238880` | `0x180225c00`, `0x180226f80` |
| `0x18022c7a0` | ref | thunk | forwards to `0x180238870` | `0x180225970`, `0x180227140` +1 |
| `0x18022c7c0` | ref | thunk | forwards to `0x1802388c0` | `0x1802270a0`, `0x180227140` |
| `0x18022c7e0` | ref | thunk | wrapper around `memcmp` | `0x1802312c0`, `0x1802328f0` +1 |
| `0x18022c850` | ref | thunk | wrapper around `memcmp` | none (vtable/callback) |
| `0x18022c930` | ref | eh_ctor | destroys an array at +0x11cba98 of 4848 × 0xd8-byte elements (no-op element function) | none (vtable/callback) |
| `0x18022dff0` | ref | const_ret | wrapper around `write` | none (vtable/callback) |
| `0x18022e180` | ref | eh_ctor | destroys an array of 97 × 0x2ed20-byte elements (element function `0x18022e1b0`) | `0x1802458d0` |
| `0x18022e1b0` | ref | eh_ctor | destroys an array at +0xf0 of 256 × 0x2ec-byte elements (no-op element function) | `0x18022c930`, `0x18022e180` +2 |
| `0x18022e1e0` | ref | copy | copies a record field by field (about 55 fields) | `0x18022ebe0`, `0x180230740` +2 |
| `0x18022edf0` | ref | thunk | forwards to `0x1801aa7b0` | `0x1802275c0` |
| `0x180230cf0` | ref | const_ret | returns the constant 19732648 | `0x1802260b0`, `0x180226a80` +1 |
| `0x1802315b0` | ref | const_ret | returns and increments the counter `0x1805e5d5c` | `0x180224ba0`, `0x180226c00` |
| `0x1802315c0` | ref | const_ret | returns and increments the counter `0x1805e5d74` | `0x180224ba0`, `0x180226c00` |
| `0x180232e00` | ref | const_ret | wrapper around `memset` | none (vtable/callback) |
| `0x180237710` | ref | zero_init | constructor/reset: sets about 14 fields to zero or defaults | `0x18022c600` |
| `0x180238ae0` | ref | thunk | forwards to `0x180238b00` | `0x180236950` |

### sense (2)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x18011b480` | core | thunk | forwards to `0x1801aa6d0` | `0x180119310`, `0x18011b4a0` |
| `0x18011d3c0` | core | thunk | reads a field of the entry selected by argument 2 in a pointer table | `0x18018aaf0`, `0x180193a90` +1 |

### steamvr (51)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x1801a3480` | core | const_ret | returns the constant 0 | none (vtable/callback) |
| `0x1801a34b0` | core | const_ret | returns the address of global `0x180351110` | none (vtable/callback) |
| `0x1801a3870` | core | zero_init | constructor/reset of `sie::psvr2::SenseDevice`: sets about 34 fields to zero or defaults | `0x1801a2040` |
| `0x1801a3a20` | fluff | dtor | destructor of `CheckThread` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x1801a3d30` | fluff | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x1801a3d40` | fluff | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x1801a3d50` | fluff | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x1801a3d60` | fluff | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x1801a3d70` | fluff | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x1801a3d80` | fluff | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x1801a3d90` | fluff | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x1801a3da0` | fluff | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x1801a3db0` | fluff | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x1801a41c0` | core | getter | returns the address of the sub-object at +0x8 | none (vtable/callback) |
| `0x1801a41d0` | core | getter | returns the address of the sub-object at +0x8 | none (vtable/callback) |
| `0x1801a41e0` | core | getter | returns the address of the sub-object at +0x8 | none (vtable/callback) |
| `0x1801a41f0` | core | getter | returns the address of the sub-object at +0x8 | none (vtable/callback) |
| `0x1801a4200` | core | getter | returns the address of the sub-object at +0x8 | none (vtable/callback) |
| `0x1801a4210` | core | getter | returns the address of the sub-object at +0x8 | none (vtable/callback) |
| `0x1801a4220` | core | getter | returns the address of the sub-object at +0x8 | none (vtable/callback) |
| `0x1801a4230` | core | getter | returns the address of the sub-object at +0x8 | none (vtable/callback) |
| `0x1801a4240` | core | getter | returns the address of the sub-object at +0x8 | none (vtable/callback) |
| `0x1801a4250` | core | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x1801a4260` | core | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x1801a4270` | core | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x1801a4280` | core | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x1801a4290` | core | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x1801a42a0` | core | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x1801a42b0` | core | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x1801a42c0` | core | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x1801a42d0` | core | setter | std::function plumbing: installs a lambda's call table into the target | none (vtable/callback) |
| `0x1801a42e0` | core | const_ret | std::function plumbing: returns the lambda's RTTI type descriptor | none (vtable/callback) |
| `0x1801a42f0` | core | const_ret | std::function plumbing: returns the lambda's RTTI type descriptor | none (vtable/callback) |
| `0x1801a4300` | core | const_ret | std::function plumbing: returns the lambda's RTTI type descriptor | none (vtable/callback) |
| `0x1801a4310` | core | const_ret | std::function plumbing: returns the lambda's RTTI type descriptor | none (vtable/callback) |
| `0x1801a4320` | core | const_ret | std::function plumbing: returns the lambda's RTTI type descriptor | none (vtable/callback) |
| `0x1801a4330` | core | const_ret | std::function plumbing: returns the lambda's RTTI type descriptor | none (vtable/callback) |
| `0x1801a4340` | core | const_ret | std::function plumbing: returns the lambda's RTTI type descriptor | none (vtable/callback) |
| `0x1801a4350` | core | const_ret | std::function plumbing: returns the lambda's RTTI type descriptor | none (vtable/callback) |
| `0x1801a4360` | core | const_ret | std::function plumbing: returns the lambda's RTTI type descriptor | none (vtable/callback) |
| `0x1801a5710` | core | dtor | destructor of `UpdateControllerThread` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x1801aae40` | ref | const_ret | zeroes the byte range [first, last) | none (vtable/callback) |
| `0x1801aae60` | ref | const_ret | zeroes the byte range [first, last) | none (vtable/callback) |
| `0x1801ab320` | ref | dtor | destructor of `Relocalizer` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x1801ab380` | ref | dtor | destructor of `RelocalizerThread` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x1801abf40` | core | const_ret | returns the address of global `0x180433908` | `0x180120290`, `0x1801211a0` +9 |
| `0x1801ac120` | core | thunk | forwards to `0x1801ad880` with extra constant arguments | `0x1801a3e50`, `0x1801a4040` +1 |
| `0x1801ac1d0` | core | thunk | forwards to `0x1801ad880` with extra constant arguments | `0x1801a4180` |
| `0x1801ad960` | core | thunk | forwards to `0x1801ad880` with extra constant arguments | `0x1801a3f30` |
| `0x1801ad980` | core | thunk | forwards to `0x1801ad880` with extra constant arguments | `0x1801a3e50` |
| `0x1801adb80` | core | thunk | forwards to `0x1801ad880` with extra constant arguments | `0x180120290`, `0x1801211a0` +1 |

### tracking (7)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x180047200` | core | eh_ctor | destroys an array of 6 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x180047220` | core | eh_ctor | destroys an array of 6 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
| `0x180047460` | core | const_ret | returns the constant 0 | none (vtable/callback) |
| `0x180047470` | core | const_ret | returns the constant 1 | none (vtable/callback) |
| `0x180047480` | core | const_ret | returns the constant 2 | none (vtable/callback) |
| `0x180047490` | core | const_ret | returns the constant 3 | none (vtable/callback) |
| `0x1800474b0` | fluff | eh_ctor | destroys an array of 4 × 0x40-byte elements (no-op element function) | none (vtable/callback) |
<!-- gen:trivial:end -->

---

## Part E: Library and runtime code

<!-- gen:library:start -->
Generated by `scripts/sony-re/triage/libcat.py`. Each row is a function
that one piece of evidence identifies: an IDA FLIRT match from the MSVC
signature sets (`flirt`; MFC, ATL and unnamed matches are dropped as
collisions: the DLL links none of them), the RTTI vtable of a library
class it sits in (`vtable`), or a string it uses (`string`). Vtable
rows are `[sure]` for the class; FLIRT and string rows are `[likely]`
(FLIRT can collide on short bodies). String rows outside the library
families are Sony fluff (VrTrace dumpers, companion-app dialogs)
named by their log text. None of this code needs porting: the Linux
driver uses the system's C++ library, jsoncpp and LAPACK.

The f2c CLAPACK block (`lib:linalg`) has its own hand-written part,
part C.

370 functions: flirt 154, string 108, vtable 108.

### app:dialog (23)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x18012da60` | fluff | string | uses 'IVRServerDriverHost_006', '[VrDialogManager] finish psvr2 by update paused' | app |
| `0x18012dd70` | fluff | flirt | `std::basic_stringbuf<char,std::char_traits<char>,alloc>::`scalar deleting destructor'(uint)` | MSVC C++ standard library |
| `0x18012e8b0` | fluff | string | uses 'IVRProperties_001', 'appmanifest_2580190.acf' | app |
| `0x18012f3a0` | fluff | string | uses '[VrTracker2] initial setup complete flow', '[VrTracker2] initial setup complete flow and vr app finish' | app |
| `0x18012f4f0` | fluff | string | uses '[VrTracker2] safety notice limit on, %d', '[VrTracker2] safety notice limit over' | app |
| `0x18012f5a0` | fluff | string | uses '[VrTracker2] m_config.ini date:%s', '%d-%d-%d %d:%d:%d' | app |
| `0x18012f830` | fluff | string | uses 'IVRProperties_001', '\dashboard\' | app |
| `0x180130020` | fluff | string | uses 'IVRProperties_001', '\dialog\' | app |
| `0x180130ee0` | fluff | string | uses '[VrDialogProcess] Lang and Cntry match:', '[VrDialogProcess] Lang match:' | app |
| `0x180131390` | fluff | string | uses 'IVRSettings_003', 'enable' | app |
| `0x180131d90` | fluff | string | uses '[VrDialogManager] isExeRunning(psvr2DesktopAppExe): %d', '[VrDialogManager] Create DesktopAppProcess' | app |
| `0x1801323d0` | fluff | vtable | virtual method of `std::basic_stringbuf<char,struct std::char_traits<char>,class std::allocator<char>>` | MSVC C++ standard library |
| `0x180132580` | fluff | flirt | `std::basic_stringbuf<char,std::char_traits<char>,alloc>::pbackfail(int)` | MSVC C++ standard library |
| `0x1801325e0` | fluff | vtable | virtual method of `CreateProcessThread` | app |
| `0x1801335a0` | fluff | string | uses '%Y-%m-%d %H:%M:%S' | app |
| `0x1801336c0` | fluff | vtable | virtual method of `std::basic_stringbuf<char,struct std::char_traits<char>,class std::allocator<char>>` | MSVC C++ standard library |
| `0x180133820` | fluff | vtable | virtual method of `std::basic_stringbuf<char,struct std::char_traits<char>,class std::allocator<char>>` | MSVC C++ standard library |
| `0x180133ca0` | fluff | string | uses 'IVRSettings_003', 'enable' | app |
| `0x180133f40` | fluff | string | uses '[VrTracker2] desktop guide off' | app |
| `0x180134070` | fluff | string | uses '[VrTracker2] controller need update %d -> %d' | app |
| `0x180134230` | fluff | string | uses '[VrTracker2] inside or outside %d -> %d' | app |
| `0x180134510` | fluff | vtable | virtual method of `std::basic_stringbuf<char,struct std::char_traits<char>,class std::allocator<char>>` | MSVC C++ standard library |
| `0x1801350e0` | fluff | string | uses '1.2.840.113549.1.9.5' | app |

### config (1)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x1801585e0` | fluff | flirt | `std::vector<void *,alloc>::_Xlen(void)` | MSVC C++ standard library |

### ctrl-boot (3)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x1800bc6e0` | fluff | string | uses 'vector too long' | lib |
| `0x1800bc8e0` | fluff | string | uses 'vector too long' | lib |
| `0x1800bcb00` | fluff | string | uses 'vector too long' | lib |

### ctrl-tracker (4)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x180050e90` | ref | flirt | `std::_Get_size_of_n<2>(unsigned __int64)` | MSVC C++ standard library |
| `0x180054070` | fluff | vtable | virtual method of `WorkerThread` | platform |
| `0x180054530` | fluff | vtable | virtual method of `WorkerThread` | platform |
| `0x180059790` | core | flirt | `??$__std_minmax_disp@$02U_Minmax_traits_d@?A0x8417fefe@@$00@?A0x8417fefe@@YA?A_PQEBX0@Z` | MSVC C runtime |

### debug:trace (86)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x18010ac20` | fluff | string | uses 'controller_pad%d.bin' | debug |
| `0x18010b1e0` | fluff | vtable | virtual method of `VrTraceApiDataDumper` | debug |
| `0x18010b230` | fluff | vtable | virtual method of `VrTraceControllerPadDumper` | debug |
| `0x18010b270` | fluff | vtable | virtual method of `VrTraceControllerTrackingDumper` | debug |
| `0x18010b2b0` | fluff | vtable | virtual method of `VrTraceHmdGeneralImageDumper` | debug |
| `0x18010b2f0` | fluff | vtable | virtual method of `VrTraceImuDumper` | debug |
| `0x18010b330` | fluff | vtable | virtual method of `VrTraceInsDumper` | debug |
| `0x18010b370` | fluff | vtable | virtual method of `VrTracePlayareaDumper` | debug |
| `0x18010b3b0` | fluff | vtable | virtual method of `VrTraceRelocContextDumper` | debug |
| `0x18010b3f0` | fluff | vtable | virtual method of `VrTraceRelocSnapshotDumper` | debug |
| `0x18010b430` | fluff | vtable | virtual method of `VrTraceSelfCalibDumper` | debug |
| `0x18010b4d0` | fluff | vtable | virtual method of `VrTraceSlamDataDumper` | debug |
| `0x18010b710` | fluff | vtable | virtual method of `VrTraceApiDataDumper` | debug |
| `0x18010b770` | fluff | vtable | virtual method of `VrTraceCameraSelfCalibDumper` | debug |
| `0x18010b830` | fluff | vtable | virtual method of `VrTraceControllerPadDumper` | debug |
| `0x18010b890` | fluff | vtable | virtual method of `VrTraceControllerTrackingDumper` | debug |
| `0x18010c0d0` | fluff | vtable | virtual method of `VrTraceInsDumper` | debug |
| `0x18010c120` | fluff | vtable | virtual method of `VrTracePlayareaDumper` | debug |
| `0x18010c170` | fluff | vtable | virtual method of `VrTraceRelocContextDumper` | debug |
| `0x18010c1c0` | fluff | vtable | virtual method of `VrTraceRelocSnapshotDumper` | debug |
| `0x18010c210` | fluff | vtable | virtual method of `VrTraceSelfCalibDumper` | debug |
| `0x18010c2e0` | fluff | vtable | virtual method of `VrTraceSlamDataDumper` | debug |
| `0x18010ce30` | fluff | vtable | virtual method of `VrTraceApiDataDumper` | debug |
| `0x18010ced0` | fluff | vtable | virtual method of `VrTraceControllerPadDumper` | debug |
| `0x18010cf70` | fluff | vtable | virtual method of `VrTraceControllerTrackingDumper` | debug |
| `0x18010d010` | fluff | vtable | virtual method of `VrTraceHmdGeneralImageDumper` | debug |
| `0x18010d060` | fluff | vtable | virtual method of `VrTraceImuDumper` | debug |
| `0x18010d090` | fluff | vtable | virtual method of `VrTraceInsDumper` | debug |
| `0x18010d130` | fluff | vtable | virtual method of `VrTracePlayareaDumper` | debug |
| `0x18010d160` | fluff | vtable | virtual method of `VrTraceRelocContextDumper` | debug |
| `0x18010d200` | fluff | vtable | virtual method of `VrTraceRelocSnapshotDumper` | debug |
| `0x18010d2a0` | fluff | vtable | virtual method of `VrTraceSelfCalibDumper` | debug |
| `0x18010d400` | fluff | vtable | virtual method of `VrTraceSlamDataDumper` | debug |
| `0x18010d4f0` | fluff | vtable | virtual method of `VrTraceApiDataDumper` | debug |
| `0x18010d560` | fluff | vtable | virtual method of `VrTraceControllerPadDumper` | debug |
| `0x18010d5d0` | fluff | vtable | virtual method of `VrTraceControllerTrackingDumper` | debug |
| `0x18010d640` | fluff | vtable | virtual method of `VrTraceHmdGeneralImageDumper` | debug |
| `0x18010d660` | fluff | vtable | virtual method of `VrTraceImuDumper` | debug |
| `0x18010d6d0` | fluff | vtable | virtual method of `VrTraceInsDumper` | debug |
| `0x18010d740` | fluff | vtable | virtual method of `VrTracePlayareaDumper` | debug |
| `0x18010d760` | fluff | vtable | virtual method of `VrTraceRelocContextDumper` | debug |
| `0x18010d7d0` | fluff | vtable | virtual method of `VrTraceRelocSnapshotDumper` | debug |
| `0x18010d840` | fluff | vtable | virtual method of `VrTraceSelfCalibDumper` | debug |
| `0x18010d930` | fluff | vtable | virtual method of `VrTraceSlamDataDumper` | debug |
| `0x18010d9a0` | fluff | vtable | virtual method of `VrTraceApiDataDumper` | debug |
| `0x18010da60` | fluff | vtable | virtual method of `VrTraceControllerPadDumper` | debug |
| `0x18010db20` | fluff | vtable | virtual method of `VrTraceControllerTrackingDumper` | debug |
| `0x18010dbe0` | fluff | vtable | virtual method of `VrTraceHmdGeneralImageDumper` | debug |
| `0x18010dc40` | fluff | vtable | virtual method of `VrTraceImuDumper` | debug |
| `0x18010dca0` | fluff | vtable | virtual method of `VrTraceInsDumper` | debug |
| `0x18010dd60` | fluff | vtable | virtual method of `VrTracePlayareaDumper` | debug |
| `0x18010ddc0` | fluff | vtable | virtual method of `VrTraceRelocContextDumper` | debug |
| `0x18010de80` | fluff | vtable | virtual method of `VrTraceRelocSnapshotDumper` | debug |
| `0x18010df40` | fluff | vtable | virtual method of `VrTraceSelfCalibDumper` | debug |
| `0x18010e1d0` | fluff | vtable | virtual method of `VrTraceSlamDataDumper` | debug |
| `0x18010e7d0` | fluff | string | uses '[SceVrTracker2][VrTraceCore]: disableCapture' | debug |
| `0x18010eae0` | fluff | string | uses '[SceVrTracker2][VrTraceCore]: Failed to start capture dumpin' | debug |
| `0x18010eed0` | fluff | string | uses '[SceVrTracker2][VrTraceCore]: enableCapture' | debug |
| `0x180110190` | fluff | vtable | virtual method of `VrTraceCameraSelfCalibDumper`, `VrTraceDumper` | debug |
| `0x180110580` | fluff | vtable | virtual method of `VrTraceApiDataDumper`, `VrTraceCameraSelfCalibDumper` | debug |
| `0x1801106e0` | fluff | string | uses '[SceVrTracker2][VrTraceDumper]: Failed to open %s' | debug |
| `0x180110840` | fluff | vtable | virtual method of `VrTraceCameraSelfCalibDumper`, `VrTraceDumper` | debug |
| `0x180110850` | fluff | vtable | virtual method of `VrTraceCameraSelfCalibDumper`, `VrTraceDumper` | debug |
| `0x1801112a0` | fluff | string | uses 'SceVrtMtxTraceRing' | debug |
| `0x180111330` | fluff | string | uses 'SceVrtMtxTraceSRing' | debug |
| `0x180111f80` | fluff | vtable | virtual method of `VrTraceManagerImpl` | debug |
| `0x180115960` | fluff | vtable | virtual method of `std::_Func_impl_no_alloc<class <lambda_07085a55780effed2c535d4a4769b40f>,void,>` | MSVC C++ standard library |
| `0x180115970` | fluff | vtable | virtual method of `std::_Func_impl_no_alloc<class <lambda_07085a55780effed2c535d4a4769b40f>,void,>` | MSVC C++ standard library |
| `0x180115e00` | fluff | string | uses '[SceVrTracker2][VrTraceCaptureAgent]: Capture stopping.' | debug |
| `0x180115e20` | fluff | string | uses '[SceVrTracker2][VrTraceCaptureAgent]: Capture starting (type' | debug |
| `0x180116770` | fluff | string | uses '[SceVrTracker2][VrTraceCaptureAgent]: Stopping capture' | debug |
| `0x180117ab0` | fluff | flirt | `std::bad_cast::bad_cast(std::bad_cast const &)` | MSVC C++ standard library |
| `0x180117af0` | fluff | flirt | `std::bad_cast::bad_cast(void)` | MSVC C++ standard library |
| `0x180118160` | fluff | vtable | virtual method of `std::basic_filebuf<char,struct std::char_traits<char>>` | MSVC C++ standard library |
| `0x1801184b0` | fluff | vtable | virtual method of `std::basic_filebuf<char,struct std::char_traits<char>>` | MSVC C++ standard library |
| `0x180118500` | fluff | flirt | `std::_Throw_bad_cast(void)` | MSVC C++ standard library |
| `0x180118520` | fluff | vtable | virtual method of `std::basic_filebuf<char,struct std::char_traits<char>>` | MSVC C++ standard library |
| `0x180118540` | fluff | flirt | `std::vector<void *,alloc>::_Xlen(void)` | MSVC C++ standard library |
| `0x180118560` | fluff | flirt | `std::vector<void *,alloc>::_Xlen(void)` | MSVC C++ standard library |
| `0x1801192c0` | fluff | vtable | virtual method of `std::basic_filebuf<char,struct std::char_traits<char>>` | MSVC C++ standard library |
| `0x180119f50` | fluff | vtable | virtual method of `std::basic_filebuf<char,struct std::char_traits<char>>` | MSVC C++ standard library |
| `0x18011a170` | fluff | vtable | virtual method of `std::basic_filebuf<char,struct std::char_traits<char>>` | MSVC C++ standard library |
| `0x18011a6c0` | fluff | vtable | virtual method of `std::basic_filebuf<char,struct std::char_traits<char>>` | MSVC C++ standard library |
| `0x18011a790` | fluff | vtable | virtual method of `std::basic_filebuf<char,struct std::char_traits<char>>` | MSVC C++ standard library |
| `0x18011d7e0` | fluff | vtable | virtual method of `std::basic_filebuf<char,struct std::char_traits<char>>` | MSVC C++ standard library |
| `0x18011d950` | fluff | vtable | virtual method of `std::basic_filebuf<char,struct std::char_traits<char>>` | MSVC C++ standard library |

### import/thunk (75)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x180041d80` | fluff | flirt | `vsnprintf` | MSVC C runtime |
| `0x1800453d0` | fluff | flirt | ``vector constructor iterator'(void *,unsigned __int64,unsigned __int64,void * (*)(void *))` | MSVC C runtime |
| `0x18004ea50` | fluff | flirt | ``std::basic_string<char,std::char_traits<char>,alloc>::assign(char const * const,unsigned __int64)'::`2'::<...` | MSVC C runtime |
| `0x18004eb10` | fluff | flirt | ``std::basic_string<char,std::char_traits<char>,alloc>::append(unsigned __int64,char)'::`2'::<lambda>::opera...` | MSVC C runtime |
| `0x18004ebf0` | fluff | flirt | ``std::basic_string<char,std::char_traits<char>,alloc>::append(char const * const,unsigned __int64)'::`2'::<...` | MSVC C runtime |
| `0x180050df0` | fluff | flirt | `std::_Deallocate<16,0>(void *,unsigned __int64)` | MSVC C++ standard library |
| `0x180115a10` | fluff | vtable | virtual method of `VrTraceThread<class <lambda_2896c985446f4b82a337b342e2297fb0>>` | MSVC runtime glue (import thunks, unwind funclets) |
| `0x1801184d0` | fluff | flirt | `std::basic_filebuf<char,std::char_traits<char>>::_Reset_back(void)` | MSVC C++ standard library |
| `0x180154810` | fluff | flirt | `snprintf` | MSVC C runtime |
| `0x180185ad0` | fluff | flirt | `free` | MSVC C runtime |
| `0x1801a5650` | fluff | string | uses 'IServerTrackedDeviceProvider_004' | MSVC runtime glue (import thunks, unwind funclets) |
| `0x180238fbc` | fluff | flirt | `std::_Fac_node::~_Fac_node(void)` | MSVC C++ standard library |
| `0x180238ff4` | fluff | flirt | `std::_Fac_tidy_reg_t::~_Fac_tidy_reg_t(void)` | MSVC C++ standard library |
| `0x180239050` | fluff | flirt | `std::_Fac_node::`scalar deleting destructor'(uint)` | MSVC C++ standard library |
| `0x1802390ac` | fluff | flirt | `std::_Facet_Register(std::_Facet_base *)` | MSVC C++ standard library |
| `0x180239204` | fluff | vtable | virtual method of `std::basic_stringbuf<char,struct std::char_traits<char>,class std::allocator<char>>` | MSVC C++ standard library |
| `0x18023920a` | fluff | vtable | virtual method of `std::basic_stringbuf<char,struct std::char_traits<char>,class std::allocator<char>>` | MSVC C++ standard library |
| `0x180239210` | fluff | vtable | virtual method of `std::basic_stringbuf<char,struct std::char_traits<char>,class std::allocator<char>>` | MSVC C++ standard library |
| `0x1802393bc` | fluff | flirt | ``anonymous namespace'::_Create_symlink(wchar_t const * const,wchar_t const * const,ulong)` | MSVC C runtime |
| `0x180239430` | fluff | flirt | ``anonymous namespace'::_Get_last_write_time_by_handle(void * const,__int64 * const)` | MSVC C runtime |
| `0x180239534` | fluff | flirt | `__std_fs_change_permissions` | MSVC C++ standard library |
| `0x1802396d0` | fluff | flirt | `__std_fs_convert_narrow_to_wide` | MSVC C++ standard library |
| `0x180239718` | fluff | flirt | `__std_fs_convert_wide_to_narrow` | MSVC C++ standard library |
| `0x180239b24` | fluff | flirt | `__std_fs_create_directory` | MSVC C++ standard library |
| `0x180239ba8` | fluff | flirt | `__std_fs_create_directory_template` | MSVC C++ standard library |
| `0x180239c58` | fluff | flirt | `__std_fs_directory_iterator_open` | MSVC C++ standard library |
| `0x180239d00` | fluff | flirt | `__std_fs_get_current_path` | MSVC C++ standard library |
| `0x180239d3c` | fluff | flirt | `__std_fs_get_file_attributes_by_handle` | MSVC C++ standard library |
| `0x180239d90` | fluff | flirt | `__std_fs_get_file_id` | MSVC C++ standard library |
| `0x180239ea4` | fluff | flirt | `__std_fs_get_final_path_name_by_handle` | MSVC C++ standard library |
| `0x180239ed4` | fluff | flirt | `__std_fs_get_full_path_name` | MSVC C++ standard library |
| `0x180239f04` | fluff | flirt | `__std_fs_get_stats` | MSVC C++ standard library |
| `0x18023a198` | fluff | flirt | `__std_fs_get_temp_path` | MSVC C++ standard library |
| `0x18023a23c` | fluff | flirt | `__std_fs_open_handle` | MSVC C++ standard library |
| `0x18023a2d8` | fluff | flirt | `__std_fs_read_reparse_data_buffer` | MSVC C++ standard library |
| `0x18023a448` | fluff | flirt | `__std_fs_resize_file` | MSVC C++ standard library |
| `0x18023a518` | fluff | flirt | `__std_fs_set_last_write_time` | MSVC C++ standard library |
| `0x18023a804` | fluff | flirt | `__std_system_error_allocate_message` | MSVC C++ standard library |
| `0x18023aa48` | fluff | flirt | `_Init_thread_abort` | MSVC C runtime |
| `0x18023aa78` | fluff | flirt | `_Init_thread_footer` | MSVC C runtime |
| `0x18023aad8` | fluff | flirt | `_Init_thread_header` | MSVC C runtime |
| `0x18023ab50` | fluff | flirt | `_Init_thread_notify` | MSVC C runtime |
| `0x18023aba4` | fluff | flirt | `_Init_thread_wait` | MSVC C runtime |
| `0x18023ac08` | fluff | flirt | ``eh vector constructor iterator'(void *,unsigned __int64,unsigned __int64,void (*)(void *),void (*)(void *))` | MSVC C runtime |
| `0x18023ac80` | fluff | flirt | ``eh vector destructor iterator'(void *,unsigned __int64,unsigned __int64,void (*)(void *))` | MSVC C runtime |
| `0x18023ad34` | fluff | flirt | `__ArrayUnwind(void *,unsigned __int64,unsigned __int64,void (*)(void *))` | MSVC C runtime |
| `0x18023adb8` | fluff | flirt | `__GSHandlerCheckCommon` | MSVC C runtime |
| `0x18023ae14` | fluff | flirt | `__GSHandlerCheck_EH4` | MSVC C runtime |
| `0x18023aeb0` | fluff | flirt | `__alloca_probe` | MSVC C runtime |
| `0x18023af10` | fluff | flirt | `__security_check_cookie` | MSVC C runtime |
| `0x18023b050` | fluff | flirt | `__scrt_acquire_startup_lock` | MSVC C runtime |
| `0x18023b08c` | fluff | flirt | `__scrt_dllmain_after_initialize_c` | MSVC C runtime |
| `0x18023b118` | fluff | flirt | `__scrt_dllmain_exception_filter` | MSVC C runtime |
| `0x18023b178` | fluff | flirt | `__scrt_dllmain_uninitialize_c` | MSVC C runtime |
| `0x18023b1bc` | fluff | flirt | `__scrt_initialize_crt` | MSVC C runtime |
| `0x18023b294` | fluff | flirt | `__scrt_is_nonwritable_in_current_image` | MSVC C runtime |
| `0x18023b32c` | fluff | flirt | `__scrt_release_startup_lock` | MSVC C runtime |
| `0x18023b350` | fluff | flirt | `__scrt_uninitialize_crt` | MSVC C runtime |
| `0x18023b37c` | fluff | flirt | `_onexit` | MSVC C runtime |
| `0x18023b400` | fluff | flirt | `__raise_securityfailure` | MSVC C runtime |
| `0x18023b6c8` | fluff | flirt | `capture_current_context` | MSVC C runtime |
| `0x18023b738` | fluff | flirt | `capture_previous_context` | MSVC C runtime |
| `0x18023b9a0` | fluff | flirt | `dllmain_dispatch(HINSTANCE__ * const,ulong,void * const)` | MSVC C runtime |
| `0x18023baf8` | fluff | flirt | `DllEntryPoint` | MSVC C runtime |
| `0x18023bb38` | fluff | flirt | `_guard_check_icall_nop` | MSVC C runtime |
| `0x18023be40` | fluff | flirt | `__scrt_get_show_window_mode` | MSVC C runtime |
| `0x18023be88` | fluff | flirt | `__scrt_is_managed_app` | MSVC C runtime |
| `0x18023bf94` | fluff | flirt | `__isa_available_init` | MSVC C runtime |
| `0x18023c1b8` | fluff | flirt | `__security_init_cookie` | MSVC C runtime |
| `0x18023c264` | fluff | flirt | `DllMain` | MSVC C runtime |
| `0x18023c40c` | fluff | flirt | `_CxxThrowException` | MSVC C runtime |
| `0x18023c556` | fluff | flirt | `terminate` | MSVC C runtime |
| `0x18023c6d0` | fluff | flirt | `_cexit` | MSVC C runtime |
| `0x18023c6d6` | fluff | flirt | `_initterm` | MSVC C runtime |
| `0x180246524` | fluff | flirt | `std::_Fac_tidy_reg_t::~_Fac_tidy_reg_t(void)` | MSVC C++ standard library |

### init (8)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x180002c20` | fluff | string | uses '\sceBoundaryMeta.bin' | static initialisers |
| `0x180002e70` | fluff | string | uses 'es-419', 'es-ES' | static initialisers |
| `0x180003c40` | fluff | string | uses 'es-419', 'es-ES' | static initialisers |
| `0x1800049d0` | fluff | string | uses 'es-419', 'es-ES' | static initialisers |
| `0x1800057a0` | fluff | string | uses 'psvr2_overlay.exe' | static initialisers |
| `0x180005800` | fluff | string | uses 'PlayStationVR2App.exe' | static initialisers |
| `0x180005900` | fluff | string | uses 'es-419', 'es-ES' | static initialisers |
| `0x180007660` | fluff | string | uses 'ImuRingBuffer' | static initialisers |

### ledsync (1)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x18019cf80` | fluff | string | uses '[playstation_vr2_driver]' | app |

### lib:crt (62)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x180042000` | fluff | flirt | `std::exception::what(void)` | MSVC C++ standard library |
| `0x180042100` | fluff | flirt | `std::bad_array_new_length::bad_array_new_length(void)` | MSVC C++ standard library |
| `0x180042330` | fluff | string | uses 'string too long' | MSVC C runtime |
| `0x180042570` | fluff | flirt | `std::error_category::default_error_condition(int)` | MSVC C++ standard library |
| `0x180042c50` | fluff | flirt | `std::_System_error_message::_System_error_message(ulong)` | MSVC C++ standard library |
| `0x180042cf0` | fluff | flirt | `std::_Iostream_error_category2::`scalar deleting destructor'(uint)` | MSVC C++ standard library |
| `0x180042d40` | fluff | string | uses 'unknown error' | MSVC C runtime |
| `0x180042e40` | fluff | flirt | `std::_Iostream_error_category2::`scalar deleting destructor'(uint)` | MSVC C++ standard library |
| `0x180043030` | fluff | flirt | `std::make_error_code(std::io_errc)` | MSVC C++ standard library |
| `0x1800432d0` | fluff | string | uses 'Unknown exception' | MSVC C runtime |
| `0x180043380` | fluff | string | uses 'Unknown exception' | MSVC C runtime |
| `0x180043470` | fluff | vtable | virtual method of `std::filesystem::filesystem_error` | MSVC C++ standard library |
| `0x180043480` | fluff | flirt | `parse_bcp47` | MSVC C runtime |
| `0x180043810` | fluff | string | uses 'current_path()' | MSVC C runtime |
| `0x180043900` | fluff | string | uses 'current_path(const path&)' | MSVC C runtime |
| `0x180043bd0` | fluff | string | uses 'map/set too long' | MSVC C runtime |
| `0x18004e890` | fluff | flirt | ``std::basic_string<char,std::char_traits<char>,alloc>::append(unsigned __int64,char)'::`2'::<lambda>::opera...` | MSVC C runtime |
| `0x18004f000` | fluff | flirt | `alloc::deallocate(wchar_t * const,unsigned __int64)` | MSVC C++ standard library |
| `0x18004f110` | fluff | flirt | `alloc::deallocate(char * const,unsigned __int64)` | MSVC C++ standard library |
| `0x1802392e4` | fluff | flirt | ``anonymous namespace'::_Allocated_string::~_Allocated_string(void)` | MSVC C runtime |
| `0x180239300` | fluff | flirt | `std::_Fs_file::~_Fs_file(void)` | MSVC C++ standard library |
| `0x180239324` | fluff | flirt | `__crt_unique_heap_ptr<wchar_t,__crt_internal_free_policy>::operator=(__crt_unique_heap_ptr<wchar_t,__crt_in...` | MSVC C runtime |
| `0x1802394d4` | fluff | flirt | ``anonymous namespace'::__vcp_Copyfile(wchar_t const * const,wchar_t const * const,bool)` | MSVC C runtime |
| `0x180239518` | fluff | flirt | ``anonymous namespace'::_Allocated_string::~_Allocated_string(void)` | MSVC C runtime |
| `0x180239688` | fluff | flirt | `__std_fs_close_handle` | MSVC C++ standard library |
| `0x1802396a8` | fluff | flirt | `__std_fs_code_page` | MSVC C++ standard library |
| `0x18023983c` | fluff | flirt | `__std_fs_convert_wide_to_narrow_replace_chars` | MSVC C++ standard library |
| `0x180239908` | fluff | flirt | `__std_fs_copy_file` | MSVC C++ standard library |
| `0x180239bec` | fluff | flirt | `__std_fs_create_hard_link` | MSVC C++ standard library |
| `0x180239c18` | fluff | flirt | `__std_fs_directory_iterator_advance` | MSVC C++ standard library |
| `0x180239c38` | fluff | flirt | `__std_fs_directory_iterator_close` | MSVC C++ standard library |
| `0x18023a318` | fluff | flirt | `__std_fs_remove` | MSVC C++ standard library |
| `0x18023a420` | fluff | flirt | `__std_fs_rename` | MSVC C++ standard library |
| `0x18023a4f8` | fluff | flirt | `__std_fs_set_current_path` | MSVC C++ standard library |
| `0x18023a5c0` | fluff | flirt | `__std_fs_space` | MSVC C++ standard library |
| `0x18023a898` | fluff | flirt | `__scrt_initialize_thread_safe_statics(void)` | MSVC C runtime |
| `0x18023a968` | fluff | flirt | `__scrt_initialize_thread_safe_statics_platform_specific(void)` | MSVC C runtime |
| `0x18023aa20` | fluff | flirt | `__scrt_uninitialize_thread_safe_statics(void)` | MSVC C runtime |
| `0x18023acf4` | fluff | flirt | `ArrayUnwindFilter(_EXCEPTION_POINTERS *)` | MSVC C runtime |
| `0x18023af68` | fluff | flirt | `operator new(unsigned __int64)` | MSVC C runtime |
| `0x18023b0d8` | fluff | flirt | `__scrt_dllmain_crt_thread_attach` | MSVC C runtime |
| `0x18023b100` | fluff | flirt | `__scrt_dllmain_crt_thread_detach` | MSVC C runtime |
| `0x18023b208` | fluff | flirt | `__scrt_initialize_onexit_tables` | MSVC C runtime |
| `0x18023b434` | fluff | flirt | `__report_gsfailure` | MSVC C runtime |
| `0x18023b508` | fluff | flirt | `__report_rangecheckfailure` | MSVC C runtime |
| `0x18023b51c` | fluff | flirt | `__report_securityfailure` | MSVC C runtime |
| `0x18023b5b8` | fluff | flirt | `__report_securityfailureEx` | MSVC C runtime |
| `0x18023b7b4` | fluff | flirt | `dllmain_crt_dispatch(HINSTANCE__ * const,ulong,void * const)` | MSVC C runtime |
| `0x18023b804` | fluff | flirt | `dllmain_crt_process_attach(HINSTANCE__ * const,void * const)` | MSVC C runtime |
| `0x18023b91c` | fluff | flirt | `dllmain_crt_process_detach(bool)` | MSVC C runtime |
| `0x18023bb88` | fluff | flirt | `__castguard_check_failure_os_handled` | MSVC C runtime |
| `0x18023bc5c` | fluff | flirt | `__castguard_slow_path_check_os_handled` | MSVC C runtime |
| `0x18023bcec` | fluff | flirt | `__crt_debugger_hook` | MSVC C runtime |
| `0x18023bcf4` | fluff | flirt | `__scrt_fastfail` | MSVC C runtime |
| `0x18023bef0` | fluff | flirt | `__scrt_unhandled_exception_filter` | MSVC C runtime |
| `0x18023bf4c` | fluff | flirt | `__scrt_throw_std_bad_alloc(void)` | MSVC C runtime |
| `0x18023bf6c` | fluff | flirt | `__scrt_throw_std_bad_array_new_length(void)` | MSVC C runtime |
| `0x18023c14c` | fluff | flirt | `__get_entropy` | MSVC C runtime |
| `0x18023c288` | fluff | flirt | `__scrt_initialize_type_info(void)` | MSVC C runtime |
| `0x18023c2a4` | fluff | flirt | `__scrt_initialize_default_local_stdio_options` | MSVC C runtime |
| `0x18023c2c8` | fluff | flirt | `_RTC_Initialize` | MSVC C runtime |
| `0x18023c304` | fluff | flirt | `_RTC_Terminate` | MSVC C runtime |

### lib:json+config (49)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x180140440` | fluff | string | uses 'additional six characters expected to parse unicode surrogat', 'expecting another \u token to begin the second half of a uni' | jsoncpp |
| `0x1801406d0` | fluff | string | uses 'additional six characters expected to parse unicode surrogat', 'expecting another \u token to begin the second half of a uni' | jsoncpp |
| `0x180140960` | fluff | string | uses 'Bad unicode escape sequence in string: four digits expected.', 'Bad unicode escape sequence in string: hexadecimal digit exp' | jsoncpp |
| `0x180140b80` | fluff | string | uses 'Bad unicode escape sequence in string: four digits expected.', 'Bad unicode escape sequence in string: hexadecimal digit exp' | jsoncpp |
| `0x180140dd0` | fluff | string | uses 'in Json::Value::duplicateAndPrefixStringValue(): Failed to a', 'in Json::Value::duplicateAndPrefixStringValue(): length too' | jsoncpp |
| `0x180140eb0` | fluff | string | uses 'in Json::Value::duplicateStringValue(): Failed to allocate s' | jsoncpp |
| `0x1801411f0` | fluff | string | uses 'in Json::Value::find(key, end, found): requires objectValue' | jsoncpp |
| `0x180141790` | fluff | string | uses 'for detail.' | jsoncpp |
| `0x180142010` | fluff | string | uses 'for detail.' | jsoncpp |
| `0x1801429d0` | fluff | string | uses 'Line %d, Column %d' | jsoncpp |
| `0x180142b00` | fluff | string | uses 'Line %d, Column %d' | jsoncpp |
| `0x180143390` | fluff | string | uses 'in Json::Value::getMemberNames(), value must be objectValue' | jsoncpp |
| `0x1801439a0` | fluff | string | uses 'collectComments', 'allowComments' | jsoncpp |
| `0x180144120` | fluff | string | uses 'indentation', 'commentStyle' | jsoncpp |
| `0x180144df0` | fluff | string | uses 'in Json::Value::operator[](int index) const: index cannot be' | jsoncpp |
| `0x180145020` | fluff | string | uses 'in Json::Value::operator[](int index) const: index cannot be' | jsoncpp |
| `0x180145250` | fluff | string | uses 'in Json::Value::operator[](int index) const: index cannot be' | jsoncpp |
| `0x180145d00` | fluff | vtable | virtual method of `Json::CharReaderBuilder` | jsoncpp |
| `0x180146250` | fluff | vtable | virtual method of `Json::StreamWriterBuilder` | jsoncpp |
| `0x180146910` | fluff | vtable | virtual method of `Json::OurCharReader` | jsoncpp |
| `0x1801469c0` | fluff | string | uses 'Extra non-whitespace after JSON value.', 'A valid JSON document must be either an array or an object v' | jsoncpp |
| `0x180146e40` | fluff | string | uses 'A valid JSON document must be either an array or an object v' | jsoncpp |
| `0x180147ba0` | fluff | string | uses 'Missing ',' or ']' in array declaration' | jsoncpp |
| `0x180147fe0` | fluff | string | uses 'Missing ',' or ']' in array declaration' | jsoncpp |
| `0x180148b40` | fluff | string | uses 'false', '%%.%dg' | jsoncpp |
| `0x1801499a0` | fluff | string | uses 'false', '%%.%dg' | jsoncpp |
| `0x18014a610` | fluff | string | uses 'nfinity' | jsoncpp |
| `0x18014af90` | fluff | string | uses 'Syntax error: value, object or array expected.', 'Exceeded stackLimit in readValue().' | jsoncpp |
| `0x18014b890` | fluff | string | uses 'Syntax error: value, object or array expected.', 'Exceeded stackLimit in readValue().' | jsoncpp |
| `0x18014c620` | fluff | string | uses 'in Json::Value::removeMember(): requires objectValue' | jsoncpp |
| `0x18014cc00` | fluff | string | uses 'in Json::Value::clear(): requires complex value', 'in Json::Value::resize(): requires arrayValue' | jsoncpp |
| `0x18014cfa0` | fluff | string | uses 'in Json::Value::resolveReference(key, end): requires objectV' | jsoncpp |
| `0x18014d260` | fluff | string | uses 'in Json::Value::resolveReference(): requires objectValue' | jsoncpp |
| `0x18014d540` | fluff | string | uses 'in Json::Value::setComment(): Comments must start with /', 'assert json failed' | jsoncpp |
| `0x18014d700` | fluff | string | uses 'collectComments', 'allowComments' | jsoncpp |
| `0x18014da10` | fluff | string | uses 'commentStyle', 'indentation' | jsoncpp |
| `0x18014ded0` | fluff | string | uses 'allowComments', 'strictRoot' | jsoncpp |
| `0x1801500f0` | fluff | string | uses '%%.%dg', '-Infinity' | jsoncpp |
| `0x180150440` | fluff | string | uses 'false' | jsoncpp |
| `0x1801504a0` | fluff | vtable | virtual method of `Json::Exception`, `Json::LogicError` | jsoncpp |
| `0x1801504b0` | fluff | vtable | virtual method of `Json::BuiltStyledStreamWriter` | jsoncpp |
| `0x180150590` | fluff | vtable | virtual method of `Json::FastWriter` | jsoncpp |
| `0x180150710` | fluff | vtable | virtual method of `Json::StyledWriter` | jsoncpp |
| `0x1801519a0` | fluff | flirt | `construct_environment_block<wchar_t>(wchar_t const * const * const,wchar_t * * const)` | MSVC C runtime |
| `0x180152590` | fluff | string | uses 'false' | jsoncpp |
| `0x180152d90` | fluff | string | uses '%%.%dg', '-1e+9999' | jsoncpp |
| `0x1801534e0` | fluff | string | uses '%%.%dg', '-1e+9999' | jsoncpp |
| `0x180153d70` | fluff | string | uses '%%.%dg', '-1e+9999' | jsoncpp |
| `0x180154760` | fluff | flirt | `fwprintf` | MSVC C runtime |

### lib:misc (4)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x180045ac0` | fluff | flirt | `__crt_internal_free_policy::operator()<wchar_t>(wchar_t const * const)` | MSVC C runtime |
| `0x18024590f` | fluff | flirt | `int ``eh vector destructor iterator'(void *,unsigned __int64,unsigned __int64,void (*)(void *))'::`1'::fin$0` | MSVC C runtime |
| `0x18024593b` | fluff | flirt | `int `__ArrayUnwind(void *,unsigned __int64,unsigned __int64,void (*)(void *))'::`1'::filt$0` | MSVC C runtime |
| `0x1802459f6` | fluff | flirt | `int `dllmain_dispatch(HINSTANCE__ * const,ulong,void * const)'::`1'::filt$0` | MSVC C runtime |

### libpad (2)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x1801ca480` | core | flirt | `std::_Tree_val<std::_Tree_simple_types<std::pair<void * const,`anonymous namespace'::_Mutex_count_pair>>>::...` | MSVC C++ standard library |
| `0x1801ca4e0` | core | flirt | `std::_Tree_val<std::_Tree_simple_types<std::pair<void * const,`anonymous namespace'::_Mutex_count_pair>>>::...` | MSVC C++ standard library |

### map+steamvr (28)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x18012cdd0` | fluff | string | uses 'steam://store/2580190' | lib |
| `0x18012ce30` | fluff | string | uses 'steam://rungameid/2580190' | lib |
| `0x18012d600` | fluff | flirt | `std::basic_stringstream<char,std::char_traits<char>,alloc>::~basic_stringstream<char,std::char_traits<char>...` | MSVC C++ standard library |
| `0x180139320` | fluff | string | uses 'ConstIterator to Iterator should never be allowed.' | lib |
| `0x18013a880` | fluff | string | uses 'in Json::Value::operator[](int index): index cannot be negat' | lib |
| `0x18013a900` | fluff | string | uses 'in Json::Value::operator[](ArrayIndex): requires arrayValue' | lib |
| `0x18013abb0` | fluff | string | uses 'in Json::Value::operator[](int index) const: index cannot be' | lib |
| `0x18013ac30` | fluff | string | uses 'in Json::Value::operator[](ArrayIndex)const: requires arrayV' | lib |
| `0x18013b480` | fluff | vtable | virtual method of `Json::BuiltStyledStreamWriter` | lib |
| `0x18013b540` | fluff | vtable | virtual method of `Json::Exception` | lib |
| `0x18013b680` | fluff | vtable | virtual method of `Json::LogicError` | lib |
| `0x18013b710` | fluff | vtable | virtual method of `Json::RuntimeError` | lib |
| `0x18013c890` | core | flirt | `std::_Tree_val<std::_Tree_simple_types<std::pair<void * const,`anonymous namespace'::_Mutex_count_pair>>>::...` | MSVC C++ standard library |
| `0x18013c9b0` | core | flirt | `std::_Tree_val<std::_Tree_simple_types<std::pair<void * const,`anonymous namespace'::_Mutex_count_pair>>>::...` | MSVC C++ standard library |
| `0x18013cbf0` | fluff | string | uses 'deque<T> too long' | lib |
| `0x18013cc10` | fluff | string | uses 'deque<T> too long' | lib |
| `0x18013cc30` | fluff | string | uses 'deque<T> too long' | lib |
| `0x18013cc50` | fluff | flirt | `std::vector<void *,alloc>::_Xlen(void)` | MSVC C++ standard library |
| `0x18013cc70` | fluff | flirt | `std::vector<void *,alloc>::_Xlen(void)` | MSVC C++ standard library |
| `0x18013cc90` | fluff | flirt | `std::vector<void *,alloc>::_Xlen(void)` | MSVC C++ standard library |
| `0x18013ccb0` | fluff | flirt | `std::vector<void *,alloc>::_Xlen(void)` | MSVC C++ standard library |
| `0x18013ccd0` | fluff | flirt | `std::vector<void *,alloc>::_Xlen(void)` | MSVC C++ standard library |
| `0x18013d2c0` | fluff | string | uses 'in Json::Value::asCString(): requires stringValue' | lib |
| `0x18013dbd0` | fluff | string | uses 'in Json::Value::clear(): requires complex value' | lib |
| `0x18013e390` | fluff | string | uses 'Unable to parse token length', '' is not a number.' | lib |
| `0x18013e970` | fluff | string | uses '' is not a number.' | lib |
| `0x18013f7a0` | fluff | string | uses 'Bad escape sequence in string', 'Empty escape sequence in string' | lib |
| `0x18013ff10` | fluff | string | uses 'Bad escape sequence in string', 'Empty escape sequence in string' | lib |

### math+threads (1)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x18016b080` | fluff | string | uses 'System::abort %s:%d' | OpenCV-derived math / `Framework` threads |

### reloc (2)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x1801d8850` | fluff | flirt | `std::bad_alloc::bad_alloc(void)` | MSVC C++ standard library |
| `0x1801d88b0` | fluff | flirt | `CIP<IBindCtx,&_GUID const IID_IBindCtx>::~CIP<IBindCtx,&_GUID const IID_IBindCtx>(void)` | MSVC C runtime |

### room (2)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x18011e320` | fluff | vtable | virtual method of `Json::StyledWriter` | trace |
| `0x18011e4f0` | fluff | flirt | `std::vector<void *,alloc>::_Xlen(void)` | MSVC C++ standard library |

### sense (4)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x18011b5d0` | fluff | vtable | virtual method of `std::basic_filebuf<char,struct std::char_traits<char>>` | MSVC C++ standard library |
| `0x18011b6b0` | fluff | vtable | virtual method of `std::basic_filebuf<char,struct std::char_traits<char>>` | MSVC C++ standard library |
| `0x18011b7c0` | fluff | vtable | virtual method of `std::basic_filebuf<char,struct std::char_traits<char>>` | MSVC C++ standard library |
| `0x18011ba70` | fluff | vtable | virtual method of `std::basic_filebuf<char,struct std::char_traits<char>>` | MSVC C++ standard library |

### steamvr (14)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x1801a3a70` | fluff | string | uses 'IVRServerDriverHost_006', 'IVRSettings_003' | app |
| `0x1801a3bc0` | fluff | string | uses 'IVRServerDriverHost_006' | app |
| `0x1801a3dc0` | fluff | vtable | virtual method of `std::_Func_impl_no_alloc<class <lambda_39f31fd7cd9f786225f1186476399710>,void,>` | MSVC C++ standard library |
| `0x1801a3dd0` | fluff | vtable | virtual method of `std::_Func_impl_no_alloc<class <lambda_3e478eb0b43a3f878d0d086f3877a967>,void,>` | MSVC C++ standard library |
| `0x1801a3de0` | fluff | vtable | virtual method of `std::_Func_impl_no_alloc<class <lambda_50409c0145798f44e8929f95d19bb712>,void,>` | MSVC C++ standard library |
| `0x1801a3df0` | fluff | vtable | virtual method of `std::_Func_impl_no_alloc<class <lambda_9d26a79a3759d3f6c2d1a2b5946c2255>,void,>` | MSVC C++ standard library |
| `0x1801a3e00` | fluff | vtable | virtual method of `std::_Func_impl_no_alloc<class <lambda_a33eb66e4ac76a8f9779fc8c7214fb0d>,void,>` | MSVC C++ standard library |
| `0x1801a3e10` | fluff | vtable | virtual method of `std::_Func_impl_no_alloc<class <lambda_b525993edc2de821a425e55a3945a903>,void,>` | MSVC C++ standard library |
| `0x1801a3e20` | fluff | vtable | virtual method of `std::_Func_impl_no_alloc<class <lambda_e09f9d417306d925894ee1d80722e651>,void,>` | MSVC C++ standard library |
| `0x1801a3e30` | fluff | vtable | virtual method of `std::_Func_impl_no_alloc<class <lambda_f086b1016d85400456618d91b98f892d>,void,>` | MSVC C++ standard library |
| `0x1801a3e40` | fluff | vtable | virtual method of `std::_Func_impl_no_alloc<class <lambda_fd920c6c5760b3252b372c64e529a06e>,void,>` | MSVC C++ standard library |
| `0x1801a4370` | fluff | flirt | `std::vector<void *,alloc>::_Xlen(void)` | MSVC C++ standard library |
| `0x1801abde0` | fluff | string | uses 'deque<T> too long' | lib |
| `0x1801abe00` | fluff | string | uses 'deque<T> too long' | lib |

### tracking (1)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x180048450` | fluff | string | uses 'UpdateMultiCamBootStats : %d frames', 'device %d:accum: %4d(%3.1f %%), current %4d(%3.1f %%) [singl' | debug |
<!-- gen:library:end -->
