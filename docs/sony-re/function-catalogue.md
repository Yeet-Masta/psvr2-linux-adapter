# Function catalogue

Per-function notes for functions that the topic files do not describe. Parts D and E are generated (`scripts/sony-re/triage/trivial.py`, `libcat.py`); only their first column counts as documentation.

Parts:

- [Part A: ctrl-boot](#part-a-ctrl-boot)
- [Part B: ctrl-tracker UKF internals](#part-b-ctrl-tracker-ukf-internals)
- [Part C: CLAPACK](#part-c-clapack)
- [Part D: Trivial functions](#part-d-trivial-functions)
- [Part E: Library and runtime code](#part-e-library-and-runtime-code)
- [Part F: libpad internals](#part-f-libpad-internals)
- [Part G: Headset USB glue, hand skeleton and jsoncpp](#part-g-headset-usb-glue-hand-skeleton-and-jsoncpp)
- [Part H: ShareManager and ConfigManager glue](#part-h-sharemanager-and-configmanager-glue)

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

### Gap fill 1: math helpers, blob stores, sort helpers (`0x180044520`–`0x1800af510`)

Most rows here have no direct caller in the inventory: they are reached
through vtables or function-pointer tables, or are out-of-line copies of
inline operators that the compiler kept. "Vec2" = two floats packed in the
low half of an SSE register, "Vec4" = four floats, "M34" = 3×4 rigid
transform (rows R | t), "M44" = 4×4.

| Function | Lines | Role |
|---|---|---|
| `0x180044520` | 13 | Vec2 splat: copy one 8-byte pair into both halves [sure] |
| `0x180044550` | 14 | Vec2 broadcast lane x to all four lanes [sure] |
| `0x180044580` | 14 | Vec2 broadcast lane y to all four lanes [sure] |
| `0x1800445c0` | 19 | Vec4 broadcast lane `i & 3` [sure] |
| `0x180044600` | 18 | Vec2 add [sure] |
| `0x180044640` | 17 | Vec2 subtract [sure] |
| `0x180044680` | 16 | Vec2 times the x lane of a Vec4 [sure] |
| `0x1800446b0` | 20 | Vec2 times scalar [sure] |
| `0x1800446e0` | 27 | Vec2 divided by scalar, through `rcp` plus two Newton steps [sure] |
| `0x180044740` | 20 | scalar times Vec2 (argument order swapped) [sure] |
| `0x180044770` | 18 | Vec2 component-wise multiply [sure] |
| `0x1800447b0` | 29 | Vec2 dot product, result in all lanes [sure] |
| `0x180044800` | 25 | Vec2 squared length [sure] |
| `0x180044880` | 10 | build Vec4 from a Vec2 and a scalar in z (w = scalar too) [sure] |
| `0x180044e40` | 54 | 3×3 rotation matrix from a unit quaternion (x, y, z, w) [sure] |
| `0x180044f10` | 38 | M34 rotate a vector: `R·v` (no translation) [sure] |
| `0x180045140` | 38 | duplicate of `0x180044f10` (separate COMDAT) [sure] |
| `0x180045190` | 45 | M34 transform a point: `R·p + t` [sure] |
| `0x180045330` | 42 | rigid inverse of an M34: `Rᵀ`, `−Rᵀ·t` [sure] |
| `0x180045ba0` | 21 | Vec2 squared length returned in a register [sure] |
| `0x180045c40` | 44 | Vec4 normalise (`v / ‖v‖`, scalar `sqrt`) [sure] |
| `0x180045cb0` | 52 | quaternion normalise, then lane reorder into (x, y, z, w) [sure] |
| `0x180045f80` | 25 | M34 from a quaternion and a translation (rotation by `0x180045d50`, t copied to row 3) [sure] |
| `0x180046040` | 58 | rigid inverse of an M34 returned as a 64-byte value (same math as `0x180045330`) [sure] |
| `0x1800460f0` | 44 | rigid inverse of an M34 written to an out-parameter; used by `0x1800e2a00` [sure] |
| `0x1800461a0` | 33 | M34 product `a·b` through `0x1800451e0`, returned by value [sure] |
| `0x1800461d0` | 45 | duplicate of `0x180045190` (`R·p + t`) [sure] |
| `0x1800464b0` | 414 | resize every array of an 11-array blob store to `n` items (three of 8-byte items, six of 4-byte items, two of 2-byte items), reallocating through each RnD::Vector's allocator; called by `0x18004b0a0`, `0x1800a0480`, `0x1800a09a0`, `0x1800d9800` [sure] |
| `0x18009dcc0` | 46 | Vec4 lane-wise `sqrt` as `x·rsqrt(x)` with two Newton steps, 0 where x = 0 [sure] |
| `0x18009df40` | 46 | duplicate of `0x18009dcc0` [sure] |
| `0x18009e590` | 72 | Vec3 safe normalise: below a tiny squared length it returns a fixed default vector [sure for the form] |
| `0x18009e660` | 59 | Vec3 normalise with `rsqrt` and two Newton steps [sure] |
| `0x18009e7c0` | 62 | Vec4 normalise with `rsqrt` and two Newton steps [sure] |
| `0x18009e840` | 123 | quaternion slerp: flips the sign for a negative dot product, uses `acos`/`sin` (`0x18009e170`, `0x18009e2a0`), and falls back to a linear blend above a dot threshold (5th argument); callers `0x1800b0980`, `0x1800da320` [likely] |
| `0x18009e9d0` | 21 | scalar-argument wrapper of the slerp `0x18009e840` [sure] |
| `0x18009ea40` | 66 | M44 from a unit quaternion, translation 0, last row (0, 0, 0, 1) [sure] |
| `0x18009ed40` | 488 | 8-lane (AVX) body of the camera model map behind `0x18009eb50`: projective 3×3 step with a divide by the third row, then the distortion polynomial, for 8 points at once [likely] |
| `0x18009f2b0` | 108 | 8-lane polynomial in r² (Horner on coefficients at `+0x20..+0x38` of the model), with an optional second branch selected by the 4th argument; distortion radial term [likely] |
| `0x18009f3b0` | 10 | `RnD::MersenneTwister` constructor: sets the vtable, then seeds through `0x18009f3e0` [sure] |
| `0x18009f6a0` | 48 | Mersenne Twister state regeneration (the 624-word twist with `0x9908b0df`); called by the readers `0x18009f530`, `0x1800a7c50` [sure] |
| `0x18009f850` | 41 | Vec3 normalise with scalar `sqrt` [sure] |
| `0x18009f940` | 24 | count set bits in a 64-bit mask over the first `n` bits; used by `0x1800e2a00` [sure] |
| `0x18009f9a0` | 99 | constructor of a per-camera state block: zeroes the counters, index `+0x28` = −1, 24 entries starting at `+0x278` set to (7.78, −1.0, …) defaults; called by `0x18009fd80` [sure for the writes, guess for the role] |
| `0x1800a2000` | 92 | per-controller reset over `+0x15c40` controllers: clears flag `+0x3358` and sets `+0x35e0` to ≈ 1e5 (`0x47c34ff3`, "no distance yet") in each camera-manager controller block [sure for the writes] |
| `0x1800a3780` | 31 | clear byte `+0x4bc` of each controller in every camera manager (stride 0x4ce0) [sure] |
| `0x1800a5600` | 20 | run the boot worker `0x1800a56b0` twice per item for `n` items: job type 1 then job type 2 [sure] |
| `0x1800a5d60` | 31 | copy each controller's per-camera blob store into the tracker (`0x1800a0bf0`, tracker `+0xa430 + 1520·(4·ctrl + cam)`) [sure] |
| `0x1800a6610` | 163 | filter a list of 2D points against a centre and radius (squared distance below `r²`), compacting the kept ones; callers `0x1800a4fb0`, `0x1800a8770` [likely] |
| `0x1800a6820` | 40 | load `./ConfigFile0.trk` into memory (`fopen`, `fseek`/`ftell`, `malloc`) and hand it to the parser `0x1800a6940` [sure] |
| `0x1800a9db0` | 77 | thread-safe static init of the shared `RnD::DefaultAllocator` (`0x1806167a0`), then build an object that uses it [sure] |
| `0x1800a9fe0` | 33 | release five RnD::Vectors of an object through their allocators and zero them (destructor body) [sure] |
| `0x1800aa240` | 12 | obfuscated-string decoder: XOR 32 bytes with a fixed key [sure] |
| `0x1800aa320` | 12 | same decoder, another string [sure] |
| `0x1800aa550` | 12 | same decoder [sure] |
| `0x1800aa5e0` | 12 | same decoder [sure] |
| `0x1800aa6c0` | 12 | same decoder [sure] |
| `0x1800aa750` | 12 | same decoder [sure] |
| `0x1800aa7e0` | 12 | same decoder [sure] |
| `0x1800aa870` | 12 | same decoder [sure] |
| `0x1800ab3a0` | 12 | allocate and zero a 0x17004-byte block (tracker work area) [sure] |
| `0x1800ab590` | 20 | sort an RnD::Vector of 8-byte items with `0x1800ad2b0` [sure] |
| `0x1800ac630` | 58 | RnD::Vector constructor for `n` 8-byte items from a given allocator [sure] |
| `0x1800ac7c0` | 14 | sort range `[a, b)` with `0x1800acfb0` [sure] |
| `0x1800ac810` | 14 | sort range `[a, b)` with `0x1800ad2b0` [sure] |
| `0x1800adad0` | 51 | find the first (float, float) pair equal to a key in a range (std::find instance) [likely] |
| `0x1800adb80` | 49 | std::make_heap on 16-byte items keyed by the second float [sure] |
| `0x1800adc80` | 35 | std::sort_heap on 8-byte items (pop loop through `0x1800ae860`) [sure] |
| `0x1800ae030` | 51 | duplicate of `0x1800adad0` for another item type [sure] |
| `0x1800ae100` | 49 | duplicate of `0x1800adb80` [sure] |
| `0x1800ae1f0` | 35 | std::sort_heap, other comparator (`0x1800aeb30`) [sure] |
| `0x1800ae930` | 25 | std::pop_heap (`0x1800ae860`) [sure] |
| `0x1800ae9b0` | 39 | introsort partition step (median-of-three or median-of-nine above 40 items) [sure] |
| `0x1800aec00` | 25 | std::pop_heap, other comparator (`0x1800aeb30`) [sure] |
| `0x1800aec70` | 39 | introsort partition step, other comparator [sure] |
| `0x1800aefb0` | 13 | heap: move the first item to the output and sift down (`0x1800ae860`) [sure] |
| `0x1800af0e0` | 13 | same, comparator `0x1800aeb30` [sure] |
| `0x1800af320` | 49 | M34 rotation about x by an angle (sincos `0x18009e2a0`) [sure] |
| `0x1800af3d0` | 41 | M34 rotation about y by an angle [sure] |
| `0x1800af470` | 38 | M34 rotation about z by a vector-lane angle [sure] |
| `0x1800af510` | 44 | M34 rotation about x by a vector-lane angle [sure] |

### Gap fill 2: rig poses, ray helpers, background-model containers (`0x1800b0250`–`0x1800c02d0`)

The background model itself is `controller-tracking.md` part E. The rows
here are its helpers and the std::vector code generated for its record
types (landmarks are 320 bytes, view candidates 288 bytes).

| Function | Lines | Role |
|---|---|---|
| `0x1800b0250` | 18 | fill the 4-camera built-in rig table at `0x1805f32e0` for a variant: `0x1800af650(variant, cam)` per camera [sure] |
| `0x1800b0c40` | 48 | store a pose (M34) and its quaternion (`0x180044bc0`) side by side in one record; used by `0x1800b0980` [sure] |
| `0x1800b16d0` | 17 | Vec2 `a −= b` in place [sure] |
| `0x1800b1710` | 25 | Vec2 "all lanes a < b" mask [sure] |
| `0x1800b1770` | 25 | Vec2 "all lanes a ≤ b" mask (operands swapped) [sure] |
| `0x1800b1940` | 13 | Vec4 from two Vec2 halves [sure] |
| `0x1800b1bd0` | 62 | Vec4 set lane `i & 3` to a scalar (`insertps`); callers `0x1800b24b0`, `0x180103580` [sure] |
| `0x1800b1d60` | 21 | read lane `j & 3` of row `i` of a matrix and broadcast it [sure] |
| `0x1800b1ee0` | 28 | 12-float `a −= b` (M34 subtract) [sure] |
| `0x1800b2000` | 18 | Vec4 lane-wise `acos` wrapper (`0x18009e170`) [sure] |
| `0x1800b21a0` | 109 | closest points of two rays `p1 + s·d1`, `p2 + t·d2`: fails when the rays are near parallel (det < 1e-6) or either point lies behind its origin (s ≤ 0 or t ≤ 0); returns both points [sure] |
| `0x1800b2370` | 83 | build a rotation from an axis and an angle: normalise the axis, then half-angle sin/cos; returns false for a zero axis [likely] |
| `0x1800b3d00` | 46 | landmark prune (background step 5): drop a landmark when either camera's miss count exceeds 300 (hit count ≥ 10) or 40 (otherwise) [sure] |
| `0x1800b3e00` | 17 | `RnD::Background` constructor: vtable, three views built by `0x1800b6d40` (destructor `0x1800b3f50`), view ids 0/1/2 [sure] |
| `0x1800b83c0` | 111 | as `0x1800b21a0`, and also returns the two ray parameters s and t [sure] |
| `0x1800b9ce0` | 88 | prune a list of 288-byte view records (`+0x90`): drop flagged entries whose counter passed the limits at `0x180351778` / `0x18035177a`, shifting the rest down [likely] |
| `0x1800ba930` | 12 | erase one 320-byte landmark (memmove the tail down) [sure] |
| `0x1800babd0` | 25 | free the buffer of a vector of 64-byte items (MSVC aligned-delete path); callers `0x1800b3f50`, `0x18023d0a0` [sure] |
| `0x1800baf90` | 14 | push_back of a 288-byte record (grow through `0x1800bbfe0`, copy `0x1800bc490`) [sure] |
| `0x1800bafd0` | 14 | same push_back, second instance [sure] |
| `0x1800bb010` | 10 | push_back without the capacity check (caller checked) [sure] |
| `0x1800bb040` | 25 | free a vector of 32-byte items; caller `0x1800b6f70` [sure] |
| `0x1800bb260` | 25 | free a vector of 64-byte items [sure] |
| `0x1800bb390` | 25 | free a vector of 32-byte items [sure] |
| `0x1800bb570` | 59 | RnD::Vector constructor for `n` items from a given allocator [sure] |
| `0x1800bb7f0` | 9 | trivially-copyable move of `[src, end)` to `dst`, returns the new end [sure] |
| `0x1800bbfe0` | 97 | vector<288-byte record> reallocate-and-insert (growth ×1.5, `length_error` at 0xe38e38e38e38e3 items) [sure] |
| `0x1800bc700` | 25 | free a vector of 64-byte items (other instance) [sure] |
| `0x1800bc840` | 32 | allocate `n` 320-byte records (aligned new above 4 KiB) [sure] |
| `0x1800bc9d0` | 17 | copy a range of 8-byte items [sure] |
| `0x1800bca60` | 32 | allocate `n` 8-byte items [sure] |
| `0x1800bcb20` | 25 | free a vector of 32-byte items (other instance) [sure] |
| `0x1800bcc10` | 21 | copy-construct a range of 288-byte records [sure] |
| `0x1800bcc60` | 95 | move a range of 288-byte records field by field (used by the grow path `0x1800bbfe0`) [sure] |
| `0x1800bce20` | 32 | allocate `n` 288-byte records [sure] |
| `0x1800bcf70` | 15 | copy-construct a range of 288-byte records (second instance) [sure] |
| `0x1800bd060` | 9 | memmove range helper (duplicate of `0x1800bb7f0`) [sure] |
| `0x1800bd0f0` | 64 | copy a range of 320-byte landmarks (two 128-byte blocks plus the tail per item); callers `0x1800bb900`, `0x1800bc7f0`, `0x1800bc810`, `0x1800bceb0` [sure] |
| `0x1800bd1d0` | 12 | byte size of `n` 320-byte items, `bad_array_new_length` on overflow [sure] |
| `0x1800bd240` | 12 | byte size of `n` 8-byte items, same check [sure] |
| `0x1800bd430` | 12 | byte size of `n` 288-byte items, same check [sure] |
| `0x1800bd460` | 15 | uninitialised copy of 288-byte records, returns the new end [sure] |
| `0x1800bd7c0` | 10 | construct one 288-byte record at the end and advance [sure] |
| `0x1800bfec0` | 18 | two u32 → two doubles (the `0x4330…` bias trick) [sure] |
| `0x1800bff60` | 9 | copy one 8-byte value [sure] |
| `0x1800c0270` | 21 | Vec2 select by a bool: `b ? y : x` [sure] |
| `0x1800c02d0` | 24 | Vec2 select by a lane mask [sure] |

### Gap fill 3: RANSAC sampler object and boot helpers (`0x1800c0640`–`0x1800cea10`)

The RANSAC object used by the boot (`controller-tracking.md` §B.2.19) keeps
its Mersenne Twister at `+0x1f8`, the model point list at `+0xd90`
(16-byte points), the observation list at `+0xda0` (8-byte points), the
candidate list at `+0xe18` (48-byte entries, index at `+0x20`), a 4-wide
neighbour table at `+0xee8`, the sampling mode at `+0xf10` and an
optional filter object at `+0xf08`. Random draws use
`u32 · 2⁻³² · n`.

| Function | Lines | Role |
|---|---|---|
| `0x1800c0640` | 150 | 8-lane nearest-point search: for each stored 2D point, distance to 8 query points, keep the smallest three distances and their indices per lane (initial value from `0x18031a660`) [likely] |
| `0x1800c0bf0` | 178 | 8-lane transform of 8 model points by an M34 (7th argument), then projection and comparison with the observations; callers via `0x1800c08a0` [likely] |
| `0x1800c1b10` | 57 | gather one sample (three model points, three observations by the indices in the sample record) and hand them to the P3P solver `0x1800f43e0` [sure] |
| `0x1800c1c60` | 177 | start of a RANSAC run: clear the result fields, refresh the point count, recompute the iteration bound (`0x1800c8b50`), then score hypotheses through `0x1800c2080` [likely] |
| `0x1800c1ff0` | 20 | hypothesis gate: if a filter object is set (`+0xf08`), ask its slot 1 first; on reject write 0 and return false, else score with `0x1800c2080` [sure] |
| `0x1800c3fe0` | 132 | sample generator by mode (`+0xf10`): mode 1 draws a candidate and two of its neighbours from the table, other modes draw three candidates independently; needs ≥ 3 observations; callers `0x1800c2960`, `0x1800c3f60` [sure] |
| `0x1800c44a0` | 65 | sample generator: three independent random candidates plus random model points [sure] |
| `0x1800c46a0` | 85 | deterministic sample from stored counters (`+0xf50` base, step and stride), so the triplets are walked in order instead of drawn [likely] |
| `0x1800c5030` | 37 | three random integers in the given ranges from one 3-draw block [sure] |
| `0x1800c50f0` | 58 | six random integers in four given ranges from one 6-draw block [sure] |
| `0x1800c5620` | 24 | in modes 6 and 7, reset the sample state: eight ints at `+0xf80` to −1 (or 0), counters `+0xf60..+0xf70` and `+0xf6c` [sure] |
| `0x1800c5690` | 158 | build a three-point sample: copy the three observation records, take the mean of the three model points and rotate it by the matrix at `+0xf20`; caller `0x1800c5ac0` [likely] |
| `0x1800c82e0` | 18 | release two RnD::Vectors of an object (destructor body); caller `0x18023d180` [sure] |
| `0x1800c83c0` | 36 | fill a 256-entry u32 table (1 KiB) with a value, memset when 0 [sure] |
| `0x1800c8550` | 59 | fill an 8-float record with a value (zero when 0); `0x1800c5620` uses it with −1 [sure] |
| `0x1800c8650` | 33 | take `k` bits at bit offset `o` from a 256-bit mask and spread them to a per-lane mask (bit i → lane i) [sure] |
| `0x1800c8840` | 70 | build a 48-byte parameter record: 16-byte header copied, then thresholds from the float arguments; uses `0x18009f3e0` to seed [likely] |
| `0x1800c8c50` | 61 | RnD::Vector constructor holding one 48-byte item copied from the argument [likely] |
| `0x1800c94e0` | 12 | uniform random int in `[0, n)` from the twister [sure] |
| `0x1800c9840` | 34 | fill 256 u32 with a value (second instance of `0x1800c83c0`) [sure] |
| `0x1800c9900` | 19 | `fill_n` of u32 [sure] |
| `0x1800c99f0` | 19 | `fill_n` of u32 (duplicate) [sure] |
| `0x1800cb9c0` | 13 | constructor of an array block: 16 entries of 0x70 bytes (`0x1800b0950`) at `+0`, then 0x1f0-byte entries at `+0x700`; callers `0x180051420`, `0x180051430`, `0x1800a1000`, `0x1800b0950` [sure for the layout] |
| `0x1800cba70` | 53 | reset four rotation slots to identity (from `sin(0)`, `cos(0)`), with zero offsets; caller `0x18009d4d0` [sure] |
| `0x1800cbcf0` | 42 | compose a rig transform for a camera pair (`0x1800cbd70`) with a pose (`0x1800451e0`) [sure] |
| `0x1800cbe50` | 30 | for each controller, predict the next-frame pose (`0x1800ceaa0`) into the tracker at `+0x9200` from the ring at `+0x8dc0` [sure] |
| `0x1800cbf60` | 54 | occupancy grid: write the bits of a rectangle (x0..x1, y0..y1) into rows of 64-bit masks [likely] |
| `0x1800cc030` | 127 | occupancy grid: does a rectangle overlap any set bit (8-lane test per row) [likely] |
| `0x1800ce870` | 63 | quaternion product (Hamilton, x y z w lanes) [sure] |
| `0x1800cea10` | 51 | copy up to `n` newest poses out of a 4-entry ring (64-byte entries, count `+0x100`, head `+0x104`) [sure] |

### Gap fill 4: controller boot/track object helpers (`0x1800cf140`–`0x1800e6120`)

"Controller" below is the 0x4ce0-byte per-controller tracking object of a
camera manager (LED count at `+0x1970`, matched-LED mask at `+0x48` of a
candidate pose, 4-entry pose ring at `+0x0`…`+0x104`).

| Function | Lines | Role |
|---|---|---|
| `0x1800cf140` | 179 | motion consistency check: from the two newest poses in the ring, form velocity and angular rate over `dt` (3rd argument) and return how far the given pose is from that extrapolation (4th argument) [likely] |
| `0x1800cf380` | 70 | map one 2D point through the camera model (`0x18009eb50`), the scalar wrapper of the 8-lane map [sure] |
| `0x1800cf440` | 70 | same, other argument order [sure] |
| `0x1800cfc00` | 69 | midpoint of two pose estimates (`0x1800ef770` on each, weights 0.5) for the multi-camera path; callers `0x1800cf540`, `0x1800ef770` [likely] |
| `0x1800d0070` | 35 | unsigned int → `std::string` (decimal); used by `0x18004e8f0` [sure] |
| `0x1800d03e0` | 10 | replace the low 8 bytes of a 16-byte value [sure] |
| `0x1800d0410` | 13 | Vec4 from two 8-byte halves [sure] |
| `0x1800d0460` | 12 | store `clock()` in a timer [sure] |
| `0x1800d05f0` | 76 | scaled norm of a vector (`‖c·v‖·k`), clamped to `10·k` [sure for the math] |
| `0x1800d08c0` | 25 | store a pose built from a quaternion (`0x180045d50`) at controller `+0xf20` [sure] |
| `0x1800d09e0` | 37 | resize a bit-set of up to 256 bits, clearing the bits above the new size in each word [sure] |
| `0x1800d0b40` | 58 | gather up to `n` LED positions for one model into an 8-lane block, padding with the value at `0x18031a5c0` [likely] |
| `0x1800d0cc0` | 16 | reset the controller's pose state: clear `+0x3354`, copy a fresh state (`0x18009fd80`, `0x1800d0d30`) to `+0x1af0` and `+0x20e0`, clear `+0x1f94` [sure] |
| `0x1800d14b0` | 72 | is a point inside a cone: angle between the normalised direction and an axis against a cosine limit [likely] |
| `0x1800d15f0` | 151 | largest angle in degrees between the three axes of a rotation and a reference rotation (`acos` of column dot products) [likely] |
| `0x1800d18c0` | 118 | rotation vector → quaternion (exp map); below `sqrt(FLT_EPSILON)` uses the series `0.5 + θ²/48` [sure] |
| `0x1800d1b80` | 55 | take a stored camera pose (`+0x10a40 + 0x40·cam` of the manager), compose it with the given pose and run the consistency check `0x1800cf140` [sure] |
| `0x1800d22c0` | 33 | swap item `i` and item `j` in every array of an 11-array blob store [sure] |
| `0x1800d2be0` | 39 | release several RnD::Vectors of the controller object (part of its destructor) [sure] |
| `0x1800d2d30` | 90 | construct an object using the shared `RnD::DefaultAllocator` (static init as `0x1800a9db0`) [sure] |
| `0x1800d2fd0` | 38 | release six RnD::Vectors (destructor body) [sure] |
| `0x1800d37e0` | 157 | configure the controller's RANSAC: copy three 128-byte setting blocks to `+0x6b0`, set the thresholds from `+0x634`/`+0x638`/`+0x690` (confidence 0.99), point the solver at its buffers and initialise it (`0x1800c18a0`, `0x1800c6df0`) [sure] |
| `0x1800d3a70` | 18 | full reset of a controller: clear `+0x601` and `+0x3354`, fresh state as `0x1800d0cc0`, then `0x1800a0ab0` [sure] |
| `0x1800d4e10` | 185 | when a pose hint is present (`+0x3360`, or bit 8 of `+0x3364`): build a pose from the quaternion at `+0x3380` and the translation at `+0x33c0` and store it as the prior at `+0x3300`…`+0x3320`, flag `+0x3334` [likely] |
| `0x1800d5a70` | 109 | accept a new pose for a controller: store it at `+0x10`, refresh derived data (`0x1800daff0`), set `+0x4bc`, seed the state at `+0x1af0` when none exists, classify the distance (`0x1800db5f0`), update the overlay data (`0x1800e4df0`) [likely] |
| `0x1800d5c10` | 10 | copy a state into `+0x1af0` and classify the distance (`0x1800db5f0`) [sure] |
| `0x1800d9310` | 186 | copy constructor of a 0x5e0-byte controller pose record (pose, seven 128-byte blocks, then scalar fields up to `+0x5e0`) [sure] |
| `0x1800d9ab0` | 133 | count the matched LEDs whose normal faces the camera (cosine above the threshold at `+0x6a4`; 0.7 for the back-facing test) [likely] |
| `0x1800da220` | 81 | mean of a per-LED value over the matched-LED mask (0 when none) [sure] |
| `0x1800dcb00` | 213 | tracking step with distance-dependent settings: picks one of four parameter tables (`0x180318b88` below 100 mm, `0x180318b60` below 200 mm, `0x180318bd8` below 300 mm, `0x180318bb8` beyond; squared mm), matches LEDs (`0x1800e2330`), projects (`0x18004d380`), scores (`0x1800dd5d0`), solves (`0x1800df5b0`); with fewer than 3 matches it retries from the prior (`0x1800ddb90`), then finishes in `0x1800e0d20` [likely] |
| `0x1800dd170` | 84 | per-camera refine gate: skip when the state at `+0x49c` is 1 and fewer than 6 frames passed (`+0x5dc`), else run the refine from an identity start [likely] |
| `0x1800ddad0` | 18 | score a candidate (`0x1800dd5d0`); if it is valid and not rejected, optionally merge its LED mask into the accepted mask [sure] |
| `0x1800e09f0` | 28 | copy each LED's matched blob (index at `+0x43c`) position and size out of the blob store [sure] |
| `0x1800e0ad0` | 29 | largest of three floats at `+0x19b0` [sure] |
| `0x1800e0b60` | 71 | predict where a point lands in a camera: transform (`0x180046240`) if it is in front (z ≥ 0) and scale the search radius by `+0x1974` [likely] |
| `0x1800e0cb0` | 12 | debug print of two counters and a value per controller (`"%s B=%d %d %6.1f"`, `" T=%d %d %6.1f"`) [sure] |
| `0x1800e1c30` | 28 | remap each matched LED's blob index through a renumbering table (`+0xa8` of the store), or −1 [sure] |
| `0x1800e25a0` | 95 | for one camera: if the camera sees the controller (bit in `+0x8960`), compose its pose with the camera transform and project the LEDs (`0x1800efd70`) [sure for the flow] |
| `0x1800e29b0` | 18 | construct four 536-byte per-camera blocks (`0x1800ef930`) [sure] |
| `0x1800e4d00` | 56 | rotation from a quaternion, then the angle to a reference axis via `0x1800d15f0` [likely] |
| `0x1800e5cc0` | 35 | unsigned int → `std::string` (duplicate of `0x1800d0070`) [sure] |
| `0x1800e5d80` | 110 | copy constructor of an RnD::Vector of 8-byte items (same allocator) [sure] |
| `0x1800e6120` | 83 | copy constructor of an RnD::Vector of 4-byte items [sure] |

### Gap fill 5: containers, images, spot helpers (`0x1800e6340`–`0x1800f4320`)

| Function | Lines | Role |
|---|---|---|
| `0x1800e6340` | 31 | clear a vector of 32-byte `RnD::Vector` items (free each inner buffer through its allocator) [sure] |
| `0x1800e63e0` | 21 | same, other instance [sure] |
| `0x1800e6990` | 31 | `std::find` of an 8-byte value (unrolled by 4) [sure] |
| `0x1800e6a50` | 83 | copy constructor of an RnD::Vector of 2-byte items [sure] |
| `0x1800e6ee0` | 14 | uniform random int in `[lo, hi)` from the twister [sure] |
| `0x1800e7660` | 21 | shrink a vector of inner vectors to `n` items, freeing the cut ones [sure] |
| `0x1800e76d0` | 31 | clear a vector of inner vectors (third instance of `0x1800e6340`) [sure] |
| `0x1800e7a60` | 14 | `std::string` from a `[begin, end)` range [sure] |
| `0x1800e7a90` | 103 | assign a range to an RnD::Vector of 8-byte items (reallocate when the capacity is short) [sure] |
| `0x1800e8760` | 21 | destroy a range of inner vectors [sure] |
| `0x1800e8c10` | 83 | copy-construct one inner vector into place (allocate and copy its items) [sure] |
| `0x1800e8dd0` | 13 | free one inner vector and zero it [sure] |
| `0x1800e9310` | 83 | copy constructor of an RnD::Vector of 4-byte items (duplicate of `0x1800e6120`) [sure] |
| `0x1800e9cf0` | 14 | Vec2 with lane `i & 1` replaced [sure] |
| `0x1800e9d30` | 14 | duplicate of `0x1800e9cf0` [sure] |
| `0x1800e9d70` | 21 | Vec2 lane write through a (pointer, lane) proxy [sure] |
| `0x1800ea0d0` | 18 | release two RnD::Vectors (destructor body) [sure] |
| `0x1800ea130` | 36 | `RnD::Spots` destructor body: release its five vectors; caller `0x1800ea090` [sure] |
| `0x1800ea240` | 94 | image object constructor from a 64-byte descriptor: copy the descriptor, allocate `width × height` pixels through the default allocator, element code 4 [likely] |
| `0x1800eec90` | 26 | image buffer constructor: flags `0x40000`, buffer from the given allocator or the default one (`0x1800ef5f0`) [sure] |
| `0x1800ef140` | 16 | of two pixel references (image, x, y), return the one with the smaller u16 value [sure] |
| `0x1800ef1a0` | 38 | copy a u16 image row by row (stride `+0x14`, width `+0x10`, height `+0x12`) [sure] |
| `0x1800ef5f0` | 18 | get the shared `RnD::DefaultAllocator` (thread-safe static init) [sure] |
| `0x1800ef770` | 96 | mean reprojection distance of a pose over a camera's observations; an observation that cannot be projected counts 999 [likely] |
| `0x1800ef930` | 52 | per-camera block constructor: capacities 24 and 24, counters zero; used by `0x1800e29b0` [sure] |
| `0x1800efae0` | 58 | reset four per-camera counter blocks [sure] |
| `0x1800f0530` | 25 | Vec2 "all lanes equal" mask [sure] |
| `0x1800f0640` | 41 | index of the smallest of 24 floats (three 8-lane blocks) when it is below the limit at `0x18031a5c0`, else −1 [sure] |
| `0x1800f1c60` | 42 | Vec2 distance `‖a − b‖` [sure] |
| `0x1800f20b0` | 24 | scalar `sin` and `cos` through `0x18009e2a0` [sure] |
| `0x1800f2130` | 104 | eigen-decomposition of a symmetric 2×2 (blob covariance): both eigenvalues and unit eigenvectors, degenerate case when the off-diagonal product is below 1e-11 [sure] |
| `0x1800f3010` | 86 | for 256 stored 2D points, 8 at a time: scaled squared distance to a query point [likely] |
| `0x1800f3100` | 56 | RnD::Vector constructor with room for 32 8-byte items [sure] |
| `0x1800f3910` | 118 | nearest stored point to a query within a radius; when the 8th argument exceeds the 6th it uses the plain search `0x1800f32c0` instead [likely] |
| `0x1800f3d30` | 59 | transpose a 3×3 and return its three columns [sure] |
| `0x1800f3e70` | 111 | transform a point by an M34 held in AVX registers [sure] |
| `0x1800f3ef0` | 61 | repack a 3×4 matrix from AVX halves into row form [sure] |
| `0x1800f4320` | 68 | rotation matrix → quaternion: `sqrt` of `1 ± diag` combinations in double precision, signs copied from the off-diagonal terms [likely] |

### Gap fill 6: P3P inputs, pose Jacobians and LM matrix plumbing (`0x1800f59e0`–`0x1800ffa90`)

The LM solvers (`WeightedLMSolver<…,6>`, part A "Solver classes") keep
their vectors and matrices in small dense objects: `{…, data +0x20/+0x28/
+0x30, size, allocator +0x28/+0x48/+0x60}` with 32-byte aligned storage.
Several constructors take a pointer into a larger expression object and
step back to its start (`a2 − 41`, `a2 − 241`, `a2 − 240`): these are the
copy-from-expression paths. Allocation goes through the allocator at
`0x1805f0700` / `0x1805f0718`. Callers listed as `0x18023d8xx`–`0x18023daxx`
are exception-unwind funclets.

| Function | Lines | Role |
|---|---|---|
| `0x1800f59e0` | 89 | 8-lane P3P input: for three observations, form unit bearing rays (8 hypotheses at once) [likely] |
| `0x1800f5b10` | 34 | three 2D observations → homogeneous Vec4 (x, y, 1, 0) [sure] |
| `0x1800f5b90` | 31 | same, other argument layout [sure] |
| `0x1800f6890` | 28 | split an M34 into its 3×3 rotation and translation [sure] |
| `0x1800f6920` | 43 | M34 → M44 with last row (0, 0, 0, 1) [sure] |
| `0x1800f6d80` | 174 | 6×6 adjoint of a rigid pose: R in the two diagonal 3×3 blocks and `[t]×·R` in the off-diagonal block (blocks written by `0x1800f8960`); caller `0x1800f76a0` [likely] |
| `0x1800f72d0` | 44 | rigid inverse of the pose stored at `+0x40` of a cost object [sure] |
| `0x1800f73f0` | 22 | free a dense buffer (`+0x30`) through its allocator (`+0x28`); unwind funclets call it [sure] |
| `0x1800f7430` | 22 | free a dense buffer (`+0x30`, allocator `+0x50`) [sure] |
| `0x1800f7580` | 44 | rigid inverse of the pose stored at `+0x50` [sure] |
| `0x1800f76a0` | 148 | propagate a 6×6 covariance through a pose: `Ad · C · Adᵀ` with `Ad` from `0x1800f6d80`, scaled by `s²` (3rd argument) [likely] |
| `0x1800f7de0` | 22 | free a dense buffer (allocator `+0x50`) [sure] |
| `0x1800f8570` | 26 | free a dense buffer (`+0x20`, allocator `+0x60`) [sure] |
| `0x1800f85f0` | 22 | free a dense buffer (`+0x28`, allocator `+0x48`) [sure] |
| `0x1800f8670` | 22 | same, other instance [sure] |
| `0x1800f86b0` | 12 | fill a dense vector with a scalar (`0x1800fe9e0`) [sure] |
| `0x1800f8750` | 22 | free a dense buffer (`+0x30`, allocator `+0x28`) [sure] |
| `0x1800f87a0` | 16 | fill 48 floats at `+0x10` with a scalar [sure] |
| `0x1800f87d0` | 61 | splat the nine entries of a 3×3 into a broadcast table [sure] |
| `0x1800f8960` | 24 | copy a 3×3 block into a 6×6 at block row/column (i, j) [sure] |
| `0x1800f8a60` | 21 | dense matrix copy-from-expression constructor (`0x1800ff7c0`), alignment 16 [sure for the form] |
| `0x1800f8ad0` | 18 | dense vector copy-from-expression constructor (`0x1800ff540`) with a scalar factor [sure for the form] |
| `0x1800f8b30` | 77 | evaluate a dense expression into a vector (element loop with the scale at `+0xd0`) [likely] |
| `0x1800f99f0` | 31 | release the solver's buffers when `+0x100` is set; funclet `0x18023d830` [sure] |
| `0x1800fb220` | 31 | same for the multi-camera solver; funclet `0x18023d860` [sure] |
| `0x1800fbda0` | 35 | reset a dense vector (free, then null fields) [sure] |
| `0x1800fc150` | 29 | dense vector constructor with `n` floats, rounded up to 32 bytes [sure] |
| `0x1800fc1f0` | 26 | dense vector destructor [sure] |
| `0x1800fc260` | 22 | dense buffer destructor (`+0x28`/`+0x48`) [sure] |
| `0x1800fc2b0` | 22 | same, other instance [sure] |
| `0x1800fc360` | 25 | dense matrix constructor for `rows × cols` with the given alignment [sure] |
| `0x1800fc410` | 20 | fill with a scalar and return the size (0 when unset) [sure] |
| `0x1800fc490` | 22 | dense buffer destructor (`+0x30`/`+0x28`) [sure] |
| `0x1800fc540` | 63 | splat the nine entries of a 3×3 into a broadcast table (second instance of `0x1800f87d0`) [sure] |
| `0x1800fc8a0` | 77 | weighted residuals for the multi-camera cost: compute residuals (`0x180107700`), then, unless weighting is off (`+0x130`), multiply each by its weight; when off, copy through `0x180100280`; caller `0x1800fb2b0` [likely] |
| `0x1800fca70` | 21 | copy-from-expression constructor (as `0x1800f8a60`) [sure for the form] |
| `0x1800fcae0` | 18 | copy-from-expression constructor with a scalar (as `0x1800f8ad0`) [sure for the form] |
| `0x1800fcb40` | 53 | dense matrix view/constructor over an existing buffer (rows, cols, strides) [likely] |
| `0x1800fcca0` | 77 | evaluate a dense expression into a vector (as `0x1800f8b30`) [likely] |
| `0x1800fcf60` | 13 | squared norm of the six floats at `[4..9]` (parameter step or gradient) [sure for the math] |
| `0x1800fd100` | 68 | accumulate `Jᵀ·r` over six parameters with special cases for fewer than 4 residual blocks; callers `0x1800f9620`, `0x1800fae70` [likely] |
| `0x1800fd250` | 53 | build the normal-equation matrix through `0x1800fef60`, using the default allocator for scratch [likely] |
| `0x1800fd4f0` | 12 | expression object → its data pointer (null-safe) [sure for the form] |
| `0x1800fd5f0` | 33 | dense matrix constructor rounded to 24-float rows [sure] |
| `0x1800fdb40` | 19 | dense buffer reassign: free, then zero the fields [sure] |
| `0x1800fe0c0` | 21 | same pattern, other layout [sure] |
| `0x1800fe210` | 20 | same pattern, other layout [sure] |
| `0x1800fe420` | 26 | 3×3 times a 3-vector from broadcast tables (output of `0x1800f87d0`) [likely] |
| `0x1800fe5b0` | 46 | weighted residuals for the single-camera cost (flag `+0x120`), as `0x1800fc8a0` [likely] |
| `0x1800fe720` | 46 | same, layout with the flag at `+0x130` [likely] |
| `0x1800fe8c0` | 16 | copy-from-expression constructor with an explicit alignment [sure for the form] |
| `0x1800fe930` | 14 | copy-from-expression constructor with scalar and alignment [sure for the form] |
| `0x1800feaf0` | 41 | dense matrix view over a buffer (as `0x1800fcb40`) [likely] |
| `0x1800fec00` | 30 | evaluate a dense expression into a vector (third instance) [likely] |
| `0x1800fedf0` | 41 | accumulate `Jᵀ·r` (as `0x1800fd100`, other cost) [likely] |
| `0x1800fef60` | 95 | normal-equation accumulation `JᵀWJ` over residual blocks; caller `0x1800fd250` [guess] |
| `0x1800ff180` | 43 | scaled `Jᵀ·r` with a damping term (λ, 4th argument) [guess] |
| `0x1800ff2d0` | 12 | expression object → data pointer (second instance) [sure for the form] |
| `0x1800ff360` | 35 | dense vector copy-assign (reuse the buffer when sizes match) [sure] |
| `0x1800ff540` | 126 | evaluate a scaled vector expression into a new buffer (six entries per block); callers `0x1800f76a0`, `0x1800f8ad0`, `0x1800fcae0`, `0x1800fe930` [likely] |
| `0x1800ff7c0` | 58 | evaluate a matrix expression into a new buffer through `0x180102c10`; callers `0x1800f76a0`, `0x1800f8a60`, `0x1800fca70`, `0x1800fe8c0` [likely] |
| `0x1800ffa90` | 21 | dense buffer reassign (fourth instance) [sure] |

### Gap fill 7: dense-expression evaluators, robust scale, distortion lanes (`0x1800ffbf0`–`0x180109f80`)

Most functions from `0x1800ffbf0` to `0x180102ed0` evaluate one kind of
dense expression of the LM solver into a buffer (copy, scaled copy,
product with the 6×6 kernel `0x180102c10`, element-wise loops through a
(matrix, linear index) iterator). They have no direct caller because the
solver reaches them through its templates' vtables. Their exact
expression types were not resolved; the shape of each body is given.

| Function | Lines | Role |
|---|---|---|
| `0x1800ffbf0` | 22 | expression → (data pointer, `rows × cols`) pair [sure] |
| `0x1800ffdd0` | 39 | gather ten scalar pointers into a contiguous 3×3-plus-one record [likely] |
| `0x1800ffff0` | 55 | difference of two (pointer, index) iterators, normalised by the row length [likely] |
| `0x180100280` | 54 | sum of squares of a dense float vector (unweighted residual cost); callers `0x1800fc710`, `0x1800fc8a0`, `0x1800fe5b0`, `0x1800fe720` [sure] |
| `0x180100330` | 92 | evaluate a scaled 6-column expression (factor at `+0xd0`) into a new matrix [likely] |
| `0x180100560` | 53 | dense vector copy-assign (second instance of `0x1800ff360`) [sure] |
| `0x180100730` | 127 | evaluate a scaled vector expression into a new buffer (as `0x1800ff540`) [likely] |
| `0x180100990` | 60 | evaluate a matrix expression into a new buffer through `0x180102c10` [likely] |
| `0x180100b90` | 15 | advance an iterator `(ptr, index)` by `n` [sure] |
| `0x180100e70` | 26 | copy nine floats into a dense matrix in row order [sure] |
| `0x180100f20` | 53 | copy between two element iterators (row/column from `index / cols`, `index % cols`) [sure for the form] |
| `0x180101020` | 34 | fill through an element iterator [sure for the form] |
| `0x180101390` | 124 | evaluate a scaled vector expression (third instance) [likely] |
| `0x1801015e0` | 56 | evaluate a matrix expression through `0x180102c10` (second instance) [likely] |
| `0x1801018e0` | 28 | scatter a list of ints into a 3-row, 4-stride matrix in column order [sure for the form] |
| `0x1801019a0` | 35 | copy between element iterators (second instance) [sure for the form] |
| `0x180101c70` | 124 | evaluate a scaled vector expression (fourth instance) [likely] |
| `0x180101ec0` | 56 | evaluate a matrix expression through `0x180102c10` (third instance) [likely] |
| `0x180102150` | 29 | append ints through an element iterator [sure for the form] |
| `0x180102280` | 124 | evaluate a scaled vector expression (fifth instance) [likely] |
| `0x1801024d0` | 56 | evaluate a matrix expression through `0x180102c10` (fourth instance) [likely] |
| `0x1801026d0` | 115 | evaluate a scaled 6-column expression (second instance of `0x180100330`) [likely] |
| `0x180102920` | 46 | 6×6 product through `0x180102c10` into a fixed 6×6 [likely] |
| `0x180102a30` | 25 | 6×6 product, then copy the 6 × 3 result block [likely] |
| `0x180102ab0` | 25 | duplicate of `0x180102a30` [sure] |
| `0x180102c10` | 33 | the 6×6 kernel: `out = M·N` (6 × 6 times 6 × k, scaled by the float argument); called by all the evaluators above [likely] |
| `0x180102d60` | 9 | copy one double [sure] |
| `0x180102ed0` | 9 | copy one double (duplicate) [sure] |
| `0x180103460` | 69 | single-camera reprojection residual: transform an LED by the pose (`0x180046240`), project, subtract the observation, return the 2D error and its squared length [likely] |
| `0x180104260` | 32 | 2 × 3 Jacobian block times a 3-vector (projection chain step) [likely] |
| `0x180104450` | 102 | 8-lane distortion: rational polynomial in r² (numerator from model `+0x20..+0x38`, divided by a second polynomial) [likely] |
| `0x180104520` | 175 | 8-lane distortion with the tangential and thin-prism terms (coefficients `+0x24..+0x44`) [likely] |
| `0x180104810` | 14 | robust scale: `1.483 · median · (1 + 5 / (n − p))`, median by partial sort (`0x1801090f0`) [sure] |
| `0x1801048e0` | 88 | rotation vector → quaternion with `sincos` (`0x18009e2a0`), angle scaled by the float argument [likely] |
| `0x180104a10` | 89 | rotation vector → rotation matrix through a quaternion (`0x180045d50`) [likely] |
| `0x180104b50` | 163 | derivative of the exp map at a rotation vector (3×3 Jacobian of `0x180104e30`), with a small-angle branch [guess] |
| `0x180105050` | 205 | 8-lane projection with Jacobians: for 8 points, the projected pixel and its derivatives with respect to the pose parameters, through the model map `0x1800c08a0` [likely] |
| `0x1801052b0` | 9 | wrapper: project with distortion (`0x180103580`) with the "no Jacobian" flag 0 [sure] |
| `0x180107040` | 84 | cost helper: pose from the 6-vector (rotation vector `+0x10`, translation `+0x1c`) through the exp map `0x180104e30`, composed with the rig transform (`0x1800451e0`) [sure for the flow] |
| `0x180107170` | 47 | residual scale of a cost: parameter count 3 or 2 from the robustifier's slot 4, then the median rule (`0x1801046e0`) over the residuals at `+0xb8` [likely] |
| `0x1801083d0` | 84 | as `0x180107040` for the multi-camera cost [sure for the flow] |
| `0x180108500` | 47 | as `0x180107170` for the multi-camera cost (`+0xc8`) [likely] |
| `0x180108620` | 19 | dense view constructor over an external buffer: data pointer and length from the arguments, one column, no allocator (decompiler had no function here; read from the disassembly) [sure] |
| `0x180108660` | 12 | median helper: partial sort so the middle element is in place, return a pointer to it [sure] |
| `0x180108810` | 35 | reset a dense vector, then size it (free, then allocate) [sure] |
| `0x180108bb0` | 49 | pose from the 6-vector with a scale on the rotation part (exp map) [likely] |
| `0x180108c70` | 47 | residual scale (third instance of `0x180107170`) [likely] |
| `0x180108d80` | 47 | residual scale (fourth instance) [likely] |
| `0x180108eb0` | 36 | reset and size a dense matrix [sure] |
| `0x180109080` | 23 | rebind a dense vector to an external buffer (free the old one first) [sure] |
| `0x180109280` | 44 | pose from the 6-vector with a scale (second instance of `0x180108bb0`) [likely] |
| `0x180109350` | 35 | reset and size a dense matrix (second instance) [sure] |
| `0x180109950` | 44 | insertion sort of a float range [sure] |
| `0x180109f50` | 19 | set every weight of a vector to 1.0 (robustifier "no weighting") [sure] |
| `0x180109f80` | 19 | duplicate of `0x180109f50` [sure] |

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

787 functions.

### app:dialog (3)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x18012dd60` | fluff | thunk | forwards to `0x18012ddb0` | none (vtable/callback) |
| `0x18012ddb0` | fluff | dtor | destructor (frees itself when the flag argument is set) | `0x18012dd60` |
| `0x18012de50` | fluff | dtor | destructor of `CreateProcessThread` (frees itself when the flag argument is set) | none (vtable/callback) |

### config (8)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x180154fd0` | ref | dtor | destructor of `ConfigManager` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180157ef0` | fluff | dtor | destructor of `ShareManager` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x180158198` | fluff | thunk | forwards to `0x1801581b0` | none (vtable/callback) |
| `0x1801581f0` | ref | dtor | destructor of `ShareManager` (frees itself when the flag argument is set) | none (vtable/callback) |
| `0x18015c670` | fluff | const_ret | wrapper around `WaitForSingleObject` | none (vtable/callback) |
| `0x18015e230` | core | const_ret | wrapper around `ReleaseMutex` | `0x1801185a0`, `0x18011b700` +1 |
| `0x18015e340` | core | const_ret | wrapper around `SetEvent` | none (vtable/callback) |
| `0x18015e480` | core | const_ret | wrapper around `ReleaseMutex` | none (vtable/callback) |

### ctrl-boot (131)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x18009db00` | fluff | const_ret | returns a constant | `0x18009db10`, `0x18009db70` +1 |
| `0x18009db10` | core | setter | wrapper around `_local_stdio_scanf_options` | none (vtable/callback) |
| `0x18009e100` | core | thunk | forwards to `0x18009e170` | none (vtable/callback) |
| `0x18009e120` | core | thunk | forwards to `0x18009e2a0` | none (vtable/callback) |
| `0x18009e430` | core | thunk | forwards to `0x18009e2a0` | none (vtable/callback) |
| `0x18009f500` | core | thunk | forwards to `0x18009f530` with extra constant arguments | none (vtable/callback) |
| `0x18009f770` | ref | dtor | destructor (frees itself when the flag argument is set) | none (vtable/callback) |
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
| `0x1800ba770` | core | zero_init | constructor/reset of `RnD::MersenneTwister`: sets about 23 fields to zero or defaults | `0x180049240`, `0x18004fe10` +1 |
| `0x1800bb680` | core | vec_grow | RnD::Vector grow/insert for 4-byte elements (allocates through the allocator, moves the old items) | none (vtable/callback) |
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

### ctrl-tracker (63)

| Function | Verdict | Shape | Role | Callers |
|----------|---------|-------|------|---------|
| `0x180051380` | core | eh_ctor | destroys an array at +0x700 of 64 × 0x1f0-byte elements (no-op element function) | `0x18023d1c0` |
| `0x1800513d0` | core | eh_ctor | destroys an array of 4 × 0x40-byte elements (no-op element function) | `0x18004ab50`, `0x180050010` +7 |
| `0x1800513f0` | core | eh_ctor | destroys an array of 16 × 0x5f0-byte elements (no-op element function) | `0x18023d2e0` |
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

### math+threads (20)

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

367 functions: flirt 154, vtable 108, string 105.

### app:dialog (21)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x18012da60` | fluff | string | uses 'IVRServerDriverHost_006', '[VrDialogManager] finish psvr2 by update paused' | app |
| `0x18012dd70` | fluff | flirt | `std::basic_stringbuf<char,std::char_traits<char>,alloc>::`scalar deleting destructor'(uint)` | MSVC C++ standard library |
| `0x18012e8b0` | fluff | string | uses 'IVRProperties_001', 'appmanifest_2580190.acf' | app |
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

### lib:json+config (48)

| Function | Verdict | Evidence | Identity / role | Library |
|----------|---------|----------|-----------------|---------|
| `0x180140440` | fluff | string | uses 'additional six characters expected to parse unicode surrogat', 'expecting another \u token to begin the second half of a uni' | jsoncpp |
| `0x1801406d0` | fluff | string | uses 'additional six characters expected to parse unicode surrogat', 'expecting another \u token to begin the second half of a uni' | jsoncpp |
| `0x180140960` | fluff | string | uses 'Bad unicode escape sequence in string: four digits expected.', 'Bad unicode escape sequence in string: hexadecimal digit exp' | jsoncpp |
| `0x180140b80` | fluff | string | uses 'Bad unicode escape sequence in string: four digits expected.', 'Bad unicode escape sequence in string: hexadecimal digit exp' | jsoncpp |
| `0x180140dd0` | fluff | string | uses 'in Json::Value::duplicateAndPrefixStringValue(): Failed to a', 'in Json::Value::duplicateAndPrefixStringValue(): length too' | jsoncpp |
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

---

## Part F: libpad internals

Gap-fill rows for the statically linked libpad (`controllers.md` part A
describes its behaviour). Names of the public entry points are not in the
binary; roles are from the bodies.

Layout used below:

- **pad slots**: six 0x3410-byte records from `0x1805974e0` (in use at
  `+0`, VID `+2`, PID `+4`, handle `+0x10`, user/index `+0x14`, HID device
  id `+0x1c`, flags `+0x6c`, device record pointer `+0x170`, pending output
  buffer `+0x29f8` with local copy `+0x2978`).
- **HID device slots**: 0x2200-byte records from `0x1805b1580` (handle
  `+0x10`, VID/PID `+0x20`/`+0x22`, firmware `+0x24`), up to `0x1805cad80`.
- **API frame**: almost every entry point checks "initialised"
  (`0x1805974c0`, else `0x80920005`), its arguments (`0x80920001`), takes
  the libpad lock (`0x1801d4280`, `0x809200ff` on failure), validates the
  handle (`0x1801c7630`, else `0x80920003`), short-cuts the special
  "virtual pad" handle in `0x1805974c8` (returns neutral data), finds the
  device (`0x1801c8280`, else `0x80920007`), then releases the lock
  (`0x1801d42b0`). Rows below give only what happens inside that frame.
- PID predicates: `0x0ce6` DualSense, `0x0df2` DualSense Edge, `0x0ce7` /
  `0x0e45` / `0x0e46` VR controller, `0x05c4` / `0x09cc` DS4 (VID
  `0x054c`).
- Output buffer flags: byte 0 = 2 marks the buffer dirty; bytes 1 and 2
  are the DualSense `valid_flag0` / `valid_flag1` bits as in Linux
  `hid-playstation`.

### F.1 Read path and state conversion [sure unless tagged]

| Function | Lines | Role |
|---|---|---|
| `0x1801bb160` | 160 | validate a trigger-effect parameter block: mode bits only in the low two, per-mode limits (for example position ≤ 9, strength ≤ 8); caller `0x1801bf060` |
| `0x1801bb330` | 151 | fill the extended controller-information record for a device: DualSense firmware ≥ `0x1003c`, Edge, DS4 v2 firmware gate `0x1801c95f0` and VR controller cases set different capability bits; callers `0x1801bd420`, `0x1801bd5d0` |
| `0x1801bb650` | 161 | read up to `n` buffered input states for a handle (capped by the ring count `+0x160`), converting each through `0x1801bc270`; zero states when the device record is gone |
| `0x1801bb9d0` | 129 | read the latest input state (single-state variant of `0x1801bb650`) |
| `0x1801bbc50` | 172 | read up to `n` states in the extended format (second layout, for the VR controller path) |
| `0x1801bbfd0` | 108 | latest state in the extended format |
| `0x1801bc210` | 27 | neutral input state: sticks 0x80, identity orientation, zero motion |
| `0x1801bc270` | 102 | convert one internal state into the output layout: button mask (`0x80009502`/`…503` filter for the "intercepted" flag), sticks, triggers, motion, touch |
| `0x1801bc450` | 28 | map device button bits 3–7 to the output button word, swapping cross/circle when the "enter button" flag is set |
| `0x1801bdd70` | 33 | API: read up to 1..64 states (`0x1801bb650`); neutral state for the virtual pad |
| `0x1801be510` | 32 | API: read the latest state (`0x1801bb9d0`) |
| `0x1801bf880` | 69 | API: read states in the extended format; with a virtual pad it reads the real pad behind it (`0x1805974c8` high dword) |
| `0x1801bfa20` | 187 | API: read and merge the states of two handles (left and right VR controllers) into one extended record |
| `0x1801bfd70` | 67 | API: latest state in the extended format, virtual-pad aware |

### F.2 Information and settings entry points [likely unless tagged]

| Function | Lines | Role |
|---|---|---|
| `0x1801bc570` | 42 | API (caller `0x18011a0f0`): for a VR controller in mode 1, reset its HID output state (`0x1801ca780`) or mark it for re-init (`0x1801ca9d0`) |
| `0x1801bcc90` | 41 | API: read a 2-entry record (count must be 2) from the HID layer (`0x1801caa40`); caller `0x18011bae0` |
| `0x1801bcd90` | 42 | API: copy the device's product string (`0x1801caeb0`) into the caller's buffer |
| `0x1801bcea0` | 46 | API: report whether the handle's device is connected (0/1) [sure] |
| `0x1801bcfa0` | 75 | API: controller information (connection type, sticks dead zone, touch-pad size) into a 32-byte record |
| `0x1801bd120` | 45 | API: device type for a handle (0 for the virtual pad) |
| `0x1801bd240` | 30 | API: look up a pad slot by user index 1..4 (`0x1801c9410`); returns `0x80920008` when no pad there [sure] |
| `0x1801bd2d0` | 49 | API: per-family capability word (DualSense, Edge, VR) |
| `0x1801bd420` | 71 | API: extended information through `0x1801bb330` |
| `0x1801bd5d0` | 62 | API: second extended information variant through `0x1801bb330` |
| `0x1801bd840` | 60 | API: init with the default allocator (`malloc`/`free`): set the allocator hooks (`0x1801d2860`, `0x1801d2830`), mode `0x1801d2880`, timer period 1 ms, start the threads (`0x1801ca700`) [sure] |
| `0x1801bd950` | 61 | API: init with a caller-supplied allocator pair; fails if either pointer is null [sure] |
| `0x1801bdb60` | 54 | API: firmware-feature check, for example DualSense firmware below `0x390` |
| `0x1801be5d0` | 31 | API: reset the orientation of a pad (`0x1801c2f50`) |
| `0x1801be660` | 35 | API: return a per-pad setting from the device record |
| `0x1801be720` | 37 | API: set or clear flag bit 0 of the pad settings word (`0x1801d4610` / `0x1801d4680`) [sure for the bit] |
| `0x1801be7d0` | 45 | API: same for the real pad behind a virtual one |
| `0x1801be8e0` | 19 | API: set the global byte `0x180351661` [sure] |
| `0x1801be930` | 20 | API: set the global audio/volume word (`0x1801cc640`), then push it to every device (`0x1801c8e60`) [sure for the flow] |
| `0x1801bea30` | 51 | API: write a 3-byte light-bar colour for a DualSense or VR pad (`0x1801c4030`); values above 12 rejected for others |
| `0x1801bebb0` | 41 | API: set a 0/1/2 mode through `0x1801c3d20` (light-bar or player-LED brightness) |
| `0x1801beda0` | 52 | API: set a 0/1/2 mode through `0x1801c54c0` (mic-mute LED) |
| `0x1801beeb0` | 19 | API: set the global byte `0x1805b1572` (`0x1801cc680`) [sure] |
| `0x1801bef50` | 45 | API: set or clear flag bit 2 of the pad settings word [sure for the bit] |
| `0x1801bf3d0` | 34 | API: vibration on a pad (`0x1801c3020`, two motor bytes) [sure] |
| `0x1801bf4a0` | 31 | API: set the vibration mode 1..5 (`0x1801c5730`) [sure] |
| `0x1801bf850` | 15 | API: return the libpad version word (`0x1801c8210`); called from many driver paths [sure] |
| `0x1801bfee0` | 34 | API: store a value > 1 at `+0x2a4c` of the device record [sure] |
| `0x1801bffb0` | 63 | API: when pads 5 and 6 are both present, re-initialise both (`0x1801c8ca0`) [sure for the flow] |
| `0x1801c0160` | 12 | API: set the boolean `0x180351658` (`0x1801c17f0`) for values > 1 [sure] |
| `0x1801c0190` | 19 | API: forward a value to `0x1801d2850` [sure] |
| `0x1801c01e0` | 12 | API: register the time callback (`0x1801c1810`) [sure] |
| `0x1801c0490` | 12 | API: register a second callback (`0x1801c1800`) [sure] |
| `0x1801c04b0` | 23 | API: set a 0..4 value globally (`0x1801d27f0`) and apply it to every VR pad (`0x1801c8ea0`) [sure for the flow] |
| `0x1801c0690` | 19 | API: forward a value to `0x1801d2890` [sure] |
| `0x1801c0a80` | 15 | call the registered time callback (`0x1805974d0`) and return its value, 0 when none [sure] |
| `0x1801c15b0` | 20 | return one of two globals (`0x180351648`/`0x180351650`) under the lock [sure] |
| `0x1801c15f0` | 26 | default LED-sync record: phase 5, frame cycle 16 683 350, offset −1, width 1000, step 105, half-width 1575; caller `0x1801c7b10` [sure] |
| `0x1801c17f0` | 9 | setter of `0x180351658` [sure] |
| `0x1801c1810` | 9 | setter of the time callback `0x1805974d8` [sure] |

### F.3 Output helpers (DualSense / VR output fields) [likely unless tagged]

| Function | Lines | Role |
|---|---|---|
| `0x1801c2aa0` | 47 | read feature `0x12` (16 bytes) and return the 6-byte device address, byte-reversed [sure] |
| `0x1801c2ba0` | 47 | same report, the second address in it (paired host) [sure for the read] |
| `0x1801c2f50` | 38 | reset orientation: VR controllers via `0x1801c9710`, DualSense by rewriting the light-bar state and output (`0x1801c4030`) |
| `0x1801c34d0` | 197 | set `valid_flag1` bits for light-bar setup and power-save (bits 2, 1, 4) with byte 36 = 1, then queue the output (`0x1801cbeb0`); caller `0x1801c8d50` |
| `0x1801c3640` | 178 | audio control bits 6–7 of `+8` (DualSense only), pending buffer created at `+0x2978` if none |
| `0x1801c37a0` | 217 | audio output-path bits 4–5 of `+8`, with `valid_flag0` bit 7 |
| `0x1801c3ac0` | 27 | write feature `0x13` with a 6-byte address (byte-reversed) and a 16-byte key (`0x1801cb7d0`): pairing info [likely] |
| `0x1801c3d20` | 175 | light-bar / player-LED brightness 0..2 for DualSense and VR controllers |
| `0x1801c4030` | 170 | light-bar colour (three bytes) into the pending output; VR controllers use their own field |
| `0x1801c4310` | 63 | power-save / haptics bits of `+8` by mode 0..2, reading the device capability at `+0x89` |
| `0x1801c50c0` | 52 | VR controller output: `valid_flag0` bit 3 and a 2-bit field at byte 4, bit 7 when `+0xa` (vibration level) is 0 |
| `0x1801c5200` | 104 | mic-mute LED and related bits (argument bit 2 selects the path) |
| `0x1801c54c0` | 45 | mic-mute LED mode into byte 9 with `valid_flag1` bit 0 |
| `0x1801c5610` | 37 | set the "output disabled" byte `+7` and clear the pending buffer |
| `0x1801c5e00` | 107 | speaker volume: `valid_flag0` bit 5 and byte 6 from the table at `0x180269890` |
| `0x1801c6a40` | 28 | parse a trigger-effect request (`0x1801c65f0`) and report whether each trigger is in a feedback mode (types 4..6) |

### F.4 Pad slots, HID devices and threads [likely unless tagged; the container rows are sure]

| Function | Lines | Role |
|---|---|---|
| `0x1801c7630` | 25 | validate a handle against the six pad slots (in-use only when asked); `0x80010002` when not found [sure] |
| `0x1801c8280` | 47 | find the in-use pad slot for a handle and return it with its device record; error when none [sure] |
| `0x1801c7680` | 21 | for the pad with HID id `n`: when its "measure" flag is set, stamp the time with `QueryPerformanceCounter` |
| `0x1801c7e90` | 12 | free the pad slot that belongs to HID id `n` |
| `0x1801c7ef0` | 22 | find the pad slot for a handle among the six and reset it (`0x1801c8b20`) |
| `0x1801c7fa0` | 14 | BCD (4 digits, each clamped to 9) → integer |
| `0x1801c8450` | 42 | reset the per-slot default output state of all pads (light-bar 13/0/0 etc. by slot type) |
| `0x1801c8570` | 141 | build and send the initial output report for a DualSense or Edge (report `0x05`/BT `0x31`, flags by connection type) [likely] |
| `0x1801c8a40` | 37 | pad slot defaults on connect (light-bar `0x0503`, mode 9, vibration level 2, `0x2b0c` …), then the initial output `0x1801c8570` |
| `0x1801c8b20` | 53 | on disconnect, send a final "all off" output report (`0x82 0x53` or `0x57` header by type) |
| `0x1801c8ca0` | 19 | re-initialise the pad slot that has handle `n` (`0x1801c8a40`) |
| `0x1801c8e60` | 7 | push the global volume setting to every HID device (`0x1801d2ef0`) |
| `0x1801c8ea0` | 7 | apply a VR-controller output field to every VR pad slot (`0x1801c50c0`) |
| `0x1801c8ef0` | 10 | stop vibration on every DualSense / Edge (`0x1801c3020(h, 0, 0)`) |
| `0x1801c8f80` | 6 | send the "all off" output to every pad (`0x1801c8b20`) |
| `0x1801c8fc0` | 13 | read feature `0x02` (48 bytes for DualSense and VR, else none) through `0x1801cba20`: calibration request |
| `0x1801c93b0` | 13 | find a pad slot by handle (optionally only in-use slots, VR range) |
| `0x1801c9410` | 7 | find a pad slot by user index |
| `0x1801c94b0` | 10 | VR controller PID test with a hand filter (`0x0e45` left, `0x0e46` right) |
| `0x1801c9530` | 10 | "DualSense family or VR controller" PID test |
| `0x1801c95f0` | 5 | DS4 (`0x05c4`, `0x09cc`) with firmware ≥ 23 |
| `0x1801c9630` | 6 | DS4 v2 firmware test (≥ `0x3000`, high nibble not 3, 4, 5, 8 or 9) |
| `0x1801c9710` | 27 | VR controller orientation-reset field by connection kind (values 13, `0x0d00`, …) |
| `0x1801c9800` | 25 | `std::wstring` less-than (map key compare) |
| `0x1801c9870` | 17 | recursive free of a `std::map<wstring, …>` subtree with the libpad allocator |
| `0x1801c9930` | 38 | `std::map<wstring, …>` lower-bound search |
| `0x1801c99f0` | 31 | `std::map<wstring, …>` insert (72-byte nodes through the libpad allocator) |
| `0x1801c9b30` | 32 | `std::wstring` from a C wide string with the libpad allocator |
| `0x1801c9c30` | 26 | `std::wstring` copy constructor with the libpad allocator |
| `0x1801c9d10` | 6 | free a buffer with the libpad free hook |
| `0x1801c9d50` | 6 | same, second instance |
| `0x1801c9d80` | 11 | `std::wstring` destructor (libpad free hook) |
| `0x1801c9dd0` | 7 | `std::map` destructor |
| `0x1801c9e10` | 11 | `std::wstring` destructor (duplicate) |
| `0x1801c9e60` | 145 | red-black tree erase-and-rebalance |
| `0x1801ca200` | 107 | red-black tree insert-and-rebalance |
| `0x1801ca540` | 42 | count connected devices by family (DualSense, DS4, VR) for a device-path filter |
| `0x1801ca700` | 14 | start libpad once: init locks and defaults, then the scanner (`0x1801cfe40`) and writer (`0x1801cdd40`) threads |
| `0x1801ca780` | 14 | find the HID device with id `n` and reset its output state (`0x1801ca7e0`) |
| `0x1801ca7e0` | 50 | reset a HID device's output: pick the report id and length by type (DS4 `0x11`/BT, DualSense `0x02`/48), check `HidD_GetPreparsedData`, send |
| `0x1801ca9d0` | 11 | mark HID device `n` for re-initialisation (`+0x178` = 1) under the device lock |
| `0x1801caa40` | 223 | read a feature-backed record for HID device `n` (two entries), under the device lock |
| `0x1801caeb0` | 33 | copy the product string of HID device `n` (`wcsncpy_s`) |
| `0x1801caf90` | 7 | `HidD_GetFeature` wrapper; `0x8001002b` on failure |
| `0x1801cb010` | 67 | read the 16-bit firmware/config word of HID device `n` |
| `0x1801cb6d0` | 34 | send a raw output block of up to 42 bytes with header `0x0e08` (and BT CRC seed `0x53` when asked, `0x1801d4490`) |
| `0x1801cc640` | 12 | store the global volume word `0x1805b1574` and mark it set |
| `0x1801cc670` | 2 | setter of `0x1805b1571` |
| `0x1801cc680` | 2 | setter of `0x1805b1572` |
| `0x1801cc690` | 44 | stop libpad: send "all off", stop the scanner and writer threads (5 s wait, then `TerminateThread`), close handles |
| `0x1801cc830` | 7 | add an event handle to the 12-entry wait list (`0x80010037` when full) |
| `0x1801cc9f0` | 12 | input-report length check by family |
| `0x1801cca80` | 75 | DS4 firmware ≥ 19: search the HID capability list for the vendor usage that marks a newer report layout |
| `0x1801ce480` | 65 | writer thread step: for each device with pending output, write it |
| `0x1801ce6c0` | 14 | close and clear one HID device record (0x2200 bytes) |
| `0x1801ce730` | 42 | remove all map entries that belong to device `n` |
| `0x1801ce8a0` | 7 | remove an event handle from the wait list |
| `0x1801cf8a0` | 21 | find a HID device by its 6-byte address |
| `0x1801cfb90` | 20 | read a device registry property (`SetupDiGetDeviceRegistryPropertyW`, property `0x24`) |
| `0x1801d1ce0` | 7 | true when every entry of the 4-slot table at `0x1805d2e40` is empty |
| `0x1801d23c0` | 109 | decode a trigger-effect type nibble (0..15) into the output flag bits at `+0x14` [likely] |
| `0x1801d27f0` | 8 | store a global value `0x1805e3498` and mark it set |
| `0x1801d2820` | 2 | setter of `0x180351661` |
| `0x1801d2830` | 7 | set the libpad free hook (default `free`) |
| `0x1801d2860` | 7 | set the libpad malloc hook (default `malloc`) |
| `0x1801d2880` | 2 | setter of the mode word `0x1805b157c` |
| `0x1801d28a0` | 44 | build a feature/output command `0x80 a b` with up to 57 payload bytes and send it; logs `SCE_PAD_ERROR_FATAL` when the lock fails |
| `0x1801d2ef0` | 18 | send the global volume to HID device `n` (Edge firmware ≥ `0x110`, DualSense) |
| `0x1801d2fb0` | 167 | rebuild the device-path map from a scan (`std::map<wstring, …>`) [likely] |
| `0x1801d3750` | 36 | VR controller: write the vibration levels into output byte 35 (`valid_flag1` bit 4) |
| `0x1801d3870` | 45 | read a 3+-byte feature from a DualSense / Edge / VR device |
| `0x1801d3a60` | 6 | set the timer period (`uPeriod`) when non-zero |
| `0x1801d3a80` | 29 | decide whether a DualSense / Edge output needs the compatibility flags (connection kinds 2..5) |
| `0x1801d3b90` | 108 | register a newly found HID device (1..12) into a free slot, copying its identity |
| `0x1801d4080` | 59 | post the device's pending events to the wait list (4 or 8 handles by family) |
| `0x1801d4360` | 4 | delete the libpad critical sections |
| `0x1801d4610` | 6 | read the pad settings word (`+0x6c`) of a handle |
| `0x1801d4650` | 4 | init a {0, 1.0} record (orientation w) |
| `0x1801d4660` | 4 | zero a 16-byte record |
| `0x1801d4670` | 5 | init a quaternion record to identity |
| `0x1801d4680` | 6 | write the pad settings word of a handle |
| `0x1801d46b0` | 6 | write the pad word at `+0x150` |
| `0x1801d46f0` | 47 | rotate the reported orientation by a correction when the gravity vector is near unit length (‖g‖ within 0.08 of 1) [likely] |
| `0x1801d5750` | 28 | 3×3 matrix inverse (adjugate / determinant) |
| `0x1801d59e0` | 18 | per-axis difference of a vector from a stored reference |

---

## Part G: Headset USB glue, hand skeleton and jsoncpp

The classifier put these in `map+steamvr`. Most of the range
`0x180135d10`–`0x18013f420` is the statically linked **jsoncpp** (it was
not matched by the MSVC FLIRT sets); the rest is the headset USB thread
classes, getters of the headset state, the hand skeleton and a few room
helpers. jsoncpp value types: 0 null, 1 int, 2 uint, 3 real, 4 string,
5 bool, 6 array, 7 object.

### G.1 Headset USB threads, state getters, room glue [sure unless tagged]

| Function | Lines | Role |
|---|---|---|
| `0x1801200d0` | 64 | pose record → four-row matrix (quaternion at `+0xc` to rotation, rows initialised to identity); caller `0x1801214a0` |
| `0x180120e80` | 25 | VrTracker2 reset under its mutex: index −1, flags cleared, state 3, log `[VrTracker2][%s][%d] pass, mapid:(…), uniid:(…)` |
| `0x1801211c0` | 18 | under the tracker mutex, latch the current map id (`+0x8e8` ← `+0x50`) and set the "pending" flag `+0x2390`; caller `0x1801a4040` |
| `0x1801212a0` | 36 | destroy the global room object `0x180354ca0` (`0x180120f20`, then free its vectors); caller `0x1801a2a40` |
| `0x180122030` | 43 | `CaesarManager` destructor: stop the clock thread (`0x18017afa0`) and delete the seven USB thread objects |
| `0x1801220f0` | 12 | `CaesarUsbThread` base destructor (thread, event, mutex); unwind funclets call it |
| `0x180122220` | 20 | `CaesarUsbThreadImuStatus` destructor: set stop flag `+0x1e0`, abort pipe `0x88`, then the base |
| `0x1801222a0` | 21 | `CaesarUsbThreadLeddet` destructor (pipe `0x89`) |
| `0x1801223b0` | 21 | `CaesarUsbThreadRelocPre` destructor (pipe `0x8a`) |
| `0x180122440` | 20 | `CaesarUsbThreadSlamTracking` destructor (pipe `0x83`) |
| `0x1801224c0` | 43 | `CaesarManager` deleting destructor |
| `0x180123930` | 15 | `CaesarUsbThreadGenData` stop: stop flag, abort pipe `0x8b` |
| `0x180123980` | 15 | `CaesarUsbThreadImage` stop (pipe `0x87`) |
| `0x1801239d0` | 15 | `CaesarUsbThreadImuStatus` stop (pipe `0x88`) |
| `0x180123a20` | 15 | `CaesarUsbThreadLeddet` stop (pipe `0x89`) |
| `0x180123b10` | 15 | `CaesarUsbThreadSlamTracking` stop (pipe `0x83`) |
| `0x180124610` | 17 | copy the 0x1084-byte calibration record stored at GenData `+0x42d50` and its 16-bit counter |
| `0x180124660` | 17 | copy the 0x1084-byte record at GenData `+0x41ccc` and its counter |
| `0x180124870` | 14 | read one status byte (`+0x4e6e` of the IMU/status thread) under its lock; callers `0x180132890`, `0x180134730`, `0x18018aaf0`, `0x180190300` |
| `0x180124b50` | 29 | copy the newest buffered image frame (2 MiB slots, index `+0x200424`) and its counter |
| `0x180124d50` | 19 | copy the newest LD packet (0x9050 bytes at `+0x9280`) and its counter; callers `0x180118920`, `0x18018aaf0` |
| `0x180124dd0` | 23 | microseconds since the last status packet (0 if none yet); callers `0x180126b80`, `0x1801270c0`, `0x180128030` |
| `0x1801250d0` | 59 | copy four 128-byte per-camera records (`+0x420`) and a counter |
| `0x180125200` | 17 | read a status word (`+0x280`) under the lock |
| `0x180125240` | 18 | copy a 128-character string from the status thread (`+0x15c`); callers `0x1801a02b0`, `0x1801a47c0` |
| `0x1801252e0` | 14 | read the last status timestamp (`+0x4b40`); caller `0x180118920` |
| `0x180127df0` | 27 | running min / max / sum / count of an int (latency statistics) |
| `0x180128030` | 81 | image-stream packet: copy the frame into a VrTrace buffer (`0x18015a070`) when capture is on, stamp it with the headset clock (`0x180124c90`, `0x180124dd0`), update the statistics; caller `0x180125e00` |
| `0x180128670` | 48 | `vector<int>` reallocate-and-insert; caller `0x18012af70` |

### G.2 Hand skeleton [sure unless tagged]

| Function | Lines | Role |
|---|---|---|
| `0x180128770` | 22 | factory: new `HandSkeletonWithController` (0x1510 bytes), bind pose table `0x18034d880` |
| `0x1801287f0` | 22 | factory: new `HandSkeletonWithoutController`, pose table `0x18034d2b0` |
| `0x1801289a0` | 98 | `HandSkeletonBase` constructor: hand id, 31 bones × two pose sets (identity rotations, zero offsets) |
| `0x180128b60` | 11 | in-place `HandSkeletonWithController` constructor |
| `0x180128ba0` | 11 | in-place `HandSkeletonWithoutController` constructor |
| `0x180128be0` | 15 | bone transform reset (identity quaternion, zero position) |
| `0x180128c60` | 21 | release two owned objects (`+0x10`, `+0x18`) through their virtual destructors |
| `0x180128cd0` | 21 | `HandSkeleton` deleting destructor |
| `0x180128d30` | 11 | `HandSkeletonBase` deleting destructor |
| `0x180128d60` | 11 | `HandSkeletonWithController` deleting destructor |
| `0x180128d90` | 11 | `HandSkeletonWithoutController` deleting destructor |
| `0x180128e50` | 30 | add the per-bone offset table to the 31 bone positions and refresh [likely] |
| `0x180128f90` | 40 | per-frame skeleton update: read inputs (`0x180128f00`), run the two virtual steps, then add bone offsets |
| `0x1801293a0` | 8 | `HandSkeletonBase` empty virtual |
| `0x180129c10` | 8 | `HandSkeletonBase` empty virtual |
| `0x18012b860` | 175 | static init of the timing/state block used by `0x18012bc00` (thread-safe statics, clock reads) [likely] |
| `0x18012bc00` | 28 | map a state from `0x18012b860` and two flags to a 0..4 result code; callers `0x180134800`, `0x180160230` [likely] |
| `0x18012c0f0` | 96 | `operator<<(ostream&, const char*)` with width padding (MSVC STL instance) |

### G.3 jsoncpp [sure for the identity; the function names are jsoncpp's]

| Function | Lines | Role |
|---|---|---|
| `0x180135d10` | 74 | `std::string operator+(const char*, const std::string&)` |
| `0x180135e90` | 9 | trivially-copyable move of a range (memmove) |
| `0x180135ec0` | 9 | same |
| `0x180135ef0` | 9 | same |
| `0x180135f20` | 9 | same |
| `0x180135f50` | 35 | copy a `std::map<CZString, Value>` subtree (node copy: key `0x180138220`, value `0x180138cf0`) |
| `0x180136040` | 30 | destroy a range of `std::string` |
| `0x1801360d0` | 30 | same, other element stride |
| `0x180136160` | 103 | `vector<48-byte>` (Reader error record) reallocate-and-insert |
| `0x1801363e0` | 103 | same for the second reader |
| `0x180136660` | 64 | `vector<std::string>` reallocate-and-insert |
| `0x1801367d0` | 54 | `vector<PathArgument>` (40 bytes) reallocate-and-insert |
| `0x180136940` | 87 | `vector<Value*>` reallocate-and-insert |
| `0x180136ae0` | 87 | `vector<std::string>` reallocate-and-insert (second instance) |
| `0x180136c70` | 77 | `vector<PathArgument>` reallocate-and-insert (second instance) |
| `0x180136e00` | 118 | `std::map<std::string, …>` find |
| `0x180136f90` | 23 | `std::map<CZString, Value>` recursive node destroy |
| `0x180137020` | 37 | `std::map<std::string, …>` recursive node destroy |
| `0x1801370e0` | 23 | `std::map<CZString, Value>` recursive destroy (second instance) |
| `0x180137170` | 132 | map insert-position search with hint (`CZString <`) |
| `0x1801373d0` | 59 | map lower bound by string key |
| `0x1801374a0` | 49 | `valueToString(int)` |
| `0x180137580` | 160 | object compare: lexicographical `<` over (key, value) pairs |
| `0x1801377d0` | 42 | move a range of 48-byte error records |
| `0x180137850` | 42 | same, second reader |
| `0x1801378d0` | 46 | move a range of `std::string` |
| `0x180137950` | 50 | move a range of `PathArgument` |
| `0x180137ad0` | 74 | object equality over (key, value) pairs |
| `0x180137c20` | 66 | map emplace by string key |
| `0x180137d40` | 51 | map emplace by `CZString` key (`Value::resolveReference` path) |
| `0x180137ea0` | 9 | memmove range helper |
| `0x180137ed0` | 9 | same |
| `0x180137f00` | 9 | same |
| `0x180137f30` | 26 | `std::ostringstream` constructor |
| `0x180137fe0` | 72 | `std::stringbuf` constructor from a string and mode |
| `0x180138130` | 28 | `BuiltStyledStreamWriter` constructor (indentation, colon, null symbol, end-line, comment style, precision) |
| `0x180138220` | 25 | `Value::CZString` copy constructor (duplicates owned strings, `0x180140eb0`) |
| `0x18013a0e0` | 35 | `Value::swap` (payload, type, allocated flag) |
| `0x18013a1c0` | 82 | `Json::parseFromStream` with a default `CharReaderBuilder`; logs `Error from reader: %s` |
| `0x18013a3b0` | 50 | `Json::writeString` with a default `StreamWriterBuilder` |
| `0x18013a520` | 22 | `CZString ==` |
| `0x18013a570` | 69 | `Value ==` (per type; arrays and objects through `0x180137ad0`) |
| `0x18013a6d0` | 44 | `Value::operator[](const std::string&)` |
| `0x18013a790` | 44 | same (const instance) |
| `0x18013a850` | 13 | `operator[](std::string)` forwarding to `resolveReference(begin, end)` |
| `0x18013ab50` | 15 | `operator[](const char*)` |
| `0x18013ab80` | 22 | `Value::find`-based const `operator[]`, returns the static null value when missing |
| `0x18013ad70` | 18 | same for `const char*` |
| `0x18013adb0` | 36 | map iterator decrement |
| `0x18013ae20` | 48 | map iterator increment |
| `0x18013aeb0` | 30 | `CZString <` |
| `0x18013af20` | 89 | `Value <` (per type) |
| `0x18013b150` | 30 | `CZString <` (comparator instance) |
| `0x18013bdf0` | 88 | `std::deque` map growth (Reader node stack) |
| `0x18013bfd0` | 88 | same, error deque |
| `0x18013c1b0` | 88 | same, second reader's error deque |
| `0x18013c390` | 177 | red-black tree insert-and-rebalance |
| `0x18013c610` | 177 | same, other map |
| `0x18013c8f0` | 33 | `vector<std::string>` reserve |
| `0x18013ca10` | 45 | clear a deque of error records |
| `0x18013cb00` | 45 | same, second reader |
| `0x18013ccf0` | 43 | `Reader::addComment` (normalise end-of-line, attach to the current value) |
| `0x18013cdb0` | 43 | same, `OurReader` |
| `0x18013ce70` | 55 | `Reader::addError` |
| `0x18013cf50` | 55 | `OurReader::addError` |
| `0x18013d030` | 10 | `addError` then `recoverFromError` |
| `0x18013d060` | 10 | same, `OurReader` |
| `0x18013d090` | 24 | collect one `PathArgument` of the wanted kind |
| `0x18013d130` | 32 | allocate `n` 32-byte items |
| `0x18013d1a0` | 32 | allocate `n` 40-byte items |
| `0x18013d220` | 17 | `Value::append` |
| `0x18013d270` | 30 | `Value::asBool` |
| `0x18013d370` | 38 | `Value::asDouble` |
| `0x18013d3e0` | 38 | `Value::asFloat` |
| `0x18013d460` | 56 | `Value::asInt64` (error "LargestUInt out of …" via the stream helpers) |
| `0x18013d580` | 65 | `Value::asInt` ("LargestInt out of Int range") |
| `0x18013d710` | 73 | `Value::asString` |
| `0x18013d880` | 59 | `Value::asUInt64` |
| `0x18013d9e0` | 64 | `Value::asUInt` |
| `0x18013db50` | 16 | first element of a non-empty array/object, else a null marker |
| `0x18013dcd0` | 133 | `codePointToUTF8` |
| `0x18013de80` | 15 | `Value::compare` (−1 / 0 / 1 from `<`) |
| `0x18013e0e0` | 18 | deallocate `n` 32-byte items |
| `0x18013e180` | 83 | `Reader::decodeNumber` (token → value) |
| `0x18013e760` | 83 | `OurReader::decodeNumber` |
| `0x18013eed0` | 83 | `Reader::decodeNumber` integer path wrapper |
| `0x18013f0e0` | 70 | integer token parse with overflow check (max/10 rule) |
| `0x18013f210` | 83 | `OurReader` integer path wrapper |
| `0x18013f420` | 70 | `OurReader` integer token parse (`LargestInt` limits) |

---

## Part H: ShareManager and ConfigManager glue

The classifier put these in `config`. Nearly all of them are methods of
the `ShareManager` singleton (`0x180354f78`, created by `0x18015bbd0`,
0x2430 bytes). It owns the named shared memory (`SHARE_VRT2_*`) that
the driver shares with Sony's PS VR2 app. Its base pointer is at object
`+0x23e0`. The block is split into sections. Each section has a mutex
handle in the object (`+0x22b0..+0x23d0`), a sequence counter in shared
memory and a payload just after it. A setter copies the payload in,
increments the counter and returns the new value. A getter copies the
payload out and returns (or writes through a pointer) the counter, so a
reader can tell a new value from an old one. The few non-ShareManager
rows are the `ConfigManager` singleton (`0x180354f68`, 0x1570 bytes).

Object fields used below [sure]: `+0x08 + 8·i` per-setting mutexes
(i < 11), `+0x70` INI file path (`std::string`), `+0x90 + 0x300·i` the
settings table (section name, key name and cached value, 0x100 bytes
each, 11 entries), `+0x2190` 36 auto-reset events, `+0x23f8`/`+0x2404`
slot sequence counters, `+0x2414`/`+0x2420` "last written slot" indexes.

### H.1 Shared-memory sections (counter / payload offsets in the block) [sure unless tagged]

| Section | Mutex | Counter | Payload | Writer → reader |
|---|---|---|---|---|
| general | `+0x22b0` | `+0x08` | qword `+0x10` | (no writer seen) → `0x18015cf20` |
| headset status info | `+0x22b8` | `+0x28` | 0x24 bytes `+0x2c` | `0x18015fc00` → `0x18015db30` |
| `0x8f` display/lens block | `+0x22c0` | `+0x54` | 0x100 bytes `+0x70` | `0x18015e990` → `0x18015cc60` |
| VD type 0x12 | `+0x22c0` | `+0x58` | 0x200 bytes `+0x170` | `0x18015ea40` → `0x18015cd30` |
| VD type 12 (headset IMU calibration) | `+0x22c0` | `+0x5c` | 0x78 bytes `+0x370` | `0x18015e850` → `0x18015cb00` |
| VD type 13 (IMU set calibration) | `+0x22c0` | `+0x60` | 0x34 bytes `+0x3e8` | `0x18015e910` → `0x18015cbc0` |
| parsed camera calibration 0 | `+0x22c0` | `+0x64` [likely] | 0xf0 bytes `+0x41c` | `0x18015f610` → (none seen) |
| VD type 10 raw (camera calibration 0) | `+0x22c0` | `+0x68` | 0x800 bytes `+0x50c` | `0x18015eb00` → `0x18015ce00` |
| VD type 0x11 | `+0x22c0` | `+0x6c` | 0x800 bytes `+0xd0c` | `0x18015eb70` → `0x18015ce90` |
| controller slots, per side | `+0x22c8 + 8·side` | per slot | 64 × 12 bytes from `+0x150c + 0x300·side` | (app) → `0x180159b60`, `0x18015b550` |
| pose slots, per index | `+0x22e0 + 8·i` | per slot | 64 × 40 bytes from `+0x1e10 + 0xa00·i` | `0x18015b830` → `0x180159dc0`, `0x18015b600` |
| image slots | `+0x22f8` | per slot | 8 headers of 0x878 bytes from `+0x3c10`; images of 0x200100 bytes | `0x18015b6a0` → `0x180159970` |
| camera mode request | `+0x2310` | `+0x8038` | 0x10 bytes `+0x803c` | `0x18015f6f0` → `0x18015d550` |
| LED-detection window | `+0x2318` | `+0x8054` | 0x18 bytes `+0x8058` | `0x18015e720` → `0x18015c9d0` |
| LED config | `+0x2320` | `+0x8078` | 0x10 bytes `+0x807c` | `0x18015f850` → `0x18015d6b0` |
| tracker settings | `+0x2328` | — | 0x400 bytes `+0x8098` | `0x18015f110` → `0x18015cfd0` |
| per-index record A | `+0x2330` | `+0x84a0 + 0x5c·i` | 0x50 bytes after it | `0x18015e670` → (none seen) |
| 0xc00-byte block | `+0x2338` | `+0x8558` | 0xc00 bytes `+0x855c` | `0x18015f1d0` → `0x18015d090` |
| per-index record B | `+0x2340` | `+0x9168 + 0x60·i` | 0x50 bytes `+0x9170 + 0x60·i` | `0x18015e7a0` → `0x18015ca50` |
| 0x44-byte record | `+0x2348` | `+0x9228` | 0x44 bytes `+0x922c` | `0x18015f330` → `0x18015d1e0` |
| 0x34-byte record | `+0x2360` | `+0x9b38` | 0x34 bytes `+0x9b3c` | `0x18015e5e0` → (none seen) |
| 0x13c-byte record [likely size] | `+0x2370` | — | from `+0x9c8c` | `0x18015f250` → `0x18015d100` |
| qword value | `+0x22b0` | `+0x9dc8` | qword `+0x9dd0` | `0x18015e520` → (none seen) |
| settings strings | `+0x08 + 8·i` | `+0x9dfc + 0x10c·i` | string `+0x9e00 + 0x10c·i` | `0x18015f470` → `0x18015d270` |
| 0x110-byte record | `+0x23a0` | `+0xa980` | 0x110 bytes `+0xa984` | `0x18015fc70` → `0x18015dba0` |
| IR-thread block | `+0x23a8` | `+0xaa9c` | 0x1b50 bytes `+0xaaa0` | `0x18015fd40` → `0x18015dc60` |
| IR-thread record | `+0x23b0` | `+0xc5f8` | 0x120 bytes `+0xc600` | `0x18015fdc0` → `0x18015dcd0` |
| byte flag | `+0x22b0` | `+0xc728` | byte `+0xc72c` | `0x18015e580` → (none seen) |
| large record | `+0x23c0` | — | from `+0xc73c` | `0x18015f760` → `0x18015d5d0` |
| client process table | `+0x22b0` | — | 11 entries from `+0xffc00`, process ids at `+0xffc58` | (clients) → `0x18015dda0` |
| app-held locks | `+0x2380 + 8·i` (i = 0, 1) | — | — | `0x18015e260` / `0x18015e4f0` / `0x18015c610` |

"(none seen)" means no caller in the driver; the reader is Sony's app or
nobody. The VD types are the headset `"VD"` calibration entries
(overview.md part B). The meaning of the unnamed records was not traced.

### H.2 Function rows [sure unless tagged]

| Function | Lines | Role |
|---|---|---|
| `0x180154910` | 136 | `ConfigManager` constructor: vtable, inner record (`0x180154be0` at `+0x0c`), zeroes the arrays, writes five small headers (3, 3, 24) from `+0xa30` and four more from `+0xf80` [likely: matrix dimensions]; callers `0x180156470`, `0x1801564b0` |
| `0x180154be0` | 138 | constructor of the `ConfigManager` inner record: zeroes the fields, builds the identity transforms (`0x18016b940`), float 1.0 at record `+0x48c` (quaternion w [likely]); caller `0x180154910` |
| `0x1801564b0` | 21 | create the `ConfigManager` singleton (0x1570 bytes, `0x180154910`) if it is missing and set the flag `0x180354f70`; then (re)load the defaults (`0x180155000`); caller `ServerDriver::Init` `0x1801a2040` |
| `0x1801577d0` | 12 | destroy the `ConfigManager` singleton (virtual destructor with free) and clear the pointer; caller `ServerDriver::Cleanup` `0x1801a2a40` |
| `0x180159970` | 85 | under the image mutex, pick the newest image slot of 8 (state 1 or 2, highest sequence) and copy out its header and image pointer; no caller in the driver [likely: the app's reader] |
| `0x180159b60` | 103 | under the side's controller mutex, pick the newest of 64 controller slots (state 1 or 2, highest sequence), take a reference and copy it out; caller `0x18011bae0` (controller poll) |
| `0x180159dc0` | 116 | same for the 64 pose slots of index `i` (40-byte slots); caller `0x18011f1b0` (room: headset pose from shared memory) |
| `0x18015b550` | 26 | release a reference to controller slot (side, n ≤ 63): decrement its count and mark it free (state 1) at zero; caller `0x18011bae0` |
| `0x18015b600` | 23 | same for pose slot (i, n ≤ 63); caller `0x18011f1b0` |
| `0x18015b6a0` | 37 | publish image slot n ≤ 7: state 1, next sequence (`+0x2404`), copy the image's size and a float (1.0 if the size is below 0x100), remember the slot, signal event `+0x21d8`; caller `0x180128030` (image-stream packet) |
| `0x18015b830` | 35 | publish a 24-byte pose into slot n ≤ 63 of index `i`: state 1, next sequence (`+0x23f8 + 4·i`), remember the slot, signal event `+0x21c0 + 8·i`; callers `0x18019ee30`, `0x1801a7bf0` (SteamVR poses) |
| `0x18015b900` | 18 | signal all 36 events (wakes every waiter, used on shutdown [likely]); no caller in the driver |
| `0x18015b950` | 56 | parse the cached value of setting `i` (object `+0x290 + 0x300·i`) with `stoi` into an int; caller `0x1801256c0` (headset init) |
| `0x18015bab0` | 68 | directory part of a path: everything before the last `/` or `\` (empty string if none); no caller in the driver |
| `0x18015bc80` | 20 | `QueryPerformanceCounter` time in microseconds as an unsigned 64-bit value; callers `0x180128030`, `0x1801a02b0` |
| `0x18015c610` | 23 | test app-held lock `i` ≤ 1: returns 1 if another owner holds it (zero-timeout wait fails), else releases it again and returns 0; callers `0x18011bae0`, `0x18011d0c0` (controller access) |
| `0x18015c9d0` | 29 | read the LED-detection window section (0x18 bytes) and its counter; caller `0x180126b80` (Set `0x14`) |
| `0x18015ca50` | 50 | read per-index record B (0x50 bytes) for index `i`, return its counter; callers `0x180133960` (BT notice), `0x18018aaf0` (IR tracker) |
| `0x18015cb00` | 48 | read the VD type 12 section (0x78 bytes) and its counter; no caller in the driver |
| `0x18015cbc0` | 36 | read the VD type 13 section (0x34 bytes) and its counter; no caller in the driver |
| `0x18015cc60` | 59 | read the `0x8f` display/lens section (0x100 bytes) and its counter; no caller in the driver |
| `0x18015cd30` | 59 | read the VD type 0x12 section (0x200 bytes) and its counter; no caller in the driver |
| `0x18015ce00` | 19 | read the VD type 10 section (0x800 bytes) and its counter; no caller in the driver |
| `0x18015ce90` | 19 | read the VD type 0x11 section (0x800 bytes) and its counter; no caller in the driver |
| `0x18015cf20` | 16 | read the general qword `+0x10` and return counter `+0x08`; no caller in the driver |
| `0x18015cf80` | 17 | `GetPrivateProfileStringA` for setting `i`: section and key from the settings table, 256-byte output, file = the INI path at `+0x70`; no caller in the driver |
| `0x18015d090` | 16 | read the 0xc00-byte block and return its counter; no caller in the driver |
| `0x18015d100` | 73 | read the record at `+0x9c8c`; no caller in the driver |
| `0x18015d1e0` | 35 | read the 0x44-byte record and return its counter; no caller in the driver |
| `0x18015d270` | 64 | read settings string `i` from shared memory under its mutex and parse it with `stoi`; returns the counter. Callers `0x18011bae0`, `0x18011d580`, `0x1801268d0`, `0x18012f3a0`, `0x18012f4f0`, `0x180132890` and more |
| `0x18015d550` | 28 | read the camera mode request (0x10 bytes) and its counter; caller `0x180125cc0` (camera mode resend) |
| `0x18015d5d0` | 64 | read the large record at `+0xc73c`; no caller in the driver |
| `0x18015d6b0` | 28 | read the LED config section (0x10 bytes) and its counter; caller `0x180126b80` (Set `0x14`) |
| `0x18015db30` | 32 | read the headset status-info record (0x24 bytes), return its counter; callers `0x180132890`, `0x180134730`, `0x180160230` |
| `0x18015dba0` | 66 | read the 0x110-byte record at `+0xa984`, return its counter; callers `0x18011bae0`, `0x180120290`, `0x180122ac0`, `0x1801a2040` |
| `0x18015dc60` | 16 | read the 0x1b50-byte IR-thread block, return its counter; caller `0x1801910f0` |
| `0x18015dcd0` | 69 | read the 0x120-byte IR-thread record, return its counter; caller `0x1801910f0` |
| `0x18015dda0` | 90 | client process table: for each of 11 entries owned by another process, clear it if `GetExitCodeProcess` says the process has ended (not 259 = still active); then copy the table out. Caller `0x1801a02b0` (HMD debug page) |
| `0x18015e260` | 13 | take app-held lock `i` ≤ 1 (infinite wait); returns −1 for a bad index; caller `0x18011d0c0` |
| `0x18015e2a0` | 13 | under mutex `+0x2398`, clear the dword at shared-memory `+0x110c408`; caller `0x1801a2040` |
| `0x18015e2f0` | 17 | post event bits: OR the argument into the flags word of the event record at `+0x23d0`, then signal its event; no caller in the driver [likely: the companion-app event bits of room.md §A.4.3] |
| `0x18015e360` | 15 | destroy the `ShareManager` singleton: close it (`0x180159770`), virtual destructor with free, clear the pointer; caller `0x1801a2a40` |
| `0x18015e3a0` | 49 | `std::to_string(int)`: decimal digits with a `-` sign; callers `0x18015ad30`, `0x18015f3d0` |
| `0x18015e4a0` | 16 | wait for event `i` with a timeout and reset it; returns −1 on timeout; callers `0x18011bae0`, `0x1801a4e60` |
| `0x18015e4f0` | 12 | release app-held lock `i` ≤ 1; no caller in the driver |
| `0x18015e520` | 17 | write the qword at `+0x9dd0` and bump counter `+0x9dc8`; no caller in the driver |
| `0x18015e580` | 17 | write the byte flag at `+0xc72c` and bump counter `+0xc728`; no caller in the driver |
| `0x18015e5e0` | 38 | write the 0x34-byte record at `+0x9b3c` and bump its counter; no caller in the driver |
| `0x18015e670` | 65 | write per-index record A (0x50 bytes) for index `i` and bump its counter; caller `0x180160230` |
| `0x18015e7a0` | 55 | write per-index record B (0x50 bytes) for index `i` and bump its counter; caller `0x18011a870` |
| `0x18015e850` | 46 | write the VD type 12 section (0x78 bytes) and bump its counter; caller `0x180127250` (VD parser, case 12) |
| `0x18015eb00` | 17 | write the VD type 10 section (0x800 bytes) and bump its counter; caller `0x180127250` (case 10) |
| `0x18015f1d0` | 17 | write the 0xc00-byte block and bump its counter; caller `0x1801617c0` |
| `0x18015f250` | 76 | write the record at `+0x9c8c` and bump its counter; no caller in the driver |
| `0x18015f330` | 37 | write the 0x44-byte record and bump its counter; caller `0x1801256c0` (headset init) |
| `0x18015f3d0` | 31 | set setting `i` from an int: `std::to_string` (`0x18015e3a0`), then the string setter `0x18015f470`; callers `0x18011bae0`, `0x18012f4f0`, `0x180134280` |
| `0x18015f610` | 75 | write the parsed camera calibration 0 record (0xf0 bytes, from `ConfigManager` `0x1801562f0`) and bump its counter; caller `0x180127250` (case 10) |
| `0x18015f6f0` | 26 | write the camera mode request (0x10 bytes) and bump its counter; callers `0x180125670` (seed, mode `0x10`), `0x180127ec0` (debug) |
| `0x18015f760` | 84 | write the large record at `+0xc73c`; no caller in the driver |
| `0x18015f850` | 26 | write the LED config section (0x10 bytes) and bump its counter; callers `0x180125a10` (seed), `0x180127f10` |
| `0x18015fc00` | 41 | write the headset status-info record (0x24 bytes) and bump its counter; caller `0x180128240` (headset.md §A.2.7) |
| `0x18015fc70` | 80 | write the 0x110-byte record at `+0xa984` and bump its counter; callers `0x18011bae0`, `0x180120290`, `0x180122ac0`, `0x1801a2040` |
| `0x18015fd40` | 17 | write the 0x1b50-byte IR-thread block and bump its counter; caller `0x1801910f0` |
| `0x18015fdc0` | 87 | write the 0x120-byte IR-thread record and bump its counter; caller `0x1801910f0` |
