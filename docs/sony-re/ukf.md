# Controller UKF (`UkfManager`): behavioural spec

Interoperability notes on how Sony's driver fuses the optical controller pose
with the controller IMU. Own words, no Sony code. Addresses are in the build
listed in [../sony-driver-re.md](../sony-driver-re.md) (IDA `sub_X` = Ghidra
`FUN_X`). Tags: **[sure]** read from code, **[likely]** from context,
**[guess]** untested.

Earlier notes: [tracking.md](tracking.md) §5.5 (P0, history, stationary
damping, divergence guard), [ctrltracker.md](ctrltracker.md) (process noise,
packed covariance, quaternion kernels), [opticalpose.md](opticalpose.md) §4
(optical feed, innovation gate, config block). This file adds the filter
structure, the measurement model and its noise.

## 1. Objects [sure]

- `UkfManager` is a singleton (0x4460 bytes, vtable `0x180318c80`) holding two
  per-hand filters (0x2218 bytes each, from `+0x28`) and a 32-byte optical
  noise block at `+0x08` (§4). It is set up by `0x18005d5d0`.
- Each per-hand filter has a state ring (0x1c8-byte entries, one per IMU
  sample) and a measurement ring (0x1e0-byte entries).
- A state entry holds, among others [likely for names, sure for offsets]:
  - `+0x00` angular rate and `+0x10` acceleration of that IMU sample (the
    process-noise inputs, `ctrltracker.md`);
  - `+0x40` position, `+0x50` velocity (float3, m, m/s);
  - `+0x60` orientation as **double** quaternion x, y, z, w;
  - `+0xc0`, `+0xd0` gyro and accel bias (float3);
  - `+0x108` time (µs).
- The covariance is the 15×15 packed upper triangle of `ctrltracker.md`.
- An optical measurement is a `DotMarkerData` object (vtable `0x180318c90`,
  base class `OIFusion::OpticalMarkerData`). The filter only talks to it
  through virtual calls (§3), so other measurement types can plug in.

## 2. Filter type [sure unless tagged]

An **error-state, augmented, symmetric sigma-point filter**:

- error state: 15 (position, velocity, attitude, gyro bias, accel bias; block
  order as `tracking.md` §5.5);
- augmented with **6 IMU noise** terms (gyro, accel) and the **m measurement
  noise** terms: n = 15 + 6 + m (`0x1800942f0`, set up by `0x180092630` and
  `0x180091930`);
- **2n + 1 sigma points**: the central point plus `±sqrt(n)` along the
  columns of the Cholesky factor. The unscented transform with α = 1,
  β = 2, κ = 0 (n + λ is computed as `n + 0 − n + n`, so λ = 0): the
  central point has mean weight 0 and covariance weight **2**, the others
  `1/(2n)` for both. So the mean is the cubature mean, but the predicted
  covariances get an extra `2·(y₀ − ŷ)(y₀ − ŷ)ᵀ` term
  (`catalogue/ctrltracker.md` §1; an earlier version of this file said
  there was no central point).

One filter step (`0x180065b70`, "kalmanFilterStep") covers the interval from
the newest filter state to the measurement time in one go:

1. The IMU samples inside the interval are reduced by `0x180082e90` to one
   **time-weighted mean** angular rate and acceleration (bias-corrected with
   the state's biases) plus the mean of their per-sample process-noise
   variances (`0x180062880`, `0x180062950`). Up to 31 samples; if the span
   reaches the config time (`+0x2168` in the manager) the step is split.
2. Each sigma point is propagated with that constant mean rate and
   acceleration over the interval (process callback `0x1800638d0`), the
   averaged variances being the 6 IMU noise terms of the augmentation.
3. The propagated points go through the measurement's `h(x)` (§3); mean,
   innovation covariance and cross covariance give the gain; the correction
   is applied to the nominal state (`0x18006c760`, attitude by the rotation
   vector, `0x18006c1b0` writes the covariance back).
4. If a flag on the filter is set, the accel-bias block of the covariance is
   rewritten from the state after the update.
5. The stationary damping (`0x18006e5a0`, `tracking.md` §5.5) is called
   only when the IMU history reached the split time in step 1, that is on
   long intervals without measurements [sure for the call site].
6. Return codes of the step: 0 ok, −1 numeric failure, −2 a specific
   solver failure. Return codes of the solver `0x1800942f0`: 0 ok, −1 a callback failed,
   −3 the augmented covariance is not positive definite, −4 `DPPTRF` of the
   innovation covariance reported a bad argument. The fixed-size transforms
   return −2 or −3 for a failed Cholesky.

Delayed measurements: a measurement older than the newest state is applied at
its own time and the IMU states after it are replayed (`0x18007add0`,
`tracking.md` §5.5). The first valid measurement initialises the filter
(`0x180068620`) with P0 from `tracking.md` §5.5.

## 3. Measurement interface (`DotMarkerData` vtable) [sure]

| Slot | VA | Use |
|---|---|---|
| +0x08 | `0x18005f910` | position (and zero velocity) if valid |
| +0x10 | `0x18005f950` | measured rotation as a 3×3 matrix (none if position-only) |
| +0x18 | `0x18005fad0` | predicted measurement `h(x)` [likely] |
| +0x20 | `0x18005fc00` | dimension: 0 invalid, 3 position-only, 12 full |
| +0x28 | `0x18005fc20` | scalar measurements and their variances (§4) |
| +0x30, +0x38 | | 2-D and 3-D blocks: none (return 0) |
| +0x40 | | returns 1 |
| +0x50 | | timestamp (`+0x88`, µs) |
| +0x58 | | byte `+0x30` |
| +0x60 | | the vector at `+0x38` (§4) |

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
| +0x38 | float3 argument: the controller position in the head-relative frame used by §4 [likely] |
| +0x48, +0x58, +0x68, +0x78 | one float3 per camera from the tracker frame (stride 0x1688), norms in mm |
| +0x88 | timestamp (µs) |
| +0x90 | copy of the manager's 32-byte noise block |

## 4. Measurement content and noise (`0x18005fc20`) [sure unless tagged]

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

The innovation gate before the update is in `opticalpose.md` §4.1 (5 cm
below 1 m/s, else 20 cm).

## 5. Still open

- The exact sigma-point process model (`0x1800638d0`) and `h(x)`
  (`0x18005fad0`), checked only for their role, not their algebra.
- The UKF solver internals are now in `catalogue/ctrltracker.md` §1:
  packed Cholesky without jitter, inverse of S by `DPPTRF` + `DPPTRI`.
  Open: when `DPPTRF` finds S not positive definite (positive info) the
  code continues with the half-factored matrix instead of failing; whether
  that ever happens in practice is unchecked.
- The manager's split time `+0x2168` and the "use accel bias flag" (`+0x38`
  of the state).
- Config limits listed in `RESEARCH.md` §11 q9.
