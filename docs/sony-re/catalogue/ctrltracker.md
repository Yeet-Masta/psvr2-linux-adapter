# Function catalogue: ctrl-tracker UKF internals (`0x180070950`–`0x18009cfff`)

These functions were first labelled as a linear-algebra library
(`catalogue/lapack.md` explains the mix-up). They are the controller UKF of
`ukf.md`: the generic sigma-point solver, fixed-size unscented transforms,
their double-precision matrix kernels, the per-sample IMU path and the
filter (re)initialisation. Own words; tags as elsewhere. Sizes are
decompiled lines.

Matrices are small Sony C++ objects: `{double* data, int rows, int cols}`
(dense) or `{double* data, int n}` (packed upper triangle, n(n+1)/2
entries, index of (i, j) with i ≤ j is `j + i·(2n − i − 1)/2`). They are
allocated through an allocator struct of two callbacks (alloc, free)
[sure].

## 1. Generic solver `0x1800942f0` (858 lines) [sure unless tagged]

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
   (`catalogue/lapack.md`). If `DPPTRF` reports a negative info → return
   **−4**. If it reports a positive info (S not positive definite) the
   code skips `DPPTRI` and carries on with the half-factored matrix
   [sure for the branch; a port should treat it as a failure].
8. K = Pxy·S⁻¹ (`0x180090b50`), innovation ν = z − ŷ (`0x18008f950`),
   x ← x + K·ν (`0x18008f740` on the 15 state rows),
   P ← P − K·S·Kᵀ (`0x180091010`), stored back packed (`0x180090850`).
   Return 0.

This corrects `ukf.md` §2, which said there is no central point.

### Solver kernels [sure for the operation, from the element loops]

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

### Work-matrix setup [likely]

| Function | Lines | Role |
|----------|-------|------|
| `0x180092020` | 105 | allocate the solver workspace (sigma-point, mean and covariance matrices) for a given state and noise size; callers are the filter ctor and reset paths (`0x18005d970`, `0x180063d30`, `0x18006d890`) |
| `0x1800928f0` | 55 | free that workspace |
| `0x180092300` | 75 | allocate 6-sized blocks (one packed 6 × 6, 21 doubles) and zero them |
| `0x180092560`, `0x180092630` | 29, 36 | build the IMU noise block (`0x180091700`) / the measurement noise block (`0x180091930`, `ukf.md` §2) |
| `0x180091700`, `0x180091930` | 100, 194 | allocate and fill those blocks |
| `0x180092a60`, `0x180096bb0`, `0x180096c80` | 20–39 | wrappers over the above with no caller |
| `0x180091580`, `0x180091d70`, `0x180091eb0` | 51–62 | further block allocators with no caller |
| `0x180092b00`, `0x1800936e0` | 501, 513 | two more copies of the generic solver with a different callback signature; only `.pdata` refers to them, so no code reaches them in this build [sure for reachability] |

## 2. Fixed-size unscented transforms [sure for sizes and callers]

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

The unreachable tilt filter of §4 (`0x18007dc60`) has its own two
transforms, `0x180097990` (158 lines) and `0x180097d90` (245), with the
same kernel set [likely]: block copy `0x180098370`, `0x1800984c0`;
Cholesky `0x1800985e0`; sigma point `0x180098760` (`0x1800989d0`* the
subtracting copy); mean `0x180098c40`; outer-product accumulate
`0x180098e10`; cross covariance `0x180098ba0`; update `0x180098f40`,
`0x180098ff0`, `0x180099040`; and a callback `0x180097340` that rotates a
vector by the sigma point's attitude (`0x180058720`). `0x180097110` and
`0x1800975f0` are the two correction calls of `0x18007dc60` into them.

## 3. Measurement step and its callbacks [sure unless tagged]

| Function | Lines | Role |
|----------|-------|------|
| `0x180070d50` | 211 | measurement step called by `kalmanFilterStep` (`0x180065b70`): picks the full-pose path (n = 18, `0x180073c10`) or the position-only path (n = 12, `0x180074540`) by the byte `+0x1a8` of its first argument; converts the state with `0x18006c4c0` (full) or `0x18006ce80` (position-only) first and writes the covariance back with `0x18006c1b0`, the state with `0x18006c760` |
| `0x180070950` | 159 | callback of both paths: maps one sigma point to the predicted measurement, rotating the three 0.1 m axis tips by the point's attitude (`0x180058720`) [likely: the `h(x)` of `ukf.md` §4] |
| `0x180070d10`, `0x180070d30` | 9 | adapters to `0x180070950` for the two sizes |
| `0x1800713b0` | 79 | compose two rotations through the 3×3 product (`0x1800621a0`) [likely] |
| `0x180071670` | 125 | callback of the n = 6 transform: quaternion product (`0x180061fa0`) then `0x1800713b0` [likely] |
| `0x1800719a0` | 683 | pre-update pass from `0x180065760`: predicted pose for the innovation gate (`opticalpose.md` §4.1); uses the relative yaw `0x18006b790` and builds a half-angle rotation (sin, cos), then runs the n = 6 transform on the rotation [likely] |

## 4. IMU sample path and filter set-up [likely unless tagged]

| Function | Lines | Role |
|----------|-------|------|
| `0x180079e10` | 104 | find the ring entry for a timestamp (log tag `LIT::searchOffsetByTimestamp`); returns 0, −1 or −2 [sure for the codes] |
| `0x18007a1f0` | 204 | reset of the per-hand filter object from `0x1800638f0`: clears counters, stores 32 at `+0x308` and two 0.01 constants [sure for the stores] |
| `0x18007c080` | 660 | per-IMU-sample processing called from the sample feed `0x1800641d0`: updates the state ring, may replay (`0x18007add0`), and runs the still / gravity checks `0x180099400`, `0x1800998f0` and `0x18009a1a0` |
| `0x180099400` | 196 | still test: accelerometer magnitude within a tolerance of 1 g and small rotation, with a slow (0.05) running correction [guess for the use] |
| `0x1800998f0` | 430 | fit of accelerometer samples against gravity (0, 1, 0) in g: residual sums per axis [guess for the use] |
| `0x18009a1a0` | 445 | collects samples with µs timestamps (× 1e-6) and calls the fit `0x18007bbd0` with a 0.1 threshold, then `0x18009b7c0` [guess: online accelerometer calibration] |
| `0x18007a9a0` | 217 | push one IMU sample into the ring (from `0x1800641d0`, `0x180068ee0`) through `0x18005a570`, `0x18005a940`, `0x180079750` |
| `0x180081bb0` | 289 | initialise a state from the first valid measurement (from `0x180068620`, `ukf.md` §2): copies the record (`0x180064f40`), attitude from `0x1800813d0` with a 0.999 threshold |
| `0x1800821a0` | 191 | extrapolate a state to a later time: p + v·dt + ½·a·dt², time += dt·10⁶ µs (from `0x180068620`) [sure for the formula] |
| `0x18007e8f0` | 74 | wrapper over `0x180079e10`, `0x180081bb0` and `0x1800821a0`; no caller |
| `0x18007cde0` | 259 | a second copy of the sample feed (log "drop sample due to large gap"); no caller [sure for reachability] |
| `0x18007dc60` | 418 | "correct(tilt)" step of an IMU-only tilt filter (logs "correct(tilt): num of sensor data is larger than internal buffer" and "(i) %p: BadState"); runs its own sigma-point update (`0x180097110` → `0x180097990`, `0x1800975f0` → `0x180097d90`) — **no caller in this build**, so the PC driver never runs a tilt-only correction [sure for reachability] |
