# irtracker: headset INS, IR tracker thread, LedSyncer

Behavioural notes for the Sony driver range `0x180170000–0x18019ffff`
(build with PE timestamp `1741940190`). Each entry says what a function
does, in our own words, so it can be reimplemented without looking at the
decompile. Our code lives in `userspace/lib/sie/irtracker/`; per-function
status is in `scripts/sony-re/status/irtracker.tsv`; tests are
`scripts/sony-re/difftest/t_irtracker_*.c`.

Conventions: quaternions are w first (`q[0]` = w). 4x4 matrices are 16
floats. "Order" notes give the summation order where it changes the float
result; two-term sums commute exactly and are not noted.

## Testing method and libm

Sony's DLL imports `sinf`, `cosf`, `atan2f`, `expf`, `asinf`, `pow`, … from
the Universal CRT (`api-ms-win-crt-math`). The mingw libm the tests link
against differs from it by 1 ulp on about 0.15 % of `sinf`/`cosf` inputs
(none on `atan2f`). `irt_difftest.h` therefore routes the libm calls of the
code under test to `ucrtbase.dll`, so the tests check our arithmetic, not
libm accuracy. "bit-exact (ucrt libm)" means: identical bits given the same
libm. With glibc, results differ from Sony's by libm's error, amplified by
the formula (worst seen: 12 288 ulp in `axis_angle_rotate` before
cancellation, i.e. still ~1e-7 absolute for unit inputs).

Pitfall seen in practice: Ghidra prints `if (x <= k)` for a branch the CPU
takes when `!(x > k)`. The two differ for NaN. Tests feed NaN inputs, and
the code is written to the observed NaN behaviour.

## INS math helpers (`0x180170120–0x1801711f0`) — `psvr2_ins_math.c`

| VA | Ours | Behaviour |
|----|------|-----------|
| `0x180170120` | `quat_conj_inplace` | Multiplies x, y, z of a w-first quaternion by `-1.0f` (a multiply, not a sign flip). Returns its argument. |
| `0x180170160` | `pose_negate` | Pose = position[3] then quaternion[4]. Multiplies position and the quaternion's x, y, z by `-1.0f`. |
| `0x1801701c0` | `vec3_lerp` | `out = (1−t)·a + t·b`, per component. |
| `0x180170640`, `0x180170670` | `vec2_len`, `vec3_len` | Euclidean length, squares summed in index order. |
| `0x180170700` | `quat_normalize` | Divides all four components by the norm; the norm sums x², y², z² and then w². No zero guard. |
| `0x1801707c0` | `vec2_normalize` | Divide by length; a length below `1e-4f` is replaced by 1. |
| `0x180170840` | `vec3_normalize` | Divide by length; a length below `1e-10` (compared as double) is replaced by 1. |
| `0x1801708f0` | — | Debug print of a 4x4 matrix (`%.2f`, row by row). Skipped. |
| `0x180170a10` | `axis_angle_rotate` | Object holds an axis at floats 1..3. Builds `q = (cos(a/2), sin(a/2)·axis)` (axis not normalised) and returns `q ⊗ (0,v) ⊗ q*`, vector part. The conjugate's components are formed as `(−axis_i)·sin`. Both products use one fixed Hamilton-product evaluation order (see `qmul` comment). The zero scalar of `(0,v)` is really multiplied (so inf/NaN propagate). |
| `0x180170cb0` | `mat4_identity` | Writes the 4x4 identity. |
| `0x180170dc0` | `quat_slerp(a, out, t, b)` | dot = a·b (x, y, z, then w). If dot < 0, **b is negated in place** and dot flipped. Weights start as (1−t, t). Unless dot > 0.9999 (NaN takes the sine path): s = √(1−dot²), θ = atan2f(s, dot), weights become sin((1−t)θ)/s… computed as `sinf(w·θ)·(1/s)`. Output = a·wa + b·wb. |
| `0x180171010` | `mat4_transpose` | In-place transpose. |
| `0x1801711f0` | `gauss_merge` | Merges two weighted isotropic Gaussians (mean₃, σ, weight). W = wa+wb; mean = (a·wa + b·wb)/W; second moment = (‖b‖²·wb + ‖a‖²·wa)/W + (σa²·wa + σb²·wb)/W, squared norms summed z, y, x; variance = that − (my² + mz² + mx²). Fails (returns false, writes nothing) unless variance > 0; else writes mean through the first pointer of its context and σ = √variance. |

## INS filters (`0x180171c10–0x1801726a0`) — `psvr2_ins_filter.c`

Sony keeps these objects as **structs of pointers** to their state words
(each field is reached through a pointer stored in the object). Our structs
use the same layout so both implementations can run on one object; the
field names are ours.

### Window filter `wf` (vec3 smoother with a settle lock)

State: estimate `est`, ring of the last ≤ 8 inputs, `fill`, `widx`,
`locked`, `settled` (byte), `status`, `miss`, `enabled` (byte), `cap`,
`init`, two per-axis gates (`gate_track`, `gate_lock`) and `alpha`.

- **init** (`0x1801726a0`, args: enabled, cap, alpha, gate_track*, half_gate_lock*, init*):
  `cap` is kept if 2..8, else 8. `gate_lock = 2·half` per axis (as `h+h`).
  Copies the rest, then does **reset**.
- **reset** (`0x180172040`): `est = init`; locked, fill, widx, miss, status,
  settled = 0.
- **mean** (`0x180172620`): if fill is 1..8, per-axis running sum of the
  ring in slot order, divided by `(float)fill`; otherwise zeros.
- **spread** (`0x180172490`): if fill is 2..8, per-axis max − min over the
  ring (max replaces when `max < x`, min when `x < min`, starting from slot
  0); otherwise 1000 on every axis.
- **update(out, in)** (`0x1801720a0`):
  1. Disabled: `out = in`, nothing else.
  2. Store `in` at `widx`; `widx` advances and wraps to 0 at `cap`; `fill`
     grows to at most `cap`.
  3. If `fill ≥ cap` and the spread passes `gate_lock`: `est = mean`,
     locked = 1, status = 3, miss = 0, settled = 1. Done.
  4. Else gate `in − est` with `gate_lock` if locked, else `gate_track`.
     Pass: `est = alpha·in + (1−alpha)·est`, status = 2, miss = 0.
  5. Fail: status = 1. If miss exceeds 32 (settled) or 8 (not settled):
     `est = mean`, locked = 0, miss = 0, settled = 0. Otherwise miss += 1.
  6. `out = est`.

  "Passes a gate" means no axis has `gate < |d|`; a NaN difference passes.

### Stillness averager `still`

Config: per-axis noise variance `var` and sample `rate` (float, truncated
to int `R`). State: `phase`, `count`, `ref`, `acc`, `result`, and a `wf`.

- **reset** (`0x180171d00`): result = NaN (`0xffc00000`) ×3, phase = 0,
  count = 0, ref = 0, acc = 0.
- **step(avg_out, in)** (`0x180171d40`), returns true when an average is
  ready:
  - phase 0: the first sample becomes `ref`; count += 1; `acc += in`.
  - phase ≠ 0: if any axis has `|in − ref|` not below `4·√var` (NaN counts
    as moved): phase = 0, count = 1, ref = acc = in. Otherwise count += 1,
    `acc += in`.
  - Then, in phase 0, once `count ≥ R/10` (integer division): ref =
    acc/count, phase = 1, count = 0, acc = 0. In phase 1, once
    `count ≥ (int)(R·1.5f)`: `avg_out = acc/count`, **reset**, return true.
- **feed(in)** (`0x180171c10`): run step; when it yields an average, pass
  it through the `wf` and store the filtered value in `result`.
- **get** (`0x180171c80`): vec = result; a 3x3 covariance with diagonal
  `var_k / rate` (float rate), zeros elsewhere; flag = wf's `settled`.

Use: accelerometer/gyro bias averaging while the headset is still [likely].

## INS parameters and covariance helpers (`0x1801729d0–0x180173370`) — `psvr2_ins_cfg.c`

`0x180172f90` returns a lazily filled global parameter block (0x80 bytes;
its float at +0x10 is negative until filled, then 0.1). Values by offset:
+0x00 0.05, 0.025, 0.005, 0.0025; +0x10 0.1; +0x14 0.3; +0x18 50; +0x1c
limits −10, 10, −5, 5, −10, 10; +0x34 `cov_small` 1e-7, 1e-4, 1e-5; +0x40
`cov_far` 1e-4, 5e-4, 1e-4; +0x4c 0.1, 1, 0.01, 0.05; +0x5c `near_dt` 1;
+0x60 `far_dt` 10; +0x64 `cov_floor` 1e-8, 1e-6, 1e-6; +0x70 gravity 9.8;
+0x74 three times 0.05235988 (3°).

- `0x1801729d0`: closure check, true when every `|A[r][c] − B[c][r]| < tol`
  (NaN fails). `0x180172b90`: a 3x3 covariance is valid when its diagonal
  is ≥ 0 and it is symmetric within 1e-6.
- `0x180172aa0` `cov3_floor(out, M, f)`: copy M if all nine elements are
  above f, else `f·I` (off-diagonals are `f·0.0`, so they carry f's sign
  and NaN).
- `0x180172da0` / `0x180172d10` / `0x180172e30` = `cov3_for_gap(…, which =
  0/1/2)`: gap `d = |a − b|`. `d < near_dt`: `cov3_floor(M, cov_floor[w])`.
  Otherwise an isotropic `cov_far[w]·I` when `d` is not below `far_dt` (NaN
  counts as far), else `cov_small[w]·I`.
- `0x1801732b0` / `0x180173270` / `0x180173370` = `cov3_small(w)`:
  `(M₈ + M₄ + M₀)/3 < cov_small[w]` and M valid.
- `0x1801732f0`: `|a|² < 3·p₀²` and `|b|² < 3·p₂²` (p = parameter block,
  squares summed z, y, x).

Pattern confirmed by tests here too: where Ghidra shows `if (k <= x)` the
NaN case behaves as `!(x < k)`.

## INS core kernels — `psvr2_ins_core.c`

Test: `t_irtracker_inscore.c`, 0 mismatches on every kernel (200 000 cases
each, 10 000 for the strapdown). **Quaternions here are x, y, z, w** (w
last), as the INS stores them; a pose is position[3] + quaternion[4].

- **`0x1801805a0` quat_mul** `r = a ⊗ b` (Hamilton). Order per lane:
  `r.x = (a.w·b.x + a.y·b.z) + (b.w·a.x − a.z·b.y)`, same pattern for y, z;
  `r.w = −(a.y·b.y + a.z·b.z) + (b.w·a.w − a.x·b.x)`. Every quaternion
  product in the INS uses this one form.
- **`0x18017f2c0` rotinc(dt, w0, w1)**: `θ = (w0 + w1)·dt/2`. The
  small-angle branch is taken when `|θ|² < sqrt(2.2888184e-5)` (≈ 0.00478:
  Sony compares the *squared* angle with the square root of the
  constant): `s = 0.5 − |θ|²/48`, `c = 1 − |θ|²/8`; else `s = sin(|θ|/2)/|θ|`,
  `c = cos(|θ|/2)`. Result `(θ·s + (w0 × w1)·dt²/24, c)`, coning term first in
  each sum. `|θ|²` is summed `(y² + x²) + z²`.
- **`0x18017f8e0` strapdown(filter, out, dt, n, samples)**: the filter is
  a struct of pointers (`+0x00` pose, `+0x08` velocity, `+0x10` gyro bias,
  `+0x18` accel bias, `+0x28` gravity, `+0x30` previous accel, `+0x38`
  previous gyro, `+0x40` delta pose; all but `+0x00` and `+0x40` through
  two levels). Samples are 28 bytes: accel[3], gyro[3], time. With
  `f_prev` the stored previous accel:
  `Sa = f_prev/2 + Σ R(dq_k)·f_k` (last term halved),
  `SSa = f_prev·n/2 + Σ_{k<n−1} R(dq_k)·f_k·(n − k − 1)`,
  `dq_k = dq_{k−1} ⊗ rotinc(dt, w_{k−1}, w_k)` with `w_{−1}` the stored
  previous gyro. `dq` is normalised (`(x²+z²) + (y²+w²)`, skipped when 0);
  the new quaternion `q ⊗ dq` is **not**. `v' = ((rot + Sa·dt) + v) + T·g`,
  `p' = ((T·v + p) + R(q)·SSa·dt²) + (g/2)·T²`, `T = n·dt`, both rotations
  with the old `q`, `R(q)x` as `x + w·t + q×t`, `t = 2 q×x`. Output (92
  bytes): delta pose copied, p', q', v', last `f − b_a`, last `w − b_g`.
  `n = 0`: `dq = (0,0,0,1)`, increments 0, previous samples kept.
- **`0x18017f500` predict(filter, dt, out)**: `dt = min(dt, 1)`; if
  `dt > 0`: `p += v·dt`, `q = q ⊗ rotinc(dt, w, w)` with the stored previous
  gyro. Then `out = delta ∘ (p, q)` (`delta.q ⊗ q`, `R(delta.q)·p + delta.p`).
  Failing `pose_ok` → identity, returns 0. Callers cap dt at INS `+0x53c`
  (0.05 s, `FUN_18017c370`).
- **`0x18017bac0` pose_ok**: no NaN in the 7 floats and
  `(x²+z²) + (y²+w²)` in `[0.9, 1.1]`.
- **`0x180181a50` blend1(blender, out, out_vel, target, target_vel, dt)**:
  target with a NaN (`FUN_180181690`) → nothing. Snap (copy target) if any
  `|Δp_i| > gate_p` (blender `+0xa0`) or `cos(gate_r/2) > |(target.q ⊗
  out.q⁻¹).w|` (`+0xac`; `q⁻¹ = (−x,−y,−z,w)/|q|²`). Else `α = min(4·dt, 1)`:
  `p += Δp·α`; slerp with `d = (t.x·o.x + t.z·o.z) + (t.y·o.y + t.w·o.w)`,
  weights `sin((1−α)θ)/sinθ`, `sin(αθ)/sinθ` for `|d| < 0.9999999` (else
  `1−α`, `α`), `α` negated when `d < 0`; no renormalisation. Velocity is
  copied from the target in both branches.
- **`0x1801733f0` imucal_mat(out, {R, _, s}, off)**: row i = `R[3i..3i+2]·s_i`,
  bias row `j = ((s1·R[3+j])·o1 + (s2·R[6+j])·o2) + (R[j]·s0)·o0`, last
  column (0,0,0,1). Applied per sample (`FUN_180174410`) as
  `a_j = 1·bias_j + ((x·m[j] + y·m[4+j]) + z·m[8+j])` on `(float)s16`.

IMU calibration records (`FUN_180173fe0`, built when both are present and
the type 13 generation changed):

| Record | Offset | Use |
|--------|--------|-----|
| type 12 | `+0x00..+0x08` | accel misalignment c0, c1, c2 → `R = k·[1,0,0; c0,1,0; c1,c2,1]` |
| type 12 | `+0x0c..+0x14` | accel scale per axis |
| type 12 | `+0x18..+0x20` | accel offset (used negated) |
| type 12 | `+0x24..+0x38` | gyro misalignment → `R = [1,m30,m34; m24,1,m38; m28,m2c,1]` (by offset) |
| type 12 | `+0x3c..+0x44` | gyro scale per axis |
| type 12 | `+0x74` | accel overall scale k |
| type 13 | `+0x28..+0x30` | gyro offset (used negated) |
