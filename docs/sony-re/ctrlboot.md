# ctrlboot: Sense controller boot / multi-camera pose search

Behavioural spec for the controller-tracker boot stage of Sony's PSVR2 SteamVR
driver (build PE timestamp 1741940190, VA range `0x1800a0000–0x1800fffff`,
plus the `RnD::` helper classes it uses). Written in our own words from
analysis; our implementation is `userspace/lib/sie/ctrlboot/`, checked by
`scripts/sony-re/difftest/t_ctrlboot_*.c`. Per-function status:
`scripts/sony-re/status/ctrlboot.tsv`.

Conventions: `|x|` is the IEEE absolute value (sign bit cleared, NaN stays
NaN). All comparisons are ordered (false when either side is NaN). Float is
binary32, no fused multiply-add.

## Common vector type

Many `RnD::` methods take a dense vector object. Only two fields matter: a
pointer to the float elements at +0x30 and an element count (64-bit) at
+0x38. Below, "vector" means that (data, n) pair.

## Robust loss functions (`RnD::Robustifier*`)

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

## Random generator (`RnD::MersenneTwister`, `RnD::Random<MT, uint>`)

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

## Fused multiply-add

Unlike the libpad/irtracker code, this module **does use FMA**: matrix ×
vector is `mul` for the first column then `vfmadd231ps` for the others
(976 FMA instructions in `0x1800a0000–0x1800fffff`). Ghidra prints a fused
`a*b + c` the same as an unfused one, so every function's fusion pattern is
taken from the disassembly. Our code is built with `-ffp-contract=off` and
calls `fmaf` exactly where the reference fuses.

## Math helpers (`psvr2_cb_math.c`)

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

## Complex and 2×2 helpers (`psvr2_cb_cplx.c`)

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
