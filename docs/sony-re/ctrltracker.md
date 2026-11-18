# Controller tracker core (ctrltracker): behavioural spec

Sony VA ranges `0x180049240–0x18004cfff` and `0x180050000–0x18006ffff` of
`driver_playstation_vr2_orig.dll` (PE timestamp 1741940190). This file
describes behaviour in our own words; the implementation in
`userspace/lib/sie/ctrltracker/` is written from it, and
`scripts/sony-re/difftest/t_ctrltracker_*.c` compares it with Sony's code
under Wine. Per-function status: `scripts/sony-re/status/ctrltracker.tsv`.

Conventions:

- Floats are IEEE single. Sony's build uses AVX but never fuses
  multiply-add, so ours must be built with `-ffp-contract=off`.
- Sums are evaluated left to right as written, `a + b + c = (a + b) + c`.
  The order of the two factors of a product does not matter (exact).
  Doubling (`2a`, written as `a + a`) is exact, so `(2a)·b = 2(a·b)`.
- Quaternions are stored `x, y, z, w` (w last).
- Matrices are column-major, each column 16 bytes (4 floats), like glm.
  `a[r][c]` is the element at row r, column c = `m[4c + r]`.

## Small vector / quaternion kernels

### Rotate a vector by a quaternion (`0x180058720`)

`out = rot(q, v)`, out/q/v pointers, returns out. Computed as two Hamilton
products, `p = q ⊗ (v, 0)` then `out = (p ⊗ q̄).xyz`, but with p's scalar
part kept with the opposite sign (`d` below), so no negation is needed:

```
px = (vx·w + vz·y) − vy·z
py = (vx·z + vy·w) − vz·x
pz = (vy·x + vz·w) − vx·y
d  = (vx·x + vy·y) + vz·z
out.x = ((d·x + px·w) − py·z) + pz·y
out.y = ((d·y + py·w) − pz·x) + px·z
out.z = ((d·z + pz·w) − px·y) + py·x
```

The same sequence is inlined in several bigger functions (e.g. the
measurement model at `0x18005fc20`).

### Quaternion length / normalise (`0x180058900`, `0x180058970`)

Length: `sqrt(((x² + y²) + z²) + w²)`. Normalise: `s = 1 / length`, then
each component times `s` (no zero check: a zero quaternion gives inf/NaN).

### Quaternion to rotation matrix (`0x180058a10`)

Writes three 16-byte columns; only lanes 0..2 are defined (lane 3 is left
as whatever was in a register; callers ignore it). With `X = x+x`,
`Y = y+y`, `Z = z+z`:

```
col0 = ((1 − Y·y) − Z·z,  Z·w + Y·x,        Z·x − Y·w)
col1 = (Y·x − Z·w,        (1 − X·x) − Z·z,  Z·y + X·w)
col2 = (Y·w + Z·x,        Z·y − X·w,        (1 − X·x) − Y·y)
```

### Matrix times vector (`0x180058e40`, `0x180058f30`)

4×4 times vec4: `out[r] = ((v1·a[r][1] + v0·a[r][0]) + v2·a[r][2]) + v3·a[r][3]`
for r = 0..3. 4×3 (three 16-byte columns) times vec3: the same without the
last term. Arguments: (matrix, out, vector); returns out.

### 4×4 inverse (`0x180059000`)

Cofactor expansion using 2×2 minors; arguments (out, in). With `a = in`:

```
m17 = a22·a30 − a32·a20     m18 = a23·a31 − a33·a21
m19 = a22·a10 − a12·a20     m20 = a23·a11 − a13·a21
m21 = a13·a31 − a33·a11     m22 = a12·a30 − a32·a10
c0 = (a12·m18 − a32·m20) − a22·m21
c1 = (a13·m17 − a33·m19) − a23·m22
c2 = (a20·m21 + a30·m20) − a10·m18
c3 = (a21·m22 + a31·m19) − a11·m17
s  = 1 / (((c1·a01 + c0·a00) + c2·a02) + c3·a03)
n27 = a02·a10 − a00·a12     n28 = a03·a11 − a01·a13
n29 = a02·a30 − a00·a32     n30 = a03·a31 − a01·a33
n31 = a02·a20 − a00·a22     n32 = a03·a21 − a01·a23
out col0 = (c0, c1, c2, c3)·s
out col1 = ((a22·n30 − a32·n32) − a02·m18, (a23·n29 − a33·n31) − a03·m17,
            (a30·n32 − a20·n30) + a00·m18, (a31·n31 − a21·n29) + a01·m17)·s
out col2 = ((a32·n28 − a12·n30) + a02·m21, (a33·n27 − a13·n29) + a03·m22,
            (a10·n30 − a30·n28) − a00·m21, (a11·n29 − a31·n27) − a01·m22)·s
out col3 = ((a12·n32 − a22·n28) + a02·m20, (a13·n31 − a23·n27) + a03·m19,
            (a20·n28 − a10·n32) − a00·m20, (a21·n27 − a11·n31) − a01·m19)·s
```

No singularity check (a singular matrix gives inf/NaN).

### libm

Sony imports `sinf`, `cosf`, `atan2f`, `acosf`, `asinf`, `atanf`, `tanf`,
`expf`, `logf`, `sin`, `cos`, `acos`, `log`, `pow`, `sqrt(f)` from the UCRT.
The difftests route our calls to the same UCRT entry points
(`ct_difftest.h`), so "bit-exact" means bit-exact given the same libm. With
mingw's libm, `sinf`/`cosf` differ by at most 1 ulp (1 % of random angles);
glibc may differ similarly.

### More kernels

- **vec3 add in place** (`0x180061be0`, (a, b)): `a[i] = b[i] + a[i]`.
- **vec3 length** (`0x180061c60`): `sqrt((x² + y²) + z²)`.
- **cross product** (`0x180061cc0`, (out, a, b)): `a × b`, each component
  `a_j·b_k − a_k·b_j`.
- **quaternion product** (`0x180061fa0`, (p, out, q)): Hamilton `p ⊗ q`:
  `x = ((px·qw + pw·qx) + py·qz) − pz·qy`, `y = ((py·qw + pw·qy) + pz·qx) −
  px·qz`, `z = ((pz·qw + pw·qz) + px·qy) − py·qx`,
  `w = ((pw·qw − px·qx) − py·qy) − pz·qz`.
- **yaw quaternion** (`0x1800620b0`, (out, angle)): `(0, sin(a·0.5), 0,
  cos(a·0.5))`.
- **3×3 product** (`0x1800621a0`, (A, out, B)): column c of the result is
  `A·B[c]`. Column 0 sums as `(B0.y·A.col1 + B0.x·A.col0) + B0.z·A.col2`;
  columns 1 and 2 as `(Bc.x·A.col0 + Bc.y·A.col1) + Bc.z·A.col2`.
- **rotation about Y** (`0x1800623d0`, (out, angle)): columns `(c, 0, −s)`,
  `(0, 1, 0)`, `(s, 0, c)`, `s = sin a`, `c = cos a`.
- **azimuth difference** (`0x1800627f0`, (out_a, out_b, a, b) → float):
  `ta = atan2(a.z, a.x)`, `tb = atan2(b.z, b.x)`, `d = ta − tb`; if
  `d > π` add `−2π`, else if `d < −π` add `2π` (π and 2π as floats).
  Stores ta, tb, returns d. Used for yaw alignment between two frames.
- **uniform random** (`0x180062bc0`, (state[2], lo, hi) → float):
  Marsaglia multiply-with-carry: `s0 = 36969·(s0 & 0xffff) + (s0 >> 16)`,
  `s1 = 18000·(s1 & 0xffff) + (s1 >> 16)`, `u = s1 + (s0 << 16)` (u32),
  `t = float((double(u) + 1) · 2.328306435454494e-10)` (that is
  `1/(2³² + 2)`, so t is strictly inside (0, 1)), result `t·(hi − lo) + lo`.

## Process noise (`0x180062880`, `0x180062950`)

Per-axis variance of a 3-vector state block, driven by the current rate
vector v (angular velocity for the first, the vector at +0x10 of the state
for the second). For axis i with the other two axes j < k:

```
var_i = ((v_i²·own + v_j²·cross) + v_k²·cross) [+ extra] + base_i
```

or `fixed_value` if the axis's "fixed" flag is set. Config offsets
(third argument): first function own +0x30, cross +0x34, fixed value +0x38,
base +0x10..0x18, no extra; flags at state +0x30..0x32. Second function own
+0x40, cross +0x44, extra +0x3c, fixed value +0x48, base +0x20..0x28; v at
state +0x10, flags at state +0x33..0x35. Arguments (state, out, config).

### Batch 3 kernels

- **vec3 subtract / scale in place** (`0x18006af00`, `0x18006af50`):
  `a[i] = a[i] − b[i]`; `a[i] = s·a[i]`.
- **shortest-arc quaternion** (`0x18006b030`, (out, a, b), a and b unit):
  `d = ((a·b summed x, y, z) + 1)`, `s = sqrt(d + d)`, `xyz = (a × b)·(1/s)`,
  `w = s·0.5`. No guard for opposite vectors (d = 0 gives inf/NaN).
- **double quaternion normalise / product** (`0x18006b250`, `0x18006b310`):
  as the float versions, in double.
- **sphere-cap quaternion** (`0x18006b4a0`, (out, v, c, r) floats;
  `0x18006b5c0` the same with double c, r and double output):
  `n = |v|²` (float), `t = sqrt((1 − c²)·n + r²)`,
  `w = (t·r − n·c) / (r² + n)`, `k = w + c`, `xyz = (v_i·(1/r))·k`,
  output w = `w`. The double version computes n in float, then t and w in
  double, rounds `k` and `1/r` to float and evaluates `(v_i·(1/r))·k` in
  float before widening.
- **duplicate**: `0x18006b700` is byte-for-byte the same routine as
  `0x1800627f0` (azimuth difference).

### Rotation matrix to quaternion (`0x180061d40`, (out, m))

m is three 4-float columns. Ghidra's decompile of the flag logic is wrong
here; this is from the disassembly.

1. `neg = 0 > (m11 + m00) + m22` (NaN: false).
2. Three flags, each "negate one diagonal element" (all false unless neg):
   - `fz = neg ∧ ¬(m22 > m00 ∧ m22 > m11)` negates m22 and m01;
   - `fx = neg ∧ (m11 > m00 ∨ m22 > m00)` negates m00 and m12;
   - `fy = neg ∧ ¬(m11 > m00 ∧ ¬(m22 > m11))` negates m11 and m20.
   So with a negative trace every diagonal element but the largest is
   negated.
3. `t = ((m11 + m00) + m22) + 1` and `h = 0.5 / sqrt(t)` with the negated
   values; `d_yz = m21 − m12`, `d_zx = m02 − m20`, `d_xy = m10 − m01` (the
   subtrahends possibly negated).
4. Lanes `p = fz ? (t, d_zx, d_xy, d_yz) : (d_yz, d_xy, d_zx, t)`; output
   `fx ? (p2, p0, p3, p1) : (p0, p2, p1, p3)`, each times h. This yields
   the usual "largest component first" formulas: w-, x-, y- or z-largest.

### Batch 4 kernels

- **relative yaw** (`0x18006b790`, (p, q) → float): `r = p ⊗ q̄` (q̄ =
  conjugate, xyz negated); `u = rot(r, (0,1,0))`; `c = shortest-arc(u →
  (0,1,0))` removes the tilt; `g = c ⊗ r`; `e = rot(g, (1,0,0))`; return the
  azimuth difference of `(1,0,0)` and `e` (that is `atan2(0,1) −
  atan2(e.z, e.x)`, wrapped). All steps are the kernels above evaluated with
  literal 0/1 components (products by 0 are kept: they matter for inf/NaN
  and signed zeros).
- **exponential map** (`0x18006bb50`, (out, v)): `len = |v|`, `half =
  len·0.5`. If `len < sqrt(FLT_EPSILON)` (`sqrt(1.1920929e-7)`, computed
  once at first use) then `k = (len·len)/48 + 0.5`, else `k = sin(half) /
  len`. `xyz = v·k`, `w = cos(half)`. (The series has the wrong sign — the
  true one is `0.5 − len²/48` — but below the threshold the term is under
  half an ulp of 0.5, so it never changes the result.)
- **attitude integration** (`0x18006bcd0`, (out, q, w, dt); q/out double,
  w/dt float): `v = dt·w` (float), `e = exp(v)` as above (float), widen e
  to double, `out = normalize(q ⊗ e)` in double.
- **reference setters** (`0x18006bf80`, `0x18006bfd0`, (obj, ref)): copy the
  4-float ref to obj+0xc0 (resp. +0xd0) and store `obj[0..2] − ref` (resp.
  `obj+0x10` vector minus ref) at obj+0xe0 (resp. +0xf0); lane 3 undefined.

## Packed covariance (15 × 15)

The filter state has 15 error components in five 3-blocks (block base
b = 0, 3, 6, 9, 12). Its covariance is stored as the packed upper triangle,
row-major, 120 floats: element (i, j), i ≤ j, lives at
`((29 − i)·i)/2 + j`.

- **reset block** (`0x18006c020`, (P, b)): for rows i = b..b+2 (only
  i < 15) clear the row from the diagonal to the end; for every earlier row
  r < b clear the three entries in columns b..b+2; then put 1.0 on the three
  diagonal entries.
- **set block** (`0x18006c1b0`, (P, b, M)): reset as above, then store the
  upper triangle of the 3×3 matrix M (three 4-float columns): (b+r, b+c) =
  `M[r][c]` for r ≤ c, where `M[r][c]` is lane c of column r (M is used as
  symmetric).
- **get block** (`0x18006c100`, (out, P, b)): read the six upper entries of
  the diagonal block and mirror them; lane 3 of each output column is not
  written.

## Lever-arm transfer (`0x18006c340`)

Arguments (out_pos, out_vel, out_rot, pos, vel, R, ω, B, lever). With
`l = R·lever` (4×3 product order as `0x180058f30`):
`out_pos = l + pos`,
`out_vel = (l.z·ω.y − l.y·ω.z, l.x·ω.z − l.z·ω.x, l.y·ω.x − l.x·ω.y) + vel`
(that is `ω × l + vel`, each sum done as `(cross term) + vel_i`),
`out_rot = R·B` via the 3×3 product `0x1800621a0` (48 bytes copied, lane 3
undefined). Moves a pose and its velocity from the body origin to a point
fixed on the body (e.g. IMU to controller origin).
