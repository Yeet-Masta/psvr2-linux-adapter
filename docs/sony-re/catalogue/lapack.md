# Function catalogue: CLAPACK block (`0x180083700`–`0x18008d3ff`)

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

## Who calls it [sure]

Only two Sony functions call into the block, both in the ctrl-boot LM
solver (`catalogue/ctrlboot.md`, "Solver classes"):

- `0x1800fce20` (45 lines), called from `0x1800f8f80` / `0x1800fa7d0`
  (slot 0 of the two `WeightedLMSolver` vtables): factorises the packed symmetric
  normal-equation matrix with `SSPTRF` and solves with `SSPTRS`. So the
  6 × 6 LM step is a Bunch–Kaufman solve on packed storage, not a
  Cholesky.
- `0x1800fd810` (68 lines), from `0x1800fa430` / `0x1800fbc60`: inverts a
  matrix with `SGETRF` + `SGETRI` (LU, then inverse from the factors)
  [likely: the covariance of the final pose].

## Routines

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

## Double-precision subset (`0x18008d910`–`0x18008ee2f`)

A second, smaller f2c block in double precision serves the controller
UKF (`catalogue/ctrltracker.md`): packed Cholesky, inverse from the
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

## The rest of the old `lib:linalg` range

The triage first labelled all of `0x180070950`–`0x18009ffe0` as a
library. Only the block above is one. The rest is Sony code and is now
labelled core (`triage.md` §5):

- `0x180070950`–`0x180082e90` and `0x18008ee30`–`0x18009cff0`: the
  controller UKF (`ukf.md`, `catalogue/ctrltracker.md`) with its
  double-precision matrix kernels, its replay path (`0x18007add0`,
  log `WARNING--->backwards!`), the tilt correction (`0x18007dc60`), the
  IMU interval reduction (`0x180082e90`) and the solver `0x1800942f0`.
  → ctrl-tracker.
- `0x18009d100`–`0x18009ffe0`: RnD math used by ctrl-boot (Mersenne
  Twister `0x18009f3e0`, SIMD `acos`/`sincos`, camera model
  `0x18009eb50`). → ctrl-boot.
