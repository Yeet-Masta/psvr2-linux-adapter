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
| `0x1800a1bf0` | 21 | reset controller `a2`: clear flag `+0x601`, reload its config copy (`+0x1af0` → `+0x20e0` via `0x1800a0bf0`), clear `+0x1f94`, reset IMU prior (`0x1800a0ab0`), zero its 0x1d00 blob store [sure] |
| `0x1800a1ce0`, `0x1800a1d60` | 19, 33 | debug overlay draw for one controller (`0x1800d6e50`, `0x1800d7410`) [sure] |
| `0x1800a0ab0` | 77 | reset the IMU-orientation prior of a controller: `+0x3358` = 0, `+0x3360..+0x351f` = 0, `+0x35f0` = 1e5 (≈ "never") [sure] |
| `0x1800a0bf0` | 126 | copy-assign of the controller config block (nested copy `0x18009fa80`, then fields up to +0x570) [sure] |
| `0x1800a1000` | 40 | ctor of a 6-slot struct (6 × 0x40 items) with defaults 10.0, 50.0, 5, 3 [sure; meaning unknown] |
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

(Work in progress: next are `0x1800a45f0`, `0x1800a6190`, `0x1800a7c50`
and the rest of the 58 large functions; then medium and small ones.)
