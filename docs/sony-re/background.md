# Background model: static IR lights (`RnD::Background`)

Sony's controller tracker keeps a 3D map of IR light sources that do not
move (lamps, reflections, other headsets' LEDs at rest) and gives every
blob of every camera a "background" score, so the controller solver can
ignore blobs that are part of the room. The Linux driver has nothing like
it. No other doc covered it; `triage.md` called its stages "boot search
stages".

Tags as elsewhere: `[sure]` read from the code, `[likely]`, `[guess]`.

## 1. Summary for the driver

- Three stereo **views**, each a pair of cameras: (0, 1), (0, 2), (1, 3)
  [sure].
- Each view keeps a list of 3D **landmarks** in world coordinates
  (320-byte records), built by triangulating blobs that both cameras of
  the view see and that are not near the headset [sure for the flow,
  likely for the details].
- Every frame each landmark is projected into both cameras with the
  current head pose, matched to the nearest blob, and its per-camera hit
  and miss counters updated [likely].
- A landmark is dropped after **40** missed frames in a camera, or **300**
  once that camera has seen it at least **10** times [sure].
- Output: per camera, a float per blob (0 = not background; otherwise the
  matched landmark's score) [sure]. Where the controller solver reads it
  is still open (§5).

## 2. Objects and calls [sure]

| Item | Value |
|------|-------|
| Interface | `RnD::IBackground` (vtable `0x180319060`; slots 1–4 pure) |
| Used on PC | `RnD::Background` (vtable `0x1803190e0`), 0x490 bytes, 3 views × 0x180 (ctor in `0x180049240`, view ctor `0x1800b6d40`, view dtor `0x1800b3f50`) |
| Alternative | `RnD::Background2` (vtable `0x180319090`), 0x2a90 bytes, 4 per-camera models × 0xaa0. Chosen only when BSS byte `0x1805f05d8` is non-zero; nothing writes that byte, so it is dead on PC [likely] |
| Owner | global `Block` (Background), `DAT_180616700` (Background2); freed in `0x18004ab50` |

| Slot | Background | Background2 | Role |
|------|------------|-------------|------|
| 0 | `0x1800b3ea0` | `0x1800b6ed0` | destructor |
| 1 | `0x1800b2190` (returns 3) | `0x1800b2100` (returns 4) | number of views / cameras |
| 2 | `0x1800b3fb0` | `0x1800b7000` | update one view (frame id, view index, frame data, blobs); Background runs only when `frame +0x10334` ≥ 2 [likely: the number of cameras in the frame] |
| 3 | `0x1800b3fe0` | `0x1800b82f0` | write the per-blob scores of one camera |
| 4 | `0x1800b6450` | `0x1800b9f00` | debug overlay (text `lmcount: %d`, drawing helpers `0x1800bdd00`, `0x1800be0c0`, `0x1800bedc0`, `0x1800bf660`) |

Per frame, in the controller per-frame core `0x18004b0a0`: slot 2 for
views 0..2, then slot 3 for every camera, then the blob lists go to the
camera managers (`0x1800a0480`) and the controller boot/track runs
(`0x1800a56b0`). The multi-camera path (`0x180048690`) does the same on
worker threads (`BackgroundThreadParams`).

View records (0x180 bytes): `+0` view id, `+4` camera A, `+8` camera B,
`+0x10` / `+0x50` world-from-camera poses of A and B (head pose ·
extrinsic, frame `+0x10a40 + 0x40·cam`), `+0x90..+0x128` per-camera blob
counts and scratch, `+0x130` landmark list (RnD::Vector of 320-byte
records).

## 3. One view update (`0x1800b40e0`) [sure unless tagged]

1. World-from-camera poses for cameras A and B; blob counts.
2. `0x1800b53c0` for A, then for B (§3.1).
3. `0x1800b3090` for A, then for B, with factor 0.99 (§3.2).
4. `0x1800b43d0`: new landmarks from the pair (§3.3).
5. Prune: for each landmark and each of its two cameras, misses
   (`+0xa0` for A, `+0x130` for B) over the limit drop it. The limit is
   300 when that camera's hit count (`+0x24` / `+0xb4`) is ≥ 10, else 40.

### 3.1 Projection and ray test (`0x1800b53c0`) [likely]

- Each landmark is projected into the camera (head pose · extrinsic,
  distortion through `0x180045650`). Landmarks behind the camera, at zero
  depth or far outside the view (a 2D magnitude test against 200,
  units not checked) are skipped. The previous predicted pixel moves to `+0x30`, the new one
  goes to `+0x28`; the per-frame flags `+0x20..+0x23` and the blob link
  `+0x84` (−1) are reset.
- Each blob's viewing ray is tested against the landmarks: a landmark
  closer than 100 mm to the ray (squared distance < 10 000 mm²) and in
  front of the camera sets that blob's bit in the view's near-landmark
  bitsets (`+0x90`, `+0xe0`).

### 3.2 Match and score (`0x1800b3090`) [likely]

Matches each projected landmark to a blob within a pixel radius
(2500 px², i.e. 50 px) using the camera's focal length (constants
185.18, 1107.07, 1110.07, 0.0027 in this function) and updates its hit
and miss counters and its score (`+0x88` for A, `+0x118` for B), with the
0.99 factor as decay [guess for the exact rule].

### 3.3 New landmarks (`0x1800b43d0`) [likely]

For blobs of camera A and B that matched no landmark: pair them when the
epipolar error is small (|y| ≤ 0.02 in normalised coordinates),
triangulate (`0x1800b24b0`), and reject points within 600 mm of the
reference (squared distance < 360 000), or within 2 m when a flag is
clear (< 4 000 000) [guess: keep controllers in hand out of the map].
Accepted points become landmarks with an initial value 20.0 per camera
(`0x1800bb900` appends).

## 4. Function map

| Function | Lines | Role |
|----------|-------|------|
| `0x1800b40e0` | 135 | one view update (§3) [sure] |
| `0x1800b53c0` | 816 | projection and ray test (§3.1) [likely] |
| `0x1800b3090` | 816 | match and score (§3.2) [likely] |
| `0x1800b43d0` | 818 | new landmarks (§3.3) [likely] |
| `0x1800b24b0` | 477 | two-view triangulation used by §3.3 [likely] |
| `0x1800b2e20` | – | helper of `0x1800b3090` |
| `0x1800bb900`, `0x1800bbbe0` | – | landmark list append / bitset grow |
| `0x1800b3fe0` | 64 | slot 3: zero the camera's score array, then write each landmark's score at its blob index (camera 0 → view 0 side A, 1 → view 0 side B, 2 → view 1 side B, 3 → view 2 side B) [sure] |
| `0x1800b6450`, `0x1800b9f00` | 470, 451 | debug overlays (slot 4) [sure] |
| `0x1800b7040` | 145 | Background2 per-camera update → `0x1800b7350`, `0x1800b85b0`, `0x1800b9740` (dead on PC) [likely] |
| `0x1800b82f0` | 39 | Background2 slot 3: scores from 288-byte spot records [sure] |
| `0x18004c9c0` | 22 | debug: draws the active model through slot 4 [sure] |

## 5. Open

- Which solver step reads the per-blob score (blob store array at
  `+0xc8`, copied to the camera managers by `0x1800a0480`), and with what
  threshold.
- Exact matching and scoring rule in `0x1800b3090`.
