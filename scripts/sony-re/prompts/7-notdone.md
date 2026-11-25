# Topic: open gaps in the dev driver (`controller-tracking.md` part B) — priority 7

Read `common.md` first, with one change: the repo is
`/home/yeetmasta/Claude/psvr2/src-dev`, so run `scripts/sony-re/trace.py`
from there. Also read `development.md` part A ("Not done, and why"),
`controllers.md` part C, `controller-tracking.md` part A and the
camera-calibration section of `overview.md` part B.

The dev driver implements everything except the items below. Each item is
blocked by a specific unknown. Find exactly that unknown: offsets, formulas,
constants and units. Do not re-document what the existing docs already
cover. Tag every claim [sure], [likely] or [guess], and give the function
address it comes from. Raw values (tables, constants) may be read from the
DLL and written down. Do not paste decompiled code.

Work through the items in order. If one stalls for long, write down what you
found and the next lead, then move on.

## 1. Raw camera calibration → camera model (blocks the pose solve)

Known:
- `FUN_180161b70` parses VD type 10/11 ("SIE" blob). Each camera record
  holds id, width, height, 9 floats K (record +0x08), then **20 doubles**
  (record +0x2c..+0xcb). The parser stores them at output +0x70.., with a
  0x110-byte stride per camera.
- The projection Sony evaluates is `FUN_180045650`, on a struct at camera
  object +0x6b0 (reached through `FUN_180046240` / `FUN_180046330`). The
  stages, in struct byte offsets:
  - Stage 0, distortion:
    - normalised x, y; r2 = x² + y²;
    - four float lanes of degree-5 polynomials in r², coefficients at
      +0xf0 + 4·lane + 0x10·power;
    - radial = lane3 / lane0;
    - tangential p1 at +0x48, p2 at +0x4c;
    - the constant at `DAT_180319dc0` (probably 2.0).
  - Stage 1: a 3×3 matrix at +0x90.., with a homogeneous divide.
  - Stage 2: `out = param[1] − param[0]·p`, with param[0] at +0x00 and
    param[1] at +0x08.
- `FUN_1800d3360` copies this 0x180-byte struct from its `param_5`. On the
  config-file path, `FUN_18004a340` passes `&DAT_1805f33e0 + cam·800`.
  `FUN_1800c8950` / `FUN_1800c8a40` initialise or fill a similar struct
  (defaults from `DAT_180319a80..DAT_180319af8`).

Find:
- The function that builds the +0x6b0 struct from the parsed 20 doubles and
  K, on the live path (VD stream → `FUN_180156500` → config manager), not
  the debug config-file path.
- The writers of `DAT_1805f32e0` / `DAT_1805f33e0`: decomp around lines
  9110 and 9405.
- Give the mapping as a table: each raw double index → struct field →
  role (P coefficient i, Q coefficient i, p1, p2, K entry, flip scale or
  offset).
- Are lanes 1 and 2 used anywhere (another projection, the Jacobian,
  undistort)?
- What are the flip-stage values for each camera? Which camera index means
  which physical camera?
- The inverse (undistort) Sony uses for blobs: `kP1PFromFailedUndistort`
  in `FUN_1800d5fc0`. Iterative or closed form? Iteration count and
  tolerance?
- The camera ↔ camera / IMU extrinsics (0x34-byte records): direction of
  the transform, units (mm or m), and which frame the LED models and the
  SLAM pose use.

## 2. Controller pose solve, the minimum needed to build one

From controller-tracking.md §A.5.3–5.6, controller-tracking.md part D, controller-tracking.md part H, headset.md part B, fill
only these gaps:
- **Blob ↔ LED association.** Gating radius and metric, use of the ellipse
  axes and angle, uniqueness rules, how the blob `id` and `link` fields are
  used, and how the per-frame core (`FUN_18004b0a0`) splits blobs between
  two controllers.
- **Boot.** The candidate generation (P3P or other), how candidates are
  scored, the minimum inlier count, the thresholds, and when boot is
  retried. The 0x60-byte tuning block before each LED model
  (`0x180257220 + m·0x1400`): name every field and value.
- **Refine.** The residual definition, the robust kernel and its constants,
  the iteration count, and how the cameras are combined.
- **UKF.** The state vector, process noise, and measurement noise values.
  How the controller IMU is fed in (rate, time alignment through the libpad
  clock fit) and what the filter outputs to SteamVR.
- **LED visibility test.** How an LED's normal and the camera direction
  decide "visible" (the angle metric at +0x15bc in `FUN_1801999f0` and its
  ≈45 threshold: degrees or cosine?).
- **Hardware type.** Which HID report or feature carries controller info
  +0x3c (hardware type → model 0xb..0xf) and +0x10 (preset)? Trace
  `FUN_180118c70` back to the bytes read from the controller.

## 3. LED-sync details still tagged [likely] / [guess]

- BG check pattern `0x180260430` (32 bytes: 2 at indices 0, 12, 16 and 24).
  Which frame of the BG window each byte refers to, and what 2 means.
  Source: `FUN_1801954f0`, `FUN_1801975e0`.
- The BG estimator exactly:
  - the window length;
  - how the samples are indexed against the controller's sweep;
  - the floor `(a+b)·0.5` (a and b are which samples?);
  - the "NO MAX MIN" and "NO DIF MAX MIN" conditions with their thresholds.
- The drift-fit value and units: what number is pushed in BROAD, and what
  in BG/STABLE (slot units, steps or µs)? Is the slope scaled before
  cmd 3/5? Source: `FUN_180194180`, `FUN_180193f70`, `FUN_18019caf0`,
  `FUN_18019bc00`.
- The sign convention: does a positive slope lengthen the controller's
  frame cycle?
- Check the BROAD accept rule: are the "3 consecutive" decodes counted per
  frame or per successful decode? The 11-frame window lines up with the
  code only once per cycle.
- Correct `controllers.md` §C.2.2: `FUN_1801cbeb0` calls the mode setter with
  **2** when firmware < 0x1006d or the probe has failed 4 times. Confirm
  this, and say which firmware field `local_b4` is (the controller's
  feature 0x20 version or something else).

## 4. Sony's driver-from-head transform

- Where Sony converts the SLAM / INS pose to the pose it gives SteamVR:
  rotation and translation constants, their units, and the order of
  application.
- Which frame the EP 0x83 SLAM pose is expressed in (IMU rig, display
  centre, or between the eyes).
- Give enough detail to check it against the Linux driver's current
  remapping in `steamvr/driver_psvr2/src/pose_source.cpp`.

## 5. Relocaliser (low priority)

- The Set 0x0a payload: layout, when it is sent, and what triggers it.
- What the PC does with the "RP" packets (EP 0x8a, 0xc8780 bytes): the
  layout of the header and the main sections.
- What a minimal driver loses without it (map recovery after tracking loss
  only, or more?).

## 6. Extreme-Env blob modes (low priority)

What sets tracker settings bytes +0x33 / +0x34 in a normal install, if
anything (a SteamVR setting, a registry key, a file)? If nothing does, say
so in one line.

## Deliver

Write `controller-tracking.md` part B with one section per item above. For each
item give:
- the answer, as tables of offsets and values, with formulas in pseudo-C;
- confidence tags and function addresses;
- for item 1, a worked example: the parsed raw values of one camera mapped
  to the struct fields. Use a real blob if one exists
  (`/sys/bus/usb/devices/*/camera_calib0` with the dev module loaded);
  otherwise give the mapping symbolically;
- what is still unknown, and the next function to read.

Finish with a short list of corrections to the existing docs.
