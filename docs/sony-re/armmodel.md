# IR-thread post-solve stages: occluded-controller latch and arm model

Sony `driver_playstation_vr2_orig.dll`, PE timestamp 1741940190, image base
`0x180000000`. Written from the decompile (IDA 9.5) in our own words; no
Sony code is reproduced. Tags: **[sure]** read in code, **[likely]**,
**[guess]**. These functions sat unnamed in the triage
([`triage.md`](triage.md) §3.2) as the four largest undocumented core
functions.

After the optical solve (`0x180189c70`) the IR tracker thread
(`0x18018aaf0`) runs two more stages on every LD frame, both enabled by
default on PC:

1. the **occluded-controller latch** (`0x1801b81c0`, flag `irt+0x4d26c`):
   when the farther controller loses its LEDs behind the nearer one, its
   position is carried along with the nearer controller;
2. the **arm model** (object at `irt+0x4d380`, functions
   `0x1801b0120–0x1801b9d70`, mode `irt+0x4d378`): a shoulder–elbow–wrist
   model driven by the head pose and the controller orientation. It feeds
   the controller UKF a pseudo position measurement when the optical fix
   is stale.

`irt` is the IR tracker object (`qword_180355510`). Frames: the latch works
in the **camera-rig frame with +z forward** (the head pose composed with
the head-from-rig transform, then turned 180° about y, `0x1801b7e40`).

## 1. Driver summary

What a Linux driver gains, in priority order (all userspace):

1. **Keep a controller tracked through short occlusion by the other
   controller.** Two-handed grips and crossed hands hide the far
   controller's LEDs behind the near one. Sony keeps the far controller's
   position rigidly attached to the near one, as seen from the cameras,
   for as long as the near one stays tracked, and blends back over at most
   0.5 s when the far one is seen again (§2).
2. **A real arm model as the no-optics fallback.** Sony's fallback
   position is not "hold the last position": an arm model gives a
   plausible hand position from the head pose and the controller
   orientation, and it is fed to the UKF as a measurement once the optical
   fix is older than 0.3 s (§3).

The Linux driver has neither: `controller_device.cpp` uses its own fixed
arm offset when there is no optical fix, and nothing handles occlusion by
the other controller.

## 2. Occluded-controller latch (`0x1801b81c0`) [sure unless tagged]

### 2.1 Call and inputs

Called once per LD frame, after the optical solve, while `irt+0x4d26c` is
set (default 1; setter `0x180192a50`, no caller found in the DLL):

- `latch` object at `irt+0x4d288`;
- an `OcclusionQueriesVrTracker` object (vtable methods: head pose at a
  time, head-from-rig transform);
- frame time `t` (host µs);
- the per-hand optical result of this frame: valid byte, position,
  orientation, for both hands;
- the per-hand "last optical position" slots (`irt+0x44158`) and "last
  optical fix time" slots (`irt+0x44148`);
- `0.3` (s): a hand counts as **optically tracked** when its last fix is
  less than 0.3 s old.

It asks the controller UKF (`0x18005df40`) for both hands' states at `t`
and at `t + 100 ms`, and expresses the positions in the rig frame.

Return value: the latched hand (0 or 1), −1 when idle, −2 when a latch was
just dropped. The IR thread stores it at `irt+0x4d270`. For a latched hand
it restores that hand's "valid" byte in the stored frame and stamps
`irt+0x4d278 + 8·hand` with `t`. The optical solve reads the latched
position and that time stamp back as its prediction hint for the next
association (`0x180189c70`, globals `0x18035be90` / `0x18035bed8`).

### 2.2 Roles

- **carrier** (`latch+0x40`): the nearer controller (smaller rig-frame z);
- **latched** (`latch+0x44`): the farther one, the one that is hidden.
- If both z are equal, or either is ≤ 0 (behind the cameras), no latch.

### 2.3 Entry (state 0 → 1)

All must hold:

- at least one hand has a UKF state;
- the carrier is optically tracked (fix < 0.3 s), the latched hand is not;
- the latched hand's last fix is less than 0.6 s old;
- the latched hand is less than 1 m from the rig origin;
- the latched hand has no bit `0x20` in its UKF status word (`+0x90`)
  [meaning of the bit: guess, "no IMU"];
- the two controllers **overlap as seen from the cameras**. Each
  controller is treated as a sphere of radius 0.0875 m; let `ρc`, `ρl` be
  their angular radii at their distances and `θ` the angle between the two
  view rays. Overlap means `θ ≤ min(20°, |ρc − ρl| + 5°)`, and the view
  rays must be less than 90° apart (positive dot product);
- timing: either less than 0.3 s since the last release or drop
  (`latch+0xe8`), or at least 0.3 s since the latched hand's last fix
  (the second is already implied by "not optically tracked").

On entry the latch records, for `t` and `t + 100 ms`, the transform from
the carrier's view-ray frame to the latched controller's view-ray frame.
A view-ray frame (`0x1801b91d0`) sits at the controller position with its
z axis along the ray from the rig origin. So the latch keeps the hidden
controller where it was **relative to the line of sight to the carrier**
(angular offset and depth difference), not relative to the carrier's own
orientation.

### 2.4 States

| State | Meaning | Output for the latched hand | Leaves when |
|------:|---------|-----------------------------|-------------|
| 0 | idle | nothing (returns −1) | entry conditions (§2.3) → 1 |
| 1 | latched | position = carrier view-ray frame · recorded transform; orientation = the hand's own UKF orientation at `t` (and at `t + 100 ms` for the prediction slot). Written into the frame result as if it were an optical fix. | carrier has no optical result this frame, carrier ≥ 1 m away, or status bit `0x20` on the latched hand → 3; latched hand has its own optical result this frame → 2 |
| 2 | release | each frame the held position moves toward the UKF position by the fraction `(t − t_release) / 0.5 s` (so it reaches it within 0.5 s) | 0.5 s elapsed, or held position more than 0.15 m from the UKF position → 0 |
| 3 | dropped | hold the last latched position, valid byte cleared (returns −2) | latched hand gets a fix again → 0 |

Positions handed downstream are interpolated between the `t` and
`t + 100 ms` slots by `0x1801b7da0`: `p(τ) = p_t + (p_{t+0.1} − p_t)·(τ − t)/0.1 s`.

### 2.5 Constants

| Value | Meaning |
|------:|---------|
| 0.3 s | "optically tracked" window; also the re-latch timing gate |
| 0.6 s | longest gap since the latched hand's last fix |
| 1 m | largest rig-frame distance of either controller |
| 0.0875 m | controller radius used for the overlap test |
| 20°, 5° | overlap cap and margin |
| 0.5 s, 0.15 m | release blend length and abort distance |
| 100 ms | look-ahead of the second recorded transform |

## 3. Arm model (`irt+0x4d380`)

### 3.1 Frames, hands, inputs [likely for the frame, sure for the numbers]

- Works in the tracker world frame with the **full head pose** `H`: the
  INS head pose composed with two config-manager transforms (IR thread,
  `0x18018aaf0`, around the `0x180156470` calls). Shoulders rotate with
  the whole head, including pitch and roll [sure].
- Axes are OpenVR-like: +x right, +y up, +z back. Evidence: the elbow hint
  moves to +z (behind) for a low hand and to −z (in front) for a raised
  hand, and the wrist sits at +z behind the grip [likely].
- **Hand 0 is right, hand 1 is left** (hand 0 uses the +x tables), matching
  the mirrored left model in `opticalpose.md` [likely].
- Quaternions in these functions are **w first**.
- Per hand and frame the IR thread passes the UKF-predicted controller pose
  (position + orientation) and whether this LD frame had an optical result.

### 3.2 Body constants (`0x180351430`–`0x180351640`) [sure]

| Item | Value (hand 0 / hand 1) |
|------|-------------------------|
| Shoulder offset from the head origin, head frame | (+0.175 / −0.175, −0.275, 0) plus neck (0, 0.0897, z) with z = −0.0978, except in mode 1 where z = 0. On PC: (±0.175, −0.185, 0) |
| Upper arm, forearm | 0.28 m, 0.27 m (`0x1801afc40` arguments; table entries `+0xe0/+0x150`, `+0xfc/+0x16c`) |
| Grip to wrist | 0.08 m along the controller's +z (table entry `+0x00/+0x1c` holds (0, 0, −0.08), applied inverted) |
| Shoulder joint frame | 90° about ±y with offset (±0.0882, −0.0359, 0.0123) (`+0x134` / `+0xc4`) |
| Longest reach | 1.0 m (straight arm beyond it) |
| Forearm cone | the elbow must lie within 75° of the wrist's backward axis (`0x1801afc40`) |
| Elbow hint ("pole") on PC | (±0.4, −0.6, z_h) in the head frame. With `d = clamp(hand_y − head_y, −0.4, 0)` and `f = (d + 0.4) / 0.4`: `z_h = (1 − f)·0.4 − f·0.3`, so +0.4 (behind) for a hand at least 0.4 m below the head, −0.3 (in front) at head height and above |
| Elbow hint, other mode | (±0.3, −0.6, 0.15) (`0x1803515e8/f8`), not used on PC |
| Swing–twist axis | the bone's z axis (`0x1803515b8` = (0, 0, 1)) |
| Twist limit | ±60° (`0x1801b1280`) |
| Hand mask | `0x1803515c4` = 3 (both hands take the PC branch) |

### 3.3 Mode and the dead reference-pose path [sure]

`irt+0x4d378` is 1 from init and nothing changes it. `0x1801b9d70` copies
it to `0x180597460` each frame. In mode 1 it also checks two stored
reference poses (`0x180597470`, one per hand) against the current head pose
(view axes within 90°, up axes within 60°, controller within half the
head's distance) and writes the result to `0x180597464`. The functions
that would capture those references (`0x1801b9640`, `0x1801b9880`) have no
caller in the DLL, so on PC the check always fails (`0x180597464` = 0) and
the "other mode" branches of `0x1801b37b0` / `0x1801b1280` never run.
`0x1801b37b0` overwrites the slot with the current head pose every frame,
so the slowly following "torso" orientation in `0x1801b1280` (2 % slerp
toward the head when the view direction changes by more than 0.5°) ends up
equal to the head orientation on PC.

### 3.4 Per frame, per hand (IR thread) [sure unless tagged]

Only for a hand whose UKF state is valid (otherwise `0x180185bd0` resets
that hand's arm state):

1. **Input smoothing** (`0x1801b08b0`, mode 1 only): keeps the UKF state
   0.1 s ahead and, when the optical fix comes back after a gap, blends
   the stored position into the new one over a configured number of
   frames (fields `+0x2e0..+0x2f4` per hand, stride 0xe0) [likely for the
   purpose].
2. **IK check** (`0x1801b37b0`, PC branch): wrist = grip + R·(0, 0, 0.08);
   shoulder from §3.2; two-bone IK with the hint (`0x1801b0120`: law of
   cosines, elbow bent toward the hint's component perpendicular to the
   shoulder→wrist line), then the 75° forearm cone (`0x1801afc40`). The
   elbow it finds is not kept. The function records the controller and
   head orientations, resets the torso angle, and returns whether the
   wrist is reachable. While a hand has no optical result it latches
   "arm raised" (`+0x222 + hand`) = wrist above shoulder − 0.1 m.
3. **Out-of-view test** (`0x1801b9560`, only if the hand moved
   (|v| ≥ 0.0125) within the last 1.5 s, or a debug flag is set): true
   when the predicted controller is **at least 80° off the view axis**
   (head −z pitched down 5°, `0x1801b93f0`) **and within 1.0 m of its
   shoulder**.
4. **Pseudo measurement** (`0x1801b1280`), when the out-of-view test is
   true **or** the last optical fix is older than the stale limit
   (default 0.3 s; tracker setting `irt+0x442f8` in µs: > 0 sets it,
   < 0 disables it):
   - forward kinematics from the shoulder through shoulder, elbow and
     wrist joint rotations (state `+0x23c/+0x25c/+0x27c` per hand,
     stride 0x10), driven by the controller orientation. The wrist joint
     is slerped 75 % toward the needed rotation, twists are limited to
     ±60° about the bone axis (swing–twist, `0x1801afaf0`), and the elbow
     solution (up or down) follows the "arm raised" flag, flipped when the
     hand moves more than 1.5 cm [likely: how the rotation is split
     between the joints is in `0x1801aeaf0`, `0x1801aefb0`, `0x1801af610`,
     not yet written up];
   - output per hand (stride 0x50 from the object start): +0x10 elbow,
     +0x1c wrist, +0x28 **hand position**;
   - the return value is an angle in radians; the IR thread accepts it if
     it is ≤ 20, which every angle meets (looks like a degrees/radians
     slip) [sure for the code];
   - the hand position goes to the UKF as a **position-only**
     measurement (`0x18005ec00`, rot_free = 1, float argument 1000 at
     record +0x34). The mode argument is 1 while the model agrees with the
     UKF prediction within 0.1 m and the model is valid, else 0; per
     `ukf.md` §4 that selects σ = 0.005 m or σ = 0.05 m.
5. Statistics: distance moved since the last frame, binned at
   0.05/0.1/0.2/0.3/0.4 m into counters (`irt+0x4f4a4`, `irt+0x4f4b0`),
   split by fixes older than 2 s [likely: diagnostics only].

## 4. Function map

| Function | Role |
|----------|------|
| `0x1801b81c0` | occluded-controller latch (§2) |
| `0x1801b7e40` | both hands' UKF states into the rig frame (head pose at `t`, head-from-rig, 180° about y) |
| `0x1801b91d0` | view-ray frame of a point: rotation taking +z to the ray, translation along it |
| `0x1801b7da0` | latched position at a time, linear between the `t` and `t + 100 ms` slots |
| `0x1801b7d90` | `OcclusionQueries` destructor (vtable reset) |
| `0x180192a50` | set latch enable `irt+0x4d26c` (under the IR lock) |
| `0x1801852e0` | IR tracker init: latch enable 1, latched hand −1, arm-model mode 1, arm-model object init `0x180184fa0` |
| `0x180184fa0` | arm-model object init (zeroes, identity joints) |
| `0x180185bd0` | reset one hand's arm state (no UKF state) |
| `0x1801b9d70` | per-frame mode copy and reference-pose check (§3.3) |
| `0x1801b9640`, `0x1801b9880` | reference-pose capture; no callers (dead on PC) |
| `0x1801b08b0` | input smoothing / re-entry blend (§3.4 step 1) |
| `0x1801b37b0` | IK check and orientation bookkeeping (§3.4 step 2); 1500 lines of other-mode code unused on PC |
| `0x1801b1280` | forward-kinematics pseudo measurement (§3.4 step 4) |
| `0x1801b9560`, `0x1801b93f0` | out-of-view test: ≥ 80° from the view axis pitched 5° down, ≤ 1 m from the shoulder |
| `0x1801b0500` | returns the reach limit 1.0 |
| `0x1801b0510`, `0x1801b05c0` | shoulder point: head pose · (shoulder offset + neck offset) |
| `0x1801b0600` | shoulder pose (point + joint-frame rotation) |
| `0x1801b0120` | two-bone IK with pole hint and reach clamp |
| `0x1801afc40` | two-bone IK plus 75° forearm cone |
| `0x1801afaf0` | swing–twist: twist of a quaternion about an axis |
| `0x1801b0f50` | quaternion → axis–angle, angle in (−π, π] |
| `0x180170dc0` | slerp (shortest path; lerp when cos > 0.9999) |
| `0x1801aeaf0`, `0x1801aefb0`, `0x1801af610` | joint solvers used by the FK chain (not yet written up) |
| `0x1801b0dc0`, `0x1801b0e70`, `0x1801b10b0` | other-mode helpers of `0x1801b37b0` (unused on PC) |
| `0x1801b11e0`, `0x1801b99a0` | arm-model helpers called from the optical solve (not yet written up) |
