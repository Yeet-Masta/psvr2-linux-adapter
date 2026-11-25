# Topic: tracking (`controller-tracking.md` part A) — priority 5

Read `common.md` first. Earlier notes: `controller-tracking.md` part H,
`controller-tracking.md` part D, `headset.md` part B. This is userspace work. Describe the
algorithms at the level needed to build an equivalent open one; do not
document every helper.

Scope:
- Headset INS (`0x180170000–0x18018ffff`): how the on-device SLAM pose
  (EP `0x83`) and the IMU are fused (`FUN_18017a5d0`, `FUN_18017cdb0`,
  `FUN_180174410`, `FUN_180180200`), prediction, resets, map latch.
- Controller tracking: per-frame core `FUN_18004b0a0` (called from
  `FUN_18018aaf0`), boot / multi-camera pose search (`0x1800a0000–0x1800fffff`,
  `ComputeBootStats`, `FUN_1800c7f00`, `FUN_1800d5fc0`), the robust cost
  (`RnD::PoseCostMultiCam`, Huber/Tukey), LED-to-blob matching, and the UKF
  (`UkfManager`, `FUN_18005f1b0`): state vector, process and measurement
  models, noise values.

Deliver, for each pipeline: the stages with inputs and outputs; the
equations in pseudo-C; every tuning constant; the coordinate frames and
units; and the lost-tracking and recovery logic.

Skip: linear algebra internals (name the operation only), VrTrace dumping,
and the logging of statistics.
