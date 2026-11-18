# Topic: optical controller pose into SteamVR (`docs/sony-re/opticalpose.md`) — priority 1

Read `common.md` first. Work from `psvr2/src-dev` (branch `dev`), not
`psvr2/src`: `docs/RESEARCH.md` there is the canonical research and wins
over older notes.

## Where the Linux driver stands

LED sync now reaches STABLE on real hardware. The LED pose solver
(`userspace/lib/psvr2_led_solver.c`, RESEARCH.md §9) solves a controller
pose in the **camera rig frame** (camera 0's frame, mm; §8.3 and §8.4).
To drive SteamVR we still need the chain:

    LED model frame -> camera rig frame -> headset tracking frame -> world
    controller IMU frame <-> LED model frame
    controller pose frame reported to SteamVR

Without these transforms the optical pose cannot be used. This topic
fills in those transforms and the fusion around them.

## Questions, in priority order

1. **Rig to headset.** How does Sony place the camera rig in the frame of
   the headset pose (the SLAM/IMU pose from EP `0x83`, RESEARCH.md §5)?
   - VD calibration type 13 (0x34 bytes) is marked "IMU ↔ camera [likely]"
     in RESEARCH.md §3.3. Confirm or correct it, and give its exact layout
     (rotation form, translation units, direction a→b, which camera).
   - Give every fixed rotation, axis flip or offset applied on the way,
     including the `G = diag(−1, −1, 1)` of §8.3 and the head lever arm of
     §5.2 (`vecDriverFromHeadTranslation`). State whether the controller
     pose is formed in the SLAM frame, the head frame or the driver frame.
   - Is camera 0's built-in rig pose (the table in §8.3) used on top of
     the calibration, or replaced by it?
2. **LED model to controller.** In what frame are the LED model positions
   (`0x180257280`, §9.1)? Give the fixed transform from that frame to the
   controller IMU frame, and from the IMU frame to the pose Sony reports
   to SteamVR (the `DriverPose_t` position and rotation, plus any
   `vecDriverFromHeadTranslation` or `qDriverFromHeadRotation` it sets for
   controllers). Per hand: the descriptor byte `0x58` of hand 1 (RESEARCH.md
   §11, question 12) is probably a mirror flag. Trace what it does.
3. **Fusion.** How does the optical pose enter the controller UKF (§9.4)?
   - The measurement model: what is measured (full pose, position only,
     LED pixels), in which frame, and with what noise values.
   - Time alignment: which timestamp the optical measurement carries (LD
     packet `+0x08` mapped how) and how far back it can apply.
   - The IMU-only path when no optical pose arrives: how long until Sony
     reports the controller as not tracking, and what `eTrackingResult`
     and `poseIsValid` it sends in each case.
   - What the solver's per-frame acceptance rule is (§11, question 13): when
     is a solved pose rejected before it reaches the UKF.
4. **Two controllers.** How Sony keeps the two controllers apart when both
   are in view, beyond LED-sync timing (§9.3 says nothing partitions the
   blobs). Does it predict both and associate jointly, or solve each
   independently against all blobs?
5. **PRESCAN window placement** (lower priority). On hardware our PRESCAN
   window (`base = c − period·step/2 − width/2` from the newest LD frame
   time) took 3.5 to 44 s to hit the camera exposure. Check whether Sony
   adds any fixed offset to the LD frame time (exposure vs packet time),
   and what the LD packet `+0x08` time marks.

## Deliver

`docs/sony-re/opticalpose.md`:
- Each transform as numbers (matrix or quaternion plus translation, with
  units and direction), plus the address it comes from.
- The full chain from a solved LED pose to the `DriverPose_t` Sony sends,
  as numbered steps.
- The UKF measurement update, its noise values and its gating, and the
  not-tracking timeouts.
- Then a short list of corrections to RESEARCH.md, in the form its
  "How to edit" section asks for.

Raw numbers and tables from the DLL are fine. Do not copy decompiled code.
Write what the code does in your own words.
