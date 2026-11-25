# Development

State of the dev branch, hardware test procedures, roadmap and credits.

Parts:

- [Part A: Dev branch notes](#part-a-dev-branch-notes)
- [Part B: Hardware tests](#part-b-hardware-tests)
- [Part C: Status and roadmap](#part-c-status-and-roadmap)
- [Part D: Acknowledgments and provenance](#part-d-acknowledgments-and-provenance)

---

## Part A: Dev branch notes

This is the `dev` branch. It holds everything taken from the reverse-engineering
notes. The research lives in `docs/sony-re/`; superseded statements have been
corrected in place and are listed once, in overview.md §A.12. **None of it has been
tested on hardware yet.** Sony's driver was used only as a specification and a
source of raw data (constants and tables). No code was copied.

### Kernel module

| Change | Files | Source |
|--------|-------|--------|
| `/dev/psvr2-ctrl`: control reports with an allowlist. SET: 05 08 09 0a 0b 12 14 17 1f 22 25 27 28 29. GET: 81 8a 8f 97 a1 a4 a5 a6 a9 de | `psvr2_ctrl.c`, `psvr2_uapi.h` | overview.md part B "Control reports" |
| Headset clock map (Get 0xde every 500 ms, RTT < 1000 µs, 100-entry ring, reset on a > 2 ms jump, drift clamped to ±50000 ppb). IMU and SLAM timestamps now use it | `psvr2_ctrl.c`, `psvr2_status.c`, `psvr2_slam.c` | headset.md §A.5.1 |
| Camera mode 0x10 stays on. It is re-sent once a second until camera and IMU calibration (VD types 10 and 12) arrive. V4L2 streaming switches to mode 1 and restores 0x10 when it stops | `psvr2_ctrl.c`, `psvr2_camera.c` | controllers.md §C.2.7 |
| VD calibration container parsed. Sysfs files `camera_calib0`, `camera_calib1`, `imu_calib`, `imu_set_calib` | `psvr2_aux.c`, `psvr2_usb.c` | overview.md part B "VD" |
| `/dev/psvr2-ld` (LED detection, 8-packet queue) and `/dev/psvr2-rp` (relocaliser, 2 packets). The magic is checked; each read returns a header followed by the payload | `psvr2_aux.c` | controllers.md §C.1 |
| Set 0x14 default `{0x55, 500, 0x24800001, 0}` on connect | `psvr2_aux.c` | controllers.md §C.2.5 |
| Aux endpoint errors are retried after 500 ms with a halt clear | `psvr2_aux.c` | |
| SLAM record: state, substate, flag, velocity, gyro/accel bias, variance, reset counter, map id. TRACKING flag set by Sony's rule `state == 3 \|\| flag == 1 \|\| substate == 5` | `psvr2_slam.c`, `psvr2_uapi.h` | controller-tracking.md §A.2.1 |
| See-through button toggle, brightness follow, Get 0x97 polled once a second | `psvr2_ctrl.c` | |
| Resume: Set 0x05 subcmd 2 sent 3 s after resume | `psvr2_usb.c` | |
| `ready` sysfs attribute | `psvr2_usb.c` | |
| udev: the new nodes get group `input`; brightness, rumble, see_through and edid_type become group-writable | `99-psvr2.rules` | |

### libpsvr2

- Control, clock, quit (0x27), see-through, EDID type, firmware/board info,
  calibration read and LD packet read APIs.
- `psvr2_pose` gains device time, tracking flag, SLAM state, velocity, biases
  and map id.
- Lens distortion uses Sony's exact LUTs and constants. The red channel now
  gets the y offset (max difference from the old code: 4.3e-4).
  `scripts/sony-re/extract_lens_lut.py` regenerates `psvr2_lens_lut.h` from
  your own copy of the DLL.
- Hidden-area mesh (`psvr2_hidden_area`, 108 vertices per eye).
- **New:** `psvr2_cam_calib_parse` (camera calibration blob, both record
  layouts), `psvr2_cam_model_from_calib` (raw values → 508×508 model,
  overview.md §A.8), `psvr2_cam_project` (OpenCV frame),
  `psvr2_cam_project_rig` (Sony's frame, 180° about Z) and
  `psvr2_cam_extrinsic_to_rig` (R′ = G·R·G, t in mm). Checked against
  Sony's form to 1.6e-9 px.
- **New:** the controller LED pose solver, see the last section.

### SteamVR driver

- EDID type 1 at init and type 0 at cleanup, a 120 Hz gate (firmware later
  than 0x06000000), and quit 0x27 on shutdown.
- Sony's projection tangents. Properties: IPD range, tracking style,
  firmware/hardware revision, frame rates, hidden-area meshes, mesh
  resolution 80, proximity sensor.
- Not-worn pose throttle, see-through pause properties.
- Play area: a 1.5 m circle written at the first tracking pose
  (`provide_chaperone`). Controllers follow the headset's universe id.
- Pose: the history is reset on a SLAM reset or map change. SLAM velocity is
  used when tracking. Running_OK is reported only while SLAM is tracking.
- Sense controller changes:
  - Sony Bluetooth framing (`sony_bt_framing`);
  - vibration strength (`vibration_strength`);
  - firmware info (feature 0x20);
  - Sony's battery mapping;
  - the polling rate is opt-in (`set_polling_rate`).
- **New, experimental, off by default (`optical_tracking`):**
  - `SenseHid` sets output flag2 bit 1 for controller firmware revision
    > 0x253, as Sony's driver always does (a libpad switch it turns on at
    start). Before this was added, the controller never lit its LEDs on
    hardware (2026-11-18).
  - `SenseHid` feeds every input report into libpad's host↔controller clock
    fit. Every output report carries the LED-sync record: phase, seq,
    period, base time ×3, cycle ×3, LEDs. This follows Sony's placement
    rules, including the 0x80/0x81 capability probe and the sync-mode choice.
  - `OpticalTracker` (`optical_tracker.cpp`):
    - reads `/dev/psvr2-ld` and maps each frame's headset time to host time;
    - keeps Set 0x09 (Normal / Many Blob) and Set 0x14 (per-preset LED
      window) in sync with the headset's echo, as Sony does;
    - decodes the blobs;
    - turns LEDs off while see-through is on;
    - logs the camera calibration once.
  - `LedSyncer` (`led_syncer.cpp`) is the per-controller INIT → PRESCAN →
    BROAD → BG → STABLE machine. It includes the Gray-code blink decoder,
    the drift fit, and the BG brightness estimator. Rules and timings follow
    overview.md §A.7 (11/20-frame windows, BG marker frames, target 10,
    ±40 fit reset).
- Head pose: Sony's lever arm `vecDriverFromHeadTranslation = (0, 0.0208,
  0.09255)` m is applied (overview.md §A.5.2); the rotation part was already
  in the remap.

### A.120 Hz frame timing (SteamVR bug workaround)

At 120 Hz SteamVR scanned out 119.88 Hz but logged `Updated HMD
Prop_DisplayFrequency_Float to 90.000000` and paced frames for 90 Hz (judder
and tearing). Cause, found in vrcompositor (Linux, 2026-11-18): the mode pick
loop writes the chosen rate into its frame timing only when the winning mode
is not the first one at the HMD resolution; otherwise a 90.0 default stays.
The 90/120 EDID lists 4000x2040@119.88 before @89.91. The runtime rate-change
path cannot fix it (its display setter is a stub on Linux).

Fix: `steamvr/vk_mode_order`, an explicit Vulkan layer that, inside
vrcompositor only, returns each resolution's modes in ascending refresh. The
installer writes its manifest to `~/.local/share/vulkan/implicit_layer.d/`.
The loader enables it only when `_STEAMVR_LOGGING_READY=1` (exported by
SteamVR's launch scripts); setting `VK_INSTANCE_LAYERS` from vrserver did not
work, because the compositor is started through steam-runtime-launch-client
with only `LD_LIBRARY_PATH` passed. `PSVR2_VK_MODE_ORDER_DISABLE=1` turns it
off. At 90 Hz the 90.0 default stays (0.1 % off
89.91, as before with the 90-only EDID).

### Not done, and why

| Item | Why |
|------|-----|
| Controller **optical pose**: Sony's UKF | Done in simulation, not yet run on hardware (2026-11-20): `controller_ukf.cpp`, see "Controller filter" below. Setting `optical_ukf` false restores the plain optical position. Arm model and occlusion latch ported 2026-11-25, also simulation only. |
| Blob association in LED sync | Sony associates blobs by predicted pose. Without a pose, the tracker syncs **one controller at a time** and `BlinkFinder` picks, per camera, the blob whose on/off pattern repeats every 11 (BROAD) or 20 (BG) frames; steady room lights and random flicker are rejected (simulated: 70 lights, 3 flickering lamps, 0 wrong picks). The first hardware run had 50–80 room blobs per camera, which the old "≤ 4 blobs" rule could never decode (stuck in BROAD). Hardware, 2026-11-18 (dark room): PRESCAN → BROAD → BG reached. A dark sync LED often still shows a dim blob (weight ~4 vs ~13 lit), so lit/dark is judged by weight against the site's own range; in BG the BROAD pick is kept while seen. First STABLE on hardware 2026-11-18 (17.7 s in BG, held 44 s). Two own changes made it lock: the drift fit slope is sent as a dimensionless rate (BG: −slope·step, BROAD: slope·2.5·step; raw it changed the cycle by 0.14 ppm against an 11 ppm drift), and every BG fit needs 10 samples (Sony: 40 after the first; BG gives ~1.3 samples/s). Open: PRESCAN → BROAD took 3.5–44 s with the controller in view. |
| Drift-fit units | BROAD: same as Sony. Sony's BROAD fit holds µs, so its raw slope is already a rate. BG: own full-size rate `−slope·step`; Sony sends the BG slope raw in steps/µs (`sony-re/controller-tracking.md` §B.3.5). |
| Controller preset | Done: Sony's preset is libpad's sync-mode byte (`sony-re/controller-tracking.md` §B.2.3), which is what the driver passes. |
| Extreme-Env blob modes (2–4) | They need tracker settings that are 0 in a normal install. |
| Changing EDID ids | The current ones work. |
| Controller input remap to Sony's L1 behaviour | The project has its own input profile. |
| Relocaliser (Set 0x0a, `/dev/psvr2-rp` consumer) | Large and optional. The kernel side already exposes the data. |

### Testing

Hardware hand-off for the 2026-11-25 ports (gyro clip per rev, arm model,
occlusion latch, SLAM delta pose): `scripts/hw-handoff.sh` builds, runs the
offline tests, installs, then guides a 2-minute session in the headset with
voice cues and packs `~/psvr2-hwtest-<date>.tar.gz` (logs, SLAM `+0x2c`
capture and verdict). Needs passwordless sudo; `--offline` runs only the
build and simulations.

The module, library and driver build clean (kernel 7.0, gcc). To try them:

1. Build and load the dev module. The udev rules are new, so install them too:

   ```bash
   make -C kernel && sudo rmmod psvr2; sudo insmod kernel/psvr2.ko
   ```

   ```bash
   sudo cp kernel/99-psvr2.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger
   ```

2. Check `dmesg` for "calibration type 10 received" and the clock lines.
   `/dev/psvr2-ctrl`, `/dev/psvr2-ld` and `/dev/psvr2-rp` should exist.
3. Point SteamVR at the dev build. The `drivers/psvr2` symlink currently
   targets `src`:

   ```bash
   ln -sfn ~/Claude/psvr2/src-dev/steamvr/driver_psvr2/build/psvr2 ~/.steam/debian-installation/steamapps/common/SteamVR/drivers/psvr2
   ```

4. Optional: set `"optical_tracking": true` in the `driver_psvr2` section of
   `steamvr.vrsettings`. Then look in `vrserver.txt` for `psvr2: optical:`
   and `psvr2: ledsync:` lines, and send the camera calibration lines back.

### Controller LED sync on hardware (2026-11-18)

From a 47 s LD trace (one controller, firmware 0x0306, preset 2):

- **PRESCAN offset.** The LEDs were lit only at re-syncs whose base time
  sat 1.87-2.58 ms (controller clock) after the one our time mapping gives,
  never at 0. The driver now starts at `optical_sync_offset_us` (default
  2000), sweeps ±8 ms in 2 ms steps only while nothing is lit, and saves the
  offset after 3 s lit.
- **BROAD does not lock.** With LED 1 as sync LED and offset 0, the LED
  showed on exactly every third LD frame, in bursts, which matches none of
  the 11-frame codes (overview.md §A.7.9). Cameras show no third-frame pattern
  in PRESCAN, so it is the controller. Open: what this firmware's BROAD
  emits.
- **BROAD divider test (2026-11-20, not yet run).** Sony's side has no
  frame divider: the decoder reads one code frame per LD packet whose
  sequence changed (controllers.md §C.2.8, §C.5.2; checked again in IDA, no
  frame-counter skip in `0x18018aaf0`). So the PC cannot explain the
  third-frame pattern; it is controller behaviour. Hypothesis: this
  firmware holds each BROAD code frame for three LD frames. With
  `optical_sync_broad` on, `LedSyncer` now decodes the code both per frame
  and over every third frame (three phases) and logs both once a second
  (`psvr2: ledsync: BROAD decodes in N frames: every frame a, every 3rd
  b/c/d`). If one phase decodes steadily, set `optical_broad_divider` 3 to
  act on it. Offline check: both dividers decode all 7 codes at all 33
  phase shifts. If neither decodes, capture the LD trace at a few fixed
  offsets (the PRESCAN sweep tool) to see what the pattern carries.
- **So tracking runs in PRESCAN** (default; `optical_sync_broad` true for
  Sony's full sequence): all LEDs lit in a 2 ms window, libpad re-centres it
  every 1 s, both controllers at once, and the pose solve runs while LEDs are
  in view.

### Optical controller pose (steamvr/driver_psvr2/src/optical_pose.cpp)

From `controller-tracking.md` part C. Runs per LD frame for each controller in
STABLE (`optical_tracking` must be on).

- Rig: camera 0 at Sony's built-in pose, **variant 2**; cameras 1-3 from
  the calibration extrinsics, read as "a -> b" = b <- a. On this headset
  that lands within 3 mm / 2 deg of Sony's variant-2 table (variant 1:
  up to 75 deg off, so variant 2 it is).
- Offline check (recorded BG frame, right controller, rev 0x0224 = Sony's
  "other" model, no mirror): boot finds a pose 41 cm ahead with 28-30 LED
  matches at 0.78 px, stable to 2 mm over 35 frames. Weak boots (4-9
  matches) were wrong, so boots need >= 12 matches (own floor).
- World <- rig = head pose at the LD frame time (interpolated from the pose
  history) ∘ head <- rig. Default head <- rig: 180 deg about y (rig +x
  left, +z forward; OpenVR head +x right, +z back), origin at the OpenVR
  head origin. Settings `optical_rig_offset_x/y/z` (m, head axes) and
  `optical_rig_tilt_deg` adjust it. Sony's chain (`INS · A⁻¹ · B`) is not
  used: whether its X frame is this rig frame is open (controller-tracking.md §C.8),
  and taken literally it would put the cameras at the eyes. Setting
  `optical_pose_log` writes rig-frame poses and head poses to
  `~/psvr2-optical-pose.txt` (3 min) to fit head <- rig from a capture.
- IMU <-> LED model: Sony uses a pure translation (pad +0x48). Our fusion's
  IMU axes may differ from Sony's, so until learned the boot tries the 24
  axis rotations (and for the left hand, mirror on/off). A guess is kept
  after 30 frames over 30 deg of rotation whose optical orientation agrees
  with gravity, then saved as `optical_remap_left/right` and
  `optical_mirror_left`. Fixes reach the controller only after that.
- Controller: a fix sets the IMU position (with Sony's 5 cm / 20 cm gate)
  and pulls `yaw_offset_` toward the optical heading; head-based
  recentering is off while fixes arrive. With no fix yet, the arm model
  stays.
- LED-sync input from the pose (Sony, controller-tracking.md §B.3.1): a hand solved
  < 1 s ago feeds LED sync from this frame's pose association: the sync
  LED is the most face-on model LED with a matched blob (view angle / 2 in
  (0, 45]), kept while seen within 1 s; per camera its blob weight, count
  1; a frame without a solve or match reads as unseen (Sony's w = 0). Such
  a hand does not use or own the blink finder, so both hands can sync at
  once. The pose is solved before LED sync each frame, and with
  `optical_sync_broad` also in BROAD/BG/PRESCAN while LEDs are in view.
  Not yet checked on hardware.
- Pose check: tracking passes gravity (from the fused IMU orientation and
  the learned mount) and the prediction as the orientation prior to
  Sony's check; boots with a learned mount get the gravity check.
- Two controllers without a pose: one syncs (BROAD/BG) at a time, sharing
  the blink finder. While the other syncs, a STABLE controller counts as in view
  when its optical fix is < 1 s old, and its 100 s BG re-check waits (own
  rules; before, the other hand's sync made it look unseen and it fell to
  PRESCAN after 15 s). Not yet checked on hardware.
- Open: the second controller is only tracked after the first reached
  STABLE; boot costs ~5 ms, one hypothesis per
  frame (up to ~10 s to learn the remap); no UKF.

### Controller filter (steamvr/driver_psvr2/src/controller_ukf.cpp)

Our own filter with Sony's behaviour (`controller-tracking.md` part G), replacing the
plain optical position and the heading pull when `optical_ukf` is on
(default).

- 15-dim error state; per IMU sample the nominal state is integrated; one
  covariance step per fix over the interval since the last, with the mean
  IMU sample and Sony's rate-dependent noise; sigma-point update (2n points,
  α 1, κ 0) on position + 10 cm axis tips; Sony's P0, gate, noise zones,
  divergence limits; fixes applied at their frame time, IMU replayed.
- Own changes, each from the simulator (`tools/ukf_sim`):
  - rotation noise floor 1 mm (Sony's is 0: with 0 the filter diverged,
    66 mm rms);
  - gate widened to 3σ of the predicted position, and a far fix after a
    stale gap (> 0.3 s) restarts the filter at once (with Sony's fixed gate
    every fix after a 1 s dropout was rejected);
  - stationary damping only while fixes are stale (applied per fix it
    halved the accuracy in smooth motion).
- IMU samples are stamped on the host clock: controller-clock steps pulled
  toward arrival minus `controller_prediction_ms`. The pose is reported at
  the filter's time with `poseTimeOffset` set from it.
- Output: filter pose while the last fix is < `optical_ukf_hold_ms` (300)
  old, then that fix's position held, rotation from the filter (Sony).
- Gyro clip per hardware rev (Sony's setImuPose): rev 1 ±33.16 rad/s,
  others ±66.32.
- Arm model (`arm_model.cpp`, setting `optical_arm_model`, default on):
  Sony's PC body constants; once the fix is 0.3 s old (or the controller is
  out of view after motion) a position-only pseudo fix at 60 Hz, σ 5 mm
  within 10 cm of the filter, else 50 mm. Its FK chain is simplified
  (`arm_model.h`). It does not change the output (still the held fix, as
  Sony); it keeps the filter near the body for when the LEDs come back.
  Simulator `UKF_ARM=1`: filter error at the end of the 1 s dropout 74 mm
  against 194 mm without it.
- Occlusion latch (`occlusion_latch.cpp`, setting `optical_latch`, default
  on): the far controller hidden behind the near one is carried along its
  line of sight to the near one and fed as a position-only fix; 0.5 s
  release blend. `tools/latch_sim` checks the states. Driver log
  `optical latch: <hand> carried by the other controller`.
- Simulator (1 kHz IMU with bias, 60 Hz fixes with 1 mm / 0.5° noise,
  25 ms latency, 5 % 10 cm outliers, 1 s dropout): 0.95 mm / 0.15° rms
  against 29.5 mm for the newest fix held; biases learned to < 2 %; all
  outliers gated.
- Open: real measurement noise of our LED solver (the sigma floors are
  settings), the IMU host-time mapping and the head-frame axes of Sony's
  noise ellipsoid need a hardware run.

### Controller LED pose solver (userspace/lib/psvr2_led_solver.c)

Status: builds, in libpsvr2.a, `make check` passes. NOT wired into
OpticalTracker yet.

- Implements overview.md §A.9: cone visibility, 120 mm self-occlusion cull,
  Huber-IRLS LM (Nielsen), MSAC score, orientation-prior 2-point boot over
  Sony's LED-pair table, Sony's < 7-inlier boot rule (15/30 deg to the IMU
  prior, < 500 mm).
- Sony's gated match (`match_sony`, controller-tracking.md §B.2.11, `cfg.sony_match`,
  default; driver setting `optical_sony_match`): three passes with
  neighbour support, anchors, and Sony's one-blob-two-LEDs cleanup.
- Sony's track step (`psvr2_led_track_ex`, controller-tracking.md §B.2.16): first gate
  3 x SEARCHRADIUS x scale per 60 Hz frame since the prediction, second
  1.5 x POTHOLE x scale; then Sony's pose check
  (`psvr2_led_pose_check`): gravity 20 deg, orientation 30 deg to the
  prior, LED 0 depth 5-1500 mm.
- Own additions (not traced in Sony's code):
  - our gated match (`cfg.sony_match = 0`): per-camera image-shift
    consensus, then nearest blob, Sony's outlier cut;
  - track acceptance on top of Sony's (`cfg.own_accept`, default on):
    >= 4 matches, >= 3/4 of the LEDs facing a camera (< 70 deg) matched,
    rms <= 1.5 px. On -1 the caller should boot.
- LED models: `scripts/sony-re/extract_led_header.py <dll> userspace/lib/psvr2_led_models.h`
  generates the header (raw float bits from the DLL);
  `psvr2_led_model_load(rev, &model)` fills a model.
- `make check` (`test_led_solver.c`): Sony models rev 1..7, 4 cameras near
  the rig positions, 200 random poses each, 0.3 px noise, 2 clutter blobs
  per camera, blobs for LEDs < 100 deg off-axis (28 LEDs matched per pose,
  as on the recorded hardware frame). With Sony's match and track step:
  track 199-200/200 right from a ~2 deg / 9 mm prediction, 0 wrong; boot
  198-200/200 from an IMU orientation ~5 deg off, 0-2 wrong. Before
  (our match, fixed 3 px gate): track 177-186, boot 191-199.
- Sony's match needs neighbours: with blobs only < 80 deg off-axis
  (`LED_VA=80`, 18 LEDs per pose) it drops to track 166-188/200 and fails
  the check, while ours keeps 192-200. If hardware shows few LEDs per
  camera, set `optical_sony_match` false. The IMU priors (`LED_PRIOR=1`)
  change nothing in the test: its wrong poses are small misses, not the
  flips the 20/30 deg checks catch.
- Open:
  - rev 1 (20 LEDs): was boot ~120/200 with 12 wrong. Cause: Sony's
    boot pair table fits the 17-LED layout; its back-trio pairs (0,16),
    (16,15) miss rev 1's back trio (0, 18, 19), and Sony does not remap
    (controller-tracking.md §B.2.13). Own fix in `boot_led()`: 15 -> 18, 16 -> 19 on
    20-LED models. Now 195/200, 0 wrong. Sony's multi-camera boot has the
    same gap (its single-camera RANSAC boot runs first).
  - hand: answered. The hand-1 descriptor byte is the char 'X': Sony
    mirrors the left model in x (positions and normals) and reverses its
    LED index lists (controller-tracking.md §C.2b.2). `optical_pose.cpp` mirrors the
    left model; our model has no index lists, so nothing to reverse.
  - Sony's gravity check (3/6 deg, accepts 5-6 inlier boots) is in
    `psvr2_led_boot_ransac` (below). `psvr2_led_boot_oriented` still has
    only the orientation rule; `optical_pose.cpp` needs >= 12 LEDs (18
    without a recent fix) there anyway, above Sony's 7.
- Sony's RANSAC boot (`psvr2_led_boot_ransac`, controller-tracking.md §B.2.19): P3P over
  3 LEDs x 3 blobs, ring stage then random stage, MSAC score per camera,
  every hypothesis through the pose check; per-camera refine; our
  support >= 0.75 gate (`own_accept`). Needs no heading: the driver
  (`optical_ransac_boot`, default on) tries it before the heading sweep
  when the remap is known, with gravity only, or with the current heading
  while fixes are recent. Test: 178-196/200 right with orientation +
  gravity, 154-183 with gravity only, 0 wrong; ~6 ms per call. The left
  model needs its LED rings reversed (`psvr2_led_model_mirror_x`). Not
  validated on hardware.
- Needs hardware before wiring: rig camera poses, blob pixel scale vs 508.

- Headset INS (`userspace/lib/psvr2_ins.c`, kernels `sie/irtracker/psvr2_ins_core.c`):
  SLAM pose carried to "now" with the 2 kHz IMU, Sony's strapdown and blender.
  Needs the IIO buffer udev rule (`kernel/99-psvr2.rules`). Driver log:
  `headset INS running (quaternion order N, IMU axis map M, fit F)`; if it
  never appears, the pose path is the plain SLAM pose as before. Setting
  `headset_ins`. Not yet tested on hardware.
  `headset_ins_delta` (default off): Sony's mode-0 blender on the SLAM
  record's second pose (+0x2c, carried by the kernel since 2026-11-25),
  with the stillness switch. Turn it on only once a capture shows +0x2c is
  a frame offset (`scripts/hw-handoff.sh` checks it).

---

## Part B: Hardware tests

These tests give the data needed to finish controller optical tracking and to
check the head-pose change. Each test says what to do and what to send back.
Run them in order: later tests need the setup from earlier ones.

Everything stays on your machine. Send back only the log lines asked for.

### B.0 Setup (once)

1. Build and load the dev kernel module, and install its udev rules:

   ```bash
   cd ~/Claude/psvr2/src-dev && make -C kernel && sudo rmmod psvr2; sudo insmod kernel/psvr2.ko
   ```

   ```bash
   sudo cp ~/Claude/psvr2/src-dev/kernel/99-psvr2.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger
   ```

2. Point SteamVR at the dev driver build:

   ```bash
   ln -sfn ~/Claude/psvr2/src-dev/steamvr/driver_psvr2/build/psvr2 ~/.steam/debian-installation/steamapps/common/SteamVR/drivers/psvr2
   ```

3. With SteamVR closed, add these two lines inside the existing
   `"driver_psvr2" : { ... }` section of
   `~/.steam/debian-installation/config/steamvr.vrsettings`:

   ```
   "optical_tracking" : true,
   "optical_log_blobs" : true,
   ```

   To undo, set both to `false`.

4. Start SteamVR once, wear the headset until the view is up, then close
   SteamVR. Check that the module saw the calibration:

   ```bash
   sudo dmesg | grep -i psvr2 | tail -40
   ```

   **Send back:** the dmesg lines that mention calibration or the clock.

### B.1 Camera calibration

Start SteamVR, wear the headset for about 10 s, then close SteamVR.

```bash
grep "psvr2: optical:" ~/.steam/debian-installation/logs/vrserver.txt | head -60
```

**Send back:** all the lines from the command, especially the
`cam N ... K ...`, `cam N raw ...`, `cam N model ...` and `extrinsic ...`
lines.

**Answers:** the real calibration image size (is 508/W exact?), whether the
unused distortion terms are about 0, and the extrinsic layout.

### B.2 LED sync

1. Turn on both Sense controllers and start SteamVR.
2. Wear the headset. Hold both controllers in front of you, about 40 cm
   away, for 30 s. Watch the controller lights: they should flicker, then
   settle.
3. Close SteamVR.

```bash
grep -E "psvr2: (ledsync|optical):" ~/.steam/debian-installation/logs/vrserver.txt | grep -v "  cam\|extrinsic" | head -80
```

**Send back:** the output. Also say what the lights did: flickered and
settled, stayed off, stayed on, or kept flickering.

**Answers:** whether each controller reaches `STABLE`, and where it stops if
it doesn't.

### B.3 Which camera is which (cover test)

The headset has four cameras on the front. Name them from **your** point of
view while wearing it: top-left, top-right, bottom-left, bottom-right.

1. Start SteamVR and wear the headset. Hold one controller still in front of
   you, about 40 cm away, at chest height. The other controller stays off.
2. Wait 10 s. Then cover **one** camera fully with a finger or a piece of
   tape for 10 s and uncover it. Wait 5 s.
3. Repeat step 2 for each camera, in this order: top-left, top-right,
   bottom-left, bottom-right.
4. Close SteamVR.

```bash
grep "psvr2: blobs:" ~/.steam/debian-installation/logs/vrserver.txt | tail -400 > ~/psvr2-test3.txt
```

**Send back:** `~/psvr2-test3.txt` and the order in which you covered the
cameras, if it differed.

**Answers:** which camera index (0–3) belongs to which physical camera. The
`seen` count of the covered camera drops to 0.

### B.4 Pixel range of each camera

1. Start SteamVR and wear the headset. Turn on one controller.
2. Slowly sweep the controller across your whole field of view for 30 s: far
   left to far right, top to bottom, and near (20 cm) to far (arm's length).
   Make sure it reaches the edges where you can barely see it.
3. Close SteamVR.

```bash
grep "psvr2: blobs:" ~/.steam/debian-installation/logs/vrserver.txt | tail -400 > ~/psvr2-test4.txt
```

**Send back:** `~/psvr2-test4.txt`.

**Answers:** the pixel range of the blob centres (`x ..`, `y ..`). This shows
whether blob coordinates are in the 508×508 model image or in the sensor's
own pixels, and the largest blob size (`box`).

### B.5 Camera directions (fixed positions)

This gives each camera's viewing direction relative to the headset.

1. Sit still and keep your head still, looking straight ahead. Start SteamVR
   with one controller on.
2. Hold the controller at each position below for 10 s, with the light ring
   facing you. Say "now" to yourself and count to 10; the exact time doesn't
   matter, but keep the order:
   1. straight ahead, 50 cm, at eye level;
   2. 50 cm ahead, 30 cm to your left;
   3. 50 cm ahead, 30 cm to your right;
   4. 50 cm ahead, 30 cm above eye level;
   5. 50 cm ahead, 30 cm below eye level (about chest height).
3. Close SteamVR.

```bash
grep "psvr2: blobs:" ~/.steam/debian-installation/logs/vrserver.txt | tail -400 > ~/psvr2-test5.txt
```

**Send back:** `~/psvr2-test5.txt`.

**Answers:** where each position lands in each camera image, and so the
cameras' rotations (camera 0's built-in rotation is not documented).

### B.6 Head pose (lever-arm change)

The dev driver now places the head origin between your eyes, not at the
headset's motion sensor. This needs no controllers. Set
`"optical_tracking" : false` first if test 2 showed problems.

1. Start SteamVR and stand in a room with a clear object about 1 m away,
   such as a table edge or a SteamVR menu.
2. Keep your eyes on the object. Turn your head left and right (about 45°
   each way), then nod up and down, several times.
3. Watch whether the object stays still, or slides and swims as you turn.
4. Repeat with the original driver for comparison:

   ```bash
   ln -sfn ~/Claude/psvr2/src/steamvr/driver_psvr2/build/psvr2 ~/.steam/debian-installation/steamapps/common/SteamVR/drivers/psvr2
   ```

   Switch back to dev afterwards with the command from step 0.2.

**Send back:** for each driver, whether the object stays still, slides a
little, or slides a lot, and in which motion (turning or nodding).

**Answers:** whether the 2.08 cm / 9.255 cm head offset is right in our
frame.

### After testing

To return to the original driver:

```bash
ln -sfn ~/Claude/psvr2/src/steamvr/driver_psvr2/build/psvr2 ~/.steam/debian-installation/steamapps/common/SteamVR/drivers/psvr2
```

---

## Part C: Status and roadmap

The goal is to expose as much PSVR2 functionality on Linux as possible, with a
hybrid design: a thin kernel module integrates device data into standard Linux
subsystems (IIO, input, V4L2), and userspace handles the heavier work (`libpsvr2`
and the SteamVR driver).

### What works today

#### Kernel module (`psvr2.ko`)

- **IMU** — accelerometer + gyroscope at ~2 kHz via **IIO**, with correct
  `*_scale` (m/s² and rad/s).
- **Controls** — function button, proximity (worn/removed) and the IPD dial via
  the **input** subsystem.
- **6DoF pose** — the headset's onboard tracker as a `struct psvr2_pose_sample`
  stream on **`/dev/psvr2-pose`** (`read()` + `poll()`; ABI in
  `kernel/psvr2_uapi.h`).
- **Eye/gaze** — per-eye and combined gaze point/direction, pupil diameter and
  blink on **`/dev/psvr2-gaze`** (stream kept alive automatically).
- **Cameras** — the two bottom tracking cameras as a **V4L2** capture device
  (`/dev/videoN`, 1280×640 grayscale stereo).
- **Brightness** via **sysfs**, and **debugfs** raw-frame dumps
  (`raw_status`, `raw_slam`, `raw_gaze`) for protocol work.

The module binds only the vendor interfaces it implements, so `snd-usb-audio`
and `usbhid` keep their interfaces.

#### Userspace

- **`libpsvr2`** — a small C API over the device nodes: device discovery,
  float-converted 6DoF pose and gaze streams (with `poll()` fds), the latest
  scaled IMU sample, the camera-node path, and brightness control.
- **SteamVR / OpenVR driver** (`steamvr/driver_psvr2`) — the PSVR2 as a native
  SteamVR HMD: **direct-mode display** (the runtime DRM-leases the headset
  connector and lights the panel while the desktop keeps running) and **6DoF head
  tracking** in the correct OpenVR frame. Per-eye FOV, lens distortion, and
  in-headset spatial calibration are still being refined — see
  [getting-started.md part C](getting-started.md#part-c-steamvr-driver).
- **Sense controllers, eye tracking, haptics** — see getting-started.md part C: Bluetooth Sense
  controllers with every input, PCM haptics and adaptive triggers (3DoF +
  arm-model position for now), OpenXR eye gaze, Sony's lens model, the IPD
  dial, headset rumble (jailbreak), and a PSVR2 Toolkit-compatible IPC server.

### Roadmap

Contributions in any of these areas are welcome.

#### Kernel / protocol

- **More camera modes** — the interleaved multi-view (controller-tracking /
  fisheye) modes and the SLAM tracking-camera frames, with a V4L2 control to
  select between them.
- **Controller tracking camera mode** — the camera mode Sony's driver uses to
  see the Sense controller LEDs, and the LED sync it negotiates with each
  controller (Monado's `pssense` driver has the controller side of the sync).
- **Multi-headset support** — per-device node naming (the `/dev/psvr2-*` nodes
  currently assume a single headset).
- **Wider gaze ABI** — several gaze packet fields are still undecoded.

#### SteamVR driver

- **6DoF controller tracking** — feed the controller-tracking camera frames to
  a constellation tracker (Monado's `t_constellation` tracks Sense controllers
  with Rift CV1 sensors and ships the Sense LED models) and fuse with the
  controller IMU, replacing the arm model.
- **Passthrough camera calibration** — real intrinsics/extrinsics for the
  bottom cameras, so Room View's 3D mode lines up (the camera is exposed today
  with estimated geometry).
- **Hand tracking** from the headset cameras.
- **Confirm Wayland** direct-mode display acquisition (DRM leasing via
  `wp_drm_lease_device_v1`; not yet validated, see getting-started.md part C).
- **Eye-camera feeds** for Baballonia (jailbreak only) and per-user gaze
  calibration.

#### Other runtimes

- **OpenXR / Monado** and **OpenHMD** integrations layered on `libpsvr2`.

### Display

The 4000×2040 panel is a standard DisplayPort (DSC) sink driven by the GPU's DRM
stack — not by this module. A VR runtime acquires it directly via **DRM leasing**
(see [getting-started.md part C](getting-started.md#part-c-steamvr-driver)); the panel can also be brought up as an ordinary
output for development (see [getting-started.md part B](getting-started.md#part-b-display-bring-up)). An amdgpu DSC/FEC EDID
quirk is carried in `patches/` for setups that need it.

### Audio

Handled by the in-tree `snd-usb-audio` driver; this module deliberately leaves
the audio interfaces alone.

---

## Part D: Acknowledgments and provenance

This project stands on a lot of public prior art. The work below made it
possible — our thanks to everyone involved. Material that was reused or directly
informed by a source is credited here and in per-file SPDX/copyright headers.

These projects were used as references during development; they are not
redistributed here (the local `references/` working copies are not part of this
repository).

### Sources

- **Monado** and its PSVR2 reverse-engineering work
  (<https://gitlab.freedesktop.org/monado/monado>, `SPDX: BSL-1.0`). The
  definitive host-side reverse engineering — the source of the USB
  interface/endpoint map, the control/status/IMU record layouts, the IMU scale
  factors, the ep0 vendor-control semantics, and the 6DoF pose frame convention
  this driver adopts. BSL-1.0 is permissive and compatible with this GPL-2.0
  project.

- **Sony's PSVR2 OSS release** — Sony's published GPL-2.0 source for the
  headset-internal MediaTek SoC (`SPDX: GPL-2.0`). This is **device-side** code
  (it does not run on the host), but it corroborates the IMU record layout, the
  USB control-event enums, the panel EDID, and the display pipeline (dprx/dsc/dsi).

- **Community PSVR2 reverse-engineering notes** — USB descriptor dumps, the
  interface map, controller IDs and FOV parameters, and the amdgpu DSC/FEC EDID
  quirk (carried here as `patches/amdgpu-psvr2-dsc-fec.patch`).

- **OpenVR SDK** — Valve (<https://github.com/ValveSoftware/openvr>,
  `SPDX: BSD-3-Clause`). The SteamVR driver is built against its
  `openvr_driver.h` and modelled on the SDK's sample HMD driver. The needed
  files (`openvr_driver.h` and the sample `driverlog` helper, SDK v2.15.6) are
  **vendored** under `steamvr/driver_psvr2/third_party/openvr/` with the OpenVR
  `LICENSE` retained, so the driver builds from a clean checkout. See that
  directory's README.

- **VR on Linux wiki** — <https://wiki.vronlinux.org/docs/hardware/psvr2/>.

### Embedded font

`userspace/tools/font8x8.h` is the Linux kernel's 8×8 CP437 console font
(`lib/fonts/font_8x8.c`, GPL-2.0), used by the on-headset display helpers.
GPL-2.0, compatible with this project.

### Licensing

This project is **GPL-2.0** (see [COPYING](../COPYING)). Where wire-protocol
structures were ported from Monado (BSL-1.0), the original copyright lines are
preserved in the relevant file headers (e.g. `kernel/psvr2_protocol.h`).

### Development

This project was developed with the assistance of **Claude** (Anthropic), via
Claude Code.

### Not affiliated with Sony

"PlayStation" and "PSVR" are trademarks of Sony Interactive Entertainment Inc.
This is an independent project, not endorsed by or affiliated with Sony.
