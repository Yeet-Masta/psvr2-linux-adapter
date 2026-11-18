# Dev branch notes

This is the `dev` branch. It holds everything taken from the reverse-engineering
notes. `docs/RESEARCH.md` is the canonical, consolidated research; where it
and the older files in `docs/sony-re/` disagree, RESEARCH.md wins (its §12
lists the superseded statements). **None of it has been
tested on hardware yet.** Sony's driver was used only as a specification and a
source of raw data (constants and tables). No code was copied.

## Kernel module

| Change | Files | Source |
|--------|-------|--------|
| `/dev/psvr2-ctrl`: control reports with an allowlist. SET: 05 08 09 0a 0b 12 14 17 1f 22 25 27 28 29. GET: 81 8a 8f 97 a1 a4 a5 a6 a9 de | `psvr2_ctrl.c`, `psvr2_uapi.h` | sony-driver-re.md "Control reports" |
| Headset clock map (Get 0xde every 500 ms, RTT < 1000 µs, 100-entry ring, reset on a > 2 ms jump, drift clamped to ±50000 ppb). IMU and SLAM timestamps now use it | `psvr2_ctrl.c`, `psvr2_status.c`, `psvr2_slam.c` | headset.md §5.1 |
| Camera mode 0x10 stays on. It is re-sent once a second until camera and IMU calibration (VD types 10 and 12) arrive. V4L2 streaming switches to mode 1 and restores 0x10 when it stops | `psvr2_ctrl.c`, `psvr2_camera.c` | ledsync.md §2.7 |
| VD calibration container parsed. Sysfs files `camera_calib0`, `camera_calib1`, `imu_calib`, `imu_set_calib` | `psvr2_aux.c`, `psvr2_usb.c` | sony-driver-re.md "VD" |
| `/dev/psvr2-ld` (LED detection, 8-packet queue) and `/dev/psvr2-rp` (relocaliser, 2 packets). The magic is checked; each read returns a header followed by the payload | `psvr2_aux.c` | ledsync.md §1 |
| Set 0x14 default `{0x55, 500, 0x24800001, 0}` on connect | `psvr2_aux.c` | ledsync.md §2.5 |
| Aux endpoint errors are retried after 500 ms with a halt clear | `psvr2_aux.c` | |
| SLAM record: state, substate, flag, velocity, gyro/accel bias, variance, reset counter, map id. TRACKING flag set by Sony's rule `state == 3 \|\| flag == 1 \|\| substate == 5` | `psvr2_slam.c`, `psvr2_uapi.h` | tracking.md §2.1 |
| See-through button toggle, brightness follow, Get 0x97 polled once a second | `psvr2_ctrl.c` | |
| Resume: Set 0x05 subcmd 2 sent 3 s after resume | `psvr2_usb.c` | |
| `ready` sysfs attribute | `psvr2_usb.c` | |
| udev: the new nodes get group `input`; brightness, rumble, see_through and edid_type become group-writable | `99-psvr2.rules` | |

## libpsvr2

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
  RESEARCH.md §8), `psvr2_cam_project` (OpenCV frame),
  `psvr2_cam_project_rig` (Sony's frame, 180° about Z) and
  `psvr2_cam_extrinsic_to_rig` (R′ = G·R·G, t in mm). Checked against
  Sony's form to 1.6e-9 px.
- **New:** the controller LED pose solver, see the last section.

## SteamVR driver

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
    RESEARCH.md §7 (11/20-frame windows, BG marker frames, target 10,
    ±40 fit reset).
- Head pose: Sony's lever arm `vecDriverFromHeadTranslation = (0, 0.0208,
  0.09255)` m is applied (RESEARCH.md §5.2); the rotation part was already
  in the remap.

## 120 Hz frame timing (SteamVR bug workaround)

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

## Not done, and why

| Item | Why |
|------|-----|
| Controller **optical pose**: Sony's UKF | Wired without a UKF (`optical_pose.cpp`, 2026-11-18, not yet run on hardware): the optical position is used as is (extrapolated 0.3 s, then held, as Sony), and each fix pulls the IMU heading. See "Optical controller pose" below. |
| Blob association in LED sync | Sony associates blobs by predicted pose. Without a pose, the tracker syncs **one controller at a time** and `BlinkFinder` picks, per camera, the blob whose on/off pattern repeats every 11 (BROAD) or 20 (BG) frames; steady room lights and random flicker are rejected (simulated: 70 lights, 3 flickering lamps, 0 wrong picks). The first hardware run had 50–80 room blobs per camera, which the old "≤ 4 blobs" rule could never decode (stuck in BROAD). Hardware, 2026-11-18 (dark room): PRESCAN → BROAD → BG reached. A dark sync LED often still shows a dim blob (weight ~4 vs ~13 lit), so lit/dark is judged by weight against the site's own range; in BG the BROAD pick is kept while seen. First STABLE on hardware 2026-11-18 (17.7 s in BG, held 44 s). Two own changes made it lock: the drift fit slope is sent as a dimensionless rate (BG: −slope·step, BROAD: slope·2.5·step; raw it changed the cycle by 0.14 ppm against an 11 ppm drift), and every BG fit needs 10 samples (Sony: 40 after the first; BG gives ~1.3 samples/s). Open: PRESCAN → BROAD took 3.5–44 s with the controller in view. |
| Drift-fit units | Sent as a dimensionless rate, not Sony's raw slope; see the blob-association row. |
| Controller preset | Sony reads it from libpad's info block (+0x10). Here libpad's sync mode (1 / 2) stands in, since it has the same timing values. |
| Extreme-Env blob modes (2–4) | They need tracker settings that are 0 in a normal install. |
| Changing EDID ids | The current ones work. |
| Controller input remap to Sony's L1 behaviour | The project has its own input profile. |
| Relocaliser (Set 0x0a, `/dev/psvr2-rp` consumer) | Large and optional. The kernel side already exposes the data. |

## Testing

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

## Optical controller pose (steamvr/driver_psvr2/src/optical_pose.cpp)

From `docs/sony-re/opticalpose.md`. Runs per LD frame for each controller in
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
  used: whether its X frame is this rig frame is open (opticalpose.md §8),
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
- Open: one controller syncs at a time, so the second is only tracked
  after the first reached STABLE; boot costs ~5 ms, one hypothesis per
  frame (up to ~10 s to learn the remap); no UKF.

## Controller LED pose solver (userspace/lib/psvr2_led_solver.c)

Status: builds, in libpsvr2.a, `make check` passes. NOT wired into
OpticalTracker yet.

- Implements RESEARCH.md §9: cone visibility, 120 mm self-occlusion cull,
  face-on-first gated match with Sony's outlier cut, Huber-IRLS LM (Nielsen),
  MSAC score, orientation-prior 2-point boot over Sony's LED-pair table,
  Sony's < 7-inlier boot rule (15/30 deg to the IMU prior, < 500 mm).
- Own additions (not traced in Sony's code):
  - per-camera image-shift consensus before matching (LEDs are ~6 px apart
    at 0.5 m; plain nearest-blob cascades into off-by-one matches);
  - track acceptance: >= 4 matches, >= 3/4 of the LEDs facing a camera
    (< 70 deg) matched, rms <= 1.5 px. On -1 the caller should boot.
- LED models: `scripts/sony-re/extract_led_header.py <dll> userspace/lib/psvr2_led_models.h`
  generates the header (raw float bits from the DLL);
  `psvr2_led_model_load(rev, &model)` fills a model.
- `make check` (`test_led_solver.c`): Sony models rev 2..7, 4 cameras near
  the rig positions, 200 random poses each, 0.3 px noise, 2 clutter blobs
  per camera. Typical: track 177-186/200 right from a ~2 deg / 9 mm
  prediction, boot 191-199/200 from an IMU orientation ~5 deg off, 0-4
  wrong poses accepted per 200 (near-planar ring ambiguity; the IMU fusion
  must catch those).
- Open:
  - rev 1 (20 LEDs): boot only ~120/200 with 12 wrong; not understood.
  - hand: both hands get the same model; Sony's hand-1 descriptor byte
    0x58 is probably a mirror flag but is not traced. No mirroring applied.
  - The gravity-only boot path (3/6 deg) is not implemented.
- Needs hardware before wiring: rig camera poses, blob pixel scale vs 508.
