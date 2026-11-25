# SteamVR devices and display

How Sony's driver presents the headset and controllers to SteamVR, and its display path.

Parts:

- [Part A: SteamVR-facing behaviour](#part-a-steamvr-facing-behaviour)
- [Part B: Display and SteamVR devices](#part-b-display-and-steamvr-devices)

---

## Part A: SteamVR-facing behaviour

Behavioural spec of the parts of Sony's SteamVR driver
(`driver_playstation_vr2_orig.dll`, PE timestamp `1741940190`, image base
`0x180000000`) that face SteamVR: device properties, display timing, lens
geometry, input components, hand skeleton, play area. Written in our own words
from analysis of the binary; no Sony code or tables are reproduced. Function
names are Ghidra addresses in that build. Method and USB protocol: see
[overview.md part B](overview.md#part-b-driver-architecture-and-code-map).

Implementation: [`userspace/lib/sie/steamvr/`](../../userspace/lib/sie/steamvr/).
Differential tests: `scripts/sony-re/difftest/t_steamvr_*.c`. Status per
function: [`scripts/sony-re/status/steamvr.tsv`](../../scripts/sony-re/status/steamvr.tsv).

Confidence tags as in overview.md part B: **[sure]** read from code and/or proven by
differential test, **[likely]**, **[guess]**.

### Test harness (fake OpenVR host)

`t_steamvr_host.h` lets the tests call Sony's device methods under Wine:

- Sony caches OpenVR interface pointers in globals (`IVRDriverContext` at
  `0x180354cb8`; `IVRProperties` at `0x180354cc0/cc8/cd8`; `IVRSettings`
  `0x180354cd0`; `IVRDriverLog` `0x180354cf8`; `IVRDriverInput` `0x180354d10`).
  The harness fills them with fake objects that record every
  `WritePropertyBatch` entry (property, tag, size, bytes) and serve settings
  from a table.
- The USB manager singleton (`0x180354d40`, created by `FUN_180124c90`) is set
  non-null, and each manager getter the code under test reaches is replaced
  by a 12-byte `mov rax, imm64; jmp rax` patch to a stub that returns the test
  input. The getters themselves only copy a block out of the device state
  under a mutex.

### Lens geometry (`FUN_1801a1a00`) [sure]

Input: a 256-byte display-calibration block the headset reports (the manager
keeps it at device state `+0x4a2c`; `FUN_180124e30` copies it out). Sony keeps
a copy of the last block it saw (initially all zero) and does nothing when the
new block is byte-identical.

Otherwise the block replaces the copy and four values are derived, kept in the
HMD device at `+0x260..+0x27f`:

| Field | Meaning [likely] |
|-------|------------------|
| `centre_l` (x, y) | left lens centre, normalised |
| `centre_r` (x, y) | right lens centre, normalised |
| `rot_l` (cos, sin) | left panel rotation |
| `rot_r` (cos, sin) | right panel rotation |

Block fields used (all little-endian `float`, byte 0 is a version):

| Offset | Use |
|--------|-----|
| 0x00 | u8 version; below 4 means "no calibration" |
| 0x08, 0x0c | left offset x, y (panel pixels [likely]) |
| 0x10, 0x14 | right offset x, y |
| 0x18, 0x1c | left, right rotation, degrees |
| 0x20 | horizontal separation term, pixels [likely] |

With version < 4 the result is fixed: `centre_l = (-0.0992, 0)` (bits
`0xbdcb25aa`), `centre_r = (+0.0992, 0)`, both rotations `(0, 0)` (sic: zero,
not identity).

With version ≥ 4 every coordinate is a pixel value `p` turned into a
normalised lens coordinate as `((p·29.9 + 14.95) / 1000 + shift) / 32.462`
(single precision, in that order; 29.9 µm pixel pitch measured from the pixel
centre, `shift` in mm, 32.462 mm half-width; the divisor is the float with
bits `0x4201d914`). All block values are negated first:

- `centre_l.x` from `-x_l - sep`, shift `-3.22`
- `centre_r.x` from `-x_r + sep`, shift `+3.22`
- `centre_l.y`, `centre_r.y` from `-y_l`, `-y_r`, shift `+0.0` (the addition is
  kept: it turns `-0` into `+0`)
- each rotation angle is `-deg`, converted in double precision
  (`deg·π/180`, rounded to float), then `(cosf, sinf)`.

Implementation: `psvr2_steamvr_lens_update`.

### Hidden-area mesh (`FUN_1801a10c0`) [sure]

Builds the mesh SteamVR uses to skip pixels outside the lens
(`Prop_DisplayHiddenArea_Binary_Start` = 5100 for the left eye, 5101 right,
tag 30 = `k_unHiddenAreaPropertyTag`, written to the HMD container
`TrackedDeviceToPropertyContainer(0)`). Only the left-eye geometry
(`centre_l`, `rot_l`) is used; the right mesh is the left mesh mirrored.

1. 32 directions at angles `k·(2π/32)` (float step bits `0x3e490fdb`).
   Per direction, march a radius `r` from 0.9 in steps of 0.004:
   - lens point `y = sin·r·S + centre_l.y`, `x = cos·r·A·S + centre_l.x`,
     `S = 0.93950` (`0x3f7082fd`), `A = 50/51` (`0x3f7afafb`);
   - stop (keeping the previous output) when `y² + x² > 1.06`;
   - radial scale `k` from a 1024-entry table indexed by `t = (y²+x²)·1023`:
     linear interpolation between entries `floor(t)` and `floor(t)+1`
     (index clamped at 0); past the end, `(t − 1023)·0.0048625 + last`
     (`0x3b9f5600`, `0x3fd8b389`);
   - distorted `dy = (y − 0.00023027)·k`, `dx = (x + 0)·k`; rotate by `rot_l`
     and map to texture space:
     `u = (c·dx − dy·s)·0.35856 + 0.66038`, `v = (dy·c + s·dx)·0.37623 + 0.5`
     (`0x3eb794b5`, `0x3f290e95`, `0x3ec0a0f8`);
   - store `(2u − 1, 2v − 1)`; stop once `u` or `v` leaves `[0, 1]`.
   So the stored point is the first one off the panel.
2. Segment `k` (0..31) emits the triangle (ray k, ray k+1, corner of its
   quadrant `q = k/8`), corners at `±1.1`: q0 `(+,+)`, q1 `(−,+)`, q2 `(−,−)`,
   q3 `(+,−)`. The first segment of each quadrant also emits
   (ray k, previous quadrant's corner, this quadrant's corner). 36 triangles,
   108 vertices.
3. All vertices map from clip to texture space: `(p + 1)·0.5`.
4. Write left; then `x = 1 − x` for every vertex and write right.

If the geometry contains a NaN, `r²` is NaN, neither stop test fires and the
march never ends (Sony's code hangs the same way; the difftest feeds the mesh
only finite geometry). The output buffer is a static array; a ray whose very first step is already
outside `r² ≤ 1.06` leaves the old value in place (never happens with real
calibration). The 1024-entry table (at `0x180262180`) is a measured lens
profile, not a closed form (a degree-8 polynomial fit is still 4·10⁻³ off), so
this project does not carry it: `psvr2_steamvr_hidden_area` takes it as a
parameter, and the difftest passes Sony's table from the loaded DLL.

### HMD device object (`sie::psvr2::HmdDevice`)

Vtables (from RTTI): `ITrackedDeviceServerDriver` at `0x180264d60`,
`IVRDisplayComponent` (object `+8`) at `0x180264d98`. The device is allocated
by `ServerDriver::Init` (`FUN_1801a2040`) with 0x3d0 bytes; the constructor
sets the panel size `+0x258 = 4000`, `+0x25c = 2040` (both eyes side by side)
and the object id `+0x10 = -1`. `+0x18` points at the `ServerDriver`, whose
`+0x88` holds the refresh rate chosen at init.

| Method | VA | Behaviour |
|--------|----|-----------|
| Activate | `0x18019d1b0` | properties, see below |
| Deactivate | `0x18019ebf0` | delete update thread, id = −1, release the `IMMDeviceEnumerator` |
| EnterStandby | `0x18019ec50` | nothing |
| GetComponent | `0x18019ec60` | `IVRDisplayComponent_003` (case-insensitive) → object `+8`; anything else null. No direct-mode or camera component |
| DebugRequest | `0x18019ec40` | empty reply |
| GetPose | `0x18019ece0` | returns the `DriverPose_t` cached at `+0x28` (filled by the update thread) |

#### Display component [sure, difftest `t_steamvr_display`]

With `W, H` = panel size (4000 × 2040):

| Method | VA | Result |
|--------|----|--------|
| GetWindowBounds | `0x18019edf0` | `(0, 0, W, H)` |
| IsDisplayOnDesktop | `0x18019ee10` | false |
| IsDisplayRealDisplay | `0x18019ee20` | true |
| GetRecommendedRenderTargetSize | `0x18019edb0` | `((W/2)·17/10, H·17/10)` in unsigned integers = 3400 × 3468 (1.7× per eye) |
| GetEyeOutputViewport | `0x18019eca0` | left `(0, 0, W/2, H)`, right `(W/2, 0, W/2, H)` |
| GetProjectionRaw | `0x18019ed60` | left eye: l = −1.84177, r = +0.94719; right eye: l = −0.94719, r = +1.84177; both t = −1.32898, b = +1.32898 (bits `0xbfebbf26`, `0x3f727b16`, `0xbfaa1c0d`) |
| ComputeDistortion | `0x18019e860` | below |
| ComputeInverseDistortion | `0x18019ebe0` | false (not provided) |

**ComputeDistortion(eye, u, v)** uses the lens geometry of that eye
(`centre`, `rot`) and three 1024-entry radial tables, red / green / blue
(`0x180261180`, `0x180262180`, `0x180263180`; green is the hidden-area table):

1. `x = ((u+u) − 1)·A·S + centre.x`, `y = centre.y + ((v+v) − 1)·S`
   (`A`, `S` as in the hidden-area mesh).
2. `t = (y·y + x·x)·1023`, `i = max(0, trunc(floor(t)))`. If `i + 1 < 1024`,
   per channel `k = f·T[i+1] + (1−f)·T[i]` with `f = t − i`; else
   `k = (t − 1023)·slope + last`, slopes `0x3b6f1e00 / 0x3b9f5600 / 0x3bf35000`,
   ends `0x3fcbd9db / 0x3fd8b389 / 0x3febbf26` (R/G/B).
3. `x0 = x + 0`; red and green use `yc = (y − 0.00023027)·k`, blue uses
   `yc = (y + 0)·k` (no vertical offset for blue).
4. With `(c, s)` the eye rotation: `U = (c·(x0·k) − yc·s)·0.35856 + ub`,
   `V = ((x0·k)·s + c·yc)·0.37623 + 0.5`, `ub` = 0.66038 left, 0.33962 right
   (`0x3f290e95`, `0x3eade2d7`).

Bit-exact over 300 000 random calls; with NaN geometry only the sign of the
resulting NaNs can differ (operand order of commutative operations).

#### HMD properties (`Activate`, `0x18019d1b0`) [sure, difftest `t_steamvr_hmd`]

`Activate(id)` stores the id at `+0x10`, logs, creates an `IMMDeviceEnumerator`
(for the "PS VR2" audio endpoints), then writes these properties, one
`WritePropertyBatch` each, to `TrackedDeviceToPropertyContainer(id)`, in this
order:

| # | Property | Value |
|---|----------|-------|
| 1 | 1005 ManufacturerName | `"SIE"` |
| 2 | 2011 EdidVendorID | `0xd94d` |
| 3 | 2015 EdidProductID | config `edid product id` if non-zero, else `0xc207` (logged as `edid product id: 0x%04X`) |
| 4–6 | 2110, 2111, 2112 (reserved in public headers) | int32 17, 4, 160 |
| 7 | 2003 UserIpdMeters | SteamVR setting `steamvr/ipd` |
| 8 | 2080 DisplayAvailableFrameRates | float[2] `{90, 120}`, only when the headset software version (first dword of the manager's HMD info, `FUN_180124a00`) is above `0x06000000`; logged `HMD sw_version: 0x%08x (support 90hz & 120hz)` |
| 9 | 2002 DisplayFrequency | 90 for old firmware, else the rate chosen at init (`ServerDriver +0x88`) |
| 10 | 2001 SecondsFromVsyncToPhotons | see timing below |
| 11 | 2042 SecondsFromPhotonsToVblank | see timing below |
| 12 | 2100 IpdUIRangeMinMeters | 0.059 |
| 13 | 2101 IpdUIRangeMaxMeters | 0.072 |
| 14 | 2003 UserIpdMeters (again) | headset IPD dial: byte 5 of the status block (`FUN_1801251a0`) × 0.001; the byte is also kept at `+0x3c8` for change detection |
| 15 | 2075 HmdTrackingStyle | 3 (inside-out cameras) |
| 16 | 2007 IsOnDesktop | false |
| 17 | 1001 ModelNumber | `"PS VR2"` |
| 18 | 1025 ContainsProximitySensor | true; then boolean input `/proximity` (handle at `+0x20`) |
| 19 | 1037 InputProfilePath | `{playstation_vr2}/input/playstation_vr2_profile.json` |
| 20–27 | icons 5004, 5001, 5002, 5003, 5005, 5006, 5007, 5009 | `{playstation_vr2}/icons/headset_status_{ready, off, searching(.gif), searching_alert(.gif), ready_alert, error, standby, standby_alert}@2x.png` (5008 low-battery is not set) |
| 28 | 2048 DriverProvidedChaperonePath | `<Prop_UserConfigPath_String of the driver handle>\chaperone_info.vrchap`; if that read fails or is not a string, just `\chaperone_info.vrchap` |
| — | | registers the device with the map/chaperone manager (`FUN_18011fc50`), lens geometry (`FUN_1801a1a00`), hidden-area mesh 5100/5101 (`FUN_1801a10c0`), then starts `UpdateThread` (priority 2) |
| 29 | 2056 DistortionMeshResolution | 80 |

**Display timing.** The driver config has `vsync to photons` and
`photons to vblank` in µs (signed). If both are below 1 the built-in values
are used: at 90 Hz (`|rate − 90| < 0.1`) 1.36 ms and 10.3 ms, otherwise
(120 Hz, or a NaN rate) 1.02 ms and 7.73 ms. Otherwise both come from the
config, `(float)(µs · 1e-6)` in double precision, even if one of them is ≤ 0.

**Differences in this project's driver** (`steamvr/driver_psvr2/src`): see
the section "Project driver vs Sony" below.

### Refresh rate and EDID type (`FUN_1801a4390`) [sure, difftest `t_steamvr_edid`]

Called by `ServerDriver::Init` with a caller tag (logged) and an optional
"changed" flag out-parameter.

1. Read `steamvr/preferredRefreshRate`. If it is more than 0.1 away from
   both 90 and 120, use 90 and write 90 back to the setting. (A NaN setting is
   kept as is.) The rate is stored at `ServerDriver +0x88`.
2. Read the headset's EDID type (Get `0xa9`; stays 0 if the read fails).
   - type 1 (90 Hz only) and rate within 0.1 of 120: Set `0x29` = 0
     (90/120 Hz), log `setEdidTypeEDID_TYPE_90HZ_120HZ`.
   - type 0 (90/120) and rate within 0.1 of 90: Set `0x29` = 1 (90 only).
   - anything else: no change.
   After a change Sleep 1000 ms and report changed = 1 (the headset
   re-enumerates its display). So Sony runs 90 Hz with a 90-only EDID and
   120 Hz with the dual EDID.
3. Logs `PreferredRefreshRate`, then the direct-mode check: `direct_mode/enable`
   from IVRSettings and from `steamvr.vrsettings` (`FUN_1801a5310`, which reads
   `<user config path>\..\steamvr.vrsettings` with jsoncpp; −1 if absent),
   before and after.

Implementation: `psvr2_steamvr_edid_select` (pure; the caller does USB, the
sleep and settings).

#### HMD update (`FUN_18019ee30`, run by `UpdateThread`) [pose: sure, difftest `t_steamvr_hmdpose`; rest likely]

Returns at once while the object id is −1. Each pass:

1. **IPD.** Read the status block (`FUN_1801251a0`). If the IPD byte (+5) is
   non-zero, write `UserIpdMeters = byte·0.001` (every pass, not only on
   change); log `ipd: %d -> %d` when it differs from `+0x3c8`.
2. **Proximity.** `UpdateBooleanComponent(/proximity, status[1] != 0, 0)`.
3. **See-through / pause.** The dialog manager's pause flag
   (`FUN_180131af0`, set while the see-through/boundary dialog runs) is
   mirrored into three bools on the HMD: 2058
   `DriverRequestsApplicationPause`, 2057 `DriverIsDrawingControllers`, 2059
   `DriverRequestsReducedRendering`: all true on entering (flag at `+0x288`
   0→1), all false on leaving; logged as `See-through Mode: On/Off, …`.
4. **Pose.** `ready = FUN_1801253c0()` (headset stream up [likely]).
   - not ready: report the cached pose at `+0x140` with poseIsValid = 1 and
     deviceIsConnected = 0;
   - ready and not paused: tracker pose of device 0 at time `now`
     (same 0x50-byte record as the controllers). Position, orientation,
     velocity, angular velocity are copied (float → double) with **no** tip
     offset; `qDriverFromHeadRotation` (w, x, y, z) and
     `vecDriverFromHeadTranslation` come from the config manager
     (`FUN_180156470`: floats at `+0x6a0, +0x6a4..0x6ac` and
     `+0x694..0x69c`, the IMU-to-head calibration [likely]);
     result 200, or 1 if the "headset requires update" flag `+0x289` is set;
     acceleration fields 0; deviceIsConnected = the ready value (any non-zero
     byte, not forced to 1); the result is also cached at `+0x140`;
   - paused: deviceIsConnected = ready value, then, if the previous report
     (`+0x28`) had deviceIsConnected, a copy of it; velocity and angular
     velocity zero.
   `poseTimeOffset = (float)(t_sample − now)·0.001·0.001` (µs to s: the
   negative age of the sample; SteamVR extrapolates over it).
5. **Throttle when not worn.** With proximity on, every pass is "fresh":
   the pose is stored as reference (`+0x2a8`, time `+0x3c0` = sample time)
   and sent. With proximity off a pass is fresh only if connected / valid /
   result differ from the reference, or for some axis
   `ref.pos − pos ≥ 0.005` or `angvel ≥ 0.03` (**signed** tests, so motion
   towards +axis or negative angular rates do not count: Sony quirk). A
   non-fresh pass is still sent while less than 500 001 µs passed since the
   reference, and after that nothing is sent (the headset goes quiet when
   put down). The report always becomes the next "previous" (`+0x28`); its
   time offset is set only when it is sent.
6. **Once, when ready:** `TrackingFirmwareVersion` (1006) = `%08x` of the HMD
   software version; `HardwareRevision_String` (1007) = board stage by index
   (`-`, `-`, `EVA2`, `EVA3`, `EVT1`, `EVT1R`, `EVT2`, `DVT1`, `DVT2`, `PVT`,
   `MP`; ≥ 11 not set); if the firmware needs an update
   (`FUN_180125480`): `Firmware_ManualUpdate` (1015) = true,
   `Firmware_ManualUpdateURL` (1016) = `https://www.playstation.com/psvr2-pc-usage/`,
   `Firmware_UpdateAvailable` (1014) = true, and the update dialog. If the
   reported firmware is newer than the server's known one, set `+0x289`.
7. **Once a second:** find the Windows audio endpoints named `PS VR2`
   (render) and `PS VR2 Audio` (capture) through `IMMDeviceEnumerator` and,
   when found, write their endpoint ids to `Audio_DefaultPlaybackDeviceId`
   (2300) and `Audio_DefaultRecordingDeviceId` (2301); each is written once
   (flags `+0x298`, `+0x299`, re-armed when the headset becomes ready).
8. Publish the pose to shared memory (slot 0) for the PS VR2 app.

### Sense controller (`sie::psvr2::SenseDevice`)

`ITrackedDeviceServerDriver` vtable `0x180264de0`: Activate `0x1801a5760`,
then `0x1801a7a50`, `0x1801a7b10`, `0x1801a7b20`, `0x1801a7b00`,
`0x1801a7b70`. Side at `+0xc` (0 left, 1 right), object id at `+8`.

#### Activate [sure, difftest `t_steamvr_sense`]

Properties, in order (container = `TrackedDeviceToPropertyContainer(id)`):

| Property | Value |
|----------|-------|
| 2075 HmdTrackingStyle | 3 |
| 2004 CurrentUniverseId | uint64 2 |
| 3002 / 3003 / 3004 Axis0..2Type | 2 (joystick), 3 (trigger), 3 (trigger) |
| 3001 SupportedButtons | `0x0000000700000087`: System, ApplicationMenu, Grip, A, Axis0–2 |
| 1001 ModelNumber | `PlayStation VR2 Sense Left` / `… Right` |
| 1003 RenderModelName | `{playstation_vr2}playstation_vr2_sense_left` / `_right` |
| 3007 ControllerRoleHint | 1 left, 2 right |
| 1037 InputProfilePath | `{playstation_vr2}/input/playstation_vr2_sense_controller_profile.json` |
| 5004, 5001, 5002, 5003, 5005, 5006, 5007, 5008, 5009 | `{playstation_vr2}/icons/<left\|right>_controller_status_<ready, off, searching(.gif), searching_alert(.gif), ready_alert, error, standby, ready_low, standby_alert>@2x.png` |

No ManufacturerName, TrackingSystemName, serial or controller type is set
here.

Input components, in creation order, with the device slot (qword index)
that receives the handle:

| # | Left | Right | Kind | Slot |
|---|------|-------|------|------|
| 0 | `/output/haptic` | same | haptic | 0x36 |
| 1–2 | `/input/square/click`, `/touch` | `/input/cross/…` | bool | L 0x3b, 0x3c; R 0x37, 0x38 |
| 3–4 | `/input/triangle/…` | `/input/circle/…` | bool | L 0x3d, 0x3e; R 0x39, 0x3a |
| 5–6 | `/input/create/…` | `/input/options/…` | bool | L 0x49, 0x4a; R 0x47, 0x48 |
| 7 | `/input/l2/touch` | `/input/r2/touch` | bool | 0x40 |
| 8 | `/input/l2/value` | `/input/r2/value` | scalar absolute one-sided | 0x3f |
| 9 | `/input/l2_sensor/value` | `/input/r2_sensor/value` | scalar one-sided | 0x41 |
| 10 | `/input/l1/touch` | `/input/r1/touch` | bool | 0x43 |
| 11 | `/input/l1/value` | `/input/r1/value` | scalar one-sided | 0x42 |
| 12 | `/input/l1_sensor/value` | `/input/r1_sensor/value` | scalar one-sided | 0x44 |
| 13–14 | `/input/left_stick/click`, `/touch` | `/input/right_stick/…` | bool | 0x4b, 0x4c |
| 15–16 | `/input/left_stick/x`, `/y` | `/input/right_stick/…` | scalar absolute two-sided | 0x4d, 0x4e |
| 17–18 | `/input/left_ps/click`, `/touch` | `/input/right_ps/…` | bool | 0x45, 0x46 |
| 19 | `/input/skeleton/left` | `/input/skeleton/right` | skeleton `/skeleton/hand/<side>`, base pose `/pose/raw`, `VRSkeletalTracking_Partial`, no grip-limit transforms | 0x4f |

Before the skeleton, Activate creates the hand-skeleton helper (0x20 bytes,
`FUN_180128870(side == right)`) at slot 0x35; the skeleton component is only
created if that allocation succeeded. The last Sense device to activate
starts a shared update thread (`0x180363598`, priority 2).

#### Controller pose (`FUN_1801a7bf0`) [sure, difftest `t_steamvr_pose`]

Run per controller by the Sense update thread. Inputs: the controller link
state (`FUN_180118a10`: connected byte at `+0x40`, a "lost" byte at `+0x5c`),
four pause conditions (`FUN_180131af0` on the dialog manager,
`FUN_1801a47b0` on the server driver, `FUN_180125480` on the USB manager,
`FUN_1801193d0` per controller), and the tracker's pose for device 2 (left)
or 1 (right) at the current time (`FUN_180160230`, 0x50 bytes):

| Offset | Field |
|--------|-------|
| 0x00 | float[3] position, m |
| 0x0c | float[4] orientation (w, x, y, z) |
| 0x28 | float[3] linear velocity |
| 0x40 | float[3] angular velocity |
| 0x4c | u8 pose valid |

`DriverPose_t` built from it (all quaternions w, x, y, z):

- **Not connected** (`+0x40` = 0 or `+0x5c` ≠ 0): the pose is all zero
  (result 0 = `TrackingResult_Uninitialized`, not connected).
- **Paused** (any of the four conditions): `deviceIsConnected = 1`; if the
  previous report had `deviceIsConnected`, start from a copy of it, else
  zeros; velocity and angular velocity become 0 (acceleration fields are kept
  from the copy); the first condition also clears `poseIsValid`.
- **Tracking**: `poseTimeOffset = 0` (no prediction), world-from-driver and
  driver-from-head rotations identity, world-from-driver translation 0,
  acceleration and angular acceleration 0, result 200 (`Running_OK`),
  `poseIsValid` = tracker valid byte, `deviceIsConnected = 1`,
  `willDriftInYaw = shouldApplyHeadModel = 0`, and
  - `qRotation = q_tracker · q_tip`, `q_tip` = 39° about +x
    (w `0x1.e2a1e8p-1`, x `0x1.55d178p-2`);
  - offset `o = base + grip·0.001` with base (m) `(±0.01432, −0.007713,
    −0.10399)` and grip (mm) left `(9.3727, −12.2481, −6.0039)`, right
    `(−20.0727, −12.2481, −6.0039)`;
  - `vecPosition = (double)p + (double)(R(qRotation)·o)` (rotation through
    a normalised 3×3 matrix, float);
  - `vecDriverFromHeadTranslation = −grip·0.001` (the head frame is the tip
    frame without the grip offset);
  - velocity and angular velocity copied unchanged (no lever-arm correction,
    no rotation into another frame).

The report goes to `TrackedDevicePoseUpdated(id, pose, 0x118)` and is kept in
the device at `+0x18` as the next "previous". It is also published to the
shared-memory block for the PS VR2 app (slot from `FUN_18015a250`: valid,
position, rotation, velocities as floats, a sequence counter, timestamps).

Then, if a hand skeleton exists and the first pause condition is off,
`FUN_18012a980` computes the skeleton twice (with controller → motion range
0, without → 1) and sends both with `UpdateSkeletonComponent` (31 bones).

### Hand skeleton (`HandSkeletonSpace`) [bones: sure, difftest `t_steamvr_skel`; blend: likely]

The Sense device owns a `HandSkeleton` (`FUN_180128870`). It holds two
0x1510-byte skeletons for the same side: "without controller" (open hand) and
"with controller". Each frame, the controller pose update (`FUN_1801a7bf0`)
calls `FUN_18012a980` twice:
- flag 1: the open-hand skeleton goes to skeleton component slot 0x4f with
  motion range 1 (`WithoutController`);
- flag 0: the with-controller skeleton goes with motion range 0 (`WithController`).
Both writes contain 31 bones.

**Input.** A 0x78-byte record comes from the input manager (`psvr2_skel_input`).
Arrays are indexed [left, right], and skeleton side 0 reads the left entries.
- `+0` buttons (libpad bits)
- `+4/+5` stick x/y
- `+8/+9` trigger
- `+0xc` touch bits
- `+0x10/+0x11` grip proximity
- `+0x12/+0x13` trigger proximity
- `+0x40` valid

Bit numbers are given as (left / right):
- A: touch/button bit 15 / 14
- B: bit 12 / 13
- C: bit 1 / 2
- D: button bit 0 / 3
- L1: button bit 10 / 11
- bit 16 (both sides)

**Bone record.** 0x28 bytes: pos[3], q (w,x,y,z), scale[3]. The skeleton object
holds:
- `+0xc` bind pose
- `+0x4e4` output
- `+0x9bc` last output
- `+0xe94` offset pose
- `+0x1370` previous input
- `+0x13e8` blend flags [31]
- `+0x1408` frame time
- `+0x1410` blend start times [31]
- `+0x1508` stick-centred flag

**Bind pose** (`FUN_18012b6a0`). Table rows are {x, y, z, rx, ry, rz in degrees},
31 rows per side.
- pos = (−x, y, z).
- q = (R_y · R_x) · R_z. Each R is an axis rotation of `(float)((double)(deg/180)·π)`,
  using the half-angle sinf/cosf.
- Then y and z of q are negated.

**Euler helper** (`FUN_18016ec10(q, a, b, c)`, radians). The half angles are
computed in float, and cos/sin in double:
- w = ca·cc·cb + sa·sc·sb
- x = sa·cc·cb − ca·sc·sb
- y = ca·cc·sb + sa·sc·cb
- z = ca·sc·cb − sa·cc·sb

**Slerp** (`FUN_180170dc0`). It takes the shortest arc. It falls back to linear
when the dot product is above 0.9999. The weights are sinf(k·θ)·(1/sinθ), with
θ = atan2f(sinθ, dot).

**Per-frame update** (`FUN_18012a9a0`):
1. Copy the bind pose to the output.
2. If the input is valid:
   - Set the stick-centred flag (`FUN_180128f00`): ‖(x−128, y−128)‖ ≤ dz, where
     dz is a per-controller deadzone byte × √(a constant).
   - Run the thumb method. It resets the whole offset pose to identity.
   - Run the finger method.
   - Compose each bone: out.pos = bind.pos + off.pos and out.q = off.q · bind.q.
   - Run the blend (below).
3. Write the 31 output bones as `VRBoneTransform_t` (pos w = 1).
4. Store the input as the previous input and the output as the last output.

**Open-hand thumb** (`FUN_18012a510`):
- touch B: bone 2 = X 18°; bones 4 and 5 = Z 20°.
- Else touch A: bone 2 = X 5°, bone 3 = Z 30°, bones 4 and 5 = Z 52°.
- Else, if touch C is set or the stick is off centre:
  - bone 2 = X(6·sy+10°) · Y(−8·sx ∓ 5°);
  - bones 4 and 5 = Z(66 − 20·sy).
  Here sx = 2x/255 − 1 and sy = 1 − 2y/255.
- Finally, any of button A, B or C adds X 3° to bone 2.

**Open-hand fingers** (`FUN_1801296d0`):
- index = 0.7·trig_prox + 0.3·trigger.
- g = 0.2·L1 + 0.8·grip_prox.
- middle, ring and little = (1 − index·k)·g + index·k, with k = 0.6, 0.4 and 0.2.
- Each finger's bones 1–4 (7–10, 12–15, 17–20, 22–25) get Z(curl·{74, 72, 70, 70}°).
- The first bone of the index, middle and ring fingers also gets
  X(curl·s) · Y(curl·s) with s = −10, −7 and −5.

**With-controller thumb** (`FUN_180129c20`). There is one shape per branch, in this
priority: touch B, touch A, touch C or stick off centre, button D, button bit 16.
- Each shape sets bone 2 (and 3) positions in the mm range and Euler rotations
  for bones 2–4.
- The stick shape also adds rotations driven by sx and sy to bones 2–5.
- The constants are in `psvr2_skeleton.c`.

**With-controller fingers** (`FUN_1801293b0`):
- If trig_prox > 5/255:
  - index bones 7/8/9 = Euler (0, 4°, −7°), (0, 0, 71°), (0, 0, 25°);
  - if trigger > 1/255, each bone is slerped by the trigger value toward
    (0, −3°, −7°), (0, 0, 80°), (0, 0, 27°).
- If trig_prox crosses 5/255 compared with the previous frame, the blend for
  bones 7–9 restarts.
- If grip_prox > 5/255 and L1 is held, bones 12–15 get Z 1°.

**Blend** (`FUN_18012af70`) [likely]:
- When some touch/button bits or the stick-centred flag change, the affected
  bones get blend flags and start times. The thumb set is bones 2–4. Toggling
  L1 marks bones 11–25 and 28–30.
- A flagged bone younger than 50 ms is slerped from the last output toward
  the new pose, with t = elapsed / remaining.
- Older flags are cleared.

Project driver (`controller_device.cpp`) differs from Sony in four ways:
- It builds both motion ranges from one set of five curls, using a
  skeleton-simulation helper.
- It infers the curls from the capacitive sensors and smooths them by 0.35
  per frame.
- The without-controller range is the same curls × 1.2.
- It has no touch-bit thumb shapes, bind-pose table or 50 ms blend.

### Server driver frame (`ServerDriver::RunFrame`, `FUN_1801a2cb0`) [likely, spec only]

RunFrame does nothing until the HMD and both Sense devices have tracked-device ids
(none of them is still -1). When it runs:

1. It writes the frame start time (100 ns clock, `FUN_18016b110`) to the watchdog block
   at +0x20a8. Before each stage it writes a stage code to +0x20b8. The codes are
   0x1e1 (events), 0x212 (USB speed), 0x220 (map), 0x222 (USB manager), 0x224, 0x226,
   0x228 (share / chaperone), 0x22a (HMD debug page), 0x22c / 0x22e (left / right
   controller stats), 0x230 (force-crash check) and 0x243 (end). At the end it writes
   the end time to +0x20b0 and sets the stage back to 0. Another thread reads this
   block to find a stalled frame.
2. Events. It drains `IVRServerDriverHost::PollNextEvent` (vtable +0x28, size 0x40).
   Every event is copied into a frame-local list, which is later passed to the map,
   share and debug stages.
   - `VREvent_DesktopViewReady` (531) sets the flag at driver +0x78.
   - `VREvent_QuitAcknowledged` (703):
     - It opens the process `data.process.pid` and reads its base module name,
       converted to lower case. It logs `[QuitAcknowledged] Process Name: %s`.
     - If the name starts with `vrmonitor.exe` or `steam.exe`, it calls USB-manager
       `FUN_180125610`, which sends HMD command 0x27 (SteamVR is quitting).
     - In every case it calls share `FUN_180130800`.
     - Then it reads `direct_mode/enable`.
       - If the value equals the cached value at +0x8c, it re-runs the EDID choice.
       - Otherwise it logs `[DirectMode Check] setting: %d -> %d at %s`, caches the
         new value and re-runs the EDID choice. If the choice did not set an EDID type,
         it logs `[DirectMode Check][WA] reboot` and asks the USB manager for a
         restart (`FUN_180127fa0(…, 1)`).
3. USB speed. It reads the USB manager link state (`FUN_1801248c0`) and acts only when
   the state has changed since the last frame. It then reads the USB speed code
   (`FUN_180125290`). A code in 1..0x2ff means a USB2 link, and it logs
   `PlayStation VR2 HMD is connected via USB2 connection. Please reconnect to a USB3 port.`
4. Then the per-frame stages, in order:
   - map `FUN_180120290(events)`
   - USB manager `FUN_180127840(0)`
   - `FUN_18011d0c0`
   - `FUN_1801617c0(0)`
   - share `FUN_180132890(events)`
   - HMD debug page `FUN_1801a02b0(events)`
   - controller stats `FUN_1801a8680` for left, then right.
5. Force crash (debug). If `DAT_18036352c` is non-zero, it counts the seconds since the
   first such frame and logs `force crash wait %d/%d` once per second. When the count
   reaches the setting, it logs `force crash is triggered!` and writes to address 0
   on purpose.

Project driver: it has no watchdog block, no QuitAcknowledged process check, no HMD
command 0x27 on quit, no direct-mode re-check and no USB2 warning.

### Project driver vs Sony

Differences found in `steamvr/driver_psvr2/src` (not edited here):

| Item | Project driver | Sony |
|------|----------------|------|
| Recommended render target | 2000 × 2040 (native per eye) | 3400 × 3468 (1.7× per eye) |
| Projection (left eye) | `tan(61.5°)=1.84177` outer, `tan(43.5°)=0.94896` inner, `tan(53°)=1.32704` vertical | outer 1.84177, inner **0.94719**, vertical **1.32898** (fixed constants) |
| IsDisplayOnDesktop | `!direct_mode` | always false |
| Window position | configurable | always (0, 0) |
| ManufacturerName | `"Sony"` | `"SIE"` |
| EdidVendorID / ProductID | `0x4DD9` / `0xA205` | `0xD94D` / `0xC207` (config-overridable) |
| UserIpdMeters from dial | `mm / 1000.0f` | `(float)mm · 0.001f` (can differ by 1 ulp) |
| SecondsFromVsyncToPhotons | 0.011 | 1.36 ms at 90 Hz, 1.02 ms at 120 Hz; plus SecondsFromPhotonsToVblank 10.3 / 7.73 ms |
| DisplayAvailableFrameRates | not set (uses DisplaySupportsMultipleFramerates) | `{90, 120}` on firmware > 0x06000000 |
| IPD UI range, HmdTrackingStyle, DistortionMeshResolution, props 2110–2112 | not set | 0.059–0.072 m, 3, 80, 17/4/160 |
| InputProfilePath | `{psvr2}/input/psvr2_hmd_profile.json` | `{playstation_vr2}/input/playstation_vr2_profile.json` |
| DriverProvidedChaperonePath | not set | `<user config>\chaperone_info.vrchap` |
| Controller pose | arm model / IMU-only fallback, smoothed velocities, `poseTimeOffset = −prediction` | optical+IMU tracker pose; `poseTimeOffset = 0`; velocities passed through unsmoothed; tip rotation 39° about x, per-side grip offset and `vecDriverFromHeadTranslation` (see "Controller pose") |
| Controller pause/disconnect | — | paused: repeat last pose, zero velocities; disconnected: all-zero pose |
| HMD poseTimeOffset | `−clamp(age, 0, 0.05)` | `(float)(t_sample − t_send)` µs → s, unclamped |
| HMD driver-from-head | — | IMU-to-head calibration from the config manager (rotation and translation) |
| HMD not worn | always reports | throttled, silent after 500 ms still (see "HMD update") |

---

## Part B: Display and SteamVR devices

What Sony's SteamVR driver (`driver_playstation_vr2_orig.dll`, PE timestamp
`1741940190`, image base `0x180000000`) reports to SteamVR and how it drives
the display: EDID/refresh selection, timing, render target, lens model, IPD
and eye/head transforms, pose output, HMD and controller properties, input
components and haptics. Written from the decompile in our own words; addresses
are Ghidra VAs. Tags: **[sure]** read in code (or executed under Wine),
**[likely]**, **[guess]**.

Builds on [part A](#part-a-steamvr-facing-behaviour) (reimplementation pass, difftested) and
[headset.md part A](headset.md#part-a-headset-usb-behaviour) (USB control reports). This file restates what the
Linux driver needs, so it can be read alone, and adds what those notes lacked:
where the lens calibration comes from, the HMD driver-from-head transform
values, pose timing, the controller input-to-component map, battery
properties, haptic event handling, server init order and the driver INI.

### B.1 Driver summary

What the Linux stack must do, in priority order:

1. **Refresh / EDID type (userspace, needs kernel ioctl).** Before SteamVR
   leases the display, Get feature `0xa9` (1 byte: 0 = 90/120 Hz EDID,
   1 = 90 Hz-only EDID). If SteamVR wants 120 Hz and the EDID is 90-only, Set
   `0x29` = 0; if SteamVR wants 90 Hz and the EDID is 90/120, Set `0x29` = 1.
   Then wait 1000 ms; the headset re-enumerates its DisplayPort sink. The
   kernel module should expose Get `0xa9` / Set `0x29` (it has neither today).
2. **Lens calibration (kernel reads, userspace uses).** Get `0x8f` (256 bytes)
   at connect, only when the USB link is USB3 (descriptor `bcdUSB ≥ 0x300`).
   It feeds the lens centres and panel rotations (section 5.1), which drive
   `ComputeDistortion` and the hidden-area mesh. Version byte < 4 ⇒ fixed
   defaults.
3. **Display component values (userspace).** Panel 4000 × 2040, viewport per
   eye 2000 × 2040, recommended render target 3400 × 3468, fixed projection
   tangents, vsync-to-photons 1.36 ms / 1.02 ms and photons-to-vblank
   10.3 ms / 7.73 ms at 90 / 120 Hz.
4. **HMD pose (userspace).** Query the tracker at "now", report with
   `poseTimeOffset = (t_query − t_send)` (a few µs, negative), constant
   driver-from-head transform `q = (−0.5, −0.5, 0.5, 0.5)`,
   `t = (−0.09255, 0, 0.02080)` m. Poll every 1 ms while the headset is
   ready, 100 ms otherwise.
5. **IPD dial and proximity (kernel parses status, userspace reports).**
   Status-header byte 5 = IPD in mm → `UserIpdMeters`; byte 1 = proximity →
   boolean `/proximity`.
6. **Controller inputs, battery, haptics (userspace).** Map the per-controller
   input record to the 25 components (section 2.4); battery = level × 0.01;
   haptic events below amplitude 0.049 are ignored, durations are raised to
   at least 80 ms.

Kernel module: control reports `0x8f`, `0xa9`, `0x29`, status stream `0x88`
(IPD/proximity). Everything else (properties, lens model, pose packaging,
input mapping) is userspace in `steamvr/driver_psvr2`.

### B.2 Wire and data formats

#### B.2.1 Control reports used by this topic [sure]

| Report | Dir | Size | When | Meaning | VA |
|--------|-----|------|------|---------|----|
| `0xa9` | Get | 1 | Server init, cleanup, direct-mode change | EDID type: 0 = 90/120 Hz, 1 = 90 Hz only; treated as 0 if the read fails | `FUN_180124970` |
| `0x29` | Set | 1 | Only when the type must change | New EDID type, same encoding. Followed by `Sleep(1000)` | `FUN_180127e80`, `FUN_1801a4390` |
| `0x8f` | Get | 256 | Headset connect (`FUN_1801256c0`), after `0x81`, `0xa1` and the descriptor; skipped if `bcdUSB < 0x300` | Display/lens calibration block (2.2). Kept at manager `+0x4a2c` and copied into the config manager (`FUN_18015e990`, at `+0x70` of its shared block) | `FUN_1801256c0`, `FUN_180124e30` |
| `0xa1` | Get | 4 | Connect | u32 compared against the plugin version at server init: if greater, Sony shows the "update the PC app" dialog and fails Init (returns 200) | `FUN_1801256c0`, `FUN_180125080`, `FUN_1801a2040` |
| `0x81` | Get | 0x54 | Connect | Firmware info; dword +0 = HMD software version (`> 0x06000000` enables 90/120 Hz in SteamVR); +0x34 board stage | `FUN_180124450`, `FUN_180124a00` |
| `0x27` | Set | 2 | `VREvent_QuitAcknowledged` from `vrmonitor.exe`/`steam.exe` | u16 `1000` | `FUN_180125610` |
| `0xa5` | Get | 0x60 | Debug page only | DisplayPort receiver status (layout in overview.md part B) | `FUN_1801248d0`, `FUN_1801a02b0` |

#### B.2.2 Display calibration block (Get `0x8f`) [sure for used fields]

All little-endian. Only these fields are read by the SteamVR side
(`FUN_1801a1a00`):

| Offset | Type | Meaning |
|--------|------|---------|
| 0x00 | u8 | Version. < 4 ⇒ no calibration, use defaults |
| 0x08 | f32 | Left lens offset x, panel pixels [likely] |
| 0x0c | f32 | Left lens offset y |
| 0x10 | f32 | Right lens offset x |
| 0x14 | f32 | Right lens offset y |
| 0x18 | f32 | Left panel rotation, degrees |
| 0x1c | f32 | Right panel rotation, degrees |
| 0x20 | f32 | Horizontal separation term, pixels [likely] |

Sony re-reads the block from the manager each time Activate runs and skips the
recomputation when it is byte-identical to the last copy.

#### B.2.3 Status header bytes used here (`0x88`, first 32 bytes) [sure]

| Byte | Use |
|------|-----|
| 1 | Proximity (worn) → `/proximity` boolean, non-zero = true (`FUN_18019ee30`) |
| 5 | IPD dial in mm → `UserIpdMeters = byte · 0.001f`; 0 = not reported, leave the property alone |

#### B.2.4 Controller input record (`FUN_180118a10`, per side) [sure]

The controller manager hands the SteamVR device a record per controller. Field
offsets (relative to the record, from `FUN_1801a8680`):

| Offset | Type | Meaning |
|--------|------|---------|
| 0x00 | u32 | Button bits |
| 0x04 | u8 | Stick x (0..255, 128 centre) |
| 0x05 | u8 | Stick y (0..255, 0 = up) |
| 0x08 | u8 | Trigger (L2/R2) analog |
| 0x0c | u32 | Touch bits |
| 0x10 | u8 | L1/R1 capacitive sensor (grip proximity) |
| 0x12 | u8 | L2/R2 capacitive sensor (trigger proximity) |
| 0x40 | u8 | Connected |
| 0x5c | u8 | Lost (treated as disconnected for pose) |

Component update, per frame (`FUN_1801a8680`), when the controller is
connected and none of the four pause conditions holds (2.6):

| Component (left / right) | Left source | Right source | Value |
|--------------------------|-------------|--------------|-------|
| `square` / `cross` click | btn bit 15 | btn bit 14 | bool |
| `square` / `cross` touch | touch bit 15 | touch bit 14 | bool |
| `triangle` / `circle` click | btn bit 12 | btn bit 13 | bool |
| `triangle` / `circle` touch | touch bit 12 | touch bit 13 | bool |
| `l1` / `r1` value | btn bit 10 | btn bit 11 | scalar 1.0 or 0.0 (digital) |
| `l1` / `r1` touch | touch bit 10 | touch bit 11 | bool |
| `l1_sensor` / `r1_sensor` value | +0x10 | +0x10 | `byte / 255` |
| `l2` / `r2` value | +0x08 | +0x08 | `byte / 255` |
| `l2` / `r2` touch | touch bit 8 | touch bit 9 | bool |
| `l2_sensor` / `r2_sensor` value | +0x12 | +0x12 | `byte / 255` |
| `left_stick` / `right_stick` click | btn bit 1 | btn bit 2 | bool |
| `…_stick` touch | touch bit 1, **or** stick off centre | touch bit 2, or off centre | bool |
| `…_stick` x | +0x04 | +0x04 | `2·x/255 − 1`, or 0 when centred |
| `…_stick` y | +0x05 | +0x05 | `1 − 2·y/255`, or 0 when centred |
| `create` / `options` click | btn bit 0 | btn bit 3 | bool |
| `left_ps` / `right_ps` click | btn bit 16 | btn bit 16 | bool |
| `ps` touch, `create`/`options` touch | — | — | created but **never updated** (stay false) |

"Centred": `‖(x − c, y − c)‖ ≤ r`, with centre `c` and radius `r` from the
controller manager (`FUN_180118bf0`, per-controller deadzone) [sure]; the
test is only made when the record's connected byte is set. When paused or
disconnected every boolean is sent false and every scalar 0.

Bits are "libpad" bits (see `controllers.md` part A); left and right use different bit
numbers for the face buttons because the record is the shared DualSense-like
layout.

#### B.2.5 Haptic event (`VREvent_Input_HapticVibration`, 1700) [sure]

Handled in the controller frame (`FUN_1801a8680`) when the event's component
handle equals the controller's `/output/haptic` handle; passed to
`FUN_18011b4d0(side, duration_s, frequency_hz, amplitude)`:

```c
if (frequency <= 0 || amplitude <= 0) {           // stop request
    if (now_us - start_us < duration_s * 1e6) stop_flag = 1;
} else if ((double)amplitude >= 0.049) {
    duration_s = max(duration_s, 0.08f);
    store(duration_s, frequency, amplitude); stop_flag = 0;
    mode = (mode != 0) + 1;                       // 1 first pulse, 2 after
    start_us = now_us;
}                                                  // amplitude < 0.049: ignored
```

Rendering into the controller's vibration output report belongs to libpad
(`controllers.md` part A).

#### B.2.6 Controller pause conditions [sure]

Inputs, pose and skeleton are frozen/zeroed while any of these holds:
1. dialog manager pause (`FUN_180131af0`: see-through/boundary dialog);
2. server-driver flag `FUN_1801a47b0`;
3. headset firmware update needed (`FUN_180125480`);
4. this controller's firmware update needed (`FUN_1801193d0`).

### B.3 Sequences

#### B.3.1 Server init (`ServerDriver::Init`, `FUN_1801a2040`) [sure unless tagged]

1. Get interfaces, log `Activating PlayStation VR2...`.
2. Start config manager, map/chaperone manager, "hmd module" (USB manager,
   `FUN_180125330`), "sense module" (controllers, `FUN_180119310`), tracker
   (`FUN_18018a7e0(2)`), relocaliser and dialogs.
3. **EDID choice** (`FUN_1801a4390`, 3.2), tag `ServerDriver::Init`.
4. **Wait for the headset**: loop until "HMD ready" (`FUN_1801253c0`: board
   stage neither 0 nor `0xffffffff`). Sleep 1000 ms per pass, **no timeout**.
   From the third pass on, log USB2 (`speed code 1..0x2ff`) or "not found".
5. Read driver INI (`FUN_1801a47c0`, section 6.2); log versions.
6. Version gate: Get `0xa1` value (cached) > plugin version ⇒ update dialog,
   Cleanup, return 200 (init failed).
7. Cache `direct_mode/enable` at `+0x8c`.
8. Create HMD device (0x3d0 bytes; panel 4000 × 2040) and add it as
   `playstation_vr2_headset`, class HMD; create and add
   `playstation_vr2_sense_controller_left` / `_right`, class controller.
9. Start the watchdog thread, log `PlayStation VR2 Loaded Successfully`.

#### B.3.2 EDID / refresh selection (`FUN_1801a4390`) [sure]

1. `rate = steamvr/preferredRefreshRate`. If `|rate−90| > 0.1` and
   `|rate−120| > 0.1`: rate = 90 and write 90 back. Store at
   `ServerDriver+0x88`.
2. `type = Get 0xa9` (0 on failure).
3. If `type == 1` and `|rate−120| ≤ 0.1`: Set `0x29` = 0, Sleep 1000 ms,
   changed = 1.
   Else if `type == 0` and `|rate−90| ≤ 0.1`: Set `0x29` = 1, Sleep 1000 ms,
   changed = 1.
4. Log rate and type; compare `direct_mode/enable` from IVRSettings with the
   value in `steamvr.vrsettings` read by file (`FUN_1801a5310`), log both.

Callers: Init, Cleanup (`FUN_1801a2a40`, so the EDID matches the setting for
the next session), and RunFrame on `VREvent_QuitAcknowledged` (3.4).

#### B.3.3 HMD Activate (`FUN_18019d1b0`) [sure]

Properties are written in the order of table 6.3; then: register with the
chaperone manager, lens geometry from the `0x8f` block (5.1), hidden-area
meshes (5.3), start the HMD update thread, finally
`DistortionMeshResolution = 80`.

#### B.3.4 RunFrame display-relevant steps (`FUN_1801a2cb0`) [sure]

- `VREvent_QuitAcknowledged` (703): if the quitting process is
  `vrmonitor.exe` or `steam.exe`, Set `0x27` u16 1000. Then re-read
  `direct_mode/enable`; re-run 3.2. If the setting changed and 3.2 did not
  switch the EDID, request a headset restart (`FUN_180127fa0(…, 1)`, Set
  `0x05`), logged `[DirectMode Check][WA] reboot`.
- Link state change: if the USB speed code is `1..0x2ff`, log the USB2
  warning.

#### B.3.5 HMD update pass (`FUN_18019ee30`, thread `FUN_1801a19a0`) [sure]

Loop period: 1 ms while the headset is ready, 100 ms otherwise.

1. IPD: if status byte 5 ≠ 0, write `UserIpdMeters = byte·0.001` (every pass).
2. `/proximity` = status byte 1 ≠ 0.
3. See-through dialog pause flag edge: set 2058
   `DriverRequestsApplicationPause`, 2057 `DriverIsDrawingControllers`, 2059
   `DriverRequestsReducedRendering` all true on entry, all false on exit.
4. `t_query = now_us()`; pose (section 5.4).
5. Not-worn throttle (5.5), submit with
   `TrackedDevicePoseUpdated(id, pose, 0x118)`.
6. Once when ready: 1006 `TrackingFirmwareVersion` (`%08x` of software
   version), 1007 `HardwareRevision_String` (board stage name), firmware
   update properties 1014/1015/1016 if needed.
7. Once a second: Windows audio endpoint ids (2300/2301). Linux: set to the
   PipeWire/ALSA node names if SteamVR on Linux uses them [guess].

#### B.3.6 Controller frame (`FUN_1801a8680`, per side, from RunFrame) [sure]

1. Events: haptic (2.5); `VREvent_SystemReport_Started` (1900) builds a text
   dump of controller link histograms into a string property (debug only).
2. Input components (2.4).
3. Battery properties: disconnected ⇒ 1026 `DeviceProvidesBatteryStatus` =
   false, 1012 `DeviceBatteryPercentage` = 0. Connected ⇒ 1026 = true,
   1012 = `clamp(level·0.01, 0, 1)`, 1011 `DeviceIsCharging` = status ∈ {1, 2}
   (`FUN_180118870` returns status and level byte).
4. On a change of the connected state (flag `+0x171`): 1006
   `TrackingFirmwareVersion` = `%04x` of controller firmware version
   (`FUN_180118900`), or `Unknown` if < 1; 1007 `HardwareRevision_String` =
   controller hardware string (`FUN_180118b10`) if any.
5. Once per connection, if this controller needs a firmware update:
   1015 = true, 1016 = `https://www.playstation.com/psvr2-pc-usage/`,
   1014 = true, and the update dialog.
6. Read `power/turnOffControllersTimeout` (SteamVR setting, int, default 0)
   and pass it to the controller manager (`FUN_18011d570`), which uses it as
   the controller auto-off time [likely].
7. Once a second: if the controller is disconnected, or idle more than 60 s
   (`FUN_180118b70` = µs since last input > 60 000 000), pulse
   `l1/touch` true then false [guess: keeps SteamVR from deciding the
   controller is idle, or forces an input-state refresh].

Controller pose and skeleton are sent from a separate thread (`FUN_1801aa660`:
1 ms loop while either controller is connected, else 100 ms), see 5.6.

### B.4 State machines

#### B.4.1 EDID type

| State (Get `0xa9`) | SteamVR rate | Action |
|--------------------|-------------|--------|
| 1 (90 only) | 120 | Set `0x29`=0, wait 1 s ⇒ state 0 |
| 0 (90/120) | 90 | Set `0x29`=1, wait 1 s ⇒ state 1 |
| any | other | none |

So Sony never leaves a 120-capable EDID visible while running 90 Hz.

#### B.4.2 HMD pose report

| State | Condition | Report |
|-------|-----------|--------|
| Not ready | `FUN_1801253c0` false | cached last pose, `poseIsValid=1`, `deviceIsConnected=0` |
| Paused | see-through dialog | previous report (if it was connected), velocities zeroed |
| Tracking | ready, not paused | tracker pose, result 200 (1 if headset firmware is newer than the server knows, flag `+0x289`) |

Plus the not-worn throttle (5.5).

#### B.4.3 Controller pose report

Disconnected (connected byte 0 or lost byte ≠ 0) ⇒ all-zero pose, result 0.
Paused (2.6) ⇒ previous pose, velocities 0; condition 1 also clears
`poseIsValid`. Tracking ⇒ tracker pose with tip transform (5.6).

### B.5 Algorithms

#### B.5.1 Lens geometry from the `0x8f` block (`FUN_1801a1a00`) [sure, difftested]

```c
struct lens { float cx, cy; float cos_r, sin_r; } L, R;

if (blk.version < 4) {
    L = (lens){ -0.0992f, 0, 0, 0 };   // 0xbdcb25aa; rotation (0,0), sic
    R = (lens){ +0.0992f, 0, 0, 0 };
} else {
    // pixel -> normalised lens coordinate; 29.9 um pitch, 32.462 mm half width
    norm(p, shift_mm) = ((p*29.9f + 14.95f) / 1000.0f + shift_mm) / 32.462f;
    L.cx = norm(-blk.lx - blk.sep, -3.22f);
    R.cx = norm(-blk.rx + blk.sep, +3.22f);
    L.cy = norm(-blk.ly, 0.0f);
    R.cy = norm(-blk.ry, 0.0f);
    a = (float)(-blk.lrot_deg * M_PI / 180.0);  L.cos_r = cosf(a); L.sin_r = sinf(a);
    a = (float)(-blk.rrot_deg * M_PI / 180.0);  R.cos_r = cosf(a); R.sin_r = sinf(a);
}
```

All single precision except the degree-to-radian step (double). Divisor
32.462 is the float `0x4201d914`.

#### B.5.2 Distortion (`ComputeDistortion`, `FUN_18019e860`) [sure, difftested]

Three measured radial tables, 1024 floats each, red/green/blue at
`0x180261180`, `0x180262180`, `0x180263180` (not closed-form; a degree-8 fit
is still 4·10⁻³ off). Per eye `e` with lens `G = L or R`:

```c
const float S = 0.93950f;   // 0x3f7082fd, lens radius scale
const float A = 50.0f/51;   // 0x3f7afafb, aspect (2000 x 2040 panel)
x = ((u+u) - 1) * A * S + G.cx;
y = G.cy + ((v+v) - 1) * S;
t = (y*y + x*x) * 1023;
i = max(0, (int)floorf(t));
for ch in R,G,B:
    if (i + 1 < 1024) k[ch] = f*T[ch][i+1] + (1-f)*T[ch][i];   // f = t - i
    else              k[ch] = (t - 1023) * slope[ch] + end[ch];
// slope R/G/B bits 0x3b6f1e00 0x3b9f5600 0x3bf35000
// end   R/G/B bits 0x3fcbd9db 0x3fd8b389 0x3febbf26
for ch:
    dy = (ch == B) ? (y + 0) * k : (y - 0.00023027f) * k;
    dx = (x + 0) * k;
    U = (G.cos_r*dx - dy*G.sin_r) * 0.35856f + ub[e];   // ub: 0.66038 left, 0.33962 right
    V = (dx*G.sin_r + G.cos_r*dy) * 0.37623f + 0.5f;
```

Constants: `0.35856 = 0x3eb794b5`, `0.37623 = 0x3ec0a0f8`,
`0.66038 = 0x3f290e95`, `0.33962 = 0x3eade2d7`. `ComputeInverseDistortion`
returns false. `DistortionMeshResolution = 80`.

The project's `psvr2_distort` already implements this and is bit-exact.

#### B.5.3 Hidden-area mesh (`FUN_1801a10c0`) [sure, difftested]

32 rays marched from r = 0.9 in steps of 0.004 through the green table until
`x²+y² > 1.06` or the distorted point leaves the panel; 36 triangles (108
vertices) per eye, left computed from `L`, right = left mirrored
(`x = 1 − x`). Written as 5100/5101 with tag 30. Full detail in
part A "Hidden-area mesh"; the implementation there is exact.

#### B.5.4 HMD pose (`FUN_18019ee30`, `FUN_180160230`) [sure unless tagged]

```c
t_query = now_us();                     // QPC -> microseconds (FUN_18016b110)
trk = tracker_pose(device 0, t_query);  // INS filter evaluated at t_query
p.vecPosition       = (double)trk.pos;        // world (tracking map) frame, m
p.qRotation         = (double)trk.q;          // (w,x,y,z), world-from-IMU-rig
p.vecVelocity       = (double)trk.vel;        // world frame
p.vecAngularVelocity= (double)trk.angvel;     // rotated into world frame by tracker
p.vecAcceleration = p.vecAngularAcceleration = 0;
p.qWorldFromDriverRotation = (1,0,0,0); p.vecWorldFromDriverTranslation = 0;
p.qDriverFromHeadRotation    = (-0.5, -0.5, 0.5, 0.5);        // (w,x,y,z)
p.vecDriverFromHeadTranslation = (-0.09255, 8e-9, 0.02080);   // m
p.result = headset_fw_newer ? 1 : 200;
p.poseIsValid = 1;  p.deviceIsConnected = ready;
...
t_send = now_us();
p.poseTimeOffset = (float)(t_query - t_send) * 0.001f * 0.001f;   // s, <= 0
```

Prediction: the tracker itself evaluates the filter at `t_query` (it may
extrapolate from the newest IMU sample to that time). SteamVR then predicts
forward from `t_send` minus the tiny offset. There is **no** extra latency
term; the IMU-to-now gap is handled inside the tracker [likely].

The driver-from-head transform is fixed rig geometry, built by the config
manager's defaults (`FUN_180155000` → `FUN_1801556b0`) from: IMU-rig frame at
`(0, −0.02774, −0.07843)` m rotated by Rx(−95°)·Rz(90°), and head frame at
`Rx(−5°)·(0, 0, 0.012)` rotated Rx(−5°). The numbers above were read by
running Sony's initialiser under Wine [sure]. Camera calibration
(`FUN_180156500`) re-runs `FUN_1801556b0` but does not change those inputs,
so the result stays constant [likely]. Quaternion `(−0.5,−0.5,0.5,0.5)` is a
120° axis-permuting rotation; `(0.5,0.5,−0.5,−0.5)` is the same rotation.

#### B.5.5 HMD not-worn throttle [sure, difftested]

With proximity true, every pass is sent and becomes the reference (time
`+0x3c0` = `t_query`). With proximity false a pass counts as "fresh" if
connected/valid/result differ from the reference, or for some axis
`ref.pos − pos ≥ 0.005` m or `angvel ≥ 0.03` rad/s (signed tests: Sony
quirk). A non-fresh pass is still sent while `t_query − ref_time < 500 001`
µs; after that nothing is sent until motion.

#### B.5.6 Controller pose (`FUN_1801a7bf0`) [sure, difftested]

Tracker device 2 = left, 1 = right, queried at `now_us()`.
`poseTimeOffset = 0`. `qRotation = q_trk · q_tip`, `q_tip` = 39° about +x.
`vecPosition = p + R(qRotation)·o`, `o = base + grip·0.001`, base
`(±0.01432, −0.007713, −0.10399)` m (x sign per side), grip mm left `(9.3727, −12.2481, −6.0039)`, right
`(−20.0727, −12.2481, −6.0039)`. `vecDriverFromHeadTranslation = −grip·0.001`,
`qDriverFromHeadRotation` identity. Velocities passed through unchanged,
result 200, `poseIsValid` = tracker valid byte.

#### B.5.7 Hand skeleton (summary) [sure for bones]

Two 31-bone skeletons per hand each frame: with-controller (motion range 0)
and without-controller (range 1). Index curl = `0.7·trigger_prox +
0.3·trigger`; middle/ring/little from `g = 0.2·L1 + 0.8·grip_prox`
blended with index by 0.6/0.4/0.2; thumb shapes from touch bits and the
stick; 50 ms slerp blends on touch changes. Full constants in
part A "Hand skeleton".

### B.6 Constants

#### B.6.1 Display

| Name | Value | Meaning | VA |
|------|-------|---------|----|
| Panel | 4000 × 2040 | both eyes side by side | `FUN_1801a2040` |
| Viewport | left (0,0,2000,2040), right (2000,0,2000,2040) | | `FUN_18019eca0` |
| Render target | 3400 × 3468 | `(W/2)·17/10`, `H·17/10` | `FUN_18019edb0` |
| Projection left | l −1.84177, r +0.94719, t −1.32898, b +1.32898 | tangents; right eye mirrored | `FUN_18019ed60` |
| Window | (0,0,4000,2040), not on desktop, real display | | `FUN_18019edf0` |
| Vsync→photons | 1.36 ms (90 Hz), 1.02 ms (120 Hz) | INI override in µs | `FUN_18019d1b0` |
| Photons→vblank | 10.3 ms (90), 7.73 ms (120) | INI override in µs | `FUN_18019d1b0` |
| 90/120 threshold | `|rate−90| < 0.1` | picks the timing pair | `FUN_18019d1b0` |
| Refresh tolerance | 0.1 Hz | rate snapping | `FUN_1801a4390` |
| EDID settle | 1000 ms | after Set `0x29` | `FUN_1801a4390` |
| 90/120 firmware gate | software version > `0x06000000` | else only 90 Hz | `FUN_18019d1b0` |
| IPD UI range | 0.059–0.072 m | | `FUN_18019d1b0` |
| IPD scale | `byte · 0.001f` | mm → m | `FUN_18019ee30` |
| HMD loop | 1 ms ready / 100 ms not | | `FUN_1801a19a0` |
| Controller loop | 1 ms any connected / 100 ms | | `FUN_1801aa660` |
| Not-worn hold | 500 001 µs | | `FUN_18019ee30` |
| Driver-from-head | q (−0.5,−0.5,0.5,0.5), t (−0.09255, 0, 0.02080) m | | `FUN_1801556b0` |
| Haptic min amplitude | 0.049 | below: ignored | `FUN_18011b4d0` |
| Haptic min duration | 0.08 s | | `FUN_18011b4d0` |
| Battery scale | 0.01 | level byte → 0..1 | `FUN_1801a8680` |
| Stick scale | 255 | `2·b/255 − 1` | `FUN_1801a8680` |
| Idle pulse | 60 s, checked each 1 s | `l1/touch` pulse | `FUN_1801a8680` |

#### B.6.2 Driver INI (`FUN_1801a47c0`) [sure]

Read with `GetPrivateProfile*` from `major_version.txt` / `version.txt` in
the driver folder. Behaviour-changing keys:

| Section / key | Default | Effect |
|---------------|---------|--------|
| `version/plugin version` | `0x04110006` | compared with Get `0xa1` (init gate) |
| `version/jenkins` | 0 | patches the low BCD digits of the plugin version |
| `debug/no seethrough` | 0 | disables see-through (USB manager flag, `FUN_180127f50`) |
| `debug/sleep action` | 2 | action when the headset sleeps [guess] |
| `debug/sleep action wait` | 0 | delay for it |
| `debug/edid product id` | 0 | overrides `EdidProductID` (default `0xc207`) |
| `debug/vsync to photons` | 0 | µs; used only if either timing key ≥ 1 |
| `debug/photons to vblank` | 0 | µs |
| `debug/force crash sec` | 0 | debug crash |

SteamVR settings read: `steamvr/preferredRefreshRate`, `steamvr/ipd`,
`direct_mode/enable`, `power/turnOffControllersTimeout`.

#### B.6.3 HMD properties (Activate order) [sure]

| Property | Value | Source |
|----------|-------|--------|
| 1005 ManufacturerName | `SIE` | const |
| 2011 EdidVendorID | `0xd94d` | const |
| 2015 EdidProductID | `0xc207` or INI | const/INI |
| 2110 / 2111 / 2112 (private) | 17 / 4 / 160 | const [guess: SteamVR-internal display hints] |
| 2003 UserIpdMeters | `steamvr/ipd`, then dial | setting, status byte 5 |
| 2080 DisplayAvailableFrameRates | `{90, 120}` if fw > 0x06000000 | `0x81` +0 |
| 2002 DisplayFrequency | 90 (old fw) or chosen rate | 3.2 |
| 2001 SecondsFromVsyncToPhotons | 6.1 | const/INI |
| 2042 SecondsFromPhotonsToVblank | 6.1 | const/INI |
| 2100 / 2101 IpdUIRange | 0.059 / 0.072 | const |
| 2075 HmdTrackingStyle | 3 | const |
| 2007 IsOnDesktop | false | const |
| 1001 ModelNumber | `PS VR2` | const |
| 1025 ContainsProximitySensor | true (+ `/proximity`) | const |
| 1037 InputProfilePath | `{playstation_vr2}/input/playstation_vr2_profile.json` | const |
| 5001–5007, 5009 icons | `headset_status_*` | const |
| 2048 DriverProvidedChaperonePath | `<UserConfigPath>\chaperone_info.vrchap` | |
| 5100 / 5101 hidden area | 5.3 | `0x8f` |
| 2056 DistortionMeshResolution | 80 | const |
| later: 1006, 1007, 1014–1016, 2057–2059, 2300/2301 | 3.5 | |

No serial number, `TrackingSystemName`, `DisplayMCType` or camera
properties are set by Sony.

#### B.6.4 Controller properties [sure]

Activate (`FUN_1801a5760`): 2075 HmdTrackingStyle 3, 2004 CurrentUniverseId
2, 3002/3003/3004 axis types 2/3/3, 3001 SupportedButtons
`0x0000000700000087`, 1001 `PlayStation VR2 Sense Left/Right`, 1003
`{playstation_vr2}playstation_vr2_sense_left/_right`, 3007 role 1/2, 1037
`{playstation_vr2}/input/playstation_vr2_sense_controller_profile.json`,
icons 5001–5009. Runtime: 1026, 1012, 1011, 1006, 1007, 1014–1016 (3.6).

Components created (order): `/output/haptic`; face buttons click+touch;
create/options click+touch; `l2/touch`, `l2/value`, `l2_sensor/value`,
`l1/touch`, `l1/value`, `l1_sensor/value`; stick click/touch/x/y;
`*_ps/click`, `*_ps/touch`; skeleton `/input/skeleton/<side>`
(`/skeleton/hand/<side>`, base `/pose/raw`, partial tracking).

### B.7 Gaps in the current Linux driver

Compared with `kernel/`, `userspace/` and `steamvr/driver_psvr2/src`
(not edited):

| Item | Linux now | Sony |
|------|-----------|------|
| EDID type switch | none: no Get `0xa9` / Set `0x29` anywhere; relies on `DisplaySupportsMultipleFramerates` | switches EDID to match the SteamVR rate, waits 1 s |
| `0x8f` lens block | kernel reads 136 bytes as "Monado layout"; `psvr2_distort` uses it | 256 bytes, fields 2.2; read only on USB3 |
| Render target | `window_width/2 × window_height` (2000 × 2040) | 3400 × 3468 |
| Projection | `tan()` of configurable degrees (inner 0.94896, vert 1.32704) | fixed 0.94719 / 1.32898 / 1.84177 |
| Vsync→photons | 0.011 s; no PhotonsToVblank | 1.36/10.3 ms or 1.02/7.73 ms |
| Frame rates | `DisplaySupportsMultipleFramerates` | `DisplayAvailableFrameRates {90,120}` |
| EDID ids | `0x4DD9` / `0xA205` | `0xd94d` / `0xc207` (endianness of the vendor word differs; confirm which SteamVR-Linux matches on) |
| HMD driver-from-head | identity | q (−0.5,−0.5,0.5,0.5), t (−0.0926, 0, 0.0208) |
| HMD poseTimeOffset | `−clamp(age, 0, 0.05)` | `t_query − t_send` (≈ 0), tracker predicts to now |
| HMD loop | blocks on pose node, 10 ms back-off | 1 ms / 100 ms |
| Not-worn throttle | none | 5.5 |
| IPD | `mm / 1000.0f` | `(float)mm · 0.001f` (1 ulp) |
| IPD UI range, TrackingStyle, DistortionMeshResolution, 2110–2112 | not set | set |
| Hidden-area mesh | not set (no `HiddenArea` in the driver) | 5100/5101 from the lens geometry |
| Controller inputs | own mapping | 2.4 (L1 value is digital; stick 0 inside deadzone; touch on PS/create/options never sent) |
| Haptic thresholds | clamps amplitude only | ignore < 0.049, min 80 ms, stop events |
| Battery | `DeviceProvidesBatteryStatus` true always | false + 0 % while disconnected |
| Controller fw/hw strings | — | 1006 `%04x`, 1007 |
| Quit command | none | Set `0x27` 1000 on SteamVR quit |

### B.8 Open questions

- `0xa9`/`0x29` on Linux: does the DP sink really re-enumerate (hotplug) and
  does amdgpu/i915 pick up the new EDID? Needs a capture: Get `0xa9`, Set
  `0x29` 0 → 1, read EDID from `/sys/class/drm/*/edid` before and after.
- `0x8f` bytes beyond 0x24: other users (config manager shares it with the
  PS VR2 app). Capture a block and compare against the 136-byte Monado layout
  the kernel uses today.
- Properties 2110–2112 (17, 4, 160): undocumented SteamVR ids; meaning
  unknown.
- `debug/sleep action` values.
- Idle `l1/touch` pulse purpose.
- Charging status codes 1 and 2 (charging vs full?) need a controller on a
  charger.
- Whether SteamVR on Linux matches `EdidVendorID` as `0xd94d` or `0x4dd9`.

### B.9 Function map

| VA | Role | Covered |
|----|------|---------|
| `FUN_1801a2040` | ServerDriver::Init | yes |
| `FUN_1801a2a40` | ServerDriver::Cleanup (re-runs EDID choice) | brief |
| `FUN_1801a2cb0` | RunFrame | display parts |
| `FUN_1801a4390` | EDID / refresh choice | yes |
| `FUN_1801a47c0` | INI config + version log | yes |
| `FUN_18019d1b0` | HMD Activate / properties | yes |
| `FUN_18019ee30` | HMD update pass | yes |
| `FUN_1801a19a0` | HMD update thread loop | yes |
| `FUN_18019e860` | ComputeDistortion | yes |
| `FUN_1801a1a00` | Lens geometry from `0x8f` | yes |
| `FUN_1801a10c0` | Hidden-area mesh | summary |
| `FUN_18019eca0`…`FUN_18019ee20` | Display component getters | yes |
| `FUN_1801256c0` | Headset connect (reads `0x8f`) | `0x8f` part |
| `FUN_180124e30` | Copy `0x8f` block | yes |
| `FUN_18015e990` | Store `0x8f` in config manager | yes |
| `FUN_180155000`, `FUN_1801556b0` | Config defaults, rig transforms | driver-from-head |
| `FUN_180160230` | Tracker pose query | interface only |
| `FUN_1801a5760` | Sense Activate | yes |
| `FUN_1801a8680` | Sense frame: events, inputs, battery, fw props | yes |
| `FUN_18011b4d0` | Haptic request | yes |
| `FUN_1801a7bf0` | Sense pose | summary |
| `FUN_1801aa660` | Sense pose thread loop | yes |
| `FUN_1801a02b0` | Debug page (DP status, stream rates) | skipped |
| `FUN_18016b110` | QPC → µs clock | yes |
| `FUN_18016f350` / `f3e0` / `f470` | Rx / Ry / Rz from degrees | one line |
