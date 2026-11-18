# Sony PS VR2 driver: SteamVR-facing behaviour

Behavioural spec of the parts of Sony's SteamVR driver
(`driver_playstation_vr2_orig.dll`, PE timestamp `1741940190`, image base
`0x180000000`) that face SteamVR: device properties, display timing, lens
geometry, input components, hand skeleton, play area. Written in our own words
from analysis of the binary; no Sony code or tables are reproduced. Function
names are Ghidra addresses in that build. Method and USB protocol: see
[`../sony-driver-re.md`](../sony-driver-re.md).

Implementation: [`userspace/lib/sie/steamvr/`](../../userspace/lib/sie/steamvr/).
Differential tests: `scripts/sony-re/difftest/t_steamvr_*.c`. Status per
function: [`scripts/sony-re/status/steamvr.tsv`](../../scripts/sony-re/status/steamvr.tsv).

Confidence tags as in the main doc: **[sure]** read from code and/or proven by
differential test, **[likely]**, **[guess]**.

## Test harness (fake OpenVR host)

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

## Lens geometry (`FUN_1801a1a00`) [sure]

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

## Hidden-area mesh (`FUN_1801a10c0`) [sure]

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

## HMD device object (`sie::psvr2::HmdDevice`)

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

### Display component [sure, difftest `t_steamvr_display`]

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

### HMD properties (`Activate`, `0x18019d1b0`) [sure, difftest `t_steamvr_hmd`]

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

## Refresh rate and EDID type (`FUN_1801a4390`) [sure, difftest `t_steamvr_edid`]

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

### HMD update (`FUN_18019ee30`, run by `UpdateThread`) [pose: sure, difftest `t_steamvr_hmdpose`; rest likely]

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

## Sense controller (`sie::psvr2::SenseDevice`)

`ITrackedDeviceServerDriver` vtable `0x180264de0`: Activate `0x1801a5760`,
then `0x1801a7a50`, `0x1801a7b10`, `0x1801a7b20`, `0x1801a7b00`,
`0x1801a7b70`. Side at `+0xc` (0 left, 1 right), object id at `+8`.

### Activate [sure, difftest `t_steamvr_sense`]

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

### Controller pose (`FUN_1801a7bf0`) [sure, difftest `t_steamvr_pose`]

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

## Hand skeleton (`HandSkeletonSpace`) [bones: sure, difftest `t_steamvr_skel`; blend: likely]

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

## Server driver frame (`ServerDriver::RunFrame`, `FUN_1801a2cb0`) [likely, spec only]

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

## Project driver vs Sony

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
