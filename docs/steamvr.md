# SteamVR / OpenVR driver (`driver_psvr2`)

A SteamVR driver that makes the PSVR2 appear as a native HMD. SteamVR (`vrserver`)
is Valve's runtime — this is a driver plugin it loads, not a replacement. The
driver is pure userspace and consumes the kernel module's device nodes through
**libpsvr2**; it needs no kernel changes.

## Status

- **Display** — direct mode via DRM leasing: the runtime leases the headset's
  display connector and lights the 4000×2040 panel while the desktop keeps
  running on your other monitors.
- **Head tracking** — 6DoF pose from the headset's onboard tracker, delivered in
  the correct OpenVR frame.
- **Lenses** — Sony's lens model (ported from Monado) with this headset's own
  calibration block, read through the `lens_calibration` sysfs attribute, plus
  the asymmetric per-eye FOV. The IPD comes from the headset's dial.
- **Eye tracking** — combined gaze on the HMD's `/eyetracking` component, so
  OpenXR apps get `XR_EXT_eye_gaze_interaction` through SteamVR.
- **Sense controllers** — paired over Bluetooth (or on USB), found
  automatically, reconnect at any time: every button, capacitive touch and
  finger proximity, battery, PCM haptics or classic rumble, and adaptive
  triggers. **Orientation is 3DoF from the controller IMU; position is an arm
  model** (see [Known limitations](#known-limitations)). Hold Create (left) or
  Options (right) while pointing the controller straight ahead and level:
  - after 1 s (short buzz) its heading is lined up with your view again;
  - keep holding to 3 s (long buzz) to also calibrate its tilt. The
    calibration is saved per controller in the `driver_psvr2` settings
    section (`left_calibration_*` / `right_calibration_*`); delete those keys
    to reset it.
- **PSVR2 Toolkit IPC** — a compatible server on `127.0.0.1:3364`, so software
  written for PSVR2 Toolkit (VRCFT modules, game mods) gets gaze, eye openness
  and adaptive trigger effects. `scripts/toolkit-ipc-test.py` exercises it.
- **Headset rumble** — the HMD's `/output/haptic`; works only on a jailbroken
  headset (see below).
- **Proximity sensor** — taking the headset off / putting it on sends SteamVR's
  user-interaction events.

## Build & install

```sh
# Builds libpsvr2 + the driver and registers it with SteamVR (via a symlink).
steamvr/install-steamvr-driver.sh
# Remove it again:
steamvr/install-steamvr-driver.sh --uninstall
```

The script auto-detects the SteamVR `drivers/` directory; override with
`STEAMVR_DRIVERS=/path/to/SteamVR/drivers`. Because it registers the build output
by symlink, rebuilding (`cmake --build steamvr/driver_psvr2/build`) is enough to
pick up changes — no reinstall needed.

The driver is built against the OpenVR SDK's `openvr_driver.h` and links
`libpsvr2` statically, so the resulting `.so` is self-contained.

## Running on Wayland

Direct mode works on a Wayland session (validated on KDE Plasma 6 / KWin) — no
TTY or X11 required — with two settings:

1. **Force the headset's GPU.** On a multi-GPU machine, point SteamVR's Vulkan at
   the GPU the headset is wired to, or the compositor can't acquire the display.
   For an AMD/RADV headset GPU, launch Steam with:

   ```sh
   VK_DRIVER_FILES=/usr/share/vulkan/icd.d/radeon_icd.json steam
   ```

   (or set it in the SteamVR launch options as `VK_DRIVER_FILES=... %command%`).

2. **Set `displayPortTrainingMode` to `1`.** With SteamVR fully shut down
   (`steam -shutdown`, so the edit isn't overwritten), add to the `"steamvr"`
   section of `~/.local/share/Steam/config/steamvr.vrsettings`:

   ```json
   "displayPortTrainingMode" : 1,
   ```

   The default (`0`) produces an invalid atomic modeset on the leased connector
   on Linux — the kernel logs `dc_stream_state is NULL for crtc` and the
   compositor drops the surface. Mode `1` takes a DP link-training path that
   modesets the 4000×2040 panel correctly.

With both, the panel lights in direct mode with correct color and 6DoF tracking
while the desktop keeps running on your other monitors.

## How direct mode works

For a standard DisplayPort-connected HMD, the driver does **not** do the leasing
or the rendering itself — SteamVR's compositor does. In particular it does **not**
implement `IVRDriverDirectModeComponent` (that interface makes a driver own GPU
swapchain presentation, and is meant for wireless or proprietary display
pipelines). Instead, direct mode is enabled by **describing the display
correctly** so `vrcompositor` finds and DRM-leases the connector itself:

- `IVRDisplayComponent::IsDisplayRealDisplay()` → `true`
- `IVRDisplayComponent::IsDisplayOnDesktop()` → `false` (the trigger for direct
  acquisition; `true` would mean an extended-desktop monitor)
- the panel's **EDID** vendor/product identity, so the compositor can match and
  lease the right connector

This makes the headset's "non-desktop" display connector — the one a desktop
compositor normally hides — available to the runtime, which leases it while your
desktop session keeps running. It is controlled by the `direct_mode` setting
(default `true`) in `default.vrsettings`; setting it `false` selects an
extended-desktop fallback instead.

## Components

```
driver_psvr2/
  src/
    hmd_driver_factory.cpp   HmdDriverFactory — the entry point vrserver loads
    device_provider.{h,cpp}  IServerTrackedDeviceProvider (lifecycle, events)
    hmd_device_driver.{h,cpp}ITrackedDeviceServerDriver + IVRDisplayComponent
    pose_source.{h,cpp}      libpsvr2 wrapper + device→OpenVR pose remap
    controller_device.{h,cpp}Sense controller: inputs, haptics, 3DoF + arm model
    sense_hid.{h,cpp}        Sense hidraw transport, calibration, output report
    sense_protocol.h         Sense report layouts (from Monado, BSL-1.0)
    trigger_effect.{h,cpp}   adaptive trigger effects -> output report bytes
    imu_fusion.h             gyro + accelerometer orientation filter
    toolkit_ipc.{h,cpp}      PSVR2 Toolkit legacy IPC server
  resources/
    driver.vrdrivermanifest
    settings/default.vrsettings
    input/                   Sense + HMD input profiles, Touch remapping,
                             legacy bindings
    rendermodels/            Sense-style controller models (boucymatt fork)
```

Controllers need read/write access to their `hidraw` node; `install.sh` puts
the udev rule in place. Pair them with your desktop's Bluetooth settings (hold
PS + Create/Options until the light blinks). The driver reads the IMU
calibration feature report on connect, which also switches the controller from
its compatibility reports to full reports, and sets the lower PC polling rate.

`enhanced_haptics` (default true) plays SteamVR haptic pulses as PCM on the
voice-coil actuators; set it false for classic rumble. `toolkit_ipc` (default
true) turns the IPC server off when real PSVR2 Toolkit owns the port.

At runtime the device provider opens `libpsvr2` and registers the HMD; a pose
thread reads 6DoF samples, remaps them into the OpenVR frame, and submits them via
`TrackedDevicePoseUpdated()`. The display component reports the panel geometry
(4000×2040 split into two eyes) and its EDID identity.

## Known limitations

- **Multi-GPU systems.** If the headset is wired to a different GPU than the one
  SteamVR renders on, the compositor can fail to acquire the display. Point
  SteamVR's Vulkan at the GPU that drives the headset — see
  [Running on Wayland](#running-on-wayland) (the same `VK_DRIVER_FILES` applies on
  X11).
- **Session type.** Direct mode is validated on both **X11** and **Wayland**
  (KDE Plasma 6 / KWin). Wayland needs both settings in
  [Running on Wayland](#running-on-wayland); X11 needs only the GPU force.
- **Controller position (6DoF).** Sony's PC driver finds the controllers by
  their LED constellation in the headset cameras; that camera mode, the LED
  sync handshake and the tracker are not implemented here yet (see
  [roadmap.md](roadmap.md)). Until then the controllers rotate correctly and
  sit at an arm-model position relative to your head; yaw can drift slowly,
  which the Create/Options hold corrects.
- **Ignition / PSVR2 Toolkit.** They run Sony's Windows driver, which drives the
  headset over libusb; it cannot share the headset with this kernel module.
  Use one or the other: unregister the other driver in SteamVR (`vrpathreg
  removedriver <path>` or disable it under SteamVR Settings > Startup /
  Shutdown > Manage Add-Ons), and turn `toolkit_ipc` off while Toolkit runs.
- **Headset rumble** needs a headset jailbroken with
  [vr2jb](https://github.com/BnuuySolutions/vr2jb) (firmware 6.0, redone after
  every power cycle; see its warnings). Stock firmware ignores the command.
- **Spatial calibration.** Floor height and forward direction come from SteamVR's
  usual calibration (Reset Seated Position / Room Setup). Driver-side
  auto-recenter and a default standing height are planned so this works out of the
  box.
