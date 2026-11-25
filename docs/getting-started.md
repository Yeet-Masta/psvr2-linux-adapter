# Getting started

Building, installing and running the PS VR2 Linux driver.

Parts:

- [Part A: Install](#part-a-install)
- [Part B: Display bring-up](#part-b-display-bring-up)
- [Part C: SteamVR driver](#part-c-steamvr-driver)

---

## Part A: Install

Three ways to install, from quickest-to-try to most integrated. All need your
distro's **kernel headers** for the running kernel and a toolchain
(`gcc`/`make`).

### A.1 Build and load manually (no install)

Good for development and a quick try:

```bash
make -C kernel
sudo insmod kernel/psvr2.ko
make -C userspace/tools          # build the smoke-test tools
```

Unload with `sudo rmmod psvr2`. Device-node permissions (IIO, input,
`/dev/psvr2-pose`, `/dev/psvr2-gaze`, `/dev/videoN`) need the udev rules from
the install paths below, or run the tools as root.

### A.2 DKMS install (any distro)

Installs the module so it persists across reboots and rebuilds on kernel
upgrades, and installs the udev rules:

```bash
./install.sh             # everything below; asks for sudo where needed
```

Parts (run one or several by name, e.g. `./install.sh kernel udev`):

| Part | What |
|------|------|
| `kernel` | psvr2 module via DKMS (sources only copied to `/usr/src/psvr2-<ver>`), loaded right away |
| `udev` | `kernel/99-psvr2.rules` to `/etc/udev/rules.d` (device access, IMU buffer for the headset INS, Sense hidraw) |
| `group` | adds you to `input` (log out and back in once) |
| `steamvr` | builds `driver_psvr2` and symlinks it into SteamVR's `drivers/` (`steamvr/install-steamvr-driver.sh`) |
| `settings` | `"displayPortTrainingMode": 1` in `steamvr.vrsettings` ([part C](#part-c-steamvr-driver)); a `.psvr2.bak` copy is kept |
| `lib` | libpsvr2 to `/usr/local` (not default; the driver links it statically) |

Iterating: `git pull && ./install.sh --update` rebuilds the DKMS module only
when `kernel/` changed (content hash in `/var/lib/psvr2`) and then reloads it,
re-installs the udev rules only when they changed, and rebuilds the SteamVR
driver incrementally (the symlink makes the new build live on the next
SteamVR start). `--no-reload` keeps the running module. Close SteamVR first:
it rewrites `steamvr.vrsettings` on exit and holds the module.

Driver settings: `./install.sh --set key=value` writes into the
`driver_psvr2` section of `steamvr.vrsettings` (defaults in
`steamvr/driver_psvr2/resources/settings/default.vrsettings`); `--set -key`
removes an override.

`./uninstall.sh` removes the module, rules, SteamVR registration and the
settings keys it added (and the `driver_psvr2` overrides); `--all` also libpsvr2
and the install state. The `input` group membership stays. Requires `dkms`,
`cmake`, a C/C++ toolchain and `python3` (for the settings edit).

### A.3 Arch package

```bash
cd packaging
makepkg -si
```

This builds `psvr2-dkms`; the `dkms` pacman hooks compile the module on install
and rebuild it on every kernel upgrade. Udev rules are installed to
`/usr/lib/udev/rules.d/`.

### Display (separate from the module)

The headset's panel is driven by your GPU's DRM stack, not this module. On AMD
you likely need the DSC/FEC EDID quirk:

```bash
patches/apply-amdgpu-dsc.sh /path/to/linux   # then rebuild amdgpu/kernel
```

Then bring the panel up (X11):

```bash
userspace/tools/psvr2-display-up.sh
```

See [part B](#part-b-display-bring-up) for AMD/NVIDIA/Wayland details.

### Verifying

```bash
dmesg | grep psvr2                   # probe messages per interface
lsusb -t                             # audio still on snd-usb-audio
ls /sys/bus/iio/devices/             # iio:deviceN named "psvr2_imu"
ls /dev/psvr2-* /dev/video*          # pose/gaze char devs + camera node
```

Then run the tools in `userspace/tools/` (`psvr2-imu-test`, `psvr2-pose-test`,
`psvr2-gaze-test`) or standard utilities (`evtest`, `v4l2-ctl`, `iio_readdev`).

### libpsvr2 (optional)

A C library wrapping the device nodes for application/runtime use:

```bash
make -C userspace/lib                 # builds libpsvr2.a/.so + psvr2-monitor
sudo make -C userspace/lib install    # installs header, libs, and pkg-config
```

Then build against it with `pkg-config --cflags --libs libpsvr2`. See
`userspace/lib/psvr2-monitor.c` for a worked example.

---

## Part B: Display bring-up

The PSVR2 panel is a standard DisplayPort 1.4 sink — **not** driven by this
project's kernel module. It is brought up by the GPU's DRM/KMS driver. The panel
runs at `4000x2040 @ 119.88 Hz` and requires **DSC** (Display Stream Compression)
and **FEC**, which some drivers do not enable for this sink without a quirk.

This is a separate track from the USB module: the IMU/buttons module in
`kernel/` works regardless of whether the display is lit (though see the open
question about whether the IMU streams before the DP link is up — noted in
[development.md part C](development.md#part-c-status-and-roadmap)).

### AMD (amdgpu)

`patches/amdgpu-psvr2-dsc-fec.patch` forces DSC 1.1 + FEC capabilities for the
PSVR2 EDID (manufacturer `0xD94D`, products `0xA205`/`0xC207`, plus the generic
MTK EDID `0x8B36:0x3612`). Without it, amdgpu may mis-detect the EDID and bring
the headset up at the wrong resolution.

Apply against a kernel source tree and rebuild amdgpu (or the kernel). The
helper script applies it (with a dry-run check first) to a git or tarball tree:

```
patches/apply-amdgpu-dsc.sh /path/to/linux
# then rebuild amdgpu / the kernel and reboot
```

### Bringing the panel up (X11)

The headset enumerates as a DRM non-desktop / lease device, so a normal desktop
won't draw to it. Under X11, promote it to a normal output with the helper
(it auto-detects the PSVR2 connector by its `4000x2040` mode):

```
userspace/tools/psvr2-display-up.sh          # enable as a normal display
userspace/tools/psvr2-display-up.sh --off    # hand it back as a lease output
```

Equivalent manual steps (replace `DP-N` with whatever connector the headset
enumerates as — the one advertising the `4000x2040` mode):

```
xrandr --output DP-N --set non-desktop 0
xrandr --output DP-N --auto
```

Under **Wayland** this is compositor-specific (the connector stays a
non-desktop/lease output for VR runtimes); the helper does not handle it.

### NVIDIA

NVIDIA GPUs are largely untested for this sink. The proprietary driver handles
DSC internally; `nouveau` DSC support is unverified here. This needs hardware
testing — tracked in [development.md part C](development.md#part-c-status-and-roadmap).

### Intel (reference)

Community reports indicate Intel Alder Lake iGPUs bring the 4K VR mode up
automatically over a USB-C→DP path, which has been one of the more reliable
display paths so far.

---

## Part C: SteamVR driver

A SteamVR driver that makes the PSVR2 appear as a native HMD. SteamVR (`vrserver`)
is Valve's runtime — this is a driver plugin it loads, not a replacement. The
driver is pure userspace and consumes the kernel module's device nodes through
**libpsvr2**; it needs no kernel changes.

### Status

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
- **Proximity sensor** — reported on the HMD's `/proximity` input, so SteamVR
  knows when the headset is worn. Putting it on also recenters the controllers
  the next time you hold them level and still in front of you
  (`recenter_on_wear`).
- **Finger tracking** — each controller has a hand skeleton
  (`/input/skeleton/left|right`, partial tracking). Thumb, index and the last
  three fingers curl from the touch and proximity sensors, so hands open,
  point and make a fist in skeletal-input games. The bone model is Valve's
  OpenVR hand-simulation sample (BSD-3, vendored in `third_party/openvr`).
- **Motion prediction** — controller poses carry smoothed linear and angular
  velocity and a `controller_prediction_ms` (default 12) look-ahead, which
  hides most of the Bluetooth latency.
- **Default trigger feel** — `default_trigger_effect` (`off`, `light`,
  `medium`, `heavy`, `click`, `rigid`; default `light`) applies whenever no
  Toolkit-aware app is driving the adaptive triggers.
- **Alerts** — the controller buzzes three times below 15 % battery and five
  times below 5 %. The headset rumbles when a controller drops out
  (`controller_alerts`).
- **Gyro-bias memory** — the learned gyro bias is saved per controller
  (`left_gyro_bias_*` / `right_gyro_bias_*`), so drift is low from the start.
- **Passthrough camera (experimental)** — the bottom tracking cameras are
  offered to SteamVR as a stereo camera (Room View). The camera geometry is
  estimated, not calibrated, so the 3D view is approximate. Turn it off with
  `passthrough_camera: false`.

### Build & install

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

### Running on Wayland

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

### How direct mode works

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

### Components

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

### Known limitations

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
  [development.md part C](development.md#part-c-status-and-roadmap)). Until then the controllers rotate correctly and
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
  usual calibration (Reset Seated Position / Room Setup). Without a room setup,
  set `standing_height_m` to about your eye height to lift the view off the
  floor.
