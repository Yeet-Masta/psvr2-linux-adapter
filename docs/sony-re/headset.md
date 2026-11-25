# Headset

Sony's headset USB handling, the headset INS and the IR tracker thread.

Parts:

- [Part A: Headset USB behaviour](#part-a-headset-usb-behaviour)
- [Part B: Headset INS and IR tracker thread](#part-b-headset-ins-and-ir-tracker-thread)

---

## Part A: Headset USB behaviour

Scope: everything Sony's SteamVR driver (`driver_playstation_vr2_orig.dll`, PE
timestamp 1741940190) does with the headset over USB: enumeration, interface
and alternate-setting choice, init order, control reports, the bulk reader
threads with their error and restart handling, the status stream,
proximity/see-through/power, firmware checks, and shutdown.

Function names are Ghidra addresses in that build. Tags: **[sure]** read in
code, **[likely]** strong inference, **[guess]**. No Sony code is quoted.

Read with [overview.md part B](overview.md#part-b-driver-architecture-and-code-map) (protocol already
decoded) and [hardware-protocol.md part B](../hardware-protocol.md#part-b-usb-protocol) (the Linux module's view).

### A.1 Driver summary

What a Linux driver must do, in priority order:

1. **Kernel: bind every interface independently.** Sony runs one reader
   object per USB interface. Each opens its own WinUSB handle on
   `vid_054c&pid_0cde&mi_NN`, retries every 500 ms while it is missing, and
   runs its own connect hook and read loop (`FUN_180127b70`). One endpoint
   failing tears down only that interface's handle; the others keep running.
2. **Kernel: on the status interface (IF7) connect, do the init sequence**
   (section 3.2): wait 1 s, Get `0x81` (retry once after 1 s, fail if the
   firmware-info dword at +0x34 is 0), Get `0xa1`, read the device
   descriptor, and, only on a USB3 link (`bcdUSB ≥ 0x300`), Get `0x8f`,
   Set `0x17 {0,3}` (see-through off), Set `0x22`, select **alternate
   setting 1 on IF7**, Set `0x12` brightness (default `0x1f`).
3. **Kernel: on the other interfaces' connect**, push their configuration
   blocks: IF3 (SLAM) sends Set `0x1f` (fixed 256-byte table); IF8 (LED
   detection) and IF10 (calibration) seed config that their loops then send
   (`0x14`/`0x09`, camera mode `0x0b`).
4. **Kernel or userspace: refuse USB2.** Every gate in Sony's code checks
   `bcdUSB ≥ 0x300`. On USB2 the status loop does nothing and the HMD is
   "not ready"; SteamVR init logs a reconnect-to-USB3 message and fails.
5. **Userspace (or kernel): clock sync.** Get `0xde` every 500 ms; keep a
   phase/drift model of the device µs clock (section 5.1). Without it,
   SLAM/LED/IMU timestamps can't be put on the host clock.
6. **Userspace: see-through on the function button.** Rising edge of status
   byte 2 while worn toggles Set `0x17`; taking the headset off forces
   see-through off. Poll Get `0x97` once a second for the real state.
7. **Userspace: brightness follow-up.** If status byte 10 differs from the
   configured brightness, re-send Set `0x12`.
8. **Userspace: refresh-rate EDID.** Get `0xa9` / Set `0x29` before the
   compositor starts, then wait 1 s for the display to re-enumerate (see
   [steamvr-and-display.md part A](steamvr-and-display.md#part-a-steamvr-facing-behaviour)).
9. **Userspace: version gates.** Compare Get `0xa1` with the driver's own
   version (refuse if the headset needs a newer driver), and the `0x81`
   firmware version with the bundled firmware (offer an update).
10. **Userspace: power events.** On resume from system suspend, Sony sends
    Set `0x05` subcmd 2 (headset restart [likely]). On SteamVR quit it sends
    Set `0x27` u16 1000.

Kernel module: items 1–4 (USB plumbing, init, alt setting, per-interface
reconnect), and exposing Get/Set to userspace (an ioctl or sysfs for `0x17`,
`0x12`, `0x29`, `0x05`, `0x27`, `0xde`). Userspace: policy (button, EDID,
version gates, power events) and the clock model.


### A.2 Wire and data formats

#### A.2.1 Interfaces and reader objects [sure]

Sony creates one `CaesarUsbThread*` object per interface (manager
`FUN_180121af0`, start `FUN_180123130`). Each object has a vtable with: thread
body `FUN_180127b70`, stop, interface number, pipe id, connect hook,
disconnect hook, loop body. Vtables were matched by their interface/pipe
getters.

| Class | IF | EP | Read size | Connect hook | Loop | Disconnect hook |
|-------|----|----|-----------|--------------|------|-----------------|
| `ImuStatus` | 7 | `0x88` | 0x4000 | `FUN_1801256c0` (main init, §A.3.2) | `FUN_1801268d0` | `FUN_180125b50` (clear firmware info, descriptor, status) |
| `SlamTracking` | 3 | `0x83` | 0x200 | `FUN_180125ab0` (Set `0x1f`) | `FUN_1801270c0` | none |
| `Image` | 6 | `0x87` | 0x200100 | none | `FUN_180125e00` | none |
| `Leddet` | 8 | `0x89` | 0x9050 | `FUN_180125a10` (seed LED config) | `FUN_180126b80` | none |
| `RelocPre` | 9 | `0x8a` | 0xc8780 | `FUN_180125a50` (Get `0x8a`) | `FUN_180126fa0` | none |
| `GenData` | 10 | `0x8b` | 0x40000 | `FUN_180125670` (seed camera mode `0x10`) | `FUN_180125cc0` | none |
| `Log` | 12 | `0x8d` | 0x100000 | none | `FUN_180126e70` | none |

IF5 (gaze, `0x85`) has no object. A base-class vtable at `0x18024a9e8`
(IF 0, pipe 0, all hooks empty) exists but no thread of it is started
[likely].

Manager slots (`FUN_180123130`, `FUN_180127840`): `+0xb8` ImuStatus,
`+0xc0` Image, `+0xc8` SlamTracking, `+0xd0` Leddet, `+0xd8` GenData,
`+0xe0` RelocPre, `+0xe8` Log. Start order: ImuStatus, Log, Image, Slam,
Leddet, GenData, RelocPre. The second argument of the thread start
(`FUN_18016b7c0`) is 3 for ImuStatus/Image/Slam/Leddet and 1 for the rest
(thread priority [guess]).

Bulk reads (`FUN_180127d60`) are blocking `WinUsb_ReadPipe` calls with
WinUSB's default policy: Sony never sets a pipe timeout, so a read waits
until data arrives or the pipe is aborted [sure]. The call returns the byte
count. A zero-length read is ignored (loop returns 0).

#### A.2.2 Control transfer framing (`FUN_1801283f0`) [sure]

| Field | Set (host to device) | Get (device to host) |
|-------|----------------------|----------------------|
| bmRequestType | `0x42` (vendor, recipient endpoint, OUT) | `0xc2` (vendor, recipient endpoint, IN) |
| bRequest | `0x09` | `0x01` |
| wValue | **`0`** in every call | report id |
| wIndex | `0` | `0` |
| wLength | len + 8 | len + 8 |

Payload (both directions): `u8 report_id, u8 0, u16 subcmd, u16 len, u16 0`,
then `len` bytes. In a Get, the 8-byte header is in the reply too and the
data is copied from offset 8. The buffer is 512 bytes, so `len ≤ 504`.
Every call in the DLL uses subcmd 1, except `0x05` (subcmd 2).

The transfer goes through the WinUSB handle of whichever interface object
makes the call (ep0 is shared). It fails at once, without I/O, when that
object has no handle (error `-0x233`) or is stopping (`-0x22c`). There is no
retry inside the function. Errors from `WinUsb_ControlTransfer` are logged
(`[Error] %s Report failed`) and stored in the object's last-error field
`+0x218`.

#### A.2.3 Control reports: complete list

Every control call site in the DLL (26 calls). "Who" is the Sony function
that issues it.

| ID | Dir | Len | When | Who | Payload / reply |
|----|-----|-----|------|-----|-----------------|
| `0x81` | Get | 0x54 | IF7 connect, after 1 s; once more 1 s later if +0x34 is 0 | `FUN_180124450` | Firmware info, §A.2.4 |
| `0xa1` | Get | 4 | IF7 connect | `FUN_1801256c0` | u32 **minimum PC-driver version** the headset needs, §A.2.5 |
| `0x8f` | Get | 0x100 | IF7 connect, USB3 only | `FUN_1801256c0` | 256-byte block, to config manager (`FUN_18015e990`). The Linux module reads the first 136 bytes as the lens calibration (Monado layout) |
| `0x17` | Set | 2 | IF7 connect (off), function button, unworn | `FUN_180124580` | `{u8 on, u8 3}` see-through. Skipped when the `no seethrough` debug setting is on |
| `0x22` | Set | 1 | IF7 connect, USB3 only | `FUN_1801256c0` | u8 from manager `+0x4e6f` (start argument). SteamVR passes **0** (`FUN_1801a2040`) |
| `0x12` | Set | 1 | IF7 connect (end); status byte 10 ≠ config | `FUN_1801256c0`, `FUN_180123f60` | u8 brightness, config key 2, default `0x1f` |
| `0xde` | Get | 0x18 | 3× at first status read; then every 500 ms | `FUN_1801268d0`, `FUN_180123fa0` | u32 device time (µs) at +4; §A.5.1 |
| `0x97` | Get | 1 | Status loop, once a second (not within 1 s of a `0x17` Set) | `FUN_1801268d0` | u8 see-through state (≠0 = on) |
| `0x1f` | Set | 0x100 | IF3 connect | `FUN_180125ab0` | Fixed table, §A.2.6 |
| `0x8a` | Get | 8 | IF9 connect | `FUN_180125a50` | u32 at +0. **Never stored**: Sony keeps it only if the return value is > 0, and the function returns 0 on success [sure, Sony bug] |
| `0x14` | Set | 0x10 | LED loop, when the LED config changes | `FUN_180126b80` | `{u32 0x55, u32 500, u32 0x24800000, u32 1}` by default (`FUN_180125a10`) |
| `0x09` | Set | 0x18 | LED loop, when the second LED config changes | `FUN_180126b80` | 24 bytes from config manager |
| `0x0b` | Set | 8 | GenData loop: config change, or once a second until calibration types 10 and 12 have arrived | `FUN_180125cc0` | `{u32 1, u32 mode}`; mode `0x10` (seeded at IF10 connect); only VrTrace capture changes it |
| `0xa9` | Get | 1 | Server init / cleanup / direct-mode change | `FUN_180124970` | EDID type |
| `0x29` | Set | 1 | Same, when the EDID type must change | `FUN_180127e80` | EDID type; then Sleep 1000 ms |
| `0x25` + `0xa5` | Set 0 / Get 0x60 | | Debug page | `FUN_1801248d0` | DP receiver status |
| `0x27` | Set | 2 | `VREvent_QuitAcknowledged` from `vrmonitor.exe` or `steam.exe` | `FUN_180125610` (from `FUN_1801a2cb0`) | u16 `1000` |
| `0x05` | Set | 0 | Resume from suspend (sleep action 2, default); direct-mode workaround | `FUN_180127fa0` | subcmd **2**, no data |
| `0x28` | Set | 0 | Resume from suspend when sleep action = 3 | `FUN_180127e40` | none |
| `0x0a` | Set | 0xd8 | Relocaliser | `FUN_180127fe0` | topic room |
| `0xa4`, `0xa6` | Get | 4, 0x20 | Controller tracking | `FUN_180124ee0`, `FUN_180124c00` | topic ctrltracker |

#### A.2.4 Firmware info, Get `0x81` (0x54 bytes) [sure unless tagged]

Stored at ImuStatus `+0x49e4`.

| Offset | Type | Meaning |
|--------|------|---------|
| 0x00 | u32 | **HMD software version**. Shown as `%08x` in `TrackingFirmwareVersion`; compared with the version in `Firmware\PS_VR2_FW\HMD2_FIRMWARE.CUP` (`FUN_180125480`: update needed when smaller); `> 0x06000000` enables 90/120 Hz in SteamVR properties (`FUN_18019d1b0`) |
| 0x04 | u32 | Set to `0xffffffff` on disconnect; meaning unknown |
| 0x34 | u32 | **Board stage** index: 2 EVA2, 3 EVA3, 4 EVT1, 5 EVT1R, 6 EVT2, 7 DVT1, 8 DVT2, 9 PVT, 10 MP (`HardwareRevision_String`). `0` = info not valid yet (init retries, then fails). The headset counts as ready only when it is neither 0 nor `0xffffffff` (`FUN_1801253c0`) |
| others | | Copied, not read by the driver |

#### A.2.5 Version gates [sure]

- **Driver version.** Sony's own version is `plugin version` from
  `major_version.txt` (default `0x04110006`), with the low 16 bits replaced
  by the BCD digits of `jenkins` from `version.txt` when that is > 0
  (`FUN_1801a47c0`). If Get `0xa1` is greater, init shows the "update
  required" dialog (`FUN_180132330`) and fails (`FUN_1801a2040`). The HMD
  update thread does the same check and sets the "requires update" flag that
  makes the pose result 1 (`FUN_18019ee30`). So `0xa1` is the **minimum host
  driver version the firmware accepts** [likely].
- **Firmware version.** `0x81 +0x00 <` the bundled CUP version means
  "firmware update available": SteamVR `Firmware_UpdateAvailable` etc. and a
  dialog. Sony's driver never flashes firmware; the URL points to the PS VR2
  app.
- **USB3.** `bcdUSB` from the device descriptor (`FUN_180124510`,
  `WinUsb_GetDescriptor`, 18 bytes) must be `≥ 0x300` for any of: Get
  `0x8f`, Set `0x22`, alt setting 1, Set `0x12`, clock sync, `0x97` polling,
  status parsing, and "HMD ready".

#### A.2.6 Set `0x1f` table (256 bytes, `FUN_180125ab0`) [sure values, guess meaning]

| Offset | Type | Value |
|--------|------|-------|
| 0x00 | u16 | 40 |
| 0x02 | float ×4 | 4.0, 0.05236 (3°), −0.35, 0.35 |
| 0x12 | u32 | 10 |
| 0x16 | u32 | 5 |
| 0x1a | float ×4 | 1.0, 1.5, 0.06981 (4°), 0.04189 (2.4°) |
| 0x2a.. | | zero |

Sent once per IF3 connect, so the onboard SLAM gets it before poses are
read. The radians and metre-sized values suggest SLAM/keyframe thresholds
[guess]. Read from `.rdata` `0x18024af80..0x18024af9f`.

#### A.2.7 Status header (first 32 bytes of each `0x88` transfer) [sure unless tagged]

Sony copies it under the lock (`FUN_1801268d0`) and passes a reduced
"status info" record to the config manager (`FUN_180128240`), which the
share manager reads (`FUN_180132890`, log names in quotes).

| Byte | Name in Sony's logs | Use |
|------|---------------------|-----|
| 0 | "mode" | 1 = VR mode / display active [likely] (hardware-protocol.md part B calls it `dprx_status`). Mode becoming 1 re-arms dialogs and the black-screen timer |
| 1 | "proximity" | 1 = worn. SteamVR `/proximity`; gate for the see-through toggle |
| 2 | (function button) | Rising edge 0→1 toggles see-through while worn |
| 3 | — | Passed on, not used [guess] |
| 5 | (IPD) | IPD dial, mm (steamvr-and-display.md part A) |
| 6 | "thermal_warning" | Logged on change; shown by the PS VR2 overlay [likely] |
| 5..8 | u32 | Passed on as a whole too |
| 9 | 4 flag bits | Bits 0..3 split into 4 bools; meaning unknown |
| 10 | (brightness) | Current panel brightness; mismatch re-sends Set `0x12` |

The transfer must be ≥ 32 bytes, else the loop fails and the interface
reconnects. IMU records follow (`FUN_180127620`, hardware-protocol.md part B).

### A.3 Sequences

#### A.3.1 Per-interface thread (`FUN_180127b70`, `FUN_180125bb0`) [sure]

```
state = DISCONNECTED
loop until stop:
    if DISCONNECTED:
        open "vid_054c&pid_0cde&mi_%02x" (SetupDi, CreateFile, WinUsb_Initialize)
        if it fails: Sleep(500); continue
        log MaxTransferSize (pipe policy 8); state = CONNECTED
        if connect_hook() != 0: goto drop
        connect_count++
    if CONNECTED:
        if loop_body() != 0: goto drop     # read error, short read, bad magic
        continue
drop:
    log "USB Inf:%02x Disconnected(%d)"; disconnect_hook()
    state = DISCONNECTED; WinUsb_Free; CloseHandle; Sleep(500)
```

Each interface is independent. A bad magic on one stream (`"SL"`, `"VI"`,
`"LD"`, `"RP"` + size `0xc8780`, `"VD"` version) closes and reopens only
that interface. `connect_count` (`+0x1e4`) of ImuStatus is the "link state"
that `RunFrame` compares across frames for the USB2 warning.

#### A.3.2 Headset init (IF7 connect hook, `FUN_1801256c0`) [sure]

1. `Sleep(1000)`.
2. Get `0x81` (0x54 bytes). If `+0x34 == 0`: `Sleep(1000)`, Get `0x81`
   again. Still 0: log "Failed to read PS VR2 firmware info", fail the
   connect (interface drops, retries in 500 ms).
3. Get `0xa1` (4 bytes) and cache it.
4. Read the device descriptor; fail the connect if that fails.
5. If `bcdUSB < 0x300`: stop here, connect succeeds (USB2: nothing else is
   set up; the loop only re-reads the descriptor).
6. Notify the relocaliser/map module (`FUN_1801ad9b0`; topic room).
7. Get `0x8f` (256 bytes) and cache it.
8. Set `0x17 {0, 3}`: see-through off (cached state = 0).
9. Set `0x22 {0}` (value from the start argument; SteamVR passes 0).
10. Clear the "PC support checked" flag.
11. Hand the `0x8f` block and the `0x81` record to the config manager.
12. `WinUsb_GetCurrentAlternateSetting` on **IF7**; if not 1,
    `WinUsb_SetCurrentAlternateSetting(1)`. A failure is logged and kept in
    `+0x218`, but init goes on.
13. Set `0x12 {brightness}`; brightness = config key 2, default `0x1f`.

The other interfaces connect in parallel, each 500 ms retry on its own:
IF3 sends Set `0x1f` at once; IF8 seeds the LED config (the loop sends
`0x14`/`0x09`); IF10 seeds camera mode `0x10` (its loop sends `0x0b`);
IF9 sends Get `0x8a`. So `0x1f` and `0x0b` can reach the headset before the
IF7 init has finished [sure: no ordering between threads].

#### A.3.3 First status read and steady state (IF7 loop, `FUN_1801268d0`) [sure]

Each pass:

1. USB2 (`bcdUSB < 0x300`): re-read the descriptor and return. No read of
   `0x88`.
2. First pass after connect: Get `0xde` three times. If every reply has a
   zero device time, log "This PS VR2 firmware is not supporting PC
   Connection" and never run clock sync. (The connect still counts.)
3. If supported and more than 500 ms (`+0x224`) passed: clock sync (§A.5.1).
4. If more than 1 s passed since the last `0x97` poll or `0x17` Set:
   Get `0x97`, cache the see-through state.
5. Read `0x88` (0x4000). Error: drop. 0 bytes: return. < 32 bytes: drop.
6. Copy the header. See-through decision (§A.4.1).
7. Publish status info, parse IMU records.
8. Brightness check: if config brightness ≠ header byte 10, Set `0x12`.

#### A.3.4 SteamVR server init (`FUN_1801a2040`) [sure]

1. Start the USB manager and all threads (`FUN_180125330` →
   `FUN_180123130`). This also reads the bundled firmware version from
   `..\Firmware\PS_VR2_FW\HMD2_FIRMWARE.CUP`.
2. EDID selection (Get `0xa9`, maybe Set `0x29` + 1 s).
3. Wait for "HMD ready" (`FUN_1801253c0`: IF7 connected, USB3, board stage
   valid): poll at 0, 1 s, 2 s. On the third miss: log "found via USB2…"
   if `bcdUSB` is 1..0x2ff, else "HMD is not found", and fail init.
   (Because IF7 connect itself sleeps 1–2 s, this is tight.)
4. Read version files and debug settings (`FUN_1801a47c0`).
5. Get `0xa1` cache vs driver version: refuse if the headset needs a newer
   driver.
6. Create the HMD and two Sense devices.

#### A.3.5 Per frame (`ServerDriver::RunFrame`, steamvr-and-display.md part A) [sure]

- USB link check: if ImuStatus `connect_count` changed and `bcdUSB` is
  1..0x2ff, log the USB2 warning.
- `VREvent_QuitAcknowledged` from `vrmonitor.exe`/`steam.exe`: Set `0x27`
  u16 1000.
- Direct-mode setting changed and the EDID choice did nothing: Set `0x05`
  subcmd 2 ("[DirectMode Check][WA] reboot").
- USB stats once a minute (`FUN_180127840`, log only).

#### A.3.6 System suspend/resume (`FUN_1801a4e60`, `FUN_1801a3c00`) [sure]

A hidden window (`PSVR2_InvisibleWindow`) receives `WM_POWERBROADCAST`.
Only `PBT_APMRESUMESUSPEND` (7) is handled:

1. `Sleep(sleep action wait)` if > 0 (default 0).
2. By `sleep action` (default **2**):
   - 1: post SteamVR event 0x2c0 (driver requests quit) [likely].
   - 2: Set `0x05` subcmd 2 (headset restart [likely]).
   - 3: Set `0x28`.
   - 0: no window, nothing.

Both settings are in `[debug]` of `version.txt` next to the DLL. Nothing is
sent on suspend itself. The USB threads see the device vanish and reconnect
on their own.

#### A.3.7 Shutdown (`ServerDriver::Cleanup`, `FUN_1801a2a40`) [sure]

1. EDID selection again (normally no change).
2. Stop all USB threads (`FUN_180128320`): each sets its stop flag, calls
   `WinUsb_AbortPipe` on its pipe if connected (unblocks the read), joins;
   then `WinUsb_Free` and `CloseHandle`.
3. Tear down share, map, tracker, controllers; delete the manager
   (`FUN_180128390`).

No "power off", "standby" or see-through command is sent. HMD
`Deactivate` (`FUN_18019ebf0`) and `EnterStandby` (`FUN_18019ec50`) send
nothing either.

### A.4 State machines

#### A.4.1 See-through (status loop, `FUN_1801268d0`, `FUN_180124580`) [sure]

State: `st` = cached see-through (from Set `0x17` success or Get `0x97`).
Inputs per status packet: `worn = hdr[1] == 1`, `press = prev hdr[2] == 0
&& hdr[2] == 1`, `blocked = share counter (+0xb0) < 100` (`FUN_180131c80`;
a PS VR2 app or share-memory state; meaning [guess]).

| Condition | Action |
|-----------|--------|
| `!blocked && worn && press` | Set `0x17 {!st, 3}` (toggle) |
| `!blocked && worn && !press` | nothing |
| `(blocked or !worn) && st` | Set `0x17 {0, 3}` (force off) |
| otherwise | nothing |

With `no seethrough` set, `0x17` is not sent; only `st` changes. A Set also
postpones the next `0x97` poll by 1 s.

#### A.4.2 Interface link (per object) [sure]

| State | Event | Action | Next |
|-------|-------|--------|------|
| DISCONNECTED | open ok | connect hook | CONNECTED (or drop) |
| DISCONNECTED | open fails | Sleep 500 ms | DISCONNECTED |
| CONNECTED | loop error | disconnect hook, free handle, Sleep 500 ms | DISCONNECTED |
| any | stop | abort pipe, free handle | exit |

#### A.4.3 HMD readiness [sure]

`ready = IF7 CONNECTED && bcdUSB ≥ 0x300 && board_stage ∉ {0, 0xffffffff}`.
Not ready: the HMD reports the cached pose with `deviceIsConnected = 0`
(steamvr-and-display.md part A). The IF7 disconnect hook clears firmware info, descriptor and
status, so readiness drops at once on unplug.

#### A.4.4 Black screen after non-VR see-through (share, `FUN_180132890`, `FUN_180131990`) [likely]

When a non-VR see-through session ends (flag `+0x10e` set and `st` = 0),
Sony sets a "black screen" flag (log "nonvr seeThrough off"). It clears it
(log "black screen off") once header byte 0 ("mode") has been 1 for more
than 2.5 s; the timer restarts on a 0→1 mode edge. This only drives what
the PS VR2 overlay shows; no USB command is sent. The rest of the share
manager (play area, tracking lost, safety notice, function see-through
guide shown for 10 s) is UI for the PS VR2 app: out of scope.

### A.5 Algorithms

#### A.5.1 Clock sync (`FUN_180123fa0`, `FUN_180123b60`, `FUN_180124a90`) [sure unless tagged]

Sample (every 500 ms):

```
t0 = host_us(); Get 0xde; t1 = host_us()
rtt = t1 - t0
if rtt < 1000 µs (+0x25c):  fit(t1, t0, dev_us)          # §below
if rtt < 1000 µs:
    ring[i++ % 100] = host_before + rtt/2 - dev_us       # simple offset
    avg = mean of non-zero ring entries
    if |sample - avg| > 2000 µs: reset ring to this sample
also: min/max/mean RTT over 20 samples (stats only)
```

The fit is a second-order phase/drift loop on the device's 32-bit µs
counter. Defaults from the constructor (`FUN_180121af0`):

| Field | Value | Use |
|-------|-------|-----|
| `+0x224` | 500 000 µs | sync period |
| `+0x22c` | 20 000 µs | largest RTT a sample may have to be used |
| `+0x238`, `+0x23c` | 50, 40 | phase gain divisor (early, late) |
| `+0x240`, `+0x244` | 10 000, 50 000 | drift gain divisor (early, late) |
| `+0x248` | 50 000 | drift limit (ppb); larger resets the model |
| `+0x250` | 1 | samples per window (best-RTT pick within a window) |
| `+0x258` | 1 000 000 000 | drift scale (ppb) |
| `+0x230`, `+0x234`, `+0x254` | 100, 120, 2000 | not decoded |

Device to host conversion (`FUN_180124a90`):

```
if dev < last_dev && last_dev - dev > 0xff000000: dev += 1 << 32   # wrap
host = dev + offset + (dev - last_dev) * drift / 1e9
```

This is the timestamp map the LED, SLAM and IMU paths use. A Linux
implementation can use any equivalent estimator (min-RTT sample, offset +
linear drift, 32-bit wrap handling).

### A.6 Constants

| Name | Value | Meaning | VA |
|------|-------|---------|----|
| VID/PID | `0x054c` / `0x0cde` | Headset | `FUN_180122ac0` |
| Reconnect delay | 500 ms | Per interface, after open failure or drop | `FUN_180127b70` |
| Init delay | 1000 ms (+1000 retry) | Before Get `0x81` | `FUN_1801256c0` |
| Status alt setting | 1 on IF7 | Selected at end of init | `FUN_1801256c0` |
| Brightness default | `0x1f` | Config key 2 | `FUN_1801256c0` |
| USB3 gate | `bcdUSB ≥ 0x300` | Everything beyond firmware info | `FUN_1801256c0`, `FUN_1801268d0`, `FUN_1801253c0` |
| PC-support probe | 3× Get `0xde` | All device times 0 = unsupported | `FUN_1801268d0` |
| Clock sync period | 500 000 µs | | `FUN_180121af0` |
| Clock RTT limit | 1000 µs | Sample accepted | `FUN_180123fa0` |
| Offset ring | 100 entries, reset at 2000 µs | | `FUN_180123fa0` |
| See-through poll | 1 s | Get `0x97` | `FUN_1801268d0` |
| See-through arg | 3 | Second byte of Set `0x17` | `FUN_180124580` |
| Camera mode | `0x10` | Default, re-sent ≤ 1/s until calib arrives | `FUN_180125670`, `FUN_180125cc0` |
| LED config | `0x55, 500, 0x24800000, 1` | Set `0x14` default | `FUN_180125a10` |
| Quit command | Set `0x27` u16 1000 | SteamVR quit | `FUN_180125610` |
| Restart command | Set `0x05` subcmd 2 | Resume / direct-mode workaround | `FUN_180127fa0` |
| EDID settle | 1000 ms | After Set `0x29` | `FUN_1801a4390` |
| HMD-ready wait | 3 polls, 1 s apart | Server init | `FUN_1801a2040` |
| Driver version | `0x04110006` default | Compared with Get `0xa1` | `FUN_1801a47c0` |
| Sleep action | 2 (debug INI) | Resume behaviour | `FUN_1801a47c0` |
| Black-screen hold | 2.5 s of mode 1 | Overlay only | `FUN_180131990` |

### A.7 Gaps in the current Linux driver

Compared with `kernel/` (`psvr2_usb.c`, `psvr2_status.c`, …),
`userspace/` and `steamvr/driver_psvr2/src`. Not edited.

1. **No init sequence.** The module sends no Get `0x81`/`0xa1`, Set
   `0x17`, `0x22`, `0x1f`, `0x14`, `0x09`. It only sets brightness from
   sysfs, camera mode from V4L2 and gaze. Whether the firmware needs `0x22`
   or `0x1f` before SLAM is good is untested; Sony always sends them.
2. **Alt-setting order.** The module selects IF7 alt 1 in probe, before
   any control report. Sony does it last, after `0x81`/`0xa1`/`0x8f`/`0x17`/
   `0x22`. Probably harmless; note it if the IMU stream misbehaves.
3. **Set wValue.** The module puts `report_id` in wValue for Set; Sony
   always sends 0. The payload header carries the id in both, so this
   likely does not matter, but Sony's form is the tested one.
4. **Control timeout.** The module uses 100 ms. Sony uses WinUSB's default
   (no short timeout). Get `0x81` right after plug-in, or `0x29`, may need
   longer.
5. **No firmware-info gate.** Nothing checks board stage `0x81 +0x34`, the
   `0xa1` driver version, or the firmware version.
6. **No clock sync.** No `0xde` polling and no device-to-host time map, so
   SLAM/IMU/LD timestamps stay in device µs.
7. **No see-through toggle.** The function button and Get `0x97`/Set `0x17`
   are not handled (`hmd_device_driver.cpp` only mirrors proximity). The
   project has a camera passthrough setting instead.
8. **No brightness follow-up** from status byte 10.
9. **IF12 (`0x8d`, firmware console log) is not bound.** It is cheap and
   useful for debugging (strip ANSI colours, split lines).
10. **No per-interface recovery.** The module resubmits URBs; on a stream
    error or bad magic Sony instead reopens that interface after 500 ms.
11. **Camera mode.** Sony runs mode `0x10` all the time (SLAM needs it, as
    `psvr2_protocol.h` notes). The module sets mode 1 when V4L2 streams and
    mode 0 when it stops, which would stop onboard tracking.
12. **Power/quit.** No Set `0x27` on SteamVR quit and no Set `0x05` on
    resume.
13. **`0x8f` length.** The module reads 136 bytes; Sony reads 256. Read
    the full block in case the rest matters.

### A.8 Open questions (need hardware)

- `0x22`: what the byte means and what changes without it.
- `0x1f` table: which firmware behaviour it tunes; compare SLAM with and
  without it.
- `0x05` subcmd 2: confirm it reboots or resets the headset (watch for
  re-enumeration). `0x28`: effect unknown.
- `0x27 {1000}`: does the headset blank or go to standby after 1000 ms?
- `0x81` fields other than +0x00 and +0x34; `0x8f` bytes 136..255.
- Status header byte 0 values other than 0/1, byte 3, byte 9 bits.
- Get `0x97` / Set `0x17`: confirm `{1,3}` turns camera see-through on in
  the headset itself, and what the `3` means.
- Whether Get `0x8a` (RelocPre) returns anything useful, given Sony
  ignores it.
- Whether the firmware refuses SLAM/LED streams without the `0x81` read.

### A.9 Function map

| VA | Role | Covered |
|----|------|---------|
| `FUN_1801283f0` | Control transfer (Set/Get framing) | yes |
| `FUN_180127d60` | Bulk read (returns length) | yes |
| `FUN_180122ac0` | Open interface `mi_%02x` (SetupDi, WinUSB) | yes |
| `FUN_180122a70` | Close handle | yes |
| `FUN_180127b70`, `FUN_180125bb0` | Per-interface thread loop / one-shot connect | yes |
| `FUN_180121af0` | Manager constructor; clock-sync defaults | yes |
| `FUN_180123130` | Manager start; reads CUP firmware version; starts threads | yes |
| `FUN_180125330` | Create manager if needed, start | yes |
| `FUN_180128320`, `FUN_180128390` | Stop threads, delete manager | yes |
| `FUN_1801221a0` etc. (`0x180122120..0x180122330`) | Per-class destructors (abort pipe) | yes |
| `FUN_1801256c0` | IF7 connect: init sequence | yes |
| `FUN_180125b50` | IF7 disconnect: clear state | yes |
| `FUN_1801268d0` | IF7 loop: status, see-through, brightness | yes |
| `FUN_180124450` | Get `0x81` | yes |
| `FUN_180124510` | Read device descriptor (bcdUSB) | yes |
| `FUN_180124580` | Set `0x17` see-through | yes |
| `FUN_180123f60` | Set `0x12` | yes |
| `FUN_180123fa0`, `FUN_180123b60` | Clock sync sample and fit | yes (algorithm) |
| `FUN_180124a90` | Device µs to host µs | yes |
| `FUN_180128240` | Status header to config "status info" | yes |
| `FUN_180127620` | IMU record parse | no (hardware-protocol.md part B) |
| `FUN_180125ab0` | IF3 connect: Set `0x1f` | yes |
| `FUN_1801270c0` | IF3 loop (SLAM) | magic only (overview.md part B) |
| `FUN_180125e00` | IF6 loop (image) | magic only (overview.md part B) |
| `FUN_180125a10`, `FUN_180126b80` | IF8 connect / loop (LED) | yes / overview.md part B |
| `FUN_180125a50`, `FUN_180126fa0` | IF9 connect (Get `0x8a`) / loop | yes |
| `FUN_180125670`, `FUN_180125cc0` | IF10 connect / loop (camera mode, VD) | yes |
| `FUN_180127250` | VD parser | overview.md part B |
| `FUN_180126e70` | IF12 loop: firmware log | yes |
| `FUN_180125570` | Per-stream fps counter | skipped (stats) |
| `FUN_180127840` | Per-frame USB stats | skipped (stats) |
| `FUN_1801253c0` | HMD ready | yes |
| `FUN_180125480` | Firmware update needed | yes |
| `FUN_180125080` | Cached `0xa1` | yes |
| `FUN_180124a00`, `FUN_1801251a0` | Copy firmware info / status header | yes |
| `FUN_180125290`, `FUN_1801248c0` | bcdUSB / connect count | yes |
| `FUN_180125610`, `FUN_180127fa0`, `FUN_180127e40`, `FUN_180127e80` | Set `0x27` / `0x05` / `0x28` / `0x29` | yes |
| `FUN_180127f50` | `no seethrough` flag | yes |
| `FUN_1801a2040` | ServerDriver::Init (USB part) | yes |
| `FUN_1801a2a40` | ServerDriver::Cleanup | yes |
| `FUN_1801a47c0` | Version and debug INI settings | yes |
| `FUN_1801a4e60`, `FUN_1801a3c00` | Power-event window and handler | yes |
| `FUN_180132890`, `FUN_180134730`, `FUN_180131990`, `FUN_180131c80` | Share manager: status-info logs, black screen, blocked flag | outline (UI) |
| `FUN_180134100`, `FUN_180133960`, `FUN_180134280` | See-through guide, BT notice, safety notice | skipped (UI) |
| `FUN_18019ee30` | HMD update (`0xa1` check) | partly (steamvr-and-display.md part A) |

---

## Part B: Headset INS and IR tracker thread

Behavioural notes for the Sony driver range `0x180170000–0x18019ffff`
(build with PE timestamp `1741940190`). Each entry says what a function
does, in our own words, so it can be reimplemented without looking at the
decompile. Our code lives in `userspace/lib/sie/irtracker/`; per-function
status is in `scripts/sony-re/status/irtracker.tsv`; tests are
`scripts/sony-re/difftest/t_irtracker_*.c`.

Conventions: quaternions are w first (`q[0]` = w). 4x4 matrices are 16
floats. "Order" notes give the summation order where it changes the float
result; two-term sums commute exactly and are not noted.

### Testing method and libm

Sony's DLL imports `sinf`, `cosf`, `atan2f`, `expf`, `asinf`, `pow`, … from
the Universal CRT (`api-ms-win-crt-math`). The mingw libm the tests link
against differs from it by 1 ulp on about 0.15 % of `sinf`/`cosf` inputs
(none on `atan2f`). `irt_difftest.h` therefore routes the libm calls of the
code under test to `ucrtbase.dll`, so the tests check our arithmetic, not
libm accuracy. "bit-exact (ucrt libm)" means: identical bits given the same
libm. With glibc, results differ from Sony's by libm's error, amplified by
the formula (worst seen: 12 288 ulp in `axis_angle_rotate` before
cancellation, i.e. still ~1e-7 absolute for unit inputs).

Pitfall seen in practice: Ghidra prints `if (x <= k)` for a branch the CPU
takes when `!(x > k)`. The two differ for NaN. Tests feed NaN inputs, and
the code is written to the observed NaN behaviour.

### INS math helpers (`0x180170120–0x1801711f0`) — `psvr2_ins_math.c`

| VA | Ours | Behaviour |
|----|------|-----------|
| `0x180170120` | `quat_conj_inplace` | Multiplies x, y, z of a w-first quaternion by `-1.0f` (a multiply, not a sign flip). Returns its argument. |
| `0x180170160` | `pose_negate` | Pose = position[3] then quaternion[4]. Multiplies position and the quaternion's x, y, z by `-1.0f`. |
| `0x1801701c0` | `vec3_lerp` | `out = (1−t)·a + t·b`, per component. |
| `0x180170640`, `0x180170670` | `vec2_len`, `vec3_len` | Euclidean length, squares summed in index order. |
| `0x180170700` | `quat_normalize` | Divides all four components by the norm; the norm sums x², y², z² and then w². No zero guard. |
| `0x1801707c0` | `vec2_normalize` | Divide by length; a length below `1e-4f` is replaced by 1. |
| `0x180170840` | `vec3_normalize` | Divide by length; a length below `1e-10` (compared as double) is replaced by 1. |
| `0x1801708f0` | — | Debug print of a 4x4 matrix (`%.2f`, row by row). Skipped. |
| `0x180170a10` | `axis_angle_rotate` | Object holds an axis at floats 1..3. Builds `q = (cos(a/2), sin(a/2)·axis)` (axis not normalised) and returns `q ⊗ (0,v) ⊗ q*`, vector part. The conjugate's components are formed as `(−axis_i)·sin`. Both products use one fixed Hamilton-product evaluation order (see `qmul` comment). The zero scalar of `(0,v)` is really multiplied (so inf/NaN propagate). |
| `0x180170cb0` | `mat4_identity` | Writes the 4x4 identity. |
| `0x180170dc0` | `quat_slerp(a, out, t, b)` | dot = a·b (x, y, z, then w). If dot < 0, **b is negated in place** and dot flipped. Weights start as (1−t, t). Unless dot > 0.9999 (NaN takes the sine path): s = √(1−dot²), θ = atan2f(s, dot), weights become sin((1−t)θ)/s… computed as `sinf(w·θ)·(1/s)`. Output = a·wa + b·wb. |
| `0x180171010` | `mat4_transpose` | In-place transpose. |
| `0x1801711f0` | `gauss_merge` | Merges two weighted isotropic Gaussians (mean₃, σ, weight). W = wa+wb; mean = (a·wa + b·wb)/W; second moment = (‖b‖²·wb + ‖a‖²·wa)/W + (σa²·wa + σb²·wb)/W, squared norms summed z, y, x; variance = that − (my² + mz² + mx²). Fails (returns false, writes nothing) unless variance > 0; else writes mean through the first pointer of its context and σ = √variance. |

### INS filters (`0x180171c10–0x1801726a0`) — `psvr2_ins_filter.c`

Sony keeps these objects as **structs of pointers** to their state words
(each field is reached through a pointer stored in the object). Our structs
use the same layout so both implementations can run on one object; the
field names are ours.

#### Window filter `wf` (vec3 smoother with a settle lock)

State: estimate `est`, ring of the last ≤ 8 inputs, `fill`, `widx`,
`locked`, `settled` (byte), `status`, `miss`, `enabled` (byte), `cap`,
`init`, two per-axis gates (`gate_track`, `gate_lock`) and `alpha`.

- **init** (`0x1801726a0`, args: enabled, cap, alpha, gate_track*, half_gate_lock*, init*):
  `cap` is kept if 2..8, else 8. `gate_lock = 2·half` per axis (as `h+h`).
  Copies the rest, then does **reset**.
- **reset** (`0x180172040`): `est = init`; locked, fill, widx, miss, status,
  settled = 0.
- **mean** (`0x180172620`): if fill is 1..8, per-axis running sum of the
  ring in slot order, divided by `(float)fill`; otherwise zeros.
- **spread** (`0x180172490`): if fill is 2..8, per-axis max − min over the
  ring (max replaces when `max < x`, min when `x < min`, starting from slot
  0); otherwise 1000 on every axis.
- **update(out, in)** (`0x1801720a0`):
  1. Disabled: `out = in`, nothing else.
  2. Store `in` at `widx`; `widx` advances and wraps to 0 at `cap`; `fill`
     grows to at most `cap`.
  3. If `fill ≥ cap` and the spread passes `gate_lock`: `est = mean`,
     locked = 1, status = 3, miss = 0, settled = 1. Done.
  4. Else gate `in − est` with `gate_lock` if locked, else `gate_track`.
     Pass: `est = alpha·in + (1−alpha)·est`, status = 2, miss = 0.
  5. Fail: status = 1. If miss exceeds 32 (settled) or 8 (not settled):
     `est = mean`, locked = 0, miss = 0, settled = 0. Otherwise miss += 1.
  6. `out = est`.

  "Passes a gate" means no axis has `gate < |d|`; a NaN difference passes.

#### Stillness averager `still`

Config: per-axis noise variance `var` and sample `rate` (float, truncated
to int `R`). State: `phase`, `count`, `ref`, `acc`, `result`, and a `wf`.

- **reset** (`0x180171d00`): result = NaN (`0xffc00000`) ×3, phase = 0,
  count = 0, ref = 0, acc = 0.
- **step(avg_out, in)** (`0x180171d40`), returns true when an average is
  ready:
  - phase 0: the first sample becomes `ref`; count += 1; `acc += in`.
  - phase ≠ 0: if any axis has `|in − ref|` not below `4·√var` (NaN counts
    as moved): phase = 0, count = 1, ref = acc = in. Otherwise count += 1,
    `acc += in`.
  - Then, in phase 0, once `count ≥ R/10` (integer division): ref =
    acc/count, phase = 1, count = 0, acc = 0. In phase 1, once
    `count ≥ (int)(R·1.5f)`: `avg_out = acc/count`, **reset**, return true.
- **feed(in)** (`0x180171c10`): run step; when it yields an average, pass
  it through the `wf` and store the filtered value in `result`.
- **get** (`0x180171c80`): vec = result; a 3x3 covariance with diagonal
  `var_k / rate` (float rate), zeros elsewhere; flag = wf's `settled`.

Use: accelerometer/gyro bias averaging while the headset is still [likely].

### INS parameters and covariance helpers (`0x1801729d0–0x180173370`) — `psvr2_ins_cfg.c`

`0x180172f90` returns a lazily filled global parameter block (0x80 bytes;
its float at +0x10 is negative until filled, then 0.1). Values by offset:
+0x00 0.05, 0.025, 0.005, 0.0025; +0x10 0.1; +0x14 0.3; +0x18 50; +0x1c
limits −10, 10, −5, 5, −10, 10; +0x34 `cov_small` 1e-7, 1e-4, 1e-5; +0x40
`cov_far` 1e-4, 5e-4, 1e-4; +0x4c 0.1, 1, 0.01, 0.05; +0x5c `near_dt` 1;
+0x60 `far_dt` 10; +0x64 `cov_floor` 1e-8, 1e-6, 1e-6; +0x70 gravity 9.8;
+0x74 three times 0.05235988 (3°).

- `0x1801729d0`: closure check, true when every `|A[r][c] − B[c][r]| < tol`
  (NaN fails). `0x180172b90`: a 3x3 covariance is valid when its diagonal
  is ≥ 0 and it is symmetric within 1e-6.
- `0x180172aa0` `cov3_floor(out, M, f)`: copy M if all nine elements are
  above f, else `f·I` (off-diagonals are `f·0.0`, so they carry f's sign
  and NaN).
- `0x180172da0` / `0x180172d10` / `0x180172e30` = `cov3_for_gap(…, which =
  0/1/2)`: gap `d = |a − b|`. `d < near_dt`: `cov3_floor(M, cov_floor[w])`.
  Otherwise an isotropic `cov_far[w]·I` when `d` is not below `far_dt` (NaN
  counts as far), else `cov_small[w]·I`.
- `0x1801732b0` / `0x180173270` / `0x180173370` = `cov3_small(w)`:
  `(M₈ + M₄ + M₀)/3 < cov_small[w]` and M valid.
- `0x1801732f0`: `|a|² < 3·p₀²` and `|b|² < 3·p₂²` (p = parameter block,
  squares summed z, y, x).

Pattern confirmed by tests here too: where Ghidra shows `if (k <= x)` the
NaN case behaves as `!(x < k)`.

### INS core kernels — `psvr2_ins_core.c`

Test: `t_irtracker_inscore.c`, 0 mismatches on every kernel (200 000 cases
each, 10 000 for the strapdown). **Quaternions here are x, y, z, w** (w
last), as the INS stores them; a pose is position[3] + quaternion[4].

- **`0x1801805a0` quat_mul** `r = a ⊗ b` (Hamilton). Order per lane:
  `r.x = (a.w·b.x + a.y·b.z) + (b.w·a.x − a.z·b.y)`, same pattern for y, z;
  `r.w = −(a.y·b.y + a.z·b.z) + (b.w·a.w − a.x·b.x)`. Every quaternion
  product in the INS uses this one form.
- **`0x18017f2c0` rotinc(dt, w0, w1)**: `θ = (w0 + w1)·dt/2`. The
  small-angle branch is taken when `|θ|² < sqrt(2.2888184e-5)` (≈ 0.00478:
  Sony compares the *squared* angle with the square root of the
  constant): `s = 0.5 − |θ|²/48`, `c = 1 − |θ|²/8`; else `s = sin(|θ|/2)/|θ|`,
  `c = cos(|θ|/2)`. Result `(θ·s + (w0 × w1)·dt²/24, c)`, coning term first in
  each sum. `|θ|²` is summed `(y² + x²) + z²`.
- **`0x18017f8e0` strapdown(filter, out, dt, n, samples)**: the filter is
  a struct of pointers (`+0x00` pose, `+0x08` velocity, `+0x10` gyro bias,
  `+0x18` accel bias, `+0x28` gravity, `+0x30` previous accel, `+0x38`
  previous gyro, `+0x40` delta pose; all but `+0x00` and `+0x40` through
  two levels). Samples are 28 bytes: accel[3], gyro[3], time. With
  `f_prev` the stored previous accel:
  `Sa = f_prev/2 + Σ R(dq_k)·f_k` (last term halved),
  `SSa = f_prev·n/2 + Σ_{k<n−1} R(dq_k)·f_k·(n − k − 1)`,
  `dq_k = dq_{k−1} ⊗ rotinc(dt, w_{k−1}, w_k)` with `w_{−1}` the stored
  previous gyro. `dq` is normalised (`(x²+z²) + (y²+w²)`, skipped when 0);
  the new quaternion `q ⊗ dq` is **not**. `v' = ((rot + Sa·dt) + v) + T·g`,
  `p' = ((T·v + p) + R(q)·SSa·dt²) + (g/2)·T²`, `T = n·dt`, both rotations
  with the old `q`, `R(q)x` as `x + w·t + q×t`, `t = 2 q×x`. Output (92
  bytes): delta pose copied, p', q', v', last `f − b_a`, last `w − b_g`.
  `n = 0`: `dq = (0,0,0,1)`, increments 0, previous samples kept.
- **`0x18017f500` predict(filter, dt, out)**: `dt = min(dt, 1)`; if
  `dt > 0`: `p += v·dt`, `q = q ⊗ rotinc(dt, w, w)` with the stored previous
  gyro. Then `out = delta ∘ (p, q)` (`delta.q ⊗ q`, `R(delta.q)·p + delta.p`).
  Failing `pose_ok` → identity, returns 0. Callers cap dt at INS `+0x53c`
  (0.05 s, `FUN_18017c370`).
- **`0x18017bac0` pose_ok**: no NaN in the 7 floats and
  `(x²+z²) + (y²+w²)` in `[0.9, 1.1]`.
- **`0x180181a50` blend1(blender, out, out_vel, target, target_vel, dt)**:
  target with a NaN (`FUN_180181690`) → nothing. Snap (copy target) if any
  `|Δp_i| > gate_p` (blender `+0xa0`) or `cos(gate_r/2) > |(target.q ⊗
  out.q⁻¹).w|` (`+0xac`; `q⁻¹ = (−x,−y,−z,w)/|q|²`). Else `α = min(4·dt, 1)`:
  `p += Δp·α`; slerp with `d = (t.x·o.x + t.z·o.z) + (t.y·o.y + t.w·o.w)`,
  weights `sin((1−α)θ)/sinθ`, `sin(αθ)/sinθ` for `|d| < 0.9999999` (else
  `1−α`, `α`), `α` negated when `d < 0`; no renormalisation. Velocity is
  copied from the target in both branches.
- **`0x1801733f0` imucal_mat(out, {R, _, s}, off)**: row i = `R[3i..3i+2]·s_i`,
  bias row `j = ((s1·R[3+j])·o1 + (s2·R[6+j])·o2) + (R[j]·s0)·o0`, last
  column (0,0,0,1). Applied per sample (`FUN_180174410`) as
  `a_j = 1·bias_j + ((x·m[j] + y·m[4+j]) + z·m[8+j])` on `(float)s16`.

IMU calibration records (`FUN_180173fe0`, built when both are present and
the type 13 generation changed):

| Record | Offset | Use |
|--------|--------|-----|
| type 12 | `+0x00..+0x08` | accel misalignment c0, c1, c2 → `R = k·[1,0,0; c0,1,0; c1,c2,1]` |
| type 12 | `+0x0c..+0x14` | accel scale per axis |
| type 12 | `+0x18..+0x20` | accel offset (used negated) |
| type 12 | `+0x24..+0x38` | gyro misalignment → `R = [1,m30,m34; m24,1,m38; m28,m2c,1]` (by offset) |
| type 12 | `+0x3c..+0x44` | gyro scale per axis |
| type 12 | `+0x74` | accel overall scale k |
| type 13 | `+0x28..+0x30` | gyro offset (used negated) |
