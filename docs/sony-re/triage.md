# Function triage: which Sony functions matter for the Linux driver

Every function in [`scripts/sony-re/inventory.tsv`](../../scripts/sony-re/inventory.tsv)
(5175 functions, Sony `driver_playstation_vr2_orig.dll`, PE timestamp
1741940190) is given one verdict in
[`scripts/sony-re/triage.tsv`](../../scripts/sony-re/triage.tsv):

| Verdict | Meaning | Functions | Decompiled lines |
|---------|---------|----------:|-----------------:|
| **core** | Behaviour the Linux driver needs: already ported, or still to port. | 2245 | ~189 k |
| **ref** | Worth reading to understand the device or Sony's design, or an optional feature (relocaliser). Nothing to port now. | 954 | ~90 k |
| **fluff** | Library code, Windows glue, Sony's companion-app UI, VrTrace debug capture, initialisers. Skip. | 1976 | ~105 k |

Columns of `triage.tsv`: `va`, `lines` (Ghidra line count from the
inventory), `subsystem` (inventory label), `verdict`, `area` (what part of
the driver it belongs to), `state` (the per-subsystem status sheet value:
`bit-exact`, `spec-only`, `tolerance`, `skip:*`, `todo`), `docs` (which
`docs/*.md` already name it), `reason` (one line, own words).

## 1. How the verdicts were made

The inventory's `subsystem` column is by address range, and several ranges
are mislabelled (section 5). So the verdicts come from IDA (9.5, the
`bin2` database) instead:

1. **Features per function** (`scripts/sony-re/triage/q_features.py`):
   referenced strings, imports, callers, callees (including function
   pointers taken), data references and the owning RTTI vtable.
2. **Seeds**, strongest first:
   - manual verdicts from reading the decompile
     (`triage/overrides.tsv`, about 60 entries and ranges);
   - the status sheets: `skip:*` is fluff; done entries
     (`bit-exact`/`spec-only`/`tolerance`) are core, except in
     `trace.tsv`, where only the room/chaperone glue is core and the
     VrTrace format parsers are ref;
   - the RTTI class of the vtable a function sits in
     (`triage/classes.tsv`, e.g. `CaesarUsbThread*`, `LedSyncer`,
     `UkfManager` core; `VrTrace*`, `Json::*`, `Framework::*` fluff;
     `Relocalizer*` ref);
   - library ranges and jsoncpp/STL error strings: fluff;
   - device I/O imports (`WinUsb_*`, `HidD_*`, `ReadFile`/`WriteFile`,
     `DeviceIoControl`): core.
3. **Propagation down the call graph**: a helper takes the best verdict
   of its callers (core > ref > fluff). Library, initialiser and OS-wrapper
   code does not pass its verdict on, since it calls everything.
4. **Leftovers**: a wrapper of core/ref code inherits upwards; a function
   between two neighbours of one verdict in the same object file (MSVC
   lays functions out per `.obj`) takes theirs; vtable-only methods in the
   tracker ranges are core, in the relocaliser ref; tiny stubs and STL
   instantiations are fluff. The last 50 or so were read by hand.

Regenerate (needs the local IDA database and decompile, which stay out of
the repo): run `q_features.py` through `ida_run.py` to get
`features.jsonl`, then `classify.py > triage.tsv`. Paths at the top of
`classify.py` point at the local workspace.

Limits: the verdict is per function, the area label is the first area that
reached it, so a shared helper can carry the "wrong" area. A core verdict
says the code is needed, not that the Linux driver lacks it; section 3 says
what is still missing.

## 2. By area

| Area | Verdict | Functions | Lines | Done in `userspace/lib/sie` | What it is | Linux driver today |
|------|---------|----------:|------:|----:|-----------|-------------------|
| ctrlboot | core | 807 | 80 k | 42 | Controller boot: multi-camera pose search, RANSAC/P3P, LM pose solve, robust losses, background model, connected components | Ported in large parts (`optical_pose.cpp`, `sie/ctrlboot`): RANSAC boot, P3P, sampler, MSAC, few-LED mode |
| tracking | core | 379 | 32 k | 0 | IR tracker thread stages, UKF manager, TrackerManager, INS feed, occlusion queries | UKF in simulation (`controller_ukf.cpp`); IR-thread stages in section 3.2 not described |
| sense | core | 196 | 18 k | 0 | Controller manager over libpad: open/close, per-frame poll, input, vibration mode | `sense_hid.cpp`, `controller_device.cpp` |
| headset-usb | core | 205 | 9 k | 0 | `CaesarManager` and the per-interface `CaesarUsbThread*` readers: enumeration, alt settings, control reports, init order, restart | Kernel module: init sequence, `0xde` clock map, firmware gate, see-through, quit/restart, per-interface recovery |
| libpad | core | 143 | 14 k | 73 | HID transport, `HID_SYNC`, calibration, output reports, clock fit | `sense_hid.cpp`, `sie/libpad` |
| steamvr | core | 200 | 14 k | 38 | `ServerDriver`, `HmdDevice`, `SenseDevice`, display, lens, hidden area, skeleton, properties | `driver_psvr2` (see `steamvr.md` "Project driver vs Sony") |
| irtracker | core | 104 | 7 k | 41 | Headset INS helpers (quaternion, filters) | `psvr2_ins.c` (strapdown + mode-1 blender) |
| room | core | 110 | 6 k | 14 | Map-state singleton, universe/chaperone file, default play area | Own chaperone file in `hmd_device_driver.cpp` (`provide_chaperone`) |
| io | core | 30 | 5 k | 0 | Low-level HID/USB I/O wrappers (libpad device threads, overlapped reads) | Replaced by hidraw / usbfs |
| ctrltracker | core | 66 | 4 k | 36 | UKF kernels, process noise, quaternion maths | `sie/ctrltracker`, `controller_ukf.cpp` |
| ledsync | core | 4 | 0.1 k | 0 | Few helpers; the LedSyncer itself is under tracking/steamvr | `led_syncer.cpp` |
| reloc | ref | 873 | 84 k | 0 | Relocaliser: keyframes, pose graph, map DB, RP frame unpack | Not done (optional, `DEV-NOTES.md`); RP data exposed on `/dev/psvr2-rp` |
| config | ref | 35 | 2 k | 0 | `ConfigManager`: `config.ini`, change notifications | Settings in `steamvr.vrsettings` instead |
| ipc | ref | 12 | 1 k | 0 | `ShareManager`: `SHARE_VRT2_*` shared memory with Sony's apps | `toolkit_ipc.cpp` (own protocol) |
| status | ref | 5 | 0.7 k | 0 | Status-info dispatcher (mode, proximity, thermal warning) to UI prompts | Field meanings only |
| firmware | ref | 6 | 0.7 k | 0 | `HMD2_FIRMWARE.CUP` version check beside the app | Not needed |
| trace-format | ref | 9 | 0.8 k | 9 | VrTrace parsers that document wire formats (image header V0200, `ST` frame TOC) | Formats in `trace.md` §4 |
| lib | fluff | 1346 | 73 k | – | CRT, STL, jsoncpp, dense linear algebra, Eigen, OpenCV-derived SVD/Rodrigues, thunks, CRT startup | Use libc / Eigen / OpenCV |
| debug | fluff | 336 | 14 k | 316 | VrTrace capture tool, IR recording writer, debug status text | – |
| trace | fluff | 105 | 9 k | – | Remaining VrTrace helpers | – |
| app | fluff | 74 | 6 k | – | `VrDialogManager` (dialogs, desktop/see-through guides, safety notice, locale), companion-app launch and Authenticode checks | – |
| init | fluff | 81 | 4 k | – | Static initialisers (`0x180001000–0x180008160`) | – |

## 3. What is still to implement or describe

### 3.1 Known gaps, checked against the code (2026-11-21)

Most gaps listed in the older docs are closed (init sequence, `0xde` clock
sync, firmware gate, see-through, quit/restart, EDID switch, hidden area,
driver-from-head, chaperone file, INS, LED sync, UKF). What is still open:

| Item | Sony functions | Verdict | Notes |
|------|----------------|---------|-------|
| Relocaliser + Set `0x0a` | thread `0x1801ac4e0`, start `0x1801abf90`, RP unpack `0x1801ab410`, run/update/optimise `0x1801df010` / `0x1801df630` / `0x1801ded40`, map load `0x1801ac1f0`, backend `0x1801e0000–0x180238880` | ref | Only needed for a play area that survives restarts. 873 functions, the largest optional block. |
| Map persistence (`sceBoundaryMeta.bin`, map id) | `0x18011fd30`, `0x180120b40`, `0x180120c80`, `0x180120f20`, `0x1801214a0` | core / ref | Tied to the relocaliser; the own chaperone file covers the non-persistent case. |
| INS mode-0 adaptive blender, stillness / shock switches, SLAM good-frame count | blender `0x180181400` and its callers (tracking.md §3.1) | core | `psvr2_ins.h` says only mode 1 is done; mode-0 gains are unknown. |
| Extreme-environment blob modes 2–4 | ledsync / ctrlboot (`notdone.md`) | core | Need tracker settings that are 0 in a normal install. |
| Hardware checks | UKF, two-controller sync, BROAD divider | core | Implemented, not yet confirmed on hardware (`DEV-NOTES.md`). |

### 3.2 Core functions no doc describes yet

143 core functions of 150+ lines (49 k lines) are not named in any
`docs/*.md` and have no status entry beyond `todo`. Many in ctrlboot are
probably covered by recent ports without being named; the rest are the
places to read next. Full list: filter `triage.tsv` on `verdict=core`,
empty `docs`, `lines>=150`. By area: ctrlboot 79, tracking 20, libpad 13,
sense 10, irtracker 6, steamvr 4, io 4, ctrltracker 3, room 3,
headset-usb 1. The largest:

| Function | Lines | Called from | Guess |
|----------|------:|-------------|-------|
| `0x1801b37b0` | 2525 | IR thread `0x18018aaf0` | IR-thread stage, float min/max heavy: blob or LED-model processing [guess] |
| `0x1801b1280` | 1619 | IR thread | same family [guess] |
| `0x1801910f0` | 950 | IR thread | same family [guess] |
| `0x1801b81c0` | 587 | IR thread | same family [guess] |
| `0x1801ba190` | 637 | libpad `0x1801bf060` | libpad-side tracking helper [guess] |
| `0x1801c58d0` | 723 | libpad `0x1801bf550` | controller report processing [guess] |
| `0x18005c7f0`, `0x18005c130`, `0x1800641d0` | 529, 457, 441 | UKF snapshot code `0x18005e5f0–0x18005e6e0` | UKF snapshot save/restore [likely] |
| `0x1801780e0` | 506 | `0x1801792a0` | INS status / version log ("%dDoF") [likely] |
| `0x18006f020`, `0x180068ee0`, `0x18006dbd0` | 594, 443, 429 | ctrltracker | UKF steps not yet in `ukf.md` |
| `0x1800b8b00`, `0x1800b43d0`, `0x1800b3090`, `0x1800b53c0`, `0x1800a88f0` | 780–822 | ctrlboot `0x1800b40e0`, `0x1800a37f0` | boot search stages |
| `0x1800ea4c0` | 411 | ctrlboot | connected components (blob labelling) |

Read the four IR-thread stages first: they run per LD frame between the
blob packet and the pose solve, which is the part the Linux optical
tracker most likely still approximates.

## 4. What is fluff, and why

- **Libraries (1346).** MSVC CRT/STL, jsoncpp, the BLAS/LAPACK-style
  linear algebra (`0x180070000–0x18009ffff`), Eigen integrators,
  OpenCV-derived SVD / Jacobi / Rodrigues / fisheye projection, import
  thunks, CRT startup and exception funclets. Linux uses its own libraries.
  The camera model those projection helpers implement is already written
  up in `notdone.md` §1.
- **VrTrace (about 440).** Sony's capture tool. On PC nothing feeds its
  rings (`trace.md` §1). Only the format notes are kept (ref).
- **Companion-app UI (74).** `VrDialogManager`: dialogs, desktop guide,
  see-through guide, safety notice records, locale matching, launching and
  Authenticode-checking `PlayStationVR2App.exe`, `psvr2_dialog.exe`,
  `psvr2_overlay.exe`. The status-info dispatcher and tracking-status
  handler in this range are kept as ref, because they show what the
  status fields mean.
- **Static initialisers (81).** Global constructors, std::string
  constants, localised installer messages. A few seed tunables (for
  example the `ImuRingBuffer` defaults); those values are in the
  subsystem docs.
- **Debug output.** IR recording file writer (`0x1801adf50–0x1801ae260`),
  the debug status text (`0x180185ef0`).

## 5. Inventory label corrections (applied to `inventory.tsv` 2026-11-21)

| Range | Inventory says | Actually |
|-------|----------------|----------|
| `0x180001000–0x180008160` | `ui` | static initialisers |
| `0x180100140–0x180109fb0` | `debug:trace` | ctrlboot solver and robust losses |
| `0x18011b3c0–0x18011d6f0` | `debug:trace` | controller manager (`sense`) |
| `0x18011e000–0x18011ffff` | `debug:trace` | map state, chaperone, default play area (`room`) |
| `0x18012d900–0x1801354ff` | `map+steamvr` | `VrDialogManager` (companion-app UI) |
| `0x180154870–0x18015ffa0` | `lib:json+config` | `ConfigManager` / `ShareManager` glue, not jsoncpp |
| `0x1801d6230–0x1801dfb00` | `libpad` | relocaliser kernels and object API |
| `0x180238f00–0x18023ffff` | `reloc` | CRT startup, import thunks, unwind funclets |

The IDA database knows 9166 functions; the 5175 in the inventory are
Ghidra's boundaries. 19 inventory entries have no IDA function at
that address (IDA merged them into a neighbour); they take the verdict of
their range.
