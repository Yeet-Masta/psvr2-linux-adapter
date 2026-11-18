# Room map, relocalisation and play area (`room`)

Sony PSVR2 SteamVR driver `driver_playstation_vr2_orig.dll`, PE timestamp
1741940190, image base `0x180000000`. Written from the decompile in our own
words. Earlier notes this builds on, and does not repeat:

- [`reloc.md`](reloc.md): RP frame unpack (`FUN_1801ab410`) and the full
  `sceMapDb.bin` layout (header, SHA-256, keyframe / edge tables).
- [`trace.md` §10](trace.md): universe / chaperone helpers in
  `0x18011e000–0x18011ffff` (default play area, chaperone JSON builder,
  `sceBoundaryMeta.bin` loader, map-state singleton fields).

Tags: **[sure]** read in code, **[likely]** strong inference, **[guess]**.

## 1. Driver summary

The room map is **optional for tracking**. The headset runs SLAM by
itself; the PC-side relocaliser only adds a persistent map, so that a
saved play area comes back in the same place after a restart. The Linux
driver today drains the RP endpoint and discards it, and tracking works.
What the Linux driver needs, in priority order:

1. **Play area in SteamVR (userspace, needed now).** Publish
   `Prop_DriverProvidedChaperonePath_String` (2048) =
   `<Prop_UserConfigPath_String>\chaperone_info.vrchap` and
   `Prop_CurrentUniverseId_Uint64` (2004) = the `universeID` in that file.
   Write the file in SteamVR's `chaperone_info` JSON format (version 5).
   With no saved room, make a default area: a 24-point circle of radius
   1.5 m around the headset (§5.3). This is all SteamVR needs for bounds.
   Bounds never go to the headset.
2. **Keep the RP endpoint drained (kernel, done).** EP `0x8a` (interface 9)
   must be read or the tracker stalls (`kernel/psvr2_aux.c`).
3. **Relocaliser feedback (userspace, optional, large).** For a persistent
   room, the host must run a relocaliser on each RP record and send the
   result back with **Set report `0x0a`** (§2.3, §3). Without this, each
   session's SLAM origin is wherever the headset started, and a saved play
   area cannot be re-anchored. Implementing it means reimplementing Sony's
   keyframe / pose-graph relocaliser (`0x1801e0000–0x18023ffff`), or
   writing an open one that consumes the RP features. It is not needed
   for a working driver.
4. **Persistence files (userspace, only with item 3).** `sceMapDb.bin`
   (map) and `sceBoundaryMeta.bin` (which map id the play area belongs
   to). Save on quit, every 60 s while worn, and after every play-area
   edit (§3.4).

Kernel: nothing beyond item 2, plus a way for userspace to send vendor
control Set `0x0a` (the generic control-transfer path already covers it).
Everything else is userspace.

## 2. Wire and data formats

### 2.1 RP record header (EP `0x8a`, 0xc8780 bytes)

Main doc and `reloc.md` give the magic and the frame unpack. Extra facts:

| Off | Size | Meaning |
|---|---|---|
| 0x00 | 2 | `"RP"` |
| 0x04 | 4 | u32 size 0xc8780 |
| 0x08 | 8 × u32 | **Command slots from the headset** [sure: loop in `FUN_1801ac4e0`]. Each slot is processed in order: 1 = process this frame, 2 = initialise map, 3 = optimise map, 5 = load map from disk. Other values are ignored. |
| 0x2c.. | | RP frame body (`reloc.md`) |
| 0x9afb4 | i32 | (frame 0x9af88) **Map id the headset currently tracks against**, −1 = none ("map latch") [sure: read as `DAT_180597018`; meaning likely] |
| 0x9afb8 | u32 | (frame 0x9af8c) headset tracking sub-state; 5 gates the coverage counter (§5.2) [sure: test; meaning guess] |

The host keeps a u16 sequence number per received record (USB thread
counter, not a record field). The relocaliser thread waits up to 100 ms
for it to change (`FUN_180128600`), copies the record (`FUN_180125000`),
and does nothing if the number is unchanged [sure].

The map id low 16 bits are dropped before comparison (`id & 0xffff0000`,
`FUN_1801216d0`), which matches the `map_id` format in `reloc.md`
(`(seed + n) << 16`, low bits bumped on edits) [sure].

### 2.2 Set `0x0a` "setRelocPreOutput" (host → headset)

Vendor control Set (`FUN_1801283f0`): header `u8 0x0a, u8 0, u16 subcmd,
u16 len = 0xd8, u16 0`, then **always 0xd8 data bytes**, whatever the
subcmd [sure: `FUN_180127fe0`]. Only sent while the device state is 2
(open); otherwise it returns `0xfffffcf7` [sure].

| subcmd | Payload (first bytes; rest of 0xd8 is whatever the static buffer holds, zero in practice) | When |
|---|---|---|
| 4 | u32 `1` | Relocaliser (re)start, before init/load (§3.1) [sure] |
| 3 | 8 × 4 bytes: u32 `0x20`, u32 `0xb4` (180), u32 `10`, u32 `4`, f32 `0.6`, f32 `0.0625`, f32 `100.0`, u32 `12` | Once, after the first processed frame following a (re)start [sure: values; meaning unknown] |
| 1 | Localiser result, §2.3 | After every processed RP frame [sure] |

Pacing [sure, `FUN_1801adba0`]: at least 20 ms between two `0x0a` sends
(sleeps the remainder). A send that returns ≤ 0 logs "busy", sleeps 1 ms
and retries, up to 5 retries, then logs "timeout" and drops it.

### 2.3 Localiser result (subcmd 1 payload, 0xd8 bytes)

Filled by RunLocalizer (`FUN_1801df010`), UpdateMap (`FUN_1801df630`)
and the thread (`FUN_1801ac4e0`). u32 words unless noted.

| Off | Type | Meaning |
|---|---|---|
| 0x00 | u32 | RP frame word 0 (frame id), echoed [sure] |
| 0x04 | u8 (+3 pad) | 1 = relocalised against the map this frame [sure: flag; cleared again if the covariance step fails] |
| 0x08 | i32 | Id from the backend, −1 = none; matched keyframe or map id [likely] |
| 0x0c | 12 × f32 | Relocalised pose: rotation 3×3 + translation 3 [likely]; valid only when +0x04 = 1 |
| 0x3c | 36 × f32 | 6×6 matrix derived from the pose covariance (`FUN_1801d6230`) [likely] |
| 0xcc | u32 | Backend value (`FUN_1801df010`) [sure: copied; meaning unknown] |
| 0xd0 | u32 | 1 = map has a current anchor keyframe ("anchored") [sure: from `FUN_1801eea80`] |
| 0xd4 | u32 | Flags, masked with `0x37f` before send. UpdateMap zeroes it; the thread sets bit 6 (`0x40`) when the map has fewer than 16 keyframes and clears it otherwise; bits 8–9 are cleared [sure] |

### 2.4 `sceBoundaryMeta.bin` (0x48 bytes) [sure]

Path: `<Prop_UserConfigPath_String (6000) of the HMD>\sceBoundaryMeta.bin`.
Writer `FUN_180120f20`, reader `FUN_18011fd30`, remover `FUN_180120b40`.

| Off | Type | Meaning |
|---|---|---|
| 0x00 | u16 | magic `0x4d42` ("BM") |
| 0x02 | u16 | version 1 (reader accepts < 2) |
| 0x04 | u32 | size `0x48` |
| 0x08 | i32 | Map id the play area belongs to (−1 = none) |
| 0x10 | u64 | Universe id of the chaperone written for that map |
| 0x18 | u64 | Write counter (incremented on each write) |
| 0x20 | u64 | 0 |
| 0x28 | u8 | Play-area flag copied from the area block `+0x830` (2 by default) [meaning guess] |
| 0x29 | 31 | 0 |

On load, the universe id is taken only if the last `sceMapDb.bin` load
succeeded; otherwise it becomes 0 (no valid play area) [sure].
The file is written only when a dirty flag (map-state `+0x60`) is set,
i.e. after a play-area change [sure].

### 2.5 `chaperone_info.vrchap` [sure]

Standard SteamVR JSON, one universe; builder documented in `trace.md` §10
(`FUN_18011e8e0`, wall height 2.43 m, `version` 5, `jsonid`
`"chaperone_info"`, `time` = `asctime()` string). The universe id is
incremented (skipping 0) on each write, then property 2004 is set to it
(`FUN_180121200`). The reader (`FUN_180120570`) fills the 0x850-byte area
block from `collision_bounds[i][0]` (x, z of each wall's first corner),
`play_area[0..1]`, and `seated`/`standing` translation + yaw.

### 2.6 `sceMapDb.bin`

Format in `reloc.md`. Path: `<Prop_UserConfigPath_String>\sceMapDb.bin`.

### 2.7 `playarea_data.bin`

VrTrace capture stream only (`FUN_18010cdd0`, `trace.md` §4.2). Not part
of normal operation; skip.

## 3. Sequences

### 3.1 Start (`FUN_1801abf90` → `FUN_1801aaf00` → `FUN_1801ad9d0`)

1. Read `vrtracker2.ini` from the current directory; section
   `[Relocalizer]`, key `loadMap=<int>`; default 1 [sure]. Nonzero means
   "load the saved map at start".
2. Create the relocaliser with a seed = `MMDDhhmmss` of local time
   (`FUN_1801debe0`), start `RelocalizerThread` [sure].
3. Configure: parameters {−1.0, 1000.0, 0, 0x100, 4, 0x18, 1, 1,
   0.9397 (= cos 20°), 0.25} (`FUN_1801df4c0`) and mode 2
   (`FUN_1801df410`) [sure: values; meaning unknown].
4. Send Set `0x0a` subcmd 4 `{1}`; queue "send subcmd 3 after the next
   frame" [sure].
5. If `loadMap` is 0: initialise an empty map. Otherwise: load
   `sceMapDb.bin` (§3.3) [sure].

The same 3–5 run again on command 6, which the USB code queues each time
the control interface alternate setting is (re)applied (`FUN_1801256c0`,
i.e. headset (re)connect) [sure].

### 3.2 Per RP record (`FUN_1801ac4e0`)

1. Wait for a new record (≤ 100 ms). Copy it.
2. Run headset command slots (§2.1).
3. Run host-queued commands (§4.1).
4. If slot command 1 was present:
   1. Unpack the RP frame.
   2. Log "Map latched" / "unlatched" when the record's map id changes
      between −1 and not −1.
   3. Tell the map state the current map id (−1 if tracking is not OK)
      (`FUN_1801216d0`, §4.2).
   4. RunLocalizer → result; UpdateMap → "map changed" flag.
   5. Every frame count ≥ 50 with any map change since the last
      optimise: OptimizeMap, reset the count [sure].
   6. If anchored and an "unremovable keyframes" request is pending: mark
      the current keyframes unremovable (`FUN_1801df5a0`).
   7. Set `0x0a` subcmd 1 with the result; then subcmd 3 if queued.
   8. Update the 3 s anchoring timer and push status to the companion app
      (§5.2).

### 3.3 Load map (`FUN_1801ac1f0`)

1. Reset the relocaliser.
2. Load `<config>\sceMapDb.bin` (`FUN_1801decf0`). On failure, mark the
   map state "map initialised" (no map) and log.
3. Set coverage count to 16 (full) [sure].
4. Remember load success (`+0x70`), then `FUN_180120a80`: try to load
   `sceBoundaryMeta.bin`; if that fails, reset the play-area state (map
   id −1, no universe) and rewrite the meta file [sure].

### 3.4 When maps are saved, loaded and discarded

| Trigger | Action | Source |
|---|---|---|
| Driver start / headset reconnect, `loadMap ≠ 0` | Load map + meta | `FUN_1801ad9d0` |
| Headset RP slot command 5 | Load map + meta | `FUN_1801ac4e0` |
| Headset RP slot command 2, or app "initialise map" | Empty map, coverage reset | `FUN_1801ac140` |
| SteamVR event 0x2bf (703, `VREvent_QuitAcknowledged` [likely]) | Write meta, save map ("Quited Save Map") | `FUN_180120290` |
| Every 60 s while the proximity sensor says worn; the timer restarts while not worn | Write meta, save map ("Routine Save Map") | `FUN_180132890` |
| Play area set / changed / removed | Write chaperone + meta, save map | `FUN_1801211a0` callers |
| App "remove play area" | Delete `sceMapDb.bin`, `sceBoundaryMeta.bin`; set universe property; initialise empty map; save | `FUN_1801a3e50`, `FUN_180120c80` |

Save is queued to the relocaliser thread (command 2) and written by the
backend in one write of the whole image (`reloc.md`) [sure].

## 4. State machines

### 4.1 Host relocaliser command queue (`param + 0x64040`)

Producers are the companion-app callbacks (§4.3) and the map state.

| Cmd | Name | Action |
|---|---|---|
| 0 | initializeMap | Reset relocaliser; lock state = 1; coverage = 0; map state "reset" + "map initialised" |
| 1 | loadMap | §3.3 |
| 2 | saveMap | Save `sceMapDb.bin` |
| 3 | removeMap | Delete `sceMapDb.bin` |
| 4 | ActivateMap | If lock state = 1, lock the current keyframes (`FUN_1801df440`); lock state = 0 |
| 5 | RefineMap | Lock state = 2; coverage = 0 |
| 6 | restart | §3.1 steps 3–5 |
| 7 | — | After this frame, mark keyframes unremovable (if anchored) |

### 4.2 Play-area state (map-state singleton `+0x68`, `FUN_1801216d0`)

Inputs: current map id `m` (from the RP record, −1 when not tracking),
stored play-area map id `P` (`+0x50`), initialised map id `I` (`+0x6c`),
and whether the stored universe id equals the current SteamVR universe
(`U`).

| Value | Meaning |
|---|---|
| 0 | Play area valid and map latched (U, and `m` matches `P`) |
| 1 | Play area exists, map not latched (U, `m` = −1) |
| 2 | No valid play area for this universe, map latched |
| 3 | No valid play area, map not latched |

Rules [sure]: `m` = −1 → 1 (if U) else 3. `m` new (≠ `P` and ≠ `I`) →
`P = m`, state 0 / 2. `m` = `P` → 0 / 2. Otherwise no change. A new
play area is accepted only in states 0 or 2 (`(state & ~2) == 0`,
`FUN_180121830`); accepting it stores `U` and forces state 0.

The status page (`FUN_180134800`) maps 0/1/2/3 to the strings
"(playarea: 0, map latch: 1)" etc., and treats state 0 rising after a
"need unremovable" arm as the trigger to queue command 7 after 200 ms.

### 4.3 Companion-app callbacks (`FUN_1801a2040` registration)

The PlayStation VR2 App talks to the driver through named shared memory
and events (`SHARE_VRT2_WIN_*`, `FUN_18015bbd0`). The driver registers
one handler per event bit [sure: RTTI-mapped]:

| Bit | Handler | Effect |
|---|---|---|
| 0x01 | `FUN_1801a40b0` | initializeMap |
| 0x02 | `FUN_1801a3f30` | RefineMap (cmd 5) |
| 0x04 | `FUN_1801a3ed0` | ActivateMap (cmd 4) |
| 0x08 | `FUN_1801a4180` | loadMap |
| 0x10 | `FUN_1801a41a0` | saveMap |
| 0x20 | `FUN_1801a3e50` | RemovePlayArea |
| 0x40 | `FUN_1801a40d0` | PlayAreaChanged: read the area drawn in the app from shared memory, apply (§4.2), save |
| 0x80 | `FUN_1801a4040` | InitializeMapAndSetDefaultPlayArea: initialise map and arm a *delayed default play area* |
| 0x100 | `FUN_1801a3f90` | SetDefaultPlayArea now (circle around the headset) |

Delayed default play area (`FUN_180120290`, each frame): when armed and
the map id has changed from the id at arming time and is not −1, build
the default area at the current headset pose, apply it, write meta and
save the map ("Delay Set Default PlayArea") [sure].

Room setup flow, as a Linux equivalent needs it: initialise map → user
walks around (coverage, §5.2) → ActivateMap → app sends the drawn
boundary (0x40) → saved. A Linux driver without the app can replace all
of this with: default area on first run, or an area from a config file.

## 5. Algorithms

### 5.1 Relocaliser (summary only)

Keyframe map (≤ 97 keyframes, ≤ 256 features each, 128-byte descriptors,
pose-graph edges with 6×6 information matrices; layout in `reloc.md`).
RunLocalizer matches the RP frame's features against the map and returns
a pose + covariance; UpdateMap inserts keyframes / edges; OptimizeMap runs
the pose graph. Keyframes can be locked (ActivateMap) or made
unremovable so that the saved room stays stable. A Linux driver only
needs this to reuse Sony maps; none of it is needed for tracking.

### 5.2 Coverage and anchoring status (UI only) [likely]

When the result is relocalised and the sub-state is 5, the yaw of the
relocalised pose in degrees (+360) sets one bit in each of four bin
masks: 4 × 90°, 4 × 90° offset by 45°, 12 × 30°, 30 × 12°. The count of
newly set bits (first three masks) is the "map registration" progress
sent to the app; a load sets it to 16. A 3 s timer (3,000,000 µs) starts
when tracking is OK but the map has not been anchored for 3 s; while it
runs, the app is told registration is lost. Not needed by Linux.

### 5.3 Play area to SteamVR

Absolute bounds (`FUN_1801214a0`) [sure]: rotate each area point `(x,z)`
by the standing yaw (half-angle quaternion about +Y, angle = −yaw) and
offset by the standing `(−x, −z)`. The result is kept in the map state
for inside/outside tests (`FUN_18011e6b0`) and the safety dialog. SteamVR
gets bounds only through the chaperone file plus property 2004.

Default area: `trace.md` §10 (`FUN_18011ef90`). Head-height assumption
1.5 m: standing origin = head position minus 1.5 m in y [sure].

## 6. Constants

| Name | Value | Meaning | VA |
|---|---|---|---|
| RP wait | 100 ms | Wait for a new RP record | `FUN_1801ac4e0` |
| Optimise period | 50 frames | With any map change | `FUN_1801ac4e0` |
| `0x0a` spacing | 20 ms | Minimum gap between sends | `FUN_1801adba0` |
| `0x0a` retries | 5 × 1 ms | Then drop | `FUN_1801adba0` |
| Few-keyframes | < 16 | Sets result flag `0x40` | `FUN_1801ac4e0` |
| Anchor timeout | 3,000,000 µs | Registration lost | `FUN_1801ac4e0` |
| Routine save | 60,000,000 µs | While worn | `FUN_180132890` |
| Unremovable delay | 200,000 µs | After state → 0 | `FUN_180134800` |
| Default radius | 1.5 m, 24 points | Default play area | `FUN_18011ef90` |
| Wall height | 2.43 m | Chaperone quads | `FUN_18011e8e0` |
| `loadMap` | 1 | `vrtracker2.ini [Relocalizer]` | `FUN_1801abf90` |
| Reloc params | −1, 1000, 0, 0x100, 4, 0x18, 1, 1, cos 20°, 0.25 | Backend config | `FUN_1801ad9d0` |
| subcmd 3 block | 0x20, 180, 10, 4, 0.6, 0.0625, 100, 12 | Sent to headset | `FUN_1801ac4e0` |

## 7. Gaps in the current Linux driver

- `kernel/psvr2_aux.c` drains interface 9 (RP) and discards it. Fine for
  tracking; nothing is exposed to userspace, so no relocaliser could run.
- No Set `0x0a` is ever sent. Sony sends subcmd 4 at start and subcmd 1
  ~once per RP frame. Unknown whether the headset behaves differently
  without them (it tracks today, so not fatal). [hardware check]
- `userspace/lib/sie/steamvr/psvr2_hmd_props.c` sets property 2048 to the
  Sony path, and `psvr2_sense.c` sets 2004 to the constant 2. Nothing
  writes `chaperone_info.vrchap`, so universe id 2 has no matching file
  unless Sony's driver wrote one earlier. Fix: on activate, read the file
  (if any) and use its `universeID`; else write a default area
  (`psvr2_chap_default_area` exists) with the JSON format above and set
  2004 to that id.
- `userspace/lib/sie/reloc/` has the RP unpack and map-DB load/save
  helpers, unused by the driver. No relocaliser thread, no meta file, no
  save scheduling.

## 8. Open questions

1. Does the headset need Set `0x0a` subcmd 4 / 1 for anything (e.g.
   accepting map-relative poses in the SLAM record's `+0x144` transform)?
   Capture Sony's driver on Windows: USB control traffic around start, and
   SLAM `+0x144` / `+0x1c8` with and without a saved map.
2. Who sets the RP slot commands 2 / 3 / 5, and when? Capture RP headers
   across a room setup in the PS VR2 App.
3. Meaning of the subcmd 3 block and of the relocaliser config values.
4. Is the SLAM record's map-anchor transform (`0x144`) produced from our
   subcmd 1 result? If yes, item 3 of §1 is what gives a stable origin.
5. Event 0x2bf identity (703 = `VREvent_QuitAcknowledged` in current
   OpenVR headers) [likely].

## Corrections to the main doc

- `0x0a` Set row: now **[sure]**. Length is always 0xd8; `subcmd` selects
  the payload (1 = localiser result, 3 = parameter block, 4 = start).
  Sent through `FUN_1801adba0` with 20 ms pacing (§2.2).
- `0x8a` row: the RP record carries 8 u32 command slots at +0x08 that the
  headset uses to drive the host relocaliser (§2.1).
- Code map row `0x180120000–0x18013ffff`: map save scheduling, play-area
  state and chaperone writing are now covered here.

## 9. Function map

| VA | Role | Covered |
|---|---|---|
| `FUN_1801ac4e0` | RelocalizerThread loop | yes |
| `FUN_1801ac140` | initializeMap | yes |
| `FUN_1801ac1f0` | loadMap | yes |
| `FUN_1801ad9d0` | Relocaliser configure + first init/load | yes |
| `FUN_1801abf90` | `vrtracker2.ini` `loadMap` read, create relocaliser | yes |
| `FUN_1801aaf00` | Relocalizer constructor, seed | yes |
| `FUN_1801adba0` | Paced Set `0x0a` sender | yes |
| `FUN_180127fe0` | Set `0x0a`, len 0xd8 | yes |
| `FUN_180125000`, `FUN_180128600` | RP record copy / wait | yes |
| `FUN_1801df010` | RunLocalizer wrapper, result layout | yes |
| `FUN_1801df630` | UpdateMap wrapper | yes |
| `FUN_1801d6230` | 6×6 covariance decomposition | partly |
| `FUN_1801abe50 … FUN_1801adc60` | Queue commands 0–7 | yes |
| `FUN_1801a3e50 … FUN_1801a4180` | Companion-app event handlers | yes |
| `FUN_1801a2040` | Handler registration (event bits) | yes |
| `FUN_180120290` | Quit save, delayed default play area | yes |
| `FUN_180132890` | Routine save (60 s), cmd 7 trigger | partly |
| `FUN_180134800` | Tracking / play-area status | partly |
| `FUN_1801216d0` | Play-area state from map id | yes |
| `FUN_180121830` | Accept new play area | yes |
| `FUN_1801214a0` | Absolute bounds | yes |
| `FUN_1801211a0` | Write meta + save map | yes |
| `FUN_180120f20` | `sceBoundaryMeta.bin` writer | yes |
| `FUN_18011fd30` | `sceBoundaryMeta.bin` reader | yes |
| `FUN_180120a80` | Load meta or reset | yes |
| `FUN_180120b40`, `FUN_180120c80` | Delete meta / chaperone + universe reset | yes |
| `FUN_180120570` | Chaperone JSON reader | yes |
| `FUN_1801218d0` | Chaperone JSON write + set 2004 | yes |
| `FUN_18011fc50` | Universe init on activate | yes |
| `FUN_18019d1b0` | HMD activate: sets property 2048 | yes |
| `FUN_1801f1360` and backend | Relocaliser internals | no (summary) |
