# Room, play area and relocaliser

Room map, play area and Sony's relocaliser.

Parts:

- [Part A: Room map and play area](#part-a-room-map-and-play-area)
- [Part B: Relocaliser](#part-b-relocaliser)

---

## Part A: Room map and play area

Sony PSVR2 SteamVR driver `driver_playstation_vr2_orig.dll`, PE timestamp
1741940190, image base `0x180000000`. Written from the decompile in our own
words. Earlier notes this builds on, and does not repeat:

- [part B](#part-b-relocaliser): RP frame unpack (`FUN_1801ab410`) and the full
  `sceMapDb.bin` layout (header, SHA-256, keyframe / edge tables).
- [`vrtrace.md` §10](vrtrace.md): universe / chaperone helpers in
  `0x18011e000–0x18011ffff` (default play area, chaperone JSON builder,
  `sceBoundaryMeta.bin` loader, map-state singleton fields).

Tags: **[sure]** read in code, **[likely]** strong inference, **[guess]**.

### A.1 Driver summary

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
   1.5 m around the headset (§A.5.3). This is all SteamVR needs for bounds.
   Bounds never go to the headset.
2. **Keep the RP endpoint drained (kernel, done).** EP `0x8a` (interface 9)
   must be read or the tracker stalls (`kernel/psvr2_aux.c`).
3. **Relocaliser feedback (userspace, optional, large).** For a persistent
   room, the host must run a relocaliser on each RP record and send the
   result back with **Set report `0x0a`** (§A.2.3, §A.3). Without this, each
   session's SLAM origin is wherever the headset started, and a saved play
   area cannot be re-anchored. Implementing it means reimplementing Sony's
   keyframe / pose-graph relocaliser (`0x1801e0000–0x18023ffff`), or
   writing an open one that consumes the RP features. It is not needed
   for a working driver.
4. **Persistence files (userspace, only with item 3).** `sceMapDb.bin`
   (map) and `sceBoundaryMeta.bin` (which map id the play area belongs
   to). Save on quit, every 60 s while worn, and after every play-area
   edit (§A.3.4).

Kernel: nothing beyond item 2, plus a way for userspace to send vendor
control Set `0x0a` (the generic control-transfer path already covers it).
Everything else is userspace.

### A.2 Wire and data formats

#### A.2.1 RP record header (EP `0x8a`, 0xc8780 bytes)

overview.md part B and part B give the magic and the frame unpack. Extra facts:

| Off | Size | Meaning |
|---|---|---|
| 0x00 | 2 | `"RP"` |
| 0x04 | 4 | u32 size 0xc8780 |
| 0x08 | 8 × u32 | **Command slots from the headset** [sure: loop in `FUN_1801ac4e0`]. Each slot is processed in order: 1 = process this frame, 2 = initialise map, 3 = optimise map, 5 = load map from disk. Other values are ignored. |
| 0x2c.. | | RP frame body (part B) |
| 0x9afb4 | i32 | (frame 0x9af88) **Map id the headset currently tracks against**, −1 = none ("map latch") [sure: read as `DAT_180597018`; meaning likely] |
| 0x9afb8 | u32 | (frame 0x9af8c) headset tracking sub-state; 5 gates the coverage counter (§A.5.2) [sure: test; meaning guess] |

The host keeps a u16 sequence number per received record (USB thread
counter, not a record field). The relocaliser thread waits up to 100 ms
for it to change (`FUN_180128600`), copies the record (`FUN_180125000`),
and does nothing if the number is unchanged [sure].

The map id low 16 bits are dropped before comparison (`id & 0xffff0000`,
`FUN_1801216d0`), which matches the `map_id` format in part B
(`(seed + n) << 16`, low bits bumped on edits) [sure].

#### A.2.2 Set `0x0a` "setRelocPreOutput" (host → headset)

Vendor control Set (`FUN_1801283f0`): header `u8 0x0a, u8 0, u16 subcmd,
u16 len = 0xd8, u16 0`, then **always 0xd8 data bytes**, whatever the
subcmd [sure: `FUN_180127fe0`]. Only sent while the device state is 2
(open); otherwise it returns `0xfffffcf7` [sure].

| subcmd | Payload (first bytes; rest of 0xd8 is whatever the static buffer holds, zero in practice) | When |
|---|---|---|
| 4 | u32 `1` | Relocaliser (re)start, before init/load (§A.3.1) [sure] |
| 3 | 8 × 4 bytes: u32 `0x20`, u32 `0xb4` (180), u32 `10`, u32 `4`, f32 `0.6`, f32 `0.0625`, f32 `100.0`, u32 `12` | Once, after the first processed frame following a (re)start [sure: values; meaning unknown] |
| 1 | Localiser result, §A.2.3 | After every processed RP frame [sure] |

Pacing [sure, `FUN_1801adba0`]: at least 20 ms between two `0x0a` sends
(sleeps the remainder). A send that returns ≤ 0 logs "busy", sleeps 1 ms
and retries, up to 5 retries, then logs "timeout" and drops it.

#### A.2.3 Localiser result (subcmd 1 payload, 0xd8 bytes)

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

#### A.2.4 `sceBoundaryMeta.bin` (0x48 bytes) [sure]

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

#### A.2.5 `chaperone_info.vrchap` [sure]

Standard SteamVR JSON, one universe; builder documented in `vrtrace.md` §10
(`FUN_18011e8e0`, wall height 2.43 m, `version` 5, `jsonid`
`"chaperone_info"`, `time` = `asctime()` string). The universe id is
incremented (skipping 0) on each write, then property 2004 is set to it
(`FUN_180121200`). The reader (`FUN_180120570`) fills the 0x850-byte area
block from `collision_bounds[i][0]` (x, z of each wall's first corner),
`play_area[0..1]`, and `seated`/`standing` translation + yaw.

#### A.2.6 `sceMapDb.bin`

Format in part B. Path: `<Prop_UserConfigPath_String>\sceMapDb.bin`.

#### A.2.7 `playarea_data.bin`

VrTrace capture stream only (`FUN_18010cdd0`, `vrtrace.md` §4.2). Not part
of normal operation; skip.

### A.3 Sequences

#### A.3.1 Start (`FUN_1801abf90` → `FUN_1801aaf00` → `FUN_1801ad9d0`)

1. Read `vrtracker2.ini` from the current directory; section
   `[Relocalizer]`, key `loadMap=<int>`; default 1 [sure]. Nonzero means
   "load the saved map at start".
2. Create the relocaliser with a seed = `MMDDhhmmss` of local time
   (`FUN_1801debe0`), start `RelocalizerThread` [sure].
3. Configure: parameters {−1.0, 1000.0, 0, 0x100, 4, 0x18, 1, 1,
   0.9397 (= cos 20°), 0.25} (`FUN_1801df4c0`) and mode 2
   (`FUN_1801df410`) [sure: values; meanings in part B, Configuration].
4. Send Set `0x0a` subcmd 4 `{1}`; queue "send subcmd 3 after the next
   frame" [sure].
5. If `loadMap` is 0: initialise an empty map. Otherwise: load
   `sceMapDb.bin` (§A.3.3) [sure].

The same 3–5 run again on command 6, which the USB code queues each time
the control interface alternate setting is (re)applied (`FUN_1801256c0`,
i.e. headset (re)connect) [sure].

#### A.3.2 Per RP record (`FUN_1801ac4e0`)

1. Wait for a new record (≤ 100 ms). Copy it.
2. Run headset command slots (§A.2.1).
3. Run host-queued commands (§A.4.1).
4. If slot command 1 was present:
   1. Unpack the RP frame.
   2. Log "Map latched" / "unlatched" when the record's map id changes
      between −1 and not −1.
   3. Tell the map state the current map id (−1 if tracking is not OK)
      (`FUN_1801216d0`, §A.4.2).
   4. RunLocalizer → result; UpdateMap → "map changed" flag.
   5. Every frame count ≥ 50 with any map change since the last
      optimise: OptimizeMap, reset the count [sure].
   6. If anchored and an "unremovable keyframes" request is pending: mark
      the current keyframes unremovable (`FUN_1801df5a0`).
   7. Set `0x0a` subcmd 1 with the result; then subcmd 3 if queued.
   8. Update the 3 s anchoring timer and push status to the companion app
      (§A.5.2).

#### A.3.3 Load map (`FUN_1801ac1f0`)

1. Reset the relocaliser.
2. Load `<config>\sceMapDb.bin` (`FUN_1801decf0`). On failure, mark the
   map state "map initialised" (no map) and log.
3. Set coverage count to 16 (full) [sure].
4. Remember load success (`+0x70`), then `FUN_180120a80`: try to load
   `sceBoundaryMeta.bin`; if that fails, reset the play-area state (map
   id −1, no universe) and rewrite the meta file [sure].

#### A.3.4 When maps are saved, loaded and discarded

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
backend in one write of the whole image (part B) [sure].

### A.4 State machines

#### A.4.1 Host relocaliser command queue (`param + 0x64040`)

Producers are the companion-app callbacks (§A.4.3) and the map state.

| Cmd | Name | Action |
|---|---|---|
| 0 | initializeMap | Reset relocaliser; lock state = 1; coverage = 0; map state "reset" + "map initialised" |
| 1 | loadMap | §A.3.3 |
| 2 | saveMap | Save `sceMapDb.bin` |
| 3 | removeMap | Delete `sceMapDb.bin` |
| 4 | ActivateMap | If lock state = 1, lock the current keyframes (`FUN_1801df440`); lock state = 0 |
| 5 | RefineMap | Lock state = 2; coverage = 0 |
| 6 | restart | §A.3.1 steps 3–5 |
| 7 | — | After this frame, mark keyframes unremovable (if anchored) |

#### A.4.2 Play-area state (map-state singleton `+0x68`, `FUN_1801216d0`)

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

#### A.4.3 Companion-app callbacks (`FUN_1801a2040` registration)

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
| 0x40 | `FUN_1801a40d0` | PlayAreaChanged: read the area drawn in the app from shared memory, apply (§A.4.2), save |
| 0x80 | `FUN_1801a4040` | InitializeMapAndSetDefaultPlayArea: initialise map and arm a *delayed default play area* |
| 0x100 | `FUN_1801a3f90` | SetDefaultPlayArea now (circle around the headset) |

Delayed default play area (`FUN_180120290`, each frame): when armed and
the map id has changed from the id at arming time and is not −1, build
the default area at the current headset pose, apply it, write meta and
save the map ("Delay Set Default PlayArea") [sure].

Room setup flow, as a Linux equivalent needs it: initialise map → user
walks around (coverage, §A.5.2) → ActivateMap → app sends the drawn
boundary (0x40) → saved. A Linux driver without the app can replace all
of this with: default area on first run, or an area from a config file.

### A.5 Algorithms

#### A.5.1 Relocaliser (summary only)

Keyframe map (≤ 97 keyframes, ≤ 256 features each, 128-byte descriptors,
pose-graph edges with 6×6 information matrices; layout in part B).
RunLocalizer matches the RP frame's features against the map and returns
a pose + covariance; UpdateMap inserts keyframes / edges; OptimizeMap runs
the pose graph. Keyframes can be locked (ActivateMap) or made
unremovable so that the saved room stays stable. A Linux driver only
needs this to reuse Sony maps; none of it is needed for tracking.

#### A.5.2 Coverage and anchoring status (UI only) [likely]

When the result is relocalised and the sub-state is 5, the yaw of the
relocalised pose in degrees (+360) sets one bit in each of four bin
masks: 4 × 90°, 4 × 90° offset by 45°, 12 × 30°, 30 × 12°. The count of
newly set bits (first three masks) is the "map registration" progress
sent to the app; a load sets it to 16. A 3 s timer (3,000,000 µs) starts
when tracking is OK but the map has not been anchored for 3 s; while it
runs, the app is told registration is lost. Not needed by Linux.

#### A.5.3 Play area to SteamVR

Absolute bounds (`FUN_1801214a0`) [sure]: rotate each area point `(x,z)`
by the standing yaw (half-angle quaternion about +Y, angle = −yaw) and
offset by the standing `(−x, −z)`. The result is kept in the map state
for inside/outside tests (`FUN_18011e6b0`) and the safety dialog. SteamVR
gets bounds only through the chaperone file plus property 2004.

Default area: `vrtrace.md` §10 (`FUN_18011ef90`). Head-height assumption
1.5 m: standing origin = head position minus 1.5 m in y [sure].

### A.6 Constants

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

### A.7 Gaps in the current Linux driver

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

### A.8 Open questions

1. Does the headset need Set `0x0a` subcmd 4 / 1 for anything (e.g.
   accepting map-relative poses in the SLAM record's `+0x144` transform)?
   Capture Sony's driver on Windows: USB control traffic around start, and
   SLAM `+0x144` / `+0x1c8` with and without a saved map.
2. Who sets the RP slot commands 2 / 3 / 5, and when? Capture RP headers
   across a room setup in the PS VR2 App.
3. Meaning of the subcmd 3 block and of the relocaliser config values.
4. Is the SLAM record's map-anchor transform (`0x144`) produced from our
   subcmd 1 result? If yes, item 3 of §A.1 is what gives a stable origin.
5. Event 0x2bf identity (703 = `VREvent_QuitAcknowledged` in current
   OpenVR headers) [likely].

### A.9 Function map

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

---

## Part B: Relocaliser

Behavioural notes on Sony's relocaliser / room-map backend in
`driver_playstation_vr2_orig.dll` (PE timestamp 1741940190, image base
`0x180000000`), VA range `0x1801e0000–0x18023ffff`, plus the "RP" record
parser it is fed by. Written in our own words from the decompile; no Sony
code is reproduced. Our implementation lives in
[`userspace/lib/sie/reloc/`](../../userspace/lib/sie/reloc/); differential
tests are `scripts/sony-re/difftest/t_reloc_*.c`; per-function status is in
[`scripts/sony-re/status/reloc.tsv`](../../scripts/sony-re/status/reloc.tsv).

Offsets are hexadecimal bytes. "kf" = keyframe.

### Pipeline

The `RelocalizerThread` loop (`FUN_1801ac4e0`, steamvr range) polls the
RelocPre USB thread for a new EP `0x8a` record (sequence number change), then:

1. runs queued map commands (1 = process frame, 2 = initialise map,
   3 = optimise map, 5 = load map from `<user dir>\sceMapDb.bin`);
2. unpacks the record into an *RP frame* (below);
3. calls *RunLocalizer* (`FUN_1801df010`) and *UpdateMap* (`FUN_1801df630`);
4. after 50 frames with a map change, calls *OptimizeMap* (`FUN_1801ded40`).

The relocaliser object API (`0x1801de000–0x1801dffff`) is a thin wrapper:
each call checks the handle (`-5` if NULL, `-4` if not initialised), records
"busy" and the result code, and forwards to the backend object in this range
(load map → `FUN_1801ef410`, save map → `FUN_1801f1750`, localise →
`FUN_1801f1360`, frame update → `FUN_1801ef4e0`).

### RunLocalizer (`FUN_1801f1360` → `FUN_1801ef4e0` → `FUN_1801fade0`) [sure for the structure]

1. Check the frame (`FUN_1801f6180`) and set the camera model
   (`FUN_1801f23e0`). Update running statistics: frame count, keypoint
   sum, the mean of the third per-keypoint float over keypoints whose
   byte 1 is 0.
2. If the caller passes a pose, keep it as the prior in the tracking state
   (`*(loc + 0x1428)`, `+0x48` and `+0x10c`).
3. Run the search (`FUN_1801fade0`) unless the localiser mode
   (`loc + 0x1420`) is 1 and the map query `FUN_180226520` returns 0
   (result code 1 then).
   Output: 12 candidate result slots of 400 bytes (byte valid at +0,
   byte at +0xc4, u32 at +0x188, keyframe id at +0x18c, −1 = none). The
   number of valid slots is returned to the caller.

The search mode is the u32 at searcher `+0x20`. Without a prior (tracking
state `+0x04` = 0) every mode runs as the default.

| Mode | Prior search (`FUN_1801fb540`) | Search A (`FUN_1801fb280`), count | Search B (`FUN_1801fb880`), count |
|---|---|---|---|
| default | no | 0 | 12 |
| 1 | no | 0 | 12 |
| 2 | no | 5 | 3 |
| 3 | no | 1 | 11 |
| 4 | yes, first; stop if it succeeds | 0 | 11 |

Before searches A and B the query is built (`FUN_1801f5a90`): copy the
usable keypoints (state 0 or `0xcc`), drop those whose third float is
outside `[searcher+0x38, searcher+0x3c]` (a bound of 0 is off), keep at
most 256, and go on only if at least `searcher+0x04` remain. Then
`FUN_1801fd280` (671 lines) checks the candidates and estimates the pose,
and, when frame `0x9ae68` is 1, `FUN_1801f8a90` runs.

#### Keyframe retrieval: vocabulary-tree bag of words [sure unless tagged]

Search B (`FUN_1801fb880` → `FUN_1801fa720`) ranks keyframes by image
similarity:

1. **Word lookup** (`FUN_1801f4430`). Each query descriptor (32 × s16,
   the 64 bytes per keypoint) walks a vocabulary tree from the root.
   - A node is 24 bytes: `+1` leaf flag, `+2` child count (2 or 3), `+8`
     child indices (s16), `+16` child centres (32 × s16 each).
   - At each level it takes the child with the smallest squared
     Euclidean distance (int16 differences, AVX2), for at most
     `vocab+0x04` levels or until a leaf.
   - Every node it passes gets its count raised (u16 per node). The
     non-zero nodes become a sparse vector: sorted node ids plus counts.
2. **Weighting** (`FUN_1801f4090`): `w_i = count_i · idf_i` (float per
   node at `vocab+0x38`), then L1-normalised (all zero if the sum is 0).
3. **Score** (`FUN_180227d30`): histogram intersection over the common
   node ids, `Σ min(q_i, k_i)`, in 0…1, higher = more alike (SSE4.2
   `pcmpestri` merge of the two sorted id lists).
4. Score every map keyframe (`FUN_1801f51d0`), or, with the per-search
   flag set, only the current keyframe plus up to 9 graph neighbours
   (`FUN_180226580(map, kf, 9, …)`). Sort (`FUN_1801f3950`), keep the
   best in the 12-slot list (`FUN_1801f7fd0`). Every listed keyframe with
   score ≤ 1.0 gets a query hit (`FUN_180225f90`).
5. If the tracking state's last keyframe (`+0x1d4` equal to `+0x38`, pose
   valid) is still in the map, it is put first with score 778.0, ahead of
   any real score.

**Vocabulary** (`FUN_1801f4d30`, built at create) [sure]. Built into the
DLL, not stored with the map: 9841 nodes = 1 + 3 + … + 3⁸, so a 3-way tree
with 8 levels below the root. One descriptor byte per node at
`0x1805e35a0` (bit 7 leaf, bits 3–6 level, bits 0–2 child count), child
indices (s16) from `0x18026a620`, child centres (64 bytes each) from
`0x18026f300`. A port has to read these tables from Sony's DLL at run time;
they are not ours to commit. The keyframe database holds 96 entries.

**idf** (`FUN_1801f5430`, after every keyframe add or drop):
`idf_i = ln(N / n_i)` with N keyframes in the database and n_i of them
using node i (0 when unused); with N = 1 all weights are 1. All stored
keyframe vectors are then re-weighted.

**Search A** (`FUN_1801fb280` → `FUN_1801fa220`): the same retrieval,
then, for each candidate, a predicted camera pose relative to that
keyframe (`FUN_1801fbac0`): the last map ↔ session alignment of the
tracking state (`+0x08` translation, `+0x14` rotation) composed with the
frame's own headset pose (anchor 1) and the keyframe pose
(`FUN_1802261e0`). `FUN_180224820` projects the keyframe's points with
that pose (z > 0, inside the image) and marks them visible. A candidate is
kept, with its predicted pose, when at least `cfg+0x1c` (24) points are
visible.

**Prior search** (`FUN_1801fb540`): the same prediction and visibility
test for the tracking state's last keyframe only (`*state`), then the
matcher with that pose.

#### Feature matching (`FUN_1802282a0`, from `FUN_1801f9ac0`) [sure]

Per candidate keyframe: nearest neighbour over its features by squared
descriptor distance, normalised by 2³⁵.

- Accept only if the best distance is < 0.0064 (normalised) and
  `best < 0.8 · second best` (ratio test on squared distances).
- Each keyframe feature keeps only its closest query keypoint (a closer
  one replaces it), so matches are one to one.
- Keypoints with a negative state byte are skipped.

#### Pose solve (`FUN_18022bb70`) [sure for the structure, likely for the weights]

Gauss-Newton on a 6-DoF pose, iteration count from the caller (`+0x04`
of its config):

- Per match (`FUN_18022b390`): project the keyframe's 3-D point with the
  current pose (divide by z), take the 2-D residual against the keypoint,
  and whiten it with the square root of a 2 × 2 matrix built from the
  feature's covariance plus identity [likely]. Add `JᵀJ` and `Jᵀr` to a
  6 × 6 system.
- Mirror the upper triangle, factor and solve (`FUN_180229910`,
  `FUN_1802296c0`), apply the step through the exponential map
  (`FUN_180237190`).

`FUN_1801f90d0` runs, per candidate (at most the search count): match
(`FUN_1801f9ac0`), then for each candidate that passed, a guided patch
check (`FUN_1801fd070`, below), the solve (`FUN_1801f9eb0`), and a pose
sanity test (`FUN_1801d6230`).

**Guided patch check** (`FUN_1801fd070` → `FUN_180228ff0`) [sure unless
tagged]. Each keyframe feature carries two 24 × 24 8-bit patches (the
0x240 halves of its 0x480 block, one per camera). For every feature with
a 3-D point:

1. Warp the reference patch into the current view with the predicted
   pose (`FUN_180236e20`, inverse by `FUN_1801eee10`) and project the
   point.
2. Skip if the projection is within 4 px of the 640 × 640 border, or the
   8 × 8 warped template (`FUN_180236960`, centred at 11 − offset) cannot
   be built.
3. Search the current image around the projection with the 8 × 8
   template, radius 7 px (`FUN_180228cc0`); score [likely: zero-mean
   NCC]. Accept if the score is > 0.49 (the feature's hit count `+6`
   goes up), else mark it rejected (`0xde`). A feature that could not be
   projected is marked `0xdf`.

The candidate passes when the accepted count is ≥ `cfg+0x1c` (24). A pose that passes is stored in the slot and the
keyframe gets a match credit (`FUN_180225fa0`).

#### Choosing the result (`FUN_1801fd280`) [sure]

Each solved slot holds a 6 × 6 covariance and the inlier count (`+0x188`).
Its confidence is:

```
sp = sqrt(P[0][0] + P[1][1] + P[2][2])     // position block
sr = sqrt(P[3][3] + P[4][4] + P[5][5])     // rotation block
f(s, lim) = (s <= lim) ? 0.5 + 0.5 * (lim - s) / lim : 0.5
conf = min(inliers, 100) * f(sp, 0.005) * f(sr, 0.004)
```

The slot with the highest confidence wins.

**Joint refinement** [sure for the flow, likely for intent]. Only when at
least 2 candidates solved: sort them by confidence and take the best
min(n, 4) keyframes. `FUN_1801fc140` expresses the winning pose in each of
those keyframes' frames (keyframe poses via `FUN_1802261f0`), runs the
guided patch check (`FUN_180228ff0`) against each, and requires ≥
`cfg+0x1c` accepted patches. `FUN_1801f94a0` pools the correspondences of
all of them into the winner's frame and solves once more
(`FUN_18022bb70`). The refined pose is written to the result if
`FUN_1801fad30` accepts it.

#### Configuration (`FUN_1801df4c0` → `FUN_1801f1e80`, `FUN_1801df410`) [sure where mapped]

The start sequence (§A.3.1, built in `FUN_1801ad9d0`) passes a 44-byte block P. Where it goes:

| P | Value at start | Copied to | Use |
|---|---|---|---|
| +0x00, +0x04 | −1.0, 1000.0 | query builder `+0x38`, `+0x3c` | keep-window for the keypoint's third float (≤ 0 = no bound). Rejected if both > 0 and min > max |
| +0x08 | 0 (8 bytes) | | not traced |
| +0x10, +0x11 (u8) | 0, 1 (written as the u32 256) | `loc+0x1404`, `+0x1405` | UpdateMap switches; `+0x1405` enables the keyframe drop test (step 6 below) |
| +0x14 | 4 | `loc+0x13c0`, `+0x13d0` | not traced |
| +0x18 | 24 | `loc+0x13d8`, `+0x13e8` | `cfg+0x1c` (cfg = `loc+0x13bc`): minimum visible points (search A, prior search) and minimum accepted patches (guided check) |
| +0x1c | 1 | `loc+0x13f0` | not traced |
| +0x20 (u8) | 1 | `loc+0x1414` | novelty test on |
| +0x24 | 0.9397 (cos 20°) | `loc+0x1418` | same-view rotation limit |
| +0x28 | 0.25 | `loc+0x141c` | same-view distance limit [likely m] |

`FUN_1801df410` with mode 2 sets searcher `+0x20` = 2: so on PC the
search is "search A with 5 keyframes, then search B with 3".

### UpdateMap (`FUN_1801df630` → `FUN_1801f2470`) [sure for the decisions; likely for intent]

After RunLocalizer, per frame:

1. `success` = any of the 12 result slots is valid.
2. `loop` = the graph distance (`FUN_1802269c0`) between the tracking
   state's anchor keyframe (`+0x10c`) and a matched keyframe is > 4 hops.
3. `novel` (when `loc+0x1414` is set) = no valid slot has its keyframe
   within the same-view limits of the current pose: rotation
   `(trace(Rᵀ_kf R) − 1)/2 > cos 20°` **and** distance² < 0.25².
   If any slot is that close, `novel` is false.
4. These flags go to `loc+0x1360..0x1362`, `+0x1388`. When the mode
   (`loc+0x1420`) is not 0 and (`!success` or `novel` or `loop` or the
   third flag), a keyframe candidate is built from the frame
   (`FUN_1801ef740`), checked (`FUN_1801ecd60`), and inserted
   (`FUN_1801f0860`: slot alloc `FUN_1801edf80`, database add
   `FUN_1801f7210` "Add Keyframe", edges `FUN_1801ea5c0`/`FUN_1801ed9f0`).
5. With a loop or the third flag, extra edges are added
   (`FUN_1801ed4f0`, `FUN_1801ed6c0`).
6. Keyframes that stopped being useful are dropped from the database
   (`FUN_1801ef1d0` visibility test → `FUN_1801ede80` → `FUN_1801f71d0`
   "Drop Keyframe Description"), and `FUN_1801f0b80` prunes keyframe
   poses and edges ("Prune KeyframePose and PoseGraph").

Details [sure unless tagged]:

- **Keyframe check** (`FUN_1801ecd60`): rotate the frame's reference
  axis (9 floats at frame `0x9af64`, in the misc-state block) by the
  candidate pose and take the middle component. The keyframe is accepted
  only if it is ≥ cos 1° (0.99985), i.e. the new keyframe's vertical axis
  is within 1° of the frame's [likely: gravity] direction. The three
  components go to `loc+0x136c..0x1374`, the flag to `loc+0x1368`.
- **Edges** (`FUN_180225970` stores one; an existing edge between the same
  two keyframes is removed first). Three types:

  | Type | Made by | Between | Relative pose / information |
  |---|---|---|---|
  | 0 | `FUN_1801ed9f0` (on insert) | tracking-state anchor (`+0x10c`) → new keyframe | the state's stored relative pose `+0x114` and 6 × 6 information `+0x144`; when a keyframe's camera index is ≥ 2, corrected through the 52-byte camera records at `+0x3b4` |
  | 1 | `FUN_1801ed4f0` (loop / third flag) | new keyframe ↔ each matched keyframe not yet linked | the slot's solved pose (`+0xc4`), inverted |
  | 2 | `FUN_1801ed6c0` | pairs of matched keyframes, using the graph distance (`FUN_1802269c0`) | [likely] closes the loop between distant parts of the graph |

  OptimizeMap gives types 1–3 the sign −1 and type 0 +1.
- **Prune** (`FUN_1801f0b80`) runs two map queries: query 6 lists
  keyframes whose pose is removed (`FUN_180227140`) unless it is the
  tracking anchor; query 3 lists keyframes whose description is dropped
  from the database (`FUN_1801f71d0`) and whose pose is then removed. The
  rules inside the query engine (`FUN_180232180`, `FUN_180230d40`) were
  not read [likely: state 2 "deleted" and unused keyframes].

### OptimizeMap (`FUN_1801ded40` → `FUN_1801efb90` → `FUN_18021d4d0`) [sure for the problem; likely for the solver]

Pose-graph problem built from the map:

- Nodes: every keyframe pose (`loc+0x14a0`).
- Edges: keyframe indices A, B (`loc+0x14e0`), relative translation
  (`+0x14f0`, 3 floats), rotation (`+0x1500`, 3 × 3), 6 × 6 information
  (`+0x1510`, 36 floats), and a per-edge sign: −1.0 for edge types 1–3,
  +1.0 otherwise (`+0x1520`) [meaning open].
- Solver `FUN_18021d4d0` (5887 lines, with a sparse symbolic analysis
  `FUN_18020c770`/`FUN_1802177d0` and a sparse factor `FUN_1802158c0`):
  at most **8 iterations**; stop early when the step norm is below
  `0.001 · node count`, or when the cost falls by less than 0.01 %
  (`cost_new > cost·(1 − 1e-4)`). After the solve, outputs in (−0.001, 0)
  are set to 0.
- On success the poses are written back (`FUN_180227670`) and the map is
  marked changed (`FUN_180226f50`, `FUN_180226f40`, `FUN_180226af0`).

For a port, any sparse pose-graph optimiser (e.g. g2o, Ceres) over the
same nodes, edges and information matrices fills this role; the 8
iteration and stop rules are what to copy.

Still open: the query
builder's minimum keypoint count (`+0x04`), the map query rules behind
the prune, and the solver's exact cost (robust kernel or not).

### RP frame (unpack of the EP 0x8a record) — `FUN_1801ab410`

The raw record is 0xc8780 bytes (magic `"RP"`, u32 size at +4). The first
0x2c bytes are a header the relocaliser does not use. The unpacked frame is
0x9b040 bytes and, for almost all of it, is simply the record body shifted
down by 0x2c (frame offset `f` comes from record offset `f + 0x2c`):

| Frame offset | Size | Content (meaning **[likely]** unless noted) |
|---|---|---|
| 0x00000 | 8 | two u32 (frame id / timestamp) |
| 0x00008 | 324 × 4 | 2-D keypoints: u16 u, u16 v (324 = 2 × 162: two cameras) |
| 0x00518 | 162 × 0x80 | per-feature 128-byte descriptors |
| 0x05618 | 324 × 2 | per-keypoint two bytes (octave / flags) |
| 0x058a0 | 162 × 0x18 | per-feature 6 floats |
| 0x067d0 | 162 × 0x48 | per-feature 18 floats |
| 0x09560 | 162 × 0x480 | per-feature 0x480-byte block (two 0x240 halves, one per camera) |
| 0x36e60 | 1 | **bool**: record u32 at 0x36e8c != 0 [sure] |
| 0x36e64 | 4 | image parameter (passed with the image) |
| 0x36e68 | 0x64000 | 8-bit image, row stride 640 (640 × 640) [sure: stride] |
| 0x9ae68 | 0x2c | misc. u32/float fields |
| 0x9ae94 | 1 | **bool**: record u32 at 0x9aec0 != 0 [sure]; "pose prior valid" |
| 0x9ae98 | 0xc0 | pose prior (used by RunLocalizer when the bool is set) |
| 0x9af58 | 0x98 | misc. state |
| 0x9aff0 | 4 | anchor count `n` |
| 0x9aff4 | 0x1c | anchor 0 (7 × u32), copied when `n ≥ 1` |
| 0x9b010 | 0x30 | anchor 1 (12 × u32), copied when `n ≥ 2` |

Exact behaviour [sure]:

* The two boolean slots are 4 bytes wide in the frame but only their first
  byte is written (0 or 1); the other three bytes keep whatever the output
  buffer held before.
* `n == 0`: anchor 0 becomes `{-1, 0, 0, 0, -1, -1, -1}` (u32).
* `n < 2`: anchor 1 becomes all zero except its first word, which is the
  quiet NaN `0xffc00000`.
* `n` itself is copied unchanged (any value ≥ 2 copies both anchors).

Implemented as `psvr2_reloc_rp_unpack` — **bit-exact**.

How the localiser reads the frame (`FUN_1801f5a90`, `FUN_1801f6180`,
`FUN_1801f23e0`, `FUN_1801ef4e0`) [sure]. It indexes the arrays above
**per keypoint**, not per feature:

| Frame offset | Per keypoint k (0…323) |
|---|---|
| 0x00004 | u32 keypoint count; the localiser refuses > 324 |
| 0x00008 + 4k | u16 u, u16 v (pixels, 640 × 640 image) |
| 0x00518 + 64k | 64-byte descriptor (the "162 × 0x80" block is 324 × 64) |
| 0x05618 + 2k | byte 0: keypoint state, copied with the keypoint; 0 or `0xcc` = usable. Byte 1: the range test below runs only when it is 0 or `0xcc`. A range-rejected keypoint gets state `0xcd` in the localiser's copy |
| 0x058a0 + 12k | 3 floats; the third is range-tested (below) [likely depth] |

Frame `0x9ae68` (u32) must be < 4; it indexes four per-kind counters in
the localiser [meaning open]. Frame `0x9ae70` holds 9 floats that
`FUN_1801f23e0` stores with the size 640 × 640 as the camera model
[likely: 3 × 3 intrinsics]. Anchor 1 (`0x9b010`, 3 + 9 floats) is passed
to the search as translation + rotation [likely: the current headset
pose].

### Room map file `sceMapDb.bin` and the in-memory map image

The map database is one flat, fixed-size byte image (0x12d18a0 bytes) that
lives in memory (pointer global `DAT_1805e5d50`) and is written to disk
verbatim. All tables are fixed-capacity arrays with occupancy bitmaps; there
are no pointers inside, so the file is position independent.

#### File header (0x28 bytes) [sure]

| Off | Type | Value |
|---|---|---|
| 0x00 | char[2] | `"MD"` |
| 0x02 | u16 | format version: 1 or 2 (writer always writes 2) |
| 0x04 | u32 | total size 0x12d18a0 |
| 0x08 | u8[32] | SHA-256 of the body (see versions) |

Versions:

* **v2** (current): body = bytes `[0x28, 0x12d18a0)`; the hash covers all of
  it.
* **v1**: the file stops at 0x12cb788 (no statistics trailer); the hash
  covers `[0x28, 0x12cb788)`. On load it is upgraded in memory: version set
  to 2 and the trailer `[0x12cb788, 0x12d18a0)` zeroed.

Load (`FUN_1802312c0`): read the header; require `"MD"`, version 1 or 2,
full-length body reads and a matching hash; on success close the file,
increment the trailer's load counter (0x12cb798) and remember the load
time. On any failure (including a missing file) the image is reset to an
empty map and given a fresh map id (see *map id*); result `-11` (or `-1`
if the file could not be opened). Save (`FUN_18022ee60`): add the seconds
elapsed since load/last save to 0x12cb79c, recompute the hash of the
v2 body, write the whole 0x12d18a0 bytes.

#### Image layout [sure: offsets and capacities; meanings marked]

| Off | Size | Content |
|---|---|---|
| 0x0000 | 0x28 | file header |
| 0x0028 | 9 | `"map_id"` signature (NUL padded, byte 0x31 forced 0) |
| 0x0034 | 4 | map id: `(seed + n) << 16`, low byte bumped on map edits |
| 0x0038 | 200 × 0x70 | *query log*: ring of `{u64 event no., char text[100]}` |
| 0x57b8 | 300 × 0x70 | *event log*: same record format, all events |
| 0xdaf8 | 4 | event-log write index (wraps 300 → 0) |
| 0xdafc | 4 | query-log write index (wraps 200 → 0) |
| 0xdb00 | 8 | next event number (starts at 1) |
| 0xdb08 | 97 × 0x2ed20 | keyframe slots |
| 0x11cb428 | 16 | keyframe occupancy bitmap (2 × u64; bits 0..96 used) |
| 0x11cb438 | 4 | kf bitmap: first word that may have a free bit |
| 0x11cb43c | 4 | kf: highest index ever allocated (−1 when empty) |
| 0x11cb440 | 4 | kf serial counter |
| 0x11cb444 | 4 | feature serial counter |
| 0x11cb450 | 97 × 0x10 | *keyframe-id table*: `{u32 index, u32 serial, u32 ?, u32 kf serial}` |
| 0x11cba60 | 16 | id-table occupancy bitmap |
| 0x11cba70 | 4 | id-table first-free-word hint |
| 0x11cba74 | 4 | id-table highest allocated index |
| 0x11cba78 | 4 | id-table serial counter |
| 0x11cba80 | 16 | id-table "marked" bitmap (set from an index list; see below) |
| 0x11cba90 | 4 | marked: lowest word |
| 0x11cba94 | 4 | marked: highest index |
| 0x11cba98 | 4848 × 0xd8 | pose-graph *edges* |
| 0x12cb518 | 76 × 8 | edge occupancy bitmap |
| 0x12cb778 | 4 | edge first-free-word hint |
| 0x12cb77c | 4 | edge highest allocated index |
| 0x12cb780 | 4 | edge serial counter |
| 0x12cb788 | 16 | (v2 trailer) creation time `{u64 unix s, u64 µs}` |
| 0x12cb798 | 4 | load counter |
| 0x12cb79c | 4 | accumulated use time, seconds |
| 0x12cb7a0 | 97 × 0x100 | per-keyframe statistics |

The 300 + 200 log texts are obfuscated: each character becomes
`(c XOR key[i mod 7]) + 0x20` with `key = "RuB1COn"`. The scratch buffer
used for this is a 128-byte static that is never re-terminated, so a short
message after a longer one carries the longer one's tail; the copy into the
log takes at most 99 characters and the 100th is forced to NUL.

#### Keyframe slot (0x2ed20 bytes)

| Off | Content |
|---|---|
| 0x00 | u32 own index |
| 0x04 | u32 serial (from 0x11cb440, pre-incremented) |
| 0x08 | u32 caller tag (e.g. map/session id) |
| 0x0c | pose: translation float[3] |
| 0x18 | pose: rotation float[3×3] (identity when constructed) |
| 0x3c | 4 × 32 bytes, opaque (copied from the frame) |
| 0xbc | 16 bytes, opaque |
| 0xcc | u32 state: 0 normal, 1 "unremovable", 2 "deleted, awaiting purge" |
| 0xd0 | u32 group (camera/map slot) |
| 0xd4 | u32 feature count (≤ 256) |
| 0xd8 | u64 timestamp |
| 0xe0 | u64 link (−1 = none) |
| 0xe8 | u32 counter A (query hits) |
| 0xec | u32 counter B (successful matches) |
| 0xf0 | 256 × 0x2ec feature records |
| 0x2ecf0 | intrinsics: `a1, a2, a0, fx1, fy1, cx1, cy1, fx2, fy2, cx2, cy2` |

Feature record (0x2ec bytes): `+0 u32, +4 serial (from 0x11cb444),
+8, +0xc u32, +0x10 0x240-byte descriptor block, +0x250 u8 invalid flag,
+0x252 64 bytes, +0x294 u32, +0x298 float[3] position, +0x2a4 float[9],
+0x2c8 i32[9] (−1 = absent)`.

Keyframe statistics record (0x100): `+0 creation time (16 bytes), +0x10 load
counter at creation, +0x14 = 2, +0x20 float[3], +0x40 float[3]`.

Edge (0xd8 bytes): `+0 index, +4 serial, +8 kf A, +0xc kf B, +0x10 type,
+0x14 u8 removed, +0x18 relative translation float[3], +0x24 rotation
float[3×3], +0x48 6×6 information matrix (float[36])`. A fresh edge has
identity rotation and zero elsewhere.

#### Allocation rule (keyframes, id table, edges)

All three tables share one allocator behaviour: starting at the bitmap word
named by the *hint*, find the first word that is not all ones; store that
word index into the hint (even if the result turns out to be out of range);
the free index is `64 × word + lowest clear bit`. If the index exceeds the
capacity − 1 the call fails with −1 (nothing else changes). Otherwise the
bit is set, *highest* is raised to the index if larger, the table's serial
counter is pre-incremented and stored in the record together with the
index. Freeing clears the bit, lowers the hint to the freed word if
smaller, and, when the freed index was the highest, recomputes *highest* as
the top set bit at or below that word (−1 if none).
