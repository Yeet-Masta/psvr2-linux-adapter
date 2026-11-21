# ledsync: keeping Sense controller LEDs in step with the headset cameras

Sony driver build PE timestamp 1741940190. Tracker side: `0x180190000–0x18019d000`
(irtracker); controller side: libpad `FUN_1801c0340`, `FUN_1801c1880`,
`FUN_1801c27d0`, `FUN_1801cbeb0`. Clock units are host µs unless stated
(`FUN_1801c8210` = QPC scaled by 1e6, constant at `0x180319960`).

## 1. Driver summary

Sony never sends raw camera frames to the PC for controller tracking. The
headset finds LED blobs itself (EP `0x89`, `"LD"` packets), and the PC only
has to make each Sense controller pulse its LEDs while the camera shutters
are open. The PC does this by putting a phase, an LED set, a time offset and
a frame period into every controller output report. A per-controller state
machine (INIT → PRESCAN → BROAD → BG → STABLE) finds the offset in two
steps: a coarse search that reads a blink code, then a fine search that
reads the LED's brightness. A small drift fit then trims the frame period.

What the Linux driver must do, in priority order:

1. **Kernel:** stop discarding EP `0x89`. Hand each `"LD"` packet (0x9050
   bytes) to userspace with its arrival time. Today `psvr2_aux.c` drains and
   drops it.
2. **Kernel (ioctl/sysfs) + userspace policy:** send Set `0x14` (LED-detect
   window, 16 bytes) and Set `0x09` (blob-detection thresholds, 24 bytes).
   Userspace picks the values (§2.5, §2.6); the kernel only forwards them.
   Camera mode `0x0b` must be `0x10` (already the case for SLAM).
3. **Userspace:** the headset device-µs → host-µs clock map (`0xde`, see
   `headset.md` §5.1) and the controller clock fit (`libpad.md`). Every
   base time in this topic goes headset frame time → host µs → controller
   time.
4. **Userspace:** the LedSyncer state machine (§4) and its commands to the
   libpad sync block (§2.1). The libpad receiver and output-report fill
   already exist in `userspace/lib/sie/libpad/` (`psvr2_ledsync_command`,
   `psvr2_sense_output_fill`); nothing sends commands to it yet.
5. **Userspace:** the LED choice for BROAD/BG (§5.1), the blink-code
   decoder (§5.2) and the brightness phase estimator (§5.3).
6. **Userspace:** the Many-Blob detection-mode switch (§4.3).

Without steps 1–4 the controllers keep their firmware default timing, and
their LEDs are mostly invisible to the headset's short exposures [guess].

## 2. Wire and data formats

### 2.1 Tracker → libpad sync command (`FUN_180194af0` → `FUN_18011b3c0` → `FUN_1801c0340`)

The tracker never writes HID itself. It hands a small byte message plus its
length to libpad (`FUN_18011b3c0` drops lengths outside 1..11). libpad
(`FUN_1801c1880`) validates the length per command, updates the controller's
LED-sync block (pad context +0x33e0), and the next output report carries it.
Every accepted command increments `seq_id` (block +0x01) [sure].

| Cmd | Len | Bytes | Effect in libpad | Source |
|-----|-----|-------|------------------|--------|
| 1 | 7 or 11 | `[1] phase, [2] period_id, [3..6] led[0..3], [7..10] u32 LE value` | Set phase. Phase 1 needs len 7 or 11; phases 2..4 need len 11; phases 5..6 need len 2. period_id clamped to ≤ 60; led[0..3] stored as-is. Phases 2..4: value = base_time offset, clamped to ±half_width. Phase 1: base_time recomputed from the headset frame clock (§5.1); with len 11 the value also sets 1frame_cycle (clamped). | [sure] `FUN_1801c1880` |
| 2 | 5 | `[1..4] led[0..3]` | Replace LED bytes only | [sure] |
| 3 | 5 | `[1..4] f32 drift` | `1frame_cycle = clamp((drift + 1.0) · 1frame_cycle)` | [sure] |
| 4 | 5 | `[1..4] u32 offset` | `base_time = clamp(offset, ±half_width)` | [sure] |
| 5 | 9 | `[1..4] f32 drift, [5..8] u32 offset` | Cmd 4 and cmd 3 together | [sure] |
| 6 | – | – | Ignored | [sure] |
| 7 | any | `[1] sub, [2] arg` | sub 1: phase := 5 (LED ALL OFF), clear sync-mode byte. sub 2: full reset of the controller's clock fit and sync block (defaults below). sub 3: start/stop libpad's perf statistics (debug, writes `../ledsync/*.txt`). | [sure] |

Note: in cmd 1 the tracker always sends len 11 except PRESCAN while the
controller's retry counter is < 2, which sends len 7 (no frame-cycle reset)
[sure] `FUN_180194af0`.

Every cmd 1/2/3/4/5 also zeroes block bytes +0x04..+0x07 before applying,
i.e. base_time restarts from 0 unless the command sets it [sure].

### 2.2 LED-sync block in libpad (pad context +0x33e0)

Extends the table in `docs/sony-driver-re.md`.

| Off | Type | Meaning | Reset value (cmd 7/2) |
|-----|------|---------|-----------------------|
| 0x00 | u8 | phase (0 INIT … 6 LED ALL ON, names at `0x180268d20`) | 5 |
| 0x01 | u8 | seq_id, +1 per accepted command and per PRESCAN refresh | 0 |
| 0x02 | u8 | period_id (≤ 60) | 0 |
| 0x04 | s32 | base_time / offset (controller µs; see units note) | 0 |
| 0x08 | u32 | 1frame_cycle | 16 683 350 |
| 0x0c | u8×4 | led[0..3] | ff ff ff ff |
| 0x10 | u64 | host µs of last PRESCAN refresh | 0 |
| 0x18 | u8 | sync mode (0, 1 or 2) from the capability probe | 0 |
| 0x1c | s32 | LED pulse width | 1000 |
| 0x20 | s32 | period step | 105 |
| 0x24 | s32 | base_time clamp half-width = step·15 | 1575 |

Sync-mode parameters (`FUN_1801c1820`, tables at `0x180268dc0`/`0x180268dc8`)
[sure]:

| Mode | When | width | step | half-width |
|------|------|-------|------|------------|
| 1 | firmware < 0x1006d, or probe failed 4 times | 1000 | 105 | 1575 |
| 2 | probe Set 0x80 `80 13 22 00` / Get 0x81 replied `0x81` | 500 | 50 | 750 |
| 0 | probe ran and reply was not 0x81 | unchanged | unchanged | unchanged |

Mode 2 halves the LED pulse and step: newer controller firmware supports a
tighter sync window [likely].

### 2.3 Output report placement

Already in `docs/sony-driver-re.md` ("Output-report placement",
`FUN_1801cbeb0`). Additions [sure]:

- base_time (+0x17) and 1frame_cycle (+0x1b) are each written ×3.
- led[0..3] (+0x1f..+0x22) are sent only in phases 2 (BROAD) and 3 (BG);
  in every other phase those bytes are `ff`. So in STABLE the controller
  keeps the LED pattern it last received [likely].
- The phase byte (+0x14) is written when phase is 0, 5 or 6, or when
  libpad's controller clock fit has a reference time (pad +0x2a70, the fit's
  t0, non-zero) [sure]. Before the first clock fit, phases 1..4 leave the
  byte at its default.

### 2.4 Phase timing presets (`FUN_18019b9e0`, `FUN_18019c3d0`; tables at `0x18026035c`..`0x18026037c`)

Each controller reports its own "preset" number (byte +0x10 of the
per-controller info block `FUN_180118c70`; 0 = not connected). The first
connected controller fixes the global preset (`0x180350d98` = preset,
`0x180350d99` = owner index, 2 = none). A second controller with a
different preset is held in INIT and Sony logs
`SHOULD BE UPDATE CONTROLLER FW!! set Preset No.%d, own Preset No.%d`
(string XOR-obfuscated in `FUN_180190300`) [sure]. So **both controllers
must use the same preset**; mixed firmware generations do not track
together.

The preset selects a time step and a search-window width per phase. The
tables are indexed by preset 1..2; values read from the DLL with the script
in §6 (they are timing parameters, not Sony data tables):

| Quantity | Preset 1 | Preset 2 | Meaning |
|----------|----------|----------|---------|
| step (µs) | 105 | 50 | One period_id unit = one LED time slot [likely] |
| PRESCAN window (µs) | 4200 | 2000 | period_id sent in PRESCAN = window/step = 40 / 40 |
| BROAD window (µs) | 5250 | 2100 | period_id 50 / 42 |
| BG window (µs) | 3150 | 1500 | period_id 30 / 30 |
| STABLE window (µs) | 1260 | 600 | period_id 12 / 12 |

Preset 1 matches libpad sync mode 1 (width 1000, step 105) and preset 2
matches sync mode 2 (width 500, step 50) [likely: same numbers, set
independently].

Derived values sent in cmd 1 (`FUN_18019b9e0`) [sure]:

- PRESCAN, BROAD, BG, STABLE: `period_id = (int)(window / step)`.
- BG entered from BROAD: value = `(int)(21·step) − BG/2` (630 µs / 300 µs).
- BG entered from STABLE: value = `STABLE/2 − BG/2` (−945 / −450).
- STABLE: value = `BG/2 − STABLE/2 − (int)(−step·(10 − x))`, where x is the
  phase estimate (in steps) of the best LED slot found in BG
  (`0x180362b00 + ctrl·0x1e8`). The running correction `x − 10` is also
  accumulated per controller (`0x180362ee0`) [sure]; meaning of the
  constant 10 [likely]: the centre of the 21-slot BG scan.
- PRESCAN (retry count ≥ 2): value = 16 683 350 (frame-cycle reset).

Defaults before any controller connects are preset 1.

### 2.5 Set `0x14`: LED-detection window (`FUN_180126b80`, values from `FUN_18019c1a0`, `FUN_180125a10`, `FUN_180127f10`)

16 bytes, sent through the config manager. `FUN_180126b80` (the LD reader
loop) sends it whenever the config version changes [sure].

| Off | Size | Meaning | Startup default | Preset 1 | Preset 2 |
|-----|------|---------|-----------------|----------|----------|
| 0x00 | u32 | A: [guess] intensity threshold or window code | 0x55 (85) | 128 | 85 |
| 0x04 | u16 | B: [likely] LED exposure window, µs (equals the libpad LED pulse width) | 500 | 1000 | 500 |
| 0x06 | u16 | 0 | 0 | 0 | 0 |
| 0x08 | u32 | flags, always `0x24800000 \| 1` | | | |
| 0x0c | u32 | 0 at startup, `0x100` once a preset is applied | 0 | 0x100 | 0x100 |

Per-preset values: tables at `0x18026047c` (A) and `0x180260484` (B),
indexed by preset [sure].

**When it is sent** (`FUN_18019c1a0`, called each frame from
`FUN_180190300`) [sure]:

- The headset echoes the active A and B as **floats** in LD header
  `+0x20` and `+0x24`. Sony compares those echoes with the preset values.
- It re-sends only when an echo differs, the B echo is non-zero, the LD
  format version is > 0x100, and it has not already sent for this preset
  (latch `0x180350d9a`, cleared when the preset owner disconnects).
- A debug override (tracker settings) can force A.

At startup (`FUN_180125a10`) Sony pushes the default block, which equals
preset 2 apart from the last dword.

### 2.6 Set `0x09`: blob-detection mode (`FUN_180192b90`)

24 bytes, sent by `FUN_180126b80` when the config version changes [sure].
The headset echoes most of it in the LD header, so Sony re-pushes it only
when the echo differs, and never on the frame right after a push
(LD `+0x14` counter == last + 1 is skipped) [sure].

| Block off | LD echo off | Normal | Many Blob | Extreme Env (2, 3) | Extreme Env (4) |
|-----------|-------------|--------|-----------|--------------------|-----------------|
| 0x00 | – | u32 0xfe | same | same | same |
| 0x04 | – | u32 0xfe | same | same | same |
| 0x08 | 0x18 | 0x78 | 0xb4 | 0xb4 | 0xff |
| 0x09 | 0x19 | 0x30 | 0x20 | 0x20 | 0x10 |
| 0x0a..0x0d | 0x1a..0x1d | `40 20 80 3c` | same | same | same |
| 0x0e | 0x28 | 0xa0 | 0xb4 | 0xb4 | 0xff |
| 0x0f | 0x29 | 0x04 | same | same | same |
| 0x10..0x13 | 0x2a..0x2d | `0a 20 02 02` | same | same | same |
| 0x14 | 0x1e | 0 | 0 | 0 | 0 |
| 0x15 | 0x1f | 0 | 0 | 1 (mode 2), 2 (mode 3) | 0 |
| 0x16..0x17 | – | 0 | 0 | 0 | 0 |

Meaning [guess]: byte 0x08 and 0x0e are blob intensity thresholds (higher
when the scene is cluttered); byte 0x09 a minimum blob size. LD byte 0x18 >
0xb3 is what the blob decoder reads as "Many Blob active" (already noted in
the main doc) [sure].

### 2.7 Camera mode `0x0b` and brightness `0x12`

- `0x0b` payload `{u32 1, u32 mode}`. Mode is **0x10** from startup
  (`FUN_180125670`); only a VrTrace capture session changes it
  (`FUN_180127ec0`, debug) [sure]. LedSync never changes the camera mode.
  `FUN_180125cc0` re-sends it at most once per second while camera or IMU
  calibration is missing, otherwise only on a config change [sure].
- `0x12` is panel brightness and plays no part in LED sync [sure].

### 2.8 LD packet fields used by LedSync (`FUN_18018aaf0`, `FUN_180192b90`)

Extends the main doc's `"LD"` table.

| Off | Type | Use |
|-----|------|-----|
| 0x08 | u32 | Headset µs of the frame. Mapped to host µs (`FUN_180124a80`); if the map is not ready, Sony uses host now − 6000 µs [sure]. Also the time source for PRESCAN base_time (§5.4). |
| 0x0c | u32 | Four 2-bit per-camera flags (bits 0–1, 8–9, 16–17, 24–25). Cameras 0 and 1 drive the Many-Blob switch [sure]; meaning [guess]: saturation/overload. |
| 0x14 | u32 | Frame counter; used to rate-limit `0x09` pushes [sure]. |
| 0x18..0x1f, 0x28..0x2d | u8 | Echo of the active `0x09` block (table above) [sure]. |
| 0x20, 0x24 | f32 | Echo of the active `0x14` A and B [sure]. |
| 0x35 | u8 | Non-zero: no detection this frame. If see-through is on, Sony then switches both controllers to LED ALL OFF [sure]. |
| 0x40 + n·0x2404 | u32 | Blob count of camera n; counts of cameras 0 and 1 drive Many-Blob (§4.3) [sure]. |

Sony waits up to 100 ms for a new LD packet each tracker cycle
(`FUN_180128590(dev, last_seq, 100)`) and works only on a packet whose
sequence number changed [sure].

### 2.9 LED tables (layout only)

Read them from the user's DLL with the script in §6. Do not commit the
values.

- **Logical-to-physical LED map**, `0x180260aa0`: 48 rows × 24 bytes. Row
  = `3·model + hand` (hand 0 = right, 1 = left). Byte k is the physical LED
  index (0..24) for logical LED k. The tracker sends physical indices in
  led[0..3] [sure] (`FUN_180194af0`, `FUN_18019bf10`). Rows 3/4, 6/7, …,
  45/46 are filled (models 1..15).
- **Sync-eligible LEDs**, `0x1802604a0`: 48 rows × 32 bytes, same row index,
  0/1 per logical LED: may this LED be the BROAD/BG sync LED [sure]
  (`FUN_1801999f0`).
- **BROAD blink codes**, `0x180260390`: 11 rows (frames) × 14 columns
  (codes), values 0/1 [sure] (`FUN_1801957d0`). Structure, which a driver
  needs: frames 0–4 off in every code, frames 5–7 on in every valid code
  (the "sync marker"), frames 8–10 a 3-bit Gray code. Only the even columns
  0, 2, …, 12 are valid codes (odd columns are all zero). The decoded column
  is the coarse slot.
- **BG check pattern**, `0x180260430` (0x20 bytes): the expected on/off
  history at fixed frames of the BG window before a brightness estimate is
  accepted [sure]; exact frame mapping [likely].
- **Model selection** (`FUN_180190300`, `FUN_18019c800`): controller info
  `+0x3c` hardware type 1→model 0xb, 2→0xc, 3→0xd, 4→0xe, 5/6/8/9/10/11→0xf;
  type 0 → model 0 and the controller is not tracked [sure].
- **The 0x60-byte block before each LED model** (`0x180257220` + m·0x1400):
  9 ints, 2 floats, 5 ints, 1 float, 2 ints, 2 floats, 3 zero dwords. It is
  identical in all 7 models of this build. It is pose-solver tuning (counts,
  ratios, pixel limits), consumed by the tracking topic, not by LedSync
  [likely].

## 3. Sequences

### 3.1 Controller connect to locked tracking (preset 1, first attempt)

Times are host µs from the moment the controller first appears. "cmd" means
a message to the libpad block (§2.1); libpad puts the result into the next
output report.

1. **Connect.** The controller info block reports preset ≠ 0. The first
   connected controller fixes the global preset (§2.4); a mismatching second
   controller stays in INIT. The model is chosen (§2.9) and the state is set
   to INIT [sure] (`FUN_180190300`, `FUN_18019b9e0`, `FUN_18019c800`).
2. **Detection config.** Once the LD version is > 0x100 and the echo
   differs, push Set `0x14` with the preset values (§2.5) [sure].
3. **INIT → PRESCAN**, next tracker frame: cmd 1 `[01 01 28 L0 L1 L2 L3]`
   (len 7; period_id 40). libpad sets base_time from the newest LD frame
   time (§5.4) and refreshes it every ≥ 1 s, bumping seq_id each time. The
   report's LED bytes are `ff` in this phase [sure].
4. **PRESCAN → BROAD:** the controller has ≥ 1 associated blob with weight
   > 1e-6 on every frame for ≥ 1 s (3 s after two or more failed BROAD
   rounds). cmd 1 `[01 02 32 L0..L3 00000000]` (len 11; period_id 50,
   offset 0) [sure] (`FUN_1801971f0`, `FUN_180195e20`).
5. **BROAD.** Each frame: choose the sync LED (§5.1) and send cmd 2 when the
   set changes. Each camera decodes the 11-frame blink history into a slot
   (§5.2). When the same best slot comes out of 3 consecutive frames:
   - slot ≠ 4: cmd 4 with offset `(slot − 4) · step · 5 / 2` µs (integer);
   - once the BROAD drift fit has ≥ 10 samples: cmd 3 (drift) or cmd 5
     (drift + offset) instead (§5.5).
   Slot 4 ("centred") held continuously for ≥ 3 s (13 s on retries) → BG.
   No usable code for 10 s → back to PRESCAN, retry counter +1 [sure]
   (`FUN_180198240`).
6. **BG:** cmd 1 `[01 03 1e L0..L3 value]` (period_id 30). value = 630 µs
   when coming from BROAD, −945 µs when coming from STABLE. Each frame the
   brightness estimator gives a sub-slot phase x (§5.3), target 10:
   - x outside [9, 11]: cmd 4 with `(10 − x) · step` µs;
   - after 10 samples (40 on later rounds) and a non-zero slope: cmd 3/5.
   Locked when the drift fit has run twice, |slope| < 2e-6, and BG has been
   good continuously for > 15 s (3 s when re-entering BG within 300 s of a
   STABLE run) → STABLE. Bad for ≥ 13 s → PRESCAN [sure] (`FUN_1801975e0`,
   `FUN_180196ff0`).
7. **STABLE:** cmd 1 `[01 04 0c ff… value]`, value = `945 + step·(10 − x)`
   (§2.4). LED bytes in the report are `ff` [sure]. While the controller
   stays visible, every 100 s it drops back to BG to re-check the phase.
   Invisible for ≥ 15 s → PRESCAN [sure] (`FUN_1801992b0`).

Best case from connect: about 1 s PRESCAN + a few frames to decode + 3 s
centred + 15 s BG ≈ 20 s to STABLE. Tracking does not wait for STABLE; it
works from BROAD onward with a wider window [likely].

### 3.2 Forced transitions (each frame, `FUN_180190300`, `FUN_18018aaf0`) [sure]

| Condition | Action |
|-----------|--------|
| Controller owned by another process (pid in the info block) | INIT |
| Tracker disabled (setting `0x18035b9f3` = 0) or tracker suspended | LED ALL OFF (5) |
| Controller not connected (preset byte 0) | INIT |
| Preset ≠ global preset | INIT, log "SHOULD BE UPDATE CONTROLLER FW" once |
| Controller info `+0x16` non-zero | LED ALL OFF |
| Headset see-through on (`+0x4e6e`, Get `0x97` / Set `0x17`) | LED ALL OFF for both; INIT when it ends |
| > 5 s since this controller's last tracker pass | INIT |
| Controller connection id (info `+0x08`) changed | INIT |
| Transport mode (info `+0x68`) went from 2 to 1 while in phases 1..4 | INIT |
| Phase hold = 1 (below; Extreme-Env mode 3 sets it, §4.3) and phase ≠ PRESCAN | PRESCAN |
| Phase hold = 2 and phase is BG or STABLE | PRESCAN |
| Phase hold = 3 and phase is STABLE | BG |
| Phase 0, 5 or 6 at end of checks | PRESCAN |

**Phase hold** (tracker setting byte `+0x05`, `0x18035b9f5`, read through
`FUN_180189c20`) [sure]. A debug knob that pins every controller to one
phase. 0 = off (the default; `FUN_18018a7e0` zero-fills it). 1 = PRESCAN,
2 = BROAD, 3 = BG, 4 = STABLE. Besides the forced moves above, each phase
handler skips its normal exit while the hold names that phase:

| Hold | Exit it blocks | Handler |
|------|----------------|---------|
| 1 | PRESCAN → BROAD | `FUN_1801971f0`, `FUN_180199150` |
| 2 | BROAD → BG (slot 4 centred), BROAD → PRESCAN after 10 s | `FUN_180198240` |
| 3 | BG → STABLE, BG → PRESCAN after 13 s | `FUN_1801975e0` |
| 4 | STABLE → BG every 100 s, STABLE → PRESCAN after 15 s invisible | `FUN_1801992b0` |

Only Extreme-Env mode 3 writes the hold from inside the DLL (§4.3). Any
other value has to come through the shared tracker-settings block
(`notdone.md` §6).

## 4. State machines

### 4.1 Phases (per controller; `0x180362ea0[hand]`; names at `0x180268d20`)

| # | Name | period_id | cmd 1 value | Report LEDs | Handler |
|---|------|-----------|-------------|-------------|---------|
| 0 | INIT | – | (no command) | – | – |
| 1 | PRESCAN | 40 / 40 | none (len 7) or 16 683 350 (len 11, retries ≥ 2) | ff | `FUN_1801971f0` inline |
| 2 | BROAD | 50 / 42 | 0 | sync LEDs | `FUN_180198240` |
| 3 | BG | 30 / 30 | 21·step − BG/2 or STABLE/2 − BG/2 | sync LEDs | `FUN_1801975e0` |
| 4 | STABLE | 12 / 12 | BG/2 − STABLE/2 + step·(10 − x) | ff | `FUN_1801992b0` |
| 5 | LED ALL OFF | – | cmd 1 len 2 | ff | – |
| 6 | LED ALL ON | – | cmd 1 len 2 | ff | not sent in normal use [likely] |

period_id is "preset 1 / preset 2".

Each phase has a two-value sub-state (`0x180362ea8`): 0 = "not good yet",
1 = "good". A frame result of "good" or "bad" either flips the sub-state
(and restarts its timer) or extends it. The timer is the elapsed host time
in the sub-state (`0x180362eb0`). The timeouts above compare that timer.

### 4.2 Bookkeeping on entering a phase (`FUN_1801943d0`) [sure]

- **INIT:** clear the LED history items, count the entry.
- **PRESCAN:** clear histories. If coming from BROAD, retry counter
  (`0x180362f34`) += 1, saturating at 2. The retry counter picks the slow
  timeouts and the len-11 PRESCAN (frame-cycle reset).
- **BG:** clear the BROAD stage. Remember whether it came from STABLE. If
  the last STABLE drift fit is younger than 300 s, keep it (fast re-lock),
  else clear it.
- **BROAD:** clear the 3-frame slot history, the per-camera slots, the
  offset accumulator and the BROAD drift fit.
- **STABLE:** count STABLE entries.
- **LED ALL OFF / ON:** clear histories; retry counter := 2.
- A successful drift correction (§5.5) resets the retry counter to 0.

### 4.3 Blob-detection mode (headset-wide, tracker `+0x4f474`, `FUN_180192b90`) [sure]

Runs once per LD packet with version > 0x100.

| Mode | Name | Enter | Leave |
|------|------|-------|-------|
| 0 | Normal | start; or Many Blob quiet for 480 frames | camera 0 or 1 reports ≥ 200 blobs, or its flag bits are set → 1 |
| 1 | Many Blob | from Normal; or from Extreme after 180 quiet frames | ≤ 49 blobs on both cameras for 480 frames → 0 |
| 2 / 3 / 4 | Extreme Env | see below | 180 frames in a row without camera 0/1 flags → 1 |

Extreme-Env entry, checked every packet, any current mode:

- tracker settings byte `+0x34` non-zero: enter at once;
- else, in Many Blob only: settings byte `+0x33` non-zero and camera 0 or 1
  flag bits set on 5 packets in a row (counter `0x18035be28`; any packet
  that fails the test resets it).

Which mode: `+0x33` = 2 → mode 3, 3 → mode 4, any other value (0 and 1
included) → mode 2. While `+0x34` stays set, the mode is re-entered on
every packet, so it never times out.

"Quiet" for the Many-Blob exit means both blob counts ≤ 49. The camera
flags do not count there unless `+0x33` is non-zero (then a flagged packet
resets the quiet count and feeds the Extreme-Env entry instead). After a
Many-Blob exit the Normal test runs on the same packet, so a flagged packet
goes straight back to Many Blob: the mode at the end of the packet is
unchanged, nothing is pushed, and only the two log lines and the switch
counter show it. "Quiet" for the Extreme-Env exit means camera 0/1 flag
bits clear; blob counts do not matter there.

The two Extreme-Env settings bytes default to 0, so a normal install only
uses Normal ↔ Many Blob [likely]. Each mode has its own `0x09` block (§2.6).

Mode 3 sets the phase hold (§3.2) to 1, so every controller sits in
PRESCAN while mode 3 lasts. Leaving mode 3 for Many Blob clears the hold
to 0. Leaving modes 2 or 4 leaves it as it was [sure].

Bookkeeping in the tracker object [sure unless tagged]:

| Off | Type | Meaning |
|-----|------|---------|
| `+0x4d24c` | u32 | Mode class: 0 Normal, 1 Many Blob, 2 any Extreme mode |
| `+0x4d250` | f32 | Seconds in the current mode: +1/60 per packet (+1/30 in mode 3 [guess: the headset halves the LD rate there]) |
| `+0x4d254` | f32 | Quiet time towards the exit: quiet counter × the same step |
| `+0x4d258` | u32 | Changes into Many Blob |
| `+0x4d25c` | u32 | Changes into an Extreme mode |
| `+0x4f3b8..+0x4f3c4` | u32 ×4 | Per-camera flag events (cameras 0..3), at most one per 60 s each; last-event host µs (u64 ×4) at `+0x4f448..+0x4f460` |
| `+0x4f3c8` | u32 | Normal → Many Blob switches |
| `+0x4f3cc` | u32 | Passed Extreme entry checks (counts every packet while `+0x34` is set) |
| `+0x4f474` | u32 | Mode |

A mode change clears both f32 timers. After the mode logic, the function
compares the LD echo with the block for the new mode and, when it differs
and the frame is not the one right after the last push, hands the 24 bytes
to the config manager (`FUN_18015e720`), which `FUN_180126b80` then sends
(§2.6).

## 5. Algorithms

### 5.1 Choosing the sync LED (`FUN_1801999f0`) [sure unless tagged]

```
N = sync_led_count            // tracker setting 0x18035ba1e, default 1
if this controller has no associated blobs: status 3, keep nothing
if the last success is < 1 s old:
    keep the current LED set; just pick the camera that sees it best
else:
    for each camera c:
        candidates = LEDs with eligible[row][led] and a matched blob of
                     state 1 and angle_metric·0.5 > ~0      // floats at +0x15bc
        keep the N with the smallest metric, each ≤ 45      // [likely] degrees
    choose the camera with the most kept LEDs; ties: smaller metric sum
if none: status 4 ("no candidate")
else: in BROAD/BG, light exactly those LEDs (cmd 2, physical indices via
      the 0x180260aa0 map, unused bytes ff); store per-camera count and
      mean blob weight
```

The angle metric is the LED-to-camera viewing angle from the predicted pose
[likely]; so the sync LED is the one facing the camera most directly.

### 5.2 BROAD blink-code decoder (`FUN_1801957d0`, `FUN_18019c500`) [sure]

Per camera, keep a history of the last 11 frames of the sync LED: 2 = seen,
0 = not seen, 1 = uncertain. A code matches when every frame equals the
table bit (2 ↔ 1, 0 ↔ 0); any uncertain frame fails the match. The matching
even column is the slot (0..12); none → −1. Across the 4 cameras pick the
slot closest to the previous accepted slot (within 32). Status codes per
camera feed the failure strings (§6).

Accept rule (`FUN_180198240`): same slot in 3 consecutive frames; then
`|slot − 4| > 4` counts as GRID SKIP, and 6 of those reset BROAD.
Otherwise `offset_us = (slot − 4)·step·5/2`, so one Gray step (2 slots) is
5·step = 525 µs (preset 1) / 250 µs (preset 2) [sure].

Meaning [likely]: during BROAD the controller sweeps its LED pulse across
the window in 5-step bins and blinks the bin index; the headset sees the
pulse only in bins that overlap the exposure.

### 5.3 BG brightness phase (`FUN_1801954f0`, `FUN_1801975e0`) [likely]

After the BG check pattern matches, the estimator takes the blob weights
of the sync LED over the BG window, subtracts the median-ish floor
(`(a+b)·0.5`), and finds the brightest sub-slot by a weighted centre of
the samples above the floor (states "NO MAX MIN", "NO DIF MAX MIN" when no
peak). The result x is in steps; the exposure centre is x = 10. An x that
jumps by > 1 from the last accepted value is skipped up to 3 times
(`0x180363110`) [sure]. The phase used for the drift fit is
`x + accumulated corrections` (`0x180362ee0`) [sure].

### 5.4 PRESCAN base time (libpad `FUN_1801c1880`, `FUN_1801c27d0`) [sure]

```
t_hdr  = newest LD packet +0x08 (headset µs)
t_host = headset_to_host(t_hdr)               // FUN_180124a90, headset.md
t_ctrl = host_to_ctrl(t_host)                 // FUN_1801c09e0: (drift·off + (t−t0)·1e9)/(drift+1e9)
base_time = t_ctrl − period_id·step/2 − width/2
```

The time source is a callback libpad receives at init (`FUN_1801c1800`,
set to `FUN_180118920`). In PRESCAN libpad repeats this every ≥ 999 999 µs
(`FUN_1801c27d0`) and increments seq_id each time. In BROAD/BG/STABLE the
value field is a signed offset, clamped to ±(15·step).

### 5.5 Drift fit (`FUN_180194180`, `FUN_180193f70`, `FUN_18019caf0`, `FUN_18019bc00`) [sure unless tagged]

Two fit objects per controller: one for BROAD (`0x180362ee8`), one shared
by BG and STABLE (`0x180362ef8`).

- Each new value is pushed with its time since the first sample. Equal
  values (within DBL_EPSILON) merge into one run (count +1, keep the
  earliest time).
- Once there are ≥ 4 runs, fit once (latched until reset):
  `slope = (n·Σ(v·t) − Σv·Σt) / (n·Σt² − (Σt)²)` over all runs except the
  first, t in µs.
- Correction (`FUN_18019bc00`): if `|slope|` > DBL_EPSILON, send cmd 3
  `[03 f32 slope]`, or cmd 5 `[05 f32 slope, s32 offset_us]` when an offset
  is also pending; reset the retry counter.
- libpad applies `cycle := clamp((1 + slope)·cycle)`.

Units [guess]: v is in slot/step units and t in µs, so the slope is not
dimensionless; either the value pushed is already in µs or Sony's scaling
relies on the clamp. Needs a capture of real cmd 3 values.

## 6. Constants

| Name | Value | Meaning | Source |
|------|-------|---------|--------|
| Nominal frame cycle | 16 683 350 | 59.94 Hz frame in ns [likely]; reset value | `FUN_180194af0`, `FUN_1801c1880` |
| Frame-cycle clamp | 15 683 350 … 17 683 350 | | `FUN_1801c08f0` |
| period_id clamp | ≤ 60 | | `FUN_1801c0980` |
| step | 105 / 50 µs | preset 1 / 2 | `0x18026035c` |
| windows PRESCAN/BROAD/BG/STABLE | 4200/5250/3150/1260, 2000/2100/1500/600 µs | | `0x180260364..37c` |
| libpad width/step/half | 1000/105/1575, 500/50/750 | sync mode 1 / 2 | `0x180268dc0`, `FUN_1801c1820` |
| PRESCAN → BROAD | 1 s, 3 s on retry | | `0x180260450` |
| BROAD centred → BG | 3 s, 13 s on retry | | `0x180260460` |
| BG good → STABLE | 15 s, 3 s on fast re-lock | | `0x180260470` |
| BROAD fail → PRESCAN | 10 s | | `FUN_180198240` |
| BG fail → PRESCAN | 13 s | | `FUN_1801975e0` |
| STABLE lost → PRESCAN | 15 s | | `FUN_1801992b0` |
| STABLE re-check → BG | 100 s | | `FUN_1801992b0` |
| Fast re-lock age | 300 s | | `0x180350cf8` |
| Watchdog | 5 s without tracker pass → INIT | | `FUN_180190300` |
| PRESCAN refresh | ≥ 1 s | | `FUN_1801c27d0` |
| BROAD accept | 3 equal frames; GRID SKIP limit 6; slot range 0..8 | | `FUN_180198240` |
| BG target / dead band | x = 10, ±1 | | `0x180319954`, `0x180319968`, `0x18031994c` |
| BG drift samples | 10 first, 40 later | | `0x180260490` |
| Lock slope | < 2e-6 | | `0x180261148` |
| Re-lock offset reset | ≥ 40 µs | | `0x180350d08` |
| Sync LED angle limit | 45 | [likely] degrees | `0x18026114c` |
| Blob present weight | > 1e-6 | | `0x180319720` |
| Many Blob enter / leave | ≥ 200 / ≤ 49 blobs for 480 frames | | `FUN_180192b90` |
| Extreme Env leave | 180 frames | | `FUN_180192b90` |
| LD wait | 100 ms | | `FUN_18018aaf0` |
| LD time fallback | now − 6000 µs | | `FUN_18018aaf0` |

Failure strings (status shown by Sony's UI, `0x180350d20`…): ALL NOTHING,
NOT DETECT 2LED, GRID SKIP, NO HIT, NO SYNC MARKER, NO 3MATCH, NO MAX MIN,
NO DIF MAX MIN, FRAME DROP (`FUN_180195c30`). They are diagnostics only.

Reading the tables from your own DLL (our code, prints shapes and
values locally; do not commit the output):

```python
import struct, sys
sys.path.insert(0, "scripts/sony-re")
from extract_led_models import reader
read = reader(sys.argv[1])
rd = lambda va, f: struct.unpack("<" + f, read(va, struct.calcsize("<" + f)))
for p in (1, 2):
    print("preset", p, "step", rd(0x18026035c + 4*p, "f")[0],
          "windows", [rd(a + 4*p, "i")[0] for a in (0x180260364, 0x18026036c, 0x180260374, 0x18026037c)],
          "0x14 A,B", rd(0x18026047c + 4*p, "i")[0], rd(0x180260484 + 4*p, "i")[0])
print("timeouts", rd(0x180260450, "6Q"))
codes = read(0x180260390, 11 * 14)            # [frame][code]
ledmap = [read(0x180260aa0 + r*24, 24) for r in range(48)]   # row = 3*model + hand
eligible = [read(0x1802604a0 + r*32, 32) for r in range(48)]
```

## 7. Gaps in the current Linux driver

| Area | State | Needed |
|------|-------|--------|
| EP `0x89` LD | `kernel/psvr2_aux.c` drains and discards it | Deliver packets + arrival time to userspace (chardev or shared ring) |
| Set `0x14`, `0x09` | not sent anywhere | ioctl/sysfs to send; userspace policy §2.5, §2.6, §4.3 |
| Camera mode | `kernel/psvr2_camera.c` uses mode 1 for V4L2, `0x10` known | Keep `0x10` while tracking; V4L2 mode 1 streaming would break LED detection [likely] |
| Headset clock map | described in `headset.md` | Needed for base_time |
| libpad sync block | `userspace/lib/sie/libpad/psvr2_hidsync.c` `psvr2_ledsync_command`, `psvr2_sense_output.c` | Present and matches §2.1–2.3. Needs a caller and a `psvr2_sync_headset_time` hook fed from LD +0x08 |
| LedSyncer state machine | absent | §3, §4 |
| LED choice, BROAD decoder, BG estimator, drift fit | absent | §5; needs the blob-to-LED association from the tracking topic |
| LD decode | `userspace/lib/sie/irtracker/psvr2_ld.c` decodes blobs | Also read header fields of §2.8 |
| SteamVR driver | `steamvr/driver_psvr2/src/sense_protocol.h` has the LedSettings fields | Fill them from the libpad block |

## 8. Open questions

1. **Units of base_time vs 1frame_cycle.** Both go into the report ×3.
   16 683 350 only fits a 60 Hz frame in ns, but base_time is computed in
   controller µs. Either the controller clock is not µs or one ×3 means
   something else. Capture: output reports in PRESCAN plus the controller
   input timestamps.
2. **Drift slope units** (§5.5). Capture real cmd 3/5 floats from a Sony
   USB/BT trace, or call `FUN_18019caf0` under Wine with test data.
3. **What A in Set `0x14` means** (128 vs 85) and the bytes of `0x09`.
   Capture: vary them and watch the LD blob counts.
4. **LED behaviour in PRESCAN and STABLE** (report LED bytes `ff`). The
   firmware presumably lights its full constellation; confirm with a camera
   capture.
5. **BG check pattern frame mapping** and the exact BG estimator (§5.3) —
   medium value; a hardware log of `phase*` lines from Sony's driver (libpad
   logs every command) would pin it down.
6. **Controller info `+0x16`** (forces LED ALL OFF) — which input-report
   bit feeds it.
7. **Sync LED count** setting default is 1 (`0x18035ba1e`); the failure
   string "NOT DETECT 2LED" suggests 2 was used at some point. Check what
   the config manager loads at runtime.

## 9. Function map

| VA | Role | Covered |
|----|------|---------|
| `FUN_18018aaf0` | IR tracker loop; LD wait, LD header use, calls LedSync | LD parts |
| `FUN_180190300` | Per-frame per-controller checks and phase dispatch | yes |
| `FUN_1801971f0` | Phase handler dispatch + PRESCAN logic | yes |
| `FUN_180198240` | BROAD handler | yes |
| `FUN_1801975e0` | BG handler | yes (estimator medium) |
| `FUN_1801992b0` | STABLE handler | yes |
| `FUN_180199150` | Alternate PRESCAN handler (same rules) | yes |
| `FUN_180194af0` | Enter phase, build and send cmd 1 | yes |
| `FUN_1801943d0` | Phase-entry bookkeeping | yes |
| `FUN_1801999f0` | Sync LED choice, per-camera counts | yes |
| `FUN_18019bf10` | Send cmd 2 (LED set) | yes |
| `FUN_1801957d0`, `FUN_180195af0` | BROAD blink-code match / BG pattern check | yes |
| `FUN_1801954f0` | BG brightness estimator | partly |
| `FUN_180195e20` | Count associated blobs | yes |
| `FUN_180196ff0` | BG drift step | yes |
| `FUN_180194180`, `FUN_180193f70`, `FUN_18019caf0` | Drift fit push / fit | yes |
| `FUN_18019bc00` | Send cmd 3 / cmd 5 | yes |
| `FUN_18019b9e0`, `FUN_18019c3d0` | Preset ownership and derived values | yes |
| `FUN_18019c800`, `FUN_18019c870` | Model select / clear | yes |
| `FUN_18019c1a0` | Choose and push Set 0x14 values | yes |
| `FUN_180192b90` | Blob-detection mode, Set 0x09 block | yes |
| `FUN_180126b80` | Send 0x14/0x09, read EP 0x89 | yes |
| `FUN_180125cc0`, `FUN_180125670`, `FUN_180127ec0` | Camera mode 0x0b | yes |
| `FUN_180125a10`, `FUN_180127f10` | 0x14 defaults / setter | yes |
| `FUN_18011b3c0` | Tracker → libpad command bridge | yes |
| `FUN_1801c0340`, `FUN_1801c1880` | libpad command receiver | yes |
| `FUN_1801c1820` | libpad sync-mode set | yes |
| `FUN_1801c27d0` | libpad PRESCAN refresh | yes |
| `FUN_1801c08a0`, `FUN_1801c08f0`, `FUN_1801c0980` | libpad clamps | yes |
| `FUN_1801c09e0` | host → controller time | yes |
| `FUN_180118920`, `FUN_1801c1800` | headset frame time callback | yes |
| `FUN_180124a90` | headset µs → host µs | referenced |
| `FUN_1801cbeb0` | Output report fill | LedSync fields |
| `FUN_1801c8210` | host µs clock (QPC) | yes |

## Corrections to the main doc

`docs/sony-driver-re.md`, section "LED sync":

1. Cmd 1 layout is `[01, phase, period_id, led0..3, u32 value]`, len 11
   (len 7 for PRESCAN without frame-cycle reset, len 2 for phases 5/6). The
   doc lists only phase + 4 LED bytes.
2. Tracker cmd 5 is "drift + offset" (9 bytes), not "sync-loss handling".
   Phases 5/6 go out as cmd 1 len 2. Cmd 6 is ignored by libpad. Cmd 7 is
   sub 1 = LED off, sub 2 = full reset, sub 3 = perf stats, not "mode 1..3".
3. Cmd 3's "drift exceeds a threshold": the threshold is DBL_EPSILON, so any
   non-zero fitted slope is sent.
4. Output report `+0x14`: written when phase ∉ 1..4 **or the controller
   clock fit has a reference time**, not "a sync request is pending".
5. `0x14` "Sony's defaults" 0x55/500 are startup values only; the active
   values come from the preset (128/1000 or 85/500), and dword 4 becomes
   0x100. LD `+0x20`/`+0x24` are the float echoes of A and B, not "exposure/
   sync phase".
6. LED byte values are physical LED indices from the map at `0x180260aa0`
   (row `3·model + hand`), not a per-(model, hand, slot) table of patterns.
7. Commands need the LedSyncer context state 4 (ready) and hand 0 or 1;
   hand 0 = right, 1 = left.
