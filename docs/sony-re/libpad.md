# libpad: Sony's controller library inside the PS VR2 SteamVR driver

Behavioural spec of the libpad part of `driver_playstation_vr2_orig.dll`
(PE timestamp 1741940190, VA range `0x1801b0000–0x1801dffff`), written for a
clean reimplementation in `userspace/lib/sie/libpad/`. Method and
conventions: [../sony-driver-re.md](../sony-driver-re.md). Equivalence tests:
`scripts/sony-re/difftest/t_libpad_*.c`; per-function status:
`scripts/sony-re/status/libpad.tsv`.

Error codes: `0x80010002` = library not initialised (lock unavailable),
`0x8001002d` = no such pad handle, `0x80920001` = bad argument.

## Global state

- **Pad table**: 8 slots of 0x3410 bytes. A slot is in use when byte 0 is
  non-zero; the u32 at +0x1c is the public handle. Lookup by handle walks
  the slots in order (`FUN_1801c9380`). Other lookups key on +0x10 and +0x14.
- **Device (HID) table**: 12 slots of 0x2200 bytes, one reader thread each.
  Byte +0x1e4 of a device slot is the hand: 1 = left.
- **Lock**: one library lock (`FUN_1801d4280`/`FUN_1801d42b0`) and one for the
  device table (`FUN_1801d42d0`/`FUN_1801d4300`); both are no-ops returning
  `0x80010002` until `FUN_1801d4320` initialises them. The reimplementation
  has no locks in the pure helpers; callers serialise.

## Device families (`FUN_1801c9440`…`FUN_1801c9690`) [sure]

All require VID `0x054c`.

| Predicate | True for |
|-----------|----------|
| VR (Sense) | PID `0ce7`, `0e45`, `0e46` |
| VR left | VR and (PID `0e45` or hand byte = 1) |
| VR right | VR and (PID `0e46` or hand byte = 0) — so `0e45` with hand 0 counts as right |
| DualSense | `0ce6` or `0df2` (Edge); separate predicates for `0ce6` only and `0df2` only |
| PS5 family | DualSense or VR |
| DS4 (new HID) | `05c4` or `09cc` with HID version > 22 |
| DS4 v2 firmware | `09cc`, firmware ≥ `0x3000` and high nibble not 3, 4, 5, 8 or 9 |
| `0ba0` | the USB wireless adapter id |

## Report CRC (`FUN_1801d4390`, `FUN_1801d43f0`, `FUN_1801d4490`) [sure]

Bluetooth reports end in a CRC-32 (IEEE, reflected, poly `0xedb88320`,
init `~0`, final `~`) computed over one seed byte followed by the report
without its last 4 bytes. The CRC is stored little-endian in those 4 bytes.
Seeds: `0xa1` input, `0xa3` get-feature (DS4/DualSense calibration over
BT). `FUN_1801d43f0` returns non-zero when the stored CRC does not match;
`FUN_1801d4490` is the raw table update with no pre/post inversion.

## Input path (`FUN_1801ce8e0`, one thread per HID device) [sure]

1. Gets the HID caps. Input report length 0x40 means USB, 0x4e (78) means
   Bluetooth (only when BT is enabled). Reads up to 32 reports per call
   (buffer `len·32`).
2. For a VR controller it reads feature `0x05` repeatedly (64 bytes) until bit 7
   of byte 1 is set, merging each part by its index (byte 1 & 0x7f) into
   the calibration (see [../sony-driver-re.md](../sony-driver-re.md)), then
   registers the pad (`FUN_1801c7b10`).
3. Read loop, 2 ms sleep between reads. A read returns `k` reports (USB:
   length multiple of 64; BT: multiple of 78). Each gets a host receive
   time: the first read uses `now − (k−1)·4000 µs` for VR (1000 µs for
   DualSense, 2000 µs over BT), later reads interpolate linearly between the
   previous read time and now: report j (1-based) gets
   `prev + (now − prev)·j/k`.
4. Bluetooth reports whose first byte is not `0x01` must pass the CRC check
   with seed `0xa1` over 78 bytes; failures are dropped.
5. VR reports go to the decoder (below), then to the per-pad sample store
   and clock fit (`FUN_1801c6ee0`).
6. Once a second the pad's output state is refreshed (`FUN_1801c8d50`), and
   the output-report path is kicked at the configured interval
   (`FUN_1801c1720`, intervals 501 000 µs or 8 000 µs).

## Sense input report (`FUN_1801d1d10`) [sure]

`r` = report start + 1 on Bluetooth (BT has one extra header byte), so `r[0]`
is the report id (`0x01` USB, `0x31` BT). The decoder ORs button bits into
a zero-initialised record (`psvr2_sense_input`):

| Report | Decoded field |
|--------|---------------|
| `r[1]`, `r[2]` | stick X, Y. Left hand: left-stick slots, right stick = `0x80,0x80`; right hand: the reverse |
| `r[3]` | trigger (L2 slot on the left, R2 slot on the right) |
| `r[4]` | trigger proximity / touch value |
| `r[5]` | grip proximity value |
| `r[7]` | 8-bit report counter |
| `r[8]` bits 0..7 | libpad buttons `0x8000, 0x4000, 0x2000, 0x1000, 0x400, 0x800, 0x100, 0x200` (□ × ○ △ L1 R1 L2 R2 in PlayStation numbering [likely]) |
| `r[9]` bits 0..4 | `0x1, 0x8, 0x2, 0x4, 0x10000` (create, options, L3, R3, PS [likely]) |
| `r[10]` low nibble, `r[9]` bit 7 | touch/capacitive bits: `r[10]<<4 | (r[9]&0x80 ? 8 : 0)` |
| `r[0x10..0x15]` | gyro x,y,z (s16) |
| `r[0x16..0x1b]` | accel x,y,z (s16) |
| `r[0x1c]` | u32 IMU timestamp, 1/3 µs ticks |
| `r[0x20]` | temperature |
| `r[0x28]` | trigger-effect state (stored per hand) |
| `r[0x29]` | trigger-effect mode |
| `r[0x2a]` | battery byte |
| `r[0x2c]` | u32 **echo**: the host timestamp (µs, low 32 bits) from the last output report the controller received |
| `r[0x30]` | u32 device timestamp, 1/3 µs ticks; 0 means "use `r[0x1c]`" |
| `r[0x34]` | bit 0 and bit 3 kept as two flags |

Not read by Sony: `r[6]`, the u32 at `r[0x0c]` (Monado's `seq_no`), `r[0x2b]`
(plug state), `r[0x35..]`.

**Mismatches with `steamvr/driver_psvr2/src/sense_protocol.h`** (InputCommon
offset = `r` offset − 1): all named fields line up. Additions: `unknown1[1]`
(`r[7]`) is the counter Sony uses; `unknown4[0]` (`r[0x34]`) carries two
flags; `buttons[2]` low nibble is touch data, not buttons. `host_timestamp`
is an echo of the output report's `host_timestamp_us`, which is what makes
the clock sync below possible.

## Controller clock fit, "HID_SYNC" (`FUN_1801c0b90`) [sure]

Each pad keeps a 0x980-byte clock record (pad +0x2a60; layout =
`psvr2_sync_state`). It maps controller time (µs = ticks/3, rounded:
`ticks/3 + (ticks%3 == 2)`) to host time (µs).

**Conversions.**
- controller → host (`FUN_1801c1520`): with the fit enabled (global flag,
  default on) `host = t + fit_off + ((t − ref)·drift)/1e9` (64-bit wrapping
  multiply, signed division); disabled: `host = t − simple_off`.
- host → controller (`FUN_1801c09e0`): fit enabled:
  `ctrl = (drift·ref + (host − fit_off)·1e9) / (drift + 1e9)`, truncated to
  32 bits; fails if offset, ref and drift are all 0. Disabled:
  `ctrl = simple_off + host`.

**Update**, per decoded report, with arguments host receive time `H`
(64-bit µs), echo `E`, ticks `T` (`r[0x30]`), ticks2 `T2` (`r[0x1c]`) and
transport `m` (0 USB, 1 BT):

1. If the requested latency mode (+0x924) changed and `m` = BT, set the
   shared BT limits: mode 0 → latency 500 µs, RTT limit 13 000; mode 1 →
   latency −4 500 µs, RTT limit 20 000. Remember the mode.
2. A tick value smaller than the previous non-zero one sets wrap flag 0x01
   (`T`) or 0x10 (`T2`). Count the call; store `T`, `T2`.
3. If `E` equals the previous echo or is 0, and not both wrap flags are set,
   store `E` and stop (no new round trip).
4. Write sample `idx`: `ticks, host, echo, ctrl_us, rtt = (u32)H − E`.
5. USB: skip the first 2 samples after init. Classify `rtt` (only if below
   the shared USB limit, 23 000): > 14 000 slow, < 7 700 fast, else medium.
   Each class has a saturating run counter (max 21); the other two reset.
   After more than 20 consecutive slow samples the USB bias becomes
   +4 000 µs, fast −1 000 µs, medium 0. Midpoint `mid = H`, correction =
   bias + shared USB latency (4 300).
   BT: `mid = (H + E_full)/2` where `E_full` is `E` with `H`'s upper 32 bits
   (one less if `E` > low 32 bits of `H`); correction = shared BT latency.
6. Sample gets `mid −= correction`, `mid − ctrl_us`, `ctrl_us − mid`,
   `H & ~0xffffffff`.
7. Window: 1 sample on USB, 30 on BT. If both wrap flags are set, remember
   this slot and restart at 0 (and fit now). Otherwise advance; fit only
   when the window is full.
8. Choose the reference sample `S`:
   - USB: the sample with the smallest RTT not above the limit (or the
     remembered wrap sample). None → stop. Record its RTT.
   - BT: sort a copy of the window by `mid − ctrl_us` (selection sort, first
     minimum wins, whole records swapped) and take element 15 (the median).
     Record its RTT; stop if it is ≥ the BT limit. Keep a 3-slot history of
     the key and its deltas.
   - If `S.ctrl_us` equals the last fitted reference, stop.
9. Simple offset: if `ctrl_us(T) − simple_off` lies within
   `[S.host_hi − 500 + E, S.host + 500]` average it with `S.ctrl − S.mid`,
   else replace it.
10. USB only, jump check: `d = S.host − S.ctrl_us`; if the previous `d` and
    this one are non-zero, 0 < fits < 100 and they differ by more than
    2 000 µs, force a reset.
11. Fit. Reset (first time, forced, or jump): `fit_off = S.mid − S.ctrl_us`,
    `drift = 0`, fits = 0, gain switch-over = 100, reset counter++.
    Otherwise predict `P` = controller→host of `S.ctrl_us` (+0x55555555 if
    both wraps happened, then clear the flags), error `e = S.mid − P`,
    fits++ and pick gains by whether fits ≥ switch-over:

    | | USB early | USB late | BT early | BT late |
    |-|-|-|-|-|
    | offset divisor | 50 | 40 | 5 | 100 |
    | drift divisor | 10 000 | 50 000 | 5 000 | 30 000 |

    `fit_off = e/offset_div − S.ctrl_us + P`;
    `drift += (e·1e9 / (u32)(S.ctrl_us − ref)) / drift_div`. If
    `|drift|` > 70 000 ppb, reset as above.
12. `ref = S.ctrl_us`.

The shared limits and latencies (`{23000, 13000}`, `{4300, 500}`) are
process-wide, not per pad. A field at +0x860 gates an alternative BT
selection by host time; nothing in this build sets it, so that path is
dead.

**Clock record init** (`FUN_1801c1660`): all zero, `simple_off =
0x7fffffffffff`, switch-over 100, run counters 20, min trackers ~0, transport
and hand bytes at +0x811/+0x810, statistics block reset (`FUN_1801c0ab0`).

## LED-sync record and commands (`FUN_1801c1820`, `FUN_1801c1880`) [sure]

The 0x30-byte record at pad +0x33e0 (`psvr2_ledsync`) adds to the table in
[../sony-driver-re.md](../sony-driver-re.md): +0x10 last re-sync host time,
+0x18 sync mode, +0x1c window width, +0x20 period step, +0x24 base-time
limit, +0x28 capability-probe count.

**Set sync mode**: probe count = 0, mode stored; mode 1 → width 1000, step
105; mode 2 → width 500, step 50; limit = 15·step.

**Clamps**: period id ≤ 60; frame cycle within 15 683 350…17 683 350 ns;
base time within ±limit. Clamped values are counted and the extreme seen is
kept in the clock record (+0x950…+0x970).

**Commands** (first byte, then payload; `len` includes the command byte).
Commands 1–5 first zero the base time; a wrong length then leaves the record
otherwise unchanged. Every accepted command increments `seq_id`.

| Cmd | Len | Effect |
|-----|-----|--------|
| 1 | 11 (phase 1: 7 or 11; phase 5, 6: 2) | phase = byte 1. Phases 2–4: period = clamp(byte 2), LEDs = bytes 3..6, base = clamp(i32 at 7). Phase 1 (PRESCAN): period and LEDs as before, then read the headset time from the registered callback; if it exists and converts to controller time `c`, `base = c − (period·step)/2 − width/2`, and with len 11 the frame cycle = clamp(u32 at 7). No time or no fit → not counted. Other phases: just stored |
| 2 | 5 | LEDs = bytes 1..4 |
| 3 | 5 | cycle = clamp((f + 1)·cycle), f = float at 1 (float → int64 truncation) |
| 4 | 5 | base = clamp(i32 at 1) |
| 5 | 9 | base = clamp(i32 at 5), cycle = clamp((f + 1)·cycle), f at 1 |
| 6 | – | ignored, not counted |
| 7 | – | sub 1: phase 5 (LED ALL OFF), mode 0. Sub 2: reset the clock record (Sony swaps the transport and hand bytes when doing so) and the LED record (LEDs `ff`, width 1000, step 105, limit 1575, phase 5, cycle 16 683 350, rest 0). Sub 3: start (byte 2 ≠ 0 and not running) or stop a performance measurement; stopping logs statistics and appends a CSV line to `../ledsync/<YYYYMMDDhhmm>.txt` |

Phase names: 0 INIT, 1 PRESCAN, 2 BROAD, 3 BG, 4 STABLE, 5 LED ALL OFF,
6 LED ALL ON.

## Sense output report (`FUN_1801c3ba0` → `FUN_1801cbeb0`) [sure]

Each pad has two 0x3c-byte output buffers (pad +0x2978, +0x29b4) used in
turn: one is being sent, the other may be filled. A refresh zeroes the
buffer, sets byte 0 = `0x02` (USB output report id) and hands 0x30 bytes to
`FUN_1801cbeb0`, which patches the controller-specific fields and queues a
copy for the writer thread (FIFO, semaphore-signalled). Buffer offset N =
`OutputSettings` offset N − 1 in `sense_protocol.h`.

For a VR controller (`psvr2_sense_output_fill`):

1. `+0x10` u32 = host clock (µs, low 32 bits) at fill time. The controller
   echoes it in input `r[0x2c]`.
2. `+0x01` bit 1 when the connection kind (pad +6) is 2 or 3, or either
   library-wide rumble switch is on. Kind 4 or 5: bit 7 when pad firmware
   (+0x50) > 0x220, else bit 1 if pad +0x68 = 1.
3. If `+0x01` bit 4 is still clear: set it and write `+0x23` (Monado's
   `reduction`) = `(11 − trigger_level) << 4 | (8 − haptic_level) & 7`,
   where a result of 11 becomes 10 and 8 becomes 7. The levels are pad bytes
   +0x0b and +0x0a (0 = full strength).
4. Sync mode: if the LED-sync record has no mode yet, use mode 2 when the
   device firmware is below `0x1006d` or 4 probes failed; otherwise probe
   (Set feature `0x80` = `80 13 22 00…`, 64 bytes; Get feature `0x81`;
   mode 2 if the reply starts with `0x81`, else 0) and count the probe.
   Mode 1 is never chosen on this path.
5. First fill after a phase is set (phase ≠ 0, last re-sync time 0): store
   the fill time as the re-sync time. Otherwise run the PRESCAN re-sync
   (`FUN_1801c27d0`): in phase 1, at least 1 s after the last re-sync, read
   the headset time; if it converts to controller time `c`, set
   `base = c − (period·step)/2 − width/2`, bump `seq_id`, store the time.
6. Polling rate (`FUN_1801cdff0`, only when pad +0x3271 is set and the
   device firmware > 0x250): on enable Set feature `0x08` = `08 0e
   <u16 rate> <u16 param>` (values from `FUN_1801c0200`, rate 6…0x540,
   param ≤ rate/2), on disable `08 0e 00 00 00 00`, then read feature 9.
7. `+0x14` phase, written only when phase is 0 or ≥ 5, or the clock fit
   has an offset; `+0x15` seq_id; `+0x16` period_id; `+0x17` u32
   base_time·3; `+0x1b` u32 frame_cycle·3; `+0x1f..+0x22` LED bytes in
   phases 2 and 3, else `ff ff ff ff`.
8. Pad firmware > 0x253: `+0x02` bit 1 and bit 4 from two library-wide
   switches (`FUN_1801d2890`, `FUN_1801d2850`).

The fill time is kept at pad +0x2a58.

### Output buffer protocol (`FUN_1801c6170`) [sure]

Setters (vibration, trigger, LED…) take a buffer: if none is in flight, the
first buffer is zeroed, filled and sent at once ("fresh"); if one is in
flight and no second is pending, the other buffer becomes pending and is
filled; if one is already pending, it is filled further. A setter that
changes nothing claims no buffer. When the writer finishes a send, the sent
buffer is zeroed (0x3c bytes) and the pending one, if any, is sent (0x30
bytes for VR/DualSense, 0x20 otherwise).

### Vibration (`FUN_1801c3020`, VR branch; `psvr2_sense_set_vibration`) [sure]

Ignored while pad +0x0a (vibration strength level) is 0. Writes byte 0 =
`0x02`, `+0x01 |= 0x40`, then `|= 0x01` for connection kind 2 or 4, or
`|= 0x20` for kind 3 or 5 or when a library rumble switch is on. A fresh
buffer always gets `|= 0x01`. `+0x03` = amplitude: **only the second
(small-motor) argument is used** on a VR controller. A non-zero call marks
the pad "vibrating" (+0x2a00) with the host time (+0x2a08).

### Trigger strength (`FUN_1801c4a30`, VR; `psvr2_sense_set_trigger_level`) [sure]

Stores the new level at pad +0x0b. 0 → 0: nothing. Non-zero → 0: `+0x01 |=
4`, `+0x05` = `0x05` (effect off). 0 → non-zero: `+0x01 |= 4` (fresh) or
`|= 0x0c` (pending) and copy the stored 11-byte effect (mode + 10 params) of
this hand (pad +0x2a19 left, +0x2a24 right) to `+0x05..+0x0f`. Any non-zero
level: `+0x01 |= 0x10` and `+0x23 |= (11 − level, 11→10) << 4 |
(8 − vibration level, 8→7)`.

### Trigger effect (`FUN_1801c44a0`; `psvr2_sense_set_trigger_effect`) [sure]

Input is already wire format: mask (bit 0 L2, bit 1 R2), L2 mode + 10
params, R2 mode + 10 params. On a VR controller with trigger level ≠ 0:
the block of its hand goes to `+0x05..+0x0f` with `+0x01 |= 4` (a pad
matching both hands takes R2). Either way the blocks are stored at pad
+0x2a19 / +0x2a24 (pad +0x18 = 3: left only, 4: right only). For a non-
DualSense, non-DS4 pad Sony then writes an uninitialised byte to pad +0x0e.

### Full state (`FUN_1801c2c80` → `FUN_1801c6250`, VR; `psvr2_sense_fill_state`) [sure]

Sent to resend everything (e.g. after connect): byte 0 `0x02`, `+0x01 |=
0x0c`, `+0x02 |= 4`; trigger: `+0x05` = `0x05` when the trigger level is 0,
else the stored effect of this hand (left by the "VR left" predicate, else
right) at `+0x05..+0x0f`; `+0x24` (status LED enable) = 1; `+0x04 |= 0x80`
when the vibration level is 0; if a library switch is on, `+0x04 |= 2·v`
(v another library value, normally 0 or 1).

### Trigger effect wire format (`FUN_1801c65f0`, `FUN_1801c90e0`) [sure]

Ten trigger positions. Byte 0–1: position mask (bit n). Bytes 2–5: ten
3-bit values packed LSB-first; a stored value v means level v + 1.

| Wire mode | scePad mode | Meaning of the bytes |
|-----------|-------------|----------------------|
| `0x05` | 0 off | — |
| `0x21` | 1 feedback, or 4 multi-position feedback | resistance level per marked position; uniform from the first mark on → (position, strength), else 10 levels |
| `0x22` | 5 slope feedback | first and last marked positions; strengths `(b2 & 7) + 1`, `(b2 >> 3 & 7) + 1` |
| `0x25` | 2 weapon | first two marks within positions 2..8 = start, end (start ≤ 7, end > start, else error `0x80920001`); strength `(b2 & 7) + 1` |
| `0x26` | 3 vibration, or 6 multi-position vibration | amplitude levels as for `0x21`; frequency byte 8 |

Decoded commands are 0x38 bytes each (u32 mode, 4 pad bytes, data) after
a mask byte; decoding sets mask bit `slot + 1`. A `0x21`/`0x26` block with
an empty mask makes Sony read one byte past its level array.

Trigger travel (0..255) at the ten positions: feedback `1 1 32 64 96 128
160 192 224 255`, vibration `8 8 39 71 103 135 167 199 231 255`, weapon
start `– – 54 86 118 150 182 214`, weapon end `– – – 96 128 160 192 224 255`.
`FUN_1801c90e0` estimates the effect state from the current trigger value
(DualSense; VR controllers report it in input `r[0x28]`): feedback 1/2
(outside/inside a marked zone), vibration 6/7, weapon 3 before start, 4
pulling, 5 past the release point (stays 5 until the trigger returns
below start), slope 2 between the marks else 1, invalid 0.

## Motion: gyro still detection, IMU conversion, orientation (`FUN_1801d4a90`) [sure]

Called once per input report from the sample-ring writer with the pad slot,
the new ring entry and the decoded input. State lives in the pad slot at
+0x6c..+0x154 (`psvr2_motion`): flags (+0x6c), gyro bias/scale and accel
offset/scale from the calibration record (+0x74..+0xa0), tick scale (+0xa4),
axis-rotation bits (+0xa8), the still detector (+0xac, 0x94 bytes), the
orientation quaternion x,y,z,w (+0x140) and a sample count (+0x150). The
ring entry gets, from +0x58: timestamp, gyro xyz (rad/s), accel xyz (g),
temperature byte, then the quaternion at +0x78.

Flags: bit 0 automatic gyro-bias tracking, bit 1 gyro dead band, bit 2
gravity tilt correction.

Per sample:

1. If bias tracking is on and at least 10 samples have been seen, the raw
   gyro goes to the still detector; when it reports a new bias, that bias
   replaces the calibration bias.
2. The first 10 samples are warm-up: gyro 0, accel (0, 1, 0), identity
   orientation, count incremented, nothing else.
3. Each axis is (raw − offset) · scale. Axis bit 0 rotates accel, bit 1
   rotates gyro, by the inverse of the fixed mounting matrix (rotation about
   X with cos 0.639166534, sin 0.769068360; the inverse is computed every
   time by the adjugate / determinant). The two rotations sum their products
   in different orders; this matters for bit-exactness.
4. Dead band: gyro components within ±0.0174532924 (1°/s) become 0.
5. With a previous ring entry: dt = ticks · tick_scale · 1e-6 s, where a
   timestamp that went backwards gets +0x10000 except on DualSense (whose
   counter is 32-bit). The quaternion is integrated as q + dt·(q/2 ⊗ ω),
   keeping the multiplications by the zero real part of ω, then normalized
   with one sqrt and one divide.
6. Tilt correction (bit 2) only when | |a| − 1 | < 0.08: rotate the accel
   into the world frame (q a q*), normalize, take the axis v × (0, 1, 0)
   (again with the zero products kept). If its length s exceeds 0.01, the
   error angle is atan2(s, v·ŷ); the step is
   angle · (min(dt / 0.008, 2) · 0.04) · 0.5 as a half-angle, and
   (axis·sin/s, cos) is pre-multiplied onto q. Note the grouping: the gain
   is multiplied by 0.04 before the angle.

Still detector (`FUN_1801d5d80`, init `FUN_1801d5c30` for Sense/DualSense,
`FUN_1801d5ad0` for DS4, reset `FUN_1801d5950`, thresholds `FUN_1801d5a60`):
"range" per axis is |gyro_plus − gyro_bias| / gyro_speed from the
calibration record, i.e. raw counts per deg/s. Init sets flags 3, the bias
to the calibration bias, k_fast 2.0 (DS4 1.0), k_slow 3.0, window length 100
(DS4 250), fast_max = range·6, slow_max = range (DS4 range·0.5), and the
thresholds 2·(range·k_fast)² and 2·(k_slow·range)². A zero speed leaves the
detector disabled.

Each sample adds to per-axis min, max and (wrapping 32-bit) sum. At the end
of a window the spread per axis is (mean − min)² + (max − mean)². If all
three are under the fast threshold and flags bit 0 is set, the new bias is
the window mean, with the step clamped to ±fast_max. Otherwise, if all are
under the slow threshold and bit 1 is set, the window's sums are
accumulated. On the 6th window, if more than 3 windows were accumulated, the
new bias is their mean with the step clamped to ±slow_max. A bias change, or
the 6th window, clears the accumulator. The function returns 1 with the new
bias and the correction (new − old) when it changes the bias.

## Sample ring (`FUN_1801c6ee0` Sense, `FUN_1801c6b20` DualSense/DS4) [sure]

Each pad slot keeps the last 64 decoded samples in a ring at +0x178
(0xa0 bytes each, `psvr2_pad_sample`), with the fill count at +0x160, the
oldest index at +0x164 (only moves once the ring is full), the next index at
+0x168 and a pointer to the newest entry at +0x170. Both writers run under
the library lock, find the slot by the input handle (+0x1c) and do:

1. Clear the entry. Copy receive time, buttons, sticks, L2/R2, the report
   counter and battery.
2. Sense only: copy capacitive touch, trigger and grip proximity and the two
   report flag bits. DualSense only: copy the two status bytes (entry +0x89),
   the touch counter and the fingers that are down (id bit 7 clear) as
   {id & 0x7f, x, y} packed from +0x22 with their count at +0x21.
3. Trigger state. If the pad slot says the controller reports it (+0x0b
   non-zero, and on DualSense neither +0x0d nor +0x0e asks for a host
   estimate), copy the two state bytes and the mode byte from the input.
   Otherwise estimate it: walk back from the new entry while entries are at
   most 20 ms older (receive time, unsigned difference), at most 64 steps,
   summing L2 and R2 including the new entry. With a dense ring the 64th
   step lands on the new entry again and counts it twice. Each mean
   (truncated) goes to the trigger state estimator with the slot's effect
   (+0x2a19 mode and 10 bytes for L2, +0x2a24 for R2) and the previous state
   from the entry the walk stopped on. The mode byte becomes 0xf0.
4. Run the motion update with the previous newest entry.
5. Sense only: report bit 0 of byte 0x34 goes to the clock state's
   latency-mode request (+0x924 of the clock state; see the clock fit's
   BT latency switch). The IMU gap since the last fitted sample is stored as
   an int16 µs value (clock +0x28); gaps of 10 ms or more count in clock
   +0x2b and +0x2a (the latter wraps from 255 to 1). A diagnostic block that
   prints "motionTSDiff" histograms when clock +0x868 is set is not
   reproduced. Then the clock fit runs, and the IMU time converted to host
   time is written to both the input and the entry (+0x08). The DualSense
   writer instead copies an already-converted time from the input.
6. Advance the ring and set the newest pointer.
7. DualSense only, before step 6's pointer update: if pad +0x08 bits 0-1 are
   clear and status bit 1 differs from the previous entry (or there is
   none), call `FUN_1801c3970(pad+0x10 handle, !bit, !bit)`, which sets pad
   +0x08 bits 2-3 and queues an output report. The reimplementation returns
   the value instead of calling. The test runs with the slot outside the
   range that function searches, so this decision is not difftested.

Bit-exactness of the motion step needs the same `atan2f`/`sinf`/`cosf` as
Sony (ucrtbase). With the Zig/mingw libm, 3 of 450 000 steps differed in the
orientation (the difference then persists in the integrated quaternion).

## DualSense input report (`FUN_1801d1f50`) [sure]

Report 0x01 on USB, 0x31 on Bluetooth with every offset one higher (the
caller passes that offset). Offsets below are for USB, from the report id.
The buttons field (input +0x14) is OR-ed into, so the caller clears it.

- 1–4 sticks LX LY RX RY, 5–6 L2 R2, 7 report counter.
- 8 low nibble D-pad: 0 up 0x10, 1 up+right 0x30, 2 right 0x20,
  3 right+down 0x60, 4 down 0x40, 5 down+left 0xc0, 6 left 0x80,
  7 up+left 0x90, 8 none. Values 0–4 are OR-ed in, 5–7 replace the whole
  field (so earlier bits in it are lost).
- 8 high nibble: square 0x8000, cross 0x4000, circle 0x2000,
  triangle 0x1000.
- 9: L1 0x400, R1 0x800, L2 0x100, R2 0x200, create 0x1, options 0x8,
  L3 0x2, R3 0x4. These match the Sense mapping of `FUN_1801d1d10`.
- 10: PS 0x10000, touchpad click 0x100000, mute 0x200000.
- 0x10 gyro xyz, 0x16 accel xyz (int16 LE), 0x1c IMU ticks (u32), 0x20
  temperature.
- 0x21 / 0x25 touch ids (bit 7 set = finger up). The 12-bit x is at bit 0 of
  0x22 / 0x26 and the 12-bit y at bit 4 of 0x23 / 0x27. The touch counter
  (input +0x32) is set to 0.
- 0x2a and 0x2b trigger effect states (to input +0x3f and +0x3e), 0x30
  trigger mode.
- 0x35 battery, 0x36–0x37 status bytes (bit 1 of 0x36 drives the
  `FUN_1801c3970` call in the ring writer).

DualSense Edge only (device slot vid/pid 054c:0df2, and only with a device
slot): byte 0x33 goes through the same D-pad / face-button mapping into a
second field (input +0x44). Byte 0x34 bit 7 sets 0x8, bit 6 sets 0x1 and
bit 5 sets 0x10000 there. Byte 0x31 gives a profile (bits 0–1, +0x54) and
flags (bit 7, bits 4–6, bit 3, bit 2 at +0x55..+0x58). Byte 0x34 bits 1–2 go
to +0x5b. The eight bytes at +0x48 default to 0x80. With a non-zero
profile, report bytes 0x18–0x1b are copied to +0x50. If the profile is
also not 3 and byte 0x32 has its low nibble clear, report bytes 0x10–0x17
are copied to +0x48. These are the gyro bytes; what Sony means by them is
unknown.

The bit-field reader `FUN_1801c83c0(p, bit, n, sign)` reads n bits LSB-first
from a bit offset and optionally sign-extends from the last bit read. The
touch coordinates use it unsigned.

## DS4 / DualSense IMU calibration report (`FUN_1801d2cf0`) [sure]

Feature report 0x02 (USB, 0x25 bytes) or 0x05 (BT, 0x29 bytes, CRC32 with
seed 0xa3 checked first): 18 int16 LE from byte 1. The parse outputs
- the gyro bias xyz,
- the three gyro (plus, minus) pairs,
- gyro speed plus and minus,
- the three accel (plus, minus) pairs.

With the grouped flag (set for BT in `FUN_1801ce8e0`, always in `FUN_1801cce70`)
the report has gyro plus xyz before minus xyz, and the parse interleaves
them. Otherwise it copies them in order.

Calibration to motion conversion (`FUN_1801d5170`), with c = the parsed
array:
- gyro bias = c[0..2];
- per-axis gyro half range h = (|plus − bias| + |minus − bias|) / 2;
- gyro scale = ((speed+ + speed−) / 2 · π/180) / h (rad/s per count), with
  speeds c[9], c[10] (deg/s);
- accel span s = |c[11+2i]| + |c[12+2i]|, offset = c[11+2i] − s/2,
  scale = 2 / s (g per count);
- tick scale (µs per tick): DualSense 1/3; DS4 7.1 for HID version < 0x13,
  else 16/3.

The axis bits are cleared and the calibrated byte (pad +0x70) is set. A zero
gyro range on any axis returns 0x80010002 and writes nothing. A branch that
forces 540 deg/s for a DS4 with HID version 0x11 can never run: its
predicate also requires version > 22.

Pad slot open (`FUN_1801c7770(handle, vid, pid, hid_ver, -, info[0x44], link,
cal, has_cal)`):
- If a slot with this handle is already in use, it updates link (+0x68),
  hid_ver, vid and pid, and bumps the generation byte +1.
- Otherwise it takes the first free slot among the first four. It keeps
  +0x01, +0x06, +0x07, +0x09, +0x10..+0x1b, +0x6c, +0x2a10..+0x2a2f across
  a full clear, and copies the 0x44-byte device info to +0x24.
- With calibration it resets the orientation, converts the calibration,
  computes the still thresholds and initialises the still detector the DS4
  way from c[0..3], c[5], c[7], c[9], c[10].
- It then sets +0x154..+0x15d. For a DualSense these are 0x1186, 0x0780,
  0x0438 and 13, 13. A DS4 gets 0x107c / 0x02f2 below fw 0x2250 at +0x46,
  else 0x1186 / 0x03ae.
- Defaults: vib level 8, trigger level 11, light_dim 0, in_use 1.

## DualSense and DS4 output state (`FUN_1801c6330`, `FUN_1801c6550`) [sure]

These fill the whole controllable state into an output buffer whose byte 0
is the report id (0x02 DualSense USB, 0x05 DS4 USB). Pad slot fields used:
- +0x09 mic LED (0 off, 1 on, 2 pulse)
- +0x0c lightbar dimming (0 full, 1 half, else low)
- +0x14 player number
- +0x2a10 DS4 RGB, +0x2a14 DualSense RGB
- +0x2a18 the trigger-effect block: a mask byte, then two 11-byte slots
  (wire mode + 10 bytes). Slot 0 goes to report byte 0x16, slot 1 to byte
  0x0b.

DualSense:
1. byte 0 = 2, byte 2 |= 0x15 (lightbar, player LEDs, mic LED valid).
2. Player 1..4 sets byte 0x2c to 0x04, 0x0a, 0x15, 0x1b (centre, inner pair,
   outer pair + centre, four outer). Other values leave it.
3. Bytes 0x2d..0x2f: RGB scaled by brightness 0xff / 0x80 / 0x40 with no
   floor (see below).
4. Bytes 9..10 = 00 00, 01 10 or 02 10 for mic LED 0 / 1 / 2. Other values
   leave them.
5. byte 1 |= 0x0c (both trigger effects valid). If the pad has trigger
   level 0, both triggers get wire mode 5 (off) and nothing else.
6. Otherwise both slots are decoded with the effect decoder. On the plain
   DualSense (054c:0ce6, not the Edge) with firmware < 0x297, a slot whose
   decoded mode is multi-position feedback, slope or multi-position
   vibration (4..6) is sent as wire mode 5 with zero data. The pad flag
   +0x0d (slot 0) or +0x0e (slot 1) is then set, which makes the ring
   writer estimate the trigger state on the host. Otherwise the slot is
   copied verbatim and the flag is cleared.

DS4 (only when the player number is non-zero): byte 0 = 5, byte 1 |= 2,
bytes 6..8 = RGB scaled by brightness 0xff / 0x80 / 0x0d with the floor.

Lightbar scaling (`FUN_1801c28a0`, brightness < 0xff only):
- pct = brightness·100/255 (integer).
- Each channel becomes c·pct/100, rounded half up.
- With the floor (13), if all three results are below 13: when they are all
  0 the input is left unchanged. Otherwise every channel is scaled again
  by (13·100/max)% with the same rounding, so the brightest channel lands
  near 13.
- The first pass divides as signed int, so a negative brightness gives
  wrapped values. Callers never pass one.

## Small helpers [sure]

- `FUN_1801c6940`: unpacks ten 3-bit levels. Bytes 0–1 are an enable mask
  (bit n ↔ level n); bytes 2–5 hold the levels packed LSB-first, 3 bits each.
  An enabled level is stored as value + 1, a disabled one as 0. Used by the
  DualSense/VR status parser (`FUN_1801c65f0`).
- `FUN_1801d44d0`: quaternion product (x, y, z, w order), evaluated in the
  order the test requires for bit equality.
- `FUN_1801c8210`: host clock = QPC converted to µs (double), as unsigned
  64-bit.

## Reimplementation status

See `scripts/sony-re/status/libpad.tsv`. Summary of what is bit-exact:
device predicates, CRC, input decode, clock conversions, clamps, clock fit,
LED-sync mode and commands (except the measurement-stop file output),
PRESCAN re-sync, the Sense output-report fields, vibration, trigger
strength/effect, the full-state fill, the trigger wire-format decoder and state estimator, the gyro still
detector, the IMU conversion / orientation integrator, both sample-ring writers, the DualSense input
decode the DS4/DualSense IMU calibration parse, and
the DualSense/DS4 output state with lightbar scaling (libm routed to
ucrtbase in the test, as Sony imports it).
