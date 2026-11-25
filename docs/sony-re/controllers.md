# Sense controllers

Sony's controller library (libpad), the Sense HID behaviour and LED sync.

Parts:

- [Part A: libpad](#part-a-libpad)
- [Part B: Sense over HID](#part-b-sense-over-hid)
- [Part C: LED sync](#part-c-led-sync)

---

## Part A: libpad

Behavioural spec of the libpad part of `driver_playstation_vr2_orig.dll`
(PE timestamp 1741940190, VA range `0x1801b0000–0x1801dffff`), written for a
clean reimplementation in `userspace/lib/sie/libpad/`. Method and
conventions: [overview.md part B](overview.md#part-b-driver-architecture-and-code-map). Equivalence tests:
`scripts/sony-re/difftest/t_libpad_*.c`; per-function status:
`scripts/sony-re/status/libpad.tsv`.

Error codes: `0x80010002` = library not initialised (lock unavailable),
`0x8001002d` = no such pad handle, `0x80920001` = bad argument.

### Global state

- **Pad table**: 8 slots of 0x3410 bytes. A slot is in use when byte 0 is
  non-zero; the u32 at +0x1c is the public handle. Lookup by handle walks
  the slots in order (`FUN_1801c9380`). Other lookups key on +0x10 and +0x14.
- **Device (HID) table**: 12 slots of 0x2200 bytes, one reader thread each.
  Byte +0x1e4 of a device slot is the hand: 1 = left.
- **Lock**: one library lock (`FUN_1801d4280`/`FUN_1801d42b0`) and one for the
  device table (`FUN_1801d42d0`/`FUN_1801d4300`); both are no-ops returning
  `0x80010002` until `FUN_1801d4320` initialises them. The reimplementation
  has no locks in the pure helpers; callers serialise.

### Device families (`FUN_1801c9440`…`FUN_1801c9690`) [sure]

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

### Report CRC (`FUN_1801d4390`, `FUN_1801d43f0`, `FUN_1801d4490`) [sure]

Bluetooth reports end in a CRC-32 (IEEE, reflected, poly `0xedb88320`,
init `~0`, final `~`) computed over one seed byte followed by the report
without its last 4 bytes. The CRC is stored little-endian in those 4 bytes.
Seeds: `0xa1` input, `0xa3` get-feature (DS4/DualSense calibration over
BT). `FUN_1801d43f0` returns non-zero when the stored CRC does not match;
`FUN_1801d4490` is the raw table update with no pre/post inversion.

### Input path (`FUN_1801ce8e0`, one thread per HID device) [sure]

1. Gets the HID caps. Input report length 0x40 means USB, 0x4e (78) means
   Bluetooth (only when BT is enabled). Reads up to 32 reports per call
   (buffer `len·32`).
2. For a VR controller it reads feature `0x05` repeatedly (64 bytes) until bit 7
   of byte 1 is set, merging each part by its index (byte 1 & 0x7f) into
   the calibration (see [overview.md part B](overview.md#part-b-driver-architecture-and-code-map)), then
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

### Sense input report (`FUN_1801d1d10`) [sure]

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

### Controller clock fit, "HID_SYNC" (`FUN_1801c0b90`) [sure]

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

### LED-sync record and commands (`FUN_1801c1820`, `FUN_1801c1880`) [sure]

The 0x30-byte record at pad +0x33e0 (`psvr2_ledsync`) adds to the table in
[overview.md part B](overview.md#part-b-driver-architecture-and-code-map): +0x10 last re-sync host time,
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

### Sense output report (`FUN_1801c3ba0` → `FUN_1801cbeb0`) [sure]

Each pad has two 0x3c-byte output buffers (pad +0x2978, +0x29b4) used in
turn: one is being sent, the other may be filled. A refresh zeroes the
buffer, sets byte 0 = `0x02` (USB output report id) and hands 0x30 bytes to
`FUN_1801cbeb0`, which patches the controller-specific fields and queues a
copy for the writer thread (FIFO, semaphore-signalled). Buffer offset N =
`OutputSettings` offset N − 1 in `sense_protocol.h`.

For a VR controller (`psvr2_sense_output_fill`):

1. `+0x10` u32 = host clock (µs, low 32 bits) at fill time. The controller
   echoes it in input `r[0x2c]`.
2. `+0x01` bit 1 when the vibration mode (pad +6, set by
   `scePadVrControllerSetVibrationMode` `FUN_1801c5730`; Sony's driver always
   sets 2) is 2 or 3, or either library-wide rumble switch is on. Mode 4 or 5: bit 7 when pad firmware
   (+0x50) > 0x220, else bit 1 if pad +0x68 = 1.
3. If `+0x01` bit 4 is still clear: set it and write `+0x23` (Monado's
   `reduction`) = `(11 − trigger_level) << 4 | (8 − haptic_level) & 7`,
   where a result of 11 becomes 10 and 8 becomes 7. The levels are pad bytes
   +0x0b and +0x0a (0 = full strength).
4. Sync mode: if the LED-sync record has no mode yet, use mode 2 when the
   device firmware (feature `0x20` +0x1c, u32, device slot +0x1d0) is below `0x1006d` or 4 probes failed; otherwise probe
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
   The setter `FUN_1801c0200` has no callers, so this build never sends
   feature `0x08`. The firmware gate reads device slot +0x1e0 (feature
   `0x20` +0x2c).
7. `+0x14` phase, written only when phase is 0 or ≥ 5, or the clock fit
   has an offset; `+0x15` seq_id; `+0x16` period_id; `+0x17` u32
   base_time·3; `+0x1b` u32 frame_cycle·3; `+0x1f..+0x22` LED bytes in
   phases 2 and 3, else `ff ff ff ff`.
8. Pad firmware > 0x253: `+0x02` bit 1 and bit 4 from two library-wide
   switches (`FUN_1801d2890`, `FUN_1801d2850`).

The fill time is kept at pad +0x2a58.

#### Output buffer protocol (`FUN_1801c6170`) [sure]

Setters (vibration, trigger, LED…) take a buffer: if none is in flight, the
first buffer is zeroed, filled and sent at once ("fresh"); if one is in
flight and no second is pending, the other buffer becomes pending and is
filled; if one is already pending, it is filled further. A setter that
changes nothing claims no buffer. When the writer finishes a send, the sent
buffer is zeroed (0x3c bytes) and the pending one, if any, is sent (0x30
bytes for VR/DualSense, 0x20 otherwise).

#### Vibration (`FUN_1801c3020`, VR branch; `psvr2_sense_set_vibration`) [sure]

Ignored while pad +0x0a (vibration strength level) is 0. Writes byte 0 =
`0x02`, `+0x01 |= 0x40`, then `|= 0x01` for vibration mode 2 or 4, or
`|= 0x20` for mode 3 or 5 or when a library rumble switch is on. A fresh
buffer always gets `|= 0x01`. `+0x03` = amplitude: **only the second
(small-motor) argument is used** on a VR controller. A non-zero call marks
the pad "vibrating" (+0x2a00) with the host time (+0x2a08).

#### Trigger strength (`FUN_1801c4a30`, VR; `psvr2_sense_set_trigger_level`) [sure]

Stores the new level at pad +0x0b. 0 → 0: nothing. Non-zero → 0: `+0x01 |=
4`, `+0x05` = `0x05` (effect off). 0 → non-zero: `+0x01 |= 4` (fresh) or
`|= 0x0c` (pending) and copy the stored 11-byte effect (mode + 10 params) of
this hand (pad +0x2a19 left, +0x2a24 right) to `+0x05..+0x0f`. Any non-zero
level: `+0x01 |= 0x10` and `+0x23 |= (11 − level, 11→10) << 4 |
(8 − vibration level, 8→7)`.

#### Trigger effect (`FUN_1801c44a0`; `psvr2_sense_set_trigger_effect`) [sure]

Input is already wire format: mask (bit 0 L2, bit 1 R2), L2 mode + 10
params, R2 mode + 10 params. On a VR controller with trigger level ≠ 0:
the block of its hand goes to `+0x05..+0x0f` with `+0x01 |= 4` (a pad
matching both hands takes R2). Either way the blocks are stored at pad
+0x2a19 / +0x2a24 (pad +0x18 = 3: left only, 4: right only). For a non-
DualSense, non-DS4 pad Sony then writes an uninitialised byte to pad +0x0e.

#### Full state (`FUN_1801c2c80` → `FUN_1801c6250`, VR; `psvr2_sense_fill_state`) [sure]

Sent to resend everything (e.g. after connect): byte 0 `0x02`, `+0x01 |=
0x0c`, `+0x02 |= 4`; trigger: `+0x05` = `0x05` when the trigger level is 0,
else the stored effect of this hand (left by the "VR left" predicate, else
right) at `+0x05..+0x0f`; `+0x24` (status LED enable) = 1; `+0x04 |= 0x80`
when the vibration level is 0; if a library switch is on, `+0x04 |= 2·v`
(v another library value, normally 0 or 1).

#### Trigger effect wire format (`FUN_1801c65f0`, `FUN_1801c90e0`) [sure]

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

### Motion: gyro still detection, IMU conversion, orientation (`FUN_1801d4a90`) [sure]

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

### Sample ring (`FUN_1801c6ee0` Sense, `FUN_1801c6b20` DualSense/DS4) [sure]

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

### DualSense input report (`FUN_1801d1f50`) [sure]

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

### DS4 / DualSense IMU calibration report (`FUN_1801d2cf0`) [sure]

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

### DualSense and DS4 output state (`FUN_1801c6330`, `FUN_1801c6550`) [sure]

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

### Small helpers [sure]

- `FUN_1801c6940`: unpacks ten 3-bit levels. Bytes 0–1 are an enable mask
  (bit n ↔ level n); bytes 2–5 hold the levels packed LSB-first, 3 bits each.
  An enabled level is stored as value + 1, a disabled one as 0. Used by the
  DualSense/VR status parser (`FUN_1801c65f0`).
- `FUN_1801d44d0`: quaternion product (x, y, z, w order), evaluated in the
  order the test requires for bit equality.
- `FUN_1801c8210`: host clock = QPC converted to µs (double), as unsigned
  64-bit.

### Reimplementation status

See `scripts/sony-re/status/libpad.tsv`. Summary of what is bit-exact:
device predicates, CRC, input decode, clock conversions, clamps, clock fit,
LED-sync mode and commands (except the measurement-stop file output),
PRESCAN re-sync, the Sense output-report fields, vibration, trigger
strength/effect, the full-state fill, the trigger wire-format decoder and state estimator, the gyro still
detector, the IMU conversion / orientation integrator, both sample-ring writers, the DualSense input
decode the DS4/DualSense IMU calibration parse, and
the DualSense/DS4 output state with lightbar scaling (libm routed to
ucrtbase in the test, as Sony imports it).

---

## Part B: Sense over HID

Scope: libpad (`0x1801b0000–0x1801dffff`) as used by Sony's controller
manager (`FUN_18011d0c0` open/close, `FUN_18011bae0` per-frame poll) in
`driver_playstation_vr2_orig.dll` (PE timestamp 1741940190). This file
builds on [part A](#part-a-libpad) (bit-level decoder, output-fill and clock-fit
notes from the reimplementation pass) and on the "Sense controllers" and
"LED sync" sections of [overview.md part B](overview.md#part-b-driver-architecture-and-code-map). Where
those already state a fact exactly, this file references it and does not
repeat it.

Tags: **[sure]** read in code, **[likely]** strong inference, **[guess]**.

### B.1 Driver summary

On PC the Sense controllers are ordinary HID devices (USB cable or Bluetooth
to the PC). Sony's driver talks to them only through Windows HID
(`HidD_*`, `ReadFile`/`WriteFile`); nothing goes through the headset. A Linux
driver therefore needs **no kernel work** for the controllers: `hid-generic`
plus `hidraw` is enough. Everything below belongs in userspace
(`steamvr/driver_psvr2`, `sense_hid.cpp`).

What the Linux driver must do, in priority order:

1. **Enumerate** VID `054c`, PID `0e45` (left) / `0e46` (right) (`0ce7` also
   counts as a VR controller; hand then comes from feature `0x20`). Tell USB
   from Bluetooth by the input report length: 64 = USB, 78 = BT [sure,
   `FUN_1801ce8e0`].
2. **Read feature `0x20`** (64 bytes; firmware info, hand bit, hardware
   revision) and **feature `0x05`** (IMU calibration, two parts) before the
   first input report is used [sure, `FUN_1801cfe40` → `FUN_1801cfca0`,
   `FUN_1801ce8e0`].
3. **Decode input reports** (table §B.2.1). Use the factory calibration for
   gyro/accel, the battery nibbles as in §B.2.4, and IMU ticks at 3 MHz.
4. **Send output reports** with the right framing. On Bluetooth Sony sends
   `31 a2` followed directly by the body: **no tag byte and no PCM block**.
   The current Linux driver sends `31 <seq<<4> 10 …` (Monado layout). See §B.7.
5. **Clock sync**: stamp output byte `+0x10` with the host µs clock right
   before writing. The controller echoes it in input `r[0x2c]`, which drives
   the `[HID_SYNC]` fit (§B.5.2). This matters only for camera LED tracking
   (LED-sync block), not for IMU-only use.
6. **Haptics**: Sony maps SteamVR haptic events to the classic vibration
   amplitude byte only. Frequency is ignored, the minimum pulse is 80 ms, and
   it resends every 100 ms (§B.5.1). There is no PCM haptics path in this
   build.
7. **Adaptive triggers**: libpad can encode them (format in
   [part A](#part-a-libpad) "Trigger effect wire format"), but **nothing in
   Sony's SteamVR driver calls the trigger setters** (§B.3.4). They are an
   optional extra for Linux, not a parity item.

Things Sony does *not* do that the Linux driver currently does: set the
polling rate (feature `0x08`), PCM haptics, and the `0x10` tag byte on BT.

### B.2 Wire and data formats

Offsets are from the report ID byte. On Bluetooth, input reports have one
extra header byte, so Sony decodes with `r = buf + 1` and `r[0]` is the ID
(`0x01` USB, `0x31` BT) [sure, `FUN_1801d1d10`]. `InputCommon` offset in
`sense_protocol.h` = `r` offset − 1.

#### B.2.1 Input report (`0x01` USB 64 B, `0x31` BT 78 B)

| `r` off | Size | Type | Meaning, units | Tag / VA |
|---|---|---|---|---|
| 0x00 | 1 | u8 | report ID | [sure] `FUN_1801d1d10` |
| 0x01 | 1 | u8 | stick X, 0..255, centre 0x80 | [sure] |
| 0x02 | 1 | u8 | stick Y, 0..255, centre 0x80 (Sony stores raw; up/down sign is the consumer's choice) | [sure] |
| 0x03 | 1 | u8 | trigger travel 0..255 | [sure] |
| 0x04 | 1 | u8 | trigger proximity/touch analogue value | [sure] |
| 0x05 | 1 | u8 | grip proximity analogue value | [sure] |
| 0x06 | 1 | – | not read | [sure] |
| 0x07 | 1 | u8 | report counter (wraps) | [sure] |
| 0x08 | 1 | bits | b0 □, b1 ×, b2 ○, b3 △, b4 L1/R1 (grip), b6 L2/R2 (trigger click); b5 and b7 are the R1/R2 bits. Left reports b0, b3, b4, b6; right reports b1, b2, b5, b7 | [sure] (bit→scePad mapping), [likely] (which bits each hand uses) |
| 0x09 | 1 | bits | b0 create, b1 options, b2 L3, b3 R3, b4 PS, b7 capacitive bit (see 0x0a) | [sure] mapping, [likely] names |
| 0x0a | 1 | bits | low nibble = 4 capacitive bits. Sony packs `touch = r[0x0a]<<4 \| (r[9]&0x80 ? 8 : 0)` and passes it through without naming the bits | [sure] |
| 0x0b..0x0f | 5 | – | not read (Monado's `seq_no` is the u32 at 0x0c) | [sure] |
| 0x10 | 6 | s16×3 | gyro x, y, z, raw LSB (convert with feature `0x05`) | [sure] |
| 0x16 | 6 | s16×3 | accel x, y, z, raw LSB | [sure] |
| 0x1c | 4 | u32 | IMU sample time, 1/3 µs ticks | [sure] |
| 0x20 | 1 | u8 | temperature (raw) | [sure] |
| 0x21..0x27 | 7 | – | not read | [sure] |
| 0x28 | 1 | u8 | trigger-effect state reported by the controller | [sure] |
| 0x29 | 1 | u8 | trigger-effect mode | [sure] |
| 0x2a | 1 | u8 | battery: low nibble level, high nibble charge state (§B.2.4) | [sure] `FUN_1801baff0` |
| 0x2b | 1 | – | not read (Monado's plug state) | [sure] |
| 0x2c | 4 | u32 | **echo** of output `+0x10` (host µs, low 32 bits) from the last output report the controller received | [sure] `FUN_1801c0b90` |
| 0x30 | 4 | u32 | device time, 1/3 µs ticks; 0 = use 0x1c | [sure] |
| 0x34 | 1 | bits | b0 = BT latency-mode request (feeds the clock fit), b3 = a second flag (stored, meaning unknown) | [sure] / [guess] |
| 0x35.. | | – | not read | [sure] |

BT only: the last 4 bytes are a CRC-32 (seed `0xa1`) over bytes 0..73.
Sony drops a BT report with a bad CRC unless its first byte is `0x01`
[sure, see [part A](#part-a-libpad) "Report CRC"].

Gyro/accel conversion: [overview.md part B](overview.md#part-b-driver-architecture-and-code-map)
"IMU calibration". Sony calls `scePadSetDynamicCalibrationState(h, 0)` on
open (`FUN_18011d0c0` → `FUN_1801be980`), which **clears motion flag bit 0
(automatic gyro-bias tracking)** [sure]. So the controller manager uses the
factory bias only. `scePadSetMotionSensorState(h, 1)` on connect sets
motion flag bit 3 (`FUN_1801bec80`) [sure]. Turning it off would clear bit 3
and reset the warm-up count to 9.

#### B.2.2 Output report body (0x30 bytes)

libpad fills a 0x30-byte body whose byte 0 is `0x02`. Body offset N =
`OutputSettings` offset N − 1. Full field rules:
[part A](#part-a-libpad) "Sense output report". Summary with the values Sony
actually sends from the SteamVR driver:

| Body off | Field | What Sony sends | Tag / VA |
|---|---|---|---|
| 0x00 | report ID | `0x02` (replaced on BT, §B.2.3) | [sure] |
| 0x01 | flag1 | `0x40` vibration valid, `0x01` + `0x02` for vibration mode 2 (§B.3.3), `0x04` trigger block valid, `0x08` trigger (pending), `0x10` reduction byte valid. A typical rumble report: `0x53` | [sure] `FUN_1801c3020`, `FUN_1801cbeb0` |
| 0x02 | flag2 | `0x02` and `0x10` from two library switches, only for firmware rev > 0x253; `0x04` in a full-state fill | [sure] |
| 0x03 | vibration amplitude | `ceil(amplitude × 20)`, so 0..20 (§B.5.1) | [sure] `FUN_18011d580` |
| 0x04 | | `0x80` when vibration level is 0 (full-state fill only) | [sure] |
| 0x05 | trigger mode | `0x05` (off) or a stored effect; never set by the SteamVR driver | [sure] |
| 0x06..0x0f | trigger params | 10 bytes | [sure] |
| 0x10 | u32 host time | host µs, low 32 bits. Written at fill time and **written again just before `WriteFile`** | [sure] `FUN_1801cba20` |
| 0x14 | LED phase | §B.2.5 | [sure] |
| 0x15 | LED seq_id | | [sure] |
| 0x16 | LED period_id | clamped to ≤ 60 again in the writer | [sure] `FUN_1801cccf0` |
| 0x17 | u32 base_time × 3 | | [sure] |
| 0x1b | u32 frame_cycle × 3 | clamped again in the writer to 47 050 050 … 53 050 050 (= ns × 3) | [sure] `FUN_1801cccf0` |
| 0x1f..0x22 | LED pattern | phases 2, 3: pattern; else `ff ff ff ff` | [sure] |
| 0x23 | reduction | `(11 − trigger_level) << 4 \| (8 − vib_level) & 7`, where a high nibble of 11 becomes 10 and a low value of 8 becomes 7. Defaults (trigger 11, vib per setting) give `0x0?` | [sure] |
| 0x24 | status LED enable | `1` in a full-state fill only | [sure] `FUN_1801c6250` |
| 0x25..0x2f | | 0 | [sure] |

#### B.2.3 Output framing and write [sure, `FUN_1801cba20`]

The writer thread takes the queued body and builds the HID report:

- **USB** (the device's output report length is ≤ 0x30): send the body as
  is, byte 0 = `0x02`, length = the HID caps output length.
- **Bluetooth** (output length > 0x30): a 78-byte buffer
  `[0] = 0x31, [1] = 0xa2, [2..0x31] = body[1..0x30]`, zero up to byte 0x49,
  CRC-32 with seed `0xa2` over bytes 0..0x49, stored LE at 0x4a..0x4d.
  Byte 1 is the constant `0xa2` (confirmed in the disassembly: a 16-bit store
  of `0xa231` at 0x1801cbc1b). There is no sequence nibble, no `0x10` tag and
  no PCM data. The DualSense branch of the same function does use
  `31 00 10` + body (settings at 3).
- The write is overlapped. It waits 100 ms on USB and 5 000 ms on BT, then
  cancels. A timeout returns `0x80010041`.

Feature reports: Set feature over BT (feature length > 0x40) appends a CRC
with seed `0x53` (`FUN_1801cb7d0`). Get feature over BT checks seed `0xa3`
(`FUN_1801cfca0`) [sure].

#### B.2.4 Battery byte `r[0x2a]` [sure, `FUN_1801baff0`]

| Nibble | Value | Meaning |
|---|---|---|
| low | 0..9 | percent = 10·n + 5 |
| low | 10 | 100 % |
| low | 11..15 | 0 % (invalid) |
| high | 0 | discharging |
| high | 1 | charging |
| high | 2 | full |
| high | other | error (state 4) |

The manager polls this once per frame (`FUN_1801bc850`) and stores
state and percent (manager +0x14c8 / +0x14cc).

#### B.2.5 LED-sync block

Fully in [overview.md part B](overview.md#part-b-driver-architecture-and-code-map) "LED sync" and
[part A](#part-a-libpad) "LED-sync record and commands". Not repeated here.

#### B.2.6 Feature reports used for a Sense controller

| ID | Dir | Len | When | Content | VA |
|---|---|---|---|---|---|
| `0x20` | get | 64 | enumeration | firmware info, §B.2.7. BT: CRC seed `0xa3` | [sure] `FUN_1801cfca0`, `FUN_1801d3490` |
| `0x09` | get | 20 | enumeration (DS4/DualSense path) | MAC address = bytes 1..6 reversed | [sure] `FUN_1801cf980` |
| `0x05` | get | 64 | after open, repeat until byte 1 bit 7 is set | IMU calibration parts | [sure] see overview.md part B |
| `0x80` / `0x81` | set / get | 64 | first output fill, fw ≥ `0x1006d`, at most 4 tries | LED-sync capability probe `80 13 22 00…`; reply byte 0 = `0x81` selects sync mode 2 | [sure] `FUN_1801cbeb0` |
| `0x08` | set | 48 | **never in this build** (the setter `FUN_1801c0200` has no callers) | polling rate `08 0e <u16 rate> <u16 param>`, rate 6..0x540, 1 ≤ param ≤ rate/2; followed by get `0x09` | [sure] `FUN_1801cdff0` |

#### B.2.7 Feature `0x20` for a Sense controller [sure for offsets, `FUN_1801d3490`; names guessed from the DualSense layout]

| Off | Size | Stored as | Meaning |
|---|---|---|---|
| 0x01 | 11 | ASCII | build date [guess] |
| 0x0c | 8 | ASCII | build time [guess] |
| 0x14 | u16 | | firmware type [guess] |
| 0x16 | u16 | | "software series" [guess]. The manager compares this with a value from the PS VR2 app config (§B.3.2) |
| 0x18 | u32 | | **hardware info**; the low 16 bits give the hardware revision (§B.3.2) |
| 0x1c | u32 | | **main firmware version**; ≥ `0x1006d` enables the LED-sync probe |
| 0x20 | 8 + 4 | | device info [guess] |
| 0x2c | u16 | pad +0x50 | **firmware revision** used by libpad's feature checks (thresholds 0x220/0x221, 0x250, 0x253) [likely: pad open copies this block from device +0x1b4 to pad +0x24, so pad +0x50 = report 0x2c] |
| 0x2e | u8 | | update image info [guess] |
| 0x30 | u8 bit 0 | | **hand: 1 = left** (used when the PID is `0ce7`) |
| 0x31, 0x35, 0x39, 0x3b, 0x3c | u32, u32, u16, u8, u32 | | stored, meaning unknown |

### B.3 Sequences

#### B.3.1 Library init and open (`FUN_18011d0c0`) [sure]

Runs when the driver gains "libpad access" (a lock shared with the PS VR2 app,
`FUN_180159940`). It is released again when the app takes the controllers.

1. `scePadInit` (`FUN_1801bdaa0`): set the timer resolution, start two
   threads, the device scanner `FUN_1801cfe40` and the output writer
   `FUN_1801cdd40`.
2. Enable BT (`FUN_1801bef00(1)`).
3. For each of the two controller slots (index 0 = **R**, 1 = **L**; strings
   at `0x180318e08`/`0x180318df4`):
   1. `scePadOpen(user = index + 2, type = index 0 → 4, index 1 → 3)`. Type 3
      reserves the left VR pad slot and type 4 the right one
      (`FUN_1801bdc90` → `FUN_1801c80c0`). The handle is
      `(generation << 8) | port`.
   2. `scePadSetDynamicCalibrationState(h, 0)`: no automatic gyro-bias
      tracking.
   3. `scePadVrControllerSetVibrationMode(h, 2)` (`FUN_1801c05d0` →
      `FUN_1801c5730`): stores mode 2 at pad +6 and sends a fresh output
      report. **Correction to part A:** pad +6 ("connection kind") is this
      vibration mode (1..5).
   4. Clear the per-controller sample history.
4. Release (`FUN_18011d0c0`, else branch): `scePadClose` on each handle
   (`FUN_1801bc4d0`), then `scePadTerm` (`FUN_1801bf650`).

#### B.3.2 Per-frame poll, connect and disconnect (`FUN_18011bae0`) [sure unless tagged]

Each frame, for each controller:

1. `scePadGetBatteryInfo`-like call (`FUN_1801bc850`), §B.2.4.
2. Read up to 64 buffered samples (`FUN_1801bde70`). A negative return resets
   the slot (`FUN_18011a3d0`).
3. For each sample:
   - **Disconnect**: sample's connected byte = 0. If the slot was connected,
     log `VrController(R|L) Disconnected. (busType: …)`, fold the statistics
     (`FUN_18011a5b0`), then reset the slot.
   - **Connect / reconnect**: connected and (not connected before, or the
     sample's connection counter changed). Read connection info
     (`FUN_1801bf670`: connected flag, bus type 1 = USB, 2 = BT, from pad
     +0x68 + 1). Then:
     1. `scePadSetMotionSensorState(h, 1)`.
     2. Hardware revision from feature `0x20` +0x18 (low 16 bits,
        `FUN_1801bc660`): `0x201–0x202` → 1, `0x203–0x204` → 2, `0x205` → 3,
        `0x206–0x208` → 4, `0x209`/`0x210` → 5, `0x211–0x215` → 6,
        `0x216–0x217` → 8, `0x218`, `0x219`, `0x220` → 9,
        `0x221–0x223` → 10, else 11. The tracker uses this as the
        LED-model selector (overview.md part B, "Sense LED models").
     3. Controller information (`FUN_1801bd740`): the manager keeps the
        firmware revision (feature `0x20` +0x2c). If +0x16 equals a value from
        the PS VR2 app config (`DAT_180354be8`, loaded by `FUN_180119400`)
        and the revision is below the configured minimum (`DAT_180354bea`),
        it sets a "firmware update needed" flag [sure for the logic,
        [likely] for the meaning].
     4. Log `Connected.`. Set the IMU offset vector: index 0 (R)
        `(−20.07, −12.25, −6.00)`, index 1 (L) `(9.37, −12.25, −6.00)`
        [sure for the values; units mm and meaning "IMU position in the LED
        model frame" are a guess].
     5. Read the `VibrationStrength` setting and call
        `scePadSetVibrationStrength` (§B.3.3).
   - If the previous bus type was BT and it is now USB, statistics are folded
     the same as on disconnect.
4. Copy the samples to the tracker (pose/IMU/LED-sync consumers belong to the
   `ctrltracker` topic).

#### B.3.3 Vibration strength and mode

`scePadSetVibrationStrength(h, s)` (`FUN_1801bf550`) [sure]: the setting
`s` = 0, 1, 2, 3 maps to vibration level 0, 8, 6, 5 (pad +0x0a). Level 0
makes libpad ignore all vibration calls. Otherwise the level shows up in
output `+0x23` low bits as `8 − level`: 0 (strong), 2 (medium), 3 (weak).
The setting name in the PS VR2 app config is `VibrationStrength`
(`FUN_18015ad30`).

Vibration mode (pad +6) effects on flag1 [sure, `FUN_1801c3020`,
`FUN_1801cbeb0`]:

| Mode | flag1 on vibrate | flag1 on every fill |
|---|---|---|
| 1 | `0x40` | – |
| 2 (Sony's choice) | `0x40 \| 0x01` | `\| 0x02` |
| 3 | `0x40 \| 0x20` | `\| 0x02` |
| 4 | `0x40 \| 0x01` | `\| 0x80` if fw rev > 0x220, else `\| 0x02` when on BT (pad +0x68 = 1) |
| 5 | `0x40 \| 0x20` | as mode 4 |

A fresh buffer always gets `| 0x01`.

#### B.3.4 Trigger effects: unused by the SteamVR driver [sure]

`scePadSetTriggerEffect` (`FUN_1801bf060`, `FUN_1801bf170`) and the trigger
level setter (`FUN_1801bf2d0`) have no callers outside libpad. Trigger level
stays at its default of 11 and the effect store stays zero. The controller
manager never changes trigger resistance. Encoding details for anyone who
wants it anyway: [part A](#part-a-libpad) "Trigger effect wire format".

#### B.3.5 Read and write threads

- Reader (`FUN_1801ce8e0`, one per HID device): calibration first, then
  `ReadFile` up to 32 reports per call with a 2 ms sleep, receive-time
  interpolation, CRC check, decode, and the clock fit. Details:
  [part A](#part-a-libpad) "Input path".
- Output refresh: once per second, and at an 8 ms or 501 ms interval
  (`FUN_1801c1720`) [sure per part A]. Setters add reports between
  refreshes. Two buffers in flight at most, so writes coalesce
  ([part A](#part-a-libpad) "Output buffer protocol").

### B.4 State machines

#### B.4.1 Controller slot (`FUN_18011bae0`)

| State | Event | Action | Next |
|---|---|---|---|
| Closed | libpad access acquired | open, dyn-cal off, vib mode 2 | Open/disconnected |
| Open/disconnected | sample connected | motion on, read hw rev and fw info, IMU offset, vib strength, log | Connected |
| Connected | sample connected = 0 | log, fold stats, reset slot | Open/disconnected |
| Connected | connection counter changed | redo the connect actions | Connected |
| Connected | bus BT → USB | fold stats | Connected |
| any | access released | close all, term | Closed |

#### B.4.2 Haptic pulse (manager +0xcdc4) [sure, `FUN_18011b4d0`, `FUN_18011d6f0`, `FUN_18011d580`]

States: 0 idle, 1 start requested, 2 running.

| State | Event | Action | Next |
|---|---|---|---|
| any | haptic event, amp ≥ 0.049 and freq > 0 | duration = max(dur, 0.08 s), store amp/freq, start = now | 1 if idle, else 2 |
| any | haptic event with amp ≤ 0 or freq ≤ 0, while inside the pulse | set stop flag | same |
| 0 | frame, > 100 ms since last send | send amplitude 0 | 0 |
| 1 | frame | send on | 2 |
| 2 | frame, elapsed ≥ duration | send amplitude 0 | 0 |
| 2 | frame, > 100 ms since last send | send on again | 2 |

"Send on" re-reads `VibrationStrength` (and re-applies it if changed). If
the stop flag is set and more than 80 ms have passed, it sends off instead.
Otherwise it sends amplitude `ceil(amp × 20)`.

### B.5 Algorithms

#### B.5.1 SteamVR haptic → vibration byte

```c
on_haptic_event(dur_s, freq_hz, amp):          // VREvent 0x6a4 (1700)
    if (freq_hz <= 0 || amp <= 0) {
        if (now - start < duration) stop_requested = 1;
        return;
    }
    if (amp < 0.049) return;                   // DAT_18024a468
    duration = max(dur_s, 0.08);               // DAT_180319790
    amplitude = amp; frequency = freq_hz;      // frequency is never used
    state = (state != 0) ? 2 : 1;
    start = now; stop_requested = 0;

per_frame():
    switch (state) {
    case 0: if (now - last_send > 100 ms) send(0); break;
    case 1: send_on(); break;
    case 2: if (now - start >= duration) send(0);
            else if (now - last_send > 100 ms) send_on(); break;
    }

send_on():
    last_send = now;
    if (stop_requested && now - start > 0.08 s) { state = 0; send(0); return; }
    state = 2;
    send(ceil(amplitude * 20));                // DAT_180319978 = 20.0
```

`send(x)` = `scePadVrControllerSetVibration(h, x)` (`FUN_1801c0510` →
`FUN_1801c3020`) → body `+0x03 = x`, flags as in §B.3.3. The PCM block is
never filled. So "haptics sample rate and buffering" for this build is:
**no PCM**, one amplitude byte, refreshed at ≥ 10 Hz while active, and output
reports coalesced through the two-buffer queue.

#### B.5.2 Controller clock fit (`[HID_SYNC]`, `FUN_1801c0b90`, called from `FUN_1801c6ee0`; log helper `FUN_1801c07a0`)

All constants and the exact update order are in [part A](#part-a-libpad)
"Controller clock fit". In brief, as pseudo-C:

```c
// Units: host = µs (QPC), ctrl = µs (= ticks/3, rounded up when ticks%3 == 2).
host_from_ctrl(t) = t + off + ((t - ref) * drift_ppb) / 1e9;
ctrl_from_host(h) = (drift_ppb * ref + (h - off) * 1e9) / (drift_ppb + 1e9);

on_report(H /*host rx µs*/, E /*echo r[0x2c]*/, T /*r[0x30]*/, T2 /*r[0x1c]*/, bt):
    track tick wraps of T and T2;
    if (E == 0 || E == last_E) return;          // no new round trip
    rtt = (u32)H - E;
    if (!bt) {                                  // USB: trust rx time
        skip first 2 samples; classify rtt (>14000 slow, <7700 fast);
        bias = (21 slow in a row) ? +4000 : (21 fast) ? -1000 : 0;
        mid = H - (bias + 4300);
        window = 1;  ref sample = smallest rtt <= 23000;
    } else {                                    // BT: midpoint of the round trip
        mid = (H + E_extended) / 2 - latency;   // latency 500 or -4500 by r[0x34] bit 0
        window = 30; ref sample = median of (mid - ctrl) over the window;
        reject if its rtt >= 13000 (or 20000 in the low-latency mode);
    }
    if (first || jump > 2000 µs (USB) || |drift| > 70000 ppb) {
        off = S.mid - S.ctrl; drift = 0; fits = 0;
    } else {
        P = host_from_ctrl(S.ctrl); e = S.mid - P; fits++;
        (od, dd) = gains(bt, fits >= 100);      // USB 50/10000 → 40/50000,
                                                // BT 5/5000 → 100/30000
        off  = e / od - S.ctrl + P;
        drift += (e * 1e9 / (S.ctrl - ref)) / dd;
    }
    ref = S.ctrl;
```

The host time written into each sample is `host_from_ctrl(T)` (IMU time in
host µs). The LED-sync base time is converted the other way with
`ctrl_from_host`.

### B.6 Constants

| Name | Value | Meaning | VA |
|---|---|---|---|
| Haptic amplitude scale | 20.0 | wire amplitude = ceil(amp·20) | `DAT_180319978`, `FUN_18011d580` |
| Haptic min amplitude | 0.049 | weaker events are ignored | `DAT_18024a468`, `FUN_18011b4d0` |
| Haptic min duration | 0.08 s | also the minimum time before a stop request takes effect | `DAT_180319790` |
| Haptic refresh | 100 000 µs | resend period while on; idle "off" keepalive | `FUN_18011d6f0` |
| Vibration strength map | 0/1/2/3 → 0/8/6/5 | libpad vibration level | `FUN_1801bf550` |
| Vibration mode | 2 | set on open | `FUN_18011d0c0` |
| Default trigger level | 11 | never changed | `FUN_1801c7770` |
| USB write timeout | 100 ms | | `FUN_1801cba20` |
| BT write timeout | 5000 ms | | `FUN_1801cba20` |
| BT output byte 1 | `0xa2` | constant | `FUN_1801cba20` @ 0x1801cbc1b |
| BT output CRC seed | `0xa2` | | `FUN_1801cba20` |
| Set-feature CRC seed (BT) | `0x53` | | `FUN_1801cb7d0` |
| Get-feature CRC seed (BT) | `0xa3` | | `FUN_1801cfca0` |
| LED-sync probe min fw | `0x1006d` | feature `0x20` +0x1c | `FUN_1801cbeb0` |
| Polling-rate min fw rev | > `0x250` | feature `0x20` +0x2c (unused path) | `FUN_1801cdff0` |
| flag2 min fw rev | > `0x253` | | `FUN_1801cbeb0` |
| Mode 4/5 bit 7 min fw rev | > `0x220` | | `FUN_1801cbeb0` |
| Writer clamp, cycle×3 | 47 050 050 … 53 050 050 | | `FUN_1801cccf0` |
| Writer clamp, period_id | ≤ 60 | | `FUN_1801cccf0` |
| IMU offset R | (−20.07, −12.25, −6.00) | [guess] mm | `FUN_18011bae0` |
| IMU offset L | (9.37, −12.25, −6.00) | [guess] mm | `FUN_18011bae0` |
| Gravity | 9.80665 | accel g → m/s² in the manager | `DAT_180319950` |

### B.7 Gaps in the current Linux driver

Compared with `steamvr/driver_psvr2/src/sense_protocol.h` and `sense_hid.cpp`,
`controller_device.cpp`. The kernel module has no controller code, and none
is needed.

**`sense_protocol.h` mismatches:**

1. `BluetoothOutputReport`: Sony sends `[0]=0x31, [1]=0xa2`, then the
   settings start at byte 2. The header has `seq_no_mode` at 1, `tag = 0x10`
   at 2 and settings at 3, plus `counter` and `pcm[32]`. That is the Monado
   layout. Both may be accepted by the firmware (Monado works on BT), but it
   is not what Sony sends. **Hardware test needed** (§B.8).
2. `kOutputReportTag = 0x10` and the comment "seq << 4; low bits 0 = PS5
   layout": Sony uses no tag and a fixed byte 1 of `0xa2` (high nibble 0xa,
   low nibble 2) [sure].
3. `kPcmSampleRate`/`kPcmSamplesPerReport`: no PCM path in Sony's driver
   [sure for this DLL]. Keep them only if the PCM mode is confirmed on
   hardware.
4. `kFlag1VibrationHigh120Hz = 0x00`, `Low60Hz = 0x20`, `Classic = 0x40`:
   Sony always sets `0x40` when vibrating and adds `0x01|0x02` (mode 2) or
   `0x20` (modes 3/5). 0x20 and 0x40 are therefore not alternatives in
   Sony's use: 0x40 is "vibration field valid" and 0x20/0x01 select the mode
   [likely].
5. `vibration_amplitude`: Sony's maximum is 20 (ceil(amp·20)). The Linux
   driver sends up to 255. Either the firmware saturates above 20 or Sony
   uses a deliberately soft scale. **Hardware test needed.**
6. `UsbOutputReport.seq_no_mode = seq<<4 | 2`: Sony sends a plain report ID
   `0x02`, with no sequence nibble. With a non-zero seq the Linux driver sends
   IDs `0x12`, `0x22`, … which are not valid report IDs [likely bug]. Sony
   also sends 0x30 bytes (caps length), not 39.
7. `InputCommon.unknown1[1]` is the report counter. `buttons[2]` low nibble
   and `buttons[1]` bit 7 are capacitive bits, not buttons. `unknown4[0]` bit 0
   is the BT latency-mode request. `host_timestamp` is the echo of output
   `+0x10` (all already noted in part A).
8. `battery_state` comment "low nibble level 0..10": Sony also treats 11..15
   as 0 % and any high nibble ≥ 3 as an error.
9. `CalibrationData`, `kCalibrationPart1/2`: consistent with Sony (part
   index = byte 1 & 0x7f, last part has bit 7).

**Behaviour gaps (`sense_hid.cpp`, `controller_device.cpp`):**

- `SetPollingRate()` sends feature `0x08` (`0e 0c00 0200`). Sony never sends
  it in this build. If it is kept, at least gate it on fw rev (feature
  `0x20` +0x2c) > 0x250 as libpad does, and validate `1 ≤ param ≤ rate/2`.
- Feature `0x20` is not read: no hand bit for PID `0ce7`, no hardware
  revision (needed to pick the right LED model for camera tracking), no
  firmware gates.
- `ReadCalibration` ignores the factory calibration (see overview.md part B). Sony
  uses it and turns automatic bias tracking off.
- Haptics: the Linux driver passes frequency and synthesises PCM
  (`enhanced_haptics`). Sony ignores frequency, enforces 80 ms minimum and
  0.049 minimum amplitude, and refreshes every 100 ms. A Sony-parity mode
  would be: amplitude byte only, the §B.5.1 state machine.
- `VibrationStrength` (off/strong/medium/weak) has no equivalent. Sony puts
  it in the `+0x23` reduction low bits (0/2/3) and drops vibration entirely
  at "off".
- `host_timestamp_us` uses `steady_clock` at fill time. Sony re-stamps right
  before `write()`, which keeps the echo-based RTT honest. The echo
  (`r[0x2c]`) and the clock fit are not used at all. This is required only
  once LED-sync camera tracking is added.
- LED phase is always 5 (all off) with no flag bits, which is fine while
  there is no camera tracking.
- Sony keeps adaptive triggers off. Linux's `EncodeTriggerEffect` is an
  addition. Check it against the wire format in part A if kept.

### B.8 Open questions (need hardware)

1. BT output: does the controller accept both `31 a2 <settings@2>` (Sony) and
   `31 <seq<<4> 10 <settings@3>` (Monado/Linux)? Capture what the PS VR2 app
   sends on Windows (usbmon is not available for BT; use a Windows HID sniffer
   or btmon on a Linux host paired the same way), or try Sony's framing and
   see whether rumble still works.
2. The amplitude range: is 20 the real full scale in vibration mode 2, or does
   255 give a stronger effect?
3. Feature `0x20` field names (build date/time, +0x16 "series", +0x18
   hardware info): dump a real report and compare with the hardware revision
   table in §B.3.2.
4. `r[0x34]` bit 3, and whether `r[0x34]` bit 0 (BT latency mode) ever toggles.
5. The meaning of LED-sync capability probe mode 0 vs 2 on current firmware.
6. Units of the IMU offset vectors (mm assumed), and which frame they are in.
7. The config values `DAT_180354be8` / `DAT_180354bea` (firmware series /
   minimum revision) come from the PS VR2 app's files. Read them there if the
   "update needed" warning is wanted.

### B.9 Function map

| VA | Role | Covered |
|---|---|---|
| `FUN_18011d0c0` | acquire/release libpad; open/close both controllers | yes |
| `FUN_18011bae0` | per-frame poll: battery, samples, connect/disconnect | decisions only |
| `FUN_18011b4d0` | SteamVR haptic event → pulse state | yes |
| `FUN_18011d6f0` | per-frame haptic state machine | yes |
| `FUN_18011d580` | haptic send (on/off), re-applies VibrationStrength | yes |
| `FUN_18011a5b0` | fold per-connection statistics | one line |
| `FUN_1801bdaa0` | scePadInit: timer, scanner and writer threads | yes |
| `FUN_1801bdc90` / `FUN_1801c80c0` | scePadOpen; VR left/right slot reservation | yes |
| `FUN_1801bc4d0`, `FUN_1801bf650` | close, term | named |
| `FUN_1801be980` | dynamic calibration (motion flag bit 0) | yes |
| `FUN_1801bec80` | motion sensor state (motion flag bit 3) | yes |
| `FUN_1801c05d0` / `FUN_1801c5730` | set vibration mode (pad +6), send report | yes |
| `FUN_1801bf550` | vibration strength 0..3 → level 0/8/6/5 | yes |
| `FUN_1801c0510` → `FUN_1801c3020` | set vibration amplitude | yes |
| `FUN_1801bf060`, `FUN_1801bf170`, `FUN_1801bf2d0` | trigger effect / level setters (no external callers) | yes |
| `FUN_1801bc850`, `FUN_1801baff0` | battery info | yes |
| `FUN_1801bc660` | hardware revision from feature `0x20` | yes |
| `FUN_1801bd740`, `FUN_1801cb240` | controller information (feature `0x20` fields) | yes |
| `FUN_1801bf670` | connection info (connected, bus type) | yes |
| `FUN_1801cfe40` | device scanner (SetupDi/HidD enumeration) | partial |
| `FUN_1801cfca0`, `FUN_1801d3490` | read and parse feature `0x20` | yes |
| `FUN_1801cf980` | feature `0x09`/`0x12`/`0xe5` read (MAC) | yes |
| `FUN_1801cc870` | HID value-cap probe (usage page 0xff00, report `0x85`, usage 0x2d) | named; purpose [guess] |
| `FUN_1801cba20` | output writer: USB/BT framing, CRC, timeouts | yes |
| `FUN_1801cccf0` | writer-side LED-sync clamps | yes |
| `FUN_1801cb7d0`, `FUN_1801cb4c0` | set/get feature with BT CRC | yes |
| `FUN_1801cdff0`, `FUN_1801c0200` | polling rate (unused) | yes |
| `FUN_1801cbeb0` | output body fill (see part A) | referenced |
| `FUN_1801c07a0` | `[HID_SYNC]` log formatter | yes |
| `FUN_1801c0b90`, `FUN_1801c1520`, `FUN_1801c09e0` | clock fit and conversions (see part A) | referenced |
| `FUN_1801c6ee0` | Sense sample ring writer (see part A) | referenced |
| `FUN_1801d1d10` | Sense input decode (see part A) | referenced |

---

## Part C: LED sync

Sony driver build PE timestamp 1741940190. Tracker side: `0x180190000–0x18019d000`
(irtracker); controller side: libpad `FUN_1801c0340`, `FUN_1801c1880`,
`FUN_1801c27d0`, `FUN_1801cbeb0`. Clock units are host µs unless stated
(`FUN_1801c8210` = QPC scaled by 1e6, constant at `0x180319960`).

### C.1 Driver summary

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
   Userspace picks the values (§C.2.5, §C.2.6); the kernel only forwards them.
   Camera mode `0x0b` must be `0x10` (already the case for SLAM).
3. **Userspace:** the headset device-µs → host-µs clock map (`0xde`, see
   `headset.md` §A.5.1) and the controller clock fit (part A). Every
   base time in this topic goes headset frame time → host µs → controller
   time.
4. **Userspace:** the LedSyncer state machine (§C.4) and its commands to the
   libpad sync block (§C.2.1). The libpad receiver and output-report fill
   already exist in `userspace/lib/sie/libpad/` (`psvr2_ledsync_command`,
   `psvr2_sense_output_fill`); nothing sends commands to it yet.
5. **Userspace:** the LED choice for BROAD/BG (§C.5.1), the blink-code
   decoder (§C.5.2) and the brightness phase estimator (§C.5.3).
6. **Userspace:** the Many-Blob detection-mode switch (§C.4.3).

Without steps 1–4 the controllers keep their firmware default timing, and
their LEDs are mostly invisible to the headset's short exposures [guess].

### C.2 Wire and data formats

#### C.2.1 Tracker → libpad sync command (`FUN_180194af0` → `FUN_18011b3c0` → `FUN_1801c0340`)

The tracker never writes HID itself. It hands a small byte message plus its
length to libpad (`FUN_18011b3c0` drops lengths outside 1..11). libpad
(`FUN_1801c1880`) validates the length per command, updates the controller's
LED-sync block (pad context +0x33e0), and the next output report carries it.
Every accepted command increments `seq_id` (block +0x01) [sure].

| Cmd | Len | Bytes | Effect in libpad | Source |
|-----|-----|-------|------------------|--------|
| 1 | 7 or 11 | `[1] phase, [2] period_id, [3..6] led[0..3], [7..10] u32 LE value` | Set phase. Phase 1 needs len 7 or 11; phases 2..4 need len 11; phases 5..6 need len 2. period_id clamped to ≤ 60; led[0..3] stored as-is. Phases 2..4: value = base_time offset, clamped to ±half_width. Phase 1: base_time recomputed from the headset frame clock (§C.5.1); with len 11 the value also sets 1frame_cycle (clamped). | [sure] `FUN_1801c1880` |
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

#### C.2.2 LED-sync block in libpad (pad context +0x33e0)

Extends the table in `overview.md` part B.

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
| 1 | never set in this build | 1000 | 105 | 1575 |
| 2 | firmware < 0x1006d (feature `0x20` +0x1c), more than 3 probes run, or probe Set 0x80 `80 13 22 00` / Get 0x81 replied `0x81` | 500 | 50 | 750 |
| 0 | probe ran and reply was not 0x81 | unchanged | unchanged | unchanged |

The steady state is always mode 2 (`controller-tracking.md` §B.3.7). Mode 2 halves the LED pulse and step: newer controller firmware supports a
tighter sync window [likely].

#### C.2.3 Output report placement

Already in `overview.md` part B ("Output-report placement",
`FUN_1801cbeb0`). Additions [sure]:

- base_time (+0x17) and 1frame_cycle (+0x1b) are each written ×3.
- led[0..3] (+0x1f..+0x22) are sent only in phases 2 (BROAD) and 3 (BG);
  in every other phase those bytes are `ff`. So in STABLE the controller
  keeps the LED pattern it last received [likely].
- The phase byte (+0x14) is written when phase is 0, 5 or 6, or when
  libpad's controller clock fit has a reference time (pad +0x2a70, the fit's
  t0, non-zero) [sure]. Before the first clock fit, phases 1..4 leave the
  byte at its default.

#### C.2.4 Phase timing presets (`FUN_18019b9e0`, `FUN_18019c3d0`; tables at `0x18026035c`..`0x18026037c`)

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
in §C.6 (they are timing parameters, not Sony data tables):

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

#### C.2.5 Set `0x14`: LED-detection window (`FUN_180126b80`, values from `FUN_18019c1a0`, `FUN_180125a10`, `FUN_180127f10`)

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

#### C.2.6 Set `0x09`: blob-detection mode (`FUN_180192b90`)

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
overview.md part B) [sure].

#### C.2.7 Camera mode `0x0b` and brightness `0x12`

- `0x0b` payload `{u32 1, u32 mode}`. Mode is **0x10** from startup
  (`FUN_180125670`); only a VrTrace capture session changes it
  (`FUN_180127ec0`, debug) [sure]. LedSync never changes the camera mode.
  `FUN_180125cc0` re-sends it at most once per second while camera or IMU
  calibration is missing, otherwise only on a config change [sure].
- `0x12` is panel brightness and plays no part in LED sync [sure].

#### C.2.8 LD packet fields used by LedSync (`FUN_18018aaf0`, `FUN_180192b90`)

Extends overview.md part B's `"LD"` table.

| Off | Type | Use |
|-----|------|-----|
| 0x08 | u32 | Headset µs of the frame. Mapped to host µs (`FUN_180124a80`); if the map is not ready, Sony uses host now − 6000 µs [sure]. Also the time source for PRESCAN base_time (§C.5.4). |
| 0x0c | u32 | Four 2-bit per-camera flags (bits 0–1, 8–9, 16–17, 24–25). Cameras 0 and 1 drive the Many-Blob switch [sure]; meaning [guess]: saturation/overload. |
| 0x14 | u32 | Frame counter; used to rate-limit `0x09` pushes [sure]. |
| 0x18..0x1f, 0x28..0x2d | u8 | Echo of the active `0x09` block (table above) [sure]. |
| 0x20, 0x24 | f32 | Echo of the active `0x14` A and B [sure]. |
| 0x35 | u8 | Non-zero: no detection this frame. If see-through is on, Sony then switches both controllers to LED ALL OFF [sure]. |
| 0x40 + n·0x2404 | u32 | Blob count of camera n; counts of cameras 0 and 1 drive Many-Blob (§C.4.3) [sure]. |

Sony waits up to 100 ms for a new LD packet each tracker cycle
(`FUN_180128590(dev, last_seq, 100)`) and works only on a packet whose
sequence number changed [sure].

#### C.2.9 LED tables (layout only)

Read them from the user's DLL with the script in §C.6. Do not commit the
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
- **BG check pattern**, `0x180260430` (0x20 bytes): four s32 pairs,
  compared with BG-window frames 0–3 and 17–19 (lit, dark, dark, lit /
  lit, dark, lit) before a brightness estimate is accepted [sure]
  (`controller-tracking.md` §B.3.2).
- **Model selection** (`FUN_180190300`, `FUN_18019c800`): controller info
  `+0x3c` hardware type 1→model 0xb, 2→0xc, 3→0xd, 4→0xe, 5/6/8/9/10/11→0xf;
  type 0 → model 0 and the controller is not tracked [sure].
- **The 0x60-byte block before each LED model** (`0x180257220` + m·0x1400):
  9 ints, 2 floats, 5 ints, 1 float, 2 ints, 2 floats, 3 zero dwords. It is
  identical in all 7 models of this build. It is pose-solver tuning (counts,
  ratios, pixel limits), consumed by the tracking topic, not by LedSync
  [likely].

### C.3 Sequences

#### C.3.1 Controller connect to locked tracking (preset 1, first attempt)

Times are host µs from the moment the controller first appears. "cmd" means
a message to the libpad block (§C.2.1); libpad puts the result into the next
output report.

1. **Connect.** The controller info block reports preset ≠ 0. The first
   connected controller fixes the global preset (§C.2.4); a mismatching second
   controller stays in INIT. The model is chosen (§C.2.9) and the state is set
   to INIT [sure] (`FUN_180190300`, `FUN_18019b9e0`, `FUN_18019c800`).
2. **Detection config.** Once the LD version is > 0x100 and the echo
   differs, push Set `0x14` with the preset values (§C.2.5) [sure].
3. **INIT → PRESCAN**, next tracker frame: cmd 1 `[01 01 28 L0 L1 L2 L3]`
   (len 7; period_id 40). libpad sets base_time from the newest LD frame
   time (§C.5.4) and refreshes it every ≥ 1 s, bumping seq_id each time. The
   report's LED bytes are `ff` in this phase [sure].
4. **PRESCAN → BROAD:** the controller has ≥ 1 associated blob with weight
   > 1e-6 on every frame for ≥ 1 s (3 s after two or more failed BROAD
   rounds). cmd 1 `[01 02 32 L0..L3 00000000]` (len 11; period_id 50,
   offset 0) [sure] (`FUN_1801971f0`, `FUN_180195e20`).
5. **BROAD.** Each frame: choose the sync LED (§C.5.1) and send cmd 2 when the
   set changes. Each camera decodes the 11-frame blink history into a slot
   (§C.5.2). When the same best slot comes out of 3 consecutive frames:
   - slot ≠ 4: cmd 4 with offset `(slot − 4) · step · 5 / 2` µs (integer);
   - once the BROAD drift fit has ≥ 10 samples: cmd 3 (drift) or cmd 5
     (drift + offset) instead (§C.5.5).
   Slot 4 ("centred") held continuously for ≥ 3 s (13 s on retries) → BG.
   No usable code for 10 s → back to PRESCAN, retry counter +1 [sure]
   (`FUN_180198240`).
6. **BG:** cmd 1 `[01 03 1e L0..L3 value]` (period_id 30). value = 630 µs
   when coming from BROAD, −945 µs when coming from STABLE. Each frame the
   brightness estimator gives a sub-slot phase x (§C.5.3), target 10:
   - x outside [9, 11]: cmd 4 with `(10 − x) · step` µs;
   - after 10 samples (40 on later rounds) and a non-zero slope: cmd 3/5.
   Locked when the drift fit has run twice, |slope| < 2e-6, and BG has been
   good continuously for > 15 s (3 s when re-entering BG within 300 s of a
   STABLE run) → STABLE. Bad for ≥ 13 s → PRESCAN [sure] (`FUN_1801975e0`,
   `FUN_180196ff0`).
7. **STABLE:** cmd 1 `[01 04 0c ff… value]`, value = `945 + step·(10 − x)`
   (§C.2.4). LED bytes in the report are `ff` [sure]. While the controller
   stays visible, every 100 s it drops back to BG to re-check the phase.
   Invisible for ≥ 15 s → PRESCAN [sure] (`FUN_1801992b0`).

Best case from connect: about 1 s PRESCAN + a few frames to decode + 3 s
centred + 15 s BG ≈ 20 s to STABLE. Tracking does not wait for STABLE; it
works from BROAD onward with a wider window [likely].

#### C.3.2 Forced transitions (each frame, `FUN_180190300`, `FUN_18018aaf0`) [sure]

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
| Phase hold = 1 (below; Extreme-Env mode 3 sets it, §C.4.3) and phase ≠ PRESCAN | PRESCAN |
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

Only Extreme-Env mode 3 writes the hold from inside the DLL (§C.4.3). Any
other value has to come through the shared tracker-settings block
(`controller-tracking.md` §B.6).

### C.4 State machines

#### C.4.1 Phases (per controller; `0x180362ea0[hand]`; names at `0x180268d20`)

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

#### C.4.2 Bookkeeping on entering a phase (`FUN_1801943d0`) [sure]

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
- A successful drift correction (§C.5.5) resets the retry counter to 0.

#### C.4.3 Blob-detection mode (headset-wide, tracker `+0x4f474`, `FUN_180192b90`) [sure]

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
uses Normal ↔ Many Blob [likely]. Each mode has its own `0x09` block (§C.2.6).

Mode 3 sets the phase hold (§C.3.2) to 1, so every controller sits in
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
(§C.2.6).

### C.5 Algorithms

#### C.5.1 Choosing the sync LED (`FUN_1801999f0`) [sure unless tagged]

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

#### C.5.2 BROAD blink-code decoder (`FUN_1801957d0`, `FUN_18019c500`) [sure]

Per camera, keep a history of the last 11 frames of the sync LED: 2 = seen,
0 = not seen, 1 = uncertain. A code matches when every frame equals the
table bit (2 ↔ 1, 0 ↔ 0); any uncertain frame fails the match. The matching
even column is the slot (0..12); none → −1. Across the 4 cameras pick the
slot closest to the previous accepted slot (within 32). Status codes per
camera feed the failure strings (§C.6).

Accept rule (`FUN_180198240`): same slot in 3 consecutive frames; then
`|slot − 4| > 4` counts as GRID SKIP, and 6 of those reset BROAD.
Otherwise `offset_us = (slot − 4)·step·5/2`, so one Gray step (2 slots) is
5·step = 525 µs (preset 1) / 250 µs (preset 2) [sure].

Meaning [likely]: during BROAD the controller sweeps its LED pulse across
the window in 5-step bins and blinks the bin index; the headset sees the
pulse only in bins that overlap the exposure.

#### C.5.3 BG brightness phase (`FUN_1801954f0`, `FUN_1801975e0`) [likely]

After the BG check pattern matches, the estimator takes the blob weights
of the sync LED over the 20-frame BG window (frames 4..16 are used), sets
the floor to half the window maximum (`(max + 0)/2`), and takes a weighted
centre of the samples **below** the floor, so the weights favour dark
frames (states "NO MAX MIN", "NO DIF MAX MIN" when no peak) [sure]. The
result x is in steps; the exposure centre is x = 10. When the fit object
already has ≥ 2 runs and a fit exists, an x with |x − 10| > 1 is skipped,
up to 3 times in a row (`0x180363110`) [sure] (`controller-tracking.md` §B.3.3–3.4). The phase used for the drift fit is
`x + accumulated corrections` (`0x180362ee0`) [sure].

#### C.5.4 PRESCAN base time (libpad `FUN_1801c1880`, `FUN_1801c27d0`) [sure]

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

#### C.5.5 Drift fit (`FUN_180194180`, `FUN_180193f70`, `FUN_18019caf0`, `FUN_18019bc00`) [sure unless tagged]

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

Units [sure]: the slope is sent unscaled. The BROAD fit holds µs (the
running offset `(int)(sum·step_ns·5/1000)/2`), so its slope is a
dimensionless rate. The BG/STABLE fit holds steps (`x + acc`), so its slope
is steps/µs. The cmd 4 offset is `(10 − x)·step` µs. Because libpad applies
`(1 + slope)·cycle`, only BG/STABLE under-corrects, by a factor of `step`
per round (`controller-tracking.md` §B.3.5). t is the LD frame time in host
µs.

### C.6 Constants

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

### C.7 Gaps in the current Linux driver

| Area | State | Needed |
|------|-------|--------|
| EP `0x89` LD | `kernel/psvr2_aux.c` drains and discards it | Deliver packets + arrival time to userspace (chardev or shared ring) |
| Set `0x14`, `0x09` | not sent anywhere | ioctl/sysfs to send; userspace policy §C.2.5, §C.2.6, §C.4.3 |
| Camera mode | `kernel/psvr2_camera.c` uses mode 1 for V4L2, `0x10` known | Keep `0x10` while tracking; V4L2 mode 1 streaming would break LED detection [likely] |
| Headset clock map | described in `headset.md` part A | Needed for base_time |
| libpad sync block | `userspace/lib/sie/libpad/psvr2_hidsync.c` `psvr2_ledsync_command`, `psvr2_sense_output.c` | Present and matches §C.2.1–2.3. Needs a caller and a `psvr2_sync_headset_time` hook fed from LD +0x08 |
| LedSyncer state machine | absent | §C.3, §C.4 |
| LED choice, BROAD decoder, BG estimator, drift fit | absent | §C.5; needs the blob-to-LED association from the tracking topic |
| LD decode | `userspace/lib/sie/irtracker/psvr2_ld.c` decodes blobs | Also read header fields of §C.2.8 |
| SteamVR driver | `steamvr/driver_psvr2/src/sense_protocol.h` has the LedSettings fields | Fill them from the libpad block |

### C.8 Open questions

1. **Units of base_time vs 1frame_cycle.** Both go into the report ×3.
   16 683 350 only fits a 60 Hz frame in ns, but base_time is computed in
   controller µs. Either the controller clock is not µs or one ×3 means
   something else. Capture: output reports in PRESCAN plus the controller
   input timestamps.
2. **Drift slope units** (§C.5.5). Capture real cmd 3/5 floats from a Sony
   USB/BT trace, or call `FUN_18019caf0` under Wine with test data.
3. **What A in Set `0x14` means** (128 vs 85) and the bytes of `0x09`.
   Capture: vary them and watch the LD blob counts.
4. **LED behaviour in PRESCAN and STABLE** (report LED bytes `ff`). The
   firmware presumably lights its full constellation; confirm with a camera
   capture.
5. **BG check pattern frame mapping** and the exact BG estimator (§C.5.3) —
   medium value; a hardware log of `phase*` lines from Sony's driver (libpad
   logs every command) would pin it down.
6. **Controller info `+0x16`** (forces LED ALL OFF) — which input-report
   bit feeds it.
7. **Sync LED count** setting default is 1 (`0x18035ba1e`); the failure
   string "NOT DETECT 2LED" suggests 2 was used at some point. Check what
   the config manager loads at runtime.

### C.9 Function map

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
