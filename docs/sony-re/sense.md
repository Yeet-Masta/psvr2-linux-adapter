# Sense controllers: what Sony's driver does over HID

Scope: libpad (`0x1801b0000–0x1801dffff`) as used by Sony's controller
manager (`FUN_18011d0c0` open/close, `FUN_18011bae0` per-frame poll) in
`driver_playstation_vr2_orig.dll` (PE timestamp 1741940190). This file
builds on [libpad.md](libpad.md) (bit-level decoder, output-fill and clock-fit
notes from the reimplementation pass) and on the "Sense controllers" and
"LED sync" sections of [../sony-driver-re.md](../sony-driver-re.md). Where
those already state a fact exactly, this file references it and does not
repeat it.

Tags: **[sure]** read in code, **[likely]** strong inference, **[guess]**.

## 1. Driver summary

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
3. **Decode input reports** (table §2.1). Use the factory calibration for
   gyro/accel, the battery nibbles as in §2.4, and IMU ticks at 3 MHz.
4. **Send output reports** with the right framing. On Bluetooth Sony sends
   `31 a2` followed directly by the body: **no tag byte and no PCM block**.
   The current Linux driver sends `31 <seq<<4> 10 …` (Monado layout). See §7.
5. **Clock sync**: stamp output byte `+0x10` with the host µs clock right
   before writing. The controller echoes it in input `r[0x2c]`, which drives
   the `[HID_SYNC]` fit (§5.2). This matters only for camera LED tracking
   (LED-sync block), not for IMU-only use.
6. **Haptics**: Sony maps SteamVR haptic events to the classic vibration
   amplitude byte only. Frequency is ignored, the minimum pulse is 80 ms, and
   it resends every 100 ms (§5.1). There is no PCM haptics path in this
   build.
7. **Adaptive triggers**: libpad can encode them (format in
   [libpad.md](libpad.md) "Trigger effect wire format"), but **nothing in
   Sony's SteamVR driver calls the trigger setters** (§3.4). They are an
   optional extra for Linux, not a parity item.

Things Sony does *not* do that the Linux driver currently does: set the
polling rate (feature `0x08`), PCM haptics, and the `0x10` tag byte on BT.

## 2. Wire and data formats

Offsets are from the report ID byte. On Bluetooth, input reports have one
extra header byte, so Sony decodes with `r = buf + 1` and `r[0]` is the ID
(`0x01` USB, `0x31` BT) [sure, `FUN_1801d1d10`]. `InputCommon` offset in
`sense_protocol.h` = `r` offset − 1.

### 2.1 Input report (`0x01` USB 64 B, `0x31` BT 78 B)

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
| 0x2a | 1 | u8 | battery: low nibble level, high nibble charge state (§2.4) | [sure] `FUN_1801baff0` |
| 0x2b | 1 | – | not read (Monado's plug state) | [sure] |
| 0x2c | 4 | u32 | **echo** of output `+0x10` (host µs, low 32 bits) from the last output report the controller received | [sure] `FUN_1801c0b90` |
| 0x30 | 4 | u32 | device time, 1/3 µs ticks; 0 = use 0x1c | [sure] |
| 0x34 | 1 | bits | b0 = BT latency-mode request (feeds the clock fit), b3 = a second flag (stored, meaning unknown) | [sure] / [guess] |
| 0x35.. | | – | not read | [sure] |

BT only: the last 4 bytes are a CRC-32 (seed `0xa1`) over bytes 0..73.
Sony drops a BT report with a bad CRC unless its first byte is `0x01`
[sure, see [libpad.md](libpad.md) "Report CRC"].

Gyro/accel conversion: [../sony-driver-re.md](../sony-driver-re.md)
"IMU calibration". Sony calls `scePadSetDynamicCalibrationState(h, 0)` on
open (`FUN_18011d0c0` → `FUN_1801be980`), which **clears motion flag bit 0
(automatic gyro-bias tracking)** [sure]. So the controller manager uses the
factory bias only. `scePadSetMotionSensorState(h, 1)` on connect sets
motion flag bit 3 (`FUN_1801bec80`) [sure]. Turning it off would clear bit 3
and reset the warm-up count to 9.

### 2.2 Output report body (0x30 bytes)

libpad fills a 0x30-byte body whose byte 0 is `0x02`. Body offset N =
`OutputSettings` offset N − 1. Full field rules:
[libpad.md](libpad.md) "Sense output report". Summary with the values Sony
actually sends from the SteamVR driver:

| Body off | Field | What Sony sends | Tag / VA |
|---|---|---|---|
| 0x00 | report ID | `0x02` (replaced on BT, §2.3) | [sure] |
| 0x01 | flag1 | `0x40` vibration valid, `0x01` + `0x02` for vibration mode 2 (§3.3), `0x04` trigger block valid, `0x08` trigger (pending), `0x10` reduction byte valid. A typical rumble report: `0x53` | [sure] `FUN_1801c3020`, `FUN_1801cbeb0` |
| 0x02 | flag2 | `0x02` and `0x10` from two library switches, only for firmware rev > 0x253; `0x04` in a full-state fill | [sure] |
| 0x03 | vibration amplitude | `ceil(amplitude × 20)`, so 0..20 (§5.1) | [sure] `FUN_18011d580` |
| 0x04 | | `0x80` when vibration level is 0 (full-state fill only) | [sure] |
| 0x05 | trigger mode | `0x05` (off) or a stored effect; never set by the SteamVR driver | [sure] |
| 0x06..0x0f | trigger params | 10 bytes | [sure] |
| 0x10 | u32 host time | host µs, low 32 bits. Written at fill time and **written again just before `WriteFile`** | [sure] `FUN_1801cba20` |
| 0x14 | LED phase | §2.5 | [sure] |
| 0x15 | LED seq_id | | [sure] |
| 0x16 | LED period_id | clamped to ≤ 60 again in the writer | [sure] `FUN_1801cccf0` |
| 0x17 | u32 base_time × 3 | | [sure] |
| 0x1b | u32 frame_cycle × 3 | clamped again in the writer to 47 050 050 … 53 050 050 (= ns × 3) | [sure] `FUN_1801cccf0` |
| 0x1f..0x22 | LED pattern | phases 2, 3: pattern; else `ff ff ff ff` | [sure] |
| 0x23 | reduction | `(11 − trigger_level) << 4 \| (8 − vib_level) & 7`, where a high nibble of 11 becomes 10 and a low value of 8 becomes 7. Defaults (trigger 11, vib per setting) give `0x0?` | [sure] |
| 0x24 | status LED enable | `1` in a full-state fill only | [sure] `FUN_1801c6250` |
| 0x25..0x2f | | 0 | [sure] |

### 2.3 Output framing and write [sure, `FUN_1801cba20`]

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

### 2.4 Battery byte `r[0x2a]` [sure, `FUN_1801baff0`]

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

### 2.5 LED-sync block

Fully in [../sony-driver-re.md](../sony-driver-re.md) "LED sync" and
[libpad.md](libpad.md) "LED-sync record and commands". Not repeated here.

### 2.6 Feature reports used for a Sense controller

| ID | Dir | Len | When | Content | VA |
|---|---|---|---|---|---|
| `0x20` | get | 64 | enumeration | firmware info, §2.7. BT: CRC seed `0xa3` | [sure] `FUN_1801cfca0`, `FUN_1801d3490` |
| `0x09` | get | 20 | enumeration (DS4/DualSense path) | MAC address = bytes 1..6 reversed | [sure] `FUN_1801cf980` |
| `0x05` | get | 64 | after open, repeat until byte 1 bit 7 is set | IMU calibration parts | [sure] see main doc |
| `0x80` / `0x81` | set / get | 64 | first output fill, fw ≥ `0x1006d`, at most 4 tries | LED-sync capability probe `80 13 22 00…`; reply byte 0 = `0x81` selects sync mode 2 | [sure] `FUN_1801cbeb0` |
| `0x08` | set | 48 | **never in this build** (the setter `FUN_1801c0200` has no callers) | polling rate `08 0e <u16 rate> <u16 param>`, rate 6..0x540, 1 ≤ param ≤ rate/2; followed by get `0x09` | [sure] `FUN_1801cdff0` |

### 2.7 Feature `0x20` for a Sense controller [sure for offsets, `FUN_1801d3490`; names guessed from the DualSense layout]

| Off | Size | Stored as | Meaning |
|---|---|---|---|
| 0x01 | 11 | ASCII | build date [guess] |
| 0x0c | 8 | ASCII | build time [guess] |
| 0x14 | u16 | | firmware type [guess] |
| 0x16 | u16 | | "software series" [guess]. The manager compares this with a value from the PS VR2 app config (§3.2) |
| 0x18 | u32 | | **hardware info**; the low 16 bits give the hardware revision (§3.2) |
| 0x1c | u32 | | **main firmware version**; ≥ `0x1006d` enables the LED-sync probe |
| 0x20 | 8 + 4 | | device info [guess] |
| 0x2c | u16 | pad +0x50 | **firmware revision** used by libpad's feature checks (thresholds 0x220/0x221, 0x250, 0x253) [likely: pad open copies this block from device +0x1b4 to pad +0x24, so pad +0x50 = report 0x2c] |
| 0x2e | u8 | | update image info [guess] |
| 0x30 | u8 bit 0 | | **hand: 1 = left** (used when the PID is `0ce7`) |
| 0x31, 0x35, 0x39, 0x3b, 0x3c | u32, u32, u16, u8, u32 | | stored, meaning unknown |

## 3. Sequences

### 3.1 Library init and open (`FUN_18011d0c0`) [sure]

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
      report. **Correction to libpad.md:** pad +6 ("connection kind") is this
      vibration mode (1..5).
   4. Clear the per-controller sample history.
4. Release (`FUN_18011d0c0`, else branch): `scePadClose` on each handle
   (`FUN_1801bc4d0`), then `scePadTerm` (`FUN_1801bf650`).

### 3.2 Per-frame poll, connect and disconnect (`FUN_18011bae0`) [sure unless tagged]

Each frame, for each controller:

1. `scePadGetBatteryInfo`-like call (`FUN_1801bc850`), §2.4.
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
        LED-model selector (main doc, "Sense LED models").
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
        `scePadSetVibrationStrength` (§3.3).
   - If the previous bus type was BT and it is now USB, statistics are folded
     the same as on disconnect.
4. Copy the samples to the tracker (pose/IMU/LED-sync consumers belong to the
   `ctrltracker` topic).

### 3.3 Vibration strength and mode

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

### 3.4 Trigger effects: unused by the SteamVR driver [sure]

`scePadSetTriggerEffect` (`FUN_1801bf060`, `FUN_1801bf170`) and the trigger
level setter (`FUN_1801bf2d0`) have no callers outside libpad. Trigger level
stays at its default of 11 and the effect store stays zero. The controller
manager never changes trigger resistance. Encoding details for anyone who
wants it anyway: [libpad.md](libpad.md) "Trigger effect wire format".

### 3.5 Read and write threads

- Reader (`FUN_1801ce8e0`, one per HID device): calibration first, then
  `ReadFile` up to 32 reports per call with a 2 ms sleep, receive-time
  interpolation, CRC check, decode, and the clock fit. Details:
  [libpad.md](libpad.md) "Input path".
- Output refresh: once per second, and at an 8 ms or 501 ms interval
  (`FUN_1801c1720`) [sure per libpad.md]. Setters add reports between
  refreshes. Two buffers in flight at most, so writes coalesce
  ([libpad.md](libpad.md) "Output buffer protocol").

## 4. State machines

### 4.1 Controller slot (`FUN_18011bae0`)

| State | Event | Action | Next |
|---|---|---|---|
| Closed | libpad access acquired | open, dyn-cal off, vib mode 2 | Open/disconnected |
| Open/disconnected | sample connected | motion on, read hw rev and fw info, IMU offset, vib strength, log | Connected |
| Connected | sample connected = 0 | log, fold stats, reset slot | Open/disconnected |
| Connected | connection counter changed | redo the connect actions | Connected |
| Connected | bus BT → USB | fold stats | Connected |
| any | access released | close all, term | Closed |

### 4.2 Haptic pulse (manager +0xcdc4) [sure, `FUN_18011b4d0`, `FUN_18011d6f0`, `FUN_18011d580`]

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

## 5. Algorithms

### 5.1 SteamVR haptic → vibration byte

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
`FUN_1801c3020`) → body `+0x03 = x`, flags as in §3.3. The PCM block is
never filled. So "haptics sample rate and buffering" for this build is:
**no PCM**, one amplitude byte, refreshed at ≥ 10 Hz while active, and output
reports coalesced through the two-buffer queue.

### 5.2 Controller clock fit (`[HID_SYNC]`, `FUN_1801c0b90`, called from `FUN_1801c6ee0`; log helper `FUN_1801c07a0`)

All constants and the exact update order are in [libpad.md](libpad.md)
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

## 6. Constants

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

## 7. Gaps in the current Linux driver

Compared with `steamvr/driver_psvr2/src/sense_protocol.h` and `sense_hid.cpp`,
`controller_device.cpp`. The kernel module has no controller code, and none
is needed.

**`sense_protocol.h` mismatches:**

1. `BluetoothOutputReport`: Sony sends `[0]=0x31, [1]=0xa2`, then the
   settings start at byte 2. The header has `seq_no_mode` at 1, `tag = 0x10`
   at 2 and settings at 3, plus `counter` and `pcm[32]`. That is the Monado
   layout. Both may be accepted by the firmware (Monado works on BT), but it
   is not what Sony sends. **Hardware test needed** (§8).
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
   `+0x10` (all already noted in libpad.md).
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
- `ReadCalibration` ignores the factory calibration (see the main doc). Sony
  uses it and turns automatic bias tracking off.
- Haptics: the Linux driver passes frequency and synthesises PCM
  (`enhanced_haptics`). Sony ignores frequency, enforces 80 ms minimum and
  0.049 minimum amplitude, and refreshes every 100 ms. A Sony-parity mode
  would be: amplitude byte only, the §5.1 state machine.
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
  addition. Check it against the wire format in libpad.md if kept.

## 8. Open questions (need hardware)

1. BT output: does the controller accept both `31 a2 <settings@2>` (Sony) and
   `31 <seq<<4> 10 <settings@3>` (Monado/Linux)? Capture what the PS VR2 app
   sends on Windows (usbmon is not available for BT; use a Windows HID sniffer
   or btmon on a Linux host paired the same way), or try Sony's framing and
   see whether rumble still works.
2. The amplitude range: is 20 the real full scale in vibration mode 2, or does
   255 give a stronger effect?
3. Feature `0x20` field names (build date/time, +0x16 "series", +0x18
   hardware info): dump a real report and compare with the hardware revision
   table in §3.2.
4. `r[0x34]` bit 3, and whether `r[0x34]` bit 0 (BT latency mode) ever toggles.
5. The meaning of LED-sync capability probe mode 0 vs 2 on current firmware.
6. Units of the IMU offset vectors (mm assumed), and which frame they are in.
7. The config values `DAT_180354be8` / `DAT_180354bea` (firmware series /
   minimum revision) come from the PS VR2 app's files. Read them there if the
   "update needed" warning is wanted.

## 9. Function map

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
| `FUN_1801cbeb0` | output body fill (see libpad.md) | referenced |
| `FUN_1801c07a0` | `[HID_SYNC]` log formatter | yes |
| `FUN_1801c0b90`, `FUN_1801c1520`, `FUN_1801c09e0` | clock fit and conversions (see libpad.md) | referenced |
| `FUN_1801c6ee0` | Sense sample ring writer (see libpad.md) | referenced |
| `FUN_1801d1d10` | Sense input decode (see libpad.md) | referenced |

## Corrections to the main doc and libpad.md

- libpad.md "Sense output report" step 2 and "Vibration": pad +6 is not a
  connection kind. It is the **vibration mode** set by
  `scePadVrControllerSetVibrationMode` (`FUN_1801c5730`). Sony's driver
  always sets 2.
- libpad.md "Sense output report", step 6 (polling rate): the setter
  `FUN_1801c0200` has no callers, so this build never sends feature `0x08`.
  The firmware gate reads device slot +0x1e0 (feature `0x20` +0x2c).
- libpad.md "Sense output report", step 4: "device firmware below
  `0x1006d`" is feature `0x20` +0x1c (u32, device slot +0x1d0).
- Main doc, "Sense controllers": add that output over BT is
  `31 a2 <body[1..]>` + CRC (seed `0xa2`), with no tag byte (§2.3).
- Main doc, "Other controller feature reports": feature `0x20` is also read
  for VR controllers, and it carries the hand bit (+0x30 bit 0) and the
  hardware revision (+0x18) (§2.7).
