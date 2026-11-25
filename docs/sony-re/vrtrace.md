# VrTrace capture and shared math (Sony driver, build 1741940190)

Behavioural spec, written from the decompile in our own words. VAs are in
the documented build (image base `0x180000000`). Status per function is in
[`scripts/sony-re/status/trace.tsv`](../../scripts/sony-re/status/trace.tsv).
Confidence tags as in [overview.md part B](overview.md#part-b-driver-architecture-and-code-map).

## 1. What VrTrace does on PC (summary) [sure]

VrTrace is the PS5 tracker's capture tool, linked into the PC driver. It has
three parts:

- **VrTraceCaptureAgent** (`0x180115ce0..0x1801167b0`): receives a text
  command (`start <dir> <args>` / `stop`) from the config manager (channel
  id `0x200`, `FUN_18015e0b0` registration, string read by `FUN_18015dff0`),
  parses capture options and starts/stops the core.
- **VrTraceCore** (`0x18010e2d0..0x18010f460`): state machine, dump
  directory, a dump thread that drains rings into files.
- **VrTraceManagerImpl** (`0x180111400..0x180115190`): owns one arena
  (≥ 0x19a00000 bytes = 410 MiB), carves it into per-stream rings, and owns
  the 13 `VrTrace*Dumper` objects (one file each).

**Important for replay:** in this PC build, nothing feeds the streaming
rings. The ring producers (`FUN_1801137a0`, `FUN_1801138d0`, `FUN_180113a10`,
`FUN_180113be0`, `FUN_180113c50`, `FUN_180113da0`, `FUN_1801141d0..
FUN_1801145a0`, `FUN_180114810`) have no call, jump or pointer reference
anywhere in the image (checked by scanning every `E8`/`E9` rel32, every
RIP-relative `LEA` and every 8-byte data word). `receiveImage`
(`FUN_1801149e0`) and `receiveSlamTraceData` (`FUN_180114b50`) are reachable
only through `VrTraceManagerImpl`'s vtable, and the only holder of the
manager instance (`DAT_180353fe8`, getter `FUN_180113b60`) is the capture
agent itself; the HMD side's `IVrTraceCallback` vtable is two `_purecall`s.
So a PC capture writes only the **static files** (§4) plus empty stream files.
Sony's DLL also contains a replay-argument parser (`FUN_180116480`, options
`--repeat`, `--timeout N`) with no callers: **there is no replay path in
Sony's DLL.** End-to-end replay therefore has to drive Sony's parse/track
functions directly (§7).

## 2. Capture command and options (`FUN_180116230`, `FUN_180115e60`) [sure]

Command string: leading spaces skipped; first token (quotes allowed) is the
verb. `stop` (4 chars) stops capture. `start` (5 chars) is followed by
arguments: one non-option token = dump directory (< 255 chars), and options
starting with `-`, which are also re-joined with single spaces into an
"args" string (< 256 bytes total) that is kept for logging.

Capture-option block (0x220 bytes): dir[0x100], args[0x100], `+0x200` dir
length, `+0x208` args length, `+0x210` u32 capture type, `+0x214` u8 GPU markers, `+0x215` u8 CPU
markers, `+0x216` u8 gaze target (all three default 1), `+0x217` u8 FSR
LUT, `+0x21c` u32 stereo target.

| Option | Effect |
|--------|--------|
| `--ct-live` | type 0 |
| `--ct-pose-w` | type 1 |
| `--ct-pose-ir` | type 2 |
| `--ct-pose-gaze` | type 3 |
| `--ct-live-gaze` | type 4 |
| `--no-cpu-markers` | `+0x215 = 0` |
| `--no-gpu-markers` | `+0x214 = 0` |
| `--no-gaze-target` | `+0x216 = 0` |
| `--fsr-lut` | `+0x217 = 1` |
| `--stereo-target-live-1hz` | `+0x21c = 1` |
| `--stereo-target-live-30hz` | `+0x21c = 2` |

(The strings are compared as 8-byte little-endian words; exact text
decoded above.) Any other option aborts with "Unknown Capture args". Start
is retried up to 3 times, 100 ms apart, while the core answers
`0x8a740008` (busy).

## 3. Core state machine (`FUN_18010e4b0`, `FUN_18010f300`, `FUN_18010f460`,
`FUN_18010e860`, `FUN_18010ec10`) [sure]

Capture type → headset camera mode sent with `FUN_180127ec0` (control
report `0x0b`, see overview.md part B), then a 100 ms sleep:

| type | 0 | 1 | 2 | 3 | 4 | 1000 | 1001 | 1002 | 1003 | 1004 |
|------|---|---|---|---|---|------|------|------|------|------|
| mode | 0x0c | 2 | 4 | 5 | 0x0f | 1 | 3 | 9 | 0x0b | 3 |

Other types fail with `0x8a740006`. Type 1004 sets a "special" flag; types 0
and 4 are **live** (records are processed but not written, except the image
dumper which writes in live mode too).

Dump directory: when none is given, `sie_vr_capture/YYYYMMDD_HHMMSS`
(local time; both levels created with `_mkdir`, failure `0x8a74001f`).
Files are `<dir>/<name>` (`"%s/%s"`), opened `"wb"`.

States at core `+0x10`: 0 idle, 1 idle(alt), 2/3 capturing, 4 stopped OK,
6 stopped with error. `startDump` refuses (`0x8a740008`) in states 2..6.

Dump thread: loops every 10 ms while enabled. Each pass drains every dumper
(§5), then if nothing was pending, sleeps 100 ms; when capture is being
disabled it does a final 50 ms drain. Any dumper's write error stops the
capture ("Failed to dump"); a full arena stops it ("Force capture stop
because of buffer full").

## 4. Files [sure unless tagged]

All multi-byte values little-endian (x86 host). "Record" = bytes copied
verbatim from the producer; the producers' layouts belong to the other
subsystems (INS, controller tracker, libpad, relocaliser).

### 4.1 Static files (written once at capture start, `FUN_180114030`)

| File | Content | Writer |
|------|---------|--------|
| `plugin_version.bin` | NUL-free ASCII plugin version string (manager `+0x918`), written once per capture session | `FUN_180114030` |
| `computer_spec.bin` | Text: `Windows version: %lu.%lu\n` (from `RtlGetVersion`) or `Windows version: Unknown\n`, then `Graphic Board: %s\n` / `Graphic Board: Unknown\n` | `FUN_180112dd0` |
| `hmd_hw_version.bin`, `hmd_sw_version.bin` | Headset HW / FW version blocks | `FUN_180113040` |
| `imu_calib.bin` (0x78), `imu_set_calib.bin` (0x34), `vrt_camera_calib_vrtrace0.bin`, `vrt_camera_calib_vrtrace1.bin` | The raw `VD` entries types 12, 13, 10, 11 (overview.md part B, calibration stream) | `FUN_1801132a0` |

### 4.2 Stream files

| File | Dumper (vtable) | Ring | Record on disk |
|------|-----------------|------|----------------|
| `db_000.dat` | SlamData (`0x180248598`) | SLAM ring | ST frame from `+0x20`: 0x796b0 bytes if frame-info version (u64 at ST+0x20) is 4, else `min(u32 at ST+0x28, 0x7ffe0)` bytes |
| `imu.bin` | Imu (`0x1802485f0`) | SLAM ring (same frames) | 0x108-byte header, then 0x2848-byte records from each ST frame (§4.6) |
| `basic_Info.bin` | ApiData (`0x180248648`) | API ring | File header (§4.3), then records of `min(u32 at rec+4, 0x240)` bytes |
| `packed_image.bin` | HmdGeneralImage (`0x1802486a0`) | image S-ring | Per image: u32 `0xeadca262`, then the 0x40-byte image header (§4.4), then `w·h` pixel bytes (`w` = u16 +0x16, `h` = u16 +0x14) |
| `reloc_context.bin` | RelocContext (`0x1802486f8`) | reloc-context S-ring | 0xfce90 bytes per record |
| `selfcalibration.bin` | SelfCalib (`0x180248750`) | 3 rings | per step, whichever are present: result 0xa0, internal points 0x2820, result points 0x3820 bytes |
| `vrt_camera_selfcalib_vrtrace.bin` | CameraSelfCalib (`0x1802487a8`) | none (polls HMD) | 0x800-byte camera-calib block, written each time the HMD's self-calibration generation counter changes (`FUN_1801246b0`) |
| `ins.bin` | Ins (`0x180248800`) | INS ring | 0x3dd0 bytes |
| `controller_tracking.bin` | ControllerTracking (`0x180248858`) | slot ring | `u32 at rec+4` bytes, valid 1..0x8440 |
| `controller_pad0.bin`, `controller_pad1.bin` | ControllerPad (`0x1802488b0`) | slot ring ×2 | `u32 at rec+4` bytes, valid 1..0x2130 |
| `reloc_snapshot.bin` | RelocSnapshot (`0x180248908`) | 1-slot ring | 0x2000030 bytes |
| `playarea_data.bin` | Playarea (`0x180248960`) | S-ring | `i32 at rec+4` bytes |

`dumpVrImage` (`FUN_18010c390`) writes the magic, then only if the file
position is > 0 (always true after the magic) the header and pixels as one
fwrite of `0x40 + (int)(pitch·height)` bytes; the product is sign-extended,
so a product ≥ 2³¹ would make the write fail (cannot happen for images that
pass §4.4). After a good write it emits per-camera telemetry events (only if
the event channel is enabled), which do not touch the file. On any stream
error it sets the dumper's failed flag (+0x10) and logs "dumpVrImage failed".

Records whose size field is out of range are skipped (not written) but
still consumed. Self-calibration result points are copied from a
0x3818-byte producer struct into 0x3820-byte slots, so the **last 8 bytes
of each such record are uninitialised** (stack contents); a reader must
ignore them.

Records that carry their own size at `+4` (controller tracking, pads, API,
playarea) are self-delimiting: read u32 at +4 of the next record, then the
whole record. Fixed-size files are read in fixed chunks. `db_000.dat`
records are self-delimiting through the frame-info size at payload `+0x08`
(except version 4, fixed 0x796b0). `packed_image.bin` is self-delimiting via
the magic and header.

### 4.3 `basic_Info.bin` header (`FUN_18010d440`) [sure]

32 bytes at file start: u64 `2` (format version), u32 capture type, then
20 zero bytes.

### 4.4 Image header V0200 (`checkImageHeader`, `FUN_18010a4f0`; `receiveImage`, `FUN_1801149e0`) [sure]

Input image buffer (from the headset) checks, in order, returning:
`-1` null/short (< 0x40 bytes); `-2` magic ≠ `"VI"`; `-3` version 0x0100
(logs once "Image Header V0100 is no longer supported"); `-1000` version
≠ 0x0200; `-5` pitch (u16 +0x16) < width (u16 +0x12); `-6` buffer size <
`u16[+0x1a]·u16[+0x18]·u16[+0x14]·u16[+0x16] + u32[+0x1c]`; `-7` buffer
size < `u32[+0x20] + u32[+0x1c]`; `-8` buffer size > 0x100000; else 0.

| Offset | Type | Meaning |
|--------|------|---------|
| 0x00 | char[2] | `"VI"` |
| 0x02 | u16 | version 0x0200 |
| 0x08 | u32 | device timestamp (µs) |
| 0x0c | u32 | copied into trace events |
| 0x10 | u16 | format (image kind; see table below) |
| 0x12 | u16 | width |
| 0x14 | u16 | height |
| 0x16 | u16 | pitch |
| 0x18 | u16 | vertical tile count |
| 0x1a | u16 | horizontal tile count |
| 0x1c | u32 | pixel data offset from buffer start |
| 0x20 | u32 | pixel data size |
| 0x24..0x3f | | copied verbatim |

The stored (trace) header rewrites the geometry so the tiles are one image:
`+0x12 = width·tiles_h(+0x18)`, `+0x14 = height·tiles_w(+0x1a)`,
`+0x16 = pitch·tiles_h(+0x18)`; record payload = `u32[+0x20]` bytes from
`+u32[+0x1c]`, slot size = new `+0x16 · +0x14` + 0x40. Formats 0x0b, 4 and
5 are dropped while a "position" capture (core flags) is active.

Per-image camera mapping used for trace events (`FUN_18010a770`: which of 22
camera/view ids 0..0x15 a format carries; `FUN_18010a6a0`: id → (layer,
plane)) only feeds ETW-style telemetry events (`FUN_180116a60`), not the
file, so it is not reimplemented.

### 4.5 `ST` SLAM-trace frame (`receiveSlamTraceData`, `FUN_180114b50`; parsers `FUN_18010c610`) [sure]

Up to 0x80000 bytes; copied whole into a 512 KiB ring slot.

| Offset | Type | Meaning |
|--------|------|---------|
| 0x00 | char[2] | `"ST"` |
| 0x02 | u16 | version, must be 0x0010 |
| 0x20 | u64 | frame-info version; < 4 rejected; 4 = fixed layout; > 4 = TOC layout |
| 0x28 | u32 | payload size from +0x20 (TOC layout) |
| 0x30 | u32 | TOC entry count (≤ 128 used) |
| 0x44 + 12·i | u16 id, u16 pad, u32 size, u32 offset | TOC entries; offset is from ST+0x20 |

TOC ids used (meanings [likely], from which fields the telemetry reads): 0 = frame header (≤ 0x20 bytes: device timestamp at +8,
camera/frame index at +0xc …), 1 = per-camera exposure block (≤ 0x50
bytes), 5 = tracking-quality block (≤ 0x30), 6 = feature block. A TOC frame
is accepted for capture only if some entry has id 0. Fixed layout (v4):
frame header at +0x30, exposure block at +0x50, quality at +0x4100, feature
block (0xbc10 bytes) at +0x4130.

### 4.6 `imu.bin` (`FUN_18010b8f0`) [sure, difftested]

For each ST frame the dumper first copies three sections into fixed-size
buffers. A section is taken from the TOC entry with that id (first match,
≤ 128 entries, data at ST+0x20+offset, size clamped to the buffer), or for
a v4 frame from a fixed offset:

| Section | TOC id | v4 offset | Buffer |
|---------|--------|-----------|--------|
| frame header | 0 | +0x30 | 0x20 |
| IMU calibration | 2 | +0x80 | 0x160 |
| feature block | 6 | +0x4130 | 0xbc10 |
| IMU block | 0xd | +0x5ab70 | 0x990 |

If the frame header, feature block or IMU block is missing, nothing is
written for the frame. Before the first record only (and only if the
calibration section is also present), a 0x108-byte file header is written:

| Off | Size | From |
|-----|------|------|
| 0x00 | 0x0c | calibration +0x04 |
| 0x0c | 0x6c | calibration +0x10 |
| 0x78 | 3 × 0x0c | IMU block +0x1f8, +0x480, +0x708 |
| 0x9c | 3 × 0x24 | IMU block +0x204, +0x48c, +0x714 |

Then each frame gives one 0x2848-byte record:

| Off | Size | From |
|-----|------|------|
| 0x00 | 4 | frame header +0x00 |
| 0x04 | 4 | frame header +0x08 |
| 0x08 | 0x0c | IMU block +0x294 |
| 0x14 | 0x24 | IMU block +0x2a0 |
| 0x38 | 0x2810 | feature block +0x93e8 |

Each fwrite stops the sequence on a stream error, so a short final record
is possible in a truncated file. Implemented by `psvr2_trace_imu_header` /
`psvr2_trace_imu_record`; `t_trace_dump` compares them with Sony's dumper
on 3000 random v4 and TOC frames (0 mismatches).

Firmware gate: when the HMD firmware version (`FUN_180124a00`) changes and is
below `0x00810003`, SLAM-trace capture is disabled ("SDK version 3.xx or
later requires FW version 00810003").

## 5. Rings [sure]

Two ring kinds. Each has a mutex and two `Framework::Event` objects
(reader-hold `+0x20`, writer-hold `+0x30`). The events are created
**manual-reset** and set in the constructor, and nothing in the trace code
ever resets them, so every wait on them returns at once: in practice the
mutex is the only synchronisation and the "reader busy" branches below
never trigger.

Common header (0x50 bytes): `+0` u32 write index, `+4` u32 read index,
`+8` u32 dropped count, `+0xc` u8 empty flag (initially 1), `+0x10` mutex
(name `SceVrtMtxTraceRing` / `SceVrtMtxTraceSRing`), `+0x20`, `+0x30`
events, `+0x40` buffer, `+0x48` u32 capacity (slots, or dwords for
S-rings).

**Slot ring** (fixed-size slots). Push: wait writer event. If full
(`write == read` and not empty): test the reader event without waiting; if
not signalled, count a drop and discard the new record; otherwise discard
the oldest (advance read; set empty if it reaches write), count a drop,
set the reader event, then write. Copy into slot `write`, clear empty,
`write = (write+1) % capacity`, set writer event. Pop: wait reader event;
if empty, set it and return none; else return slot `read`. Release:
`read = (read+1) % capacity`, set empty when it reaches write, set the
reader event.

**S-ring** (variable-size, dword units). A record of `n` bytes uses
`k = ceil(n/4) + 1` dwords: a u32 length word `k` followed by the data.
Reserve: if `read < write` or empty and the space to the end of the buffer
is too small, write `-1` (wrap marker) at `write` and restart at 0. If the
space up to `read` is too small, drop the oldest record (same rule as
above) and retry. Commit: store `k` at `write`, clear empty, advance
`write = (write+k) % capacity`. Pop skips a `-1` wrap marker to index 0.
The record pointer returned is length word + 4.

Arena carving (`createBuffers`, `FUN_1801129c0`): requires ≥ 0x19a00000
bytes; `numFrames = arena / 0x3f4980`. Fixed rings: reloc snapshot 1 ×
0x2000030; selfcalib 0x80 × 0xa0, 0x400 × 0x2820, 0x80 × 0x3820; API
0xe38e × 0x240. Per-frame rings: playarea S-ring `numFrames·0x3c000` bytes;
reloc-context S-ring `numFrames·0x2a270`; INS `numFrames·0x3dd0` (slot
0x3dd0); SLAM `numFrames` × 0x80000; controller tracking `numFrames` ×
0x8440; two pad rings `numFrames` × 0x2130; image S-ring = rest. Errors
`0x8a740006` (bad args), `0x8a740009` (allocation).

## 6. Dumper object (`VrTraceDumper`, base vtable `0x180249110`) [sure]

Fields: `+8` FILE*, `+0x10` u8 write-error, `+0x11` u8 opened, `+0x18`
progress object (always null on PC: `FUN_180115c90` returns 0), `+0x20`
ring fill stats, `+0x2c` burst count (records per drain pass, starts 1),
`+0x30`/`+0x38` last-drain / last-flush time (µs), `+0x40` ring, `+0x48`
current record.

Vtable slots: 0 open(file?, type, dir) → closes any old file, resets,
opens `dir/name` if asked, then calls slot 9; 1 "open file even in live
mode"; 2 process record (validation / telemetry; image dumper writes here);
3 write record; 4 acquire next record; 5 release record; 6 file name; 7
reset progress; 8 update fill stats; 9 write file header.

Drain pass (`FUN_180110250`): per dumper, up to `burst` times: acquire; if
the record is to be handled (no file needed, or file open) call process, and
in non-live mode also write; release. Afterwards `fflush` any file not
flushed for 2 s. Burst adaptation (`FUN_180110860`): dumpers are ranked by
ring fill; the three fullest get burst+1 (max 30), the rest burst−1 (min 0;
a 0 becomes 1). The fill key is `(float)u32[+0x20] / (float)u32[+0x24]`
(0 when `+0x24` is 0). The dumper pointer array is sorted by that key in
descending order with MSVC `std::sort` (`FUN_18010fe80` plus helpers
`FUN_18010f690`/`f800`/`f900`/`fd30`: introsort, insertion sort for ≤ 32
elements, heap sort fallback). The sort is not stable, so the order of
equal keys depends on the MSVC algorithm. With ≤ 13 dumpers only the
insertion-sort path runs, and that path is stable: an element moves only
past strictly smaller keys.

## 7. Replay harness design

Because Sony's DLL has no replay path and does not record sensor streams on
PC (§1), use our own USB-level capture and feed Sony's parsing entry points
directly:

1. **Record** at the transport layer (our kernel module / userspace driver):
   one file per stream in the `psvr2_trace` container (`psvr2_trace.h`, P2TR),
   record = {u32 tag, u32 size, u64 host_time_ns, payload}. Tags: bulk
   endpoint number (`0x83` SLAM, `0x88` status/IMU, `0x89` LD, `0x8b` VD,
   `0x87` image), control-report replies (`0x100 | report id`), Sense HID
   input reports (`0x200 | pad`).
2. **Replay into Sony's code** under Wine: load the DLL (`sony.h`), and for
   each subsystem call the function that consumes one transfer, in record
   order, with a fake device context whose clock returns the recorded
   host time. Entry points already identified: LD packet → IR tracker
   (`FUN_18018aaf0` state machine consumes the packet copy), SLAM record
   → `FUN_18017a5d0`, VD → `FUN_180127250` body, Sense input report →
   libpad parse. Each consumer touches Sony globals (device singleton
   `FUN_180124c90`); the harness must construct that object with Sony's
   own constructor and patch the USB vtable with stubs that return the
   recorded bytes.
3. **Replay into ours** with the same file and compare outputs per record
   (poses, LED-sync commands, …) bit for bit or with the stated ulp
   tolerance.

`userspace/lib/sie/trace/psvr2_trace.[ch]` implements the container and a
reader for every Sony stream file in §4.2, so a Sony capture made on a PS5
devkit or by a future build that wires the producers can be replayed the
same way.

## 8. Shared math (`0x180160000–0x18016ffff`) [sure]

Implementation: `userspace/lib/sie/trace/psvr2_math.[ch]`, all bit-exact
(test `t_trace_math.c`). Conventions: column-major matrices (element
(r, c) at `m[c*N + r]`), w-first quaternions, pose = float[10] {position,
quaternion, scale}. Every dot product / matrix element is summed in index
order k = 0, 1, 2(, 3), each term `a[r,k]*b[k,c]`; Sony's build has no FMA.
Behaviour per function (only non-obvious points listed):

- `0x18016baf0` 3x3 product, `0x18016c630` in place; `0x18016bd20` 4x4
  product, `0x18016c810` in place; `0x18016bc90` 3x3·vec3;
  `0x18016c130` 4x4·vec4.
- `0x18016df80` quaternion → 4x4: divides by the norm first (squares
  summed x, y, z, w), then the usual `1 − 2(y² + z²)` form, written as
  `(1 − 2y·y) − 2z·z` with the doubled factors formed as `(y + y)·y`;
  translation zero, `m15 = 1`. `0x18016da40` is the same with swapped
  arguments.
- `0x18016dc50` pose → T·R·S; `0x18016db00` the same from separate scale,
  quaternion, position; `0x18016dda0` inverse S⁻¹·Rᵀ·T(−p) (reciprocal
  scales, `1/s`).
- `0x18016c220` / `0x18016cc80` Hamilton product: w term
  `((a0b0 − a1b1) − a2b2) − a3b3`; vector terms start with the cross
  product term, then add `a0·b_i` and `a_i·b0`.
- `0x18016c360` rotate a vector by the matrix of a quaternion;
  `0x18016c420` pose product (position not scaled by the parent's scale;
  scales multiply).
- `0x18016ec10` Euler → quaternion, double cos/sin of the half angles
  (halved in float).
- `0x18016d270` / `0x18016f2d0` axis-angle → quaternion (float sinf/cosf;
  axis not normalised); `0x18016d9a0` → 4x4 via the normalising
  quaternion → matrix.
- `0x18016d170` angle in degrees between two 2D vectors in [0, 360):
  normalise (as 3D with z = 0, `0x180170840`: length below 1e-10 leaves the
  vector unchanged), `acosf(dot)·180/π`, mirrored to `360 − a` when the 2D
  cross product is negative.
- `0x18016d430` / `0x18016ee40` look-at, `0x18016d800` / `0x18016f210`
  perspective (fovy in degrees, `1/tan` in double), `0x18016ed80` /
  `0x18016d370` frustum: OpenGL forms (see header comments).
- `0x18016e150` (3x3) / `0x18016e3c0` (4x4) rotation matrix → quaternion
  (Shepperd's method, largest of the four `1 ± …` candidates, ties to the
  first; NaN candidates are never chosen over a number; negative → identity).
- `0x18016e630` / `0x18016e910` 4x4 → scale, quaternion, position: column
  lengths with the square root taken in double; a column whose four
  entries (including the projective row) are all non-zero gets a negative
  scale; any zero scale → identity rotation and failure (return 0 / −1).
- `0x18016f350`, `0x18016f3e0`, `0x18016f470` rotations about x, y, z with
  the angle in degrees (`deg·π/180` in float); `0x18016f540` translation.
- `0x18016f590` in-place 4x4 inverse: adjugate times `1.0f / det`, with no
  check for a singular matrix. Every cofactor is three triple products
  added left to right, then three subtracted one by one. Each triple is
  `(m[a]·m[b])·m[c]`, and the pair multiplied first matters for rounding.
  The determinant is 12 quadruple products `((m[a]·m[b])·m[c])·m[d]` added,
  then 12 subtracted, in that order. All terms read the input matrix. The
  exact index tables are `inv_cof` / `inv_det` in `psvr2_math.c`.
- `0x180168e40` Rodrigues vector → 3x3 (double, **row-major**):
  `R = (c·I + (1−c)·k kᵀ) + s·[k]×`, θ < 2⁻⁵² → identity (NaN θ takes the
  rotation path). The output is a small matrix object (9 doubles, then
  int64 rows 3, cols 3, step 24 bytes).

Not reimplemented (library code, OpenCV-derived; use OpenCV or Eigen):
`_SVDcompute` (`0x1801635f0`), the one-sided Jacobi SVD kernels
(`0x180163da0` float, `0x1801646a0` double), the matrix → Rodrigues path
(`0x180168a00`, which orthonormalises through the SVD), `cvRodrigues2`-style
dispatcher with Jacobian (`0x180168490`, "r_vec needs matrix") and the
fisheye/projection helpers around `0x180166200..0x18016a8b0`.

## 9. Framework threading and clocks (spec only) [sure]

Thin Win32 wrappers (`Framework::*`, objects are {vtable, HANDLE}):

- Mutex: `CreateMutexA(NULL, FALSE, name)`; lock = `WaitForSingleObject`
  (timeout argument, INFINITE in practice), returns true on
  `WAIT_OBJECT_0` or `WAIT_ABANDONED`; unlock = `ReleaseMutex`.
- Event: `CreateEventA(NULL, TRUE /*manual reset*/, FALSE, name)`; set,
  reset, wait (`0` means INFINITE), try-wait (timeout 0, true if
  signalled).
- Semaphore: `CreateSemaphoreA(NULL, initial, max, name)`.
- Thread: `_beginthreadex` running the object's virtual `run`; optional
  affinity mask (`-1` = none) and priority level 0..4 → Win32
  `-1, 0, 1, 2, 15`; join via `_Thrd_join`.
- Clocks: `0x18016b110` µs and `0x18016b0a0` ms since boot, from
  `QPC / QPF` in double times 1e6 / 1e3, converted to u64 (values ≥ 2⁶³
  wrap through the high bit); `0x18016b180` raw QPC; `0x18016aff0` UTC
  wall clock as {seconds since 1970, milliseconds·1000}.

## 10. Universe / chaperone code in `0x18011e000–0x18011ffff` [sure unless tagged]

This code is not VrTrace. It is VrTracker2's map-state and chaperone
glue that the linker put in the trace range. Its callers are in
`0x18012xxxx` (map + SteamVR) and in irtracker.

Conventions differ from §8: quaternions are **x-first** (`x, y, z, w`), and
a pose7 is `position[3], quaternion[4]`. Implemented and difftested in
`userspace/lib/sie/trace/psvr2_chaperone.{h,c}` (`t_trace_chap`).

- `0x18011e190` pose compose `out = a ∘ b`. Position: `t = 2·(a.q.xyz ×
  b.p)`, then `p = ((a.w·t + b.p) + (a.q.xyz × t)) + a.p` for each axis.
  Orientation: Hamilton product `a.q · b.q`, each component formed as
  (two products) + (two products). This is a shared helper; irtracker
  calls it too. The exact operand order is in `psvr2_chap_pose_mul`.
- `0x1801613a0` pose7 inverse: orientation = conjugate (no
  normalisation); position = `−((t·w + p) + (c.xyz × t))` with
  `t = 2·(c.xyz × p)`, `c` the conjugate. Shared helper (irtracker and
  calibration code call it); `psvr2_chap_pose_inv`.
- `0x180121360` (in the 0x18012 range, needed here) x-first quaternion → 3x3
  without normalisation, `y2 = y+y`, `z2 = z+z`, `x·(x+x)`, `w·(x+x)`.
- `0x18011f420` yaw of a quaternion, in radians. Let `X` be the image of
  the x axis and `Z` the image of the z axis. Sony forms each component as
  a sum with `0·` terms, so `−0` and NaN behave as in Sony. If `|Z.y| ≤ |X.y|`,
  use Z: normalise `(Z.x, Z.z)` by `sqrt((Z.z² + 0) + Z.x²)` (only when that
  sum is `> 0`); then cos = `Z.z` and the sign source = `Z.x`. Otherwise
  use X: normalise `(X.x, X.z)` unless the sum is `≤ 0`; then cos = `X.x`
  and the sign source = `−X.z`. Result: `acosf(cos) · (−s/|s|)`, or
  `acosf(cos) · 0` when the sign source `s` is 0. Every call logs
  "[VrTracker2] yaw %f".
- `0x18011ef90` default play area (used when there is no chaperone). It
  reads the headset pose7 (`0x18011f1b0`: shared-memory pose at +0x20
  composed with the device pose at +0x7c; identity when unavailable). It
  then fills the 0x830-byte area block (`struct psvr2_chap_area`):
  - `+0` = 0; `+4` and `+8` = `sqrt(2)·1.5`.
  - 24 points `(cos(i·15°)·1.5, sin(i·15°)·1.5)` (ucrt `cosf`/`sinf`;
    angle `(float)i · 0.2617994f`).
  - standing = `(−x, −(y − 1.5), −z, yaw)`; seated = `(−x, −y, −z, yaw)`.
    `yaw` is the yaw of the inverse orientation: conjugate / `|q|²`, with
    `|q|² = (x² + z²) + (y² + w²)` (SIMD horizontal add; the decompiler shows a misleading serial order), or the zero quaternion unless `|q|² > 0` (NaN included).
- `0x18011e8e0` chaperone_info JSON builder (jsoncpp, spec only). It
  increments the universe id at `+0x20`, skipping 0, and adds one universe:
  - `seated` / `standing` = `{translation: [x, y, z], yaw}`.
  - `play_area` = `[w, h]`.
  - `collision_bounds`: for each polygon edge i → (i+1) mod n, a quad
    `[[x_i, 0, z_i], [x_i, 2.43, z_i], [x_j, 2.43, z_j], [x_j, 0, z_j]]`.
    The wall height is the double of float 2.43.
  - `universeID` (u64) and `time` (caller's value).
  Root: `version` = 5, `jsonid` = "chaperone_info", `universes` = [...].
  Floats are widened to double for jsoncpp.
- `0x18011f7d0` universe initialise (once per object, flag `+0x28`). It
  reads string property 0x800 of the given container through
  `IVRProperties_001` as the chaperone file path (std::string at `+0`). It
  reads the file in 255-byte `fgets` chunks and parses it with
  jsoncpp (comments allowed). On success it stores
  `universes[0].universeID` (u64) at `+0x20`; otherwise it logs "chaperone
  parse failed" or "chaperone empty".
- Map state singleton (`0x18011f300`, 0x8f0 bytes, mutex at `+0`):
  `+0x50` i32 map id (−1), `+0x58` u64, `+0x60` u8 map-initialised, `+0x64`
  u32, `+0x68` i32 state (5 at creation, 3 after reset / map
  initialised), `+0x6c` i32 previous map id (−1), `+0x70` u8, `+0x90` 0x850-
  byte area copy, `+0x8e0` u8 (0xff), `+0x8e4` u8 pending flag, `+0x8e8` i32
  pending map id.
  - `0x18011fff0` "mapInitialized": previous = map id, map id = −1, `+0x58`
    = 0, initialised = 1, state = 3.
  - `0x18011e510` / `0x18011f710` "reset": try `loadFile`; on failure (the
    second variant: always) map id = −1, `+0x58`/`+0x64` = 0,
    initialised = 0, state = 3. Then call `0x180120f20(obj, 1)` (0x18012
    range).
  - `0x18011f3d0` get state; `0x18011e650` "map changed" =
    pending flag set ∧ map id ≠ −1 ∧ map id ≠ pending id; `0x18011faf0`
    clears the pending flag (id −1).
  - `0x18011fc50` initialises the universe object, loads the area block
    (`0x180120570`), applies it (`0x1801214a0`), copies it to `+0x90`, and
    publishes it to shared memory (`0x18015f9d0`).
- `0x18011fd30` "loadFile": `<string property 6000 of the driver
  handle>` + `\sceBoundaryMeta.bin`. Property 6000 is the user config path
  in OpenVR [likely]. It reads a 0x48-byte header. Valid means u16
  magic `0x4d42` ("BM"), u16 version < 2, and u32 size field 0x48. If valid,
  it copies under the mutex: `+8` → map id `+0x50`, `+0x10` (u64, only if
  `+0x70` is set) → `+0x58`, `+0x18` → `+0x64`, `+0x28` (u8) → `+0x8e0`.
  Returns 0, −1 (no file / short read), or −2 (bad header). The writer is in
  the 0x18012 range (`0x180120b40`, `0x180120c80`, `0x180120f20`).
  [likely: field meanings beyond map id]

## 11. VrController pad reader in `0x18011b3c0–0x18011d6f0` (outline only) [likely]

This is not VrTrace either. It is the Sense-controller side of VrTracker2,
built on the embedded libpad (`0x1801b0000+`). Only an outline is given
here; the functions stay `todo` in trace.tsv. A full spec belongs with the
libpad / controller-tracker work.

- `0x18011d0c0` opens a pad: "libpad access aquired/released",
  `scePadOpen`, `scePadSetDynamicCalibrationState`,
  `scePadVrControllerSetVibrationMode` (logged with their return codes).
- `0x18011bae0` per-controller poll (object: lock at `+0x10`/`+0x20`, pad
  handle `+0x30`, side `+0x34` (0 = one hand, else the other), connected
  `+0x67`, bus type `+0x68` (0..2, names from `PTR_DAT_18034d020`)):
  - reads up to 64 states (0x78 bytes each) plus 64 extension records
    (0x40 bytes each) through `0x1801bde70`.
  - A state with byte +0x40 = 0 means disconnected ("VrController(%s)
    Disconnected"). On a new connection it reads controller info
    (`0x1801bf670`), enables motion sensors (`scePadSetMotionSensorState`),
    reads the firmware version (`0x1801bd740`), and stores a per-side
    constant offset vector:
    - side 0: (−20.0727, −12.2481, −6.0039)
    - other side: (9.3727, −12.2481, −6.0039)

    It then sets vibration strength (`scePadSetVibrationStrength`) from a
    shared-memory setting.
  - Each state goes into a 256-entry history (`+0x1588 + 0x78·i`; extension
    at `+0x8d88 + 0x40·i`; index `+0xcd88`). It is also published to
    shared memory (`0x18015a160` / `0x18015b780`, slot 1 + side).
  - Battery / connection statistics with thresholds 40000, 16000 and 12000
    (each divided by 2000). A 3-second staleness check on an unchanged
    timestamp. A rolling 10000-sample counter. A 1-second call to
    `0x18011a870`, which builds statistics with `std::sort`.
- `0x18011a3d0` / `0x18011a5b0` reset and disconnect the controller
  state; `0x18011d3d0` / `0x18011d580` / `0x18011d6f0` are timing and
  pose helpers around it.

## 12. Calibration blob parser (`FUN_180161b70`) [sure; field meanings guess]

This is a pure function `(unused, blob, size, out) → bool`. Callers are
`0x180156500` and irtracker `0x180182e80`. Implemented in
`userspace/lib/sie/trace/psvr2_calib.{h,c}` and difftested in
`t_trace_calib` (0/20000).

Blob header (0x18 bytes):

| Off | Type | Meaning |
|-----|------|---------|
| 0 | char[3] | `"SIE"` |
| 3 | u8 | version, must be ≤ 1 |
| 4 | u16 | total size, must equal the size argument |
| 6 | u8 | copied to out `+0x7bc` |
| 7 | char[8] | version string, copied to out `+0x7bd` (16 bytes, zero padded) |
| 0xf | u8 | camera count `nc` → out `+0x550` (u32) |
| 0x10 | u8 | IMU count `ni` → out `+0x7b8` (u32) |

The fields at `+0x7bc`, `+0x7bd`, `+0x550` and `+0x7b8` are written before
validation, so they are set even when the blob is rejected.

The version string selects the record format. If a `'d'` appears after a
`':'`, records hold doubles and are 0xe4 bytes; otherwise they hold floats
and are 0x94 bytes. Validation, in order:
- `size ≥ 0x18 + nc·rec + ni·0x34`;
- the magic is `"SIE"`, version ≤ 1, and the u16 size equals `size`.

Returns 1 or 0 (only the low byte is meaningful).

Camera record i, from blob `+0x18 + rec·i`, goes to out `+0x110·i`:
- u8 `+0` → u32 `+0`; u16 `+4` → u32 `+4`; u16 `+6` → u32 `+8` (id,
  width, height [guess]);
- 9 floats from `+8` → doubles at `+0x10` (intrinsics [guess]);
- 20 values from `+0x2c` → doubles at `+0x70`. In float records they are
  widened from float; in double records they are copied raw (distortion
  and extrinsics [guess]).

The rest of each record is ignored. Then IMU record i (0x34 bytes, after
the cameras) goes to out `+0x558 + 0x98·i`:
- u8 `+0` → u32 `+0`; u8 `+1` → u32 `+4`;
- 9 floats `+4` → doubles `+8` (rotation [guess]);
- 3 floats `+0x28` → doubles `+0x68` (offset [guess]).

There is no bound on the counts. Camera 5 overwrites the count and IMU
area, and the fifth IMU record (index 4) lands on `+0x7b8`; the write order (all
cameras, then all IMUs) decides what survives.
