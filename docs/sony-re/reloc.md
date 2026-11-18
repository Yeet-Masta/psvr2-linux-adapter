# Relocaliser and room map (`reloc`)

Behavioural notes on Sony's relocaliser / room-map backend in
`driver_playstation_vr2_orig.dll` (PE timestamp 1741940190, image base
`0x180000000`), VA range `0x1801e0000–0x18023ffff`, plus the "RP" record
parser it is fed by. Written in our own words from the decompile; no Sony
code is reproduced. Our implementation lives in
[`userspace/lib/sie/reloc/`](../../userspace/lib/sie/reloc/); differential
tests are `scripts/sony-re/difftest/t_reloc_*.c`; per-function status is in
[`scripts/sony-re/status/reloc.tsv`](../../scripts/sony-re/status/reloc.tsv).

Offsets are hexadecimal bytes. "kf" = keyframe.

## Pipeline

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

## RP frame (unpack of the EP 0x8a record) — `FUN_1801ab410`

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

## Room map file `sceMapDb.bin` and the in-memory map image

The map database is one flat, fixed-size byte image (0x12d18a0 bytes) that
lives in memory (pointer global `DAT_1805e5d50`) and is written to disk
verbatim. All tables are fixed-capacity arrays with occupancy bitmaps; there
are no pointers inside, so the file is position independent.

### File header (0x28 bytes) [sure]

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

### Image layout [sure: offsets and capacities; meanings marked]

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

### Keyframe slot (0x2ed20 bytes)

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

### Allocation rule (keyframes, id table, edges)

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
