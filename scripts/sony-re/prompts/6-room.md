# Topic: room map and play area (`docs/sony-re/room.md`) — priority 6

Read `common.md` first. Earlier notes: `docs/sony-re/reloc.md`. Lowest
priority: only needed for persistent room setup.

Scope: the "RP" record from EP `0x8a`; the relocaliser
(`0x1801e0000–0x18023ffff`) at the level of what it does with the record and
what it outputs; the map files (`sceMapDb.bin`, `sceBoundaryMeta.bin`,
`playarea_data.bin`) and their on-disk formats; save/load scheduling
(`FUN_180120290`, `FUN_1801abf90`, `FUN_1801ac1f0`, `FUN_1801ac4e0`); play
area, chaperone and universe handling (`FUN_180120570`).

Deliver: the file formats as tables; when maps are saved, loaded and
discarded; what the headset must be sent for relocalisation (if anything);
and how play-area bounds reach SteamVR.

Skip: keyframe and pose-graph internals beyond a one-paragraph summary,
unless the driver must reproduce them to use a saved map.
