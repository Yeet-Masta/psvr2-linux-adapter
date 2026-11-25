# Sony PSVR2 driver: documentation pass (shared brief)

You are documenting how Sony's PSVR2 SteamVR driver
(`driver_playstation_vr2_orig.dll`, PE timestamp 1741940190) works, so that
another engineer can implement the same behaviour in an open Linux driver: a
kernel module (`/home/yeetmasta/Claude/psvr2/src/kernel`, USB/HID/display
side) plus userspace (`userspace/`, `steamvr/driver_psvr2`). That engineer
will only read your document. They will not read Sony's code.

The goal is **understanding, not reimplementation**. Do not write C
implementations or difftests. Read the decompiled code, trace it from the
entry points below, and write down what it does and why.

## Tools (read-only; do not modify)

- `/home/yeetmasta/psvr2-re-work/decomp.c`: Ghidra decompile of all 5175
  functions (`// ==== FUN_x @ addr` headers).
- `/home/yeetmasta/psvr2-re-work/fn.sh FUN_x`: print one function.
- `scripts/sony-re/trace.py` (in the repo). Run it from `psvr2/src`. Library
  code is pruned automatically.
  - `trace.py tree FUN_x -d 3`: call tree from an entry point, with sizes and
    log strings.
  - `trace.py callers FUN_x`: who calls a function.
  - `trace.py find 'regex'`: functions by log string. Log strings are the
    fastest way to name things.
  - `trace.py info FUN_x`: size, strings, callers and callees.
  - `trace.py bundle FUN_x -d 3`: write an entry and everything it reaches to
    one file in `/home/yeetmasta/psvr2-re-work/bundles/`, for top-down
    reading.
- Constants and tables: read them from the DLL with python (image base
  0x180000000; see `scripts/sony-re/extract_led_models.py` for a section
  reader). `uv` is available.
- Sony DLL:
  `/home/yeetmasta/.steam/debian-installation/steamapps/common/PlayStation VR2 App/SteamVR_Plug-In/bin/win64/driver_playstation_vr2_orig.dll`.
- Wine and zig are available at `/home/yeetmasta/psvr2-re-work/` if you need
  to confirm one ambiguous computation by calling Sony's function. Use this
  rarely. It is not the method.

## Read first

1. `overview.md` part B: the protocol already decoded (USB transport,
   reports, endpoints, LED sync, calibration). Do not redo it. Extend it and
   correct it.
2. The earlier per-subsystem notes in `docs/sony-re/*.md` that your topic
   names. They come from a reimplementation pass. Mine them for facts.

## What matters, and what to skip

Spend effort in proportion to what the Linux driver needs.

**High value** (document completely, with exact values):
- Every byte the driver sends to or reads from the headset or controllers:
  report IDs, field layouts, units, ranges and endianness.
- The order and timing of commands. Include init, mode changes, shutdown,
  suspend/resume, error and retry paths, and timeouts.
- State machines: the states, what moves between them, and what is sent on
  each transition.
- Constants that change behaviour: rates, clamps, thresholds, periods, gains
  and magic numbers. Give the value and what it controls.
- Data a userspace stack needs to consume: calibration layouts, coordinate
  frames and conventions, timestamps and clock domains.

**Medium value** (explain the algorithm, not every helper):
- Tracking and fusion algorithms. Give the stages, the inputs and outputs of
  each, key equations in pseudo-C, tuning constants, and failure handling.

**Skip** (one line at most, then move on):
- CRT/STL, jsoncpp, linear algebra/BLAS, threading wrappers, logging and
  string formatting. Generic math helpers get one line, such as "quaternion
  multiply, (w,x,y,z) order".
- UI dialogs, localisation, debug pages, VrTrace dumpers, telemetry and
  settings plumbing. The exception is a setting that changes device
  behaviour: note its name and default.
- Getters, setters, constructors, destructors and copy routines.

Rule of thumb: if a Linux driver author would not change a line of code
because of what you learned, do not write it up. When a function is large,
find the few decisions it makes and describe those.

## Writing rules

- Use your own words. Do not paste or transliterate decompiled code.
  Pseudo-C is fine for algorithms and sequences, but write it as a clean
  description with meaningful names, not Ghidra's variable soup.
- Tag each claim **[sure]** (read in code), **[likely]** (strong inference)
  or **[guess]**.
- Cite the function VA for each fact (for example "`FUN_180128590`") so it
  can be checked. Cite addresses only; never quote Sony's code.
- Never commit anything, and never switch branch. Never send Sony code or
  data off this machine. Bundles stay in `/home/yeetmasta/psvr2-re-work/`.
- Write only the topic file you were given (one of the category files in
  `docs/sony-re/`). Other sessions edit other files at the same time. When
  you find a statement elsewhere is wrong, list it (where, old, correct) at
  the end of your report; the merging session fixes it in place and adds a
  row to the errata table, `overview.md` §A.12. Never add a separate
  "Corrections" section.
- Save often. Write each finding into the doc as soon as you confirm it, so
  the work survives if you stop.

## Document structure (`docs/sony-re/<topic>.md`)

1. **Driver summary.** About a screen. Say what the Linux driver must do for
   this topic, in priority order. Say which parts belong in the kernel
   module and which in userspace.
2. **Wire and data formats.** Tables: offset, size, type, meaning and units.
3. **Sequences.** Numbered steps with timing, for example "send 0x0b mode 2,
   wait for 0x88 status bit 3, then …".
4. **State machines.** States, events and actions.
5. **Algorithms.** Pseudo-C plus constants.
6. **Constants.** Name, value, meaning and source VA.
7. **Gaps in the current Linux driver.** Compare with
   `/home/yeetmasta/Claude/psvr2/src/kernel`, `userspace/` and
   `steamvr/driver_psvr2/src`. Do not edit those files.
8. **Open questions.** Include what needs a hardware capture to confirm.
9. **Function map.** VA, role in one line, and whether you covered it.
   Include only the functions you looked at.

## When to stop

Stop when the high-value list for your topic is fully covered, or when you
run out of budget. Your last message is a short report: what is now
documented, the open questions, and anything the user must run on hardware.
