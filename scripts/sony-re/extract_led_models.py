#!/usr/bin/env python3
"""Print the Sense controller LED models from the user's own copy of Sony's
PSVR2 SteamVR driver, as JSON (millimetres, controller frame).

Offsets are for the build documented in docs/sony-driver-re.md (PE timestamp
1741940190). The script checks every header, so a different build fails
loudly instead of printing garbage.

usage: extract_led_models.py driver_playstation_vr2(_orig).dll > models.json
"""
import json
import struct
import sys

IMAGE_BASE = 0x180000000
FIRST_MODEL = 0x180257280
MODEL_STRIDE = 0x1400
MODEL_COUNT = 7
MAX_LEDS = 32


def read_sections(path):
    with open(path, "rb") as f:
        data = f.read()
    pe = struct.unpack_from("<I", data, 0x3C)[0]
    nsec = struct.unpack_from("<H", data, pe + 6)[0]
    opt = struct.unpack_from("<H", data, pe + 20)[0]
    sec = pe + 24 + opt
    out = []
    for i in range(nsec):
        vsize, va, rsize, raw = struct.unpack_from("<IIII", data, sec + i * 40 + 8)
        out.append((va, max(vsize, rsize), raw))
    return data, out


def reader(path):
    data, sections = read_sections(path)

    def read(va, n):
        rva = va - IMAGE_BASE
        for sva, size, raw in sections:
            if sva <= rva and rva + n <= sva + size:
                return data[raw + rva - sva: raw + rva - sva + n]
        raise ValueError(f"address {va:#x} not in file")

    return read


def main():
    read = reader(sys.argv[1])
    models = []
    for m in range(MODEL_COUNT):
        base = FIRST_MODEL + m * MODEL_STRIDE
        hdr = struct.unpack("<6i", read(base, 24))
        count = hdr[0]
        if not 0 < count <= MAX_LEDS or hdr[2:4] != (2, 1):
            sys.exit(f"model {m}: unexpected header {hdr}; different driver build?")
        pos = struct.unpack(f"<{MAX_LEDS * 3}f", read(base + 0x18, MAX_LEDS * 12))
        nrm = struct.unpack(f"<{MAX_LEDS * 3}f", read(base + 0x18 + MAX_LEDS * 12, MAX_LEDS * 12))
        models.append({
            "address": hex(base),
            "header": hdr,
            "leds": [{"position_mm": pos[i * 3:i * 3 + 3], "normal": nrm[i * 3:i * 3 + 3]}
                     for i in range(count)],
        })
    json.dump(models, sys.stdout, indent=1)
    print()


if __name__ == "__main__":
    main()
