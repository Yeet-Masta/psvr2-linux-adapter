#!/bin/sh
# Decompile Sony's PSVR2 SteamVR driver with Ghidra (headless) for
# interoperability research. Output stays local: it is Sony's code and must
# not be committed. Findings go to docs/sony-re/overview.md part B in our own words.
#
# usage: decompile.sh <ghidra_install_dir> <driver_playstation_vr2(_orig).dll> <out_dir>
# Produces <out_dir>/decomp.c (every function) and strings_xref.tsv
# (address, referencing functions, string).
set -eu
ghidra=$1 dll=$2 out=$3
here=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$out/project"
"$ghidra/support/analyzeHeadless" "$out/project" psvr2 -import "$dll" \
	-overwrite -scriptPath "$here" -postScript DumpAll.java "$out" -max-cpu "$(nproc)"
