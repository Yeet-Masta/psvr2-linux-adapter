#!/bin/sh
# Differential tests: our clean implementations (userspace/lib/sie) against
# Sony's driver, called in-process under Wine.
#
# usage: run.sh <driver_playstation_vr2(_orig).dll> [subsystem]
#
# A test t_<subsystem>_<name>.c is built with userspace/lib/sie/<subsystem>/*.c
# and userspace/lib/sie/common/*.c only, so one subsystem's work in progress
# never breaks another's tests. With [subsystem], only its tests run.
# Needs wine64 and zig (pip install ziglang; or ZIG=/path/to/zig).
set -eu
here=$(cd "$(dirname "$0")" && pwd)
lib=$here/../../../userspace/lib/sie
zig=${ZIG:-zig}
out=${TMPDIR:-/tmp}/psvr2-difftest-$$
mkdir -p "$out"
cp "$1" "$out/sony.dll"
status=0
for t in "$here"/t_${2:-*}_*.c; do
	[ -e "$t" ] || continue
	name=$(basename "$t" .c)
	sub=$(echo "$name" | cut -d_ -f2)
	srcs=$(ls "$lib/$sub"/*.c "$lib/common"/*.c 2>/dev/null || true)
	"$zig" cc -target x86_64-windows-gnu -O2 -ffp-contract=off -I"$here" -I"$lib/$sub" -I"$lib/common" \
		"$t" $srcs -o "$out/$name.exe" -lm || { status=1; continue; }
	(cd "$out" && WINEDEBUG=-all wine64 "$name.exe" sony.dll) || status=1
done
rm -rf "$out"
exit $status
