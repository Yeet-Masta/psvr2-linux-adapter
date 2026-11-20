#!/bin/sh
# Switch SteamVR back to this project's driver: disable Sony's plug-in,
# enable driver_psvr2 (and clear a safe-mode block), load the dev psvr2.ko.
# Run with SteamVR closed.
set -e
here=$(cd "$(dirname "$0")" && pwd)
cfg=$HOME/.steam/debian-installation/config/steamvr.vrsettings
if pgrep -x vrserver >/dev/null; then
	echo "close SteamVR first" >&2
	exit 1
fi
tmp=$(mktemp)
jq '.["driver_SteamVR_Plug-In"].enable = false | .driver_psvr2.enable = true | .driver_psvr2.blocked_by_safe_mode = false' "$cfg" > "$tmp"
cp "$tmp" "$cfg"
rm -f "$tmp"
if ! lsmod | grep -q '^psvr2 '; then
	sudo modprobe -a industrialio kfifo_buf videobuf2_vmalloc videobuf2_v4l2 videodev
	sudo insmod "$here/../../kernel/psvr2.ko"
fi
echo "psvr2 driver selected"
