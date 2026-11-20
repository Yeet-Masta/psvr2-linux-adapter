#!/bin/sh
# Switch SteamVR to Sony's driver (via Ignition + PSVR2Toolkit): enable
# driver_SteamVR_Plug-In, disable driver_psvr2, unload psvr2.ko (Sony's
# driver opens the headset through libusb and needs it unclaimed).
# Undo with use-ours.sh. Run with SteamVR closed.
set -e
cfg=$HOME/.steam/debian-installation/config/steamvr.vrsettings
if pgrep -x vrserver >/dev/null; then
	echo "close SteamVR first" >&2
	exit 1
fi
cp "$cfg" "$cfg.before-sony"
jq '.["driver_SteamVR_Plug-In"].enable = true | .driver_psvr2.enable = false' "$cfg.before-sony" > "$cfg"
if lsmod | grep -q '^psvr2 '; then
	sudo rmmod psvr2
fi
echo "Sony driver selected (settings backup: $cfg.before-sony)"
