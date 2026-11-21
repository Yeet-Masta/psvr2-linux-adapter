#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0
#
# Undo install.sh:
#   kernel   unload psvr2, remove the DKMS module and /usr/src copy
#   udev     /etc/udev/rules.d/99-psvr2.rules
#   steamvr  the SteamVR driver symlink and the Vulkan layer manifest
#   settings steamvr.vrsettings keys install.sh added (and the driver_psvr2
#            section with --set overrides); a .psvr2.bak copy stays
#   lib      libpsvr2 from /usr/local (only when named, or with --all)
#
# Usage:
#   ./uninstall.sh               kernel udev steamvr settings
#   ./uninstall.sh steamvr       only the named parts
#   ./uninstall.sh --all         also lib and the install state
#
# Your "input" group membership is left alone (sudo gpasswd -d "$USER" input).
# Run as your normal user (root steps go through sudo) or with sudo.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME=psvr2
VERSION="$(sed -n 's/^PACKAGE_VERSION="\(.*\)"/\1/p' "${SCRIPT_DIR}/kernel/dkms.conf")"
SRC="/usr/src/${NAME}-${VERSION}"
RULES_DST=/etc/udev/rules.d/99-psvr2.rules
STATE_ROOT=/var/lib/psvr2

die() { echo "error: $*" >&2; exit 1; }
say() { echo ">> $*"; }
note() { echo "   $*"; }

if [ "$(id -u)" -eq 0 ]; then
	TARGET_USER="${SUDO_USER:-root}"
	as_root() { "$@"; }
	as_user() { if [ "${TARGET_USER}" = root ]; then "$@"; else sudo -u "${TARGET_USER}" -H "$@"; fi; }
else
	TARGET_USER="$(id -un)"
	as_root() { sudo "$@"; }
	as_user() { "$@"; }
fi
USER_HOME="$(getent passwd "${TARGET_USER}" | cut -d: -f6)"
[ -n "${USER_HOME}" ] || USER_HOME="${HOME}"
USER_STATE="${USER_HOME}/.local/state/psvr2"

ALL=0
PARTS=()
while [ $# -gt 0 ]; do
	case "$1" in
	--all) ALL=1 ;;
	kernel | udev | steamvr | settings | lib) PARTS+=("$1") ;;
	-h | --help) sed -n '3,19p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
	*) die "unknown argument: $1 (see --help)" ;;
	esac
	shift
done
[ ${#PARTS[@]} -gt 0 ] || PARTS=(kernel udev steamvr settings)
[ "${ALL}" -eq 1 ] && PARTS+=(lib)
want() { local p; for p in "${PARTS[@]}"; do [ "$p" = "$1" ] && return 0; done; return 1; }

steamvr_running() { pgrep -x vrserver >/dev/null 2>&1 || pgrep -x vrcompositor >/dev/null 2>&1; }

if want kernel; then
	say "kernel module"
	if lsmod | grep -q "^${NAME} "; then
		steamvr_running && note "SteamVR is running: the unload may fail until it is closed"
		as_root modprobe -r "${NAME}" 2>/dev/null || note "could not unload ${NAME} (in use?); it goes on reboot"
	fi
	if command -v dkms >/dev/null 2>&1 && dkms status -m "${NAME}" -v "${VERSION}" 2>/dev/null | grep -q "${NAME}"; then
		as_root dkms remove -m "${NAME}" -v "${VERSION}" --all >/dev/null || true
		note "DKMS ${NAME}/${VERSION} removed"
	fi
	as_root rm -rf "${SRC}"
	as_root rm -f "${STATE_ROOT}/kernel.sha"
fi

if want udev; then
	say "udev rules"
	as_root rm -f "${RULES_DST}"
	as_root udevadm control --reload-rules 2>/dev/null || true
fi

if want steamvr; then
	say "SteamVR driver"
	as_user env HOME="${USER_HOME}" STEAMVR_DRIVERS="${STEAMVR_DRIVERS:-}" \
		"${SCRIPT_DIR}/steamvr/install-steamvr-driver.sh" --uninstall 2>&1 | sed 's/^/   /' || true
fi

if want settings; then
	say "SteamVR settings"
	if steamvr_running; then
		note "SteamVR is running and would overwrite the file on exit: skipped (close it and re-run)"
	elif ! command -v python3 >/dev/null 2>&1; then
		note "python3 not found: remove displayPortTrainingMode / driver_psvr2 by hand"
	else
		for f in "${USER_HOME}/.local/share/Steam/config/steamvr.vrsettings" \
			"${USER_HOME}/.steam/steam/config/steamvr.vrsettings" \
			"${USER_HOME}/.steam/root/config/steamvr.vrsettings"; do
			[ -f "$f" ] || continue
			as_user python3 - "$f" "${USER_STATE}/vrsettings-added" <<'PY'
import json, os, sys
path, added_log = sys.argv[1], sys.argv[2]
with open(path, encoding="utf-8") as fh:
    text = fh.read().strip()
data = json.loads(text) if text else {}
keys = []
if os.path.exists(added_log):
    with open(added_log, encoding="utf-8") as fh:
        keys = [l.strip() for l in fh if "." in l]
gone = []
for k in keys:
    sec, _, key = k.partition(".")
    if sec == "driver_psvr2":
        continue  # whole section below
    if key in data.get(sec, {}):
        data[sec].pop(key)
        gone.append(k)
        if not data[sec]:
            data.pop(sec)
if data.pop("driver_psvr2", None) is not None:
    gone.append("driver_psvr2")
if gone:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=3, sort_keys=True)
        fh.write("\n")
    os.replace(tmp, path)
print("   %s: %s" % (path, "removed " + ", ".join(gone) if gone else "nothing to remove"))
PY
			break
		done
		as_user rm -f "${USER_STATE}/vrsettings-added"
	fi
fi

if want lib; then
	say "libpsvr2 (/usr/local)"
	for f in /usr/local/include/libpsvr2.h /usr/local/lib/libpsvr2.a /usr/local/lib/libpsvr2.so \
		/usr/local/lib/pkgconfig/libpsvr2.pc; do
		as_root rm -f "$f"
	done
fi

if [ "${ALL}" -eq 1 ]; then
	as_root rm -rf "${STATE_ROOT}"
	as_user rm -rf "${USER_STATE}"
fi

say "done"
