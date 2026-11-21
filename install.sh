#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0
#
# Install (or update) everything the PS VR2 needs on Linux:
#   kernel   psvr2 module via DKMS (rebuilt on kernel upgrades)
#   udev     kernel/99-psvr2.rules (device access, IMU buffer, Sense hidraw)
#   group    add you to the "input" group the rules grant access to
#   steamvr  build driver_psvr2 and register it with SteamVR (symlink)
#   settings steamvr.vrsettings: "displayPortTrainingMode": 1 (docs/steamvr.md)
#   lib      libpsvr2 header/libs/pkg-config in /usr/local (not needed by the driver)
#
# Usage:
#   ./install.sh                 everything except lib (asks for sudo when needed)
#   ./install.sh --update        after a git pull / rebuild: only redo what changed,
#                                reload the module if its sources changed
#   ./install.sh kernel steamvr  only the named parts
#   ./install.sh --set headset_ins=false --set display_frequency=120
#                                write driver_psvr2 overrides into steamvr.vrsettings
#   ./install.sh --no-reload     never unload/reload the running module
#
# Run as your normal user (root steps go through sudo) or with sudo (user steps
# then run as $SUDO_USER). SteamVR should be closed: it rewrites
# steamvr.vrsettings on exit, and the module cannot reload while it runs.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME=psvr2
VERSION="$(sed -n 's/^PACKAGE_VERSION="\(.*\)"/\1/p' "${SCRIPT_DIR}/kernel/dkms.conf")"
SRC="/usr/src/${NAME}-${VERSION}"
RULES_SRC="${SCRIPT_DIR}/kernel/99-psvr2.rules"
RULES_DST=/etc/udev/rules.d/99-psvr2.rules
STATE_ROOT=/var/lib/psvr2

die() { echo "error: $*" >&2; exit 1; }
say() { echo ">> $*"; }
note() { echo "   $*"; }

# ---- who runs what ---------------------------------------------------------
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

# ---- arguments -------------------------------------------------------------
UPDATE=0
RELOAD=1
PARTS=()
SETS=()
while [ $# -gt 0 ]; do
	case "$1" in
	-u | --update) UPDATE=1 ;;
	--no-reload) RELOAD=0 ;;
	--set) [ $# -ge 2 ] || die "--set needs key=value"; SETS+=("$2"); shift ;;
	--set=*) SETS+=("${1#--set=}") ;;
	kernel | udev | group | steamvr | settings | lib) PARTS+=("$1") ;;
	-h | --help) sed -n '3,25p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
	*) die "unknown argument: $1 (see --help)" ;;
	esac
	shift
done
if [ ${#PARTS[@]} -eq 0 ]; then
	PARTS=(kernel udev group steamvr settings)
	# --set alone only writes the overrides.
	[ ${#SETS[@]} -gt 0 ] && [ "${UPDATE}" -eq 0 ] && PARTS=()
fi
want() {
	local p
	[ ${#PARTS[@]} -gt 0 ] || return 1
	for p in "${PARTS[@]}"; do [ "$p" = "$1" ] && return 0; done
	return 1
}

steamvr_running() { pgrep -x vrserver >/dev/null 2>&1 || pgrep -x vrcompositor >/dev/null 2>&1; }

# Content hash of a list of files (order-stable).
hash_files() { cat "$@" 2>/dev/null | sha256sum | cut -d' ' -f1; }
stamp_get() { cat "${STATE_ROOT}/$1" 2>/dev/null || true; }
stamp_set() { as_root mkdir -p "${STATE_ROOT}" && echo "$2" | as_root tee "${STATE_ROOT}/$1" >/dev/null; }
# Per-user record of the steamvr.vrsettings keys this script added, so
# uninstall.sh removes only those.
USER_STATE="${USER_HOME}/.local/state/psvr2"

# ---- kernel module (DKMS) --------------------------------------------------
KERNEL_FILES=()
for f in "${SCRIPT_DIR}"/kernel/*.c "${SCRIPT_DIR}"/kernel/*.h; do
	case "$f" in *.mod.c) continue ;; esac # kbuild output of an in-tree build
	KERNEL_FILES+=("$f")
done
KERNEL_FILES+=("${SCRIPT_DIR}/kernel/Makefile" "${SCRIPT_DIR}/kernel/dkms.conf")
MODULE_CHANGED=0

install_kernel() {
	command -v dkms >/dev/null 2>&1 || die "dkms not found: install the 'dkms' package first"
	[ -d "/lib/modules/$(uname -r)/build" ] ||
		die "kernel headers for $(uname -r) not found: install your distro's linux-headers package"
	local h
	h="$(hash_files "${KERNEL_FILES[@]}")-$(uname -r)"
	if [ "${UPDATE}" -eq 1 ] && [ "$(stamp_get kernel.sha)" = "$h" ] &&
		dkms status -m "${NAME}" -v "${VERSION}" 2>/dev/null | grep -q installed; then
		say "kernel module: unchanged"
		return
	fi
	say "kernel module: DKMS ${NAME}/${VERSION} for $(uname -r)"
	if dkms status -m "${NAME}" -v "${VERSION}" 2>/dev/null | grep -q "${NAME}"; then
		as_root dkms remove -m "${NAME}" -v "${VERSION}" --all >/dev/null || true
	fi
	# Sources only: stale .o/.ko from an in-tree build must not reach DKMS.
	as_root rm -rf "${SRC}"
	as_root mkdir -p "${SRC}"
	as_root cp "${KERNEL_FILES[@]}" "${RULES_SRC}" "${SRC}/"
	as_root dkms add -m "${NAME}" -v "${VERSION}" >/dev/null
	if ! as_root dkms build -m "${NAME}" -v "${VERSION}" >/dev/null; then
		echo "error: DKMS build failed; log:" >&2
		as_root sh -c "tail -30 /var/lib/dkms/${NAME}/${VERSION}/build/make.log" >&2 || true
		exit 1
	fi
	as_root dkms install -m "${NAME}" -v "${VERSION}" >/dev/null
	stamp_set kernel.sha "$h"
	MODULE_CHANGED=1
	note "installed"
}

reload_module() {
	if [ "${RELOAD}" -eq 0 ]; then
		note "module not reloaded (--no-reload)"
		return
	fi
	if steamvr_running; then
		note "SteamVR is running: close it, then: sudo modprobe -r ${NAME} && sudo modprobe ${NAME}"
		return
	fi
	if lsmod | grep -q "^${NAME} "; then
		say "reloading ${NAME}"
		as_root modprobe -r "${NAME}" || { note "unload failed (in use?); replug the headset"; return; }
	fi
	# Loads now if the headset is plugged in; otherwise it auto-loads on plug.
	as_root modprobe "${NAME}" || note "modprobe ${NAME} failed: see dmesg"
}

# ---- udev ------------------------------------------------------------------
RULES_CHANGED=0
install_udev() {
	if [ -f "${RULES_DST}" ] && cmp -s "${RULES_SRC}" "${RULES_DST}"; then
		say "udev rules: unchanged"
		return
	fi
	say "udev rules: ${RULES_DST}"
	as_root install -m 0644 "${RULES_SRC}" "${RULES_DST}"
	as_root udevadm control --reload-rules || true
	RULES_CHANGED=1
}

retrigger_udev() {
	# Apply to nodes that already exist (a module reload re-creates them anyway).
	local s
	for s in iio input misc video4linux hidraw; do
		as_root udevadm trigger --action=add --subsystem-match="$s" 2>/dev/null || true
	done
	as_root udevadm trigger --action=bind --subsystem-match=usb --attr-match=idVendor=054c 2>/dev/null || true
}

# ---- input group -----------------------------------------------------------
install_group() {
	[ "${TARGET_USER}" = root ] && return
	if id -nG "${TARGET_USER}" | tr ' ' '\n' | grep -qx input; then
		say "group: ${TARGET_USER} already in 'input'"
		return
	fi
	say "group: adding ${TARGET_USER} to 'input' (log out and back in afterwards)"
	as_root usermod -aG input "${TARGET_USER}"
}

# ---- SteamVR driver --------------------------------------------------------
install_steamvr() {
	say "SteamVR driver: build + register"
	command -v cmake >/dev/null 2>&1 || die "cmake not found: install cmake (and a C++ compiler)"
	# The script builds incrementally and only re-links the symlink, so this
	# is also the fast path after code changes.
	as_user env STEAMVR_DRIVERS="${STEAMVR_DRIVERS:-}" HOME="${USER_HOME}" \
		"${SCRIPT_DIR}/steamvr/install-steamvr-driver.sh" | sed 's/^/   /'
	steamvr_running && note "restart SteamVR to load the new driver" || true
}

install_lib() {
	say "libpsvr2: /usr/local"
	as_user make -s -C "${SCRIPT_DIR}/userspace/lib"
	as_root make -s -C "${SCRIPT_DIR}/userspace/lib" install
}

# ---- steamvr.vrsettings ----------------------------------------------------
vrsettings_path() {
	local c
	for c in "${USER_HOME}/.local/share/Steam/config/steamvr.vrsettings" \
		"${USER_HOME}/.steam/steam/config/steamvr.vrsettings" \
		"${USER_HOME}/.steam/root/config/steamvr.vrsettings"; do
		[ -f "$c" ] && { echo "$c"; return 0; }
	done
	# Not created until SteamVR's first run: use the usual location.
	echo "${USER_HOME}/.local/share/Steam/config/steamvr.vrsettings"
}

# edit_vrsettings <section> <key=json-or-string>... ; "-key" removes a key.
edit_vrsettings() {
	command -v python3 >/dev/null 2>&1 || { note "python3 not found: edit steamvr.vrsettings by hand"; return 1; }
	local f
	f="$(vrsettings_path)"
	as_user mkdir -p "${USER_STATE}"
	as_user python3 - "$f" "${USER_STATE}/vrsettings-added" "$@" <<'PY'
import json, os, shutil, sys
path, added_log, section, items = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4:]
data = {}
if os.path.exists(path):
    with open(path, encoding="utf-8") as fh:
        text = fh.read().strip()
    data = json.loads(text) if text else {}
    bak = path + ".psvr2.bak"
    if not os.path.exists(bak):
        shutil.copy2(path, bak)
else:
    os.makedirs(os.path.dirname(path), exist_ok=True)
sec = data.setdefault(section, {})
changed = []
for it in items:
    if it.startswith("-"):
        if sec.pop(it[1:], None) is not None:
            changed.append("removed %s.%s" % (section, it[1:]))
        continue
    key, _, raw = it.partition("=")
    try:
        val = json.loads(raw)
    except ValueError:
        val = raw
    if key not in sec:
        with open(added_log, "a", encoding="utf-8") as fh:
            fh.write("%s.%s\n" % (section, key))
    if sec.get(key) != val:
        sec[key] = val
        changed.append("%s.%s = %s" % (section, key, json.dumps(val)))
if not sec:
    data.pop(section)
if changed:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=3, sort_keys=True)
        fh.write("\n")
    os.replace(tmp, path)
print("   " + path + (": " + "; ".join(changed) if changed else ": unchanged"))
PY
}

install_settings() {
	say "SteamVR settings"
	if steamvr_running; then
		note "SteamVR is running and would overwrite the file on exit: skipped (close it and re-run)"
		return
	fi
	# Without it the leased connector gets an invalid atomic modeset on Linux.
	edit_vrsettings steamvr "displayPortTrainingMode=1" || true
}

apply_sets() {
	[ ${#SETS[@]} -gt 0 ] || return 0
	say "driver_psvr2 overrides"
	if steamvr_running; then
		note "SteamVR is running and would overwrite the file on exit: skipped"
		return
	fi
	local s
	for s in "${SETS[@]}"; do
		case "$s" in *=* | -*) ;; *) die "--set wants key=value (or -key to remove), got '$s'" ;; esac
	done
	edit_vrsettings driver_psvr2 "${SETS[@]}" || true
}

# ---- run -------------------------------------------------------------------
[ "${UPDATE}" -eq 1 ] && say "update mode: only changed parts"
want kernel && install_kernel
want udev && install_udev
want group && install_group
if [ "${MODULE_CHANGED}" -eq 1 ]; then
	reload_module
elif [ "${RULES_CHANGED}" -eq 1 ]; then
	retrigger_udev
fi
want lib && install_lib
want steamvr && install_steamvr
want settings && install_settings
apply_sets

say "done"
if [ "${UPDATE}" -eq 0 ] && want kernel; then
	note "Later: git pull && ./install.sh --update"
	note "Logs: SteamVR web console 'driver_psvr2', dmesg | grep psvr2"
	note "Multi-GPU: launch Steam with VK_DRIVER_FILES=<headset GPU ICD> (docs/steamvr.md)"
fi
