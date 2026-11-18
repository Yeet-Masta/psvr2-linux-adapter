#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0
#
# Build driver_psvr2 and register it with SteamVR by symlinking the staged
# package into SteamVR's drivers/ directory. Re-run after rebuilding; the
# symlink means you don't have to reinstall for each code change.
#
# Usage: steamvr/install-steamvr-driver.sh [--uninstall]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DRIVER_SRC="${SCRIPT_DIR}/driver_psvr2"
BUILD_DIR="${DRIVER_SRC}/build"
PKG_DIR="${BUILD_DIR}/psvr2"

# Find the SteamVR drivers directory across common Steam install layouts.
find_steamvr_drivers() {
	local c
	for c in \
		"${HOME}/.steam/steam/steamapps/common/SteamVR/drivers" \
		"${HOME}/.local/share/Steam/steamapps/common/SteamVR/drivers" \
		"${HOME}/.steam/root/steamapps/common/SteamVR/drivers"; do
		[ -d "$c" ] && { echo "$c"; return 0; }
	done
	return 1
}

DRIVERS_DIR="$(find_steamvr_drivers)" || {
	echo "Could not find SteamVR's drivers/ directory. Is SteamVR installed?" >&2
	echo "Set it explicitly:  STEAMVR_DRIVERS=/path/to/SteamVR/drivers $0" >&2
	[ -n "${STEAMVR_DRIVERS:-}" ] && DRIVERS_DIR="${STEAMVR_DRIVERS}" || exit 1
}
DRIVERS_DIR="${STEAMVR_DRIVERS:-$DRIVERS_DIR}"
LINK="${DRIVERS_DIR}/psvr2"
LAYER_JSON="${XDG_DATA_HOME:-${HOME}/.local/share}/vulkan/implicit_layer.d/VkLayer_PSVR2_mode_order.json"

if [ "${1:-}" = "--uninstall" ]; then
	rm -f "${LINK}" "${LAYER_JSON}" "${LAYER_JSON/implicit_layer.d/explicit_layer.d}"
	echo "Removed ${LINK} and ${LAYER_JSON}"
	exit 0
fi

# libpsvr2.a must exist for the static link.
make -C "${SCRIPT_DIR}/../userspace/lib" libpsvr2.a >/dev/null

cmake -S "${DRIVER_SRC}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "${BUILD_DIR}" -j >/dev/null

[ -f "${PKG_DIR}/bin/linux64/driver_psvr2.so" ] || {
	echo "Build did not produce driver_psvr2.so" >&2; exit 1; }

ln -sfn "${PKG_DIR}" "${LINK}"
echo "Linked ${LINK} -> ${PKG_DIR}"

# Vulkan layer that fixes vrcompositor's 120 Hz frame timing (see
# steamvr/vk_mode_order/vk_mode_order.c). It loads only in SteamVR processes.
mkdir -p "$(dirname "${LAYER_JSON}")"
sed "s#@LIB@#${PKG_DIR}/bin/linux64/libVkLayer_PSVR2_mode_order.so#" \
	"${SCRIPT_DIR}/vk_mode_order/VkLayer_PSVR2_mode_order.json.in" > "${LAYER_JSON}"
echo "Installed ${LAYER_JSON}"
echo "Start SteamVR; check 'driver_psvr2' in the SteamVR web console for logs."
