#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0
#
# Hardware test hand-off for the 2026-11-25 ports (controller-ukf branch):
#   - controller UKF gyro clip per hardware rev,
#   - arm-model pseudo fixes (optical_arm_model),
#   - occluded-controller latch (optical_latch),
#   - SLAM delta pose (+0x2c) in the kernel sample and the INS mode-0
#     blender (headset_ins_delta, default off until this test says yes).
#
# Run from a desktop terminal as your normal user (uses sudo, which must not
# ask for a password), with the headset plugged in, both Sense controllers
# paired, Steam running and SteamVR CLOSED:
#
#   scripts/hw-handoff.sh                 full run (build, install, guided test)
#   scripts/hw-handoff.sh --offline       build + offline tests only
#   scripts/hw-handoff.sh --no-install    skip install.sh (module/driver already current)
#   scripts/hw-handoff.sh --apply-delta   at the end, turn headset_ins_delta on if the
#                                         SLAM check says +0x2c is a frame offset
#   scripts/hw-handoff.sh --restore       at the end, put steamvr.vrsettings back
#
# Everything lands in ~/psvr2-hwtest-<date>/ and in a .tar.gz next to it:
# send that file back.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${SCRIPT_DIR}/.." && pwd)"
STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="${HOME}/psvr2-hwtest-${STAMP}"
OFFLINE=0 INSTALL=1 APPLY_DELTA=0 RESTORE=0
for a in "$@"; do
	case "$a" in
	--offline) OFFLINE=1 ;;
	--no-install) INSTALL=0 ;;
	--apply-delta) APPLY_DELTA=1 ;;
	--restore) RESTORE=1 ;;
	-h | --help) sed -n '3,25p' "$0"; exit 0 ;;
	*) echo "unknown option: $a" >&2; exit 2 ;;
	esac
done

mkdir -p "${OUT}"
exec > >(tee -a "${OUT}/handoff.log") 2>&1

say() { echo; echo ">> $*"; }
note() { echo "   $*"; }
fail() { echo "!! $*"; echo "FAIL: $*" >>"${OUT}/summary.txt"; }
pass() { echo "   ok: $*"; echo "ok: $*" >>"${OUT}/summary.txt"; }
info() { echo "   $*"; echo "   $*" >>"${OUT}/summary.txt"; }
die() { fail "$*"; pack; exit 1; }

# Audio cue while the headset is on (speech if available, else a bell).
cue() {
	echo "   [cue] $*"
	if command -v spd-say >/dev/null 2>&1; then
		spd-say -w "$*" >/dev/null 2>&1 || true
	elif command -v espeak-ng >/dev/null 2>&1; then
		espeak-ng "$*" >/dev/null 2>&1 || true
	elif command -v paplay >/dev/null 2>&1 && [ -f /usr/share/sounds/freedesktop/stereo/bell.oga ]; then
		paplay /usr/share/sounds/freedesktop/stereo/bell.oga >/dev/null 2>&1 || true
	else
		printf '\a'
	fi
}
countdown() { # seconds label
	local s=$1
	while [ "$s" -gt 0 ]; do
		printf '\r   %s: %3d s ' "$2" "$s"
		sleep 1
		s=$((s - 1))
	done
	printf '\r   %s: done      \n' "$2"
}

pack() {
	say "packing results"
	tar -C "$(dirname "${OUT}")" -czf "${OUT}.tar.gz" "$(basename "${OUT}")" 2>/dev/null &&
		note "send back: ${OUT}.tar.gz" || note "results in ${OUT}"
}

steam_root() {
	local c
	for c in "${HOME}/.steam/debian-installation" "${HOME}/.local/share/Steam" "${HOME}/.steam/steam" \
		"${HOME}/.steam/root"; do
		[ -d "$c/config" ] && { readlink -f "$c"; return; }
	done
	echo ""
}
steamvr_running() { pgrep -x vrserver >/dev/null 2>&1 || pgrep -x vrcompositor >/dev/null 2>&1; }

: >"${OUT}/summary.txt"
echo "psvr2 hardware hand-off ${STAMP}" >>"${OUT}/summary.txt"

# ---- 0. environment --------------------------------------------------------
say "environment"
{
	echo "date: $(date -Is)"
	echo "kernel: $(uname -a)"
	echo "git: $(git -C "${REPO}" rev-parse --short HEAD 2>/dev/null) $(git -C "${REPO}" rev-parse --abbrev-ref HEAD 2>/dev/null)"
	git -C "${REPO}" status --short 2>/dev/null | head -20
	echo "--- lsusb (Sony)"; lsusb 2>/dev/null | grep -i '054c' || echo "(no Sony USB device)"
	echo "--- bluetooth"; command -v bluetoothctl >/dev/null && bluetoothctl devices Connected 2>/dev/null
} >"${OUT}/env.txt" 2>&1
cat "${OUT}/env.txt"
for t in gcc g++ make python3; do
	command -v "$t" >/dev/null 2>&1 || die "missing tool: $t"
done

# ---- 1. build + offline tests ----------------------------------------------
say "build and offline tests (log: build.log)"
{
	set -x
	make -C "${REPO}/userspace/lib" check
	sh "${REPO}/steamvr/driver_psvr2/tools/ukf_sim/build.sh" && "${REPO}/steamvr/driver_psvr2/tools/ukf_sim/ukf_sim"
	UKF_ARM=1 "${REPO}/steamvr/driver_psvr2/tools/ukf_sim/ukf_sim"
	sh "${REPO}/steamvr/driver_psvr2/tools/latch_sim/build.sh" && "${REPO}/steamvr/driver_psvr2/tools/latch_sim/latch_sim"
	sh "${REPO}/steamvr/driver_psvr2/tools/optical_replay/build.sh" "${OUT}/optical_replay_build" >/dev/null
	set +x
} >"${OUT}/build.log" 2>&1
grep -E '^(PASS|FAIL)$' "${OUT}/build.log" | sort | uniq -c | sed 's/^/   /'
n_pass=$(grep -c '^PASS$' "${OUT}/build.log")
n_fail=$(grep -c '^FAIL$' "${OUT}/build.log")
[ "${n_fail}" -eq 0 ] && [ "${n_pass}" -ge 5 ] && pass "offline tests (${n_pass} PASS)" ||
	fail "offline tests: ${n_pass} PASS, ${n_fail} FAIL (see build.log)"
grep -E 'dropout|arm-model|latch|delta 3 cm' "${OUT}/build.log" | sed 's/^/   /'
if [ "${OFFLINE}" -eq 1 ]; then
	pack
	exit 0
fi
sudo -n true 2>/dev/null || die "sudo asks for a password; this script needs passwordless sudo"
command -v cmake >/dev/null 2>&1 || die "missing tool: cmake (SteamVR driver build)"

# ---- 2. install ------------------------------------------------------------
if steamvr_running; then
	cue "Please close Steam VR"
	note "SteamVR is running: close it (it rewrites steamvr.vrsettings and holds the module)."
	for _ in $(seq 120); do steamvr_running || break; sleep 1; done
	steamvr_running && die "SteamVR still running"
fi
SR="$(steam_root)"
[ -n "${SR}" ] || die "Steam root not found"
VRS="${SR}/config/steamvr.vrsettings"
LOG="${SR}/logs/vrserver.txt"
note "Steam root ${SR}"
[ -f "${VRS}" ] && cp "${VRS}" "${OUT}/steamvr.vrsettings.before"

if [ "${INSTALL}" -eq 1 ]; then
	if command -v dkms >/dev/null 2>&1; then
		say "install: kernel (DKMS), udev, SteamVR driver (log: install.log)"
		parts="kernel udev steamvr"
	else
		say "install: no dkms, so in-tree module + udev + SteamVR driver (log: install.log)"
		parts="udev steamvr"
	fi
	# shellcheck disable=SC2086
	if ! (cd "${REPO}" && ./install.sh ${parts}) >"${OUT}/install.log" 2>&1; then
		tail -20 "${OUT}/install.log"
		die "install.sh failed"
	fi
	tail -8 "${OUT}/install.log" | sed 's/^/   /'
	if [ "${parts}" = "udev steamvr" ]; then
		make -C "${REPO}/kernel" >>"${OUT}/install.log" 2>&1 || die "kernel module build failed (headers installed?)"
		sudo "${REPO}/scripts/dev-load.sh" >>"${OUT}/install.log" 2>&1 || die "dev-load.sh failed"
	else
		# install.sh reloads only when the build changed; reload anyway.
		sudo modprobe -r psvr2 2>/dev/null || true
		sudo modprobe psvr2 || die "modprobe psvr2 failed"
	fi
	sleep 3
fi
sudo dmesg | grep -i psvr2 | tail -40 >"${OUT}/dmesg-load.txt"
lsmod | grep -q '^psvr2 ' && pass "psvr2 module loaded" || die "psvr2 module not loaded"
for d in /dev/psvr2-pose /dev/psvr2-ld /dev/psvr2-ctrl; do
	[ -e "$d" ] && pass "$d present" || fail "$d missing"
done

# The new pose sample carries the delta pose: check the running module's size.
cat >"${OUT}/possize.c" <<EOF
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <unistd.h>
#include "psvr2_uapi.h"
int main(void)
{
	char buf[4096];
	int fd = open("/dev/psvr2-pose", O_RDONLY | O_NONBLOCK);
	if (fd < 0) { perror("open"); return 2; }
	struct pollfd p = { fd, POLLIN, 0 };
	if (poll(&p, 1, 3000) <= 0) { printf("no sample in 3 s (headset idle?)\n"); return 3; }
	ssize_t n = read(fd, buf, sizeof(buf));
	printf("sample %zd bytes, expected %zu\n", n, sizeof(struct psvr2_pose_sample));
	return n == (ssize_t)sizeof(struct psvr2_pose_sample) ? 0 : 1;
}
EOF
gcc -I"${REPO}/kernel" -o "${OUT}/possize" "${OUT}/possize.c" 2>>"${OUT}/build.log"

# ---- 3. settings -----------------------------------------------------------
say "driver settings for the test"
(cd "${REPO}" && ./install.sh --set optical_tracking=true --set optical_ukf=true --set optical_arm_model=true \
	--set optical_latch=true --set optical_trace=true --set headset_ins=true --set headset_ins_delta=false) \
	>>"${OUT}/install.log" 2>&1 || fail "install.sh --set failed"
python3 - "${VRS}" <<'PY' | sed 's/^/   /'
import json, sys
s = json.load(open(sys.argv[1])).get("driver_psvr2", {})
for k in ("optical_tracking", "optical_ukf", "optical_arm_model", "optical_latch", "optical_trace",
          "headset_ins", "headset_ins_delta"):
    print(f"{k} = {s.get(k)}")
PY

# ---- 4. SteamVR ------------------------------------------------------------
say "start SteamVR"
LOG0=$( [ -f "${LOG}" ] && wc -l <"${LOG}" || echo 0 )
if command -v steam >/dev/null 2>&1; then
	(steam steam://rungameid/250820 >/dev/null 2>&1 &)
	note "asked Steam to start SteamVR"
else
	note "start SteamVR from Steam now"
fi
for _ in $(seq 180); do steamvr_running && break; sleep 1; done
steamvr_running || die "SteamVR did not start within 3 minutes"
# A restarted log starts over: count from 0 then.
sleep 5
[ "$(wc -l <"${LOG}")" -lt "${LOG0}" ] && LOG0=0
for _ in $(seq 60); do tail -n +"$((LOG0 + 1))" "${LOG}" 2>/dev/null | grep -q 'psvr2:' && break; sleep 1; done
pass "SteamVR running, driver logging"
if "${OUT}/possize" >"${OUT}/possize.txt" 2>&1; then
	pass "pose sample size matches the new uapi ($(cat "${OUT}/possize.txt"))"
else
	fail "pose sample size: $(cat "${OUT}/possize.txt") (old module still loaded? replug the headset)"
fi

mark() { # phase name: remember the vrserver.txt line number
	echo "$1 $(wc -l <"${LOG}") $(date +%s.%N)" >>"${OUT}/phases.txt"
}
: >"${OUT}/phases.txt"

# SLAM record capture from debugfs (root), every new record, in the background.
sudo mount -t debugfs none /sys/kernel/debug 2>/dev/null || true
RAW=$(sudo sh -c 'ls /sys/kernel/debug/psvr2*/raw_slam /sys/kernel/debug/psvr2/*/raw_slam 2>/dev/null | head -1')
cat >"${OUT}/slamcap.py" <<'PY'
import struct, sys, time
path, out, secs = sys.argv[1], sys.argv[2], float(sys.argv[3])
f = open(out, "w")
f.write("host_s,dev_us,state,substate,flag,reset,map_id,"
        "p0,p1,p2,q0,q1,q2,q3,d0,d1,d2,e0,e1,e2,e3\n")
last = None
end = time.time() + secs
while time.time() < end:
    try:
        b = open(path, "rb").read()
    except OSError:
        time.sleep(0.05)
        continue
    if len(b) < 0x1e8:
        time.sleep(0.01)
        continue
    dev = struct.unpack_from("<I", b, 0x08)[0]
    if dev != last:
        last = dev
        p = struct.unpack_from("<3f", b, 0x10); q = struct.unpack_from("<4f", b, 0x1c)
        d = struct.unpack_from("<3f", b, 0x2c); e = struct.unpack_from("<4f", b, 0x38)
        mid = struct.unpack_from("<I", b, 0x1e4)[0]
        f.write("%.4f,%d,%d,%d,%d,%d,%d," % (time.time(), dev, b[0x0c], b[0x0d], b[0x0e], b[0x1e0], mid))
        f.write(",".join("%.6g" % v for v in p + q + d + e) + "\n")
    time.sleep(0.004)
PY
CAP_PID=""
if [ -n "${RAW}" ]; then
	sudo python3 "${OUT}/slamcap.py" "${RAW}" "${OUT}/slam.csv" 200 &
	CAP_PID=$!
	note "capturing SLAM records from ${RAW}"
else
	fail "debugfs raw_slam not found: no SLAM delta check"
fi

# ---- 5. guided phases (audio cues; you wear the headset) -------------------
cat <<'EOF'

  Guided test, about 2 minutes. Put the headset on and hold both controllers.
  Each step is announced by voice (or a bell). Steps:
    1. Stand still, both controllers in front of you, 15 s.
    2. Occlusion: hold the LEFT controller about 10 cm straight behind the
       RIGHT one, both at chest height in front of the headset, 10 s; then
       move them apart. Then the same with the RIGHT behind the LEFT.
    3. Out of view: put both controllers behind your back, 10 s, then bring
       them back in front and wave them slowly, 10 s.
    4. Relocalisation: cover all headset cameras with your hands 5 s, then
       uncover and look around the room slowly, 15 s.
    5. Head turns: keep the controllers still in front, turn your head left
       and right slowly, 15 s.

EOF
read -r -p "   Press Enter, then put the headset on (10 s)... " _
sleep 10
mark start
cue "Step 1. Stand still, controllers in front."; mark still; countdown 15 still
cue "Step 2. Left controller behind the right one."; mark occl_left; countdown 10 occlusion-left
cue "Move them apart."; sleep 4
cue "Now the right controller behind the left one."; mark occl_right; countdown 10 occlusion-right
cue "Move them apart."; sleep 4
cue "Step 3. Both controllers behind your back."; mark out_of_view; countdown 10 out-of-view
cue "Bring them back in front and wave slowly."; mark back_in_view; countdown 10 back-in-view
cue "Step 4. Cover the headset cameras."; mark covered; countdown 5 covered
cue "Uncover and look around slowly."; mark uncovered; countdown 15 uncovered
cue "Step 5. Controllers still, turn your head left and right."; mark head_turns; countdown 15 head-turns
mark end
cue "Done. You can take the headset off."
if [ -n "${CAP_PID}" ]; then
	sudo pkill -f "${OUT}/slamcap.py" 2>/dev/null || true
	wait "${CAP_PID}" 2>/dev/null || true
fi

# ---- 6. collect ------------------------------------------------------------
say "collecting logs"
cp "${LOG}" "${OUT}/vrserver.txt" 2>/dev/null
tail -n +"$((LOG0 + 1))" "${LOG}" | grep 'psvr2' >"${OUT}/psvr2-lines.txt" 2>/dev/null
sudo dmesg | tail -300 >"${OUT}/dmesg.txt"
for f in "${HOME}/psvr2-ld-trace.txt"; do [ -f "$f" ] && cp "$f" "${OUT}/"; done

# Per-phase grep of the driver log.
python3 - "${OUT}/phases.txt" "${OUT}/vrserver.txt" <<'PY' | tee -a "${OUT}/summary.txt"
import re, sys
ph = [l.split() for l in open(sys.argv[1]) if l.strip()]
lines = open(sys.argv[2], errors="replace").read().splitlines()
keys = {
    "latch": r"optical latch: .* carried",
    "arm": r"arm model fixes",
    "restart": r"optical filter \(re\)started",
    "fixes": r"optical pose: .* \d+ fixes",
    "rev": r"hardware rev",
}
print("per-phase driver log counts:")
for i in range(len(ph) - 1):
    a, b = int(ph[i][1]), int(ph[i + 1][1])
    seg = [l for l in lines[a:b] if "psvr2" in l]
    c = {k: sum(1 for l in seg if re.search(v, l)) for k, v in keys.items()}
    print("  %-13s %s" % (ph[i][0], " ".join("%s=%d" % kv for kv in c.items())))
PY
grep -m4 -E 'hardware rev' "${OUT}/psvr2-lines.txt" | sed 's/^/   /' | tee -a "${OUT}/summary.txt"
grep -q 'optical latch: .* carried' "${OUT}/psvr2-lines.txt" && pass "occlusion latch engaged" ||
	fail "occlusion latch never engaged (were both controllers optically tracked before step 2?)"
grep -q 'arm model fixes' "${OUT}/psvr2-lines.txt" && pass "arm-model pseudo fixes applied" ||
	fail "no arm-model fixes logged (step 3)"
grep -q 'optical pose: .* tracking' "${OUT}/psvr2-lines.txt" && pass "optical pose tracked at some point" ||
	fail "optical pose never tracking (LED sync / cameras: see psvr2-lines.txt)"

# ---- 7. SLAM delta verdict -------------------------------------------------
DELTA_OK=0
if [ -s "${OUT}/slam.csv" ]; then
	say "SLAM second pose (+0x2c) analysis"
	python3 - "${OUT}/slam.csv" "${OUT}/phases.txt" <<'PY' | tee "${OUT}/slam_summary.txt" | tee -a "${OUT}/summary.txt"
import csv, math, sys
rows = list(csv.DictReader(open(sys.argv[1])))
if len(rows) < 50:
    print("too few SLAM records: %d" % len(rows)); print("VERDICT unclear"); sys.exit()
f = lambda r, k: float(r[k])
def qang(q):  # angle from identity, either w-first or w-last
    w = max(abs(q[0]), abs(q[3]))
    return math.degrees(2 * math.acos(min(1.0, w)))
dn = [math.sqrt(sum(f(r, "d%d" % i) ** 2 for i in range(3))) for r in rows]
da = [qang([f(r, "e%d" % i) for i in range(4)]) for r in rows]
dup = [math.sqrt(sum((f(r, "d%d" % i) - f(r, "p%d" % i)) ** 2 for i in range(3))) for r in rows]
pn = [math.sqrt(sum(f(r, "p%d" % i) ** 2 for i in range(3))) for r in rows]
steps = sum(1 for i in range(1, len(rows)) if
            math.sqrt(sum((f(rows[i], "d%d" % k) - f(rows[i - 1], "d%d" % k)) ** 2 for k in range(3))) > 0.005)
maps = len(set(r["map_id"] for r in rows)); resets = len(set(r["reset"] for r in rows))
med = lambda v: sorted(v)[len(v) // 2]
print("records %d, map ids %d, reset counters %d" % (len(rows), maps, resets))
print("main pose |p|: median %.3f m" % med(pn))
print("second pose |p|: median %.4f m, max %.4f m; angle median %.3f deg, max %.3f deg" %
      (med(dn), max(dn), med(da), max(da)))
print("second vs main position: median %.4f m" % med(dup))
print("second-pose steps > 5 mm between records: %d" % steps)
quiet = med(dn) < 0.05 and med(da) < 2.0
moving_main = med(pn) > 0.1
if med(dup) < 0.01 and moving_main:
    print("VERDICT duplicate: +0x2c follows the main pose; leave headset_ins_delta off")
elif quiet and steps < len(rows) * 0.05:
    print("VERDICT offset: +0x2c is near identity and changes in steps; headset_ins_delta can be tried")
else:
    print("VERDICT unclear: send slam.csv back")
PY
	grep -q '^VERDICT offset' "${OUT}/slam_summary.txt" && DELTA_OK=1
fi

# ---- 8. optional follow-ups ------------------------------------------------
if [ "${APPLY_DELTA}" -eq 1 ] && [ "${DELTA_OK}" -eq 1 ]; then
	say "turning headset_ins_delta on (takes effect on the next SteamVR start)"
	cue "Please close Steam VR"
	for _ in $(seq 120); do steamvr_running || break; sleep 1; done
	if steamvr_running; then
		fail "SteamVR still running: run ./install.sh --set headset_ins_delta=true later"
	else
		(cd "${REPO}" && ./install.sh --set headset_ins_delta=true) >>"${OUT}/install.log" 2>&1 &&
			info "headset_ins_delta=true written; look for 'mode-0 blender on the SLAM delta pose' in vrserver.txt"
	fi
fi
if [ "${RESTORE}" -eq 1 ] && [ -f "${OUT}/steamvr.vrsettings.before" ]; then
	steamvr_running && note "close SteamVR to restore steamvr.vrsettings" &&
		for _ in $(seq 120); do steamvr_running || break; sleep 1; done
	steamvr_running || { cp "${OUT}/steamvr.vrsettings.before" "${VRS}" && info "steamvr.vrsettings restored"; }
fi

say "summary"
cat "${OUT}/summary.txt"
pack
