#!/bin/bash
# Record Sony's driver (running via Ignition, see use-sony.sh) as ground
# truth, with SteamVR already running:
#   usb.pcap    every USB transfer to/from the headset (usbmon): LD blob
#               packets, IMU, SLAM and every control report Sony sends;
#   hid.txt     every report written to / read from the Sense controllers'
#               hidraw nodes (LED sync, haptics, input with IMU);
#   poses.txt   SteamVR's device poses (posepoll.c, CLOCK_MONOTONIC like
#               bpftrace nsecs): Sony's output.
# usage: capture.sh [seconds] [out_dir]   (default 120 s, ~/sony-capture-<time>)
set -e
secs=${1:-120}
out=${2:-$HOME/sony-capture-$(date +%Y%m%d-%H%M%S)}
mkdir -p "$out"
pgrep -x vrserver >/dev/null || { echo "start SteamVR (Sony driver) first" >&2; exit 1; }

bus=$(lsusb -d 054c:0cde | awk '{print $2 + 0; exit}')
[ -n "$bus" ] || { echo "headset not found" >&2; exit 1; }
sudo modprobe usbmon
lsusb -v -d 054c:0cde > "$out/lsusb.txt" 2>/dev/null || true
for h in /sys/class/hidraw/hidraw*; do
	echo "$(basename "$h") $(cat "$h/dev") $(grep -h HID_NAME "$h/device/uevent")"
done > "$out/hidraw.txt"

# -Z root: tcpdump otherwise drops to user tcpdump, which cannot write
# into $out.
# -G/-W: tcpdump stops itself after $secs. It must: AppArmor's tcpdump
# profile refuses signals from some confined callers, even from root.
# The headset streams ~30 MB/s (camera), so expect ~4 GB for 120 s.
sudo tcpdump -Z root -B 524288 -i "usbmon$bus" -s 0 -G "$secs" -W 1 -w "$out/usb.pcap" 2>"$out/tcpdump.log" &
tcp=$!
# Output reports (hidraw write path) and input reports, with kernel time.
sudo bpftrace -q -e '
interval:s:'"$secs"' { exit(); }
kprobe:hidraw_send_report {
	$f = (struct file *)arg0;
	printf("OUT %llu dev %d type %d len %d %rx\n", nsecs, $f->f_inode->i_rdev, arg3, arg2,
	       buf(uptr(arg1), arg2 > 96 ? 96 : arg2));
}
kprobe:hidraw_report_event {
	$h = (struct hid_device *)arg0;
	printf("IN %llu %s len %d %rx\n", nsecs, $h->name, arg2, buf(arg1, arg2 > 96 ? 96 : arg2));
}' > "$out/hid.txt" 2>"$out/bpftrace.log" &
bpf=$!
vr=$HOME/.steam/debian-installation/steamapps/common/SteamVR/bin/linux64
gcc -O2 -Wall -o "$out/posepoll" "$(dirname "$0")/posepoll.c" -ldl
(cd "$vr" && LD_LIBRARY_PATH=. "$out/posepoll" "$secs" > "$out/poses.txt" 2>"$out/posepoll.log") &
pos=$!
date +%s%N > "$out/start_ns.txt"
cat /proc/uptime > "$out/start_uptime.txt"

echo "recording $secs s into $out: move both controllers in view"
wait "$pos" "$bpf" 2>/dev/null || true
# tcpdump -G only rotates when a packet arrives after the deadline.
wait "$tcp" 2>/dev/null || true
cp "$HOME/.steam/debian-installation/logs/vrserver.txt" "$out/vrserver.txt"
date +%s%N > "$out/end_ns.txt"
ls -l "$out"
