#!/bin/sh
# Build optical_replay: the driver's own optical_pose.cpp against stub
# controller/HMD/settings classes. The sources are copied next to the stubs
# because a quoted #include searches the including file's folder first.
set -e
here=$(cd "$(dirname "$0")" && pwd)
drv=$here/../..
lib=$drv/../../userspace/lib
out=${1:-$here/build}
mkdir -p "$out/src"
cp "$drv/src/optical_pose.cpp" "$drv/src/optical_pose.h" "$drv/src/vr_math.h" "$drv/src/sie_c.h" \
   "$here/stubs/"*.h "$out/src/"
for c in psvr2_led_solver psvr2_camera_model; do
	gcc -O2 -g -Wall -I"$lib" -c "$lib/$c.c" -o "$out/$c.o"
done
g++ -std=c++17 -O2 -g -Wall -I"$out/src" -I"$drv/third_party/openvr" -I"$lib" \
    "$out/src/optical_pose.cpp" "$here/replay.cpp" \
    "$out/psvr2_led_solver.o" "$out/psvr2_camera_model.o" -lm -o "$out/optical_replay"
echo "$out/optical_replay"
