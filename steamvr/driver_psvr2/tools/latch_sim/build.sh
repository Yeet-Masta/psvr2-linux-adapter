#!/bin/sh
# Build the offline OcclusionLatch check.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
src="$here/../../src"
${CXX:-g++} -std=c++17 -O2 -Wall -Wextra -I"$src" -isystem "$here/../../third_party/openvr" \
	"$here/latch_sim.cpp" "$src/occlusion_latch.cpp" -o "$here/latch_sim"
