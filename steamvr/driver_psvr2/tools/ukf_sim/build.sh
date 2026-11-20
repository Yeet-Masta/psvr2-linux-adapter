#!/bin/sh
# Build the offline ControllerUkf check.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
src="$here/../../src"
${CXX:-g++} -std=c++17 -O2 -Wall -Wextra -I"$src" -isystem "$here/../../third_party/openvr" \
	"$here/ukf_sim.cpp" "$src/controller_ukf.cpp" -o "$here/ukf_sim"
