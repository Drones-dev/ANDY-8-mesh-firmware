#!/usr/bin/env sh
set -eu
# Download only generated MAVLink C headers; PX4 itself remains a separate
# checkout so this repository does not silently modify a teammate's PX4 tree.
mkdir -p simulation/third_party
if [ ! -d simulation/third_party/mavlink/.git ]; then
  git clone --depth 1 https://github.com/mavlink/c_library_v2.git simulation/third_party/mavlink
fi
cmake -S . -B build -DMAVLINK_INCLUDE_DIR="$PWD/simulation/third_party/mavlink"
cmake --build build --parallel
ctest --test-dir build --output-on-failure
