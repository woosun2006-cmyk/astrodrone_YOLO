#!/bin/sh
# Builds (if needed) and runs yolo_headless.cpp - the no-dashboard/no-stream
# twin of yolo_live.cpp meant for actual flights (see yolo_headless.cpp's
# top-of-file comment). Same /target endpoint control/target_distance.cpp
# polls; no arguments needed - it finds the repo root itself.
set -eu
cd "$(dirname "$0")"

mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. >/dev/null
make -j"$(nproc)"

exec ./yolo_headless
