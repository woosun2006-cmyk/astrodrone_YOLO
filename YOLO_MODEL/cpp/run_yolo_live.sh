#!/bin/sh
# Builds (if needed) and runs the C++ port of ../yolo_live.py.
# No arguments - yolo_live.cpp finds the repo root itself and reads
# ../0812best.onnx/.engine, ../classes.txt, ../../setting/cam_sets.yaml.
set -eu
cd "$(dirname "$0")"

mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. >/dev/null
make -j"$(nproc)"

exec ./yolo_live
