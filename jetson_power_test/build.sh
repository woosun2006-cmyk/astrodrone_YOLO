#!/bin/sh
set -eu
cd "$(dirname "$0")"
/usr/local/cuda-10.2/bin/nvcc -O3 -std=c++11 -Xcompiler=-pthread jetson_stress.cu -o jetson_stress
chmod +x monitor.py
echo "Built: $(pwd)/jetson_stress"
