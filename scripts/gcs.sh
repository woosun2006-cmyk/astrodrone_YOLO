#!/bin/bash
# gcs/sender의 telem_sender를 포그라운드로 실행한다: 픽스호크 MAVLink(모드/armed/
# heartbeat) + target_distance.cpp가 broadcast하는 타겟/고도를 JSON으로 묶어
# FEC 씌운 UDP로 setting/gcs.yaml의 dest_host:dest_port(노트북)로 쏜다.
# 영상(MJPEG)은 이 프로그램이 안 건드림 - 노트북의 gcs_bridge.py/dashboard.html이
# yolo_live.cpp의 /stream을 직접 받아서 그린다 (gcs/tools/gcs_bridge.py 참고).
set -e
cd "$(dirname "$0")"
REPO_ROOT="$(cd .. && pwd)"

BUILD_DIR="$REPO_ROOT/gcs/build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. >/dev/null
make -j"$(nproc)"

exec ./telem_sender
