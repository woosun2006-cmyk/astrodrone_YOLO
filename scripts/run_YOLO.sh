#!/bin/bash
# 욜로 '만' 돌린다. YOLO_MODEL/cpp/yolo_live 를 포그라운드로 실행해서
# 카메라 -> 추론 -> HTTP /target(기본 127.0.0.1:8002) 스트림을 올린다.
# 다른 프로그램(target_distance, full_mission.sh)이 이 /target 을 폴링하는
# 대상이라, 이 스크립트만 따로 켜서 인식 상태를 확인/디버깅할 때 쓴다.
set -e
cd "$(dirname "$0")"
REPO_ROOT="$(cd .. && pwd)"

exec "$REPO_ROOT/YOLO_MODEL/cpp/run_yolo_live.sh"
