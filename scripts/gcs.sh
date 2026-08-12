#!/bin/bash
# gcs 테스트에 필요한 4개를 한번에 띄운다 (control.cpp는 안 띄움 - arm/이륙
# 없이 GCS 수신만 확인하고 싶을 때 이 스크립트 하나로 끝나도록):
#   1. mavlink_proxy (백그라운드) - 픽스호크 시리얼을 혼자 열고 UDP
#      14550/14551/14552(mavlink_control/mavlink_sensor/mavlink_gcs)로
#      fan-out. setting/port.yaml이 예전부터 "외부에서 mavproxy/mavlink-router
#      가 떠서 해준다"고 전제하던 그 릴레이 - 이 저장소엔 없었고, 이 젯슨의
#      mavproxy.py는 실행하자마자 core dump 나서 control/mavlink_proxy.cpp로
#      직접 만든 것. target_distance/telem_sender는 이제 이 프록시를 거쳐서
#      MAVLink에 붙는다 (직접 시리얼을 열지 않음 - 여러 프로세스가 시리얼
#      포트 하나를 두고 경합하던 문제 해결).
#   2. yolo_headless (백그라운드) - 카메라+추론+/target
#   3. target_distance (백그라운드) - /target+ALTITUDE -> 거리계산 -> 로컬
#      UDP(target_track.udp_port) broadcast
#   4. telem_sender (포그라운드) - 픽스호크 MAVLink(모드/armed/heartbeat) +
#      target_distance가 broadcast하는 타겟/거리/고도를 JSON으로 묶어 FEC
#      씌운 UDP로 setting/gcs.yaml의 dest_host:dest_port(노트북)로 쏜다.
#
# telem_sender는 target_distance.cpp의 UDP 브로드캐스트를 받아야만
# target_detected/x_px/y_px/distance_m을 채운다 (gcs/sender/telem_sender_main.cpp
# 상단 주석 참고) - target_distance가 없으면 그 필드들은 계속 null만 나가서,
# yolo_headless + target_distance까지 같이 띄워야 한다.
#
# 영상(MJPEG)은 이 프로그램이 안 건드림 - yolo_headless는 애초에 스트리밍이
# 없다 (YOLO_MODEL/cpp/yolo_headless.cpp 참고, 실비행에 쓰는 것과 동일).
# 실제 영상까지 보고 싶으면 이 스크립트 대신 YOLO_MODEL/cpp/run_yolo_live.sh를
# 따로 띄워서 브라우저(http://<jetson-ip>:8002/)로 직접 볼 것.
set -eu
cd "$(dirname "$0")"
REPO_ROOT="$(cd .. && pwd)"

PROXY_LOG=/tmp/mavlink_proxy.log
YOLO_LOG=/tmp/yolo_headless.log
TARGET_DIST_LOG=/tmp/target_distance.log

CONTROL_BUILD_DIR="$REPO_ROOT/control/build"
mkdir -p "$CONTROL_BUILD_DIR"
(cd "$CONTROL_BUILD_DIR" && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. >/dev/null &&
 make -j"$(nproc)" mavlink_proxy target_distance)

echo "starting mavlink_proxy (serial<->UDP relay) -> $PROXY_LOG"
(cd "$CONTROL_BUILD_DIR" && setsid nohup ./mavlink_proxy >"$PROXY_LOG" 2>&1 </dev/null &)
sleep 2

echo "starting yolo_headless (target detection) -> $YOLO_LOG"
(cd "$REPO_ROOT/YOLO_MODEL/cpp" && setsid nohup ./run_yolo_headless.sh >"$YOLO_LOG" 2>&1 </dev/null &)
sleep 1

echo "starting target_distance (range calc) -> $TARGET_DIST_LOG"
(cd "$CONTROL_BUILD_DIR" && setsid nohup ./target_distance >"$TARGET_DIST_LOG" 2>&1 </dev/null &)
sleep 1

cleanup() {
    echo
    echo "[gcs] stopping mavlink_proxy, yolo_headless, target_distance..."
    pkill -f './mavlink_proxy' 2>/dev/null || true
    pkill -f './yolo_headless' 2>/dev/null || true
    pkill -f './target_distance' 2>/dev/null || true
}
trap cleanup EXIT

BUILD_DIR="$REPO_ROOT/gcs/build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. >/dev/null
make -j"$(nproc)"

echo "starting telem_sender (foreground, Ctrl+C stops all four)"
./telem_sender
