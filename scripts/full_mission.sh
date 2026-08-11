#!/bin/bash
# 미션 전체를 하나로 묶는 진입점: 켜놓으면
#   1. health_check.sh 로 비행 가능한 상태인지 게이트 확인
#   2. gcs.sh (telem_sender) 를 백그라운드로 자동 시작 - 안전검사 통과하면
#      바로 MAVLink 텔레메트리 + target_distance의 타겟/거리를 JSON+FEC로
#      노트북(setting/gcs.yaml dest_host:dest_port)에 쏘기 시작한다. 영상은
#      안 보냄 (아래 3번 참고).
#   3. yolo_headless 로 타겟 탐지 시작 (백그라운드) - 이후로는 이것만 돌아간다.
#      yolo_live.cpp(대시보드+/stream)는 사람이 브라우저로 수동 확인/디버깅
#      할 때 쓰는 테스트용이라 실비행에는 안 씀 - yolo_headless.cpp가 같은
#      /target을 내지만 영상 인코딩/스트리밍 없이 모델만 돌린다. 영상은
#      나중에 gcs가 완성되면 그쪽에서 받아본다 (gcs/PLAN.md 참고).
#   4. 욜로 /target 이 confirmed=true 를 낼 때까지 가볍게 폴링만 하며 대기.
#      target_distance/control은 아직 띄우지 않는다 - 탐지된 것도 없는데
#      거리 계산/픽스호크 연결까지 미리 돌릴 필요가 없기 때문 (젯슨 리소스
#      절약). "젯슨이 제어하는 구조"는 욜로가 대상을 확정했을 때만 켜진다.
#   5. target_distance.cpp 로 탐지 결과 + 고도를 거리로 변환 (백그라운드)
#   6. control --auto-intercept (포그라운드): 픽스호크가 (이미 다른 경로로
#      arm되어) AUTO 모드로 미션을 날고 있는 걸 지켜보다가, 욜로가 타겟을
#      lock_confirm_sec 이상 안정적으로 잡으면 그때 GUIDED로 가로채서 접근
#      -> 타겟을 놓치거나 intercept_approach_duration_sec이 지나면 다시
#      AUTO로 돌려주고 재탐지 대기 (setting/MAVLink.yaml target_track 참고).
# health_check가 실패하면 아예 시작하지 않는다. Ctrl+C는 대기 단계(4번)든
# 비행 단계(6번)든 상관없이 telem_sender/yolo_headless/target_distance까지
# 정리한다.
#
# 이 스크립트는 AUTO 진입/arm 자체는 하지 않는다 - 그건 미션 업로드와 함께
# 조종기/GCS로 조작하는 부분. control --auto-intercept는 그 상태를 그냥
# 지켜보다가 필요할 때만 끼어든다.
#
# 한 번 target_distance/control이 뜬 뒤로는, 비행 중 타겟을 잠깐 놓치는 것쯤은
# control --auto-intercept 자체가 처리한다 (AUTO 복귀 후 재탐지) - 여기서
# 다시 죽였다 살렸다 하지 않는다. 이 게이트는 "시작 시점"에만 적용된다.
#
# 알려진 위험 - telem_sender와 시리얼 포트 경합: telem_sender는
# setting/MAVLink.yaml의 real.serial(/dev/ttyACM0)을 직접 여는 반면,
# control.cpp/target_distance.cpp는 real.proxy_udp(mavlink-router 등 외부
# 프록시가 그 시리얼을 UDP 14550/14551로 fan-out 해준다는 전제, 자세한 내용은
# setting/port.yaml)로 붙는다. 그 외부 프록시가 실제로 떠 있지 않다면
# 두 그룹이 같은 시리얼 장치를 두고 경합하거나(최악의 경우 MAVLink 파싱이
# 깨짐), 반대로 control/target_distance 쪽이 아무 데이터도 못 받을 수 있다.
# gcs/sender/telem_sender_main.cpp 상단 주석의 KNOWN LIMITATION 그대로임 -
# 아직 코드로 해결된 문제가 아니라 그대로 남겨둠.
set -u
cd "$(dirname "$0")"
REPO_ROOT="$(cd .. && pwd)"

echo "=== 1/6: health_check ==="
if ! ./health_check.sh; then
    echo "[full_mission] health_check 실패 - 비행 시퀀스를 시작하지 않음." >&2
    exit 1
fi

GCS_LOG=/tmp/telem_sender.log
YOLO_LOG=/tmp/yolo_headless.log
TARGET_DIST_LOG=/tmp/target_distance.log

echo "=== 2/6: gcs (telem_sender, 백그라운드, 로그: $GCS_LOG) ==="
(cd "$REPO_ROOT/scripts" && setsid nohup ./gcs.sh >"$GCS_LOG" 2>&1 </dev/null &)
sleep 1

cleanup() {
    echo
    echo "[full_mission] 종료 - telem_sender, yolo_headless, target_distance 정리 중..."
    pkill -f './telem_sender' 2>/dev/null || true
    pkill -f './yolo_headless' 2>/dev/null || true
    pkill -f './target_distance' 2>/dev/null || true
}
trap cleanup EXIT

echo "=== 3/6: yolo_headless (백그라운드, 로그: $YOLO_LOG) ==="
(cd "$REPO_ROOT/YOLO_MODEL/cpp" && setsid nohup ./run_yolo_headless.sh >"$YOLO_LOG" 2>&1 </dev/null &)
sleep 1

# setting/MAVLink.yaml target_track.yolo_port 값을 그대로 읽어온다 (yolo_headless는
# 이 스크립트가 방금 로컬로 띄운 것이므로 host는 항상 127.0.0.1).
YOLO_PORT="$(grep -E '^\s*yolo_port:' "$REPO_ROOT/setting/MAVLink.yaml" | head -1 |
             sed -E 's/^[^:]*:\s*([0-9]+).*/\1/')"
YOLO_PORT="${YOLO_PORT:-8002}"
YOLO_TARGET_URL="http://127.0.0.1:${YOLO_PORT}/target"
DETECT_POLL_SEC=0.5

echo "=== 4/6: 욜로 탐지 대기 (target_distance/control 아직 미실행, $YOLO_TARGET_URL 폴링) ==="
while true; do
    body="$(curl -s --max-time 1 "$YOLO_TARGET_URL" 2>/dev/null)"
    if echo "$body" | grep -qE '"confirmed":[[:space:]]*true'; then
        echo "[full_mission] 타겟 확정 감지 - target_distance/control 시작."
        break
    fi
    sleep "$DETECT_POLL_SEC"
done

echo "=== 5/6: target_distance (백그라운드, 로그: $TARGET_DIST_LOG) ==="
(cd "$REPO_ROOT/control/build" && setsid nohup ./target_distance >"$TARGET_DIST_LOG" 2>&1 </dev/null &)
sleep 1

echo "=== 6/6: control --auto-intercept (포그라운드 - AUTO 감시 + 필요시 GUIDED 요격. Ctrl+C로 전체 종료) ==="
cd "$REPO_ROOT/control/build"
./control --auto-intercept
