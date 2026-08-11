#!/bin/bash
# 미션 전체를 하나로 묶는 진입점: 켜놓으면
#   1. health_check.sh 로 비행 가능한 상태인지 게이트 확인
#   2. run_YOLO.sh 로 타겟 탐지 시작 (백그라운드) - 이후로는 이것만 돌아간다
#   3. 욜로 /target 이 confirmed=true 를 낼 때까지 가볍게 폴링만 하며 대기.
#      target_distance/control은 아직 띄우지 않는다 - 탐지된 것도 없는데
#      거리 계산/픽스호크 연결까지 미리 돌릴 필요가 없기 때문 (젯슨 리소스
#      절약). "젯슨이 제어하는 구조"는 욜로가 대상을 확정했을 때만 켜진다.
#   4. target_distance.cpp 로 탐지 결과 + 고도를 거리로 변환 (백그라운드)
#   5. control --auto-intercept (포그라운드): 픽스호크가 (이미 다른 경로로
#      arm되어) AUTO 모드로 미션을 날고 있는 걸 지켜보다가, 욜로가 타겟을
#      lock_confirm_sec 이상 안정적으로 잡으면 그때 GUIDED로 가로채서 접근
#      -> 타겟을 놓치거나 intercept_approach_duration_sec이 지나면 다시
#      AUTO로 돌려주고 재탐지 대기 (setting/MAVLink.yaml target_track 참고).
# health_check가 실패하면 아예 시작하지 않는다. Ctrl+C는 대기 단계(3번)든
# 비행 단계(5번)든 상관없이 yolo_live/target_distance까지 정리한다.
#
# 이 스크립트는 AUTO 진입/arm 자체는 하지 않는다 - 그건 미션 업로드와 함께
# 조종기/GCS로 조작하는 부분. control --auto-intercept는 그 상태를 그냥
# 지켜보다가 필요할 때만 끼어든다.
#
# 한 번 target_distance/control이 뜬 뒤로는, 비행 중 타겟을 잠깐 놓치는 것쯤은
# control --auto-intercept 자체가 처리한다 (AUTO 복귀 후 재탐지) - 여기서
# 다시 죽였다 살렸다 하지 않는다. 이 게이트는 "시작 시점"에만 적용된다.
set -u
cd "$(dirname "$0")"
REPO_ROOT="$(cd .. && pwd)"

echo "=== 1/5: health_check ==="
if ! ./health_check.sh; then
    echo "[full_mission] health_check 실패 - 비행 시퀀스를 시작하지 않음." >&2
    exit 1
fi

YOLO_LOG=/tmp/yolo_live.log
TARGET_DIST_LOG=/tmp/target_distance.log

echo "=== 2/5: run_YOLO (백그라운드, 로그: $YOLO_LOG) ==="
(cd "$REPO_ROOT/YOLO_MODEL/cpp" && setsid nohup ./run_yolo_live.sh >"$YOLO_LOG" 2>&1 </dev/null &)
sleep 1

cleanup() {
    echo
    echo "[full_mission] 종료 - yolo_live, target_distance 정리 중..."
    pkill -f './yolo_live' 2>/dev/null || true
    pkill -f './target_distance' 2>/dev/null || true
}
trap cleanup EXIT

# setting/MAVLink.yaml target_track.yolo_port 값을 그대로 읽어온다 (yolo_live는
# 이 스크립트가 방금 로컬로 띄운 것이므로 host는 항상 127.0.0.1).
YOLO_PORT="$(grep -E '^\s*yolo_port:' "$REPO_ROOT/setting/MAVLink.yaml" | head -1 |
             sed -E 's/^[^:]*:\s*([0-9]+).*/\1/')"
YOLO_PORT="${YOLO_PORT:-8002}"
YOLO_TARGET_URL="http://127.0.0.1:${YOLO_PORT}/target"
DETECT_POLL_SEC=0.5

echo "=== 3/5: 욜로 탐지 대기 (target_distance/control 아직 미실행, $YOLO_TARGET_URL 폴링) ==="
while true; do
    body="$(curl -s --max-time 1 "$YOLO_TARGET_URL" 2>/dev/null)"
    if echo "$body" | grep -qE '"confirmed":[[:space:]]*true'; then
        echo "[full_mission] 타겟 확정 감지 - target_distance/control 시작."
        break
    fi
    sleep "$DETECT_POLL_SEC"
done

echo "=== 4/5: target_distance (백그라운드, 로그: $TARGET_DIST_LOG) ==="
(cd "$REPO_ROOT/control/build" && setsid nohup ./target_distance >"$TARGET_DIST_LOG" 2>&1 </dev/null &)
sleep 1

echo "=== 5/5: control --auto-intercept (포그라운드 - AUTO 감시 + 필요시 GUIDED 요격. Ctrl+C로 전체 종료) ==="
cd "$REPO_ROOT/control/build"
./control --auto-intercept
