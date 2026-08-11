#!/bin/bash
# ~/astro-drone/health-check 의 진단 프로그램들로 "지금 비행해도 안전한가"를
# 한 번에 점검한다. full_mission.sh가 비행 시작 전 게이트로 호출하고,
# 단독으로 돌려서 이륙 전 점검용으로 써도 된다.
#
# health-check/ 안의 5개 프로그램 중 자동 게이트에 쓰는 건 2개뿐:
#   - check_link   : heartbeat/모드/배터리/gps 가 실제로 들어오는지 (연결 확인)
#   - emergency    : setting/safety.yaml 기준(배터리/heartbeat/gps/prearm/ekf)
#                     BREACH 여부 (control.cpp가 비행 중 쓰는 것과 동일한 기준)
# check_alt(고도 유지 모니터)/get_gps_location 은 비행 중/정보성이라 제외했고,
# test_arm 은 실제로 모터에 arm 신호를 보내는 프로그램이라 자동 스크립트에
# 절대 넣지 않는다 - 프로펠러 안전 확인 후 사람이 직접 돌려야 하는 도구.
set -u
cd "$(dirname "$0")"
REPO_ROOT="$(cd .. && pwd)"
BUILD_DIR="$REPO_ROOT/health-check/build"

CHECK_LINK_SEC="${CHECK_LINK_SEC:-8}"
EMERGENCY_SEC="${EMERGENCY_SEC:-8}"

if [ ! -x "$BUILD_DIR/check_link" ] || [ ! -x "$BUILD_DIR/emergency" ]; then
    echo "[health_check] health-check 빌드가 없음: $BUILD_DIR" >&2
    echo "  cd $REPO_ROOT/health-check && mkdir -p build && cd build && cmake .. && make" >&2
    exit 2
fi

fail=0

echo "[health_check] 1/2 check_link: 연결/heartbeat/배터리/gps 확인 (${CHECK_LINK_SEC}s)"
LINK_LOG="$(mktemp)"
"$BUILD_DIR/check_link" --listen "$CHECK_LINK_SEC" >"$LINK_LOG" 2>&1
link_status=$?
cat "$LINK_LOG"
if [ $link_status -ne 0 ]; then
    echo "[health_check] check_link 실패 (exit $link_status) - 픽스호크 연결/케이블/전원 확인 필요"
    fail=1
fi
rm -f "$LINK_LOG"

echo "[health_check] 2/2 emergency: safety.yaml 기준 BREACH 여부 확인 (${EMERGENCY_SEC}s)"
EMER_LOG="$(mktemp)"
timeout "$EMERGENCY_SEC" "$BUILD_DIR/emergency" >"$EMER_LOG" 2>&1
emer_status=$?
cat "$EMER_LOG"
# timeout이 정상적으로 창을 다 채우고 죽였을 때(exit 124)는 실패가 아님 -
# emergency는 원래 Ctrl+C 전까지 무한 루프. 그 외 비정상 종료(예: heartbeat
# 없음 -> exit 1)만 하드 실패로 취급.
if [ $emer_status -ne 124 ] && [ $emer_status -ne 0 ]; then
    echo "[health_check] emergency 비정상 종료 (exit $emer_status)"
    fail=1
elif tail -n 1 "$EMER_LOG" | grep -q '\[BREACH\]'; then
    echo "[health_check] BREACH 상태로 점검 종료 - 마지막 상태 위 로그 참고"
    fail=1
fi
rm -f "$EMER_LOG"

if [ $fail -ne 0 ]; then
    echo "[health_check] 결과: 비행 불가 (UNSAFE)"
    exit 1
fi
echo "[health_check] 결과: 비행 가능 (SAFE)"
exit 0
