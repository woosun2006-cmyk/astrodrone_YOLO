#!/bin/bash
# control/motor_test 실행 래퍼. 지상 모터 테스트(모터 1개를 스로틀 5%로
# 돌리기 시작해서 키보드로 올리고 내리는 도구)를 사람이 직접 돌리기 위한
# 진입점 - health_check.sh 주석에 적혀 있듯 실제로 모터를 돌리는 프로그램은
# full_mission.sh/health_check.sh 같은 자동 스크립트에 절대 넣지 않는다.
#
# motor_test는 setting/port.yaml의 mavlink_motor_test(14553) 포트로 붙는데,
# 그 포트에 시리얼 데이터를 fan-out해주는 건 mavlink_proxy뿐이다. 그래서 이
# 스크립트는 --address로 직접 다른 주소(예: 시리얼 직결)를 준 게 아니면,
# mavlink_proxy가 이미 떠 있는지 확인하고 없으면 백그라운드로 대신
# 띄워준다 - 그 경우에만 종료 시 다시 정리한다 (원래 떠 있던 걸 죽이지
# 않음, full_mission.sh 등 다른 세션과 공유 중일 수 있으므로).
#
# motor_test 자체는 반드시 포그라운드로 실행한다 (setsid/nohup으로 백그라운드에
# 두지 않음) - 터미널을 raw 모드로 바꿔 키 입력을 읽고, 시작 전 "no prop"
# 확인을 stdin으로 직접 입력받아야 하기 때문.
#
# 사용법: scripts/motor_test.sh [motor_test 옵션...]
#   예) scripts/motor_test.sh --motor 1 --motor-count 4 --step 5
#   예) scripts/motor_test.sh --all --dwell 3   # 전체 모터 순서대로 자동 순환
#   예) scripts/motor_test.sh --together        # 전체 모터 동시 회전
# 옵션은 모두 그대로 control/motor_test에 전달된다 (--address/--motor/
# --motor-count/--step/--all/--dwell/--together). 상세 사용법은
# scripts/motor_test.sh --help 로 확인.
set -u
cd "$(dirname "$0")"
REPO_ROOT="$(cd .. && pwd)"
CONTROL_BUILD_DIR="$REPO_ROOT/control/build"

if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
    cat <<'EOF'
scripts/motor_test.sh [옵션...]

지상 모터 테스트 도구 (control/motor_test)를 빌드하고 실행한다.
실제 모터가 돌아가므로 프로펠러를 반드시 제거하고 사용할 것.

옵션 (그대로 control/motor_test에 전달됨):
  --address <addr>      MAVLink 접속 주소. 기본값은 setting/MAVLink.yaml의
                         real.proxy_udp.address + setting/port.yaml의
                         mavlink_motor_test(14553) 포트. control.cpp 전용
                         포트(mavlink_control)와 분리되어 있어 둘이 동시에
                         떠 있어도 충돌하지 않는다.
  --motor <n>            테스트할 모터 번호, 1부터 (기본값 1)
  --motor-count <n>      기체의 모터 개수 (기본값 4). 숫자키 1-9 중 이
                         값보다 큰 키는 무시됨 - 4모터 기체면 5-9는 반응 없음
  --step <percent>       한 번 키를 누를 때 스로틀 증감폭 (기본값 5)
  --all                  시작부터 전체 모터 순차 자동 순환 모드로 켠다
  --dwell <sec>          자동 순환 모드에서 모터 1개당 도는 시간 (기본값 3)
  --together             시작부터 전체 모터 동시 회전 모드로 켠다
                         (--all과 동시에 줄 수 없음)

기본 주소(UDP 프록시)를 쓸 때는 mavlink_proxy가 떠 있어야 시리얼 데이터가
14553 포트로 전달된다 - 이 스크립트가 안 떠 있으면 알아서 백그라운드로
띄워준다. --address로 시리얼 등 다른 주소를 직접 지정하면 이 과정을
건너뛴다.

실행 중 조작법:
  w / +   스로틀 증가         s / -   스로틀 감소
  0       즉시 정지(0%)       1-4     해당 모터로 즉시 전환 (--motor-count까지, 기본 4)
  n / p   다음 / 이전 모터     a       전체 모터 순차 자동 순환 켜기/끄기
  t       전체 모터 동시 회전 켜기/끄기
  q       모터 정지 후 종료   Ctrl+C  즉시 정지 후 종료

MAV_CMD_DO_MOTOR_TEST 자체엔 "N개 동시" 파라미터가 없다. `t`(동시 회전)는
ArduCopter 구현이 커맨드가 올 때마다 그 모터 채널에만 값을 쓰고 다른 채널은
그대로 둔 채 전체 세션의 데드맨 타이머만 갱신한다는 점을 이용해, 전체
모터에 짧은 간격으로 계속 명령을 돌려가며 보내 동시 회전을 흉내낸다 -
ArduCopter 구현 세부사항에 기대는 방식이라 펌웨어 버전에 따라 다르게 동작할
수 있다. 동시에 안 도는 것처럼 보이면 즉시 중단할 것. 여러 모터가 한꺼번에
돌면 합산 추력이 커지니 고정을 더 단단히 해야 한다.

시작하면 이미 armed 상태인지 확인해 armed면 거부하고, 이어서 "no prop"을
정확히 입력해야 진행된다 (대소문자 무관, 건너뛰는 옵션 없음). 모든
명령에는 짧은 타임아웃이 실려 있어(데드맨 스위치), 프로그램이 죽거나
링크가 끊겨도 ArduPilot이 스스로 모터를 멈춘다.

예:
  scripts/motor_test.sh                          # 모터 1번, 4모터 기체 기준
  scripts/motor_test.sh --motor 3 --step 2       # 모터 3번, 2%씩 조절
  scripts/motor_test.sh --all --dwell 3          # 전체 모터 3초씩 순차 자동 순환
  scripts/motor_test.sh --together               # 전체 모터 동시 회전
  scripts/motor_test.sh --address /dev/ttyACM1   # UDP 프록시 대신 시리얼 직결
EOF
    exit 0
fi

echo "=== 경고: control/motor_test는 실제 모터를 회전시킵니다. 프로펠러를 제거하세요. ==="

needs_proxy=1
for arg in "$@"; do
    if [ "$arg" = "--address" ]; then
        needs_proxy=0
        break
    fi
done

started_proxy=0
PROXY_LOG=/tmp/mavlink_proxy.log
if [ "$needs_proxy" = "1" ]; then
    if pgrep -f './mavlink_proxy' >/dev/null 2>&1; then
        echo "[motor_test.sh] mavlink_proxy가 이미 실행 중 - 그대로 사용."
    else
        echo "[motor_test.sh] mavlink_proxy 미실행 - 백그라운드로 새로 띄움 (로그: $PROXY_LOG)"
        mkdir -p "$CONTROL_BUILD_DIR"
        (cd "$CONTROL_BUILD_DIR" && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. >/dev/null && \
         make -j"$(nproc)" mavlink_proxy)
        if [ $? -ne 0 ]; then
            echo "[motor_test.sh] mavlink_proxy 빌드 실패" >&2
            exit 1
        fi
        (cd "$CONTROL_BUILD_DIR" && setsid nohup ./mavlink_proxy >"$PROXY_LOG" 2>&1 </dev/null &)
        started_proxy=1
        sleep 2
    fi
fi

cleanup() {
    if [ "$started_proxy" = "1" ]; then
        echo "[motor_test.sh] 이 스크립트가 띄운 mavlink_proxy 정리 중..."
        pkill -f './mavlink_proxy' 2>/dev/null || true
    fi
}
trap cleanup EXIT

mkdir -p "$CONTROL_BUILD_DIR"
(cd "$CONTROL_BUILD_DIR" && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. >/dev/null && \
 make -j"$(nproc)" motor_test)
build_status=$?
if [ $build_status -ne 0 ]; then
    echo "[motor_test.sh] 빌드 실패 (exit $build_status)" >&2
    exit $build_status
fi

cd "$CONTROL_BUILD_DIR"
./motor_test "$@"
exit $?
