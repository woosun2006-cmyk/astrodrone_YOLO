#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/real-runtime-common.sh"

target=''
router_serial=''
command_endpoint=''
telemetry_endpoint=''
allow_arm=0
confirm=0
commands_enabled=0
router_owned=0
while (( $# )); do
  case "$1" in
    --target) target="$2"; shift 2 ;;
    --router-serial|--connect) router_serial="$2"; shift 2 ;;
    --command-endpoint) command_endpoint="$2"; shift 2 ;;
    --telemetry-endpoint) telemetry_endpoint="$2"; shift 2 ;;
    --allow-arm) allow_arm=1; shift ;;
    --confirm-real-flight) confirm=1; shift ;;
    --commands-enabled) commands_enabled=1; shift ;;
    --router-owned) router_owned=1; shift ;;
    -h|--help)
      printf '%s\n' '사용법: flight-real.sh --target real --connect /dev/serial/by-id/... --command-endpoint udp:127.0.0.1:14550 --telemetry-endpoint udp:127.0.0.1:14551 --allow-arm --confirm-real-flight --commands-enabled --router-owned'
      exit 0
      ;;
    *) printf '[실패] 알 수 없는 옵션: %s\n' "$1" >&2; exit 2 ;;
  esac
done

if [[ "$target" != real || "$allow_arm" != 1 || "$confirm" != 1 ||
      "$commands_enabled" != 1 || "$router_owned" != 1 ]]; then
  printf '%s\n' '[실행 거부] flight-real은 target real, 이중 확인, --commands-enabled, --router-owned가 모두 필요합니다.' >&2
  exit 2
fi
[[ "$router_serial" == /dev/serial/by-id/* && "$router_serial" != *REPLACE_WITH* ]] || {
  printf '%s\n' '[실행 거부] --router-serial은 /dev/serial/by-id/... 경로여야 합니다.' >&2
  exit 2
}
[[ -e "$router_serial" ]] || {
  printf '%s\n' '[실행 거부] router serial 경로가 존재하지 않습니다. serial을 열지는 않았습니다.' >&2
  exit 2
}
[[ "$command_endpoint" == udp:127.0.0.1:14550 ]] || {
  printf '%s\n' '[실행 거부] command endpoint는 udp:127.0.0.1:14550이어야 합니다.' >&2
  exit 2
}
[[ "$telemetry_endpoint" == udp:127.0.0.1:14551 ]] || {
  printf '%s\n' '[실행 거부] telemetry endpoint는 udp:127.0.0.1:14551이어야 합니다.' >&2
  exit 2
}

export ASTRODRONE_TARGET=real
export ASTRODRONE_COMMAND_MODE=flight
export ASTRODRONE_ALLOW_MAVLINK_WRITES=1
export ASTRODRONE_ALLOW_VEHICLE_COMMANDS=1
export ASTRODRONE_ALLOW_ARM=1
export ASTRODRONE_COMMANDS_ENABLED=1
export ASTRODRONE_ROUTER_OWNERSHIP_CONFIRMED=1
export ASTRODRONE_ALLOW_TELEMETRY_CONFIGURATION=1
export ASTRODRONE_CONFIRM_REAL_FLIGHT=1
export ASTRODRONE_ROUTER_SERIAL="$router_serial"
export ASTRODRONE_ROUTER_OWNERSHIP_CONFIRMED=1
export MAVPROXY_CONTROL_ENDPOINT="$command_endpoint"
export MAVPROXY_TELEMETRY_ENDPOINT="$telemetry_endpoint"
export REQUIRE_RC_PREFLIGHT=1
export REQUIRE_VISION_PREFLIGHT=1
export DRONE_PROFILE=real
export CONTROL_ARGS="--self-launch"
export GCS_VIDEO_ENABLED="${GCS_VIDEO_ENABLED:-1}"
export MAVPROXY_GCS_TELEMETRY_ENDPOINT="${MAVPROXY_GCS_TELEMETRY_ENDPOINT:-udp:127.0.0.1:14553}"

printf '%s\n' '[경고] flight-real 정책 검증 통과. 실제 운항 명령은 외부 router와 연결될 때만 발생합니다.'
publisher_pid=''
cleanup() {
  if [[ -n "$publisher_pid" ]] && kill -0 "$publisher_pid" 2>/dev/null; then
    kill "$publisher_pid" 2>/dev/null || true
    wait "$publisher_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM
"$REPO_ROOT/gcs/run_publisher.sh" \
  >"${LOG_DIR:-$REPO_ROOT/logs}/flight-real-telemetry-publisher.log" 2>&1 &
publisher_pid=$!
"$SCRIPT_DIR/autonomy-cpp" --profile real
