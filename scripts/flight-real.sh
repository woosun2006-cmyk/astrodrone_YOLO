#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/real-runtime-common.sh"

target=''
serial_endpoint=''
serial_owner=''
allow_arm=0
confirm=0
commands_enabled=0
upload_mission=0
arm=0
mission_altitude=''
mission_waypoint_lat=''
mission_waypoint_lon=''
while (( $# )); do
  case "$1" in
    --target) target="$2"; shift 2 ;;
    --connect) serial_endpoint="$2"; shift 2 ;;
    --serial-owner) serial_owner="$2"; shift 2 ;;
    --allow-arm) allow_arm=1; shift ;;
    --confirm-real-flight) confirm=1; shift ;;
    --commands-enabled) commands_enabled=1; shift ;;
    --upload-mission) upload_mission=1; shift ;;
    --arm) arm=1; shift ;;
    --altitude) mission_altitude="$2"; shift 2 ;;
    --waypoint-lat) mission_waypoint_lat="$2"; shift 2 ;;
    --waypoint-lon) mission_waypoint_lon="$2"; shift 2 ;;
    -h|--help)
      printf '%s\n' '사용법: flight-real.sh --target real --connect /dev/serial/by-id/... --serial-owner onboard --allow-arm --confirm-real-flight --commands-enabled --upload-mission --arm'
      exit 0
      ;;
    *) printf '[실패] 알 수 없는 옵션: %s\n' "$1" >&2; exit 2 ;;
  esac
done

if [[ "$target" != real || -z "$serial_endpoint" || "$serial_owner" != onboard ||
      "$allow_arm" != 1 || "$confirm" != 1 || "$commands_enabled" != 1 ||
      "$upload_mission" != 1 || "$arm" != 1 ]]; then
  printf '%s\n' '[실행 거부] target real, --connect, --serial-owner onboard, 이중 승인, --commands-enabled, --upload-mission, --arm이 모두 필요합니다.' >&2
  exit 2
fi
require_serial_endpoint "$serial_endpoint"

export ASTRODRONE_TARGET=real
export ASTRODRONE_COMMAND_MODE=flight
export ASTRODRONE_SERIAL_ENDPOINT="$serial_endpoint"
export ASTRODRONE_SERIAL_OWNER_CONFIRMED=1
export ASTRODRONE_ALLOW_MAVLINK_WRITES=1
export ASTRODRONE_ALLOW_VEHICLE_COMMANDS=1
export ASTRODRONE_ALLOW_ARM=1
export ASTRODRONE_COMMANDS_ENABLED=1
export ASTRODRONE_ALLOW_TELEMETRY_CONFIGURATION=1
export ASTRODRONE_CONFIRM_REAL_FLIGHT=1
export REQUIRE_RC_PREFLIGHT=1
export REQUIRE_VISION_PREFLIGHT=1
export DRONE_PROFILE=real
export AUTONOMY_MISSION_SETUP_REQUESTED=1
export GCS_VIDEO_ENABLED="${GCS_VIDEO_ENABLED:-1}"
export GCS_VIDEO_ANNOTATED="${GCS_VIDEO_ANNOTATED:-0}"
export ASTRODRONE_GCS_TELEMETRY_ENDPOINT="${GCS_TELEMETRY_ENDPOINT:-udp:127.0.0.1:14553}"

run_log="${LOG_DIR:-$REPO_ROOT/logs}/flight-real_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_log"
export LOG_DIR="$run_log"
export AUTONOMY_DETAIL_LOG="$run_log/autonomy.log"
export CAMERA_FRAME_READY_FILE="$run_log/camera_frame_ready"
export CAMERA_SOURCE_READY_FILE="$run_log/camera_source_ready"
export YOLO_READY_FILE="$run_log/yolo_ready"
rm -f -- "$YOLO_READY_FILE"
export YOLO_HTTP_ENDPOINT="http://127.0.0.1:8002"
export AUTONOMY_MISSION_SETUP_COMPLETE_FILE="$run_log/mission_setup_complete"
[[ -n "$mission_altitude" ]] && export MISSION_ALTITUDE_M="$mission_altitude"
[[ -n "$mission_waypoint_lat" ]] && export MISSION_WAYPOINT_LAT="$mission_waypoint_lat"
[[ -n "$mission_waypoint_lon" ]] && export MISSION_WAYPOINT_LON="$mission_waypoint_lon"

printf '%s\n' '[경고] onboard serial owner 정책 통과. 실제 운항 명령은 모든 preflight와 adapter lock 이후에만 허용됩니다.'
yolo_pid=''
publisher_pid=''
cleanup() {
  for pid in "$publisher_pid" "$yolo_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

printf '[정보] TensorRT 엔진=%s\n' "$YOLO_ENGINE_PATH" | tee -a "$run_log/launcher.log"
"$YOLO_DIR/cpp/run_yolo_live.sh" \
  --frame-source jetson \
  --onnx "$YOLO_ONNX_PATH" \
  --engine "$YOLO_ENGINE_PATH" \
  </dev/null >"$run_log/yolo.stdout.log" 2>"$run_log/yolo.stderr.log" &
yolo_pid=$!
export YOLO_PROCESS_PID="$yolo_pid"

yolo_ready=0
for _ in $(seq 1 "${YOLO_READY_TIMEOUT_SEC:-90}"); do
  if ! kill -0 "$yolo_pid" >/dev/null 2>&1; then
    printf '%s\n' '[실패] YOLO가 readiness 전에 종료되었습니다.' >&2
    printf '[로그] %s\n' "$run_log" >&2
    exit 1
  fi
  if ss -H -lnt 2>/dev/null | awk '$4 ~ /^127\.0\.0\.1:8002$/ { found=1 } END { exit(found ? 0 : 1) }'; then
    yolo_ready=1
    break
  fi
  sleep 1
done
if (( yolo_ready == 0 )); then
  printf '%s\n' '[실패] YOLO HTTP readiness 시간 초과로 onboard control을 시작하지 않습니다.' >&2
  printf '[로그] %s\n' "$run_log" >&2
  exit 1
fi

ASTRODRONE_GCS_TELEMETRY_ENDPOINT="$ASTRODRONE_GCS_TELEMETRY_ENDPOINT" \
  "$REPO_ROOT/gcs/run_publisher.sh" </dev/null >"$run_log/telemetry-publisher.log" 2>&1 &
publisher_pid=$!

# FlightMissionApp's AutopilotMavlinkAdapter opens the serial endpoint,
# acquires the exclusive lock, fans out telemetry, and owns all commands.
"$SCRIPT_DIR/autonomy-cpp" --profile real \
  > >(tee -a "$run_log/autonomy.log") 2>&1
