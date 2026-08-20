#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/real-runtime-common.sh"

duration=10
telemetry_only=0
while (( $# )); do
  case "$1" in
    --duration) duration="$2"; shift 2 ;;
    --telemetry-only) telemetry_only=1; shift ;;
    -h|--help)
      printf '%s\n' '사용법: observe-real.sh [--duration 초] [--telemetry-only]'
      exit 0
      ;;
    *) printf '[실패] 알 수 없는 옵션: %s\n' "$1" >&2; exit 2 ;;
  esac
done

require_real_router_endpoints
require_no_direct_serial_endpoint "$MAVPROXY_TELEMETRY_ENDPOINT"
export ASTRODRONE_COMMAND_MODE=observe
export ASTRODRONE_ALLOW_MAVLINK_WRITES=0
export ASTRODRONE_ALLOW_VEHICLE_COMMANDS=0
export ASTRODRONE_ALLOW_TELEMETRY_CONFIGURATION=0

mkdir -p "$LOG_DIR"
run_log="$LOG_DIR/observe-real_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_log"
audit_report="$run_log/command_audit.json"
write_zero_command_audit "$audit_report" observe-real
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

if (( telemetry_only == 0 )); then
  export GCS_VIDEO_ENABLED="${GCS_VIDEO_ENABLED:-1}"
  export GCS_VIDEO_ANNOTATED="${GCS_VIDEO_ANNOTATED:-0}"
  "$YOLO_DIR/cpp/run_yolo_live.sh" \
    --frame-source jetson \
    --onnx "$YOLO_ONNX_PATH" \
    --engine "$YOLO_ENGINE_PATH" \
    >"$run_log/yolo.stdout.log" 2>"$run_log/yolo.stderr.log" &
  yolo_pid=$!
  if ! wait_http_port 8002 30; then
    printf '[실패] Jetson YOLO HTTP 준비 실패\n[로그] %s\n' "$run_log" >&2
    exit 1
  fi
else
  printf '%s\n' '[정보] fake UDP telemetry 관찰 모드: camera/YOLO는 실행하지 않음' >"$run_log/launcher.log"
fi

MAVPROXY_GCS_TELEMETRY_ENDPOINT="${MAVPROXY_GCS_TELEMETRY_ENDPOINT:-udp:127.0.0.1:14553}" \
  "$REPO_ROOT/gcs/run_publisher.sh" \
  >"$run_log/telemetry-publisher.log" 2>&1 &
publisher_pid=$!

python3 "$REPO_ROOT/health-check/monitor/status_monitor.py" \
  --profile real \
  --connect "$MAVPROXY_TELEMETRY_ENDPOINT" \
  --read-only \
  --duration "$duration" \
  > >(tee -a "$run_log/status-monitor.log") 2>&1
