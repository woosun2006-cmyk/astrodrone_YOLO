#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/real-runtime-common.sh"

duration=10
telemetry_only=0
serial_endpoint="${ASTRODRONE_SERIAL_ENDPOINT:-}"
while (( $# )); do
  case "$1" in
    --duration) duration="$2"; shift 2 ;;
    --connect) serial_endpoint="$2"; shift 2 ;;
    --telemetry-only) telemetry_only=1; shift ;;
    -h|--help)
      printf '%s\n' '사용법: observe-real.sh --connect /dev/serial/by-id/... [--duration 초] [--telemetry-only]'
      exit 0
      ;;
    *) printf '[실패] 알 수 없는 옵션: %s\n' "$1" >&2; exit 2 ;;
  esac
done

require_serial_endpoint "$serial_endpoint"
export ASTRODRONE_SERIAL_ENDPOINT="$serial_endpoint"
export ASTRODRONE_SERIAL_OWNER_CONFIRMED=1
export ASTRODRONE_COMMAND_MODE=observe
export ASTRODRONE_ALLOW_MAVLINK_WRITES=0
export ASTRODRONE_ALLOW_VEHICLE_COMMANDS=0
export ASTRODRONE_ALLOW_TELEMETRY_CONFIGURATION=0
export ASTRODRONE_HEALTH_TELEMETRY_ENDPOINT="${ASTRODRONE_HEALTH_TELEMETRY_ENDPOINT:-udp:127.0.0.1:14554}"

mkdir -p "$LOG_DIR"
run_log="$LOG_DIR/observe-real_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_log"
export CAMERA_FRAME_READY_FILE="$run_log/camera_frame_ready"
export CAMERA_SOURCE_READY_FILE="$run_log/camera_source_ready"
export YOLO_HTTP_ENDPOINT="http://127.0.0.1:8002"
audit_report="$run_log/command_audit.json"
write_zero_command_audit "$audit_report" observe-real
yolo_pid=''
publisher_pid=''
autonomy_pid=''
cleanup() {
  for pid in "$autonomy_pid" "$publisher_pid" "$yolo_pid"; do
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
  printf '[정보] TensorRT 엔진=%s\n' "$YOLO_ENGINE_PATH" | tee -a "$run_log/launcher.log"
  "$YOLO_DIR/cpp/run_yolo_live.sh" \
    --frame-source jetson \
    --onnx "$YOLO_ONNX_PATH" \
    --engine "$YOLO_ENGINE_PATH" \
    </dev/null >"$run_log/yolo.stdout.log" 2>"$run_log/yolo.stderr.log" &
  yolo_pid=$!
  export YOLO_PROCESS_PID="$yolo_pid"
  if ! wait_http_port 8002 30; then
    printf '[실패] Jetson YOLO HTTP 준비 실패\n[로그] %s\n' "$run_log" >&2
    exit 1
  fi
else
  printf '%s\n' '[정보] fake UDP telemetry 관찰 모드: camera/YOLO는 실행하지 않음' >"$run_log/launcher.log"
fi

ASTRODRONE_GCS_TELEMETRY_ENDPOINT="${GCS_TELEMETRY_ENDPOINT:-udp:127.0.0.1:14553}" \
  "$REPO_ROOT/gcs/run_publisher.sh" \
  </dev/null >"$run_log/telemetry-publisher.log" 2>&1 &
publisher_pid=$!

export DRONE_PROFILE=real
export LOG_DIR="$run_log"
export AUTONOMY_DETAIL_LOG="$run_log/autonomy.log"
"$SCRIPT_DIR/autonomy-cpp" --profile real \
  </dev/null > >(tee -a "$run_log/autonomy.log") 2>&1 &
autonomy_pid=$!

health_args=(
  --target real \
  --connect "$ASTRODRONE_HEALTH_TELEMETRY_ENDPOINT" \
  --duration-sec "$duration" \
  --period-ms 500
)
if [[ "${HEALTH_CHECK_TERMINAL:-0}" == 1 ]]; then
  health_args+=(--health-check-terminal)
  "$REPO_ROOT/scripts/health-check-cpp" "${health_args[@]}" \
    </dev/null > >(tee -a "$run_log/status-monitor.log") 2>&1
else
  "$REPO_ROOT/scripts/health-check-cpp" "${health_args[@]}" \
    </dev/null >>"$run_log/status-monitor.log" 2>&1
fi
