#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/real-runtime-common.sh"

duration=10
duration_set=0
telemetry_only=0
telemetry_bench=0
serial_endpoint="${ASTRODRONE_SERIAL_ENDPOINT:-}"
gcs_build_dir="${GCS_BUILD_DIR:-$REPO_ROOT/gcs/build}"
while (( $# )); do
  case "$1" in
    --duration) duration="$2"; duration_set=1; shift 2 ;;
    --connect) serial_endpoint="$2"; shift 2 ;;
    --telemetry-only) telemetry_only=1; shift ;;
    --telemetry-bench) telemetry_bench=1; shift ;;
    -h|--help)
      printf '%s\n' '사용법: observe-real.sh --connect /dev/serial/by-id/... [--duration 초] [--telemetry-only] [--telemetry-bench]'
      exit 0
      ;;
    *) printf '[실패] 알 수 없는 옵션: %s\n' "$1" >&2; exit 2 ;;
  esac
done

if (( telemetry_bench == 1 && telemetry_only == 1 )); then
  printf '%s\n' '[실패] --telemetry-bench는 YOLO/카메라를 실행하므로 --telemetry-only와 함께 사용할 수 없습니다.' >&2
  exit 2
fi

duration="$(resolve_observe_duration "$duration" "$duration_set" "$telemetry_bench")"

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
export YOLO_READY_FILE="$run_log/yolo_ready"
rm -f -- "$YOLO_READY_FILE"
export YOLO_HTTP_ENDPOINT="http://127.0.0.1:8002"
audit_report="$run_log/command_audit.json"
if (( telemetry_bench == 1 )); then
  printf '%s\n' '{"profile":"telemetry-bench","mode_command_count":0,"arm_command_count":0,"disarm_command_count":0,"velocity_command_count":0,"rc_override_count":0,"mission_upload_count":0,"set_message_interval_count":0,"vehicle_affecting_command_count":0,"direct_serial_write_count":0,"mavlink_tx_packets":0,"flight_enabled":false,"source":"explicit-read-only-telemetry-bench"}' >"$audit_report"
else
  write_zero_command_audit "$audit_report" observe-real
fi
yolo_pid=''
publisher_pid=''
autonomy_pid=''
target_distance_pid=''
telemetry_bench_pid=''
health_pid=''
cleanup_done=0
stop_process_tree() {
  local pid="$1" child
  [[ -n "$pid" ]] || return 0
  for child in $(pgrep -P "$pid" 2>/dev/null || true); do
    stop_process_tree "$child"
  done
  if kill -0 "$pid" 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
}
cleanup() {
  (( cleanup_done == 1 )) && return 0
  cleanup_done=1
  for pid in "$health_pid" "$telemetry_bench_pid" "$autonomy_pid" "$target_distance_pid" "$publisher_pid" "$yolo_pid"; do
    stop_process_tree "$pid"
  done
}
trap cleanup EXIT
if (( telemetry_bench == 1 )); then
  trap 'cleanup; exit 0' INT TERM
else
  trap cleanup INT TERM
fi

if (( telemetry_bench == 1 )); then
  printf '%s\n' '[PREBUILD] runtime 시작 전 순차 선빌드 시작' | tee -a "$run_log/launcher.log"
  if ! GCS_BUILD_DIR="$gcs_build_dir" \
      "$SCRIPT_DIR/prebuild-runtime-binaries.sh" >>"$run_log/launcher.log" 2>&1; then
    printf '[실패] runtime binary 선빌드 실패; 비행/bench를 시작하지 않습니다.\n[로그] %s\n' \
      "$run_log/launcher.log" >&2
    exit 1
  fi
fi

if (( telemetry_only == 0 )); then
  export GCS_VIDEO_ENABLED="${GCS_VIDEO_ENABLED:-1}"
  export GCS_VIDEO_ANNOTATED="${GCS_VIDEO_ANNOTATED:-0}"
  if (( telemetry_bench == 1 )) && [[ -n "${TENSORRT_LIB_DIR:-}" ]]; then
    export LD_LIBRARY_PATH="/usr/lib/wsl/lib:$TENSORRT_LIB_DIR:/usr/local/cuda/lib64:/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  fi
  printf '[정보] TensorRT 엔진=%s\n' "$YOLO_ENGINE_PATH" | tee -a "$run_log/launcher.log"
  if (( telemetry_bench == 1 )); then
    "$YOLO_LIVE_BUILD_DIR/yolo_live" \
      --onnx "$YOLO_ONNX_PATH" \
      --engine "$YOLO_ENGINE_PATH" \
      --frame-source jetson \
      </dev/null >"$run_log/yolo.stdout.log" 2>"$run_log/yolo.stderr.log" &
  else
    "$YOLO_DIR/cpp/run_yolo_live.sh" \
      --frame-source jetson \
      --onnx "$YOLO_ONNX_PATH" \
      --engine "$YOLO_ENGINE_PATH" \
      </dev/null >"$run_log/yolo.stdout.log" 2>"$run_log/yolo.stderr.log" &
  fi
  yolo_pid=$!
  export YOLO_PROCESS_PID="$yolo_pid"
  if ! wait_http_port 8002 30; then
    printf '[실패] Jetson YOLO HTTP 준비 실패\n[로그] %s\n' "$run_log" >&2
    exit 1
  fi
  if (( telemetry_bench == 1 )); then
    readiness_timeout="${YOLO_READY_TIMEOUT_SEC:-90}"
    wait_for_marker() {
      local marker="$1" label="$2"
      for _ in $(seq 1 "$readiness_timeout"); do
        if [[ -s "$marker" ]]; then
          printf '[BENCH] %s readiness 확인: %s\n' "$label" "$marker" | tee -a "$run_log/launcher.log"
          return 0
        fi
        sleep 1
      done
      printf '[실패] %s readiness 시간 초과: %s\n[로그] %s\n' "$label" "$marker" "$run_log" >&2
      return 1
    }
    wait_for_marker "$YOLO_READY_FILE" 'YOLO HTTP/model' || exit 1
    wait_for_marker "$CAMERA_SOURCE_READY_FILE" 'IMX219 camera source' || exit 1
    wait_for_marker "$CAMERA_FRAME_READY_FILE" 'camera frame' || exit 1
    printf '[BENCH] frame marker:\n%s\n' "$(cat "$CAMERA_FRAME_READY_FILE")" | tee -a "$run_log/launcher.log"
  fi
else
  printf '%s\n' '[정보] fake UDP telemetry 관찰 모드: camera/YOLO는 실행하지 않음' >"$run_log/launcher.log"
fi

if (( telemetry_bench == 1 )); then
  ASTRODRONE_GCS_TELEMETRY_ENDPOINT="${GCS_TELEMETRY_ENDPOINT:-udp:127.0.0.1:14553}" \
    "$gcs_build_dir/telem_sender" \
    --telemetry-endpoint "${GCS_TELEMETRY_ENDPOINT:-udp:127.0.0.1:14553}" \
    </dev/null >"$run_log/telemetry-publisher.log" 2>&1 &
else
  ASTRODRONE_GCS_TELEMETRY_ENDPOINT="${GCS_TELEMETRY_ENDPOINT:-udp:127.0.0.1:14553}" \
    "$REPO_ROOT/gcs/run_publisher.sh" \
    </dev/null >"$run_log/telemetry-publisher.log" 2>&1 &
fi
publisher_pid=$!

if (( telemetry_bench == 1 )); then
  export DRONE_PROFILE=telemetry-bench
  export LOG_DIR="$run_log"
  export ASTRODRONE_GCS_TELEMETRY_ENDPOINT="${GCS_TELEMETRY_ENDPOINT:-udp:127.0.0.1:14553}"
  "$BUILD_DIR/target_distance" \
    </dev/null >"$run_log/target-distance.log" 2>&1 &
  target_distance_pid=$!
  "$BUILD_DIR/telemetry_bench" \
    --connect "$serial_endpoint" \
    --duration-sec "$duration" \
    --telemetry-endpoint "${ASTRODRONE_TELEMETRY_ENDPOINT:-udp:127.0.0.1:14551}" \
    --gcs-telemetry-endpoint "$ASTRODRONE_GCS_TELEMETRY_ENDPOINT" \
    --health-telemetry-endpoint "$ASTRODRONE_HEALTH_TELEMETRY_ENDPOINT" \
    </dev/null >"$run_log/telemetry-bench.log" 2>&1 &
  telemetry_bench_pid=$!
  printf '%s\n' '[BENCH] read-only telemetry bench; flight disabled' | tee -a "$run_log/launcher.log"
else
  export DRONE_PROFILE=real
  export LOG_DIR="$run_log"
  export AUTONOMY_DETAIL_LOG="$run_log/autonomy.log"
  "$SCRIPT_DIR/autonomy-cpp" --profile real \
    </dev/null > >(tee -a "$run_log/autonomy.log") 2>&1 &
  autonomy_pid=$!
fi

health_args=(
  --target real \
  --connect "$ASTRODRONE_HEALTH_TELEMETRY_ENDPOINT" \
  --duration-sec "$duration" \
  --period-ms 500
)
if (( telemetry_bench == 1 )); then
  health_args+=(--telemetry-bench)
fi
if (( telemetry_bench == 1 )); then
  set +e
  if [[ "${HEALTH_CHECK_TERMINAL:-0}" == 1 ]]; then
    health_args+=(--health-check-terminal)
    "$BUILD_DIR/health_check" "${health_args[@]}" \
      </dev/null > >(tee -a "$run_log/status-monitor.log") 2>&1 &
  else
    "$BUILD_DIR/health_check" "${health_args[@]}" \
      </dev/null >>"$run_log/status-monitor.log" 2>&1 &
  fi
  health_pid=$!
  set -e
else
  if [[ "${HEALTH_CHECK_TERMINAL:-0}" == 1 ]]; then
    health_args+=(--health-check-terminal)
    set +e
    "$REPO_ROOT/scripts/health-check-cpp" "${health_args[@]}" \
      </dev/null > >(tee -a "$run_log/status-monitor.log") 2>&1
    health_status=$?
    set -e
  else
    set +e
    "$REPO_ROOT/scripts/health-check-cpp" "${health_args[@]}" \
      </dev/null >>"$run_log/status-monitor.log" 2>&1
    health_status=$?
    set -e
  fi
fi

if (( telemetry_bench == 1 )); then
  set +e
  wait "$telemetry_bench_pid"
  bench_status=$?
  wait "$health_pid"
  health_status=$?
  set -e
  if (( bench_status != 0 )); then
    printf '[실패] telemetry-bench read-only 경로 종료 상태=%s\n[로그] %s\n' \
      "$bench_status" "$run_log" >&2
    exit "$bench_status"
  fi
fi
if (( health_status != 0 )); then
  printf '[실패] health_check 종료 상태=%s\n[로그] %s\n' "$health_status" "$run_log" >&2
  exit "$health_status"
fi
