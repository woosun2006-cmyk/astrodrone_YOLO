#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/real-runtime-common.sh"

duration=60
serial_endpoint="${ASTRODRONE_SERIAL_ENDPOINT:-}"
while (( $# )); do
  case "$1" in
    --duration) duration="$2"; shift 2 ;;
    --connect) serial_endpoint="$2"; shift 2 ;;
    -h|--help)
      printf '%s\n' '사용법: shadow-real.sh --connect /dev/serial/by-id/... [--duration 초]'
      exit 0
      ;;
    *) printf '[실패] 알 수 없는 옵션: %s\n' "$1" >&2; exit 2 ;;
  esac
done

require_serial_endpoint "$serial_endpoint"
export ASTRODRONE_SERIAL_ENDPOINT="$serial_endpoint"
export ASTRODRONE_SERIAL_OWNER_CONFIRMED=1
export ASTRODRONE_COMMAND_MODE=shadow
export ASTRODRONE_ALLOW_MAVLINK_WRITES=0
export ASTRODRONE_ALLOW_VEHICLE_COMMANDS=0
export ASTRODRONE_ALLOW_TELEMETRY_CONFIGURATION=0

mkdir -p "$LOG_DIR"
run_log="$LOG_DIR/shadow-real_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_log"
export CAMERA_FRAME_READY_FILE="$run_log/camera_frame_ready"
export CAMERA_SOURCE_READY_FILE="$run_log/camera_source_ready"
export YOLO_READY_FILE="$run_log/yolo_ready"
rm -f -- "$YOLO_READY_FILE"
export YOLO_HTTP_ENDPOINT="http://127.0.0.1:8002"
audit_report="$run_log/command_audit.json"
control_events="$run_log/control_events.jsonl"
write_zero_command_audit "$audit_report" shadow-real
yolo_pid=''
autonomy_pid=''
publisher_pid=''
cleanup() {
  for pid in "$autonomy_pid" "$publisher_pid" "$yolo_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

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
if ! wait_http_port 8002 90; then
  printf '[실패] Jetson YOLO HTTP 준비 실패\n[로그] %s\n' "$run_log" >&2
  exit 1
fi

GCS_VIDEO_ENABLED="${GCS_VIDEO_ENABLED:-1}" \
ASTRODRONE_GCS_TELEMETRY_ENDPOINT="${GCS_TELEMETRY_ENDPOINT:-udp:127.0.0.1:14553}" \
  "$REPO_ROOT/gcs/run_publisher.sh" \
  </dev/null >"$run_log/telemetry-publisher.log" 2>&1 &
publisher_pid=$!

export LOG_DIR="$run_log"
export AUTONOMY_DETAIL_LOG="$run_log/autonomy.log"
export DRONE_PROFILE=real
export MAVLINK_AUDIT_EVENT_FILE="$control_events"
"$SCRIPT_DIR/autonomy-cpp" --profile real \
  </dev/null > >(tee -a "$run_log/autonomy.log") 2>&1 &
autonomy_pid=$!
timed_out=0
if ! timeout --foreground "$duration" tail --pid="$autonomy_pid" -f /dev/null; then
  timed_out=1
  kill "$autonomy_pid" 2>/dev/null || true
fi
set +e
wait "$autonomy_pid"
autonomy_status=$?
set -e

python3 - "$control_events" "$audit_report" <<'PY'
import json
import pathlib
import sys

events_path = pathlib.Path(sys.argv[1])
report_path = pathlib.Path(sys.argv[2])
decisions = blocked = sent = interval = 0
if events_path.exists():
    for line in events_path.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            continue
        if event.get("event") != "command_decision":
            continue
        decisions += 1
        blocked += int(bool(event.get("blocked")))
        sent += int(bool(event.get("sent")))
        interval += int(event.get("command_type") == "SET_MESSAGE_INTERVAL")
report_path.write_text(json.dumps({
    "profile": "shadow-real",
    "decision_count": decisions,
    "blocked_command_count": blocked,
    "vehicle_affecting_command_count": sent,
    "direct_serial_write_count": 0,
    "set_message_interval_count": interval,
    "source": "SafetyMonitor/CommandAuditLogger",
}, ensure_ascii=False) + "\n", encoding="utf-8")
if sent or interval:
    raise SystemExit(1)
PY

if (( timed_out == 1 )); then
  exit 0
fi
exit "$autonomy_status"
