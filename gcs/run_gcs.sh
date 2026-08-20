#!/bin/bash
# Starts the onboard YOLO pipeline and read-only router-telemetry publisher.
# Pixhawk serial is never opened here; an external MAVLink router owns it.
set -e
cd "$(dirname "$0")"
REPO_ROOT="$(cd .. && pwd)"

YOLO_LOG=${YOLO_LOG:-/tmp/yolo_live.log}
echo "starting yolo_live (video) -> $YOLO_LOG"
(cd "$REPO_ROOT/YOLO_MODEL/cpp" && \
  GCS_VIDEO_ENABLED="${GCS_VIDEO_ENABLED:-1}" \
  setsid nohup ./run_yolo_live.sh > "$YOLO_LOG" 2>&1 < /dev/null &)
sleep 1

cleanup() {
    echo
    echo 'stopping yolo_live...'
    pkill -f './yolo_live' 2>/dev/null || true
}
trap cleanup EXIT

echo 'starting telem_sender (router UDP telemetry, foreground)'
MAVPROXY_GCS_TELEMETRY_ENDPOINT="${MAVPROXY_GCS_TELEMETRY_ENDPOINT:-${MAVPROXY_TELEMETRY_ENDPOINT:-udp:127.0.0.1:14551}}" \
  "$REPO_ROOT/gcs/run_publisher.sh"
