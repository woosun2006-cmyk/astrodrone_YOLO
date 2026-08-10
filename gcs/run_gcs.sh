#!/bin/bash
# Starts both the YOLO video pipeline and telem_sender together. Run this
# once from an SSH session instead of two separate terminals. Ctrl+C stops
# both.
set -e
cd "$(dirname "$0")"
REPO_ROOT="$(cd .. && pwd)"

YOLO_LOG=/tmp/yolo_live.log
echo "starting yolo_live (video) -> $YOLO_LOG"
(cd "$REPO_ROOT/YOLO_MODEL/cpp" && setsid nohup ./run_yolo_live.sh > "$YOLO_LOG" 2>&1 < /dev/null &)
sleep 1

cleanup() {
    echo
    echo 'stopping yolo_live...'
    pkill -f './yolo_live' 2>/dev/null || true
}
trap cleanup EXIT

echo 'starting telem_sender (foreground, Ctrl+C stops both)'
cd "$REPO_ROOT/gcs/build"
./telem_sender
