#!/bin/bash
# Gazebo/SITL run of the mission stack WITHOUT camera perception.
#
# Same ordering as full_mission.sh steps 2,3,4,6,7, with two substitutions
# that simulation forces:
#
#   1. mavlink_proxy talks to a PC-hosted ArduPilot SITL instead of the
#      Pixhawk. mavlink_proxy.cpp only ever opens setting/MAVLink.yaml's
#      real.serial.address, so gazeboSim/bridge_sitl.sh gives it a PTY that
#      relays to the SITL and that address is pointed at the PTY. This script
#      does the swap and ALWAYS restores the original file on exit.
#
#   2. yolo_headless is replaced by gazeboSim/fake_yolo_target.py. It cannot
#      run against Gazebo at all: it opens the camera through a hardcoded
#      `nvarguscamerasrc` pipeline (Jetson CSI/Argus only). This variant
#      serves a static synthetic detection, so perception is NOT validated -
#      only target_distance.cpp's range maths and control.cpp's GUIDED
#      takeover are. Use scripts/sim_mission_camera.sh for real Gazebo video.
#
# health_check.sh is NOT run here: its check_link opens the serial device
# directly and would fight mavlink_proxy for the PTY. Run it separately
# before this script if you want the gate.
#
# Prerequisites on the PC (see gazeboSim/README.md):
#   - Gazebo + ArduPilot SITL running, SITL SERIAL1 (tcp 5762) reachable
#   - the reverse-tunnel chain to droneVideo:25760 up
set -u
cd "$(dirname "$0")"
REPO_ROOT="$(cd .. && pwd)"

BACKUP="$REPO_ROOT/gazeboSim/MAVLink.yaml.sim-backup"
SETTINGS="$REPO_ROOT/setting/MAVLink.yaml"

if [ ! -f "$REPO_ROOT/gazeboSim/bridge_sitl.sh" ]; then
    echo "[sim] gazeboSim/ not found - this script only runs from the simulation setup." >&2
    exit 1
fi

echo "=== 0/5: backing up setting/MAVLink.yaml ==="
cp "$SETTINGS" "$BACKUP"
echo "backup: $BACKUP"

restore_settings() {
    if [ -f "$BACKUP" ]; then
        cp "$BACKUP" "$SETTINGS"
        echo "[sim] setting/MAVLink.yaml restored (real.serial.address back to the Pixhawk)."
    fi
}

cleanup() {
    echo
    echo "[sim] shutting down..."
    pkill -f './control --auto-intercept' 2>/dev/null
    pkill -f './target_distance' 2>/dev/null
    pkill -f './telem_sender' 2>/dev/null
    pkill -f './mavlink_proxy' 2>/dev/null
    pkill -f 'fake_yolo_target' 2>/dev/null
    pkill -f 'socat .*sitl_serial' 2>/dev/null
    restore_settings
}
trap cleanup EXIT INT TERM

echo
echo "=== 1/5: PTY bridge to the PC's SITL ==="
./../gazeboSim/bridge_sitl.sh || { echo "[sim] bridge failed" >&2; exit 1; }

echo
echo "=== 2/5: pointing real.serial.address at the bridge PTY ==="
sed -i 's|^\( *address: \)/dev/tty[A-Za-z0-9]*|\1/tmp/sitl_serial|' "$SETTINGS"
grep -A2 '^real:' "$SETTINGS"

echo
echo "=== 3/5: mavlink_proxy + telem_sender ==="
CONTROL_BUILD="$REPO_ROOT/control/build"
GCS_BUILD="$REPO_ROOT/gcs/build"
mkdir -p "$CONTROL_BUILD" "$GCS_BUILD"
(cd "$CONTROL_BUILD" && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. >/dev/null &&
 make -j"$(nproc)" mavlink_proxy target_distance control) || exit 1
(cd "$GCS_BUILD" && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. >/dev/null &&
 make -j"$(nproc)") || exit 1

(cd "$CONTROL_BUILD" && setsid nohup ./mavlink_proxy >/tmp/mavlink_proxy.log 2>&1 </dev/null &)
sleep 3
(cd "$GCS_BUILD" && setsid nohup ./telem_sender >/tmp/telem_sender.log 2>&1 </dev/null &)
sleep 1

echo
echo "=== 4/5: fake /target responder (NO perception) ==="
(cd "$REPO_ROOT" && setsid nohup python3 gazeboSim/fake_yolo_target.py --port 8002 \
    >/tmp/fake_yolo.log 2>&1 </dev/null &)
sleep 2
curl -s --max-time 2 http://127.0.0.1:8002/target || { echo "[sim] fake responder failed" >&2; exit 1; }
echo

echo
echo "=== 5/5: target_distance, then control --auto-intercept (foreground) ==="
(cd "$CONTROL_BUILD" && setsid nohup ./target_distance >/tmp/target_distance.log 2>&1 </dev/null &)
sleep 3

echo "[sim] the vehicle must already be armed and flying AUTO on the SITL."
echo "[sim] Ctrl+C stops everything and restores setting/MAVLink.yaml."
cd "$CONTROL_BUILD"
./control --auto-intercept
