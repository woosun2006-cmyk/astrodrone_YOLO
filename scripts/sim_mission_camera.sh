#!/bin/bash
# Gazebo/SITL run of the mission stack WITH real camera perception.
#
# Identical to scripts/sim_mission_no_camera.sh except for where /target comes
# from: instead of the static fake responder, detections are produced from the
# actual Gazebo camera by gazeboSim/gazebo_camera_target.py, which runs ON THE
# PC (the simulated camera only exists there) and is reached through an SSH
# reverse tunnel so it appears on this Jetson's 127.0.0.1:8002 - exactly where
# setting/MAVLink.yaml's target_track.yolo_host/yolo_port already point. No
# settings change is needed for the perception side.
#
# yolo_headless is still not used: YOLO_MODEL/cpp/yolo_headless.cpp opens the
# camera through a hardcoded `nvarguscamerasrc` pipeline (Jetson CSI/Argus),
# which has no Gazebo equivalent. Giving it a Gazebo source would mean editing
# that file - see gazeboSim/README.md.
#
# PC-side prerequisites (see gazeboSim/README.md for the exact commands):
#   1. Gazebo + SITL running, SITL SERIAL1 (tcp 5762) tunnelled to
#      droneVideo:25760
#   2. camera streaming enabled on the gimbal sensor's
#      .../image/enable_streaming topic (GstCameraPlugin -> udp 5600)
#   3. gazebo_camera_target.py running on the PC, its port 8002 reverse
#      tunnelled to this Jetson's 127.0.0.1:8002
set -u
cd "$(dirname "$0")"
REPO_ROOT="$(cd .. && pwd)"

BACKUP="$REPO_ROOT/gazeboSim/MAVLink.yaml.sim-backup"
SETTINGS="$REPO_ROOT/setting/MAVLink.yaml"

if [ ! -f "$REPO_ROOT/gazeboSim/bridge_sitl.sh" ]; then
    echo "[sim] gazeboSim/ not found - this script only runs from the simulation setup." >&2
    exit 1
fi

echo "=== 0/5: checking the PC-side camera detector is reachable ==="
if ! curl -s --max-time 3 http://127.0.0.1:8002/target >/dev/null; then
    echo "[sim] nothing serving /target on 127.0.0.1:8002." >&2
    echo "[sim] Start gazebo_camera_target.py on the PC and tunnel its port here," >&2
    echo "[sim] or use scripts/sim_mission_no_camera.sh instead." >&2
    exit 1
fi
curl -s --max-time 3 http://127.0.0.1:8002/target
echo

echo
echo "=== 0b/5: backing up setting/MAVLink.yaml ==="
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
    pkill -f 'socat .*sitl_serial' 2>/dev/null
    # the camera detector lives on the PC; it is intentionally left running
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
echo "=== 4/5: target_distance (polling the Gazebo camera detector) ==="
(cd "$CONTROL_BUILD" && setsid nohup ./target_distance >/tmp/target_distance.log 2>&1 </dev/null &)
sleep 3
tail -3 /tmp/target_distance.log

echo
echo "=== 5/5: control --auto-intercept (foreground) ==="
echo "[sim] the vehicle must already be armed and flying AUTO on the SITL."
echo "[sim] Ctrl+C stops everything and restores setting/MAVLink.yaml."
cd "$CONTROL_BUILD"
./control --auto-intercept
