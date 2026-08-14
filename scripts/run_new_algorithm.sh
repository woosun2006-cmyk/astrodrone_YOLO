#!/bin/bash
# Runs control/new_algorithm's hybrid_guidance binary instead of `control`.
# Mirrors full_mission.sh / sim_mission_no_camera.sh / sim_mission_camera.sh's
# structure and process ordering exactly - none of those scripts, or anything
# they launch, are modified. The only substitution is the final foreground
# process: hybrid_guidance instead of `control [--auto-intercept]`.
#
# Modes:
#   (no flags)        real hardware - same gate as full_mission.sh: waits for
#                      yolo_headless's /target to report confirmed:true before
#                      starting target_distance + hybrid_guidance. Unlike
#                      `control --auto-intercept`, hybrid_guidance arms and
#                      takes off ITSELF (see control/new_algorithm/README.md) -
#                      there is no separate AUTO mission to watch for.
#   --sim              Gazebo/SITL, synthetic /target (sim_mission_no_camera.sh
#                      equivalent) - PC-side bring-up (gazeboSim/start_pc_sim.sh
#                      etc.) must already be running, see gazeboSim/README.md.
#   --sim --camera      Gazebo/SITL, real Gazebo camera detections
#                      (sim_mission_camera.sh equivalent) - additionally needs
#                      gazebo_camera_target.py tunnelled to 127.0.0.1:8002.
#
# In --sim mode this restores setting/MAVLink.yaml on exit, same as the
# existing sim scripts.
set -u
cd "$(dirname "$0")"
REPO_ROOT="$(cd .. && pwd)"

SIM=0
CAMERA=0
for arg in "$@"; do
    case "$arg" in
        --sim) SIM=1 ;;
        --camera) CAMERA=1 ;;
        *) echo "[run_new_algorithm] unknown flag: $arg" >&2; exit 2 ;;
    esac
done

CONTROL_BUILD="$REPO_ROOT/control/build"
NEW_ALGO_SRC="$REPO_ROOT/control/new_algorithm"
NEW_ALGO_BUILD="$NEW_ALGO_SRC/build"
NEW_ALGO_BIN="$REPO_ROOT/control/build_new_algorithm/hybrid_guidance"
GCS_BUILD="$REPO_ROOT/gcs/build"

BACKUP="$REPO_ROOT/gazeboSim/MAVLink.yaml.sim-backup"
SETTINGS="$REPO_ROOT/setting/MAVLink.yaml"

restore_settings() {
    if [ "$SIM" = "1" ] && [ -f "$BACKUP" ]; then
        cp "$BACKUP" "$SETTINGS"
        echo "[run_new_algorithm] setting/MAVLink.yaml restored."
    fi
}

cleanup() {
    echo
    echo "[run_new_algorithm] shutting down..."
    pkill -f './hybrid_guidance' 2>/dev/null || true
    pkill -f './target_distance' 2>/dev/null || true
    pkill -f './telem_sender' 2>/dev/null || true
    pkill -f './yolo_headless' 2>/dev/null || true
    pkill -f './mavlink_proxy' 2>/dev/null || true
    if [ "$SIM" = "1" ]; then
        pkill -f 'fake_yolo_target' 2>/dev/null || true
        pkill -f 'socat .*sitl_serial' 2>/dev/null || true
    fi
    restore_settings
}
trap cleanup EXIT INT TERM

echo "=== build: control/'s existing pieces (mavlink_proxy, target_distance) ==="
mkdir -p "$CONTROL_BUILD"
(cd "$CONTROL_BUILD" && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. >/dev/null &&
 make -j"$(nproc)" mavlink_proxy target_distance) || exit 1

echo "=== build: control/new_algorithm/hybrid_guidance (standalone CMake project) ==="
mkdir -p "$NEW_ALGO_BUILD"
(cd "$NEW_ALGO_BUILD" && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. >/dev/null &&
 make -j"$(nproc)") || exit 1
[ -x "$NEW_ALGO_BIN" ] || { echo "[run_new_algorithm] build did not produce $NEW_ALGO_BIN" >&2; exit 1; }

if [ "$SIM" = "1" ]; then
    if [ ! -f "$REPO_ROOT/gazeboSim/bridge_sitl.sh" ]; then
        echo "[run_new_algorithm] gazeboSim/ not found - --sim only runs from the simulation setup." >&2
        exit 1
    fi

    echo
    echo "=== sim 0: backing up setting/MAVLink.yaml ==="
    cp "$SETTINGS" "$BACKUP"

    echo
    echo "=== sim 1: PTY bridge to the PC's SITL ==="
    "$REPO_ROOT/gazeboSim/bridge_sitl.sh" || { echo "[run_new_algorithm] bridge failed" >&2; exit 1; }

    echo
    echo "=== sim 2: pointing real.serial.address at the bridge PTY ==="
    sed -i 's|^\( *address: \)/dev/tty[A-Za-z0-9]*|\1/tmp/sitl_serial|' "$SETTINGS"
    grep -A2 '^real:' "$SETTINGS"
fi

echo
echo "=== mavlink_proxy (background) ==="
(cd "$CONTROL_BUILD" && setsid nohup ./mavlink_proxy >/tmp/mavlink_proxy.log 2>&1 </dev/null &)
sleep 3

if [ -d "$GCS_BUILD" ] || [ -f "$REPO_ROOT/gcs/CMakeLists.txt" ]; then
    echo "=== telem_sender (background) ==="
    mkdir -p "$GCS_BUILD"
    (cd "$GCS_BUILD" && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. >/dev/null && make -j"$(nproc)") &&
        (cd "$GCS_BUILD" && setsid nohup ./telem_sender >/tmp/telem_sender.log 2>&1 </dev/null &)
    sleep 1
fi

if [ "$SIM" = "1" ]; then
    if [ "$CAMERA" = "1" ]; then
        echo
        echo "=== sim 3: checking the PC-side camera detector is reachable ==="
        if ! curl -s --max-time 3 http://127.0.0.1:8002/target >/dev/null; then
            echo "[run_new_algorithm] nothing serving /target on 127.0.0.1:8002 - start" >&2
            echo "[run_new_algorithm] gazebo_camera_target.py on the PC and tunnel it here first." >&2
            exit 1
        fi
        curl -s --max-time 3 http://127.0.0.1:8002/target; echo
    else
        echo
        echo "=== sim 3: fake /target responder (NO perception) ==="
        (cd "$REPO_ROOT" && setsid nohup python3 gazeboSim/fake_yolo_target.py --port 8002 \
            >/tmp/fake_yolo.log 2>&1 </dev/null &)
        sleep 2
        curl -s --max-time 2 http://127.0.0.1:8002/target || { echo "[run_new_algorithm] fake responder failed" >&2; exit 1; }
        echo
    fi

    echo
    echo "=== sim 4: target_distance (background) ==="
    (cd "$CONTROL_BUILD" && setsid nohup ./target_distance >/tmp/target_distance.log 2>&1 </dev/null &)
    sleep 3
    tail -3 /tmp/target_distance.log

    echo
    echo "=== sim 5: hybrid_guidance (foreground) - arms + takes off + approaches itself ==="
    echo "[run_new_algorithm] Ctrl+C stops everything and restores setting/MAVLink.yaml."
    cd "$(dirname "$NEW_ALGO_BIN")"
    ./hybrid_guidance
else
    echo
    echo "=== yolo_headless (background) ==="
    (cd "$REPO_ROOT/YOLO_MODEL/cpp" && setsid nohup ./run_yolo_headless.sh >/tmp/yolo_headless.log 2>&1 </dev/null &)
    sleep 1

    YOLO_PORT="$(grep -E '^\s*yolo_port:' "$SETTINGS" | head -1 | sed -E 's/^[^:]*:\s*([0-9]+).*/\1/')"
    YOLO_PORT="${YOLO_PORT:-8002}"
    YOLO_TARGET_URL="http://127.0.0.1:${YOLO_PORT}/target"

    echo "=== 욜로 탐지 대기 (target_distance/hybrid_guidance 아직 미실행, $YOLO_TARGET_URL 폴링) ==="
    while true; do
        body="$(curl -s --max-time 1 "$YOLO_TARGET_URL" 2>/dev/null)"
        if echo "$body" | grep -qE '"confirmed":[[:space:]]*true'; then
            echo "[run_new_algorithm] 타겟 확정 감지 - target_distance/hybrid_guidance 시작."
            break
        fi
        sleep 0.5
    done

    echo "=== target_distance (background) ==="
    (cd "$CONTROL_BUILD" && setsid nohup ./target_distance >/tmp/target_distance.log 2>&1 </dev/null &)
    sleep 1

    echo "=== hybrid_guidance (foreground) - arms + takes off + approaches itself ==="
    cd "$(dirname "$NEW_ALGO_BIN")"
    ./hybrid_guidance
fi
