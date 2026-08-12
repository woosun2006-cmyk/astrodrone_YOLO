#!/usr/bin/env bash
# PC-side bring-up for the Gazebo/SITL harness, in dependency order.
#
# Runs on the machine hosting Gazebo (WSL Ubuntu here), NOT on the Jetson.
# The Jetson side is scripts/sim_mission_no_camera.sh / sim_mission_camera.sh.
#
# World/camera selection: prefer the repo's own simulation/ assets when they
# are present on the checked-out branch, and only fall back to ardupilot_gazebo's
# stock iris_runway otherwise. This matters for perception, not just cosmetics:
#
#   simulation/models/ensamb_with_standoffs has a FIXED downward camera -
#     <sensor name='down_camera'> carries <pose>0 0 0 0 1.5708 0</pose> and hangs
#     off a fixed joint, so it always looks straight down.
#   ardupilot_gazebo's stock iris_with_gimbal camera sits on a 3-axis gimbal whose
#     default position is 0 rad = pointing FORWARD. ArduPilotPlugin only publishes
#     /gimbal/cmd_* when it sees non-zero PWM on channels 8/9/10, so in a plain
#     SITL run those topics carry zero messages and the camera stares at the
#     horizon - the detector then never sees a target on the ground.
#     (simulation/reports/2026-08-11-gimbal-direction-investigation.md measured
#     exactly this: optical axis body +X by default, body -Z only after a
#     commanded pitch of +90 deg.)
#
# Both cameras carry GstCameraPlugin on udp 5600, so gazebo_camera_target.py
# works unchanged either way.
#
# simulation/ is only ever READ here - never modified.
#
# Startup order matters:
#   Gazebo first  - ArduPilotPlugin binds the JSON FDM port (udp 9002); a SITL
#                   started before it, or left running across a Gazebo world
#                   reset, loops "No JSON sensor message received, resending
#                   servos" forever and never produces valid state.
#   SITL second   - connects the FDM link and opens SERIAL0 (tcp 5760).
#   MAVProxy last - SITL blocks its own startup until a SERIAL0 client
#                   connects, and only then opens SERIAL1/5762, which is the
#                   port the Jetson bridge consumes.
set -u

AP="${ARDUPILOT_DIR:-$HOME/ardupilot}"
AG="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
VENV="${ARDUPILOT_VENV:-$HOME/venv-ardupilot}"
RUNTIME="${SIM_RUNTIME_DIR:-$HOME/astrodrone-sim/runtime}"
SPAWN_TARGET="${SPAWN_TARGET:-1}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd -P)"
SIM_DIR="$REPO_ROOT/simulation"

if [ -z "${SIM_WORLD:-}" ] && [ -f "$SIM_DIR/worlds/ensamb_iris_runway.sdf" ]; then
  SIM_WORLD="$SIM_DIR/worlds/ensamb_iris_runway.sdf"
  SIM_WORLD_NAME="ensamb_iris_runway"
  echo "using the repo's simulation/ world (fixed down camera)"
else
  SIM_WORLD="${SIM_WORLD:-$AG/worlds/iris_runway.sdf}"
  SIM_WORLD_NAME="${SIM_WORLD_NAME:-iris_runway}"
  echo "using ardupilot_gazebo's stock world (gimbal camera points FORWARD by"
  echo "default - run: gz topic -t /gimbal/cmd_pitch -m gz.msgs.Double -p 'data: 1.5708')"
fi
WORLD="$SIM_WORLD"
WORLD_NAME="$SIM_WORLD_NAME"

for p in "$AP/build/sitl/bin/arducopter" "$AG/build/libArduPilotPlugin.so" "$VENV/bin/mavproxy.py"; do
  [ -e "$p" ] || { echo "missing prerequisite: $p" >&2; exit 1; }
done
[ -f "$WORLD" ] || { echo "world not found: $WORLD" >&2; exit 1; }
mkdir -p "$RUNTIME"

echo "world: $WORLD"

echo
echo "=== stopping anything already running ==="
pkill -f 'mavproxy.py --master=tcp:127.0.0.1:5760' 2>/dev/null || true
pkill -f 'sim_vehicle.py' 2>/dev/null || true
pkill -f 'build/sitl/bin/arducopter' 2>/dev/null || true
pkill -f 'gz-sim' 2>/dev/null || true
sleep 3
pkill -9 -f 'build/sitl/bin/arducopter' 2>/dev/null || true
pkill -9 -f 'gz-sim' 2>/dev/null || true
# a new SITL started while the old one still holds 5760 dies with
# "bind failed on port 5760 - Address already in use"
for _ in $(seq 1 30); do
  ss -lnt 2>/dev/null | grep -q ':5760' || break
  sleep 1
done
rm -f "$RUNTIME"/*.pid
echo "stopped."

echo
echo "=== 1/4 Gazebo ==="
export GZ_SIM_SYSTEM_PLUGIN_PATH="$AG/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/models:$SIM_DIR/models:$SIM_DIR/worlds:$AG/models:$AG/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
setsid nohup gz sim -v 4 -r "$WORLD" >"$HOME/gazebo_run.log" 2>&1 </dev/null &
disown
for i in $(seq 1 90); do
  ss -lun 2>/dev/null | grep -q ':9002' && { echo "FDM port 9002 bound after ${i}s"; break; }
  sleep 1
done
ss -lun 2>/dev/null | grep -q ':9002' || { echo "Gazebo did not bind 9002 - see ~/gazebo_run.log" >&2; exit 1; }

if [ "$SPAWN_TARGET" = "1" ]; then
  echo
  echo "=== 1b/4 spawning target_basket ==="
  "$SCRIPT_DIR/spawn_target.sh" "$WORLD_NAME" 8 0 || echo "(spawn failed - camera detection will find nothing)"
fi

echo
echo "=== 2/4 ArduCopter SITL ==="
setsid nohup python3 "$AP/Tools/autotest/sim_vehicle.py" \
  -v ArduCopter -f gazebo-iris --model JSON -I 0 -S 1 -N \
  --no-mavproxy --use-dir "$RUNTIME/sitl-0" \
  >"$HOME/sitl_run.log" 2>&1 </dev/null &
disown
for i in $(seq 1 90); do
  ss -lnt 2>/dev/null | grep -q ':5760' && { echo "SERIAL0 5760 listening after ${i}s"; break; }
  sleep 1
done

echo
echo "=== 3/4 MAVProxy (also unblocks SERIAL1/5762 for the Jetson) ==="
setsid nohup "$VENV/bin/python" "$VENV/bin/mavproxy.py" \
  --master=tcp:127.0.0.1:5760 \
  --out=udpout:127.0.0.1:14551 \
  --streamrate=4 --heartbeat-rate=1 \
  --default-modules=link --non-interactive --no-state \
  >"$HOME/router_run.log" 2>&1 </dev/null &
disown
sleep 20

echo
echo "=== 4/4 status ==="
ss -lnt 2>/dev/null | grep -E ':(5760|5762|5763)' || echo "WARNING: SITL ports missing"
ss -lun 2>/dev/null | grep ':9002' || echo "WARNING: FDM port missing"
echo
tail -3 "$HOME/router_run.log" 2>/dev/null
echo
echo "--- image topics in this world ---"
gz topic -l 2>/dev/null | grep -E 'image$' || echo "(none yet)"
