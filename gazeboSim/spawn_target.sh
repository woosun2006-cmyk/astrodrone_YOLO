#!/usr/bin/env bash
# Spawn gazeboSim/models/target_basket into an already-running Gazebo world,
# so the simulated camera has something for gazebo_camera_target.py to detect.
#
# Spawning into the live world rather than shipping a modified world file
# keeps this independent of which world is running (the repo's
# simulation/worlds/ensamb_iris_runway.sdf, or ardupilot_gazebo's stock
# iris_runway.sdf) and leaves simulation/ untouched.
#
# Usage: spawn_target.sh [world] [north_m] [east_m]
#   world defaults to iris_runway; north/east are metres from the world origin,
#   which is where ArduPilot SITL puts home.
set -eu

WORLD="${1:-iris_runway}"
NORTH="${2:-8}"
EAST="${3:-0}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

# Gazebo's x is north/forward and y is east/right in the ENU frame these
# worlds use, so the arguments map straight onto pose x/y.
SDF_FILE="$SCRIPT_DIR/models/target_basket/model.sdf"
[ -f "$SDF_FILE" ] || { echo "target_basket model missing: $SDF_FILE" >&2; exit 1; }

echo "spawning target_basket into world '$WORLD' at x=$NORTH y=$EAST"

# Remove a previous instance first so re-running is idempotent; a failure here
# just means it was not there yet.
gz service -s "/world/${WORLD}/remove" \
  --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 \
  --req "name: 'target_basket', type: MODEL" >/dev/null 2>&1 || true

REQ="sdf_filename: '${SDF_FILE}', name: 'target_basket', pose: {position: {x: ${NORTH}, y: ${EAST}, z: 0}}"

gz service -s "/world/${WORLD}/create" \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 8000 \
  --req "$REQ"

echo
echo "models now in the world:"
gz model --list 2>/dev/null | head -20 || true
