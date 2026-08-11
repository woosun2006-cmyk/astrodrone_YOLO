#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=_common.sh
source "$SCRIPT_DIR/_common.sh"

require_dir "$ARDUPILOT_GAZEBO_DIR"
require_command gz
gz sim --version >/dev/null 2>&1 || sim_die 'installed gz command does not provide gz sim'
plugin="$ARDUPILOT_GAZEBO_DIR/build/libArduPilotPlugin.so"
world_file="$(resolve_world_file)"
model_file="$(resolve_model_file)"
require_file "$plugin"
require_file "$world_file"
require_file "$model_file"
ldd "$plugin" 2>&1 | grep -q 'not found' && sim_die "plugin has unresolved shared libraries: $plugin"
grep -q "<uri>model://$SIM_MODEL</uri>" "$world_file" ||
  sim_die "world does not include selected model '$SIM_MODEL': $world_file"
grep -q "<fdm_port_in>$GAZEBO_FDM_PORT</fdm_port_in>" "$model_file" ||
  sim_die "model does not use configured plugin port $GAZEBO_FDM_PORT: $model_file"
port_in_use "$GAZEBO_FDM_PORT" && sim_die "Gazebo plugin UDP port is already in use: $GAZEBO_FDM_PORT"

export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$ARDUPILOT_GAZEBO_DIR/models:$ARDUPILOT_GAZEBO_DIR/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

cmd=(gz sim -v "$GZ_VERBOSITY" -r)
case "$GAZEBO_HEADLESS" in
  0) ;;
  1) cmd+=(-s) ;;
  *) sim_die "GAZEBO_HEADLESS must be 0 or 1 (got: $GAZEBO_HEADLESS)" ;;
esac
cmd+=("$world_file")

sim_log "world=$world_file model=$SIM_MODEL FDM=udp:127.0.0.1:$GAZEBO_FDM_PORT"
sim_log "GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH"
sim_log "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
printf '[simulation] command:'; printf ' %q' "${cmd[@]}"; printf '\n'
run_foreground_tracked gazebo "$world_file" "${cmd[@]}"
