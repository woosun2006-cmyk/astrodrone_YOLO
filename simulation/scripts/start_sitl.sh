#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=_common.sh
source "$SCRIPT_DIR/_common.sh"

require_dir "$ARDUPILOT_DIR"
launcher="$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py"
binary="$ARDUPILOT_DIR/build/sitl/bin/arducopter"
require_executable "$launcher"
require_executable "$binary"
require_uint SITL_INSTANCE "$SITL_INSTANCE"
[[ "$SITL_SPEEDUP" =~ ^[0-9]+([.][0-9]+)?$ ]] || sim_die "SITL_SPEEDUP must be positive numeric (got: $SITL_SPEEDUP)"
awk -v value="$SITL_SPEEDUP" 'BEGIN {exit !(value > 0)}' || sim_die 'SITL_SPEEDUP must be greater than zero'
[[ "$SITL_FRAME" == gazebo-* ]] || sim_die "SITL_FRAME must be an ArduPilot Gazebo frame (got: $SITL_FRAME)"
[[ "$SITL_MASTER_TCP_PORT" == "$((5760 + 10 * SITL_INSTANCE))" ]] ||
  sim_die "SITL_MASTER_TCP_PORT must be 5760 + 10*instance ($((5760 + 10 * SITL_INSTANCE)))"
for endpoint in \
  "tcp:127.0.0.1:$SITL_MASTER_TCP_PORT" \
  "udp:127.0.0.1:$TELEMETRY_UDP_PORT"; do
  require_loopback_endpoint simulation "$endpoint"
done
for required_port in \
  "$SITL_MASTER_TCP_PORT" \
  "$((5762 + 10 * SITL_INSTANCE))" \
  "$((5763 + 10 * SITL_INSTANCE))" \
  "$((9005 + 10 * SITL_INSTANCE))"; do
  port_in_use "$required_port" && sim_die "SITL-required port is already in use: $required_port"
done

state_dir="$SIM_RUNTIME_DIR/sitl-$SITL_INSTANCE"
mkdir -p -- "$state_dir"
cmd=(
  "$launcher" -v ArduCopter -f "$SITL_FRAME" --model JSON
  -I "$SITL_INSTANCE" -S "$SITL_SPEEDUP" -N
  --no-mavproxy --no-extra-ports --use-dir "$state_dir"
)
if [[ -n "$SITL_LOCATION" ]]; then
  grep -Eq "^${SITL_LOCATION}[[:space:]]*=" "$ARDUPILOT_DIR/Tools/autotest/locations.txt" ||
    sim_die "SITL_LOCATION is not present in ArduPilot locations.txt: $SITL_LOCATION"
  cmd+=(-L "$SITL_LOCATION")
fi
if [[ -n "$SITL_PARAM_FILE" ]]; then
  param_real="$(realpath -e -- "$SITL_PARAM_FILE" 2>/dev/null)" || sim_die "parameter file not found: $SITL_PARAM_FILE"
  params_root="$(realpath -e -- "$SCRIPT_DIR/../params")"
  [[ "$param_real" == "$params_root/"* ]] || sim_die 'SITL_PARAM_FILE must be under simulation/params'
  if grep -Eiq '^[[:space:]]*(BRD_|SERIAL|CAN_|GPS1_TYPE|GPS2_TYPE)' "$param_real"; then
    sim_die "hardware/serial parameters are forbidden in simulation parameter file: $param_real"
  fi
  cmd+=(--add-param-file "$param_real")
fi

sim_log 'vehicle=ArduCopter model=JSON; --wipe is not used and existing SITL state is not deleted'
sim_log "Gazebo plugin endpoint=udp:127.0.0.1:$GAZEBO_FDM_PORT"
sim_log "router source=tcp:127.0.0.1:$SITL_MASTER_TCP_PORT"
sim_log "additional SITL ports: SERIAL1=tcp:127.0.0.1:$((5762 + 10 * SITL_INSTANCE)) SERIAL2=tcp:127.0.0.1:$((5763 + 10 * SITL_INSTANCE)) IRLock=udp:127.0.0.1:$((9005 + 10 * SITL_INSTANCE))"
sim_log "required MAVLink output after MAVProxy: telemetry=udp:127.0.0.1:$TELEMETRY_UDP_PORT"
sim_log "optional outputs: control_enabled=$ENABLE_CONTROL_OUTPUT GCS_enabled=$ENABLE_GCS_OUTPUT"
printf '[simulation] command:'; printf ' %q' "${cmd[@]}"; printf '\n'
run_foreground_tracked sitl "$launcher" "${cmd[@]}"
