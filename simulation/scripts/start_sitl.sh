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
case "$SITL_MAVPROXY_MODE" in
  bundled|external) ;;
  *) sim_die "SITL_MAVPROXY_MODE must be bundled or external (got: $SITL_MAVPROXY_MODE)" ;;
esac
if [[ "$SITL_MAVPROXY_MODE" == bundled ]]; then
  mavproxy_path="$(find_mavproxy)" ||
    sim_die 'MAVProxy not found (set MAVPROXY_BIN or provide it in the ArduPilot venv)'
  verify_mavproxy "$mavproxy_path" ||
    sim_die "MAVProxy executable/version/pymavlink verification failed: $mavproxy_path"
  # sim_vehicle.py starts the literal command `mavproxy.py`.
  export PATH="$(dirname -- "$mavproxy_path"):$PATH"
fi
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
  -I "$SITL_INSTANCE" -S "$SITL_SPEEDUP" -N --wipe
  --no-extra-ports --use-dir "$state_dir"
)
if [[ "$SITL_MAVPROXY_MODE" == bundled ]]; then
  cmd+=(
    --out="127.0.0.1:$CONTROL_UDP_PORT"
    --out="127.0.0.1:$TELEMETRY_UDP_PORT"
    --out="127.0.0.1:$GCS_UDP_PORT"
    --mavproxy-args "--non-interactive --default-modules=link --streamrate $MAVPROXY_STREAMRATE --heartbeat-rate 1"
  )
else
  cmd+=(--no-mavproxy)
fi
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

if [[ "$SITL_MAVPROXY_MODE" == bundled ]]; then
  version="$($mavproxy_path --version 2>&1 | awk -F': ' '/MAVProxy Version:/{print $2; exit}')"
  sim_log 'stack=sim_vehicle.py -> ArduCopter SITL + bundled MAVProxy'
else
  sim_log 'stack=sim_vehicle.py -> ArduCopter SITL only; external MAVProxy expected'
fi
sim_log 'vehicle=ArduCopter model=JSON; --wipe resets the simulation EEPROM on every start'
sim_log "Gazebo plugin endpoint=udp:127.0.0.1:$GAZEBO_FDM_PORT"
if [[ "$SITL_MAVPROXY_MODE" == bundled ]]; then
  sim_log "MAVProxy=$version path=$mavproxy_path master=tcp:127.0.0.1:$SITL_MASTER_TCP_PORT"
else
  sim_log "external MAVProxy master=tcp:127.0.0.1:$SITL_MASTER_TCP_PORT"
fi
sim_log "additional SITL ports: SERIAL1=tcp:127.0.0.1:$((5762 + 10 * SITL_INSTANCE)) SERIAL2=tcp:127.0.0.1:$((5763 + 10 * SITL_INSTANCE)) IRLock=udp:127.0.0.1:$((9005 + 10 * SITL_INSTANCE))"
if [[ "$SITL_MAVPROXY_MODE" == bundled ]]; then
  sim_log "MAVProxy outputs: control=udp:127.0.0.1:$CONTROL_UDP_PORT telemetry=udp:127.0.0.1:$TELEMETRY_UDP_PORT GCS=udp:127.0.0.1:$GCS_UDP_PORT"
fi
printf '[simulation] command:'; printf ' %q' "${cmd[@]}"; printf '\n'
run_foreground_tracked sitl "$launcher" "${cmd[@]}"
