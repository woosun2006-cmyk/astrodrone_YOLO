#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=_common.sh
source "$SCRIPT_DIR/_common.sh"

# This simulation-only executable contains receive/connect/bind calls but no
# network send call and no MAVLink command packer. The vehicle-affecting control
# executable is intentionally not reachable from this wrapper.
shadow_executable="$SIM_BUILD_DIR/simulation-tests/read_only_telemetry"
require_executable "$shadow_executable"
endpoint="udp:127.0.0.1:$TELEMETRY_UDP_PORT"
require_loopback_endpoint telemetry "$endpoint"

sitl_pid_file="$SIM_RUNTIME_DIR/sitl.pid"
[[ -f "$sitl_pid_file" ]] || sim_die "no simulation SITL PID file found: $sitl_pid_file"
IFS=$'\t' read -r sitl_pid sitl_started sitl_expected <"$sitl_pid_file"
[[ -r "/proc/$sitl_pid/stat" ]] || sim_die 'recorded SITL process is not running'
[[ "$(pid_start_time "$sitl_pid")" == "$sitl_started" ]] || sim_die 'SITL PID was reused; refusing telemetry connection'
sitl_cmdline="$(tr '\0' ' ' <"/proc/$sitl_pid/cmdline")"
[[ "$sitl_cmdline" == *sim_vehicle.py* && "$sitl_cmdline" == *ArduCopter* ]] ||
  sim_die "recorded process is not the tracked ArduCopter SITL launcher: $sitl_cmdline"

cmd=("$shadow_executable" --address "$endpoint" --timeout "$SMOKE_TELEMETRY_TIMEOUT" \
  --freshness "$TELEMETRY_FRESHNESS_SEC")
sim_log 'safe shadow executable=read_only_telemetry (no message-interval, mode, arm, takeoff, setpoint, land, RC, mission, or parameter messages)'
sim_log 'control main is never invoked by this script'
printf '[simulation] command:'; printf ' %q' "${cmd[@]}"; printf '\n'
"${cmd[@]}"
