#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=_common.sh
source "$SCRIPT_DIR/_common.sh"

: "${SMOKE_RUN_ID:=$(date +%Y%m%d_%H%M%S)}"
log_dir="$ASTRODRONE_REPO/simulation/logs"
mkdir -p -- "$log_dir"
prefix="$log_dir/smoke_${SMOKE_RUN_ID}"
audit_report="${prefix}_audit.json"

require_executable "$SIM_BUILD_DIR/simulation-tests/read_only_telemetry"
[[ ! -e "$SIM_RUNTIME_DIR/gazebo.pid" && ! -e "$SIM_RUNTIME_DIR/sitl.pid" &&
   ! -e "$SIM_RUNTIME_DIR/audit.pid" && ! -e "$SIM_RUNTIME_DIR/router.pid" ]] ||
  sim_die 'tracked simulation PID files already exist; stop or inspect the existing run first'

wrappers=()
cleaned=0
cleanup() {
  if [[ "$cleaned" == 0 ]]; then
    "$SCRIPT_DIR/stop_simulation.sh" || true
    cleaned=1
  fi
  for pid in "${wrappers[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

wait_for_port() {
  local port="$1" timeout="$2" label="$3"
  local deadline=$((SECONDS + timeout))
  while (( SECONDS < deadline )); do
    port_in_use "$port" && { sim_log "$label ready on port $port"; return 0; }
    sleep 1
  done
  sim_die "$label did not open port $port within ${timeout}s"
}

wait_for_text() {
  local file="$1" text_value="$2" timeout="$3" label="$4"
  local deadline=$((SECONDS + timeout))
  while (( SECONDS < deadline )); do
    [[ -f "$file" ]] && grep -q "$text_value" "$file" && { sim_log "$label confirmed"; return 0; }
    sleep 1
  done
  sim_die "$label was not observed in $file within ${timeout}s"
}

sim_log "smoke run id=$SMOKE_RUN_ID"
"$SCRIPT_DIR/check_environment.sh" | tee "${prefix}_environment.log"

GAZEBO_HEADLESS=1 DISPLAY= WAYLAND_DISPLAY= "$SCRIPT_DIR/start_gazebo.sh" \
  >"${prefix}_gazebo.log" 2>"${prefix}_gazebo.err.log" &
wrappers+=("$!")
wait_for_port "$GAZEBO_FDM_PORT" 30 Gazebo-plugin
wait_for_text "${prefix}_gazebo.log" 'Enabling camera sensor' 30 Gazebo-sensors

DISPLAY= WAYLAND_DISPLAY= "$SCRIPT_DIR/start_sitl.sh" \
  >"${prefix}_sitl.log" 2>"${prefix}_sitl.err.log" &
wrappers+=("$!")
wait_for_port "$SITL_MASTER_TCP_PORT" 45 ArduCopter-SITL

MAVLINK_AUDIT_REPORT="$audit_report" "$SCRIPT_DIR/start_mavlink_audit.sh" \
  >"${prefix}_audit.log" 2>"${prefix}_audit.err.log" &
wrappers+=("$!")
wait_for_port "$MAVLINK_AUDIT_TCP_PORT" 20 MAVLink-audit

MAVPROXY_MASTER_ENDPOINT="tcp:127.0.0.1:$MAVLINK_AUDIT_TCP_PORT" \
ENABLE_CONTROL_OUTPUT=0 ENABLE_GCS_OUTPUT=0 "$SCRIPT_DIR/start_router.sh" \
  >"${prefix}_mavproxy.log" 2>"${prefix}_mavproxy.err.log" &
wrappers+=("$!")
wait_for_text "${prefix}_audit.log" 'SITL_CONNECTED' 30 MAVProxy-to-SITL
wait_for_text /tmp/ArduCopter.log 'JSON received:' 30 plugin-SITL-JSON

"$SCRIPT_DIR/run_control_shadow.sh" | tee "${prefix}_telemetry.log"

"$SCRIPT_DIR/stop_simulation.sh"
cleaned=1
for pid in "${wrappers[@]}"; do wait "$pid" 2>/dev/null || true; done

mavproxy_path="$(find_mavproxy)"
python="$(mavproxy_python "$mavproxy_path")"
"$python" -c '
import json, sys
report=json.load(open(sys.argv[1], encoding="utf-8"))
assert report["vehicle_affecting_command_count"] == 0, report
counts=report["mavproxy_to_sitl"]["message_counts"]
assert any(key.startswith("66:REQUEST_DATA_STREAM") for key in counts), counts
print("PACKET_AUDIT vehicle_affecting_command_count=0 REQUEST_DATA_STREAM=observed")
' "$audit_report"

remaining_ports="$(ss -H -lntup 2>/dev/null | awk '{print $5}' | grep -E ":($GAZEBO_FDM_PORT|$SITL_MASTER_TCP_PORT|$((SITL_MASTER_TCP_PORT + 2))|$((SITL_MASTER_TCP_PORT + 3))|$MAVLINK_AUDIT_TCP_PORT|$TELEMETRY_UDP_PORT|$CONTROL_UDP_PORT|$GCS_UDP_PORT|9005)$" || true)"
[[ -z "$remaining_ports" ]] || sim_die "simulation ports remain after cleanup: $remaining_ports"
remaining_pid_files="$(find "$SIM_RUNTIME_DIR" -maxdepth 1 -type f -name '*.pid' -print 2>/dev/null || true)"
[[ -z "$remaining_pid_files" ]] || sim_die "tracked PID files remain after cleanup: $remaining_pid_files"
remaining_processes="$(ps -eo args= | grep -E "${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py|${ARDUPILOT_GAZEBO_DIR}/.*gz sim|mavlink_audit_proxy.py|${mavproxy_path}" | grep -v grep || true)"
[[ -z "$remaining_processes" ]] || sim_die "simulation processes remain after cleanup: $remaining_processes"
sim_log 'CLEANUP PASS: tracked processes/PID files and relevant ports remaining=0'
sim_log "SMOKE PASS logs=${prefix}_*.log audit=$audit_report"
trap - EXIT INT TERM
