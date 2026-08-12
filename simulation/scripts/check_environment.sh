#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=_common.sh
source "$SCRIPT_DIR/_common.sh"

errors=0
warnings=0
pass() { printf '%-27s %-8s %s\n' "$1" PASS "$2"; }
warn() { printf '%-27s %-8s %s\n' "$1" WARN "$2"; warnings=$((warnings + 1)); }
fail() { printf '%-27s %-8s %s\n' "$1" FAIL "$2"; errors=$((errors + 1)); }

printf '%-27s %-8s %s\n' CHECK RESULT DETAIL
printf '%-27s %-8s %s\n' '---------------------------' '--------' '------'

if grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null; then
  pass WSL "$(uname -r)"
else
  fail WSL 'not running under WSL'
fi

if [[ -d "$ARDUPILOT_DIR/.git" ]]; then
  ap_commit="$(git -C "$ARDUPILOT_DIR" rev-parse HEAD 2>/dev/null || printf unknown)"
  pass ArduPilot "$ARDUPILOT_DIR @ $ap_commit"
else
  fail ArduPilot "Git directory missing: $ARDUPILOT_DIR"
fi

if [[ -d "$ARDUPILOT_GAZEBO_DIR/.git" ]]; then
  ag_commit="$(git -C "$ARDUPILOT_GAZEBO_DIR" rev-parse HEAD 2>/dev/null || printf unknown)"
  pass ardupilot_gazebo "$ARDUPILOT_GAZEBO_DIR @ $ag_commit"
else
  fail ardupilot_gazebo "Git directory missing: $ARDUPILOT_GAZEBO_DIR"
fi

gazebo_command=''
gazebo_version=''
if command -v gz >/dev/null 2>&1 && gz sim --version >/dev/null 2>&1; then
  gazebo_command='gz sim'
  gazebo_version="$(gz sim --version 2>&1 | head -1)"
elif command -v gazebo >/dev/null 2>&1; then
  gazebo_command='gazebo'
  gazebo_version="$(gazebo --version 2>&1 | head -1)"
elif command -v ign >/dev/null 2>&1 && ign gazebo --version >/dev/null 2>&1; then
  gazebo_command='ign gazebo'
  gazebo_version="$(ign gazebo --version 2>&1 | head -1)"
fi
if [[ "$gazebo_command" == 'gz sim' ]]; then
  pass Gazebo "$gazebo_command $gazebo_version"
elif [[ -n "$gazebo_command" ]]; then
  fail Gazebo "$gazebo_command $gazebo_version is not the command used by this installed plugin"
else
  fail Gazebo 'none of gz sim, gazebo, or ign gazebo is available'
fi

plugin="$ARDUPILOT_GAZEBO_DIR/build/libArduPilotPlugin.so"
if [[ -f "$plugin" ]]; then
  if ldd "$plugin" 2>&1 | grep -q 'not found'; then
    fail 'plugin library' "$plugin has unresolved shared libraries"
  else
    plugin_sim="$(ldd "$plugin" 2>/dev/null | awk '/libgz-sim\.so/{print $1; exit}')"
    pass 'plugin library' "$plugin (${plugin_sim:-Gazebo ABI unknown})"
  fi
else
  fail 'plugin library' "missing: $plugin"
fi

effective_plugin_path="$ARDUPILOT_GAZEBO_DIR/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
effective_resource_path="$SIMULATION_DIR/models:$SIMULATION_DIR/worlds:$ARDUPILOT_GAZEBO_DIR/models:$ARDUPILOT_GAZEBO_DIR/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
pass GZ_SIM_SYSTEM_PLUGIN_PATH "current='${GZ_SIM_SYSTEM_PLUGIN_PATH:-<unset>}' effective='$effective_plugin_path'"
pass GZ_SIM_RESOURCE_PATH "current='${GZ_SIM_RESOURCE_PATH:-<unset>}' effective='$effective_resource_path'"

world_file="$(resolve_world_file)"
model_file="$(resolve_model_file)"
[[ -f "$world_file" ]] && pass world "$world_file" || fail world "missing: $world_file"
[[ -f "$model_file" ]] && pass model "$model_file" || fail model "missing: $model_file"
if [[ -f "$model_file" ]] && grep -q '<fdm_port_in>9002</fdm_port_in>' "$model_file"; then
  [[ "$GAZEBO_FDM_PORT" == 9002 ]] && pass 'Gazebo FDM endpoint' 'udp:127.0.0.1:9002' ||
    fail 'Gazebo FDM endpoint' "model is fixed at 9002 but GAZEBO_FDM_PORT=$GAZEBO_FDM_PORT"
else
  fail 'Gazebo FDM endpoint' 'selected model does not declare the expected 9002 plugin port'
fi

sitl_launcher="$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py"
sitl_binary="$ARDUPILOT_DIR/build/sitl/bin/arducopter"
[[ -x "$sitl_launcher" ]] && pass sim_vehicle.py "$sitl_launcher" || fail sim_vehicle.py "missing/not executable: $sitl_launcher"
[[ -x "$sitl_binary" ]] && pass ArduCopter "$sitl_binary" || fail ArduCopter "missing/not executable: $sitl_binary"

python_detail="$(python3 --version 2>&1 || true) @ $(command -v python3 2>/dev/null || printf missing)"
command -v python3 >/dev/null 2>&1 && pass Python "$python_detail" || fail Python "$python_detail"
for tool in cmake ninja gcc g++; do
  if command -v "$tool" >/dev/null 2>&1; then
    version="$($tool --version 2>&1 | head -1)"
    pass "$tool" "$version"
  else
    fail "$tool" 'not installed'
  fi
done

if mavproxy_path="$(find_mavproxy)" && verify_mavproxy "$mavproxy_path"; then
  mavproxy_version="$($mavproxy_path --version 2>&1 | awk -F': ' '/MAVProxy Version:/{print $2; exit}')"
  mavproxy_py="$(mavproxy_python "$mavproxy_path")"
  mavproxy_prefix="$($mavproxy_py -c 'import sys; print(sys.prefix)' 2>/dev/null)"
  pymavlink_version="$($mavproxy_py -c 'import importlib.metadata as m; print(m.version("pymavlink"))' 2>/dev/null)"
  pass MAVProxy 'FOUND'
  pass 'MAVProxy path' "$mavproxy_path"
  pass 'MAVProxy version' "$mavproxy_version"
  pass 'MAVProxy venv' "$mavproxy_prefix (python=$mavproxy_py, pymavlink=$pymavlink_version)"
else
  fail MAVProxy 'NOT FOUND (checked MAVPROXY_BIN, PATH, and known ArduPilot venvs)'
fi
if command -v mavlink-routerd >/dev/null 2>&1; then
  pass mavlink-router "INSTALLED (optional): $(command -v mavlink-routerd)"
else
  pass mavlink-router 'NOT INSTALLED (optional)'
fi

endpoint_values=(
  "tcp:127.0.0.1:$SITL_MASTER_TCP_PORT"
  "tcp:127.0.0.1:$((5762 + 10 * SITL_INSTANCE))"
  "tcp:127.0.0.1:$((5763 + 10 * SITL_INSTANCE))"
  "udp:127.0.0.1:$((9005 + 10 * SITL_INSTANCE))"
  "udp:127.0.0.1:$TELEMETRY_UDP_PORT"
)
endpoint_labels=('SITL router source' 'SITL SERIAL1' 'SITL SERIAL2' 'SITL IRLock' 'read-only telemetry')
declare -A seen_ports=()
for i in "${!endpoint_values[@]}"; do
  endpoint="${endpoint_values[$i]}"
  label="${endpoint_labels[$i]}"
  if require_loopback_endpoint "$label" "$endpoint" 2>/dev/null; then
    port="${endpoint##*:}"
    if [[ ${seen_ports[$port]+x} ]]; then
      fail "$label endpoint" "duplicate port $port"
    elif port_in_use "$port"; then
      warn "$label endpoint" "$endpoint is already in use"
    else
      pass "$label endpoint" "$endpoint"
    fi
    seen_ports[$port]=1
  else
    fail "$label endpoint" "$endpoint is not loopback-only"
  fi
done
if [[ "$ENABLE_CONTROL_OUTPUT" == 1 ]]; then
  pass 'control endpoint' "ENABLED udp:127.0.0.1:$CONTROL_UDP_PORT"
else
  pass 'control endpoint' "DISABLED (optional) udp:127.0.0.1:$CONTROL_UDP_PORT"
fi
if [[ "$ENABLE_GCS_OUTPUT" == 1 ]]; then
  pass 'GCS endpoint' "ENABLED udp:127.0.0.1:$GCS_UDP_PORT"
else
  pass 'GCS endpoint' "DISABLED (optional) udp:127.0.0.1:$GCS_UDP_PORT"
fi
if port_in_use "$GAZEBO_FDM_PORT"; then
  warn 'Gazebo plugin port' "udp:127.0.0.1:$GAZEBO_FDM_PORT is already in use"
else
  pass 'Gazebo plugin port' "udp:127.0.0.1:$GAZEBO_FDM_PORT is free"
fi

prod_serial="$(awk '/^[[:space:]]*address:[[:space:]]*\/dev\//{print $2; exit}' "$ASTRODRONE_REPO/setting/MAVLink.yaml" 2>/dev/null || true)"
if [[ -n "$prod_serial" ]]; then
  pass 'serial isolation' "production-only $prod_serial exists in settings; simulation scripts never select it"
else
  pass 'serial isolation' 'no /dev serial endpoint selected by simulation configuration'
fi
configured_sitl="$(awk '/^[[:space:]]*address:[[:space:]]*(tcp|udp):/{print $2}' "$ASTRODRONE_REPO/setting/MAVLink.yaml" 2>/dev/null | tr '\n' ' ')"
pass 'repository SITL config' "declares ${configured_sitl:-<none>}; scripts use dedicated loopback endpoints above"

printf '\nSummary: %d error(s), %d warning(s).\n' "$errors" "$warnings"
if (( errors > 0 )); then
  exit 1
fi
