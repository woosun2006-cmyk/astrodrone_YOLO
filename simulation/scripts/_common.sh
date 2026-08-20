#!/usr/bin/env bash

# Shared configuration and safety checks for simulation scripts.
# shellcheck shell=bash

SIM_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
SIMULATION_DIR="$(cd -- "$SIM_SCRIPT_DIR/.." && pwd -P)"
DEFAULT_REPO="$(cd -- "$SIMULATION_DIR/.." && pwd -P)"
LOCAL_SIM_ENV="$SIMULATION_DIR/env/simulation.env"

SIM_ENV_KEYS=(
  ARDUPILOT_DIR ARDUPILOT_GAZEBO_DIR ASTRODRONE_REPO SIM_BUILD_DIR SIM_RUNTIME_DIR
  SIM_WORLD SIM_MODEL SIMULATION_PROFILE SIM_IMU_UPDATE_RATE SITL_FRAME SITL_INSTANCE SITL_LOCATION SITL_SPEEDUP SITL_PARAM_FILE SITL_MAVPROXY_MODE
  GAZEBO_HEADLESS GAZEBO_USE_GPU GAZEBO_GPU_ADAPTER GZ_VERBOSITY GAZEBO_FDM_PORT SITL_MASTER_TCP_PORT
  CONTROL_UDP_PORT TELEMETRY_UDP_PORT GCS_UDP_PORT GCS_TELEMETRY_UDP_PORT
  ENABLE_GCS_TELEMETRY_OUTPUT MAVPROXY_BIN MAVPROXY_STREAMRATE
  SMOKE_TELEMETRY_TIMEOUT TELEMETRY_FRESHNESS_SEC ENABLE_CONTROL_OUTPUT ENABLE_GCS_OUTPUT
  MAVPROXY_MASTER_ENDPOINT MAVLINK_AUDIT_TCP_PORT MAVLINK_AUDIT_REPORT MAVLINK_AUDIT_EVENT_FILE
  MAVLINK_AUDIT_READY_TIMEOUT MAVLINK_AUDIT_PYTHON
  CAMERA_SIM_WORLD CAMERA_SIM_MODEL CAMERA_BODY_MODEL CAMERA_LINK CAMERA_SENSOR
)

declare -A _SIM_EXPLICIT=()
declare -A _SIM_EXPLICIT_VALUE=()
for _sim_key in "${SIM_ENV_KEYS[@]}"; do
  if [[ ${!_sim_key+x} ]]; then
    _SIM_EXPLICIT["$_sim_key"]=1
    _SIM_EXPLICIT_VALUE["$_sim_key"]="${!_sim_key}"
  fi
done

if [[ -f "$LOCAL_SIM_ENV" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "$LOCAL_SIM_ENV"
  set +a
fi

for _sim_key in "${!_SIM_EXPLICIT[@]}"; do
  printf -v "$_sim_key" '%s' "${_SIM_EXPLICIT_VALUE[$_sim_key]}"
  export "$_sim_key"
done
unset _sim_key _SIM_EXPLICIT _SIM_EXPLICIT_VALUE

: "${ARDUPILOT_DIR:=/home/hyojin/ardupilot}"
: "${ARDUPILOT_GAZEBO_DIR:=/home/hyojin/ardupilot_gazebo}"
: "${ASTRODRONE_REPO:=$DEFAULT_REPO}"
: "${SIM_BUILD_DIR:=/tmp/astrodrone-control-sim-build}"
: "${SIM_RUNTIME_DIR:=/tmp/astrodrone-simulation-runtime}"
: "${SIM_WORLD:=iris_runway.sdf}"
: "${SIM_MODEL:=iris_with_gimbal}"
: "${SIMULATION_PROFILE:=default}"
: "${SIM_IMU_UPDATE_RATE:=250.0}"
: "${SITL_FRAME:=gazebo-iris}"
: "${SITL_INSTANCE:=0}"
: "${SITL_LOCATION:=}"
: "${SITL_SPEEDUP:=1}"
: "${SITL_PARAM_FILE:=$SIMULATION_DIR/params/gazebo-iris.parm}"
: "${SITL_MAVPROXY_MODE:=bundled}"
: "${GAZEBO_HEADLESS:=0}"
: "${GAZEBO_USE_GPU:=1}"
: "${GAZEBO_GPU_ADAPTER:=}"
: "${GZ_VERBOSITY:=4}"
: "${GAZEBO_FDM_PORT:=9002}"
: "${SITL_MASTER_TCP_PORT:=$((5760 + 10 * SITL_INSTANCE))}"
: "${CONTROL_UDP_PORT:=14550}"
: "${TELEMETRY_UDP_PORT:=14551}"
: "${GCS_UDP_PORT:=14552}"
: "${GCS_TELEMETRY_UDP_PORT:=14553}"
: "${ENABLE_GCS_TELEMETRY_OUTPUT:=0}"
: "${MAVPROXY_BIN:=}"
: "${MAVPROXY_STREAMRATE:=10}"
: "${SMOKE_TELEMETRY_TIMEOUT:=30}"
: "${TELEMETRY_FRESHNESS_SEC:=3}"
: "${ENABLE_CONTROL_OUTPUT:=0}"
: "${ENABLE_GCS_OUTPUT:=0}"
: "${MAVPROXY_MASTER_ENDPOINT:=tcp:127.0.0.1:$SITL_MASTER_TCP_PORT}"
: "${MAVLINK_AUDIT_TCP_PORT:=5770}"
: "${MAVLINK_AUDIT_REPORT:=/tmp/astrodrone-mavlink-audit.json}"
: "${MAVLINK_AUDIT_EVENT_FILE:=}"
: "${MAVLINK_AUDIT_READY_TIMEOUT:=60}"
: "${MAVLINK_AUDIT_PYTHON:=}"

export ARDUPILOT_DIR ARDUPILOT_GAZEBO_DIR ASTRODRONE_REPO SIM_BUILD_DIR SIM_RUNTIME_DIR
export SIM_WORLD SIM_MODEL SIMULATION_PROFILE SIM_IMU_UPDATE_RATE SITL_FRAME SITL_INSTANCE SITL_LOCATION SITL_SPEEDUP SITL_PARAM_FILE SITL_MAVPROXY_MODE
export GAZEBO_HEADLESS GAZEBO_USE_GPU GAZEBO_GPU_ADAPTER GZ_VERBOSITY GAZEBO_FDM_PORT SITL_MASTER_TCP_PORT
export CONTROL_UDP_PORT TELEMETRY_UDP_PORT GCS_UDP_PORT GCS_TELEMETRY_UDP_PORT
export ENABLE_GCS_TELEMETRY_OUTPUT
export MAVPROXY_BIN MAVPROXY_STREAMRATE SMOKE_TELEMETRY_TIMEOUT TELEMETRY_FRESHNESS_SEC
export ENABLE_CONTROL_OUTPUT ENABLE_GCS_OUTPUT MAVPROXY_MASTER_ENDPOINT
export MAVLINK_AUDIT_TCP_PORT MAVLINK_AUDIT_REPORT MAVLINK_AUDIT_EVENT_FILE MAVLINK_AUDIT_READY_TIMEOUT MAVLINK_AUDIT_PYTHON

sim_log() { printf '[simulation] %s\n' "$*"; }
sim_warn() { printf '[simulation] WARNING: %s\n' "$*" >&2; }
sim_die() { printf '[simulation] ERROR: %s\n' "$*" >&2; exit 1; }

require_dir() { [[ -d "$1" ]] || sim_die "directory not found: $1"; }
require_file() { [[ -f "$1" ]] || sim_die "file not found: $1"; }
require_executable() { [[ -x "$1" ]] || sim_die "executable not found: $1"; }
require_command() { command -v "$1" >/dev/null 2>&1 || sim_die "command not found: $1"; }

require_uint() {
  [[ "$2" =~ ^[0-9]+$ ]] || sim_die "$1 must be a non-negative integer (got: $2)"
}

require_port() {
  require_uint "$1" "$2"
  (( 1 <= 10#$2 && 10#$2 <= 65535 )) || sim_die "$1 must be in 1..65535 (got: $2)"
}

require_loopback_endpoint() {
  local label="$1" value="$2"
  [[ "$value" != *'/dev/'* && "$value" != *'tty'* && "$value" != *'serial'* ]] ||
    sim_die "$label must never name a serial or device endpoint: $value"
  [[ "$value" =~ ^(tcp|udp|udpout):127\.0\.0\.1:[0-9]+$ ]] ||
    sim_die "$label must be a loopback tcp/udp endpoint (got: $value)"
}

find_mavproxy() {
  local candidate=''
  if [[ -n "$MAVPROXY_BIN" ]]; then
    candidate="$MAVPROXY_BIN"
  elif command -v mavproxy.py >/dev/null 2>&1; then
    candidate="$(command -v mavproxy.py)"
  else
    local candidates=(
      /home/hyojin/venv-ardupilot/bin/mavproxy.py
      "$ARDUPILOT_DIR/venv/bin/mavproxy.py"
      "$ARDUPILOT_DIR/.venv/bin/mavproxy.py"
      "$ARDUPILOT_DIR/venv-ardupilot/bin/mavproxy.py"
      "$ARDUPILOT_DIR/../venv-ardupilot/bin/mavproxy.py"
    )
    for candidate in "${candidates[@]}"; do
      [[ -x "$candidate" ]] && break
      candidate=''
    done
  fi
  [[ -n "$candidate" && -x "$candidate" ]] || return 1
  realpath -e -- "$candidate"
}

mavproxy_python() {
  local binary="$1" shebang
  IFS= read -r shebang <"$binary" || return 1
  [[ "$shebang" == '#!'* ]] || return 1
  shebang="${shebang#\#!}"
  [[ -x "$shebang" ]] || return 1
  printf '%s\n' "$shebang"
}

verify_mavproxy() {
  local binary="$1" python
  [[ -x "$binary" ]] || return 1
  python="$(mavproxy_python "$binary")" || return 1
  "$binary" --version 2>&1 | grep -q 'MAVProxy Version:' || return 1
  "$python" -c 'import MAVProxy, pymavlink' >/dev/null 2>&1 || return 1
}

resolve_world_file() {
  if [[ "$SIM_WORLD" = /* ]]; then
    printf '%s\n' "$SIM_WORLD"
  elif [[ -f "$SIMULATION_DIR/worlds/$SIM_WORLD" ]]; then
    printf '%s/worlds/%s\n' "$SIMULATION_DIR" "$SIM_WORLD"
  else
    printf '%s/worlds/%s\n' "$ARDUPILOT_GAZEBO_DIR" "$SIM_WORLD"
  fi
}

resolve_model_file() {
  if [[ -f "$SIMULATION_DIR/models/$SIM_MODEL/model.sdf" ]]; then
    printf '%s/models/%s/model.sdf\n' "$SIMULATION_DIR" "$SIM_MODEL"
  else
    printf '%s/models/%s/model.sdf\n' "$ARDUPILOT_GAZEBO_DIR" "$SIM_MODEL"
  fi
}

port_in_use() {
  local port="$1"
  command -v ss >/dev/null 2>&1 || return 1
  ss -H -lntu 2>/dev/null | awk '{print $5}' | grep -Eq "[:.]${port}$"
}

pid_start_time() {
  local pid="$1"
  [[ -r "/proc/$pid/stat" ]] || return 1
  awk '{print $22}' "/proc/$pid/stat" 2>/dev/null
}

run_foreground_tracked() {
  local name="$1" expected="$2"
  shift 2
  mkdir -p -- "$SIM_RUNTIME_DIR"
  local pid_file="$SIM_RUNTIME_DIR/$name.pid"
  [[ ! -e "$pid_file" ]] || sim_die "PID file already exists: $pid_file (run stop_simulation.sh or inspect it)"

  require_command setsid
  setsid "$@" &
  local child=$!
  local started
  started="$(pid_start_time "$child")" || {
    wait "$child" || true
    sim_die "could not record $name process identity"
  }
  printf '%s\t%s\t%s\n' "$child" "$started" "$expected" >"$pid_file"
  sim_log "$name started as PID $child; PID file: $pid_file"

  _sim_forward_signal() {
    kill -TERM -- "-$child" 2>/dev/null || true
  }
  trap _sim_forward_signal INT TERM
  local rc=0
  wait "$child" || rc=$?
  rm -f -- "$pid_file"
  trap - INT TERM
  return "$rc"
}
