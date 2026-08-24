#!/usr/bin/env bash

# Shared path and endpoint contract for Repo A launchers.
# This file is intentionally side-effect free when sourced.

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"

resolve_repo_root() {
  local root
  if [[ -n "${REPO_ROOT:-}" ]]; then
    root="$REPO_ROOT"
  elif root="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null)"; then
    :
  else
    root="$(cd -- "$SCRIPT_DIR/.." && pwd -P)"
  fi
  cd -- "$root" && pwd -P
}

REPO_ROOT="$(resolve_repo_root)"
CONTROL_DIR="${CONTROL_DIR:-$REPO_ROOT/control}"
YOLO_DIR="${YOLO_DIR:-$REPO_ROOT/YOLO_MODEL}"
SETTING_DIR="${SETTING_DIR:-$REPO_ROOT/setting}"
SIMULATION_DIR="${SIMULATION_DIR:-$REPO_ROOT/simulation}"
ASTRODRONE_TARGET="${ASTRODRONE_TARGET:-sitl}"
RUNTIME_SITL_SETTINGS="${RUNTIME_SITL_SETTINGS:-$SETTING_DIR/runtime.sitl.yaml}"
RUNTIME_REAL_SETTINGS="${RUNTIME_REAL_SETTINGS:-$SETTING_DIR/runtime.real.yaml}"

# External projects are dependencies, not repository contents. $HOME keeps
# these defaults portable across WSL users and Jetson deployments.
ARDUPILOT_DIR="${ARDUPILOT_DIR:-${HOME}/ardupilot}"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-${HOME}/ardupilot_gazebo}"
MISSION_PLANNER_DIR="${MISSION_PLANNER_DIR:-${HOME}/MissionPlanner}"
YOLOV5_DIR="${YOLOV5_DIR:-${HOME}/yolov5}"
ARDUPILOT_VENV="${ARDUPILOT_VENV:-${HOME}/venv-ardupilot}"
YOLO_VENV="${YOLO_VENV:-${HOME}/venv-yolov5}"
YOLO_GAZEBO_SCRIPT="${YOLO_GAZEBO_SCRIPT:-$YOLO_DIR/gazebo_yolo.py}"
TENSORRT_ROOT="${TENSORRT_ROOT:-}"
if [[ -z "$TENSORRT_ROOT" && -d "$HOME/opt/TensorRT-8.6.1.6" ]]; then
  TENSORRT_ROOT="$HOME/opt/TensorRT-8.6.1.6"
fi
TENSORRT_LIB_DIR="${TENSORRT_LIB_DIR:-}"
if [[ -z "$TENSORRT_LIB_DIR" && -n "$TENSORRT_ROOT" ]]; then
  case "$(uname -m)" in
    x86_64) TENSORRT_LIB_DIR="$TENSORRT_ROOT/targets/x86_64-linux-gnu/lib" ;;
    aarch64|arm64) TENSORRT_LIB_DIR="$TENSORRT_ROOT/targets/aarch64-linux-gnu/lib" ;;
  esac
fi

# TensorRT engines are target-specific. An explicit YOLO_ENGINE_PATH always
# wins; otherwise select the real/Jetson or WSL SITL engine below.
YOLO_MODEL_PATH="${YOLO_MODEL_PATH:-$YOLO_DIR/best_v5.pt}"
YOLO_ONNX_PATH="${YOLO_ONNX_PATH:-$YOLO_DIR/best_v5.onnx}"
YOLO_REAL_ENGINE_PATH="$YOLO_DIR/0812best.engine"
YOLO_SITL_WSL_ENGINE_PATH="$YOLO_DIR/best_v5_wsl.engine"
YOLO_ARCH="$(uname -m)"
case "$YOLO_ARCH" in
  x86_64)
    YOLO_PLATFORM="${YOLO_PLATFORM:-x86_64}"
    YOLO_LIVE_BUILD_DIR="${YOLO_LIVE_BUILD_DIR:-$REPO_ROOT/build/yolo_live-x86_64}"
    ;;
  aarch64|arm64)
    YOLO_PLATFORM="${YOLO_PLATFORM:-aarch64}"
    YOLO_LIVE_BUILD_DIR="${YOLO_LIVE_BUILD_DIR:-$REPO_ROOT/build/yolo_live-aarch64}"
    ;;
  *)
    YOLO_PLATFORM="${YOLO_PLATFORM:-$YOLO_ARCH}"
    YOLO_LIVE_BUILD_DIR="${YOLO_LIVE_BUILD_DIR:-$REPO_ROOT/build/yolo_live-${YOLO_PLATFORM}}"
    ;;
esac
if [[ -z "${YOLO_ENGINE_PATH:-}" ]]; then
  if [[ "${ASTRODRONE_TARGET:-sitl}" == real ]]; then
    YOLO_ENGINE_PATH="$YOLO_REAL_ENGINE_PATH"
  elif [[ "${ASTRODRONE_TARGET:-sitl}" == sitl && "$YOLO_ARCH" == x86_64 ]]; then
    YOLO_ENGINE_PATH="$YOLO_SITL_WSL_ENGINE_PATH"
  else
    YOLO_ENGINE_PATH="$YOLO_REAL_ENGINE_PATH"
  fi
fi

BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build}"
LOG_DIR="${LOG_DIR:-$REPO_ROOT/logs}"

# Repo A custom simulation assets.
SIM_WORLD="${SIM_WORLD:-$SIMULATION_DIR/worlds/ensamb_iris_runway.sdf}"
SIM_MODEL="${SIM_MODEL:-$SIMULATION_DIR/models/ensamb_with_gimbal}"
CAMERA_MODEL_DIR="${CAMERA_MODEL_DIR:-$SIMULATION_DIR/models/ensamb_with_standoffs}"
TARGET_MODEL_DIR="${TARGET_MODEL_DIR:-$SIMULATION_DIR/models/target_basket}"
CAMERA_TOPIC="${CAMERA_TOPIC:-/world/ensamb_iris_runway/model/ensamb_with_gimbal/model/ensamb_with_standoffs/link/down_camera_link/sensor/down_camera/image}"

# Endpoint contract. FlightMissionApp owns the SITL master connection and
# fans telemetry out to receive-only subscribers.
SITL_MASTER_ENDPOINT="${SITL_MASTER_ENDPOINT:-tcp:127.0.0.1:5760}"
SITL_SERIAL1_ENDPOINT="${SITL_SERIAL1_ENDPOINT:-tcp:127.0.0.1:5762}"
SITL_SERIAL2_ENDPOINT="${SITL_SERIAL2_ENDPOINT:-tcp:127.0.0.1:5763}"
SITL_COMMAND_ENDPOINT="${SITL_COMMAND_ENDPOINT:-$SITL_MASTER_ENDPOINT}"
SITL_TELEMETRY_ENDPOINT="${SITL_TELEMETRY_ENDPOINT:-udp:127.0.0.1:14551}"
GCS_TELEMETRY_ENDPOINT="${GCS_TELEMETRY_ENDPOINT:-udp:127.0.0.1:14553}"
HEALTH_TELEMETRY_ENDPOINT="${HEALTH_TELEMETRY_ENDPOINT:-udp:127.0.0.1:14554}"
MISSION_UPLOAD_ENDPOINT="${MISSION_UPLOAD_ENDPOINT:-$SITL_SERIAL1_ENDPOINT}"
TARGET_UDP_ENDPOINT="${TARGET_UDP_ENDPOINT:-udp:127.0.0.1:15020}"
GCS_TARGET_UDP_ENDPOINT="${GCS_TARGET_UDP_ENDPOINT:-udp:127.0.0.1:15022}"
YOLO_HTTP_ENDPOINT="${YOLO_HTTP_ENDPOINT:-http://127.0.0.1:8002}"
GAZEBO_FDM_ENDPOINT="${GAZEBO_FDM_ENDPOINT:-udp:127.0.0.1:9002}"

export SCRIPT_DIR REPO_ROOT CONTROL_DIR YOLO_DIR SETTING_DIR SIMULATION_DIR ASTRODRONE_TARGET
export RUNTIME_SITL_SETTINGS RUNTIME_REAL_SETTINGS
export ARDUPILOT_DIR ARDUPILOT_GAZEBO_DIR MISSION_PLANNER_DIR YOLOV5_DIR YOLO_GAZEBO_SCRIPT
export ARDUPILOT_VENV YOLO_VENV YOLO_PLATFORM YOLO_LIVE_BUILD_DIR
export YOLO_REAL_ENGINE_PATH YOLO_SITL_WSL_ENGINE_PATH
export TENSORRT_ROOT TENSORRT_LIB_DIR BUILD_DIR LOG_DIR
export SIM_WORLD SIM_MODEL CAMERA_MODEL_DIR TARGET_MODEL_DIR CAMERA_TOPIC
export SITL_MASTER_ENDPOINT SITL_SERIAL1_ENDPOINT SITL_SERIAL2_ENDPOINT
export SITL_COMMAND_ENDPOINT SITL_TELEMETRY_ENDPOINT GCS_TELEMETRY_ENDPOINT HEALTH_TELEMETRY_ENDPOINT
export MISSION_UPLOAD_ENDPOINT TARGET_UDP_ENDPOINT GCS_TARGET_UDP_ENDPOINT YOLO_HTTP_ENDPOINT YOLO_ENGINE_PATH YOLO_ONNX_PATH YOLO_MODEL_PATH GAZEBO_FDM_ENDPOINT

# Compatibility name used by older project tools. New launchers use REPO_ROOT.
REPO_DIR="$REPO_ROOT"
export REPO_DIR

path_exists() { [[ -e "$1" ]]; }
dir_exists() { [[ -d "$1" ]]; }
file_exists() { [[ -f "$1" ]]; }
command_exists() { command -v "$1" >/dev/null 2>&1; }

print_dependency_report() {
  printf 'REPO_ROOT=%s\n' "$REPO_ROOT"
  printf 'CONTROL_DIR=%s\nYOLO_DIR=%s\nSETTING_DIR=%s\nSIMULATION_DIR=%s\n' \
    "$CONTROL_DIR" "$YOLO_DIR" "$SETTING_DIR" "$SIMULATION_DIR"
  printf 'ARDUPILOT_DIR=%s (%s)\n' "$ARDUPILOT_DIR" "$(dir_exists "$ARDUPILOT_DIR" && printf present || printf missing)"
  printf 'ARDUPILOT_GAZEBO_DIR=%s (%s)\n' "$ARDUPILOT_GAZEBO_DIR" "$(dir_exists "$ARDUPILOT_GAZEBO_DIR" && printf present || printf missing)"
  printf 'MISSION_PLANNER_DIR=%s (%s)\n' "$MISSION_PLANNER_DIR" \
    "$(dir_exists "$MISSION_PLANNER_DIR" && printf present || printf missing)"
  printf 'YOLOV5_DIR=%s (%s)\n' "$YOLOV5_DIR" "$(dir_exists "$YOLOV5_DIR" && printf present || printf missing)"
  printf 'BUILD_DIR=%s\nLOG_DIR=%s\n' "$BUILD_DIR" "$LOG_DIR"
  printf 'SIM_WORLD=%s\nSIM_MODEL=%s\nCAMERA_TOPIC=%s\n' "$SIM_WORLD" "$SIM_MODEL" "$CAMERA_TOPIC"
  printf 'SITL master=%s\nSITL SERIAL1=%s\nSITL SERIAL2=%s\n' \
    "$SITL_MASTER_ENDPOINT" "$SITL_SERIAL1_ENDPOINT" "$SITL_SERIAL2_ENDPOINT"
  printf 'FlightMissionApp=%s\ntelemetry=%s\nGCS=%s\nhealth=%s\nmission upload=%s\ntarget UDP=%s\nYOLO HTTP=%s\nYOLO engine=%s\n' \
    "$SITL_COMMAND_ENDPOINT" "$SITL_TELEMETRY_ENDPOINT" \
    "$GCS_TELEMETRY_ENDPOINT" "$HEALTH_TELEMETRY_ENDPOINT" "$MISSION_UPLOAD_ENDPOINT" \
    "$TARGET_UDP_ENDPOINT" "$YOLO_HTTP_ENDPOINT" "$YOLO_ENGINE_PATH"
}
