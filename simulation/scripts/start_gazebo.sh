#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=_common.sh
source "$SCRIPT_DIR/_common.sh"

# Gazebo server-only mode must not inherit a desktop display session. Camera
# sensors still render through the server's rendering system without a GUI
# client, so only the GUI display variables are removed here.
if [[ "$GAZEBO_HEADLESS" == 1 ]]; then
  unset DISPLAY WAYLAND_DISPLAY
fi

if [[ "$GAZEBO_USE_GPU" == 1 && -e /dev/dxg ]]; then
  # WSL2 exposes the Windows GPU through Mesa's D3D12 Gallium driver. Without
  # this selection, headless EGL falls back to llvmpipe/swrast even though
  # CUDA and nvidia-smi are available.
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
  if [[ -z "${MESA_D3D12_DEFAULT_ADAPTER_NAME:-}" ]]; then
    if [[ -n "$GAZEBO_GPU_ADAPTER" ]]; then
      detected_gpu="$GAZEBO_GPU_ADAPTER"
    elif command -v nvidia-smi >/dev/null 2>&1; then
      detected_gpu="$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null |
        awk 'NR == 1 {sub(/[[:space:]]+$/, ""); print; exit}' || true)"
    else
      detected_gpu=""
    fi
    [[ -z "$detected_gpu" ]] || export MESA_D3D12_DEFAULT_ADAPTER_NAME="$detected_gpu"
  fi
  sim_log "GPU rendering enabled: GALLIUM_DRIVER=$GALLIUM_DRIVER adapter=${MESA_D3D12_DEFAULT_ADAPTER_NAME:-auto}"
fi

require_dir "$ARDUPILOT_GAZEBO_DIR"
require_command gz
gz sim --version >/dev/null 2>&1 || sim_die 'installed gz command does not provide gz sim'
plugin="$ARDUPILOT_GAZEBO_DIR/build/libArduPilotPlugin.so"
world_file="$(resolve_world_file)"
model_file="$(resolve_model_file)"
require_file "$plugin"
require_file "$world_file"

# The control-validation profile keeps the same world, model names, custom
# STL, camera topic, and GstCameraPlugin. It only creates a disposable runtime
# copy of the two custom model directories, lowers the IMU production rate,
# and disables Gazebo/ArduPilot lockstep for wall-clock transport diagnostics.
# This avoids changing the source model used for real-fidelity runs.
runtime_model_root=''
if [[ "$SIMULATION_PROFILE" == control-validation ]]; then
  runtime_model_root="$SIM_RUNTIME_DIR/control-validation/models"
  rm -rf -- "$runtime_model_root/ensamb_with_gimbal" "$runtime_model_root/ensamb_with_standoffs"
  mkdir -p -- "$runtime_model_root"
  cp -a -- "$SIMULATION_DIR/models/ensamb_with_gimbal" "$runtime_model_root/"
  cp -a -- "$SIMULATION_DIR/models/ensamb_with_standoffs" "$runtime_model_root/"
  validation_model="$runtime_model_root/ensamb_with_standoffs/model.sdf"
  sed -E -i "s#<update_rate>1000([.]0)?</update_rate>#<update_rate>${SIM_IMU_UPDATE_RATE}</update_rate>#" "$validation_model"
  validation_gimbal="$runtime_model_root/ensamb_with_gimbal/model.sdf"
  sed -E -i 's#<lock_step>1</lock_step>#<lock_step>0</lock_step>#' "$validation_gimbal"
  model_file="$runtime_model_root/ensamb_with_gimbal/model.sdf"
  sim_log "control-validation runtime overlay=$runtime_model_root imu_update_rate=${SIM_IMU_UPDATE_RATE}Hz lock_step=0 source_model=$SIMULATION_DIR/models/ensamb_with_standoffs/model.sdf"
fi
require_file "$model_file"
ldd "$plugin" 2>&1 | grep -q 'not found' && sim_die "plugin has unresolved shared libraries: $plugin"
grep -q "<uri>model://$SIM_MODEL</uri>" "$world_file" ||
  sim_die "world does not include selected model '$SIM_MODEL': $world_file"
grep -q "<fdm_port_in>$GAZEBO_FDM_PORT</fdm_port_in>" "$model_file" ||
  sim_die "model does not use configured plugin port $GAZEBO_FDM_PORT: $model_file"
port_in_use "$GAZEBO_FDM_PORT" && sim_die "Gazebo plugin UDP port is already in use: $GAZEBO_FDM_PORT"

export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
if [[ -n "$runtime_model_root" ]]; then
  export GZ_SIM_RESOURCE_PATH="$runtime_model_root:$SIMULATION_DIR/models:$SIMULATION_DIR/worlds:$ARDUPILOT_GAZEBO_DIR/models:$ARDUPILOT_GAZEBO_DIR/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
else
  export GZ_SIM_RESOURCE_PATH="$SIMULATION_DIR/models:$SIMULATION_DIR/worlds:$ARDUPILOT_GAZEBO_DIR/models:$ARDUPILOT_GAZEBO_DIR/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
fi

cmd=(gz sim -v "$GZ_VERBOSITY" -r)
case "$GAZEBO_HEADLESS" in
  0) ;;
  1) cmd+=(-s) ;;
  *) sim_die "GAZEBO_HEADLESS must be 0 or 1 (got: $GAZEBO_HEADLESS)" ;;
esac
cmd+=("$world_file")

sim_log "world=$world_file model=$SIM_MODEL FDM=udp:127.0.0.1:$GAZEBO_FDM_PORT"
sim_log "resolved_model=$model_file"
sim_log "GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH"
sim_log "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
printf '[simulation] command:'; printf ' %q' "${cmd[@]}"; printf '\n'
run_foreground_tracked gazebo "$world_file" "${cmd[@]}"
