#!/usr/bin/env bash
set -euo pipefail

# Canonical Repo A launcher. Existing helper scripts remain available for
# focused diagnostics, but normal simulation runs should use this entrypoint.

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"
SIMULATION_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd -P)"
REPO_ROOT="$(cd -- "$SIMULATION_DIR/.." && pwd -P)"
SETTING_DIR="$REPO_ROOT/setting"

# Pin all simulation-owned paths and endpoints before loading the shared
# helper. No caller-provided network endpoint can replace these defaults.
export ASTRODRONE_REPO="$REPO_ROOT"
export SIM_WORLD="$SIMULATION_DIR/worlds/ensamb_iris_runway.sdf"
export SIM_MODEL="ensamb_with_gimbal"
export ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
export ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
export MISSION_PLANNER_DIR="${MISSION_PLANNER_DIR:-$HOME/MissionPlanner}"
export SIM_BUILD_DIR="${SIM_BUILD_DIR:-/tmp/astrodrone-control-sim-build}"
export SIM_RUNTIME_DIR="${SIM_RUNTIME_DIR:-/tmp/astrodrone-simulation-runtime}"
export SITL_FRAME="gazebo-iris"
export SITL_INSTANCE=0
export SITL_MAVPROXY_MODE=bundled
export GAZEBO_FDM_PORT=9002
export SITL_MASTER_TCP_PORT=5760
export CONTROL_UDP_PORT=14550
export TELEMETRY_UDP_PORT=14551
export GCS_UDP_PORT=14552
export GCS_TELEMETRY_UDP_PORT=14553
export MAVLINK_AUDIT_TCP_PORT=5770
export MAVPROXY_MASTER_ENDPOINT="tcp:127.0.0.1:5760"
export MISSION_PLANNER_ENDPOINT="udp:127.0.0.1:14552"
export TARGET_UDP_ENDPOINT="udp:127.0.0.1:15020"
export YOLO_HTTP_ENDPOINT="http://127.0.0.1:8002"

# shellcheck source=_common.sh
source "$SCRIPT_DIR/_common.sh"

if [[ -n "${MAVLINK_AUDIT_PYTHON:-}" ]]; then
  AUDIT_VALIDATION_PYTHON="$MAVLINK_AUDIT_PYTHON"
else
  audit_mavproxy_path="$(find_mavproxy)" || {
    printf '%s\n' '[실패] 시뮬레이션 telemetry 검증용 pymavlink/MAVProxy 환경이 없습니다.' >&2
    exit 1
  }
  AUDIT_VALIDATION_PYTHON="$(mavproxy_python "$audit_mavproxy_path")" || {
    printf '%s\n' '[실패] MAVProxy Python 환경을 확인하지 못했습니다.' >&2
    exit 1
  }
fi
require_executable "$AUDIT_VALIDATION_PYTHON"

profile=observe
headless=0
duration=0
heartbeat_diagnostic=0
simulation_profile=default
scenario=default
preflight_order=preflight-first
with_mission_planner=0
status_monitor=0
upload_mission=0
arm=0
mission_args=()
mission_altitude_set=0
mission_waypoint_lat_set=0
mission_waypoint_lon_set=0

usage() {
  cat <<'EOF'
Usage:
  simulation/scripts/run_simulation.sh [options]

Profiles:
  observe       Gazebo + SITL + direct MAVProxy/router + telemetry probe
  shadow        Gazebo + SITL + audit relay + C++ YOLO/control command sink
  sitl-flight   Gazebo + SITL + audit relay + explicit loopback commands

Options:
  --profile NAME              observe, shadow, or sitl-flight
  --headless                  Run Gazebo server without its GUI
  --duration SEC              Stop automatically after SEC seconds
  --heartbeat-diagnostic      Observe only: add the packet audit relay so the
                              SITL-to-router and router-to-UDP heartbeat paths
                              can be measured in one run
  --simulation-profile NAME  default, control-validation,
                              path-angle-validation, or
                              diagonal-approach-validation; validation
                              profiles are runtime-only overlays
  --scenario NAME             default, yolo-target-centered-hold,
                              or yolo-lateral-path;
                              scenarios are simulation-only
  --preflight-order ORDER     preflight-first (default) or mission-first;
                              mission-first is a SITL-only legacy diagnostic
  --mission-planner           Start Mission Planner and use UDP 14552
  --status-monitor            Run the read-only Korean simulation status monitor
  --upload-mission            sitl-flight only; upload the default mission
  --arm                       sitl-flight only; ARM after mission upload
  --waypoint-lat VALUE        sitl-flight mission latitude
  --waypoint-lon VALUE        sitl-flight mission longitude
  --altitude VALUE            sitl-flight mission altitude
  -h, --help                  Show this help
EOF
}

while (( $# )); do
  case "$1" in
    --profile)
      [[ $# -ge 2 ]] || { printf '%s\n' '[실패] --profile 값이 필요합니다.' >&2; exit 2; }
      profile="$2"
      shift 2
      ;;
    --headless)
      headless=1
      shift
      ;;
    --duration)
      [[ $# -ge 2 && "$2" =~ ^[0-9]+$ ]] || { printf '%s\n' '[실패] --duration에는 0 이상의 정수가 필요합니다.' >&2; exit 2; }
      duration="$2"
      shift 2
      ;;
    --heartbeat-diagnostic)
      heartbeat_diagnostic=1
      shift
      ;;
    --simulation-profile)
      [[ $# -ge 2 ]] || { printf '%s\n' '[실패] --simulation-profile 값이 필요합니다.' >&2; exit 2; }
      simulation_profile="$2"
      shift 2
      ;;
    --scenario)
      [[ $# -ge 2 ]] || { printf '%s\n' '[실패] --scenario 값이 필요합니다.' >&2; exit 2; }
      scenario="$2"
      shift 2
      ;;
    --preflight-order)
      [[ $# -ge 2 ]] || { printf '%s\n' '[실패] --preflight-order 값이 필요합니다.' >&2; exit 2; }
      preflight_order="$2"
      shift 2
      ;;
    --mission-planner)
      with_mission_planner=1
      shift
      ;;
    --status-monitor)
      status_monitor=1
      shift
      ;;
    --upload-mission)
      upload_mission=1
      shift
      ;;
    --arm)
      arm=1
      shift
      ;;
    --waypoint-lat|--waypoint-lon|--altitude)
      [[ $# -ge 2 ]] || { printf '[실패] %s 값이 필요합니다.\n' "$1" >&2; exit 2; }
      mission_args+=("$1" "$2")
      [[ "$1" == --altitude ]] && mission_altitude_set=1
      [[ "$1" == --waypoint-lat ]] && mission_waypoint_lat_set=1
      [[ "$1" == --waypoint-lon ]] && mission_waypoint_lon_set=1
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf '[실패] 알 수 없는 옵션: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "$profile" in
  observe|shadow) ;;
  sitl-flight) ;;
  *) printf '[실패] 지원하지 않는 프로파일: %s\n' "$profile" >&2; exit 2 ;;
esac

case "$preflight_order" in
  preflight-first|mission-first) ;;
  *) printf '[실패] --preflight-order는 preflight-first 또는 mission-first여야 합니다.\n' >&2; exit 2 ;;
esac
if [[ "$preflight_order" == mission-first && "$profile" != sitl-flight ]]; then
  printf '%s\n' '[실패] mission-first는 SITL sitl-flight 진단에서만 허용됩니다.' >&2
  exit 2
fi
if [[ "$preflight_order" == mission-first && ( "$upload_mission" != 1 || "$arm" != 1 ) ]]; then
  printf '%s\n' '[실패] mission-first 진단은 --upload-mission --arm과 함께 사용해야 합니다.' >&2
  exit 2
fi

case "$profile" in
  observe) export ASTRODRONE_COMMAND_MODE=observe ;;
  shadow) export ASTRODRONE_COMMAND_MODE=shadow ;;
  sitl-flight) export ASTRODRONE_COMMAND_MODE=flight ;;
esac
export ASTRODRONE_TARGET=sitl

if [[ "$profile" != sitl-flight && ( "$upload_mission" == 1 || "$arm" == 1 || ${#mission_args[@]} -gt 0 ) ]]; then
  printf '%s\n' 'mission upload, ARM, and mission options require --profile sitl-flight' >&2
  exit 2
fi
if [[ "$arm" == 1 && "$upload_mission" != 1 ]]; then
  printf '%s\n' '--arm requires --upload-mission' >&2
  exit 2
fi
if [[ "$profile" == shadow && "$with_mission_planner" == 1 ]]; then
  printf '%s\n' '--mission-planner is disabled in shadow to keep the vehicle command sink closed' >&2
  exit 2
fi
if [[ "$heartbeat_diagnostic" == 1 && "$profile" != observe ]]; then
  printf '%s\n' '--heartbeat-diagnostic is supported only with --profile observe' >&2
  exit 2
fi
case "$simulation_profile" in
  default|control-validation|path-angle-validation|diagonal-approach-validation) ;;
  *) printf 'unsupported simulation profile: %s\n' "$simulation_profile" >&2; exit 2 ;;
esac
case "$scenario" in
  default) ;;
  yolo-target-centered-hold)
    [[ "$profile" == sitl-flight ]] || {
      printf '%s\n' 'yolo-target-centered-hold requires --profile sitl-flight' >&2
      exit 2
    }
    [[ "$upload_mission" == 1 && "$arm" == 1 ]] || {
      printf '%s\n' 'yolo-target-centered-hold requires --upload-mission and --arm' >&2
      exit 2
    }
    ;;
  yolo-lateral-path)
    [[ "$profile" == sitl-flight ]] || {
      printf '%s\n' 'yolo-lateral-path requires --profile sitl-flight' >&2
      exit 2
    }
    [[ "$upload_mission" == 1 && "$arm" == 1 ]] || {
      printf '%s\n' 'yolo-lateral-path requires --upload-mission and --arm' >&2
      exit 2
    }
    [[ "$simulation_profile" == path-angle-validation ]] || {
      printf '%s\n' 'yolo-lateral-path requires --simulation-profile path-angle-validation' >&2
      exit 2
    }
    ;;
  *) printf 'unsupported scenario: %s\n' "$scenario" >&2; exit 2 ;;
esac
export SIMULATION_PROFILE="$simulation_profile"
if [[ "$simulation_profile" == control-validation ]]; then
  export SIM_IMU_UPDATE_RATE="${SIM_IMU_UPDATE_RATE:-250.0}"
  [[ "$SIM_IMU_UPDATE_RATE" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
    printf 'SIM_IMU_UPDATE_RATE must be numeric: %s\n' "$SIM_IMU_UPDATE_RATE" >&2
    exit 2
  }
fi
if [[ "$simulation_profile" == path-angle-validation ]]; then
  # These are consumed only by control.cpp when this explicit simulation
  # profile is selected. Production YAML and default execution are unchanged.
  export GUIDANCE_MAX_FORWARD_SPEED_MPS=0.2
  export GUIDANCE_MAX_LATERAL_SPEED_MPS=0.2
  export GUIDANCE_MAX_VECTOR_SPEED_MPS=0.2
  export GUIDANCE_MAX_DESCENT_SPEED_MPS=0.2
  export GUIDANCE_DESIRED_PATH_ANGLE_RAD=0.785398
fi
if [[ "$simulation_profile" == diagonal-approach-validation ]]; then
  # Diagonal validation is an opt-in overlay. Production centered_descent and
  # the real-vehicle settings remain unchanged.
  export GUIDANCE_MAX_FORWARD_SPEED_MPS=0.3
  export GUIDANCE_MAX_LATERAL_SPEED_MPS=0.3
  export GUIDANCE_MAX_VECTOR_SPEED_MPS=0.3
  export GUIDANCE_MAX_DESCENT_SPEED_MPS=0.3
  export GUIDANCE_DESIRED_PATH_ANGLE_RAD=0.785398
fi

case "$headless" in
  0) export GAZEBO_HEADLESS=0 ;;
  1) export GAZEBO_HEADLESS=1 ;;
esac

for endpoint in \
  "tcp:127.0.0.1:$SITL_MASTER_TCP_PORT" \
  "udp:127.0.0.1:$GAZEBO_FDM_PORT" \
  "udp:127.0.0.1:$CONTROL_UDP_PORT" \
  "udp:127.0.0.1:$TELEMETRY_UDP_PORT" \
  "udp:127.0.0.1:$GCS_UDP_PORT" \
  "udp:127.0.0.1:$GCS_TELEMETRY_UDP_PORT"; do
  [[ "$endpoint" =~ ^(tcp|udp):127\.0\.0\.1:[0-9]+$ ]] || {
    printf 'non-loopback endpoint rejected: %s\n' "$endpoint" >&2
    exit 2
  }
done

[[ -f "$SIM_WORLD" ]] || { printf 'custom world not found: %s\n' "$SIM_WORLD" >&2; exit 1; }
[[ -f "$SIMULATION_DIR/models/$SIM_MODEL/model.sdf" ]] || {
  printf 'custom model not found: %s\n' "$SIMULATION_DIR/models/$SIM_MODEL/model.sdf" >&2
  exit 1
}
[[ -x "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" ]] || {
  printf 'ArduPilot SITL launcher not found: %s\n' "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" >&2
  exit 1
}
[[ -x "$ARDUPILOT_DIR/build/sitl/bin/arducopter" ]] || {
  printf 'ArduCopter SITL binary not found: %s\n' "$ARDUPILOT_DIR/build/sitl/bin/arducopter" >&2
  exit 1
}

: "${RUN_ID:=$(date +%Y%m%d_%H%M%S)}"
log_dir="$REPO_ROOT/simulation/logs/run_${profile}_${RUN_ID}"
mkdir -p -- "$log_dir"
cd -- "$REPO_ROOT"

log_launcher() {
  printf '%s\n' "$*" >>"$log_dir/launcher.log"
}

scenario_settings_dir=''
if [[ "$scenario" == yolo-target-centered-hold || "$scenario" == yolo-lateral-path ]]; then
  # This scenario keeps the real Gazebo target and the real C++ TensorRT YOLO
  # path. Only disposable world/settings copies are changed; no fixed target
  # range is supplied to target-distance.
  scenario_root="$SIM_RUNTIME_DIR/scenarios/yolo-target-centered-hold"
  scenario_settings_dir="$scenario_root/setting"
  scenario_model_root="$scenario_root/models"
  scenario_world="$scenario_root/ensamb_iris_yolo_target_centered_hold.sdf"
  fixture_scale='0.004'
  [[ "$scenario" == yolo-lateral-path ]] && fixture_scale='0.008'
  mkdir -p -- "$scenario_settings_dir"
  rm -rf -- "$scenario_model_root/target_basket_yolo_fixture"
  mkdir -p -- "$scenario_model_root"
  cp -a -- "$SIMULATION_DIR/models/target_basket" "$scenario_model_root/target_basket_yolo_fixture"
  sed -E -i \
    -e 's#model://target_basket/meshes/#model://target_basket_yolo_fixture/meshes/#' \
    -e "s#<scale>0\\.001 0\\.001 0\\.001</scale>#<scale>${fixture_scale} ${fixture_scale} ${fixture_scale}</scale>#" \
    "$scenario_model_root/target_basket_yolo_fixture/model.sdf"
  cp -- "$SIMULATION_DIR/worlds/ensamb_iris_runway.sdf" "$scenario_world"
  # gazeboXYZToNED rotates Gazebo +y into SITL local north for this world.
  # Place the disposable target at local north=3.0m. The lateral scenario
  # adds a 2.0m east offset so the real detector produces signed x/y error
  # and the controller can be observed generating vx+vy+vz.
  target_pose='0 3.00 0 0 0 0'
  target_waypoint_lat='-35.3632359'
  target_waypoint_lon='149.1652370'
  if [[ "$scenario" == yolo-lateral-path ]]; then
    target_pose='1.00 4.00 0 0 0 0'
    # Stop AUTO at north=2m while the target remains 2m forward and 1m
    # lateral. This leaves both signed image errors present when the control
    # gate opens, instead of passing the target before control starts.
    target_waypoint_lat='-35.3632440'
    target_waypoint_lon='149.1652370'
  fi
  sed -E -i \
    -e 's#<uri>model://target_basket</uri>#<uri>model://target_basket_yolo_fixture</uri>#' \
    -e "/<uri>model:\/\/target_basket_yolo_fixture<\/uri>/,/<\/include>/ s#<pose>0 4 0 0 0 0</pose>#<pose>${target_pose}</pose>#" \
    "$scenario_world"
  cp -- "$SETTING_DIR/MAVLink.yaml" "$scenario_settings_dir/MAVLink.yaml"
  cp -- "$SETTING_DIR/safety.yaml" "$scenario_settings_dir/safety.yaml"
  cp -- "$SETTING_DIR/port.yaml" "$scenario_settings_dir/port.yaml"
  cp -- "$SETTING_DIR/rate.yaml" "$scenario_settings_dir/rate.yaml"
  sed -E -i \
    -e 's/^([[:space:]]*handoff_min_altitude_m:).*/\1 0.5/' \
    -e 's/^([[:space:]]*lock_confirm_sec:).*/\1 0.25/' \
    -e 's/^([[:space:]]*center_tolerance_px:).*/\1 25.0/' \
    -e 's/^([[:space:]]*guidance_theta_safe_threshold_rad:).*/\1 1.5/' \
    -e 's/^([[:space:]]*guidance_desired_path_angle_rad:).*/\1 0.35/' \
    -e 's/^([[:space:]]*intercept_approach_duration_sec:).*/\1 120.0/' \
    "$scenario_settings_dir/MAVLink.yaml"
  export SIM_WORLD="$scenario_world"
  export ASTRODRONE_SETTINGS_DIR="$scenario_settings_dir"
  export GZ_SIM_RESOURCE_PATH="$scenario_model_root"
  unset FIXED_TARGET_NORTH_M FIXED_TARGET_EAST_M FIXED_TARGET_MIN_ALT_M
  if (( mission_waypoint_lat_set == 0 )); then
    # The detector-visible pose is also the disposable waypoint; the C++
    # controller still has to align on the real pixel observation before hold.
    mission_args+=(--waypoint-lat "$target_waypoint_lat")
  fi
  if (( mission_waypoint_lon_set == 0 )); then
    mission_args+=(--waypoint-lon "$target_waypoint_lon")
  fi
  if (( mission_altitude_set == 0 )); then
    # Keep this disposable test low enough for the physical basket mesh to
    # remain resolvable by the real detector while still exercising takeoff,
    # guided handoff, range <= 2m, and centered hold.
    if [[ "$scenario" == yolo-lateral-path ]]; then
      mission_args+=(--altitude 3.0)
    else
      mission_args+=(--altitude 0.8)
    fi
  fi
fi

# The default world is the production geometry and must keep the production
# settings.  Give the explicit path-angle validation profile only a disposable
# settings copy with enough approach time to reach the 2 m stop distance at
# its intentionally limited 0.2 m/s horizontal speed.
if [[ ( "$simulation_profile" == path-angle-validation ||
        "$simulation_profile" == diagonal-approach-validation ) &&
      -z "$scenario_settings_dir" ]]; then
  path_angle_settings_root="$SIM_RUNTIME_DIR/scenarios/path-angle-validation"
  path_angle_settings_dir="$path_angle_settings_root/setting"
  mkdir -p -- "$path_angle_settings_dir"
  cp -- "$SETTING_DIR/MAVLink.yaml" "$path_angle_settings_dir/MAVLink.yaml"
  cp -- "$SETTING_DIR/safety.yaml" "$path_angle_settings_dir/safety.yaml"
  cp -- "$SETTING_DIR/port.yaml" "$path_angle_settings_dir/port.yaml"
  cp -- "$SETTING_DIR/rate.yaml" "$path_angle_settings_dir/rate.yaml"
  sed -E -i \
    -e 's/^([[:space:]]*intercept_approach_duration_sec:).*/\1 120.0/' \
    "$path_angle_settings_dir/MAVLink.yaml"
  if [[ "$simulation_profile" == diagonal-approach-validation ]]; then
    sed -E -i \
      -e 's/^([[:space:]]*guidance_mode:).*/\1 diagonal_approach/' \
      "$path_angle_settings_dir/MAVLink.yaml"
  fi
  export ASTRODRONE_SETTINGS_DIR="$path_angle_settings_dir"
fi

printf '[run_simulation] profile=%s\n' "$profile" >"$log_dir/launcher.log"
printf '[run_simulation] scenario=%s\n' "$scenario" >>"$log_dir/launcher.log"
if [[ "$preflight_order" == mission-first ]]; then
  printf '%s\n' '[실행 순서] LEGACY_MISSION_FIRST_DIAGNOSTIC' >>"$log_dir/launcher.log"
  printf '%s\n' '[run_simulation] WARNING: mission-first is SITL-only legacy compatibility diagnostics, not production policy.' >>"$log_dir/launcher.log"
else
  printf '%s\n' '[실행 순서] PREFLIGHT_FIRST' >>"$log_dir/launcher.log"
fi
reported_imu_rate='source-model'
[[ "$simulation_profile" == control-validation ]] && reported_imu_rate="${SIM_IMU_UPDATE_RATE}"
printf '[run_simulation] simulation_profile=%s imu_update_rate=%sHz camera_update_rate=10Hz\n' \
  "$simulation_profile" "$reported_imu_rate" >>"$log_dir/launcher.log"
if [[ "$simulation_profile" == control-validation ]]; then
  printf '%s\n' '[run_simulation] control-validation uses production guidance speed limits; only the simulation overlay is changed' >>"$log_dir/launcher.log"
fi
printf '[run_simulation] world=%s model=%s\n' "$SIM_WORLD" "$SIM_MODEL" >>"$log_dir/launcher.log"
printf '[run_simulation] endpoints FDM=9002 SITL=5760 control=14550 telemetry=14551 GCS=14552 GCS_telemetry=14553\n' >>"$log_dir/launcher.log"

pids=()
essential_pids=()
essential_names=()
mission_pid=''
status_monitor_pid=''
shadow_router_pid=''
preflight_ready_file=''
mission_setup_complete_file=''
audit_report="$log_dir/packet_audit.json"
telemetry_probe_report="$log_dir/telemetry_probe.json"
gazebo_stats_log="$log_dir/gazebo_stats.log"
export MAVLINK_AUDIT_EVENT_FILE="$log_dir/control_events.jsonl"
audit_required=0
path_angle_report_required=0
telemetry_probe_required=0
telemetry_probe_pid=''
gazebo_stats_pid=''
normal_completion=0
cleaned=0

stop_group() {
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
}

cleanup() {
  (( cleaned == 1 )) && return 0
  cleaned=1
  if [[ -n "$status_monitor_pid" ]]; then
    printf '[run_simulation] cleanup: read-only status monitor\n' >>"$log_dir/cleanup.log"
    stop_group "$status_monitor_pid"
    wait "$status_monitor_pid" 2>/dev/null || true
  fi
  if [[ -n "$mission_pid" ]]; then
    printf '[run_simulation] cleanup: mission/upload process\n' >>"$log_dir/cleanup.log"
    stop_group "$mission_pid"
  fi
  # The autonomy and shadow sink processes have no tracked simulation PID
  # file, so stop them before the exact-group simulation cleanup below.
  if [[ -n "${autonomy_pid:-}" ]]; then
    printf '[run_simulation] cleanup: control/YOLO\n' >>"$log_dir/cleanup.log"
    stop_group "$autonomy_pid"
  fi
  if [[ -n "$telemetry_probe_pid" ]]; then
    printf '[run_simulation] cleanup: direct telemetry probe\n' >>"$log_dir/cleanup.log"
    stop_group "$telemetry_probe_pid"
  fi
  if [[ -n "$gazebo_stats_pid" ]]; then
    printf '[run_simulation] cleanup: Gazebo stats probe\n' >>"$log_dir/cleanup.log"
    stop_group "$gazebo_stats_pid"
  fi
  if [[ -n "$shadow_router_pid" ]]; then
    printf '[run_simulation] cleanup: shadow command sink\n' >>"$log_dir/cleanup.log"
    stop_group "$shadow_router_pid"
  fi
  # stop_simulation.sh owns the tracked order: router/MAVProxy, audit relay,
  # SITL, then Gazebo. Do not TERM the outer wrappers first; that was the
  # source of the bundled MAVProxy interpreter shutdown race.
  printf '[run_simulation] cleanup: MAVProxy/router -> audit -> SITL -> Gazebo\n' >>"$log_dir/cleanup.log"
  "$SCRIPT_DIR/stop_simulation.sh" >>"$log_dir/cleanup.log" 2>&1 || true
  for pid in "${pids[@]:-}" "$shadow_router_pid" "$mission_pid"; do
    [[ -n "$pid" ]] && wait "$pid" 2>/dev/null || true
  done
  if (( audit_required == 1 )); then
    if [[ ! -s "$audit_report" ]]; then
      printf '[simulation] ERROR: required packet audit report is missing: %s\n' "$audit_report" >>"$log_dir/cleanup.log"
      exit 1
    elif ! grep -q '"vehicle_affecting_command_count"' "$audit_report"; then
      printf '[simulation] ERROR: packet audit report is missing vehicle command count: %s\n' "$audit_report" >>"$log_dir/cleanup.log"
      exit 1
    fi
    if ! "$AUDIT_VALIDATION_PYTHON" - "$audit_report" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    report = json.load(stream)

heartbeat = report.get("sitl_to_mavproxy", {}).get("heartbeat", {})
if heartbeat.get("valid_ardupilot_count", 0) < 1:
    raise SystemExit("audit parser produced no valid ArduPilot HEARTBEAT")
if report.get("sitl_to_mavproxy", {}).get("direction") != "SITL_AUTOPILOT_TO_MAVPROXY":
    raise SystemExit("audit SITL direction metadata is missing")
if report.get("mavproxy_to_sitl", {}).get("direction") != "MAVPROXY_TO_SITL_AUTOPILOT":
    raise SystemExit("audit downstream direction metadata is missing")
PY
    then
      printf '[simulation] ERROR: packet audit validation failed\n' >>"$log_dir/cleanup.log"
      exit 1
    fi
    if (( path_angle_report_required == 1 )); then
      flight_csv=""
      while IFS= read -r candidate; do
        flight_csv="$candidate"
        break
      done < <(find "$log_dir" -maxdepth 1 -type f -name 'flight_*.csv' -print | sort)
      path_report_cmd=(
        "$AUDIT_VALIDATION_PYTHON"
        "$SIMULATION_DIR/tests/path_angle_report.py"
        --audit "$audit_report"
        --output "$log_dir/path_angle_report.json"
      )
      if [[ "$simulation_profile" == diagonal-approach-validation ]]; then
        path_report_cmd+=(--stop-distance 1.0)
      fi
      [[ -z "$flight_csv" ]] || path_report_cmd+=(--flight-csv "$flight_csv")
      if ! "${path_report_cmd[@]}" >>"$log_dir/cleanup.log" 2>&1; then
        printf '[simulation] ERROR: path-angle report generation failed\n' >>"$log_dir/cleanup.log"
        exit 1
      fi
    fi
  fi
  if (( telemetry_probe_required == 1 )) && [[ ! -s "$telemetry_probe_report" ]]; then
    printf '[simulation] ERROR: direct telemetry probe report is missing: %s\n' "$telemetry_probe_report" >>"$log_dir/cleanup.log"
    exit 1
  elif (( telemetry_probe_required == 1 )) && ! "$AUDIT_VALIDATION_PYTHON" - "$telemetry_probe_report" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    report = json.load(stream)

if report.get("direction") != "SITL_AUTOPILOT_TO_MAVPROXY_ROUTER":
    raise SystemExit("direct telemetry probe direction metadata is missing")
for name in ("control", "telemetry"):
    heartbeat = report.get("streams", {}).get(name, {}).get("heartbeat", {})
    if heartbeat.get("valid_ardupilot_count", 0) < 1:
        raise SystemExit(f"no valid ArduPilot HEARTBEAT on direct {name} output")
PY
  then
    printf '[simulation] ERROR: direct telemetry probe validation failed\n' >>"$log_dir/cleanup.log"
    exit 1
  fi
}
trap cleanup EXIT INT TERM

start_process() {
  local name="$1"
  shift
  printf '[run_simulation] starting %s:' "$name" >>"$log_dir/launcher.log"
  printf ' %q' "$@" >>"$log_dir/launcher.log"
  printf '\n' >>"$log_dir/launcher.log"
  setsid "$@" >"$log_dir/$name.log" 2>&1 &
  started_pid="$!"
  pids+=("$started_pid")
}

start_status_monitor() {
  printf '[run_simulation] starting status-monitor: %q %q --profile simulation --log-dir %q --watch --duration 0\n' \
    "$AUDIT_VALIDATION_PYTHON" \
    "$REPO_ROOT/health-check/monitor/status_monitor.py" \
    "$log_dir" >>"$log_dir/launcher.log"
  setsid env PYTHONUNBUFFERED=1 \
    "$AUDIT_VALIDATION_PYTHON" \
    "$REPO_ROOT/health-check/monitor/status_monitor.py" \
    --profile simulation \
    --log-dir "$log_dir" \
    --watch \
    --duration 0 \
    > >(tee -a "$log_dir/status-monitor.log") 2>&1 &
  status_monitor_pid="$!"
  pids+=("$status_monitor_pid")
}

track_essential_process() {
  essential_names+=("$1")
  essential_pids+=("$2")
}

process_is_alive() {
  local pid="$1"
  kill -0 "$pid" 2>/dev/null || return 1
  local state
  state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d ' ')"
  [[ -n "$state" && "$state" != Z* ]]
}

monitor_essential_processes() {
  local index pid name
  for index in "${!essential_pids[@]}"; do
    pid="${essential_pids[$index]}"
    name="${essential_names[$index]}"
    if ! process_is_alive "$pid"; then
      # Target-loss is a normal production termination in the current SITL
      # policy: control holds LOITER and exits with status 0. Do not turn that
      # deliberate completion into a launcher failure.
      if [[ "$name" == autonomy && -f "$log_dir/autonomy.log" ]] &&
        grep -q '\[autonomy-cpp\] child exited: control status=0' "$log_dir/autonomy.log"; then
        printf '[run_simulation] autonomy completed normally after its control policy\n' >>"$log_dir/launcher.log"
        normal_completion=1
        essential_pids[$index]=''
        essential_names[$index]=''
        continue
      fi
      {
        printf '[run_simulation] ERROR: monitored process exited: %s pid=%s\n' "$name" "$pid"
        printf '[run_simulation] inspect %s and the corresponding process log for the failure.\n' "$log_dir/launcher.log"
        if [[ "$name" == gazebo ]]; then
          printf '[run_simulation] Gazebo log: %s/gazebo.log\n' "$log_dir"
        fi
      } >>"$log_dir/launcher.log"
      printf '[실패] %s 프로세스가 종료되었습니다.\n[로그] %s\n' "$name" "$log_dir" >&2
      return 1
    fi
  done
}

wait_for_port() {
  local port="$1" timeout="$2" label="$3"
  local deadline=$((SECONDS + timeout))
  while (( SECONDS < deadline )); do
    ss -H -lntup 2>/dev/null | awk '{print $5}' | grep -Eq "[:.]${port}$" && return 0
    sleep 0.2
  done
  printf '[run_simulation] %s did not open loopback port %s\n' "$label" "$port" >>"$log_dir/launcher.log"
  printf '[실패] %s 준비에 실패했습니다.\n[로그] %s\n' "$label" "$log_dir" >&2
  return 1
}

wait_for_process() {
  local pid="$1" label="$2"
  sleep 0.5
  kill -0 "$pid" 2>/dev/null || {
    printf '[run_simulation] %s exited during startup; inspect logs\n' "$label" >>"$log_dir/launcher.log"
    printf '[실패] %s 시작에 실패했습니다.\n[로그] %s\n' "$label" "$log_dir" >&2
    return 1
  }
}

wait_for_gazebo_ready() {
  local pid="$1"
  wait_for_port "$GAZEBO_FDM_PORT" 30 Gazebo >/dev/null 2>&1 || return 1
  if (( headless == 1 )); then
    process_is_alive "$pid"
    return
  fi
  # GUI startup can open the FDM socket before OGRE/rendering finishes. Keep
  # one bounded stability window so a crash during GUI initialization is
  # retried once instead of being reported as a ready SITL.
  for _ in $(seq 1 50); do
    process_is_alive "$pid" || return 1
    sleep 0.1
  done
}

wait_for_log_text() {
  local file="$1" pattern="$2" timeout="$3" label="$4"
  local deadline=$((SECONDS + timeout))
  while (( SECONDS < deadline )); do
    [[ -f "$file" ]] && grep -q -- "$pattern" "$file" && return 0
    sleep 0.2
  done
  printf '[run_simulation] %s was not observed in %s\n' "$label" "$file" >>"$log_dir/launcher.log"
  printf '[실패] %s 확인에 실패했습니다.\n[로그] %s\n' "$label" "$log_dir" >&2
  return 1
}

wait_for_file() {
  local file="$1" timeout="$2" label="$3" pid="${4:-}"
  local deadline=$((SECONDS + timeout))
  while (( SECONDS < deadline )); do
    [[ -f "$file" ]] && return 0
    if [[ -n "$pid" ]] && ! process_is_alive "$pid"; then
      printf '[run_simulation] %s process exited before %s was ready\n' "$label" "$file" >>"$log_dir/launcher.log"
      printf '[실패] %s 준비 전에 autonomy가 종료되었습니다.\n[로그] %s\n' "$label" "$log_dir" >&2
      return 1
    fi
    sleep 0.2
  done
  printf '[run_simulation] %s was not observed: %s\n' "$label" "$file" >>"$log_dir/launcher.log"
  printf '[실패] %s 확인에 실패했습니다.\n[로그] %s\n' "$label" "$log_dir" >&2
  return 1
}

wait_for_yolo_ready() {
  local pid="$1" log_file="$2" timeout="$3"
  local deadline=$((SECONDS + timeout))
  while (( SECONDS < deadline )); do
    if [[ -f "$log_file" ]] && grep -q '^\[cpp-yolo-autonomy-sitl\] YOLO_READY ' "$log_file"; then
      return 0
    fi
    if ! kill -0 "$pid" 2>/dev/null; then
      printf '[run_simulation] shadow pipeline incomplete: autonomy exited before YOLO readiness\n' >>"$log_dir/launcher.log"
      printf '[실패] YOLO가 준비되기 전에 autonomy가 종료되었습니다.\n[로그] %s\n' "$log_dir" >&2
      return 1
    fi
    sleep 0.2
  done
  printf '[run_simulation] shadow pipeline incomplete: YOLO readiness timeout after %ss\n' "$timeout" >>"$log_dir/launcher.log"
  printf '[실패] YOLO 준비 시간 초과입니다.\n[로그] %s\n' "$log_dir" >&2
  return 1
}

if (( status_monitor == 1 )); then
  start_status_monitor
fi

if (( headless == 1 )); then
  gazebo_command=(env -u DISPLAY -u WAYLAND_DISPLAY "$SCRIPT_DIR/start_gazebo.sh")
else
  gazebo_command=(env -u WAYLAND_DISPLAY DISPLAY="${DISPLAY:-}" "$SCRIPT_DIR/start_gazebo.sh")
fi
start_process gazebo "${gazebo_command[@]}"
gazebo_pid="$started_pid"
track_essential_process gazebo "$gazebo_pid"
gazebo_ready=0
if wait_for_gazebo_ready "$gazebo_pid"; then
  gazebo_ready=1
fi
if (( gazebo_ready == 0 && headless == 0 )); then
  printf '%s\n' '[run_simulation] GUI Gazebo startup failed once; retrying exactly once.' >>"$log_dir/launcher.log"
  stop_group "$gazebo_pid"
  wait "$gazebo_pid" 2>/dev/null || true
  essential_pids[${#essential_pids[@]}-1]=''
  start_process gazebo-retry "${gazebo_command[@]}"
  gazebo_pid="$started_pid"
  track_essential_process gazebo "$gazebo_pid"
  if wait_for_gazebo_ready "$gazebo_pid"; then
    gazebo_ready=1
    printf '%s\n' '[run_simulation] GUI Gazebo retry succeeded.' >>"$log_dir/launcher.log"
  else
    printf '%s\n' '[run_simulation] GUI Gazebo retry failed; no further retry will be attempted.' >>"$log_dir/launcher.log"
  fi
fi
if (( gazebo_ready == 0 )); then
  {
    printf '[run_simulation] ERROR: Gazebo startup failed before FDM port %s became ready.\n' "$GAZEBO_FDM_PORT"
    printf '[run_simulation] GUI mode environment: DISPLAY=%q WAYLAND_DISPLAY=unset GAZEBO_HEADLESS=%q\n' "${DISPLAY:-}" "$GAZEBO_HEADLESS"
    printf '[run_simulation] inspect %s for the exact Gazebo command and GUI/OpenGL error.\n' "$log_dir/gazebo.log"
  } >>"$log_dir/launcher.log"
  printf '[실패] Gazebo 준비에 실패했습니다.\n[로그] %s\n' "$log_dir" >&2
  exit 1
fi
printf '[run_simulation] READY Gazebo\n' >>"$log_dir/launcher.log"

if [[ "$heartbeat_diagnostic" == 1 ]]; then
  # Gazebo publishes this server-side topic even in -s mode. Keep it as a
  # diagnostic-only process so real-time factor can be compared with the
  # heartbeat gaps without touching the production transport.
  start_process gazebo-stats \
    gz topic -e --json-output -t "/world/ensamb_iris_runway/stats"
  gazebo_stats_pid="$started_pid"
  wait_for_process "$gazebo_stats_pid" gazebo-stats
fi

start_process sitl \
  env SITL_MAVPROXY_MODE=external \
  "$SCRIPT_DIR/start_sitl.sh"
track_essential_process sitl "$started_pid"
wait_for_port "$SITL_MASTER_TCP_PORT" 45 SITL

if [[ "$profile" == shadow || "$profile" == sitl-flight || "$heartbeat_diagnostic" == 1 ]]; then
  audit_required=1
  [[ "$simulation_profile" == path-angle-validation ||
     "$simulation_profile" == diagonal-approach-validation ]] && path_angle_report_required=1
  # The audit relay is normally required only for shadow/sitl-flight. The
  # observe diagnostic option enables it explicitly so the upstream SITL
  # timestamp can be compared with both router UDP outputs.
  start_process audit \
    env MAVLINK_AUDIT_REPORT="$audit_report" \
    "$SCRIPT_DIR/start_mavlink_audit.sh"
  track_essential_process audit "$started_pid"
  wait_for_port "$MAVLINK_AUDIT_TCP_PORT" "$MAVLINK_AUDIT_READY_TIMEOUT" audit
  wait_for_log_text "$log_dir/audit.log" '^AUDIT_LISTEN ' 10 audit-listener
else
  telemetry_probe_required=1
  printf '[run_simulation] observe: audit disabled; direct telemetry probe starts after ArduPilot READY\n' >>"$log_dir/launcher.log"
fi

if [[ "$profile" == shadow ]]; then
  router_log="$log_dir/shadow_router.log"
  mavproxy_binary="$(find_mavproxy)" || { printf '%s\n' 'MAVProxy is required for shadow router Python discovery' >&2; exit 1; }
  router_python="$(mavproxy_python "$mavproxy_binary")" || {
    printf '%s\n' 'MAVProxy Python interpreter could not be resolved' >&2
    exit 1
  }
  setsid "$router_python" "$SCRIPT_DIR/shadow_udp_router.py" \
    --listen 14553 --control 14550 --target 14551 --gcs 14552 \
    >"$router_log" 2>&1 &
  shadow_router_pid="$!"
  wait_for_port 14553 10 shadow-router
fi

# Run MAVProxy as a separately tracked process. sim_vehicle.py remains the
# SITL launcher, but no longer owns a MAVProxy daemon thread during shutdown.
if [[ "$profile" == shadow ]]; then
  start_process router \
    env MAVPROXY_MASTER_ENDPOINT="tcp:127.0.0.1:$MAVLINK_AUDIT_TCP_PORT" \
        CONTROL_UDP_PORT=14550 TELEMETRY_UDP_PORT=14553 \
        ENABLE_CONTROL_OUTPUT=0 ENABLE_GCS_OUTPUT=0 \
    "$SCRIPT_DIR/start_router.sh"
elif [[ "$profile" == sitl-flight ]]; then
  start_process router \
    env MAVPROXY_MASTER_ENDPOINT="tcp:127.0.0.1:${MAVLINK_AUDIT_TCP_PORT}" \
        CONTROL_UDP_PORT=14550 TELEMETRY_UDP_PORT=14551 \
        ENABLE_CONTROL_OUTPUT=1 ENABLE_GCS_OUTPUT=1 ENABLE_GCS_TELEMETRY_OUTPUT=1 \
    "$SCRIPT_DIR/start_router.sh"
elif [[ "$heartbeat_diagnostic" == 1 ]]; then
  telemetry_probe_required=1
  start_process router \
    env MAVPROXY_MASTER_ENDPOINT="tcp:127.0.0.1:${MAVLINK_AUDIT_TCP_PORT}" \
        CONTROL_UDP_PORT=14550 TELEMETRY_UDP_PORT=14551 \
        ENABLE_CONTROL_OUTPUT=1 ENABLE_GCS_OUTPUT=1 ENABLE_GCS_TELEMETRY_OUTPUT=1 \
    "$SCRIPT_DIR/start_router.sh"
else
  start_process router \
    env MAVPROXY_MASTER_ENDPOINT="tcp:127.0.0.1:${SITL_MASTER_TCP_PORT}" \
        CONTROL_UDP_PORT=14550 TELEMETRY_UDP_PORT=14551 \
        ENABLE_CONTROL_OUTPUT=1 ENABLE_GCS_OUTPUT=1 ENABLE_GCS_TELEMETRY_OUTPUT=1 \
    "$SCRIPT_DIR/start_router.sh"
fi
track_essential_process router "$started_pid"
if (( audit_required == 1 )); then
  wait_for_log_text "$log_dir/audit.log" '^SITL_CONNECTED ' 30 audit-sitl-connection
else
  wait_for_log_text "$log_dir/router.log" 'Connect tcp:' 30 direct-mavproxy-connection
fi
wait_for_log_text "$log_dir/router.log" '^\[simulation\] router=MAVProxy ' 20 mavproxy
# Do not start the profile's duration window while ArduPilot is still waiting
# for Gazebo JSON/IMU input. This is a readiness gate only; it does not alter
# heartbeat timeout or streamrate settings.
wait_for_log_text "$log_dir/router.log" 'AP: ArduPilot Ready' 45 ardupilot-ready
printf '[run_simulation] READY HEARTBEAT\n' >>"$log_dir/launcher.log"
printf '[run_simulation] READY HEALTH\n' >>"$log_dir/launcher.log"
printf '[run_simulation] READY MAVLINK\n' >>"$log_dir/launcher.log"
wait_for_log_text "$log_dir/gazebo.log" 'Camera images for .*down_camera' 45 camera-frame
printf '[run_simulation] READY CAMERA\n' >>"$log_dir/launcher.log"

# Start heartbeat statistics only after Gazebo FDM, SITL, MAVProxy, and the
# ArduPilot readiness marker are all present. This avoids counting startup
# starvation as steady-state transport loss.
if [[ "$profile" == observe ]]; then
  if (( heartbeat_diagnostic == 1 )); then
    printf '[run_simulation] observe heartbeat diagnostic: audit upstream + UDP 14550/14551 probes\n' >>"$log_dir/launcher.log"
  else
    printf '[run_simulation] observe: direct UDP 14550/14551 probe\n' >>"$log_dir/launcher.log"
  fi
  telemetry_probe_required=1
  start_process telemetry-probe \
    env \
    "$AUDIT_VALIDATION_PYTHON" "$SIMULATION_DIR/tests/telemetry_probe.py" \
    --control "udp:127.0.0.1:$CONTROL_UDP_PORT" \
    --telemetry "udp:127.0.0.1:$TELEMETRY_UDP_PORT" \
    --report "$telemetry_probe_report"
  telemetry_probe_pid="$started_pid"
  track_essential_process telemetry-probe "$telemetry_probe_pid"
  wait_for_process "$telemetry_probe_pid" telemetry-probe
  wait_for_log_text "$log_dir/telemetry-probe.log" '^TELEMETRY_PROBE_READY ' 10 telemetry-probe
fi

if [[ "$profile" == shadow || "$profile" == sitl-flight ]]; then
  printf '[run_simulation] prebuilding target-distance/control before runtime gating\n' >>"$log_dir/launcher.log"
  if ! DRONE_PROFILE=sitl \
    "$REPO_ROOT/scripts/autonomy-cpp" --profile sitl --build-only \
    >"$log_dir/autonomy-build.log" 2>&1; then
    printf '[run_simulation] ERROR: target-distance/control build failed; inspect %s\n' \
      "$log_dir/autonomy-build.log" >>"$log_dir/launcher.log"
    printf '[실패] target-distance/control 준비에 실패했습니다.\n[로그] %s\n' "$log_dir" >&2
    exit 1
  fi
  printf '[run_simulation] target-distance/control build ready\n' >>"$log_dir/launcher.log"
  printf '[run_simulation] READY BUILD\n' >>"$log_dir/launcher.log"
fi

if [[ "$profile" == shadow ]]; then
  start_process autonomy \
    env LOG_DIR="$log_dir" \
        YOLO_READY_FILE="$log_dir/yolo_ready.txt" \
        "$REPO_ROOT/scripts/cpp-yolo-autonomy-sitl"
  autonomy_pid="${started_pid:-}"
  wait_for_process "$autonomy_pid" autonomy
  wait_for_yolo_ready "$autonomy_pid" "$log_dir/autonomy.log" "${YOLO_READY_TIMEOUT_SEC:-90}"
  # Fail closed if the controller dies after YOLO readiness. Otherwise SITL
  # could continue in AUTO without the autonomy process supervising it.
  track_essential_process autonomy "$autonomy_pid"
fi

# In sitl-flight, the default starts C++ before mission setup so its read-only
# preflight gates the launcher. The legacy diagnostic starts YOLO first but
# holds target-distance/control behind a gate until mission setup is complete.
if [[ "$profile" == sitl-flight ]]; then
  preflight_ready_file="$log_dir/preflight_ready"
  mission_setup_complete_file="$log_dir/mission_setup_complete"
  autonomy_env=(
    LOG_DIR="$log_dir"
    YOLO_READY_FILE="$log_dir/yolo_ready.txt"
    AUTONOMY_PREFLIGHT_READY_FILE="$preflight_ready_file"
    AUTONOMY_MISSION_SETUP_COMPLETE_FILE="$mission_setup_complete_file"
  )
  if [[ "$preflight_order" == mission-first ]]; then
    autonomy_env+=(AUTONOMY_CONTROL_GATE_FILE="$mission_setup_complete_file")
  fi
  start_process autonomy env "${autonomy_env[@]}" \
    "$REPO_ROOT/scripts/cpp-yolo-autonomy-sitl"
  autonomy_pid="${started_pid:-}"
  wait_for_process "$autonomy_pid" autonomy
  wait_for_yolo_ready "$autonomy_pid" "$log_dir/autonomy.log" "${YOLO_READY_TIMEOUT_SEC:-90}"
  printf '[run_simulation] READY YOLO\n' >>"$log_dir/launcher.log"
  track_essential_process autonomy "$autonomy_pid"
  if [[ "$preflight_order" == preflight-first ]]; then
    printf '[run_simulation] YOLO readiness confirmed; waiting for C++ preflight: %s\n' \
      "$preflight_ready_file" >>"$log_dir/launcher.log"
    wait_for_file "$preflight_ready_file" 45 preflight-ready "$autonomy_pid"
    printf '[run_simulation] READY PREFLIGHT: %s\n' "$preflight_ready_file" >>"$log_dir/launcher.log"
  else
    printf '[run_simulation] mission-first diagnostic: C++ control is gated until mission setup; preflight remains mandatory afterward\n' \
      >>"$log_dir/launcher.log"
  fi
fi

if [[ "$with_mission_planner" == 1 ]]; then
  mission_binary="$MISSION_PLANNER_DIR/MissionPlanner.exe"
  [[ -f "$mission_binary" ]] || { printf 'Mission Planner not found: %s\n' "$mission_binary" >&2; exit 1; }
  [[ -x "$(command -v mono)" ]] || { printf '%s\n' 'mono is required for Mission Planner' >&2; exit 1; }
  start_process mission-planner \
    env -u WAYLAND_DISPLAY DISPLAY="${DISPLAY:-}" \
    mono "$mission_binary"
  mission_pid="${started_pid:-}"
  printf '[run_simulation] Mission Planner UDP endpoint: 127.0.0.1:14552\n' >>"$log_dir/launcher.log"
fi

if [[ "$upload_mission" == 1 ]]; then
  mission_cmd=(
    "$HOME/venv-ardupilot/bin/python"
    "$SCRIPT_DIR/upload_mission.py"
    --connect "udp:127.0.0.1:$GCS_UDP_PORT"
    "${mission_args[@]}"
    --auto
  )
  [[ "$arm" == 1 ]] && mission_cmd+=(--arm)
  printf '[run_simulation] explicit sitl-flight mission command\n' >>"$log_dir/launcher.log"
  "${mission_cmd[@]}" >"$log_dir/upload_mission.log" 2>&1 &
  mission_pid="$!"
  wait "$mission_pid"
fi

if [[ "$profile" == sitl-flight && "$preflight_order" == mission-first ]]; then
  # Open the autonomy gate only after the legacy mission/AUTO/ARM sequence has
  # completed. C++ preflight still runs after this point and is not bypassed.
  : >"$mission_setup_complete_file"
  printf '[run_simulation] mission-first setup complete; opened C++ control gate: %s\n' \
    "$mission_setup_complete_file" >>"$log_dir/launcher.log"
  wait_for_file "$preflight_ready_file" 45 preflight-ready "$autonomy_pid"
  printf '[run_simulation] READY PREFLIGHT: %s\n' "$preflight_ready_file" >>"$log_dir/launcher.log"
fi

# The full YOLO scenario must not let a ground-level camera detection start
# the handoff state before takeoff. This helper is read-only: it only consumes
# SITL telemetry and never sends a MAVLink command.
if [[ "$scenario" == yolo-target-centered-hold || "$scenario" == yolo-lateral-path ]]; then
  printf '%s\n' '[run_simulation] waiting for scenario takeoff altitude before opening YOLO control gate' >>"$log_dir/launcher.log"
  if ! "$HOME/venv-ardupilot/bin/python" \
      "$SCRIPT_DIR/wait_sitl_altitude.py" \
      --connect "udp:127.0.0.1:$GCS_UDP_PORT" \
      --min-altitude 0.6 \
      --timeout 30 \
      >"$log_dir/scenario-altitude-gate.log" 2>&1; then
    printf '[run_simulation] ERROR: scenario altitude gate failed; inspect %s\n' \
      "$log_dir/scenario-altitude-gate.log" >>"$log_dir/launcher.log"
    printf '[실패] 시나리오 이륙 고도 확인에 실패했습니다.\n[로그] %s\n' "$log_dir" >&2
    exit 1
  fi
  cat "$log_dir/scenario-altitude-gate.log" >>"$log_dir/launcher.log"
fi

# Release the control automation session only after mission setup. In the
# default order this follows a completed preflight; in the legacy diagnostic
# order the gate above starts C++ first, then this marker releases its session.
if [[ "$profile" == sitl-flight ]]; then
  if [[ -z "$mission_setup_complete_file" ]]; then
    printf '%s\n' '[run_simulation] ERROR: sitl-flight mission setup marker was not initialized.' >>"$log_dir/launcher.log"
    printf '[실패] SITL 미션 setup marker가 준비되지 않았습니다.\n[로그] %s\n' "$log_dir" >&2
    exit 1
  fi
  if [[ "$preflight_order" == preflight-first ]]; then
    : >"$mission_setup_complete_file"
    printf '[run_simulation] mission setup complete; opened control session: %s\n' \
      "$mission_setup_complete_file" >>"$log_dir/launcher.log"
  fi
fi

printf '[run_simulation] READY profile=%s logs=%s\n' "$profile" "$log_dir" >>"$log_dir/launcher.log"
if [[ "$profile" == observe ]]; then
  printf '%s\n' '[run_simulation] observe only: no YOLO, control, mission upload, or ARM started.' >>"$log_dir/launcher.log"
elif [[ "$profile" == shadow ]]; then
  printf '%s\n' '[run_simulation] shadow: C++ YOLO/control calculation runs; UDP vehicle commands are dropped.' >>"$log_dir/launcher.log"
else
  printf '%s\n' '[run_simulation] sitl-flight: loopback SITL vehicle commands are enabled explicitly.' >>"$log_dir/launcher.log"
fi

if (( duration > 0 )); then
  deadline=$((SECONDS + duration))
  while (( SECONDS < deadline )); do
    monitor_essential_processes || exit 1
    (( normal_completion == 1 )) && break
    sleep 1
  done
else
  while true; do
    monitor_essential_processes || exit 1
    (( normal_completion == 1 )) && break
    sleep 1
  done
fi

printf '[run_simulation] EVENT COMPLETE profile=%s\n' "$profile" >>"$log_dir/launcher.log"
if (( status_monitor == 1 )); then
  sleep 0.6
fi
