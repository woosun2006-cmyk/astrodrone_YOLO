#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=_common.sh
source "$SCRIPT_DIR/_common.sh"

: "${CAMERA_SMOKE_RUN_ID:=$(date +%Y%m%d_%H%M%S)}"
: "${CAMERA_FRAME_TIMEOUT:=15}"
: "${CAMERA_SAMPLE_SECONDS:=3}"
: "${CAMERA_SIM_WORLD:=ensamb_iris_runway.sdf}"
: "${CAMERA_SIM_MODEL:=ensamb_with_gimbal}"
: "${CAMERA_BODY_MODEL:=ensamb_with_standoffs}"
: "${CAMERA_LINK:=down_camera_link}"
: "${CAMERA_SENSOR:=down_camera}"

# This script is the project camera scenario. The general SITL smoke keeps the
# upstream defaults from _common.sh; only this process and its children use the
# explicitly selected custom world and model.
SIM_WORLD="$CAMERA_SIM_WORLD"
SIM_MODEL="$CAMERA_SIM_MODEL"
export SIM_WORLD SIM_MODEL

log_dir="$ASTRODRONE_REPO/simulation/logs/camera_smoke_${CAMERA_SMOKE_RUN_ID}"
receiver_source="$ASTRODRONE_REPO/simulation/tools/camera_frame_receiver.cpp"
pose_probe_source="$ASTRODRONE_REPO/simulation/tools/down_camera_pose_probe.cpp"
receiver_build_dir="$SIM_RUNTIME_DIR/camera-smoke-tools"
receiver="$receiver_build_dir/camera_frame_receiver"
pose_probe="$receiver_build_dir/down_camera_pose_probe"
sample_image="$log_dir/sample.ppm"
gazebo_log="$log_dir/gazebo.log"
gazebo_error_log="$log_dir/gazebo.err.log"
topic_list_log="$log_dir/topics.log"
topic_name_candidates_log="$log_dir/camera_topic_names.log"
topic_candidates_log="$log_dir/camera_topic_candidates.log"
topic_info_log="$log_dir/image_topic_info.log"
frame_log="$log_dir/frame.log"
runtime_pose_log="$log_dir/runtime_pose.log"
summary_log="$log_dir/summary.log"
world_file="$(resolve_world_file)"
vehicle_model_file="$(resolve_model_file)"
camera_body_model_file="$SIMULATION_DIR/models/$CAMERA_BODY_MODEL/model.sdf"
target_model_file="$SIMULATION_DIR/models/target_basket/model.sdf"

require_command gz
require_command pkg-config
require_command g++
require_command rg
require_command file
require_command python3
require_file "$receiver_source"
require_file "$pose_probe_source"
require_file "$world_file"
require_file "$vehicle_model_file"
require_file "$camera_body_model_file"
require_file "$target_model_file"
pkg-config --exists gz-transport gz-msgs ||
  sim_die 'installed gz-transport and gz-msgs development packages are required'
require_uint CAMERA_FRAME_TIMEOUT "$CAMERA_FRAME_TIMEOUT"
require_uint CAMERA_SAMPLE_SECONDS "$CAMERA_SAMPLE_SECONDS"
(( CAMERA_FRAME_TIMEOUT > 0 )) || sim_die 'CAMERA_FRAME_TIMEOUT must be positive'

grep -q "<uri>model://$SIM_MODEL</uri>" "$world_file" ||
  sim_die "selected world does not include vehicle model $SIM_MODEL: $world_file"
grep -q "<uri>model://$CAMERA_BODY_MODEL</uri>" "$vehicle_model_file" ||
  sim_die "vehicle model does not include camera body model $CAMERA_BODY_MODEL"

expected_local_world="$(realpath -e -- "$SIMULATION_DIR/worlds/$CAMERA_SIM_WORLD")"
expected_local_vehicle="$(realpath -e -- "$SIMULATION_DIR/models/$CAMERA_SIM_MODEL/model.sdf")"
[[ "$(realpath -e -- "$world_file")" == "$expected_local_world" ]] ||
  sim_die "custom camera world must resolve inside the repository: actual=$world_file expected=$expected_local_world"
[[ "$(realpath -e -- "$vehicle_model_file")" == "$expected_local_vehicle" ]] ||
  sim_die "custom camera vehicle must resolve inside the repository: actual=$vehicle_model_file expected=$expected_local_vehicle"
for local_asset in "$world_file" "$vehicle_model_file" "$camera_body_model_file" "$target_model_file"; do
  [[ ! -L "$local_asset" ]] || sim_die "repository custom asset must not be an external symlink: $local_asset"
done

world_name="$(sed -n 's/.*<world[[:space:]][^>]*name="\([^"]*\)".*/\1/p' "$world_file" | head -1)"
[[ -n "$world_name" ]] || sim_die "could not resolve world name from $world_file"
metadata="$(python3 -c '
import sys, xml.etree.ElementTree as ET
model_file, link_name, sensor_name = sys.argv[1:]
root = ET.parse(model_file).getroot()
sensor = root.find(f".//link[@name=\"{link_name}\"]/sensor[@name=\"{sensor_name}\"]")
if sensor is None or sensor.get("type") != "camera":
    raise SystemExit(1)
camera = sensor.find("camera")
values = [sensor.findtext("pose", ""), camera.findtext("image/width", ""),
          camera.findtext("image/height", ""),
          camera.findtext("horizontal_fov", "")]
if not all(values):
    raise SystemExit(1)
print("\t".join(values))
' "$camera_body_model_file" "$CAMERA_LINK" "$CAMERA_SENSOR")" ||
  sim_die 'could not resolve selected camera metadata from the SDF include chain'
IFS=$'\t' read -r camera_pose camera_width camera_height camera_fov <<<"$metadata"
read -r camera_x camera_y camera_z camera_roll camera_pitch camera_yaw <<<"$camera_pose"
[[ "$camera_width" == 640 && "$camera_height" == 480 ]] ||
  sim_die "custom down_camera must be 640x480 (got ${camera_width}x${camera_height})"
expected_image_topic="/world/$world_name/model/$SIM_MODEL/model/$CAMERA_BODY_MODEL/link/$CAMERA_LINK/sensor/$CAMERA_SENSOR/image"

[[ ! -e "$SIM_RUNTIME_DIR/gazebo.pid" && ! -e "$SIM_RUNTIME_DIR/sitl.pid" &&
   ! -e "$SIM_RUNTIME_DIR/audit.pid" && ! -e "$SIM_RUNTIME_DIR/router.pid" ]] ||
  sim_die 'tracked simulation PID files already exist; stop or inspect the existing run first'

mkdir -p -- "$log_dir" "$receiver_build_dir"
wrappers=()
helpers=()
cleaned=0
cleanup() {
  if [[ "$cleaned" == 0 ]]; then
    "$SCRIPT_DIR/stop_simulation.sh" || true
    cleaned=1
  fi
  local pid
  for pid in "${wrappers[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
  for pid in "${helpers[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

wait_for_text() {
  local file="$1" text_value="$2" timeout="$3" label="$4"
  local deadline=$((SECONDS + timeout))
  while (( SECONDS < deadline )); do
    [[ -f "$file" ]] && grep -q "$text_value" "$file" && {
      sim_log "$label confirmed"
      return 0
    }
    [[ ${wrappers[0]+x} ]] && kill -0 "${wrappers[0]}" 2>/dev/null ||
      sim_die "$label failed because Gazebo exited"
    sleep 1
  done
  sim_die "$label was not observed in $file within ${timeout}s"
}

count_publishers() {
  awk '
    /^Publishers / {inside=1; next}
    /^(Subscribers |No subscribers)/ {inside=0}
    inside && /^[[:space:]]+(tcp|udp):\/\// {count++}
    END {print count+0}
  ' "$1"
}

sim_log "camera smoke run id=$CAMERA_SMOKE_RUN_ID"
sim_log 'scenario=project-custom-downward-camera'
sim_log "selected_world=$world_file"
sim_log "selected_vehicle_model=$SIM_MODEL"
sim_log "resolved_vehicle_sdf=$vehicle_model_file"
sim_log "resolved_camera_body_sdf=$camera_body_model_file"
sim_log "resolved_target_sdf=$target_model_file"
sim_log "camera_owner_model=$CAMERA_BODY_MODEL camera_link=$CAMERA_LINK camera_sensor=$CAMERA_SENSOR"
sim_log "camera_sdf_pose=$camera_pose image=${camera_width}x${camera_height} horizontal_fov=$camera_fov"
"$SCRIPT_DIR/check_environment.sh" >"$log_dir/environment.log"

# Build only in /tmp through SIM_RUNTIME_DIR; no repository build artifact is made.
# shellcheck disable=SC2046
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic "$receiver_source" \
  -o "$receiver" $(pkg-config --cflags --libs gz-transport gz-msgs)
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic "$pose_probe_source" \
  -o "$pose_probe" $(pkg-config --cflags --libs gz-transport gz-msgs)

GAZEBO_HEADLESS=1 DISPLAY= WAYLAND_DISPLAY= "$SCRIPT_DIR/start_gazebo.sh" \
  >"$gazebo_log" 2>"$gazebo_error_log" &
wrappers+=("$!")
wait_for_text "$gazebo_log" 'Enabling camera sensor' 30 Gazebo-camera

gz model --list >"$log_dir/model_list.log" 2>&1
grep -Eq '^[[:space:]]+- ensamb_with_gimbal$' "$log_dir/model_list.log" ||
  sim_die "selected vehicle model was not created; see $log_dir/model_list.log"
grep -Eq '^[[:space:]]+- target_basket$' "$log_dir/model_list.log" ||
  sim_die "target_basket model was not created; see $log_dir/model_list.log"

gz topic -l | sort >"$topic_list_log"
rg -i 'camera|image' "$topic_list_log" >"$topic_name_candidates_log" ||
  sim_die "no camera/image topic name found in actual topic list: $topic_list_log"

image_topics=()
: >"$topic_info_log"
while IFS= read -r candidate; do
  candidate_info="$(gz topic -i -t "$candidate" 2>&1 || true)"
  if awk '
      /^Publishers / {inside=1; next}
      /^(Subscribers |No subscribers)/ {inside=0}
      inside && /gz\.msgs\.Image$/ {found=1}
      END {exit !found}
    ' <<<"$candidate_info"; then
    image_topics+=("$candidate")
    {
      printf '===== %s\n' "$candidate"
      printf '%s\n' "$candidate_info"
    } >>"$topic_info_log"
  fi
done <"$topic_name_candidates_log"
printf '%s\n' "${image_topics[@]}" >"$topic_candidates_log"
if (( ${#image_topics[@]} != 1 )); then
  sim_die "expected exactly one gz.msgs.Image publisher, found ${#image_topics[@]}; candidates and owners: $topic_candidates_log"
fi
image_topic="${image_topics[0]}"
[[ "$image_topic" == "$expected_image_topic" ]] ||
  sim_die "Image publisher ownership does not match selected SDF chain: actual=$image_topic expected=$expected_image_topic"

selected_info="$log_dir/selected_image_topic_info.log"
gz topic -i -t "$image_topic" >"$selected_info"
publisher_count="$(count_publishers "$selected_info")"
(( publisher_count > 0 )) || sim_die "image topic has no publisher: $image_topic"
message_type="$(awk -F', ' '/^[[:space:]]+(tcp|udp):\/\// && /gz\.msgs\.Image/{print $2; exit}' "$selected_info")"
[[ "$message_type" == 'gz.msgs.Image' ]] ||
  sim_die "unexpected image message type: ${message_type:-missing}"

mapfile -t pose_topics < <(rg '/dynamic_pose/info$' "$topic_list_log")
(( ${#pose_topics[@]} == 1 )) ||
  sim_die "expected exactly one runtime dynamic pose topic, found ${#pose_topics[@]}"
pose_topic="${pose_topics[0]}"
"$pose_probe" "$pose_topic" "$SIM_MODEL" "$CAMERA_BODY_MODEL" "$CAMERA_LINK" \
  "$camera_roll" "$camera_pitch" "$camera_yaw" 2 >"$runtime_pose_log" &
pose_probe_pid=$!
helpers+=("$pose_probe_pid")
"$receiver" "$image_topic" "$sample_image" \
  "$CAMERA_FRAME_TIMEOUT" "$CAMERA_SAMPLE_SECONDS" | tee "$frame_log"
wait "$pose_probe_pid"

body_axis="$(awk -F= '/^final_optical_axis_body=/{print $2}' "$runtime_pose_log")"
world_axis="$(awk -F= '/^final_optical_axis_world=/{print $2}' "$runtime_pose_log")"
[[ -n "$body_axis" && -n "$world_axis" ]] || sim_die 'runtime optical axis is missing'
IFS=, read -r body_axis_x body_axis_y body_axis_z <<<"$body_axis"
IFS=, read -r world_axis_x world_axis_y world_axis_z <<<"$world_axis"
awk -v x="$body_axis_x" -v y="$body_axis_y" -v z="$body_axis_z" \
  'BEGIN {exit !(x > -0.01 && x < 0.01 && y > -0.01 && y < 0.01 && z < -0.999)}' ||
  sim_die "camera optical axis is not body -Z: $body_axis"
awk -v z="$world_axis_z" 'BEGIN {exit !(z < -0.99)}' ||
  sim_die "camera optical axis is not downward in world coordinates: $world_axis"

sim_log "actual_camera_topic=$image_topic publisher_count=$publisher_count type=$message_type"
sim_log "runtime_optical_axis_body=$body_axis runtime_optical_axis_world=$world_axis"

grep -q '^width=[1-9][0-9]*$' "$frame_log" || sim_die 'received image width is zero or missing'
grep -q '^height=[1-9][0-9]*$' "$frame_log" || sim_die 'received image height is zero or missing'
grep -q "^width=$camera_width$" "$frame_log" || sim_die 'received width does not match selected SDF'
grep -q "^height=$camera_height$" "$frame_log" || sim_die 'received height does not match selected SDF'
grep -Eq '^pixel_format=(L_INT8|RGB_INT8|BGR_INT8|RGBA_INT8|BGRA_INT8)$' "$frame_log" ||
  sim_die 'received image pixel format is unsupported'
grep -q '^all_black=false$' "$frame_log" || sim_die 'sample image is entirely black'
[[ -s "$sample_image" ]] || sim_die "sample image was not saved: $sample_image"
file "$sample_image" | tee "$log_dir/sample_file_type.log"
grep -q 'Netpbm image data' "$log_dir/sample_file_type.log" ||
  sim_die 'saved sample does not open as a Netpbm image'

"$SCRIPT_DIR/stop_simulation.sh"
cleaned=1
for pid in "${wrappers[@]}"; do wait "$pid" 2>/dev/null || true; done

remaining_port="$(ss -H -lntup 2>/dev/null | awk '{print $5}' |
  grep -E ":${GAZEBO_FDM_PORT}$" || true)"
[[ -z "$remaining_port" ]] || sim_die "Gazebo FDM port remains after cleanup: $remaining_port"
remaining_pid_files="$(find "$SIM_RUNTIME_DIR" -maxdepth 1 -type f -name '*.pid' -print 2>/dev/null || true)"
[[ -z "$remaining_pid_files" ]] || sim_die "tracked PID files remain after cleanup: $remaining_pid_files"
remaining_processes="$(ps -eo args= | grep -F "$ARDUPILOT_GAZEBO_DIR/worlds/$(basename -- "$(resolve_world_file)")" |
  grep -v grep || true)"
[[ -z "$remaining_processes" ]] || sim_die "Gazebo processes remain after cleanup: $remaining_processes"

{
  printf 'scenario=project-custom-downward-camera\n'
  printf 'selected_world=%s\n' "$world_file"
  printf 'spawned_vehicle_model=%s\n' "$SIM_MODEL"
  printf 'resolved_vehicle_sdf=%s\n' "$vehicle_model_file"
  printf 'resolved_camera_body_sdf=%s\n' "$camera_body_model_file"
  printf 'resolved_target_sdf=%s\n' "$target_model_file"
  printf 'target_basket_created=true\n'
  printf 'camera_owner_model=%s\n' "$CAMERA_BODY_MODEL"
  printf 'camera_link=%s\n' "$CAMERA_LINK"
  printf 'camera_sensor=%s\n' "$CAMERA_SENSOR"
  printf 'camera_topic=%s\n' "$image_topic"
  printf 'sdf_camera_pose=%s\n' "$camera_pose"
  printf 'sdf_image_width=%s\n' "$camera_width"
  printf 'sdf_image_height=%s\n' "$camera_height"
  printf 'sdf_horizontal_fov=%s\n' "$camera_fov"
  printf 'optical_axis_body=%s\n' "$body_axis"
  printf 'optical_axis_world=%s\n' "$world_axis"
  grep -E '^final_camera_(link_world_position|link_world_quaternion|sensor_world_quaternion)=' "$runtime_pose_log"
  printf 'camera_smoke=PASS\n'
  printf 'fixture=Gazebo-only\n'
  printf 'image_topic=%s\n' "$image_topic"
  printf 'message_type=%s\n' "$message_type"
  printf 'publisher_count=%s\n' "$publisher_count"
  grep -E '^(width|height|pixel_format|message_timestamp|first_frame_wall_time|first_frame_latency_ms|frame_count|receive_span_sec|fps|pixel_min|pixel_max|pixel_mean|all_black|sample_image)=' "$frame_log"
  printf 'vehicle_affecting_command_count=0\n'
  printf 'mavlink_processes_started=0\n'
  printf 'cleanup_processes=0\n'
  printf 'cleanup_pid_files=0\n'
  printf 'cleanup_fdm_ports=0\n'
} | tee "$summary_log"

sim_log "CAMERA SMOKE PASS logs=$log_dir sample=$sample_image"
trap - EXIT INT TERM
