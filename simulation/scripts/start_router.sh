#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=_common.sh
source "$SCRIPT_DIR/_common.sh"

master="$MAVPROXY_MASTER_ENDPOINT"
control_out="udpout:127.0.0.1:$CONTROL_UDP_PORT"
telemetry_out="udpout:127.0.0.1:$TELEMETRY_UDP_PORT"
gcs_out="udpout:127.0.0.1:$GCS_UDP_PORT"
gcs_telemetry_out="udpout:127.0.0.1:$GCS_TELEMETRY_UDP_PORT"
for endpoint in "$master" "$telemetry_out"; do
  require_loopback_endpoint router "$endpoint"
done

mavproxy_path="$(find_mavproxy)" || sim_die 'MAVProxy not found (set MAVPROXY_BIN or provide it in PATH/a known ArduPilot venv)'
verify_mavproxy "$mavproxy_path" || sim_die "MAVProxy executable/version/pymavlink verification failed: $mavproxy_path"
[[ "$MAVPROXY_STREAMRATE" =~ ^[1-9][0-9]*$ ]] || sim_die 'MAVPROXY_STREAMRATE must be a positive integer'
[[ "$ENABLE_CONTROL_OUTPUT" =~ ^[01]$ ]] || sim_die 'ENABLE_CONTROL_OUTPUT must be 0 or 1'
[[ "$ENABLE_GCS_OUTPUT" =~ ^[01]$ ]] || sim_die 'ENABLE_GCS_OUTPUT must be 0 or 1'
[[ "$ENABLE_GCS_TELEMETRY_OUTPUT" =~ ^[01]$ ]] || sim_die 'ENABLE_GCS_TELEMETRY_OUTPUT must be 0 or 1'
if [[ "$ENABLE_GCS_TELEMETRY_OUTPUT" == 1 ]]; then
  require_loopback_endpoint GCS_TELEMETRY "$gcs_telemetry_out"
fi

cmd=("$mavproxy_path" --master="$master" --out="$telemetry_out"
  --streamrate="$MAVPROXY_STREAMRATE" --heartbeat-rate=1 --default-modules=link
  --non-interactive --no-state)
outputs="telemetry=$telemetry_out"
if [[ "$ENABLE_CONTROL_OUTPUT" == 1 ]]; then
  require_loopback_endpoint control "$control_out"
  cmd+=(--out="$control_out")
  outputs+=" control=$control_out"
fi
if [[ "$ENABLE_GCS_OUTPUT" == 1 ]]; then
  require_loopback_endpoint GCS "$gcs_out"
  cmd+=(--out="$gcs_out")
  outputs+=" GCS=$gcs_out"
fi
if [[ "$ENABLE_GCS_TELEMETRY_OUTPUT" == 1 ]]; then
  cmd+=(--out="$gcs_telemetry_out")
  outputs+=" GCS_TELEMETRY=$gcs_telemetry_out"
fi
version="$($mavproxy_path --version 2>&1 | awk -F': ' '/MAVProxy Version:/{print $2; exit}')"
sim_log "router=MAVProxy $version path=$mavproxy_path source=$master"
sim_log "outputs: $outputs"
sim_log 'MAVProxy may send GCS HEARTBEAT and REQUEST_DATA_STREAM; it is not the receive-only executable'
printf '[simulation] command:'; printf ' %q' "${cmd[@]}"; printf '\n'
run_foreground_tracked router "$mavproxy_path" "${cmd[@]}"
