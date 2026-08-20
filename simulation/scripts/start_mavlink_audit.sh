#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=_common.sh
source "$SCRIPT_DIR/_common.sh"

stage_start_ms="$(date +%s%3N)"
if [[ -n "$MAVLINK_AUDIT_PYTHON" ]]; then
  require_executable "$MAVLINK_AUDIT_PYTHON"
  # Keep the validated venv entry point. Resolving its symlink to
  # /usr/bin/python would discard the venv sys.prefix and pymavlink install.
  python="$MAVLINK_AUDIT_PYTHON"
  sim_log "AUDIT_WRAPPER_TIMING dependency_mode=launcher-prevalidated elapsed_ms=$(( $(date +%s%3N) - stage_start_ms ))"
else
  mavproxy_path="$(find_mavproxy)" || sim_die 'audit dependency validation failure: MAVProxy venv is required for pymavlink parsing'
  find_done_ms="$(date +%s%3N)"
  verify_mavproxy "$mavproxy_path" || sim_die "audit dependency validation failure: MAVProxy/pymavlink verification failed: $mavproxy_path"
  verify_done_ms="$(date +%s%3N)"
  python="$(mavproxy_python "$mavproxy_path")"
  sim_log "AUDIT_WRAPPER_TIMING dependency_mode=standalone find_ms=$((find_done_ms-stage_start_ms)) verify_ms=$((verify_done_ms-find_done_ms)) total_ms=$((verify_done_ms-stage_start_ms))"
fi
audit_program="$SIMULATION_DIR/tests/mavlink_audit_proxy.py"
require_file "$audit_program"
require_port MAVLINK_AUDIT_TCP_PORT "$MAVLINK_AUDIT_TCP_PORT"
upstream="tcp:127.0.0.1:$SITL_MASTER_TCP_PORT"
listen="tcp:127.0.0.1:$MAVLINK_AUDIT_TCP_PORT"
require_loopback_endpoint audit-upstream "$upstream"
require_loopback_endpoint audit-listen "$listen"
port_in_use "$MAVLINK_AUDIT_TCP_PORT" && sim_die "audit TCP port is already in use: $MAVLINK_AUDIT_TCP_PORT"

cmd=("$python" "$audit_program" --upstream "$upstream" --listen "$listen" --report "$MAVLINK_AUDIT_REPORT")
if [[ -n "${MAVLINK_AUDIT_EVENT_FILE:-}" ]]; then
  cmd+=(--control-events "$MAVLINK_AUDIT_EVENT_FILE")
fi
sim_log "transparent audit relay: MAVProxy $listen -> SITL $upstream"
sim_log "audit report=$MAVLINK_AUDIT_REPORT"
printf '[simulation] command:'; printf ' %q' "${cmd[@]}"; printf '\n'
sim_log "AUDIT_WRAPPER_TIMING exec_ready_ms=$(( $(date +%s%3N) - stage_start_ms ))"
run_foreground_tracked audit "$audit_program" "${cmd[@]}"
