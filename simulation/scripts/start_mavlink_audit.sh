#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=_common.sh
source "$SCRIPT_DIR/_common.sh"

mavproxy_path="$(find_mavproxy)" || sim_die 'MAVProxy venv is required for pymavlink audit parsing'
verify_mavproxy "$mavproxy_path" || sim_die "MAVProxy/pymavlink verification failed: $mavproxy_path"
python="$(mavproxy_python "$mavproxy_path")"
audit_program="$SIMULATION_DIR/tests/mavlink_audit_proxy.py"
require_file "$audit_program"
require_port MAVLINK_AUDIT_TCP_PORT "$MAVLINK_AUDIT_TCP_PORT"
upstream="tcp:127.0.0.1:$SITL_MASTER_TCP_PORT"
listen="tcp:127.0.0.1:$MAVLINK_AUDIT_TCP_PORT"
require_loopback_endpoint audit-upstream "$upstream"
require_loopback_endpoint audit-listen "$listen"
port_in_use "$MAVLINK_AUDIT_TCP_PORT" && sim_die "audit TCP port is already in use: $MAVLINK_AUDIT_TCP_PORT"

cmd=("$python" "$audit_program" --upstream "$upstream" --listen "$listen" --report "$MAVLINK_AUDIT_REPORT")
sim_log "transparent audit relay: MAVProxy $listen -> SITL $upstream"
sim_log "audit report=$MAVLINK_AUDIT_REPORT"
printf '[simulation] command:'; printf ' %q' "${cmd[@]}"; printf '\n'
run_foreground_tracked audit "$audit_program" "${cmd[@]}"
