#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=../scripts/_common.sh
source "$SCRIPT_DIR/../scripts/_common.sh"
# shellcheck source=../scripts/_audit_readiness.sh
source "$SCRIPT_DIR/../scripts/_audit_readiness.sh"

test_dir="$(mktemp -d /tmp/astrodrone-audit-readiness.XXXXXX)"
test_port=15770
log_file="$test_dir/audit.log"
error_file="$test_dir/audit.err.log"
helper_pid=''
cleanup() {
  if [[ -n "$helper_pid" ]] && kill -0 "$helper_pid" 2>/dev/null; then
    kill -TERM "$helper_pid"
    wait "$helper_pid" 2>/dev/null || true
  fi
  [[ -f "$log_file" ]] && unlink "$log_file"
  [[ -f "$error_file" ]] && unlink "$error_file"
  rmdir "$test_dir"
}
trap cleanup EXIT

port_in_use "$test_port" && sim_die "test port already in use: $test_port"
python3 "$SCRIPT_DIR/delayed_tcp_listener.py" --port "$test_port" --delay 2 \
  >"$log_file" 2>"$error_file" &
helper_pid=$!
started="$(audit_now_ms)"
wait_for_audit_ready "$helper_pid" "$log_file" "$error_file" "$test_port" 5
elapsed=$(( $(audit_now_ms) - started ))
(( elapsed >= 1900 && elapsed < 5000 )) || sim_die "unexpected readiness elapsed_ms=$elapsed"
sim_log "AUDIT_SLOW_START_TEST PASS delay_ms=2000 readiness_ms=$elapsed false_timeout=0"
