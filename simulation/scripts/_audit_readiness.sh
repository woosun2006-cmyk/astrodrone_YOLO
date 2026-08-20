#!/usr/bin/env bash

# Shared, process-aware readiness checks for the MAVLink audit relay.
# shellcheck shell=bash

audit_now_ms() { date +%s%3N; }

audit_timing_log() {
  sim_log "$*"
  if [[ -n ${AUDIT_STARTUP_TIMING_LOG:-} ]]; then
    printf '[simulation] %s\n' "$*" >>"$AUDIT_STARTUP_TIMING_LOG"
  fi
}

validate_audit_dependencies() {
  local total_start find_start find_end verify_start verify_end import_start import_end
  local mavproxy_path python_path
  total_start="$(audit_now_ms)"
  find_start="$total_start"
  mavproxy_path="$(find_mavproxy)" || {
    sim_die 'audit dependency validation failure: MAVProxy not found'
  }
  find_end="$(audit_now_ms)"
  verify_start="$find_end"
  verify_mavproxy "$mavproxy_path" || {
    sim_die "audit dependency validation failure: MAVProxy/version/pymavlink check failed: $mavproxy_path"
  }
  verify_end="$(audit_now_ms)"
  python_path="$(mavproxy_python "$mavproxy_path")" || {
    sim_die "audit dependency validation failure: Python shebang is invalid: $mavproxy_path"
  }
  import_start="$verify_end"
  "$python_path" -c \
    'from pymavlink import mavutil; from pymavlink.dialects.v20 import ardupilotmega' \
    >/dev/null 2>&1 || {
      sim_die "audit dependency validation failure: audit pymavlink dialect import failed: $python_path"
    }
  import_end="$(audit_now_ms)"
  AUDIT_VALIDATED_MAVPROXY="$mavproxy_path"
  AUDIT_VALIDATED_PYTHON="$python_path"
  audit_timing_log "AUDIT_DEPENDENCY_TIMING find_mavproxy_ms=$((find_end-find_start)) verify_mavproxy_ms=$((verify_end-verify_start)) audit_import_ms=$((import_end-import_start)) total_ms=$((import_end-total_start))"
}

audit_tcp_listener_exists() {
  local port="$1"
  ss -H -ltn 2>/dev/null | awk -v port="$port" '$4 ~ (":" port "$") {found=1} END {exit !found}'
}

wait_for_audit_ready() {
  local wrapper_pid="$1" log_file="$2" error_file="$3" port="$4" timeout="$5"
  local started now elapsed listen_logged=0 socket_ready=0 wrapper_rc=0 category
  started="$(audit_now_ms)"
  while true; do
    [[ -f "$log_file" ]] && grep -q "^AUDIT_LISTEN tcp:127\.0\.0\.1:${port}$" "$log_file" &&
      listen_logged=1 || listen_logged=0
    audit_tcp_listener_exists "$port" && socket_ready=1 || socket_ready=0
    now="$(audit_now_ms)"
    elapsed=$((now-started))

    if (( listen_logged == 1 && socket_ready == 1 )); then
      audit_timing_log "AUDIT_READY elapsed_ms=$elapsed log_marker=1 tcp_listener=1 wrapper_pid=$wrapper_pid"
      return 0
    fi

    if ! kill -0 "$wrapper_pid" 2>/dev/null; then
      if wait "$wrapper_pid" 2>/dev/null; then wrapper_rc=0; else wrapper_rc=$?; fi
      if [[ -f "$error_file" ]] && grep -Eqi 'address already in use|bind.*fail|port.*in use' "$error_file"; then
        category='BIND_FAILURE_OR_PORT_COLLISION'
      elif [[ -f "$error_file" ]] && grep -Eqi \
          'dependency validation failure|ModuleNotFoundError|No module named|ImportError' "$error_file"; then
        category='DEPENDENCY_VALIDATION_FAILURE'
      else
        category='AUDIT_PROCESS_EARLY_EXIT'
      fi
      sim_warn "AUDIT_START_FAILURE category=$category elapsed_ms=$elapsed wrapper_rc=$wrapper_rc log_marker=$listen_logged tcp_listener=$socket_ready"
      [[ -s "$error_file" ]] && { sim_warn 'audit stderr follows:'; tail -20 "$error_file" >&2; }
      return 1
    fi

    if (( elapsed >= timeout * 1000 )); then
      if (( listen_logged == 1 )); then
        category='AUDIT_LISTEN_LOG_WITHOUT_SOCKET'
      else
        category='AUDIT_READINESS_TIMEOUT'
      fi
      sim_warn "AUDIT_START_FAILURE category=$category elapsed_ms=$elapsed wrapper_alive=1 log_marker=$listen_logged tcp_listener=$socket_ready timeout_s=$timeout"
      [[ -s "$error_file" ]] && { sim_warn 'audit stderr follows:'; tail -20 "$error_file" >&2; }
      return 1
    fi
    sleep 0.1
  done
}
