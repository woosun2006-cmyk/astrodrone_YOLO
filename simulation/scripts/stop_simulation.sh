#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=_common.sh
source "$SCRIPT_DIR/_common.sh"

[[ -d "$SIM_RUNTIME_DIR" ]] || {
  sim_log "no runtime directory; nothing to stop: $SIM_RUNTIME_DIR"
  exit 0
}

stopped=0
process_group_alive() {
  local pgid="$1"
  ps -o pid= -g "$pgid" 2>/dev/null | grep -q '[0-9]'
}
wait_process_group() {
  local pgid="$1" attempts="$2"
  local attempt_index
  for ((attempt_index=0; attempt_index<attempts; attempt_index++)); do
    process_group_alive "$pgid" || return 0
    sleep 0.1
  done
  ! process_group_alive "$pgid"
}
remove_pid_file() {
  local path="$1"
  [[ ! -e "$path" ]] || unlink "$path" || sim_warn "could not remove stale PID file: $path"
}
for name in router audit sitl gazebo; do
  pid_file="$SIM_RUNTIME_DIR/$name.pid"
  [[ -f "$pid_file" ]] || continue
  IFS=$'\t' read -r pid started expected <"$pid_file" || {
    sim_warn "unreadable PID file left untouched: $pid_file"
    continue
  }
  if [[ ! "$pid" =~ ^[0-9]+$ || ! -r "/proc/$pid/stat" ]]; then
    sim_warn "$name PID $pid is no longer running; removing stale PID file"
    remove_pid_file "$pid_file"
    continue
  fi
  current_started="$(pid_start_time "$pid" || true)"
  if [[ -z "$current_started" && ! -r "/proc/$pid/stat" ]]; then
    sim_warn "$name PID $pid exited during cleanup identity check; removing stale PID file"
    remove_pid_file "$pid_file"
    continue
  fi
  cmdline="$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)"
  if [[ -z "$cmdline" && ! -r "/proc/$pid/stat" ]]; then
    sim_warn "$name PID $pid exited during cleanup command check; removing stale PID file"
    remove_pid_file "$pid_file"
    continue
  fi
  if [[ "$current_started" != "$started" || -z "$expected" || "$cmdline" != *"$expected"* ]]; then
    sim_warn "identity mismatch for PID $pid; refusing to terminate it ($cmdline)"
    continue
  fi
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  if [[ -z "$pgid" && ! -r "/proc/$pid/stat" ]]; then
    sim_warn "$name PID $pid exited during cleanup process-group check; removing stale PID file"
    remove_pid_file "$pid_file"
    continue
  fi
  if [[ "$pgid" != "$pid" ]]; then
    sim_warn "PID $pid is not its recorded process-group leader; refusing to terminate it"
    continue
  fi
  sim_log "sending SIGTERM to tracked $name process group $pgid"
  if ! kill -TERM -- "-$pgid" 2>/dev/null; then
    if [[ ! -r "/proc/$pid/stat" ]]; then
      sim_warn "$name PID $pid exited before SIGTERM; removing stale PID file"
      remove_pid_file "$pid_file"
      continue
    fi
    sim_warn "could not signal tracked $name process group $pgid"
    continue
  fi
  stopped=$((stopped + 1))
  if ! wait_process_group "$pgid" 50; then
    sim_warn "$name process group $pgid ignored SIGTERM; sending SIGINT to that tracked group"
    kill -INT -- "-$pgid" 2>/dev/null || true
  fi
  if ! wait_process_group "$pgid" 20; then
    sim_warn "$name process group $pgid ignored graceful signals; sending SIGKILL to that tracked group only"
    kill -KILL -- "-$pgid" 2>/dev/null || true
  fi
  if wait_process_group "$pgid" 20; then
    remove_pid_file "$pid_file"
  else
    sim_warn "$name process group $pgid still exists after exact-group cleanup; PID file retained"
  fi
done
sim_log "stopped $stopped tracked process(es); no name-based pkill was used"
