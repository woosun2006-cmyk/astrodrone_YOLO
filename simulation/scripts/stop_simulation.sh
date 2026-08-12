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
for name in router audit sitl gazebo; do
  pid_file="$SIM_RUNTIME_DIR/$name.pid"
  [[ -f "$pid_file" ]] || continue
  IFS=$'\t' read -r pid started expected <"$pid_file" || {
    sim_warn "unreadable PID file left untouched: $pid_file"
    continue
  }
  if [[ ! "$pid" =~ ^[0-9]+$ || ! -r "/proc/$pid/stat" ]]; then
    sim_warn "$name PID $pid is no longer running; removing stale PID file"
    rm -f -- "$pid_file"
    continue
  fi
  current_started="$(pid_start_time "$pid" || true)"
  cmdline="$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)"
  if [[ "$current_started" != "$started" || -z "$expected" || "$cmdline" != *"$expected"* ]]; then
    sim_warn "identity mismatch for PID $pid; refusing to terminate it ($cmdline)"
    continue
  fi
  pgid="$(ps -o pgid= -p "$pid" | tr -d ' ')"
  if [[ "$pgid" != "$pid" ]]; then
    sim_warn "PID $pid is not its recorded process-group leader; refusing to terminate it"
    continue
  fi
  sim_log "sending SIGTERM to tracked $name process group $pgid"
  kill -TERM -- "-$pgid"
  stopped=$((stopped + 1))
  for _ in {1..50}; do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 "$pid" 2>/dev/null; then
    sim_warn "$name PID $pid did not exit; no SIGKILL was sent"
  else
    rm -f -- "$pid_file"
  fi
done
sim_log "stopped $stopped tracked process(es); no name-based pkill was used"
