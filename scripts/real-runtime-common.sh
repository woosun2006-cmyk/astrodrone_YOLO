#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"
export ASTRODRONE_TARGET=real
# shellcheck disable=SC1091
source "$SCRIPT_DIR/project-env.sh"

export ASTRODRONE_REPO="$REPO_ROOT"
export ASTRODRONE_SETTINGS_DIR="${ASTRODRONE_SETTINGS_DIR:-$SETTING_DIR}"

require_loopback_endpoint() {
  local endpoint="$1"
  [[ "$endpoint" =~ ^(udp|tcp):127\.0\.0\.1:[0-9]+$ ]] || {
    printf '[실패] loopback endpoint가 아닙니다: %s\n' "$endpoint" >&2
    return 1
  }
}

require_serial_endpoint() {
  local endpoint="${1:-}"
  [[ "$endpoint" == /dev/serial/by-id/* && "$endpoint" != *REPLACE_WITH* ]] || {
    printf '[실패] serial endpoint는 /dev/serial/by-id/...만 허용합니다: %s\n' "$endpoint" >&2
    return 1
  }
  [[ -e "$endpoint" ]] || {
    printf '[실패] 지정한 serial endpoint가 존재하지 않습니다: %s\n' "$endpoint" >&2
    return 1
  }
}

wait_http_port() {
  local port="$1" timeout="$2"
  local deadline=$((SECONDS + timeout))
  while (( SECONDS < deadline )); do
    if ss -H -lnt 2>/dev/null | awk -v p=":$port" '$4 ~ p { found=1 } END { exit(found ? 0 : 1) }'; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}

write_zero_command_audit() {
  local report="$1" profile="$2"
  mkdir -p "$(dirname -- "$report")"
  printf '{"profile":"%s","vehicle_affecting_command_count":0,"decision_count":0,"blocked_command_count":0,"direct_serial_write_count":0,"set_message_interval_count":0,"source":"read-only-policy"}\n' \
    "$profile" >"$report"
}
