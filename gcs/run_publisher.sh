#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
BUILD_DIR="${GCS_BUILD_DIR:-$SCRIPT_DIR/build}"
ENDPOINT="${MAVPROXY_GCS_TELEMETRY_ENDPOINT:-udp:127.0.0.1:14553}"

case "$ENDPOINT" in
  udp:127.0.0.1:[0-9]*) ;;
  *) printf '[실패] GCS telemetry publisher는 loopback UDP만 허용합니다: %s\n' "$ENDPOINT" >&2; exit 2 ;;
esac

cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=RelWithDebInfo >/dev/null
cmake --build "$BUILD_DIR" --target telem_sender --parallel "${GCS_BUILD_JOBS:-$(nproc)}" >/dev/null
exec "$BUILD_DIR/telem_sender" --telemetry-endpoint "$ENDPOINT" "$@"
