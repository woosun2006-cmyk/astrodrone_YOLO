#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
BUILD_DIR="${GCS_BUILD_DIR:-$SCRIPT_DIR/build}"
ENDPOINT="${ASTRODRONE_GCS_TELEMETRY_ENDPOINT:-udp:127.0.0.1:14553}"

case "$ENDPOINT" in
  udp:127.0.0.1:[0-9]*) ;;
  *) printf '[실패] GCS telemetry publisher는 loopback UDP만 허용합니다: %s\n' "$ENDPOINT" >&2; exit 2 ;;
esac

if [[ ! -f "$BUILD_DIR/CMakeCache.txt" ||
      "$SCRIPT_DIR/CMakeLists.txt" -nt "$BUILD_DIR/CMakeCache.txt" ]]; then
  cmake_version="$(cmake --version | sed -n '1s/.*version //p')"
  cmake_major="${cmake_version%%.*}"
  cmake_minor="${cmake_version#*.}"
  cmake_minor="${cmake_minor%%.*}"
  if (( cmake_major > 3 || (cmake_major == 3 && cmake_minor >= 13) )); then
    cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=RelWithDebInfo >/dev/null
  else
    (cd "$BUILD_DIR" && cmake "$SCRIPT_DIR" -DCMAKE_BUILD_TYPE=RelWithDebInfo >/dev/null)
  fi
fi

build_jobs="${GCS_BUILD_JOBS:-$(nproc)}"
if cmake --help 2>/dev/null | grep -q -- '--build <dir>'; then
  if cmake --help 2>/dev/null | grep -q -- '--parallel'; then
    cmake --build "$BUILD_DIR" --target telem_sender --parallel "$build_jobs" >/dev/null
  else
    cmake --build "$BUILD_DIR" --target telem_sender -- -j"$build_jobs" >/dev/null
  fi
else
  make -C "$BUILD_DIR" -j"$build_jobs" telem_sender >/dev/null
fi
exec "$BUILD_DIR/telem_sender" --telemetry-endpoint "$ENDPOINT" "$@"
