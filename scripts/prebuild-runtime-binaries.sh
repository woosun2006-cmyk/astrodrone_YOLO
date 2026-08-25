#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/project-env.sh"

GCS_BUILD_DIR="${GCS_BUILD_DIR:-$REPO_ROOT/gcs/build}"
BUILD_LOCK_PATH="${ASTRODRONE_BUILD_LOCK:-${TMPDIR:-/tmp}/astrodrone-runtime-build.lock}"

mkdir -p -- "$(dirname -- "$BUILD_LOCK_PATH")"
command -v flock >/dev/null 2>&1 || {
  printf '%s\n' '[PREBUILD 실패] flock이 필요합니다. 동일 build directory 동시 빌드를 허용하지 않습니다.' >&2
  exit 1
}
exec 9>"$BUILD_LOCK_PATH"
flock -x 9

printf '[PREBUILD] build lock 획득: %s\n' "$BUILD_LOCK_PATH"

cmake_supports_build() {
  cmake --help 2>/dev/null | grep -q -- '--build <dir>'
}

cmake_supports_source_binary() {
  local version major minor
  version="$(cmake --version | sed -n '1s/.*version //p')"
  major="${version%%.*}"
  minor="${version#*.}"
  minor="${minor%%.*}"
  (( major > 3 || (major == 3 && minor >= 13) ))
}

cmake_configure() {
  local source_dir="$1" build_dir="$2" label="$3"
  mkdir -p -- "$build_dir"
  if [[ ! -f "$build_dir/CMakeCache.txt" ||
        "$source_dir/CMakeLists.txt" -nt "$build_dir/CMakeCache.txt" ]]; then
    printf '[PREBUILD] configure %s\n' "$label"
    if ! cmake_supports_source_binary; then
      if ! (cd "$build_dir" && cmake "$source_dir" -DCMAKE_BUILD_TYPE=RelWithDebInfo >/dev/null 2>&1); then
        printf '[PREBUILD 실패] configure %s\n' "$label" >&2
        return 1
      fi
    elif ! cmake -S "$source_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=RelWithDebInfo >/dev/null 2>&1; then
      printf '[PREBUILD 실패] configure %s\n' "$label" >&2
      return 1
    fi
  fi
}

cmake_build_target() {
  local build_dir="$1" target="$2" label="$3"
  printf '[PREBUILD] build %s jobs=1\n' "$label"
  if cmake_supports_build; then
    if ! cmake --build "$build_dir" --target "$target" -- -j1 >/dev/null 2>&1; then
      printf '[PREBUILD 실패] build %s\n' "$label" >&2
      return 1
    fi
  elif ! make -C "$build_dir" -j1 "$target" >/dev/null 2>&1; then
    printf '[PREBUILD 실패] build %s\n' "$label" >&2
    return 1
  fi
}

require_binary() {
  local label="$1" binary="$2"
  [[ -x "$binary" ]] || {
    printf '[PREBUILD 실패] %s binary가 없습니다: %s\n' "$label" "$binary" >&2
    return 1
  }
  printf '[PREBUILD] ready %s: %s\n' "$label" "$binary"
}

cmake_configure "$YOLO_DIR/cpp" "$YOLO_LIVE_BUILD_DIR" YOLO
cmake_build_target "$YOLO_LIVE_BUILD_DIR" yolo_live YOLO
require_binary YOLO "$YOLO_LIVE_BUILD_DIR/yolo_live"

cmake_configure "$CONTROL_DIR" "$BUILD_DIR" control
cmake_build_target "$BUILD_DIR" telemetry_bench telemetry_bench
require_binary telemetry_bench "$BUILD_DIR/telemetry_bench"
cmake_build_target "$BUILD_DIR" health_check health_check
require_binary health_check "$BUILD_DIR/health_check"

printf '[PREBUILD] build target_distance jobs=1\n'
if ! CMAKE_BUILD_PARALLEL_LEVEL=1 "$SCRIPT_DIR/target-distance-cpp" --build-only >/dev/null 2>&1; then
  printf '%s\n' '[PREBUILD 실패] build target_distance' >&2
  exit 1
fi
require_binary target_distance "$BUILD_DIR/target_distance"

cmake_configure "$REPO_ROOT/gcs" "$GCS_BUILD_DIR" GCS
cmake_build_target "$GCS_BUILD_DIR" telem_sender GCS
require_binary GCS "$GCS_BUILD_DIR/telem_sender"

printf '%s\n' '[PREBUILD] all runtime binaries ready; runtime build disabled'
