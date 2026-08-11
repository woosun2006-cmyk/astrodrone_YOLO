#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=_common.sh
source "$SCRIPT_DIR/_common.sh"

require_dir "$ASTRODRONE_REPO/control"
require_dir "$ASTRODRONE_REPO/setting"
require_command cmake
require_command ninja

repo_real="$(realpath -e -- "$ASTRODRONE_REPO")"
mkdir -p -- "$SIM_BUILD_DIR"
build_root="$(realpath -e -- "$SIM_BUILD_DIR")"
[[ "$build_root" != "$repo_real" && "$build_root" != "$repo_real/"* ]] ||
  sim_die "SIM_BUILD_DIR must be outside the repository: $build_root"
[[ "$build_root" != / && "$build_root" != /home && "$build_root" != /tmp ]] ||
  sim_die "SIM_BUILD_DIR is too broad: $build_root"

control_build="$build_root/build"
mkdir -p -- "$control_build"
settings_link="$build_root/setting"
if [[ -e "$settings_link" || -L "$settings_link" ]]; then
  [[ -L "$settings_link" && "$(realpath -e -- "$settings_link")" == "$(realpath -e -- "$ASTRODRONE_REPO/setting")" ]] ||
    sim_die "refusing to replace unexpected path: $settings_link"
else
  ln -s -- "$ASTRODRONE_REPO/setting" "$settings_link"
fi

sim_log "configure source=$ASTRODRONE_REPO/control build=$control_build (outside repository)"
cmake --fresh -S "$ASTRODRONE_REPO/control" -B "$control_build" -G Ninja \
  -DBUILD_TESTS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$control_build"

simulation_test_build="$build_root/simulation-tests"
cmake --fresh -S "$ASTRODRONE_REPO/simulation/tests" -B "$simulation_test_build" -G Ninja \
  -DASTRODRONE_REPO="$ASTRODRONE_REPO" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$simulation_test_build"

sim_log 'built executables:'
find "$control_build" -maxdepth 1 -type f -executable -printf '  %p\n' | sort
printf '  %s\n' "$simulation_test_build/read_only_telemetry"
sim_log "CTest command: ctest --test-dir $control_build --output-on-failure"
