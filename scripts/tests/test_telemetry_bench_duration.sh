#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../real-runtime-common.sh"

[[ "$(resolve_observe_duration 10 0 0)" == 10 ]]
[[ "$(resolve_observe_duration 10 0 1)" == 0 ]]
[[ "$(resolve_observe_duration 10 1 1)" == 10 ]]
[[ "$(resolve_observe_duration 7 1 0)" == 7 ]]

printf '%s\n' 'telemetry-bench duration regression: PASS'
