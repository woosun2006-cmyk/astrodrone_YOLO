#!/bin/bash
set -eu

test_dir="$(cd "$(dirname "$0")" && pwd)"
stamp="$(date +%Y%m%d_%H%M%S)"
mkdir -p "$test_dir/logs"
log_path="$test_dir/logs/power_test_${stamp}.csv"

echo "CPU target: 85%, GPU target: 85%, duration: 10 minutes"
echo "Log: $log_path"
exec sudo python3 "$test_dir/monitor.py" \
  --seconds 600 \
  --target 85 \
  --gpu-target 85 \
  --log "$log_path"
