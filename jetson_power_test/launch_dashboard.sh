#!/bin/sh
set -eu
APP_DIR=/home/astro/jetson_power_test
DISPLAY_VALUE=${DISPLAY:-:1}
AUTHORITY_FILE=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}
exec env DISPLAY="$DISPLAY_VALUE" XAUTHORITY="$AUTHORITY_FILE" python3 "$APP_DIR/dashboard.py"
