#!/usr/bin/env bash
# Runs ON the Jetson (astro-drone/gazeboSim/bridge_sitl.sh).
#
# control/mavlink_proxy.cpp only ever opens setting/MAVLink.yaml's
# real.serial.address and has no SITL/network path, so to point it at a
# PC-hosted Gazebo + ArduPilot SITL we give it a PTY that looks like a serial
# port and relay that PTY to the SITL's MAVLink over TCP.
#
# Full chain:
#   mavlink_proxy -> /tmp/sitl_serial (PTY, this script)
#                 -> TCP 192.168.0.34:25760           (droneVideo, LAN)
#                 -> socat relay to droneVideo 127.0.0.1:25760
#                 -> ssh -R reverse tunnel
#                 -> PC WSL 127.0.0.1:5770            (MAVProxy tcpin leg)
#                 -> MAVProxy -> SITL 127.0.0.1:5760  (ArduCopter SERIAL0)
#
# Notes on the port choices, all verified rather than assumed:
#   - 5770 not 5760: SITL's SERIAL0 TCP serves one client and MAVProxy holds
#     it; a second connection attaches but receives nothing.
#   - 25760 not 15760: the Jetson's sshd already listens on 15760.
#   - the last leg is a socat TCP relay, not an ssh -R straight to the Jetson,
#     because the Jetson's sshd refuses remote port forwards.
set -u

PTY_LINK=/tmp/sitl_serial
RELAY_HOST=192.168.0.34
RELAY_PORT=25760

pkill -f "socat .*${PTY_LINK}" 2>/dev/null
sleep 1
rm -f "$PTY_LINK"

setsid nohup socat -d -d \
  "PTY,raw,echo=0,link=${PTY_LINK}" \
  "TCP:${RELAY_HOST}:${RELAY_PORT}" \
  >/tmp/bridge_sitl.log 2>&1 </dev/null &
disown
sleep 3

echo "--- socat process ---"
pgrep -af "socat .*${PTY_LINK}" || echo BRIDGE_NOT_RUNNING
echo "--- pty ---"
ls -l "$PTY_LINK" 2>&1 || echo PTY_MISSING
echo "--- log ---"
tail -5 /tmp/bridge_sitl.log 2>/dev/null || true
