# gazeboSim/

Gazebo-simulation-only scaffolding. Nothing here is used by the real-flight
path (`scripts/full_mission.sh`, `YOLO_MODEL/`, the Pixhawk). It exists so the
real `control` / `health-check` / `gcs` binaries can be exercised against a
PC-hosted Gazebo + ArduPilot SITL instead of the vehicle.

`setting/`, `control/`, `health-check/` and `gcs/` are **not** modified by any
of this, with one exception that is always reverted: the launcher scripts
temporarily rewrite `setting/MAVLink.yaml`'s `real.serial.address` and restore
it from `MAVLink.yaml.sim-backup` on exit (including on Ctrl+C).

---

## Why the simulation needs scaffolding at all

Two files in the real stack cannot reach a simulator without being edited, so
they are worked around rather than changed:

**1. `control/mavlink_proxy.cpp` only opens a serial port.**
It reads `setting/MAVLink.yaml`'s `real.serial.address` and nothing else -
there is no SITL/network path and no CLI flag. (`sitl:` exists in that YAML but
only `health-check/test_arm.cpp` reads it.) So `bridge_sitl.sh` creates a PTY
that *looks* like a serial port and relays it to the SITL over TCP, and the
launcher points that one address at the PTY.

**2. `YOLO_MODEL/cpp/yolo_headless.cpp` only opens a Jetson CSI camera.**
Its GStreamer pipeline is hardcoded to `nvarguscamerasrc`, the Jetson L4T Argus
stack. There is no Gazebo equivalent, and no source-selection flag. So it is
not run in simulation; something else serves the same `/target` HTTP contract
`control/target_distance.cpp` polls.

> For reference, astroquad/uav-onboard solves (2) the other way - a
> `--vision gazebo` / `--vision rpicam` flag inside the vision node. That is the
> only way to validate the real model on simulated video, and would require
> editing `yolo_headless.cpp`.

---

## Files

| file | runs on | purpose |
|---|---|---|
| `bridge_sitl.sh` | Jetson | PTY ↔ TCP relay so `mavlink_proxy` reaches the PC's SITL |
| `fake_yolo_target.py` | Jetson | static synthetic `/target` - **no perception** |
| `gazebo_camera_target.py` | **PC** | real detections from the Gazebo camera, same `/target` contract |
| `MAVLink.yaml.sim-backup` | Jetson | pristine copy of `setting/MAVLink.yaml` |

Restore the settings file manually at any time with:

```bash
cp gazeboSim/MAVLink.yaml.sim-backup setting/MAVLink.yaml
```

---

## Network path

The Jetson cannot reach the PC directly; droneVideo (`192.168.0.34` on the LAN,
`10.0.0.1` on WireGuard) sits between them.

```
Jetson mavlink_proxy
  -> /tmp/sitl_serial                  PTY, created by bridge_sitl.sh
  -> tcp 192.168.0.34:25760            droneVideo, LAN
  -> socat relay -> droneVideo 127.0.0.1:25760
  -> ssh -R reverse tunnel
  -> PC (WSL) 127.0.0.1:5762           ArduPilot SITL SERIAL1
```

Port choices, all verified rather than assumed:

- **5762, not 5760.** SITL's SERIAL0 TCP serves exactly one client and MAVProxy
  holds it; a second connection attaches but receives nothing (sockets sit in
  `CLOSE-WAIT`). SERIAL1 is also what `setting/MAVLink.yaml`'s `sitl.local_tcp`
  already declares. SITL only opens 5762/5763 *after* a SERIAL0 client
  connects, so MAVProxy must be attached to 5760 first.
- **25760, not 15760.** The Jetson's own sshd already listens on 15760.
- **A socat TCP relay for the last leg, not `ssh -R` straight to the Jetson.**
  The Jetson's sshd refuses remote port forwards (`remote port forwarding
  failed`) even for a free port with `allowtcpforwarding yes`.

---

## Running it

### PC side (WSL, where Gazebo lives)

```bash
# 1. Gazebo, then SITL, then MAVProxy - in that order.
#    A Gazebo world reset invalidates ArduPilotPlugin's JSON FDM session on
#    udp 9002; SITL then loops "No JSON sensor message received" forever, so
#    Gazebo must come up first and SITL must be restarted after any reset.
cd ~/astrodrone_YOLO
bash simulation/scripts/start_gazebo.sh &

python3 ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris \
  --model JSON -I 0 -S 1 -N --no-mavproxy \
  --use-dir ~/astrodrone-sim/runtime/sitl-0 &

~/venv-ardupilot/bin/python ~/venv-ardupilot/bin/mavproxy.py \
  --master=tcp:127.0.0.1:5760 --out=udpout:127.0.0.1:14551 \
  --streamrate=4 --heartbeat-rate=1 \
  --default-modules=link --non-interactive --no-state &

# 2. reverse tunnel to droneVideo, and a LAN relay there
sshpass -p astro ssh -N -R 25760:127.0.0.1:5762 astro@10.0.0.1 &
sshpass -p astro ssh astro@10.0.0.1 \
  'setsid nohup socat TCP-LISTEN:25760,bind=192.168.0.34,fork,reuseaddr \
     TCP:127.0.0.1:25760 >/tmp/socat_expose.log 2>&1 </dev/null & disown'
```

Arm the vehicle and start an AUTO mission before running the Jetson side -
`control --auto-intercept` never arms or takes off itself, it only watches an
already-flying AUTO mission and takes over.

Keep the mission altitude under `setting/safety.yaml`'s
`altitude_limit.hard_limit_m` (**5.0 m**), or `control.cpp` will spend the
whole run commanding a corrective descent instead of intercepting.

### Camera (only for `sim_mission_camera.sh`)

```bash
# enable ardupilot_gazebo's GstCameraPlugin RTP stream (-> udp 127.0.0.1:5600)
gz topic -t /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image/enable_streaming \
  -m gz.msgs.Boolean -p "data: true"

# needs GStreamer-enabled OpenCV: apt python3-opencv, NOT the pip wheel
# (pip's opencv-python is built without GStreamer and cannot open the pipeline)
/usr/bin/python3 gazebo_camera_target.py --port 8002

# expose it as the Jetson's own 127.0.0.1:8002, so target_track.yolo_host
# stays 127.0.0.1 and no settings change is needed
sshpass -p astro ssh -N -R 28002:127.0.0.1:8002 astro@10.0.0.1 &
sshpass -p astro ssh astro@10.0.0.1 \
  'setsid nohup socat TCP-LISTEN:28002,bind=192.168.0.34,fork,reuseaddr \
     TCP:127.0.0.1:28002 >/tmp/socat_yolo.log 2>&1 </dev/null & disown'
# on the Jetson:
socat TCP-LISTEN:8002,bind=127.0.0.1,fork,reuseaddr TCP:192.168.0.34:28002 &
```

### Jetson side

```bash
cd ~/astro-drone
./scripts/sim_mission_no_camera.sh   # synthetic target, no perception
# or
./scripts/sim_mission_camera.sh      # real Gazebo camera detections
```

Both restore `setting/MAVLink.yaml` on exit, including on Ctrl+C.

`scripts/health_check.sh` is not called by either: its `check_link` opens the
serial device directly and would fight `mavlink_proxy` for the PTY - the same
reason `full_mission.sh` runs it *before* starting the proxy. To run the gate,
start `bridge_sitl.sh`, point the YAML at the PTY, run `health_check.sh`, then
launch a sim script.

---

## Known blocker: `control --auto-intercept` never reaches the lock stage

Verified against this setup: the vehicle armed and flying AUTO at 3.5 m,
`/target` returning `confirmed: true`, `mavlink_proxy` relaying, `control`
connecting and correctly reporting `system = 1`. The log then repeats forever:

```
[AUTO_INTERCEPT] AUTO + armed 확인됨 - 타겟 확정 대기.
[AUTO_INTERCEPT] AUTO/armed 상태 이탈 - 대기 상태로 복귀.
```

`run_auto_intercept()` creates a fresh `HealthState health;` on every pass of
its outer loop, so `health.have_armed` starts `false`. It hands that straight
to `wait_for_lock()`, whose first checks are:

```cpp
if (!health.have_armed || !health.armed ||
    health.custom_mode != copter_mode_mapping().at("AUTO")) {
    return false;          // silent - the caller prints the "이탈" line
}
```

Between those two points there is a single `recv_match(..., 0.1s)`. Only a
HEARTBEAT sets `have_armed`, and HEARTBEAT is 1 Hz, so the check almost always
runs before one has arrived and bails out immediately.

The decisive evidence: `wait_for_lock()`'s own status line

```
[AUTO_INTERCEPT] AUTO 비행 중 - 타겟 대기 (tracking=... healthy=...)
```

prints every 5 s from inside that loop and **never appeared once** - the
function exits within milliseconds every time.

This is in `control/control.cpp` and was left unfixed deliberately (the
simulation work was scoped to not modify `control/`). It affects real flight
the same way: nothing about it is simulation-specific.
