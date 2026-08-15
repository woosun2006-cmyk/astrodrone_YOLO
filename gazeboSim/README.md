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

## Resolved (2026-08-12): `control --auto-intercept` never reached the lock stage

> **This is fixed.** Repaired in `control/control.cpp` by commit `7d85d66`
> ("control: fix the AUTO-intercept lock race and the command-target
> overwrite", 2026-08-12). The diagnosis below is kept as the record of what
> was originally observed; see **How it was fixed** at the end of the section
> for what actually changed. Do not cite this section as an open defect.

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

**How it was fixed.** Commit `7d85d66` (2026-08-12) did *not* hoist the
`HealthState` out of the loop - `run_auto_intercept()` still declares a fresh
`HealthState health;` inside its outer `while (true)` (`control.cpp:932-933`).
What changed is that `wait_for_auto_armed()` already decodes the HEARTBEAT
proving AUTO + armed, and now **seeds the caller's `HealthState` from that
same heartbeat** instead of discarding it. `wait_for_lock()` also takes
`HealthState&` by reference (`control.cpp:829`) rather than constructing its
own. By the time its `!have_armed` check runs the flag is already true, so the
race is closed.

That commit's own evidence: `wait_for_lock()`'s 5 s status line
`[AUTO_INTERCEPT] AUTO 비행 중 - 타겟 대기 (tracking=... healthy=...)` now
prints, having never appeared once before the change.

---

## Changelog

### 2026-08-12

**Added**

- `start_pc_sim.sh` - one-command PC-side bring-up (Gazebo -> SITL -> MAVProxy,
  in that order, with the port/reset pitfalls handled).
- `spawn_target.sh` + `models/target_basket/` - spawns a target into the
  running world so the camera has something to detect. The model is a copy of
  `simulation/models/target_basket` with the material changed from white to
  red; geometry and collision box are unchanged. `simulation/` itself is only
  read, never modified.

**Gazebo configuration: this harness vs `simulation/`**

These differ in one way that decides whether perception works at all, so
`start_pc_sim.sh` now prefers `simulation/`'s world whenever it is present on
the checked-out branch and only falls back to the stock one otherwise:

| | `simulation/` (ensamb_iris_runway) | ardupilot_gazebo stock (iris_runway) |
|---|---|---|
| camera | `ensamb_with_standoffs::down_camera` | `iris_with_gimbal` gimbal camera |
| mounting | fixed joint, `<pose>0 0 0 0 1.5708 0</pose>` | 3-axis gimbal, default 0 rad |
| where it looks | straight down, always | **forward** until commanded |
| FOV | 2.7507 rad | 2.7507 rad (locally modified) |
| video out | GstCameraPlugin, udp 5600 | GstCameraPlugin, udp 5600 |
| ground | `grass_ground` | runway only |

The stock gimbal camera cannot see a target on the ground in a plain SITL run:
`ArduPilotPlugin` only publishes `/gimbal/cmd_roll|pitch|yaw` once it sees
non-zero PWM on channels 8/9/10, so with nothing driving those channels the
topics stay empty and the camera holds its default forward attitude.
`simulation/reports/2026-08-11-gimbal-direction-investigation.md` measured the
same thing: optical axis body +X by default, body -Z only after a commanded
+90 deg pitch. Since both cameras carry GstCameraPlugin on the same udp port,
`gazebo_camera_target.py` works unchanged against either.

To use the stock world anyway, point the gimbal down first:

```bash
gz topic -t /gimbal/cmd_pitch -m gz.msgs.Double -p "data: 1.5708"
```

**Verified on 2026-08-12**

- Gazebo Jetty 10.5.0 + ArduCopter SITL bring-up via `start_pc_sim.sh`, with
  both the stock world and `simulation/`'s ensamb world.
- `target_basket` spawns into the live world (`gz model --list` shows it).
- Down-camera RTP stream reaches `gazebo_camera_target.py` at ~8 fps on
  `.../down_camera_link/sensor/down_camera/image`.
- Arm, takeoff and GUIDED reposition to 4 m under `safety.yaml`'s 5 m
  `hard_limit_m`.
- Earlier the same day: full AUTO waypoint mission + RTL + auto-land + disarm;
  `scripts/health_check.sh` passing against the bridged SITL (GPS fix_type 6,
  10 sats); `mavlink_proxy` relaying the SITL over the PTY bridge.

**Not yet working**

- *Stable target lock.* The detector registers the basket transiently
  (`x_px` ~199, i.e. near the frame edge) but does not hold it while hovering
  over the spawn point, so `confirmed` never latches. The spawn offset, the
  camera FOV footprint at 4 m, and the HSV threshold all still need tuning
  against each other. The transport itself is proven - frames arrive and the
  detector runs.
- *`control --auto-intercept`.* Blocked upstream of perception by the
  `wait_for_lock()` / `HealthState` race described above; it never reaches the
  lock stage regardless of what `/target` returns.

**Note on branches**

`simulation/` lives on `Document`; `gazeboSim/` and `scripts/` live on
`control_program_test1`. `start_pc_sim.sh` degrades gracefully when
`simulation/` is absent, but the fixed down camera - the configuration that
actually makes perception viable - only exists on `Document`. Merging the two
branches would remove the need for that fallback.

### 2026-08-12 (later) - stable target lock achieved

`models/target_basket` is now scaled 8x (mesh scale 0.001 -> 0.008, collision
box to match, ~1.90 x 1.10 x 0.56 m).

**Why the original size never locked.** `ensamb_with_standoffs`' `down_camera`
uses `horizontal_fov` 2.7507 rad (~157 deg). At 4 m altitude that covers
roughly 39 m of ground across 640 px - about 16 px per metre. The stock
0.238 m basket therefore rendered ~4 px wide, an ~8 px^2 blob: below
`gazebo_camera_target.py`'s 80 px^2 contour floor, and far too few pixels to
hold a centroid steady even with the floor lowered. At 8x it subtends ~30 px
(~450 px^2) and segments cleanly.

Frame analysis confirmed the earlier failure was visibility, not colour: with
the small target the captured frame contained **zero** red pixels under any
threshold and 97% of saturated pixels sat at hue 57 (grass).

**Measured result** - vehicle hovering at 4.0 m over the target, 45 s hold:

| metric | value |
|---|---|
| `found` | 4196 / 4196 samples (100%) |
| `confirmed` | 4196 / 4196 samples (100%) |
| longest continuous `confirmed` streak | 45.0 s |
| pixel offset | `x_px` ~ -28, `y_px` ~ +-3 (stable) |

`control.cpp` needs `lock_confirm_sec` = 1.0 s of continuous tracking, so this
clears the requirement by a wide margin.

**Reproducing it**

```bash
gazeboSim/start_pc_sim.sh                    # spawns the target at 8 m north
gazeboSim/spawn_target.sh ensamb_iris_runway 0 0   # or put it at the origin
gz topic -t /world/ensamb_iris_runway/model/ensamb_with_gimbal/model/ensamb_with_standoffs/link/down_camera_link/sensor/down_camera/image/enable_streaming \
  -m gz.msgs.Boolean -p "data: true"
/usr/bin/python3 gazeboSim/gazebo_camera_target.py --port 8002
# then arm, take off to ~4 m over the target
curl -s http://127.0.0.1:8002/target
```

**Re-tune if you change anything**

The 8x scale is tied to the camera FOV and the test altitude, not to the real
basket. Anything that makes the target subtend fewer than ~15 px will stop
locking reliably - raising the altitude, narrowing the FOV, or restoring the
original scale all do that. `--hsv-lo` / `--hsv-hi` on the detector adjust the
colour band if the world lighting changes.

**Still blocked**

`control --auto-intercept` remains unable to use any of this: the
`wait_for_lock()` / `HealthState` race described above bails before perception
is ever consulted. Perception is now proven to deliver a stable
`confirmed: true`, so that defect is the only thing between this harness and a
full intercept run.
