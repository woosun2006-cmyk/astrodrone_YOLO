# Project custom downward-camera smoke

## Selection

- world: repository `simulation/worlds/ensamb_iris_runway.sdf`
- spawned vehicle: repository `simulation/models/ensamb_with_gimbal`
- included body/camera model: repository `simulation/models/ensamb_with_standoffs`
- target: repository `simulation/models/target_basket`
- fixed camera: `down_camera_link::down_camera`
- SITL / MAVProxy / control / YOLO: not started

This scenario is intentionally separate from the upstream connection smoke in
`smoke_test.md`, which keeps `iris_runway.sdf` and `iris_with_gimbal`.

## Run

```bash
simulation/scripts/camera_smoke_test.sh
```

The script resolves the camera metadata from the selected SDF chain, starts
headless Gazebo, inspects every advertised topic, and requires exactly one
`gz.msgs.Image` publisher owned by the selected model/link/sensor. It receives
a frame, saves a PPM, and verifies the runtime optical axis is body -Z and
world-down. Multiple Image publishers are a hard failure and are reported
without selecting the first candidate.

It also requires the resolved world and vehicle SDF to be non-symlink files
inside `simulation/`, places repository resources before external resources,
and confirms that both `ensamb_with_gimbal` and `target_basket` were created.
