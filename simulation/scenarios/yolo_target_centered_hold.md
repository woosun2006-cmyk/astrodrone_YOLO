# C++ YOLO 표적 접근 시나리오

이 시나리오는 고정 표적 입력을 사용하지 않는다.

```text
Gazebo down_camera
  -> C++ TensorRT YOLO
  -> http://127.0.0.1:8002/target
  -> target-distance
  -> UDP 127.0.0.1:15020
  -> control
  -> MAVLink velocity
  -> TargetCenteredHold
```

기존 world와 production 설정은 수정하지 않는다. 실행 시 `/tmp/astrodrone-simulation-runtime`에 world와 설정을 복사하고, SITL waypoint 바로 아래에 실제 `target_basket` 위치를 `(0, 3.00, 0)`으로 둔다. 이 위치는 0.8m 비행 고도에서 실제 C++ TensorRT YOLO가 안정적으로 중심을 확인하도록 하는 시나리오 전용 pose이며, production 중심 허용값 `25px`은 바꾸지 않는다. 미션 waypoint는 target 영역으로 이동시키고, control은 실제 pixel observation으로 수평 정렬한 뒤 2m 이내 hold 조건을 평가한다. 실제 `basket.stl`은 그대로 사용하되, 0.8m 고도·157도 FOV에서 검출 가능한 크기가 되도록 `/tmp` 전용 visual scale만 4배로 한다. 충돌 geometry와 production 모델은 변경하지 않는다. 임시 설정은 handoff 최소 고도를 0.5m로 낮추고, 설정된 표적 lock 조건을 검증하기 위해 scenario 전용 값만 사용한다. 미션 업로드와 ARM 뒤 relative altitude가 0.6m에 도달할 때까지 read-only telemetry gate를 기다린 후 YOLO/control gate를 연다. `target-distance`에는 `--fixed-target-ned`를 전달하지 않는다.

실행:

```bash
cd /mnt/c/Users/leems/OneDrive/Desktop/astrodrone_YOLO
env -u DISPLAY -u WAYLAND_DISPLAY \
  YOLO_READY_TIMEOUT_SEC=90 \
  simulation/scripts/run_simulation.sh \
    --profile sitl-flight \
    --scenario yolo-target-centered-hold \
    --headless \
    --duration 145 \
    --upload-mission \
    --arm
```

성공 조건은 `yolo_cpp_*.stdout.log`의 C++ TensorRT engine 로드, `target-distance` 로그의 `source=gazebo:` provenance, `/target`의 frame metadata, 실제 velocity packet, `dist_m <= 2`, `TARGET_CENTERED_HOLD`, GUIDED zero-velocity 유지다.
