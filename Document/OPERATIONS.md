# 실행과 운영

이 문서는 현재 launcher와 설정을 기준으로 한 실행 순서입니다. 실제 장치에서
검증하지 않은 내용은 `확인 필요`로 표시합니다.

## 1. 공통 원칙

- SITL은 `127.0.0.1` loopback endpoint만 사용합니다.
- 실제 Pixhawk, serial, `/dev/tty*`, 외부 LAN은 사용하지 않습니다.
- 한 번에 하나의 `run_simulation.sh`만 실행합니다.
- `observe`와 `shadow`에서는 vehicle 명령을 보내지 않습니다.
- `sitl-flight`의 `--arm`은 loopback SITL에서만 사용합니다.
- force arm이나 ARMING_CHECK 비활성화는 사용하지 않습니다.
- 실패하면 다음 단계로 넘어가지 않고 해당 로그를 먼저 확인합니다.

## 2. 주요 endpoint

| 용도 | endpoint |
| --- | --- |
| Gazebo FDM | UDP `127.0.0.1:9002` |
| SITL master | TCP `127.0.0.1:5760` |
| control 명령 | UDP `127.0.0.1:14550` |
| telemetry | UDP `127.0.0.1:14551` |
| Mission Planner/GCS | UDP `127.0.0.1:14552` |
| GCS telemetry fanout | UDP `127.0.0.1:14553` |
| target link | UDP `127.0.0.1:15020` |
| YOLO HTTP | TCP `127.0.0.1:8002` |
| audit relay | TCP `127.0.0.1:5770` |

Mission Planner endpoint는 `run_simulation.sh --mission-planner`를 사용할 때의
loopback GCS 경로입니다. headless 검증에서는 실행하지 않습니다.

## 3. SITL 실행

기본 world는 `simulation/worlds/ensamb_iris_runway.sdf`, vehicle은
`ensamb_with_gimbal`, 하향 카메라는 `ensamb_with_standoffs/down_camera`입니다.

### observe

Gazebo, SITL, MAVProxy/router와 telemetry 관찰만 수행합니다.

```bash
env -u DISPLAY -u WAYLAND_DISPLAY \
simulation/scripts/run_simulation.sh \
  --profile observe \
  --headless \
  --duration 30
```

이 단계에서 mission upload, ARM, YOLO control 명령은 실행하지 않습니다. heartbeat
간격과 cleanup을 먼저 확인합니다.

### shadow

실제 Gazebo camera를 C++ TensorRT YOLO에 연결하고 거리 계산과 guidance를 실행하되,
vehicle 명령은 sink/audit 경로에서 차단합니다.

```bash
env -u DISPLAY -u WAYLAND_DISPLAY \
YOLO_READY_TIMEOUT_SEC=90 \
simulation/scripts/run_simulation.sh \
  --profile shadow \
  --headless \
  --duration 20
```

YOLO readiness에 실패하면 shadow pipeline 미완료입니다. fixed target, synthetic
observation, confidence 완화로 성공 처리하지 않습니다.

### sitl-flight

observe와 shadow가 통과한 뒤에만 loopback SITL 명령 검증에 사용합니다.

```bash
env -u DISPLAY -u WAYLAND_DISPLAY \
YOLO_READY_TIMEOUT_SEC=90 \
simulation/scripts/run_simulation.sh \
  --profile sitl-flight \
  --simulation-profile diagonal-approach-validation \
  --headless \
  --status-monitor \
  --upload-mission \
  --arm
```

`--upload-mission`과 `--arm`은 이 프로파일에서만 명시적으로 사용합니다. 명령
packet, actual mode, target detection, TargetCenteredHold, cleanup을 별도로
기록합니다. `diagonal-approach-validation`은 기본 guidance를 바꾸지 않는
검증용 프로파일입니다.

GUI가 필요하면 `--headless`를 빼되, WSL의 DISPLAY/OpenGL 조건이 맞아야 합니다.
GUI 오류와 SITL/FDM readiness 오류는 로그에서 구분해야 합니다.

## 4. 실기체 프로파일

실기체 실행은 실제 장치에서 검증하지 않았습니다. 기본 원칙은 external MAVLink
router가 Pixhawk serial을 단독 소유하고, 애플리케이션은 router의 UDP만 읽거나
사용하는 것입니다.

### observe-real

```bash
scripts/observe-real.sh --duration 10
```

router telemetry를 읽기 전용으로 표시합니다. HEARTBEAT, GPS, EKF, SYS_STATUS,
배터리 freshness를 확인하지만 ARM, mode, takeoff, LAND, velocity,
`SET_MESSAGE_INTERVAL`을 보내지 않습니다. 실제 endpoint와 장치 연결은 확인
필요합니다.

### shadow-real

```bash
scripts/shadow-real.sh --duration 60
```

Jetson camera → TensorRT YOLO → target-distance → guidance 계산까지 수행하는
구성입니다. vehicle 명령은 ShadowCommandSink에서 차단되어야 합니다. 실제 Jetson
camera와 TensorRT runtime이 없으면 성공으로 표시하지 않습니다.

### flight-real

이번 작업에서 실행하지 않습니다. 기본 동작은 거부이며, 실제 운용에는 다음과
같은 이중 확인과 external router 정책 확인이 필요합니다.

```text
--target real
--connect /dev/serial/by-id/...
--command-endpoint udp:127.0.0.1:14550
--telemetry-endpoint udp:127.0.0.1:14551
--allow-arm
--confirm-real-flight
--commands-enabled
--router-owned
```

이 조건이 있다고 해서 실제 비행 검증이 끝난 것은 아닙니다. preflight, RC 정책,
serial owner, camera calibration을 별도로 확인해야 합니다.

## 5. 안전한 실행 순서

1. 저장소와 외부 의존성 경로를 확인합니다.
2. `observe`로 world/model, camera topic, SITL heartbeat를 확인합니다.
3. observe가 통과하면 `shadow`로 YOLO부터 ShadowCommandSink까지 확인합니다.
4. shadow가 통과한 경우에만 `sitl-flight`를 실행합니다.
5. 각 실행 후 로그, packet audit, process와 port cleanup을 확인합니다.

status monitor는 읽기 전용으로 상태를 표시합니다. `SAFE_HOVER`는 자동으로
GUIDED에 복귀하지 않는 latch 상태입니다.

## 6. 로그 확인

simulation 로그는 보통 다음 디렉터리에 저장됩니다.

```text
simulation/logs/run_<profile>_<timestamp>/
```

먼저 `launcher.log`를 보고, 이후 `gazebo.log`, `sitl.log`, `router.log`,
`autonomy.log`, `yolo_cpp*.stdout`, `yolo_cpp*.stderr`, `upload_mission.log`,
`packet_audit.json`, `cleanup.log`를 확인합니다. 실제 파일명은 실행 시점에 따라
달라질 수 있습니다.

확인 순서는 다음과 같습니다.

1. Gazebo world/model과 FDM `9002` readiness
2. SITL master `5760`와 ArduPilot HEARTBEAT
3. router의 `14550/14551/14552` 출력
4. YOLO TensorRT binary, model, HTTP `8002` readiness
5. camera topic과 frame freshness
6. target UDP `15020`과 control 로그
7. packet audit의 방향, 명령 수, cleanup

TensorRT 개발 라이브러리 누락, GUI/OpenGL 오류, heartbeat 간격 초과, endpoint
충돌은 각각 별도 원인으로 기록합니다. 로그만으로 확정할 수 없는 내용은
`확인 필요`로 남깁니다.

## 7. 실제 장치 연결 전 체크리스트

- Pixhawk serial을 external router만 소유하는가
- 애플리케이션이 `/dev/tty*`를 직접 열지 않는가
- observe/shadow의 `commands_enabled`가 false인가
- command와 telemetry endpoint가 의도한 router UDP인가
- HEARTBEAT, GPS, EKF, 배터리, RC freshness가 충분한가
- camera source, 해상도, timestamp, stale timeout이 확인됐는가
- calibration과 body-axis 부호를 실제 장착 기준으로 검증했는가
- `CommandGate`, `ControlAuthority`, `PreflightGate`가 활성화됐는가
- packet audit와 상태 로그를 수집할 수 있는가
- 노트북 GCS가 vehicle command 경로를 갖고 있지 않은가

위 체크리스트를 통과해도 실제 Pixhawk, Jetson, serial, 외부 LAN과 실기체 비행은
별도 승인과 검증이 필요합니다.
