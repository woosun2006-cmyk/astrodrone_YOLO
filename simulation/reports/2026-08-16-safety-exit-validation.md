# SITL 종료·안전 시나리오 검증 기록

검증일: 2026-08-16

## 범위

- 대상은 `astrodrone_YOLO` 저장소의 loopback SITL이다.
- 모든 MAVLink endpoint는 `127.0.0.1`을 사용했다.
- Mission Planner, 실제 Pixhawk, serial, `/dev/tty*`, LAN endpoint는 사용하지 않았다.
- force arm과 `ARMING_CHECK` 비활성화는 사용하지 않았다.
- production guidance, gain, geometry, MAVLink packet serialization은 변경하지 않았다.

## 외부 모드 변경 시나리오

실행 명령:

```bash
env -u DISPLAY -u WAYLAND_DISPLAY \
  TARGET_DISTANCE_ARGS='--fixed-target-ned 4 0 --fixed-target-min-alt 4.5' \
  RUN_ID=external_audit_final_20260816_145802 \
  SIM_RUNTIME_DIR=/tmp/astrodrone-simulation-external_audit_final_20260816_145802 \
  YOLO_READY_TIMEOUT_SEC=90 \
  simulation/scripts/run_simulation.sh \
    --profile sitl-flight --headless --duration 120 --upload-mission --arm
```

GUIDED 인계가 확인된 뒤 loopback MAVLink 주입기로 `STABILIZE` 외부 모드 변경을 한
번 발생시켰다. Mission Planner는 실행하지 않았다.

로그:

`simulation/logs/run_sitl-flight_external_audit_final_20260816_145802/`

결과:

- 종료 코드: `0`
- `packet_audit.json` event 수: `971`
- ControlLocked 시각: `2026-08-16T05:59:47.123Z`
- 시간순 `vehicle_command` event 중 잠금 전: `42`
- 잠금 후 실제 vehicle command: `0`
- 잠금 후 차단 decision: `573`
- 기존 packet summary의 `vehicle_affecting_command_count`: `38`
- 잠금 이유: `UnexpectedOperatorOrGcsModeChange`
- 잠금 후 `SET_MODE`, `VELOCITY_SETPOINT`는 decision에 차단으로 기록됐고
  SITL로 전달되지 않았다.
- HEARTBEAT와 telemetry 수신은 잠금 후에도 계속됐다.

`lock_before_vehicle_command_count`와 `lock_after_vehicle_command_count`는 시간순
`vehicle_command` event 기준이다. 기존 `vehicle_affecting_command_count`는
packet proxy의 forbidden-command summary 기준이므로 두 값은 같은 집계 범위가
아니다. 따라서 audit 파일만으로 잠금 전 송신, 잠금 후 차단, 잠금 후 실제 송신을
구분할 수 있다.

## Target loss 및 LAND 시나리오

기본 target-loss 정책은 기존대로 `AUTO` 복귀다. 이번 검증에서만
`SITL_TARGET_LOSS_POLICY=LAND`를 별도 환경변수로 설정했다. 이 설정은 production
기본값이나 guidance 알고리즘을 변경하지 않는다.

실행 명령:

```bash
env -u DISPLAY -u WAYLAND_DISPLAY \
  TARGET_DISTANCE_ARGS='--fixed-target-ned 4 0 --fixed-target-min-alt 4.5' \
  SITL_TARGET_LOSS_POLICY=LAND \
  SITL_LAND_OBSERVATION_SEC=35 \
  RUN_ID=target_loss_land_audit_20260816_145400 \
  SIM_RUNTIME_DIR=/tmp/astrodrone-simulation-target_loss_land_audit_20260816_145400 \
  YOLO_READY_TIMEOUT_SEC=90 \
  simulation/scripts/run_simulation.sh \
    --profile sitl-flight --headless --duration 140 --upload-mission --arm
```

검증 중 target-distance를 일시 중지해 target loss를 재현했다.

로그:

`simulation/logs/run_sitl-flight_target_loss_land_audit_20260816_145400/`

결과:

- 종료 코드: `0`
- `packet_audit.json` event 수: `706`
- `MAV_CMD_NAV_LAND` request: command count 기준 `1`
- 실제 HEARTBEAT: `LAND` mode 진입 확인
- 고도: 약 `4.6m`에서 `0.0m`까지 감소하는 position event 확인
- DISARM: LAND 중 실제 HEARTBEAT에서 `armed=false` 확인
- mission 완료: 별도의 ArduPilot mission-complete packet이 아니라
  controller와 launcher가 정상 종료되고 cleanup이 완료된 것으로 기록
- 실제 vehicle-affecting command count: `166`

LAND request 송신, 실제 LAND mode 진입, 고도 감소, DISARM, 시나리오 완료는 서로
다른 이벤트로 기록된다. LAND mode 진입만으로 착륙 완료로 판정하지 않는다.

## 로그·감사 형식

`packet_audit.json`은 packet event, HEARTBEAT mode, position event, CommandGate
decision을 시간순으로 합친다. 각 command event에는 다음 정보가 포함된다.

- timestamp
- command type
- allowed / blocked / sent
- block reason
- control lock timestamp
- lock 전 vehicle command count
- lock 후 vehicle command count
- lock 후 blocked command count

ControlGate decision 원본은 같은 실행 디렉터리의 `control_events.jsonl`에 남고,
MAVLink packet audit proxy가 이를 packet/state event와 병합한다.

## 제외한 항목과 남은 작업

- RC takeover는 이번 검증에 포함하지 않았다.
- ArduPilot failsafe 증거를 이용한 별도 시나리오는 아직 실행하지 않았다.
- 따라서 RC takeover와 ArduPilot failsafe는 미구현·미검증 항목으로 남긴다.
- CTest, control build, shell syntax, audit proxy Python 문법 검사는 통과했다.
- 두 시나리오 모두 종료 후 SITL, MAVProxy, Gazebo, audit process와 관련 loopback
  port 잔여가 없었다.
