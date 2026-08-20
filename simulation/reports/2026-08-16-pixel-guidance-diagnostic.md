# 픽셀 기반 Guidance 진단 기록

## 결론

기존 픽셀 기반 guidance에는 body-forward 부호 오류가 있었다. 하향 카메라의
실제 pose와 고정 표적의 위치를 비교했을 때, 표적이 화면 위쪽에 있으면 기체
앞쪽으로 가야 하지만 기존 코드는 음의 `vx`를 만들었다. 이 부분만 수정했고,
gain, LOITER 안전 정책, production 최대 속도 설정은 변경하지 않았다.

## 실제 카메라 축 확인

- World: `simulation/worlds/ensamb_iris_runway.sdf`
- Vehicle: `ensamb_with_gimbal`
- Camera model: `ensamb_with_standoffs/down_camera`
- Camera SDF pose: `simulation/models/ensamb_with_standoffs/model.sdf`
- Sensor: 640x480, 10Hz, 수평 FOV 2.7507rad
- 표적: world 좌표 `(0, 4)`에 있는 `target_basket`

실행한 pose probe 결과:

- camera link quaternion: `(0, 0, 0.707106781, 0.707106781)`
- sensor quaternion: `(-0.5, 0.5, 0.5, 0.5)`
- optical axis in body: `(0, 0, -1)`
- optical axis in world: `(0, 0, -1)`

따라서 현재 카메라에서는 이미지 위쪽이 body forward(+x), 이미지 오른쪽이
body right(+y)이다. 카메라 local +X는 아래쪽을 향하고, local +Z가 body
forward, local +Y가 body right로 매핑된다.

## 수정한 변환과 예상 부호

`control/pos_calculator.cpp`의 변환은 다음 규칙을 사용한다.

```text
body_forward_m =  y_px / focal_length * altitude
body_right_m   =  x_px / focal_length * altitude
vx = clamp(gain * body_forward_m)
vy = clamp(gain * body_right_m)
```

합성 입력 테스트(`control/tests/test_pos_calculator.cpp`) 결과:

| 입력 | 예상 결과 |
|---|---|
| 화면 중앙 | `vx=0`, `vy=0` |
| 화면 위쪽 | `vx > 0` |
| 화면 아래쪽 | `vx < 0` |
| 화면 왼쪽 | `vy < 0` |
| 화면 오른쪽 | `vy > 0` |

이 테스트는 통과했다.

## 관측 CSV

`control/control.cpp`의 FlightLogger CSV에 다음 필드를 추가했다.

`u_px,v_px,normalized_x,normalized_y,body_forward_m,body_right_m,observation_age_ms,vy`

최신 validation CSV:

`logs/flight_20260816_211302.csv`

대표 관측:

```text
u_px=0.10, v_px=166.90, normalized_y=0.70,
body_forward_m=1.57, body_right_m=0.00,
observation_age_ms=43.10, vx=0.10, vy=0.00
```

표적이 화면 위쪽에 있을 때 `body_forward_m`과 `vx`가 모두 양수이므로, 실제
runtime 축과 guidance 부호가 일치한다. 최신 실행에서는 434개 추적 행이
기록됐고, 마지막 추적 시점은 약 21.8초였다.

## 속도 제한 validation

검증 명령은 다음 profile을 사용했다.

```bash
env -u DISPLAY -u WAYLAND_DISPLAY \
  YOLO_READY_TIMEOUT_SEC=90 \
  simulation/scripts/run_simulation.sh \
    --profile sitl-flight \
    --simulation-profile control-validation \
    --headless --duration 35 --upload-mission --arm
```

`control-validation`에서만 `CONTROL_VALIDATION_MAX_SPEED_MPS=0.1`을 적용한다.
기본 production 설정 `setting/MAVLink.yaml`의 `max_forward_speed: 0.5`는
그대로다. 최신 실행은 `vx=0.10`으로 제한된 상태에서 추적했고, loopback
SITL에서만 vehicle command를 전송했다.

## 표적 유실 분리 진단

이전 실행 `simulation/logs/run_sitl-flight_20260816_210742/`와
`logs/flight_20260816_210822.csv`에서는 마지막 유효 표적 직후 `tracking=0`이
되었지만 `observation_age_ms`는 약 20~42ms였다. 즉 frame/HTTP 관측 자체는
신선했으며, 카메라 timeout으로 판단할 근거는 없었다. 해당 구간은 YOLO의
유효 표적 또는 confirmation 상태가 탈락한 tracking freshness 문제로
분류한다.

그 실행에서 control은 표적 유실 후 `vx=0`, `vy=0`을 보냈고, 기존 정책대로
10초 뒤 LOITER 호버링을 유지했다. LOITER 안전 정책은 변경하지 않았다.

이번 최신 실행 `simulation/logs/run_sitl-flight_20260816_211217/`에서는
추적이 약 21.8초까지 계속됐으며, 유실 전환은 발생하지 않았다. 따라서 현재
남은 문제는 부호가 아니라 탐지/확정 상태가 끊기는 조건을 별도로 재현하는
것이다.

## 검증 결과

- `control` build: 통과
- CTest: 6/6 통과
- shell syntax 검사: 통과
- pose probe: 통과
- Gazebo camera topic: 생성 및 C++ YOLO 입력 확인
- YOLO TensorRT engine: `YOLO_MODEL/best_v5_wsl.engine` 로드 확인
- SITL heartbeat: 최신 audit에서 유효 ArduPilot heartbeat 64개, 3초 timeout 초과 없음
- 최신 SITL audit: loopback vehicle-affecting command 446개 기록
- 실제 Pixhawk/serial/LAN: 사용하지 않음
- 실행 종료: 추적 프로세스 정리 완료. Gazebo만 SIGTERM에 응답하지 않아 launcher가 해당 tracked process group에 제한적으로 SIGINT/SIGKILL을 사용함

Shadow 실행은 YOLO와 target-distance까지는 시작됐지만 control이 첫 heartbeat/setup
대기 상태에 머물러 guidance/ShadowCommandSink를 실제로 exercise하지 못했다. 따라서
shadow 결과를 guidance 성공으로 표시하지 않는다.
