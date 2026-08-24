# Astrodrone 구조

이 문서는 현재 실행 코드 기준의 전체 구조를 요약한다. 실행 명령은
[OPERATIONS.md](OPERATIONS.md), 변경 이력은
[developinglogHJ.md](developinglogHJ.md)를 참고한다.

## 1. 전체 흐름
### SITL
```text
Gazebo camera
  -> YOLO_MODEL/cpp/yolo_live.cpp
  -> HTTP /target :8002
  -> control/target_distance.cpp
  -> TargetRangeMsg UDP :15020
  -> FlightMissionApp + SafetyMonitor + 선택 알고리즘
  -> AutopilotMavlinkAdapter
  -> tcp:127.0.0.1:5760
  -> ArduPilot SITL
```
### 실기체

```text
Jetson CSI camera
  -> C++ TensorRT YOLO
  -> /target
  -> target-distance
  -> FlightMissionApp + SafetyMonitor + 알고리즘
  -> AutopilotMavlinkAdapter
  -> /dev/serial/by-id/... Pixhawk
```
Adapter가 받은 telemetry는 별도 UDP로 fan-out한다.

```text
AutopilotMavlinkAdapter
  -> :14551 telemetry
  -> :14553 GCS/Mission Planner read-only telemetry
  -> :14554 health_check read-only telemetry
```
GCS 영상은 YOLO의 비동기 JPEG publisher가 보내고, GCS가 detection metadata를
사용해 bbox를 그린다. GCS와 영상은 비행 명령 경로가 아니다.

## 2. 폴더별 역할
| 폴더 | 역할 |
|---|---|
| `control/app/` | 실행 조립, runtime, preflight, 비행 단계 |
| `control/autopilot/` | MAVLink transport, telemetry 상태, 명령 API |
| `control/safety/` | freshness, preflight, 승인, failsafe 판단 |
| `control/mission/` | mission setup와 두 guidance 알고리즘 |
| `control/` | target observation과 거리 계산 연결 |
| `YOLO_MODEL/cpp/` | camera 입력, TensorRT 추론, `/target`, 영상 publisher |
| `setting/` | runtime, MAVLink, camera, target 설정 |
| `scripts/` | C++ 빌드와 실기체 실행 wrapper |
| `simulation/` | Gazebo world/model과 SITL launcher |
| `gcs/` | telemetry/video 송신과 노트북 viewer |
| `control/tools/` | read-only 상태 확인 도구 |

## 3. 실행에 관여하는 주요 파일
| 파일 | 역할 |
|---|---|
| `control/app/main.cpp` | CLI를 읽고 `FlightMissionApp`을 실행 |
| `control/app/flight_mission_app.cpp` | 전체 lifecycle과 모듈 연결 |
| `control/app/runtime_config.*` | target, mode, endpoint 설정 해석 |
| `control/app/preflight_readiness.*` | 시작 전 readiness 확인 |
| `control/autopilot/autopilot_mavlink_adapter.*` | MAVLink 연결, 수신, 상태 갱신, 명령 전송 |
| `control/autopilot/autopilot_state.hpp` | 차량 telemetry의 단일 상태 원본 |
| `control/autopilot/mav_transport.hpp` | UDP/TCP/serial transport 추상화 |
| `control/safety/safety_monitor.*` | health, 승인, ControlLocked, failsafe 판단 |
| `control/mission/mission_setup.*` | mission clear/upload/read-back, AUTO, ARM 준비 |
| `control/mission/centered_vertical_descent_algorithm.*` | 중앙 정렬 후 수직 하강 |
| `control/mission/diagonal_approach_algorithm.*` | 45도 사선 접근 |
| `control/target_distance.cpp` | YOLO pixel observation과 target observation 생성 |
| `YOLO_MODEL/cpp/frame_source.*` | Gazebo 또는 Jetson frame 입력 |
| `YOLO_MODEL/cpp/tensorrt_engine.*` | TensorRT 전처리와 추론 |
| `YOLO_MODEL/cpp/yolo_live.cpp` | detection, confirmation, `/target` 제공 |
| `YOLO_MODEL/cpp/gcs_video_publisher.*` | 비동기 축소 JPEG 전송 |
| `gcs/sender/telem_sender_main.cpp` | telemetry와 target metadata를 GCS로 전달 |
| `gcs/tools/gcs_bridge.py` | 노트북 UDP 수신 및 브라우저 연결 |
| `gcs/tools/dashboard.html` | 영상, bbox, 상태 overlay 표시 |
| `control/tools/health_check.cpp` | read-only telemetry 상태 출력 |

## 4. 실행 모드
| 모드 | 동작 | 차량 명령 |
|---|---|---|
| `observe` | telemetry와 readiness만 확인 | 금지 |
| `shadow` | camera, YOLO, target observation, guidance 계산 | 차단 |
| `flight` | preflight와 승인 후 mission/control 실행 | 허용 가능 |

`flight`는 기본 차단 상태다. SITL은 loopback endpoint를 사용하고, 실기체는
명시된 real 승인과 serial 정책을 모두 통과해야 한다. 실기체의 Pixhawk serial은
`AutopilotMavlinkAdapter`만 직접 소유한다.

## 5. 통신 경계
| 용도 | SITL | 실기체 |
|---|---|---|
| MAVLink 연결 | `tcp:127.0.0.1:5760` | `/dev/serial/by-id/...` |
| target observation | `udp:127.0.0.1:15020` | 동일한 onboard 경로 |
| GCS target metadata | `udp:127.0.0.1:15022` | GCS 전용 fan-out |
| GCS telemetry | `udp:127.0.0.1:14553` | router/adapter 설정에 따름 |
| health telemetry | `udp:127.0.0.1:14554` | 동일한 read-only fan-out |
| YOLO HTTP | `127.0.0.1:8002` | onboard local endpoint |

SITL과 실기체 모두 실제 vehicle 상태의 원본은 Adapter의 `AutopilotState`다.
target-distance, GCS, health_check는 차량 serial을 직접 열지 않는다. GCS는
ARM, mode 변경, takeoff, LAND, velocity를 보내지 않는다.

## 6. 알고리즘
### 중앙 정렬 후 수직 하강
`CenteredVerticalDescentAlgorithm`은 약 5m 탐색 고도에서 fresh 표적을 기다린다.
`CENTERING`에서 수평 오차를 줄이고, `CENTER_DWELL` 동안 중앙 상태를 유지한다.
그 후 `VERTICAL_DESCENT`에서 하강하며 실제 고도가 1m 이하가 되면
`HOLD_1M`에서 zero velocity를 유지한다. 하강 중에는 필요한 범위에서만 수평
재정렬을 수행한다.

### 45도 사선 접근
`DiagonalApproachAlgorithm`은 표적을 확인하고 중앙을 맞춘 뒤
`APPROACH_45_DEG`로 진입한다. 기본 경로각은 설정의
`diagonal_path_angle_deg: 45.0`을 사용한다. 접근 조건을 만족하거나 고도가
1m 이하가 되면 속도를 멈추고 최종 중앙 유지와 호버로 전환한다.

두 알고리즘은 MAVLink를 직접 다루지 않는다. 계산 결과는 App과 SafetyMonitor를
거쳐 승인된 경우에만 Adapter 명령 API로 전달된다.

## 7. 표적 유실과 안전 처리
- GUIDED 중 표적이 유실되면 GUIDED를 유지하고 zero velocity를 보낸다.
- App은 `REACQUIRE`에서 같은 표적의 fresh 관측과 재탐지 dwell을 기다린다.
- 재탐지 조건을 만족하면 유실 직전 알고리즘 상태를 이어갈 수 있다.
- `HOLD_1M`은 재탐지되어도 하강을 재개하지 않는다.
- heartbeat, GPS, EKF, 배터리, 고도 또는 RC가 stale이면 SafetyMonitor가 명령을 차단한다.
- 외부에서 모드가 바뀌면 ControlLocked와 안전 호버 정책을 적용하며 자동으로
  비행을 재개하지 않는다.
- LOITER, LAND, DISARM은 표적 유실 자체가 아니라 현재 safety 정책과 lifecycle
  조건에 따라 결정된다.

## 8. 공통점과 차이점
| 항목 | SITL | 실기체 |
|---|---|---|
| camera | Gazebo frame source | Jetson CSI frame source |
| MAVLink | loopback TCP | onboard serial |
| control core | 동일한 C++ App/Adapter/Safety | 동일한 C++ App/Adapter/Safety |
| guidance | 동일한 두 알고리즘 | 동일한 두 알고리즘 |
| vehicle | ArduPilot SITL | Pixhawk |
| 승인 | SITL 실행 옵션과 FLY | real 승인, serial 소유권, FLY |
| 위험 | Gazebo/FDM 지연 | 실제 센서와 serial 환경 |

두 환경은 control과 guidance의 책임 구조를 공유하지만 camera와 MAVLink
transport만 분리한다. 실제 장치 검증 상태와 운용 절차는
[OPERATIONS.md](OPERATIONS.md)에 기록한다.
