# Architecture

> 분석 기준: `Document` 브랜치, 커밋 `81b6b6e (0808(final))`
> 이 문서는 저장소의 소스·설정·문서·Git 이력을 읽어 작성한 1단계 아키텍처 분석이다. 분석 단계에서는 빌드, 실행, MAVLink 송신 및 파일 변경을 수행하지 않았다. 모델 가중치와 압축 파일 같은 바이너리의 내부 동작은 정적 분석 대상에서 제외했다.

표기:

- **[현재 확인]** 저장소 코드나 설정에서 직접 확인된 사실
- **[설계 제안]** 다음 단계에 적용할 구조
- **[미확정]** 실측, 정책 결정 또는 SITL 검증이 필요한 내용

문서 안의 파일 경로는 모두 저장소 루트를 기준으로 한 상대경로다.

---

## 1. 현재 구조 요약

### 1.1 Git 이력

**[현재 확인]**

Git 이력상 프로젝트는 다음 순서로 발전했다.

1. Python 기반 MAVLink 연결·제어 예제
2. 설정 파일 중앙화
3. C++ 제어 코드와 MAVLink 헤더 도입
4. YOLO → 거리 계산 → UDP → 제어 파이프라인 추가
5. 고도·배터리·GPS·EKF·heartbeat 안전 검사와 CSV 로그 추가
6. `emergency.cpp`를 자동 명령 송신 프로세스가 아닌 진단 도구로 변경

주요 이력은 `53ce59d(Python→C++)`, `02d6727(target approach pipeline)`, `81b6b6e(safety/logging final)`에서 확인된다.

현재 문서 일부는 Python 시절 구조를 설명하고 있어 실제 C++ 구조와 맞지 않는다. 근거: `Document/README.md`, `Document/developinglogMJ.md`.

### 1.2 주요 디렉터리

| 디렉터리                       | 현재 역할                                                            |
| ------------------------------ | -------------------------------------------------------------------- |
| `control/`                     | C++ MAVLink 연결, 자동 비행, 진단 도구, YOLO 중계                    |
| `YOLO_MODEL/`                  | IMX219 영상 처리와 YOLO 추론, HTTP 결과 제공                         |
| `setting/`                     | MAVLink 주소, 안전값, 카메라, HTTP/UDP 포트, Pixhawk 파라미터 스냅샷 |
| `Document/`                    | 개발 계획과 변경 로그                                                |
| `test_cam/`                    | 카메라 송출·FPS 측정 도구                                            |
| `jetson_power_test/`           | Jetson 부하·온도·전력 측정                                           |
| `control/third_party/mavlink/` | 생성된 MAVLink C 헤더                                                |
| `control/tests/`               | 장치·네트워크를 사용하지 않는 CTest 단위/저수준 characterization 테스트 |

현재 저장소에는 Gazebo world/vehicle 모델, SITL 실행 구성과 GCS 소스가 없다. 자동화 테스트는 2단계 1차 변경에서 JSON parser와 `MavConnection` 저수준 범위부터 추가됐다.

### 1.3 실행 파일별 책임

| 실행 파일          | 현재 코드에서 확인되는 책임                                                   |
| ------------------ | ----------------------------------------------------------------------------- |
| `control`          | GUIDED 요청, ARM, 이륙, 목표 접근, safety 판단, LOITER/GUIDED/LAND 요청, 로그 |
| `target_distance`  | YOLO HTTP 조회, Pixhawk 고도 수신, 픽셀 오프셋을 거리로 변환, UDP 전송        |
| `check_link`       | heartbeat, 배터리, GPS, 상태 텍스트 관찰                                      |
| `check_alt`        | 고도 관찰과 제한 초과 경고                                                    |
| `get_gps_location` | GPS 정보 관찰                                                                 |
| `emergency`        | 배터리/GPS/EKF/heartbeat 진단 출력                                            |
| `test_arm`         | 모드 변경 및 ARM/DISARM 시험                                                  |
| `yolo_live.py`     | 카메라 캡처, YOLO 추론, 검출 안정화, `/target` HTTP 응답                      |

`pos_calculator.cpp`는 비어 있으며 빌드 대상도 아니다.

### 1.4 현재 미션 동작

**[현재 확인]**

현재 `control/control.cpp`는 의도한 AUTO 미션 감시 프로그램이 아니다. `main()`이 다음을 순차 실행한다.

1. `real.proxy_udp` 연결
2. GUIDED 요청
3. 대기
4. ARM
5. 목표 고도 `10`으로 takeoff
6. `approach_target()`을 정해진 시간 실행
7. LAND

AUTO 모드, Mission Planner mission index, `MISSION_CURRENT`, 기존 waypoint 재개는 처리하지 않는다.

또한 `control/drone_lib.cpp::set_mode()`는 MAVLink `SET_MODE` 메시지를 송신하고 즉시 성공을 반환한다. 실제 HEARTBEAT mode를 기다리지 않으며, 사용 중인 `SET_MODE` 방식에서 COMMAND_ACK가 제공되는지 확인하거나 처리하는 코드도 없다. 따라서 현재 코드는 request 송신과 실제 mode 전환을 구분하지 않는다.

### 1.5 현재 설정

**[현재 확인]**

`setting/MAVLink.yaml`에는 다음 연결이 있다.

- 실제 serial: `/dev/ttyACM0`, 115200
- 실제 proxy UDP: `udp:127.0.0.1:14550`
- SITL TCP: `tcp:127.0.0.1:5762`
- SITL UDP: `udp:0.0.0.0:14561`
- YOLO HTTP: TCP 8002
- 내부 목표 데이터: UDP 15020

**[현재 확인]**

현재 파라미터 스냅샷 `setting/mav.parm`에는 다음 값이 기록되어 있다.

- `MIS_RESTART=0`
- `GUID_TIMEOUT=3`
- `FS_GCS_ENABLE=0`
- `FENCE_ENABLE=0`
- `BATT_MONITOR=0`
- `FLTMODE_CH=5`
- 확인된 `RCx_OPTION`은 모두 0

단, 이 파일은 런타임에 프로그램이 읽는 설정이 아니며 실제 Pixhawk에 같은 값이 적용되어 있다는 증거도 아니다. 따라서 모두 **[미확정] 실제 기체 확인 필요**다.

---

## 2. 현재 데이터 흐름

### 2.1 YOLO에서 Pixhawk까지

**[현재 확인]**

```text
IMX219
  ↓
YOLO_MODEL/yolo_live.py
  - 640×480 캡처
  - YOLO 추론
  - 최고 confidence 객체 선택
  - 같은 class의 단일 검출이 연속된 경우 confirmed
  - HTTP /target 제공
  ↓ HTTP polling
control/target_distance.cpp
  - x_px, y_px 읽기
  - Pixhawk ALTITUDE 수신
  - pixel_to_meter 고정 배율로 ground_offset 계산
  - distance = hypot(ground_offset, altitude)
  ↓ localhost UDP :15020
control/target_link.cpp
  ↓
control/control.cpp::approach_target()
  - vx: 계산 거리와 stop_distance 차이
  - vy: x_px에 gain 적용
  - vz: 고도 제한 대응
  ↓
drone::send_velocity_body()
  ↓ SET_POSITION_TARGET_LOCAL_NED / BODY_OFFSET_NED
Pixhawk / ArduCopter
```

YOLO 코드는 MAVLink를 직접 송신하지 않는다.

### 2.2 현재 필요한 동시 프로세스

현재 목표 접근 파이프라인에는 최소한 다음 세 프로세스가 필요하다.

1. `yolo_live.py`
2. `target_distance`
3. `control`

또한 `real.proxy_udp`를 사용하려면 Pixhawk serial과 UDP endpoint를 연결하는 외부 MAVLink router/proxy가 필요해 보이지만, 그 구성과 실행 방법은 저장소에 없다.

### 2.3 확인된 파이프라인 결함

**[현재 확인]**

`yolo_live.py`는 기본 `json.dumps()`를 사용하므로 다음처럼 공백이 포함된 JSON을 반환한다.

```json
{ "found": true, "confirmed": true }
```

1단계 분석 당시 `target_distance.cpp::json_bool()`은 `:` 바로 다음 문자가 `true`일 것으로 가정했다. 따라서 이 조합의 공백 때문에 `found`와 `confirmed`를 false로 해석하는 결함이 있었다.

근거:

- `YOLO_MODEL/yolo_live.py::_json()`
- 변경 전 `control/target_distance.cpp::json_bool()`
- 변경 후 `control/target_json.cpp::json_bool()` 및 `control/tests/test_target_json.cpp`

2단계 1차 변경에서는 해당 사례를 회귀 테스트로 고정한 뒤, 정상 JSON 공백을 처리하는 작은 top-level field parser로 교체했다. missing, null 또는 잘못된 token은 값 없음으로 반환한다. 이 parser는 `/target`의 최상위 boolean/number만 지원하며 범용 JSON parser는 아니다.

---

## 3. 발견된 구조 및 안전 문제

### 3.1 MAVLink 연결 생성 위치

| 위치                        | 연결 및 송신 가능성                                      |
| --------------------------- | -------------------------------------------------------- |
| `control.cpp`               | 전역 `drone::connect()`를 통해 연결; 모든 비행 명령 가능 |
| `target_distance.cpp`       | 독립 연결; `MAV_CMD_SET_MESSAGE_INTERVAL` 송신           |
| `emergency.cpp`             | 독립 연결; message interval 명령 송신                    |
| `check_alt.cpp`             | 독립 연결; message interval 명령 송신                    |
| `get_gps_location.cpp`      | 독립 연결; message interval 명령 송신                    |
| `check_link.cpp`            | 독립 연결; 현재 소스상 수신 전용                         |
| `test_arm.cpp`              | 독립 연결; 모드 및 ARM/DISARM 송신                       |
| `play_bacchanale_motors.py` | serial 연결; parameter 변경과 reboot 명령                |
| `inspect_pixhawk_audio.py`  | serial 연결; 버전·parameter 요청 송신                    |

`MavConnection::send()`가 공개되어 있어 구조적으로 어느 실행 파일도 송신을 우회할 수 있다.

### 3.2 vehicle-affecting 명령 위치

**[현재 확인]**

- `control.cpp`
  - GUIDED, LOITER
  - ARM
  - takeoff
  - body velocity, 0 velocity
  - LAND
- `test_arm.cpp`
  - mode 변경
  - ARM/DISARM
- `play_bacchanale_motors.py`
  - parameter 변경
  - Pixhawk reboot

`target_distance`, `emergency`, `check_alt`, `get_gps_location`은 비행 이동 명령은 보내지 않지만 `MAV_CMD_SET_MESSAGE_INTERVAL`을 보낸다. 따라서 엄밀한 receive-only 도구는 아니다.

### 3.3 연결 구조 위험

**[현재 확인]**

여러 프로그램이 기본값으로 동일한 `udp:127.0.0.1:14550`에 각각 bind한다. `SO_REUSEADDR` 사용은 모든 프로세스에 같은 MAVLink packet이 복제 전달되는 것을 보장하지 않는다.

따라서 현재 구조는 다음 문제가 있다.

- telemetry가 프로세스 사이에서 나뉠 수 있음
- ACK가 요청한 프로세스가 아닌 다른 socket으로 갈 수 있음
- 각 프로세스의 vehicle 상태가 불완전할 수 있음
- 미래 GCS가 같은 endpoint에 추가되면 충돌 가능

**[설계 제안]** Pixhawk와 연결하는 MAVLink 소유자는 한 프로세스로 제한하고, GCS와 진단 도구에는 MAVLink router의 별도 endpoint 또는 내부 read-only telemetry bus를 제공한다.

### 3.4 안전 문제

1. **AUTO 미션이 구현되지 않았다.** `control.cpp::main()`은 곧바로 GUIDED·ARM·이륙을 요청한다.
2. **모드 전환 성공을 검증하지 않는다.** `drone::set_mode()`는 송신 성공을 실제 모드 성공으로 간주한다.
3. **현재 safety가 LOITER를 요청한다.** `approach_target()`은 battery/GPS/EKF/heartbeat 문제에서 LOITER를 요청하고 회복 시 GUIDED를 자동 요청한다. 이는 새 정책과 충돌한다.
4. **manual takeover와 lockout이 없다.** `RC_CHANNELS` 수신, 허용 스위치, latch, 외부 모드 변경 판별이 없다.
5. **명령 gate가 없다.** Safety, mission, guidance 또는 진단 코드가 직접 송신할 수 있다.
6. **이륙 고도와 software ceiling이 충돌한다.** 이륙 목표는 코드에 10으로 고정되어 있지만 현재 `safety.yaml` hard ceiling은 5다. 고도 제한은 접근 루프에 들어간 뒤에만 적용된다.
7. **고도 safety가 목표 UDP freshness에 결합되어 있다.** 목표 데이터가 끊기면 `last.altitude_m` 사용도 끊겨 고도 대응이 사라진다.
8. **telemetry freshness가 불완전하다.** heartbeat 이외 battery/GPS/EKF 값에 개별 수신 시각과 만료 판단이 없다.
9. **목표 유실 정책이 다르다.** 현재 코드는 target이 stale이면 접근 velocity를 0으로 만들 수 있지만, 동일 목표 재탐지, 제한 시간, AUTO/LAND 정책 상태머신은 없다.
10. **픽셀-거리 계산이 검증되지 않았다.** `pixel_to_meter=0.01` 고정 배율을 사용하며 FOV, 카메라 보정, 자세가 반영되지 않는다.
11. **`y_px`가 실제 유도에 사용되지 않는다.** 거리를 계산하는 데는 포함되지만 접근 제어는 `vx`와 `x_px→vy` 중심이다.
12. **목표 동일성이 보장되지 않는다.** `yolo_live.py`는 같은 class가 연속 검출되면 confirmed로 판단한다. 물리적으로 같은 객체인지 확인하지 않는다.
13. **다중 객체 정책이 단순하다.** 최고 confidence 하나를 선택하며 다중 검출 시 confirmation streak가 깨진다.
14. **UDP 내부 자료형이 취약하다.** `TargetRangeMsg` C++ 구조체를 그대로 보낸다. version, magic, 단위 명시, source timestamp, endian/packing 계약이 없다.
15. **예외 종료 시 안전 동작이 일관적이지 않다.** `approach_target()`의 `FinallyGuard`는 0 velocity를 보내지만, 그 외 단계에서 예외가 나면 같은 보장이 없다. 반대로 미래 takeover latch 뒤에는 이 0 velocity조차 금지되어야 한다.
16. **ArduPilot 자체 failsafe 설정이 확인되지 않았다.** `mav.parm` 스냅샷은 `FS_GCS_ENABLE=0`, `FENCE_ENABLE=0`, `BATT_MONITOR=0`이지만 실제 FC 설정인지 알 수 없다.
17. **물리 RC 수동조종은 현장에서 가능했지만 Jetson 수신은 확인되지 않았다.** 조종자가 송신기의 mode switch와 stick으로 기체를 수동 조종한 경험은 있으나, Jetson 애플리케이션이 MAVLink `RC_CHANNELS`를 안정적으로 수신하는지, 어떤 endpoint와 주기로 수신하는지는 현재 코드와 기록으로 검증되지 않았다.

### 3.5 문서와 구현의 차이

**구현됐지만 오래된 문서와 다른 내용**

- Python 제어가 아니라 C++ 제어다.
- `control_sim`은 현재 CMake 대상이 아니다.
- `emergency.cpp`는 자동 LOITER 송신기가 아니라 진단 프로그램이다.
- `PID.py`는 존재하지 않는다.
- CSV flight logging은 현재 `control.cpp::FlightLogger`에 있다.

**문서 또는 개발 의도에는 있지만 구현되지 않은 내용**

- AUTO mission 감시
- AUTO→GUIDED 인계 상태머신
- mode request 방식별 optional COMMAND_ACK 처리와 fresh HEARTBEAT actual mode 확인
- mission index 저장과 AUTO 재개
- RC takeover
- 외부 mode change lockout
- 중앙 command gate
- 목표 유실 timeout 정책
- 실거리 geometry
- 상태머신과 SITL 자동화 테스트. 현재는 JSON parser와 `MavConnection`의 장치 없는 CTest만 존재
- Gazebo 및 GCS 통합

### 3.6 과도하게 결합된 책임

`control.cpp::approach_target()` 하나가 다음을 모두 담당한다.

- telemetry 요청과 수신
- safety 상태 계산
- 모드 변경
- 목표 freshness 판단
- guidance 계산
- 고도 제한
- 명령 송신
- 로그 기록
- 종료 처리

`main()` 역시 설정 로드뿐 아니라 실제 GUIDED, ARM, takeoff, LAND를 직접 조립한다.

---

## 4. 제안 모듈 구조

### 4.1 데이터 흐름

**[설계 제안]**

```text
카메라/YOLO → target ───────────────┐
                 TargetObservation  │
                 TargetEstimate     ↓
Pixhawk MAVLink → autopilot → VehicleState
                         ├→ safety → SafetyStatus / ControlAuthorityState
                         └─────────→ mission
                                      - 7개 상태
                                      - guidance 순수 함수
                                      - 종료정책 선택
                                            ↓ CommandRequest
                              autopilot::CommandSender
                                - 최종 authority 재확인
                                - freshness/유효시간 확인
                                - 실제 송신 또는 차단
                                            ↓
                                      Pixhawk MAVLink
```

모든 단계의 입력·출력과 gate 결과는 logging으로 복사하지만 logging은 판단에 관여하지 않는다.

현재 프로젝트에는 `app`, `autopilot`, `target`, `mission`, `safety`, `logging`의 6개 논리 모듈이면 충분하다. 모듈은 반드시 독립 라이브러리나 class 하나를 의미하지 않는다. 같은 디렉터리 안에서 파일과 순수 함수로 책임을 분리할 수 있다.

- perception과 geometry는 우선 `target` 안에서 파일로 분리한다.
- mission과 guidance는 `mission` 안에서 상태 전이 파일과 순수 함수로 분리한다.
- control authority는 `safety`에 포함한다.
- 중앙 command gate는 `autopilot::CommandSender`에 포함한다.
- 독립 변경 주기, 별도 테스트·배포, 재사용 또는 소유권이 생길 때만 별도 모듈로 승격한다.

### 4.2 모듈 책임

| 모듈        | 입력                                                          | 출력                                                     | 금지되는 의존성                                       |
| ----------- | ------------------------------------------------------------- | -------------------------------------------------------- | ----------------------------------------------------- |
| `app`       | 설정, 모듈 초기화 결과                                        | control loop와 automation session                        | 알고리즘·packet 생성·정책 구현                        |
| `autopilot` | MAVLink bytes, `CommandRequest`, authority snapshot           | `VehicleState`, optional ACK evidence, `CommandDecision` | target 선택, mission/safety 정책                      |
| `target`    | 카메라/YOLO, 카메라 보정, 필요한 read-only vehicle data       | `TargetObservation`, `TargetEstimate`                    | MAVLink 송신, mode/종료정책                           |
| `mission`   | vehicle, target, safety, authority                            | 상태 전이, `GuidanceSetpoint`, `CommandRequest`          | MAVLink packet 생성, 직접 송신, safety threshold 평가 |
| `safety`    | vehicle/target freshness, mode evidence, optional RC evidence | `SafetyStatus`, `ControlAuthorityState`                  | 직접 MAVLink 송신, guidance 계산                      |
| `logging`   | 각 모듈 이벤트 사본                                           | 구조화 로그                                              | 비행 판단·상태 전이·명령 송신                         |

`app/main`은 설정 로드, 객체와 함수 연결, 프로그램 시작 mode baseline 확정, automation session 시작·종료와 전체 loop 조립만 담당한다.

`autopilot`은 `mavlink_connection`, `vehicle_state`, `mode_request_tracker`, `command_sender` 같은 파일로 나눈다. production mission runtime의 모든 vehicle-affecting 명령은 하나의 `CommandSender`를 통과한다. `CommandSender`는 실제 송신 직전에 lock과 request expiry를 다시 확인한다.

`target`은 `yolo_adapter`, `target_confirmation`, `target_geometry`, `target_freshness` 같은 파일로 나눈다. 초기에는 Python YOLO 프로세스와 C++ adapter 구성을 유지해도 된다. target 모듈 통합은 반드시 단일 프로세스 통합을 의미하지 않는다. 보정 근거가 부족하면 meter 위치를 만들지 않고 방향 또는 invalid 결과만 반환한다.

`mission`은 `mission_state`, `mission_transition`, `guidance`, `exit_policy` 같은 파일로 나눈다. guidance와 종료정책 선택은 가능한 한 순수 함수로 작성한다.

`safety`는 safety 평가와 control authority latch를 함께 소유한다. ArduPilot failsafe는 이 모듈의 하위 기능이 아니라 FC에서 독립적으로 유지되는 보호 계층이다.

### 4.3 제안 자료형

불필요한 거대 공통 구조체 대신 목적별 구조체를 둔다.

```cpp
struct TargetObservation {
    Timestamp observed_at;
    uint64_t frame_id;
    bool found;
    bool confirmed;
    TargetClass class_id;
    float confidence;
    PixelPoint center_px;
    ImageSize image_size;
    uint32_t detection_count;
    Optional<TargetTrackId> track_id;
};
```

`track_id` 생성 방식은 미확정이며, 없을 때 같은 class만으로 동일 목표라고 단정하면 안 된다.

```cpp
struct TargetEstimate {
    Timestamp source_observed_at;
    bool valid;
    CoordinateFrame frame;
    Vector3 direction;
    Optional<Vector3> position_m;
    GeometryQuality quality;
};
```

실제 거리를 계산할 근거가 부족하면 `position_m`을 만들지 않고 방향만 제공한다.

```cpp
struct VehicleState {
    TimedValue<FlightMode> actual_mode;
    TimedValue<bool> armed;
    TimedValue<Attitude> attitude;
    TimedValue<Altitude> altitude;
    TimedValue<Position> position;
    TimedValue<BatteryState> battery;
    TimedValue<GpsState> gps;
    TimedValue<EkfState> ekf;
    Optional<TimedValue<RcChannels>> rc;
    TimedValue<uint16_t> mission_index;
    Timestamp last_heartbeat_at;
};
```

각 telemetry 필드가 자체 timestamp를 가져야 한다.

```cpp
enum class ModeAckSupport {
    Unknown,
    Supported,
    Unsupported
};

enum class ModeRequestMethod {
    SetModeMessage,
    CommandLongDoSetMode
};

struct ModeRequestState {
    // Jetson 내부 추적 ID다. MAVLink COMMAND_ACK가 이 값을 반환한다고
    // 가정하지 않는다.
    ModeRequestId internal_id;
    FlightMode requested_mode;
    ModeRequestMethod request_method;
    Timestamp sent_at;
    Timestamp expires_at;
    ModeAckSupport ack_support;
    Optional<CommandAck> ack;
    bool actual_mode_confirmed;
};
```

동시에 outstanding 상태가 될 수 있는 mode request는 하나로 제한한다. 사용한 request 방식이 ACK를 지원한다고 확인된 경우 명시적 ACK 거부는 즉시 실패로 처리한다. ACK 허용은 성공의 충분조건이 아니며 fresh HEARTBEAT의 actual mode가 요청 mode가 되어야 성공이다.

request 방식이 ACK를 지원하지 않거나 ACK가 도착하지 않은 경우에는 ACK 부재만으로 성공이나 실패를 결정하지 않는다. `mode-transition timeout` 동안 fresh HEARTBEAT를 기다리고, timeout 전에 actual mode가 요청 mode로 확인되면 성공, 끝까지 확인되지 않으면 실패다.

`COMMAND_ACK`는 Jetson 내부 `ModeRequestId`를 echo한다고 가정하지 않는다. ACK가 지원되는 방식에서는 command 종류, target, 송신 시각과 제한된 시간창을 현재 단일 outstanding 요청과 대조한다. 이전 요청에서 늦게 도착한 ACK를 현재 요청에 연결하지 않아야 한다. 이 규칙은 GUIDED, AUTO 복귀와 LAND mode 전환에 동일하게 적용한다.

```cpp
enum class MissionState {
    MonitoringAuto,
    RequestingGuided,
    TrackingTarget,
    TargetLost,
    RequestingExit,
    ControlLocked,
    Finished
};

enum class ExitAction {
    ResumeAuto,
    Land
};

enum class ControlLockReason {
    RcTakeoverEvidence,
    UnexpectedOperatorOrGcsModeChange,
    ArduPilotFailsafeModeChange,
    UnknownExternalModeChange
};

enum class MissionResult {
    AutoResumed,
    LandRequested,
    LandConfirmed,
    ControlLocked,
    Failed
};

struct ExitDecision {
    ExitAction action;
    FailureReason reason;
};

struct GuidanceSetpoint {
    CoordinateFrame frame;
    Vector3 velocity_mps;
    Optional<PositionSetpoint> position;
    Timestamp valid_until;
};
```

`MissionResult::LandRequested`는 LAND command 송신 사실만 나타내며 mode 전환 성공이나 착륙 완료를 뜻하지 않는다. fresh HEARTBEAT actual LAND는 LAND mode 전환 성공만 뜻한다. `MissionResult::LandConfirmed`와 mission 완료는 별도로 정할 실제 착륙 완료 telemetry와 disarm 조건이 충족된 뒤에만 기록한다.

```cpp
struct SafetyStatus {
    SafetySeverity severity;
    vector<SafetyReason> reasons;
    bool vehicle_state_fresh;
    bool target_fresh;
};

struct ControlAuthorityState {
    bool automation_session_active;
    bool initial_mode_established;
    bool locked;
    Optional<ControlLockReason> lock_reason;
    Optional<Timestamp> locked_at;
};

struct CommandRequest {
    CommandType type;
    CommandPayload payload;
    CommandOrigin origin;
    Timestamp created_at;
    Timestamp valid_until;
};

struct CommandDecision {
    bool allowed;
    GateBlockReason reason;
    ControlAuthorityState authority_snapshot;
    Optional<Timestamp> sent_at;
};
```

별도의 거대한 `MissionInput`이나 모든 모듈이 공유하는 공통 상태 구조체는 만들지 않는다. mission tick에는 필요한 자료형만 명시적으로 전달한다. 로그도 하나의 거대 record 대신 `StateTransitionEvent`, `ModeRequestEvent`, `ActualModeEvent`, `ControlLockEvent`, `TargetEvent`, `GuidanceEvent`, `CommandDecisionEvent`, `MissionFinishedEvent`처럼 event별 자료형을 사용한다.

guidance와 내부 실패 종료정책은 상태나 class를 추가하기보다 순수 함수로 시작한다.

```cpp
GuidanceSetpoint calculate_guidance(
    const TargetEstimate& target,
    const VehicleState& vehicle,
    const GuidanceLimits& limits);

ExitDecision select_exit_action(
    FailureReason reason,
    const VehicleState& vehicle,
    const SafetyStatus& safety,
    const MissionPolicy& policy);
```

### 4.4 접근 방식 비교

| 구분        | 후보 A: 수평 정렬 후 수직 하강                                                           | 후보 B: 수평+하강 사선 접근                                             |
| ----------- | ---------------------------------------------------------------------------------------- | ----------------------------------------------------------------------- |
| 장점        | 수평·수직 축을 따로 검증 가능, 제어가 비교적 단순, 하향 카메라에서 중심 유지 판단이 명확 | 접근 시간이 짧을 수 있고 정지·하강 단계를 줄일 수 있음                  |
| 단점        | 정렬 중 drift, 정지 시간이 길어짐, 하강 중 target 가림·downwash 가능                     | 축 결합이 강하고 tuning이 어려움, overshoot와 FOV 이탈 위험             |
| 필요한 입력 | 보정된 target 방향, 고도/range, 자세, 정렬 안정성                                        | 신뢰할 수 있는 3차원 상대 위치 또는 방향·range, 자세, 하강 경로         |
| 주요 위험   | 안전하지 않은 지면 위 수직 하강, 목표물 위 장애물, 거리 오차                             | 사선 경로 장애물, 고도 변화에 따른 영상 scale 변화, 수평·수직 오차 누적 |
| 검증 순서   | geometry 단위 테스트 → 정렬 SITL → 정지 안정성 → 하강 SITL                               | geometry 단위 테스트 → 수평만 → 하강만 → 결합 SITL → 교란 조건          |

**[미확정]** 이번 단계에서는 어느 방식을 선택하지 않는다.

---

## 5. 상태머신

아래 상태머신은 **[설계 제안]**이며 현재 구현된 동작이 아니다. target의 confirmed 판정, 공통 lock 상태, 공통 exit 상태와 terminal 결과를 사용해 7개 상태로 줄인다. 모든 timeout은 이름 있는 설정값을 사용하며 수치는 미확정이다.

### 5.1 상태 생명주기

| 상태               | 목적                                                                                       | 진입 조건                                                                | timeout                                                 | 정상 다음 상태                                                                                                                                            | 실패/외부개입 다음 상태                                                                                                    |
| ------------------ | ------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------ | ------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| `MonitoringAuto`   | actual AUTO와 confirmed target 감시                                                        | automation session 활성, actual AUTO, authority 허용                     | 없음                                                    | confirmed target이면 `RequestingGuided`                                                                                                                   | 내부 종료 필요 시 `RequestingExit`; 외부 mode 변경 시 `ControlLocked`                                                      |
| `RequestingGuided` | GUIDED 요청 후 optional ACK evidence와 actual mode 확인                                    | confirmed target, safety 허용, outstanding mode request 없음             | mode-transition timeout                                 | 요청이 유효하고 명시적 ACK 거부가 없으며 fresh actual GUIDED이면 `TrackingTarget`                                                                         | 지원되는 ACK의 명시적 거부 또는 timeout까지 actual GUIDED 미확인 시 `RequestingExit`; 외부 변경은 `ControlLocked`          |
| `TrackingTarget`   | 제한된 target 접근 setpoint 생성                                                           | current GUIDED request가 유효하고 fresh HEARTBEAT actual GUIDED가 확인됨 | 없음                                                    | target 유실 시 `TargetLost`; 접근 종료 시 `RequestingExit`                                                                                                | safety 종료는 `RequestingExit`; 외부 변경은 `ControlLocked`                                                                |
| `TargetLost`       | GUIDED 유지, 0 velocity 반복, 제한 시간 재탐지                                             | tracking 중 target lost/stale                                            | target-lost timeout                                     | 같은 target confirmed 시 `TrackingTarget`                                                                                                                 | timeout/safety는 `RequestingExit`; 외부 변경은 `ControlLocked`                                                             |
| `RequestingExit`   | `ExitAction`에 따른 AUTO/LAND request, optional ACK evidence, actual mode와 LAND 완료 확인 | target loss, safety 또는 내부 실패에서 종료정책 선택                     | mode-transition timeout 및 별도 LAND-completion timeout | fresh actual AUTO이면 `MonitoringAuto`; fresh actual LAND이면 mode 전환 성공으로 기록하고 착륙 완료를 계속 관찰; 실제 착륙 완료/disarm 확인 후 `Finished` | 지원되는 ACK의 명시적 거부, actual mode timeout 또는 모든 종료정책 실패 시 `Finished(Failed)`; 외부 변경은 `ControlLocked` |
| `ControlLocked`    | 모든 Jetson 비행 명령 차단                                                                 | 예상 밖 actual mode 변경 또는 검증된 RC takeover evidence                | 없음                                                    | 지상 reset 시 현재 session을 `Finished(ControlLocked)`로 종료                                                                                             | 자동 해제·자동 복귀 없음                                                                                                   |
| `Finished`         | session 결과와 실패 원인 관찰                                                              | LAND 완료, 명시적 종료, lock reset 또는 종료정책 실패                    | 없음                                                    | terminal                                                                                                                                                  | terminal                                                                                                                   |

### 5.2 상태별 명령과 로그

| 상태               | 허용 출력                                        | 금지 출력                                                          | 필수 로그                                                                                              |
| ------------------ | ------------------------------------------------ | ------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------ |
| `MonitoringAuto`   | 일반적으로 없음                                  | velocity, position, ARM, takeoff, LOITER                           | actual mode, mission index, confirmed/freshness, safety                                                |
| `RequestingGuided` | gate를 통과한 GUIDED 요청 하나                   | actual GUIDED 확인 전 모든 tracking setpoint, ARM, takeoff, LOITER | request 방식, ACK 지원 여부·수신값, 내부 request ID, fresh actual mode와 각 시각                       |
| `TrackingTarget`   | 제한된 velocity/position setpoint                | LOITER, 무단 mode/ARM/LAND                                         | observation, estimate, raw/clamped setpoint, 실제 송신                                                 |
| `TargetLost`       | authority가 허용된 동안 주기적 0 velocity        | 이동 setpoint, LOITER                                              | loss 시작, 마지막 target, hold 송신, 남은 timeout                                                      |
| `RequestingExit`   | `ExitAction::ResumeAuto` 또는 `ExitAction::Land` | 추적 setpoint, LOITER, 선택되지 않은 종료 명령                     | 선택 정책·근거, request 방식, ACK 지원 여부·수신값, fresh actual mode, LAND 완료/disarm, mission index |
| `ControlLocked`    | 없음                                             | 0 velocity와 LAND를 포함한 모든 vehicle-affecting 명령             | 이전/actual mode, `ControlLockReason`, evidence, 차단 명령 수                                          |
| `Finished`         | 없음                                             | 자동 비행 명령                                                     | `MissionResult`, `FailureReason`, 마지막 setpoint, actual mode, FC failsafe 상태                       |

### 5.3 주요 전이 규칙

- YOLO 1 frame 검출은 target 내부 confirmation만 갱신하며 mission은 `MonitoringAuto`를 유지한다.
- target이 confirmed이고 fresh할 때만 `RequestingGuided`로 간다.
- GUIDED request가 현재 유효한 단일 outstanding 요청이고 fresh HEARTBEAT actual GUIDED가 확인된 경우에만 `TrackingTarget`에 진입한다. ACK 지원 방식에서 명시적 거부가 수신되면 진입하지 않는다. ACK 허용만으로도 진입하지 않는다. ACK 비지원 또는 ACK 미수신이어도 timeout 전에 fresh actual GUIDED가 확인되면 성공이다.
- target 유실은 `TargetLost`로 전이하고 authority가 허용된 동안에만 0 velocity를 주기적으로 송신한다.
- 동일 target이 timeout 전 confirmed되면 `TrackingTarget`으로 돌아간다.
- target-loss timeout, telemetry/safety 문제와 Jetson 내부 실패는 `select_exit_action()`으로 `ExitAction`을 고른 뒤 `RequestingExit`로 간다.
- AUTO request는 ACK 지원 방식에서 명시적 거부가 수신되면 실패다. ACK 허용 여부와 관계없이 fresh actual AUTO가 확인되어야 AUTO 복귀 성공이며, ACK가 없더라도 timeout 전에 actual AUTO가 확인되면 성공이다.
- LAND request도 같은 원칙을 사용한다. ACK 지원 방식의 명시적 거부는 실패이고, fresh actual LAND가 확인되어야 LAND mode 전환 성공이다. ACK가 없어도 timeout 전 actual LAND가 확인되면 mode 전환은 성공이다.
- actual LAND 확인과 실제 착륙 완료/disarm은 별개 사건이다. LAND mode만 확인된 상태에서는 `MissionResult::LandConfirmed` 또는 mission 완료를 기록하지 않고 `RequestingExit`에서 별도의 착륙 완료 조건을 기다린다.
- 외부 actual mode 변경은 모든 활성 상태에서 `ControlLocked`로 전이하며 RC_CHANNELS가 없어도 적용한다.
- `ControlLocked`에서는 YOLO 재검출, SafetyMonitor LAND, mode 복귀 또는 통신 복구가 발생해도 0 velocity를 포함한 명령을 보내지 않는다.
- `ControlLocked`는 disarm과 명시적 지상 reset 전까지 자동 해제하지 않는다. reset 후 같은 session을 자동 재개하지 않고 `Finished`로 종료한다.
- 내부 실패는 `ControlLocked`로 위장하지 않고 사전 정의된 AUTO/LAND 종료정책을 거친다.

`Finished(MissionResult::Failed)`는 안전 동작 자체가 아니라 Jetson이 더 이상 신뢰할 수 있는 종료 명령을 수행하지 못한 관찰 결과다. 안전성은 실제 mode, `GUID_TIMEOUT` 및 ArduPilot 자체 failsafe 설정을 SITL과 실기체에서 별도로 검증해야 한다.

### 5.4 AUTO mission 재개

#### 현재 코드

AUTO 요청, mission index 수신, 재개 처리가 전혀 없다.

#### 공식 동작

ArduPilot 문서상 AUTO 재진입은 기본적으로 마지막 mission command에서 재개하고, `MIS_RESTART=1`이면 처음부터 재시작한다. 일반 resume 경로는 마지막 active waypoint로 직접 이동한다. `DO_SET_RESUME_REPEAT_DIST`를 사용하면 이전 mission 경로로 rewind할 수 있고 이 기능은 `MIS_RESTART=0`이 필요하다.

- [ArduPilot AUTO Mode](https://ardupilot.org/copter/docs/auto-mode.html)
- [Mission Rewind on Resume](https://ardupilot.org/copter/docs/common-mission-rewind.html)

현재 `mav.parm`에는 `MIS_RESTART=0`이 있지만 실제 FC 설정이라는 보장은 없다. 따라서 항상 특정 waypoint부터 재개한다고 단정하면 안 된다.

#### SITL 검증 항목

- GUIDED 전환 직전 `MISSION_CURRENT`
- GUIDED 중 mission index 변화 여부
- AUTO request 방식과 optional ACK evidence; 명시적 거부 여부
- fresh HEARTBEAT actual AUTO 확인 시각과 mode-transition timeout 결과
- actual HEARTBEAT mode=AUTO 시각
- AUTO 복귀 직후 `MISSION_CURRENT`
- 실제로 향하는 waypoint
- `MIS_RESTART=0/1` 비교
- `DO_SET_RESUME_REPEAT_DIST` 없음/있음 비교
- DO_JUMP, LAND, spline 등 mission item별 차이
- mission이 GCS에서 수정됐을 때 history reset 여부

---

## 6. 제어권 및 수동 개입 정책

### 6.1 우선순위

**[설계 제안]**

```text
ControlLocked
> Jetson SafetyMonitor
> Mission
> Guidance
```

ArduPilot은 자세 안정화, position controller와 motor 출력을 담당한다. Jetson은 high-level GUIDED velocity/position setpoint만 생성한다.

ArduPilot failsafe는 Jetson 내부 arbitration 항목이 아니다. FC에서 Jetson과 독립적으로 항상 활성화되어야 하는 보호 계층이다. FC failsafe가 mode를 변경한 경우에도 Jetson 관점에서는 자신이 요청하지 않은 actual mode 변경이므로 `ControlLocked`를 latch하고 `ControlLockReason::ArduPilotFailsafeModeChange`를 기록한 뒤 추가 명령을 중단한다. Jetson이 이를 원상 복구하려고 mode를 다시 요청해서는 안 된다.

### 6.2 mode 변경의 출처 판별

프로그램 시작 시 처음 관찰한 mode는 외부 mode 변경으로 간주하지 않고 초기 상태로 기록한다. external mode change 감시는 다음 조건을 모두 만족한 후 활성화한다.

- fresh HEARTBEAT를 최소 한 번 이상 수신함
- 초기 actual mode를 확정함
- automation session이 명시적으로 시작됨
- 직전 actual mode를 저장함

따라서 프로그램 시작 당시 LOITER, STABILIZE 또는 다른 mode였다는 이유만으로 `ControlLocked`를 latch하지 않는다. automation session 활성화 이후 actual mode가 변경됐을 때만 Jetson의 outstanding mode request와 대조해 정상 변경 또는 외부 변경으로 분류한다.

다음 evidence를 같은 monotonic clock 기준으로 비교한다.

1. 최근 Jetson mode request와 Jetson 내부 request ID
2. 요청 송신 시각과 유효시간
3. request 방식의 COMMAND_ACK 지원 여부와, 지원되는 경우 해당 ACK 및 수신 시각
4. HEARTBEAT actual mode와 수신 시각
5. RC 자동제어 허용 스위치 상태와 수신 시각

모드 변경은 다음처럼 분류한다.

- 유효한 Jetson request가 하나 outstanding이고 fresh actual mode가 요청 mode가 됐으며, ACK 지원 방식이라면 명시적 거부가 없음 → Jetson 요청에 따른 정상 변경. ACK 비지원 또는 미수신이어도 timeout 전 actual mode가 확인되면 정상 변경
- 검증된 RC automation allow 채널에서 OFF가 fresh하게 확인됨 → `ControlLocked`, reason은 `RcTakeoverEvidence`
- actual mode가 바뀌었지만 유효한 Jetson mode request와 대응하지 않음 → `RC_CHANNELS` 유무와 관계없이 `ControlLocked`; 판별 가능한 원인을 `ControlLockReason`에 기록
- ACK 지원 방식에서 명시적 거부 수신 → 즉시 전환 실패
- ACK 허용이 있어도 해당 mode의 fresh HEARTBEAT가 없음 → 전환 미완료
- ACK 비지원 또는 미수신이고 fresh actual mode도 아직 다름 → timeout까지 대기; ACK 부재만으로 성공·실패 판단 금지
- ACK에는 Jetson 내부 `ModeRequestId`가 되돌아온다고 가정하지 않으며 mode request를 둘 이상 동시에 outstanding으로 두지 않음
- 이전 mode request의 늦은 ACK가 현재 request의 command 종류, target과 시간창에 대응하지 않으면 폐기하고 별도 로그로 기록

`ControlLocked`는 원인 분류가 아니라 Jetson 명령 차단 상태다. lockout event에는 변경 전·후 actual mode와 함께 최소한 다음 `ControlLockReason`을 구분해 기록한다.

- `RcTakeoverEvidence`
- `UnexpectedOperatorOrGcsModeChange`
- `ArduPilotFailsafeModeChange`
- `UnknownExternalModeChange`

원인을 판별할 수 없어도 명령 차단은 동일하게 수행한다. 조종자, GCS 또는 ArduPilot failsafe 중 어느 주체가 mode를 바꿨는지 확인되지 않았다면 조종자 개입으로 단정하지 않고 `UnknownExternalModeChange`로 기록한다.

공식 문서도 MAVLink 전달은 보장되지 않으므로 차량 상태를 확인해야 하며, 실제 mode는 HEARTBEAT `custom_mode`에서 확인하도록 설명한다. 사용할 방식은 현재 코드의 `SET_MODE` 유지 또는 `COMMAND_LONG/MAV_CMD_DO_SET_MODE`로 아직 결정되지 않았다. 어느 방식을 사용해도 최종 성공 판정은 fresh HEARTBEAT actual mode다. ACK가 지원되는 방식의 상관관계는 내부 ID echo가 아니라 단일 outstanding 요청, command 종류, target과 제한된 시간창으로 관리한다.

- [MAVLink Basics](https://ardupilot.org/dev/docs/mavlink-basics.html)
- [Get and Set FlightMode](https://ardupilot.org/dev/docs/mavlink-get-set-flightmode.html)

### 6.3 latch 이후

`ControlLocked` latch는 원인과 관계없이 다음을 차단한다.

- mode 변경
- velocity/position setpoint
- 0 velocity setpoint
- arm/disarm
- takeoff/LAND
- `RC_CHANNELS_OVERRIDE`
- `MANUAL_CONTROL`
- 기타 vehicle-affecting command

허용 작업은 다음뿐이다.

- HEARTBEAT, RC_CHANNELS, mode, 위치, battery, GPS, EKF 수신
- 로그 기록

다음 이벤트로 자동 해제하지 않는다.

- stick 중앙 복귀
- RC 허용 스위치 ON 복귀
- mode가 AUTO/GUIDED로 복귀
- YOLO 재검출
- MAVLink 복구

해제 조건:

```text
disarmed
AND (검증·설정된 RC automation allow 채널이 있으면 switch ON)
AND (프로그램 재시작 OR 명시적인 지상 reset)
```

reset 요청 자체도 로그에 남기며 airborne 상태에서는 거부한다.

### 6.4 RC 설정

**[미확정]**

물리 RC 수동조종이 현장에서 가능했다는 것과 Jetson이 MAVLink `RC_CHANNELS`를 수신할 수 있다는 것은 별개의 사실이다. 후자는 아직 **[미확정]**이다. 따라서 automation allow 채널은 takeover의 필수 전제나 external lockout의 조건으로 사용하지 않는다. 수신 경로가 검증된 뒤에만 `ControlLockReason::RcTakeoverEvidence`를 기록하는 추가 증거로 사용할 수 있다.

조종자의 운용 절차는 **stick을 움직이기 전에 RC mode switch를 조종자용 mode로 먼저 변경하는 것**으로 정의한다. 그 mode 변경은 HEARTBEAT에서 actual mode 변화로 관찰되며 Jetson은 즉시 external lockout을 걸어야 한다. 조종자용 mode 자체는 아직 미확정이다.

채널 번호, PWM 범위, RC option은 정하지 않는다. 다음만 설정·검증한다.

- 어느 물리 스위치와 채널을 쓸지
- 2단 또는 3단 스위치인지
- endpoint/min/max/trim 실제 PWM
- ON/OFF 구간과 dead band
- `RC_CHANNELS` 갱신 주기와 stale timeout
- 송신기/수신기 failsafe 시 해당 채널 값
- ArduPilot의 `RCx_OPTION`, `FLTMODE_CH`, mode switch와 충돌 여부
- 스위치 OFF부터 command gate 차단까지의 최대 latency
- 송신기 조작 → Pixhawk mode 변경 → HEARTBEAT 수신 → 첫 control tick latch → 마지막 vehicle command 차단까지의 end-to-end latency
- 전원 재인가·수신기 단절 시 기본 상태

---

## 7. 단계별 리팩터링 계획

| 단계 | 작업                                                                                                                               | 호환성                                                | 완료 조건                                                                                                         | rollback 기준                                                             |
| ---- | ---------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------- |
| 0    | `json_bool()` 공백 결함을 재현하는 parser 테스트와 현재 명령 sequence를 기록하는 `FakeTransport` characterization test를 먼저 추가 | 기존 실행 파일·설정 유지                              | 알려진 JSON 결함이 테스트로 재현되고 실제 연결 없이 송신 sequence 검증 가능                                       | 재현 불가능하거나 FakeTransport가 실제 API를 대표하지 못하면 중단         |
| 1    | `autopilot::CommandSender`와 최종 gate를 추가하고 모든 production 송신 경로를 연결                                                 | 초기에는 authority 허용 상태에서 기존 동작 보존       | production runtime 우회 송신 0건, allow/block FakeTransport 테스트 통과                                           | 직접 송신 경로가 남거나 기존 sequence가 불필요하게 변경됨                 |
| 2    | `safety/control_authority`에 최초 mode baseline, automation session과 HEARTBEAT external lockout 구현                              | RC_CHANNELS 없이 동작, 기존 YAML 유지                 | 시작 mode 오탐 없이 예상 밖 mode 변경의 첫 tick 이후 송신 0건                                                     | 정상 Jetson mode 변경 오탐 또는 차단 지연 기준 초과                       |
| 3    | MAVLink 수신과 timestamp가 있는 `VehicleState` 갱신을 `autopilot`에 집중하고 mode request 방식별 optional ACK evidence를 보존      | 진단 도구는 read-only endpoint 또는 telemetry 구독    | production 연결 소유자 하나, telemetry freshness, ACK 지원 여부와 late ACK routing 검증                           | packet 손실 또는 GCS 연결 퇴행                                            |
| 4    | YOLO adapter, confirmation, geometry와 freshness를 `target` 책임으로 정리                                                          | 현재 Python/C++ 프로세스와 HTTP/UDP adapter 유지 가능 | confirmed/freshness 출력과 기존 좌표 결과 보존                                                                    | 검출 결과 또는 좌표 부호가 의도치 않게 변경됨                             |
| 5    | 7개 `MissionState`를 기존 흐름과 함께 shadow 실행                                                                                  | 기존 명령 송신 경로 유지                              | 실제 송신 없이 기존 흐름과 새 상태 전이를 로그로 비교 가능                                                        | 설명할 수 없는 상태 불일치                                                |
| 6    | `mission`이 guidance 순수 함수로 `CommandRequest`를 생성하고 실제 sender에 연결                                                    | 기존 CLI/YAML 보존                                    | ACK 필수 가정 없이 fresh actual GUIDED 후에만 tracking, explicit ACK rejection 처리, target loss 시 제한된 0 hold | request/ACK만으로 tracking 또는 stale target 이동 발생                    |
| 7    | `select_exit_action()`과 `RequestingExit`를 AUTO/LAND 공통 mode 전환 규칙에 연결                                                   | AUTO/LAND policy 설정 보존                            | fresh actual AUTO/LAND로만 mode 성공 판단하고 LAND mode와 착륙 완료를 분리                                        | 내부 실패가 `ControlLocked`로 잘못 분류되거나 LAND mode만으로 완료 처리됨 |
| 8    | RC_CHANNELS를 별도 검증하고 성공한 경우 optional `RcTakeoverEvidence` 추가                                                         | RC 미검증 환경에서는 HEARTBEAT lockout으로 운용       | fresh evidence 첫 tick 차단과 end-to-end latency 측정                                                             | RC 누락/stale이 authority 오판을 유발                                     |
| 9    | 카메라 보정 기반 `target_geometry` 도입                                                                                            | legacy 계산과 비교 로그 제공                          | 방향·거리 오차 기준 충족                                                                                          | 고정 배율보다 정확하다는 근거 부족                                        |
| 10   | 후보 A/B를 각각 SITL에서 검증                                                                                                      | 구조 변경과 알고리즘 선택 분리                        | 검증 보고서와 선택 근거 승인                                                                                      | safety limit 위반 또는 재현성 부족                                        |
| 11   | 전체 SITL와 bench 검증 후 기존 경로 제거                                                                                           | 기존 파일은 새 경로 검증 전 삭제 금지                 | gate 우회 0건, lock 이후 명령 0건, 종료정책 확인                                                                  | packet audit 또는 상태 로그 불일치                                        |

각 단계마다 별도 commit을 만들고 빌드·단위 테스트·SITL 로그를 보존한다. 구조 변경 단계에서 gain, timeout, 접근 알고리즘을 함께 변경하지 않는다.

`vehicle-affecting sender 하나` 제약은 production mission runtime에 적용한다. `test_arm` 같은 정비 도구는 별도 executable로 유지할 수 있지만 production runtime 및 GCS command sender와 동시 실행해서는 안 된다. 정비 도구에는 최소한 disarmed 확인, 명시적 bench mode, 프로펠러 제거 확인 절차, 사용자 확인과 단독 connection 같은 보호조건을 둔다. 이러한 조건은 실제 비행 중 정비 도구 실행을 허용한다는 뜻이 아니다.

---

## 8. 단위 테스트 계획

### Target

- 1 frame 검출만으로 confirmed가 되지 않음
- 안정적인 연속 검출 후에만 confirmed
- 다중 객체에서 선택 정책대로 동작
- `json.dumps()`의 공백 포함 JSON을 정상 parse
- stale observation 차단
- 이미지 중심 기준 x/y 부호 확인
- 픽셀 오프셋 부호에 맞는 body-frame 방향 생성
- camera mount/attitude 변환 부호 검증
- 보정 정보가 없으면 실제 meter 위치를 만들어내지 않음

### Mission/mode transition

- 단일 frame 검출로 GUIDED 요청 0건
- confirmed target 이후 GUIDED 요청 1건
- ACK 지원 방식에서 명시적 ACK 거부 시 즉시 전환 실패
- ACK 성공만 있고 actual mode가 바뀌지 않으면 성공 아님
- ACK가 없어도 timeout 전에 fresh actual GUIDED가 확인되면 성공
- ACK가 없고 actual mode도 바뀌지 않으면 timeout 실패
- fresh actual GUIDED 확인 전 tracking setpoint 0건
- 오래된 HEARTBEAT로 mode 전환 성공 판단 금지
- 이전 mode request의 늦은 ACK를 현재 request에 잘못 연결하지 않음
- 동시에 outstanding mode request가 둘 이상 생기지 않음
- Jetson의 정상 GUIDED 전환을 외부 mode 변경으로 오인하지 않음
- 예상하지 않은 mode 변경 시 lockout
- actual AUTO 확인 전 AUTO 복귀 성공 기록 금지
- actual LAND 확인 전 LAND mode 성공 기록 금지
- LAND mode 확인 후에도 실제 착륙 완료/disarm 전에는 `MissionResult::LandConfirmed` 또는 mission 완료 기록 금지

### Target loss

- target 유실 즉시 이동 velocity 중지
- authority가 허용된 `TargetLost`에서만 0 velocity 송신
- timeout 이전 같은 target 안정 재탐지 시 tracking 재개
- 다른 target이면 자동 재개하지 않음
- timeout 시 설정된 AUTO 또는 LAND 요청
- stale target으로 추적 재개하지 않음

### Guidance/safety

- 수평 속도 제한 적용
- 상승·하강률 제한 적용
- position/velocity setpoint expiry 적용
- stale altitude, attitude, GPS, EKF 차단
- safety output이 mission/guidance보다 우선
- Jetson이 LOITER를 요청하는 정상·safety 코드 경로 0건

### Control authority와 gate

- 요청하지 않은 actual mode 변경 확인 시 RC_CHANNELS 없이도 `ControlLocked`
- 검증된 RC allow OFF evidence 수신 시 `ControlLocked`와 `RcTakeoverEvidence` 기록
- fresh takeover evidence를 수신한 첫 tick부터 vehicle-affecting 송신 0건
- 0 velocity도 송신하지 않음
- AUTO/GUIDED/LOITER 요청 0건
- stick 중앙 복귀 후 latch 유지
- RC allow ON 복귀 후 latch 유지
- actual mode AUTO/GUIDED 복귀 후 latch 유지
- YOLO 재검출 후 latch 유지
- SafetyMonitor LAND 요청 차단
- telemetry와 logging은 지속
- disarm만으로 reset 불가
- disarm + switch ON이어도 명시적 reset/restart 없으면 해제 불가
- airborne reset 거부
- 명시적 지상 reset 조건 모두 충족 시에만 해제
- 송신 직전 authority가 바뀌면 이미 생성된 command도 차단
- 모든 `CommandDecision`에 허용/차단 이유 기록
- `ModeRequestId`를 ACK가 그대로 반환한다고 가정하지 않음
- 같은 시점의 outstanding mode request는 하나뿐임
- 조종자 mode switch부터 마지막 vehicle command 차단까지 end-to-end latency 측정
- 내부 mission 오류는 external lockout이 아니라 설정된 AUTO/LAND 종료정책으로 진행

---

## 9. SITL 테스트 계획

1. Mission Planner mission 업로드 후 AUTO 수행
2. AUTO 중 단일 target frame—AUTO 유지
3. confirmed target—GUIDED 요청
4. ACK 지원 방식에서 GUIDED 명시적 ACK 거부—즉시 실패
5. ACK 허용, actual mode 미변경—timeout 전까지 성공 아님
6. ACK 미수신, timeout 전 fresh actual GUIDED—성공 후 setpoint 시작
7. ACK 미수신, actual mode 미변경—mode-transition timeout 실패
8. 오래된 GUIDED HEARTBEAT—전환 성공 판단과 setpoint 금지
9. fresh actual GUIDED 확인 전 tracking setpoint 0건
10. 이전 mode request의 늦은 ACK가 현재 request에 연결되지 않음
11. 동시에 outstanding mode request가 둘 이상 생성되지 않음
12. tracking 중 짧은 target 유실—0 velocity hold
13. 동일 target 재탐지—tracking 재개
14. 다른 target 검출—자동 재개 금지
15. target-loss timeout 후 AUTO request; fresh actual AUTO 전 성공 기록 금지
16. ACK가 없어도 timeout 전 fresh actual AUTO면 AUTO 복귀 성공
17. ACK가 없고 actual AUTO도 없으면 timeout 실패
18. target-loss timeout 후 LAND request; fresh actual LAND 전 mode 성공 기록 금지
19. ACK가 없어도 timeout 전 fresh actual LAND면 LAND mode 전환 성공
20. actual LAND 확인 후 실제 착륙 완료/disarm 전 `LandConfirmed`와 mission 완료 기록 금지
21. LAND mode 진입 후 별도로 정한 착륙 완료 telemetry·disarm 조건 검증
22. AUTO 복귀 후 `MISSION_CURRENT`와 실제 waypoint 확인
23. `MIS_RESTART=0/1` 비교
24. mission rewind command 유무 비교
25. `SET_MODE`와 `COMMAND_LONG/MAV_CMD_DO_SET_MODE` 각각의 ACK 응답 및 HEARTBEAT 전환 비교
26. GUIDED 중 조종자가 stick 조작 전에 RC mode switch를 조종자용 mode로 변경
27. takeover 이후 MAVLink packet capture에서 vehicle command 0건 확인
28. takeover 이후 SafetyMonitor LAND 발생—packet 0건 확인
29. 예상하지 않은 외부 mode 변경—lockout
30. heartbeat 단절과 복구 후 latch 유지
31. GPS fix 저하와 EKF 이상
32. battery/고도 safety 조건
33. ArduPilot failsafe가 독립적으로 mode를 변경했을 때 Jetson external lockout과 명령 중단 확인
34. GCS와 Jetson 동시 연결에서 optional ACK/telemetry routing 확인
35. 정상 종료·비정상 종료·disarm 및 disarmed 지상 reset
36. 로그에서 request 방식·ACK 지원 여부·ACK evidence→fresh actual mode→command decision을 한 timeline으로 재구성할 수 있는지 확인

ArduPilot GCS failsafe 동작은 파라미터 의존적이며 통신 회복 시 이전 mode로 자동 복귀한다고 가정하면 안 된다. 실제 firmware와 설정을 고정한 SITL matrix가 필요하다.

- [ArduPilot GCS Failsafe](https://ardupilot.org/copter/docs/gcs-failsafe.html)

---

## 10. 미확정 사항

1. 최종 접근 방식: 수평 정렬 후 수직 하강 또는 사선 접근
2. 목표 YOLO class
3. 다중 객체 선택 정책
4. 동일 목표 재탐지 판정 방법
5. camera intrinsic calibration
6. 실제 horizontal/vertical FOV
7. 카메라 장착 위치와 각도
8. 이미지 해상도와 YOLO 좌표계 관계
9. lens distortion 보정 여부
10. 픽셀에서 지상 오프셋으로 변환하는 모델
11. 사용할 altitude/range source
12. RC takeover 채널
13. ON/OFF PWM 범위와 dead band
14. RC channel freshness
15. 관련 `RCx_OPTION`과 flight-mode 설정
16. AUTO 복귀 또는 LAND 기본 정책
17. target 확인 frame 수와 시간
18. target-loss timeout
19. 접근 속도·가속도·하강률
20. 정지 거리와 목표 도달 판정
21. GPS/EKF/battery/고도 threshold
22. soft limit과 hard limit별 action
23. AUTO 복귀 후 실제 mission index
24. mission rewind 사용 여부
25. 실제 ArduCopter firmware version
26. 실제 Pixhawk에 적용된 parameter
27. 조종자 mode switch가 선택할 실제 mode
28. GCS/MAVLink router endpoint 구성
29. Jetson 재시작 후 latch 상태를 디스크에 보존할지 여부
30. GUIDED 완료 후 다음 mission 상태
31. 접근 A/B에서 장애물·지면 안전을 판단할 sensor
32. 현재 `mav.parm`의 `BATT_MONITOR=0`, `FS_GCS_ENABLE=0`, `FENCE_ENABLE=0`이 실제 기체에도 적용됐는지 여부
33. AUTO/LAND 종료정책 명령이 실패한 상태에서 실제 기체 mode가 GUIDED로 남아 있을 경우의 최종 처리
34. GUIDED setpoint 중단 후 실제 ArduCopter `GUID_TIMEOUT` 동작과 해당 파라미터의 실기체 적용값
35. 기존 `ABORTED` 의미인 `Finished(Failed)` 진입 시 마지막 송신 setpoint, actual mode와 ArduPilot failsafe 상태를 어떻게 기록하고 운용자에게 알릴지
36. 현재 사용할 mode request 방식: `SET_MODE` 또는 `COMMAND_LONG/MAV_CMD_DO_SET_MODE`
37. 실제 ArduCopter firmware에서 각 mode request 방식의 COMMAND_ACK 지원 및 응답 여부
38. LAND mode 진입 후 실제 착륙 완료를 판단할 telemetry와 조건
39. LAND mode 확인부터 실제 착륙 완료/disarm까지의 timeout과 실패 정책

### Target-loss 정책 비교

| 정책      | 사용하기 적합한 조건                                          | 장점                       | 위험                                                          |
| --------- | ------------------------------------------------------------- | -------------------------- | ------------------------------------------------------------- |
| AUTO 복귀 | mission 재개 동작과 경로가 SITL에서 검증됐고 telemetry가 정상 | 기존 임무 계속 가능        | 예상치 못한 waypoint로 이동, mission restart/rewind 설정 차이 |
| LAND      | AUTO 경로 복귀보다 현재 위치 착륙이 더 안전하다고 검증됨      | mission 진행을 확실히 중단 | 착륙 지면·장애물·사람·목표물 위 착륙 위험                     |

어느 정책도 현재 기본값으로 선택하지 않는다.

---

## 11. 다음 구현 단계에서 가장 먼저 해야 할 한 가지

**현재 다음 구현 단계의 첫 작업은 모든 production mission vehicle-affecting 송신이 통과하는 `autopilot::CommandSender` 경계를 추가하는 것**이다. JSON parser 회귀 테스트, `FakeTransport`, MAVLink 수신 보존·target discovery와 기존 command serialization characterization은 완료됐다. 이 테스트를 기준으로 명령 필드와 순서를 유지하면서 sender를 한곳으로 모은 뒤, 최종 gate와 HEARTBEAT 기반 `ControlLocked` 전이를 순서대로 배치한다.

AUTO 상태머신이나 접근 알고리즘부터 추가하면 현재처럼 여러 경로가 직접 송신할 수 있어 takeover latch를 구현해도 우회 가능하다. 우선 다음이 자동으로 증명되어야 한다.

```text
production mission runtime의 vehicle-affecting MAVLink sender 수 = 1
ControlLocked 이후 송신 수 = 0
차단된 LAND와 0 velocity도 로그에서 확인 가능
telemetry 수신과 로그는 계속됨
```

이 기반이 고정된 다음에 mode request 방식을 선택하고, optional ACK evidence와 fresh HEARTBEAT actual mode를 분리해 검증하는 AUTO→GUIDED 상태머신을 추가하는 순서가 안전하다. 구현 전 FakeTransport에는 ACK 지원·거부·미수신·late ACK와 stale/fresh HEARTBEAT 조합을 모두 표현할 수 있어야 한다.

### 11.1 2단계 1차 구현 기록

**[현재 확인]**

- `BUILD_TESTS=ON`일 때 CTest가 `target_json`, `mav_connection`, `command_characterization` 테스트를 빌드하며 세 테스트가 모두 통과한다.
- `control/target_json.*`는 `/target` 최상위 boolean/number의 JSON 공백, null, 음수와 소수를 처리한다. string decoding, 임의 schema 검증과 범용 JSON 처리는 지원하지 않는다.
- `control/tests/FakeTransport`는 incoming bytes를 메모리 queue에서 반환하고 outgoing bytes를 메모리에만 기록한다. socket, serial, sleep과 외부 endpoint를 사용하지 않는다.
- `MavConnection`은 한 read chunk의 모든 byte를 parsing하고 완성된 message를 pending queue에 보존한다. 여러 packet이 한 chunk에 합쳐진 경우와 한 packet이 여러 chunk로 분할된 경우를 모두 테스트했다. filter가 찾는 message보다 먼저 온 불일치 message는 기존 의미대로 소비되며, 모든 telemetry의 timestamp 보존은 아직 구현되지 않았다.
- command target은 검증된 최초 ArduPilot flight-controller HEARTBEAT로만 설정한다. 다른 system/component message와 이후 다른 autopilot HEARTBEAT는 target을 자동 변경하지 않으며, target 확정 전 `send()`는 차단된다.
- SET_MODE, ARM/DISARM, TAKEOFF, LOCAL/BODY velocity, 0 velocity, LAND와 message interval의 현재 MAVLink serialization 및 명시적 테스트 호출 순서를 FakeTransport로 고정했다. 이는 command 생성 API 검사이며 `control.cpp` 전체 mission의 end-to-end 실행 검증은 아니다.
- global `drone` API의 module-level `g_master`, `control.cpp`의 sleep, 설정·target/telemetry loop 결합은 남아 있다. `CommandSender`, timestamp가 있는 `VehicleState`, `ControlLocked`와 AUTO→GUIDED 상태머신은 아직 설계 제안이며 구현되지 않았다.

## 향후 GCS 및 시뮬레이션 확장

[설계 제안]

향후 Gazebo/ArduPilot SITL과 GCS를 추가하되, 현재 onboard mission의
책임과 섞지 않는다.

저장소는 초기에는 하나의 monorepo로 유지하고 다음 논리 영역을 둔다.

- onboard: Jetson에서 실행되는 target/mission/safety/autopilot/logging
- sim: Gazebo world/model, ArduPilot SITL 설정, 실행 스크립트와 시나리오
- gcs: telemetry·영상 표시, 운용자 입력과 로그
- docs: 공통 architecture와 onboard-GCS protocol 문서

현재 control/, YOLO_MODEL/, setting/ 경로를 이번 단계에서 즉시
이동하지 않는다. 새 구조는 기존 빌드와 실행 경로를 보존하면서
단계적으로 도입한다.

경계 원칙:

- Gazebo는 카메라·기체·환경 입력을 제공하지만 mission 판단을 수행하지 않는다.
- ArduPilot SITL은 실제 Pixhawk와 같은 autopilot 역할을 한다.
- onboard는 실제 기체와 SITL에서 동일한 mission/guidance 코드를 사용한다.
- 실기체와 SITL의 차이는 transport, target source와 config로 제한한다.
- 시뮬레이션 전용 조건문을 mission/guidance에 퍼뜨리지 않는다.
- GCS는 telemetry 표시, 영상 표시, 로그와 운용자 요청을 담당한다.
- GCS는 YOLO, target guidance와 mission-critical 판단을 수행하지 않는다.
- GCS 연결이 끊겨도 onboard safety와 ArduPilot failsafe가 유지되어야 한다.
- GCS에서 오는 요청은 onboard의 safety와 ControlLocked보다 우선할 수 없다.
- GCS와 onboard 사이의 메시지는 version이 있는 명시적 protocol로 정의한다.
- GCS와 진단 도구는 production CommandSender를 우회해 Pixhawk 비행
  명령을 보내지 않는다.

상세 GCS UI, telemetry schema, 영상 protocol, Gazebo 모델과 world 구성은
각 기능을 실제로 시작할 때 별도 설계 문서로 작성한다.
