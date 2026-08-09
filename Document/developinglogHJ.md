# Developing Log (HJ)

## 2026-08-09

### 1. Architecture.md 작성 및 기존 구조 분석
- 현재 `control` 프로그램은 Mission Planner의 AUTO mission을 감시하는 구조가
  아니라 GUIDED 요청 → ARM → takeoff → target approach → LAND 순서로
  실행됨을 확인했다.
- 현재 프로젝트 규모에 맞춰 `app`, `autopilot`, `target`, `mission`,
  `safety`, `logging`의 6개 논리 모듈 구조를 설계했다.
- `MonitoringAuto`, `RequestingGuided`, `TrackingTarget`, `TargetLost`,
  `RequestingExit`, `ControlLocked`, `Finished`의 7개 `MissionState`와
  수동 개입 이후 모든 비행 명령을 차단하는 `ControlLocked` 정책을 문서화했다.
- 위 6개 모듈 구조, 7개 상태머신, CommandSender, VehicleState,
  ControlLocked는 승인된 설계 제안이며 아직 구현 완료된 기능이 아니다.

### 2. target JSON parsing 결함 수정
- Python `json.dumps()`가 출력하는 JSON의 colon 뒤 공백 때문에 기존
  `json_bool()`이 `found`와 `confirmed`를 false로 해석하던 문제를 수정했다.
- boolean과 number parsing을 `control/target_json.cpp`와
  `control/target_json.hpp`로 최소 분리했다.
- colon 주변 whitespace, 줄바꿈, key 순서, malformed token, null, missing,
  음수·소수·지수 number와 실제 `/target` 형태를 테스트에 추가했다.
- 이 parser는 현재 `/target` 응답의 최상위 boolean/number 필드만 처리하는
  제한 parser이며 범용 JSON parser가 아니다.

### 3. FakeTransport 및 CTest 기반 추가
- 실제 socket, serial, Pixhawk 없이 MAVLink incoming/outgoing bytes를
  메모리 queue와 buffer에서 검증하는 테스트 전용 `FakeTransport`를 추가했다.
- `BUILD_TESTS` 옵션, `enable_testing()`과 `control/tests` CMake 구성을
  추가해 CTest로 JSON 및 MAVLink 테스트를 실행할 수 있게 했다.

### 4. MAVLink 수신 결함 수정
- 한 번의 `read_bytes()`에 여러 MAVLink packet이 들어오면 첫 번째 일치
  메시지를 반환한 뒤 같은 chunk의 나머지 packet이 유실되던 문제를
  characterization test로 재현하고 수정했다.
- read chunk의 모든 byte를 끝까지 parser에 전달하고 완성된 메시지를
  `MavConnection`의 pending queue에 보존하도록 변경했다.
- 하나의 packet이 여러 read chunk로 나뉘는 경우, 빈 read, garbage 뒤 정상
  packet과 incomplete packet 처리도 테스트했다.
- `recv_match()`가 찾는 메시지보다 먼저 도착한 filter 불일치 메시지는
  기존 동작처럼 소비될 수 있다. 모든 telemetry의 timestamp 포함 보존은
  향후 `VehicleState`에서 처리할 예정이다.

### 5. MAVLink command target 안정화
- 모든 수신 메시지의 `sysid`와 `compid`가 command target을 덮어쓰던
  문제를 수정했다.
- 검증된 최초 ArduPilot flight-controller HEARTBEAT로만 target을 설정하고,
  다른 system/component의 일반 telemetry와 이후 다른 autopilot
  HEARTBEAT가 target을 자동 변경하지 않도록 했다.
- target 확인 전에는 `MavConnection::send()`가 transport에 bytes를
  기록하지 않고 명시적으로 실패하도록 차단했다.

### 6. production 명령 characterization test
- SET_MODE, ARM/DISARM, TAKEOFF, LOCAL_NED/BODY_OFFSET_NED velocity,
  0 velocity, LAND와 `MAV_CMD_SET_MESSAGE_INTERVAL`의 msgid, source/target,
  command/custom mode, frame, type mask, velocity·altitude 필드와 호출 순서를
  FakeTransport로 고정했다.
- 이 테스트는 기존 명령 생성 API의 serialization 검사다. sleep, 설정,
  target/telemetry와 결합된 `control.cpp` 전체 미션을 실행하는 end-to-end
  테스트는 아니다.
- 비행 알고리즘, guidance 계산, gain, timeout, safety 정책과 명령 반복
  주기는 변경하지 않았다.

### 7. 테스트 결과
- `target_json`: 통과.
- `mav_connection`: 통과.
- `command_characterization`: 통과.
- 별도 clean build directory에서 전체 CTest 3/3 통과를 확인했다.
- 실제 장치, socket, serial, Pixhawk 및 외부 MAVLink 송신은 사용하지 않았다.

### 8. 남은 작업
- production mission runtime의 단일 송신 경계인 `CommandSender` 도입.
- timestamp와 freshness를 포함한 `VehicleState` 추가.
- 전역 `MAVLINK_COMM_0` parser 상태를 connection별로 분리.
- `ControlLocked`와 RC mode switch 기반 수동 개입 차단 구현.
- AUTO → GUIDED mission 상태머신 구현.
- SITL 검증 후 실제 Pixhawk와 실기체 설정·동작 검증.
