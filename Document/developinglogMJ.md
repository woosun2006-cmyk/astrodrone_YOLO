# Developing Log (MJ)

## 2026-08-08

### 1. control_sim.cpp - CMakeLists 등록 해지
- 08-06 로그에서 확인 필요로 남겨뒀던 항목: 디스크에 파일이 없는데
  `CMakeLists.txt`엔 여전히 등록돼 있어 `make all`이 실패하던 문제.
- `add_executable(control_sim control_sim.cpp)` /
  `target_link_libraries(control_sim ...)` 두 줄 제거.

### 2. target_distance.cpp - 픽셀->실거리 변환 수정 필요 (고도 미반영)
- 현재 `ground_offset_m = hypot(x_px, y_px) * pixel_to_meter`로, 고도와
  무관하게 고정 배율만 곱해서 계산 중.
- 실제로는 고도에 따라 카메라가 담는 지상 범위가 달라지므로, 같은 픽셀
  오프셋이라도 고도가 높을수록 실거리는 커져야 함 - 지금 방식은 틀린 값을
  낼 수 있음. 수정 필요.
- 이 계산(고도 반영 픽셀->실거리 변환)은 `pos_calculator.cpp`가 담당해야
  함 (현재 빈 파일). `control/README.md`에도 동일 내용 기록.
- (같은 날 재확인) `pos_calculator.cpp`는 여전히 빈 파일. 실제 거리 계산은
  `target_distance.cpp:236-238`의 고정 `pixel_to_meter` 곱셈이 유일한
  구현 - **미해결**.

### 3. 소프트웨어 고도 제한 (완료)
- `setting/safety.yaml` 신설: `altitude_limit`
  (`soft_limit_m: 4.0`, `hard_limit_m: 5.0`, `poll_rate_hz`,
  `descent_speed_mps`, `recovery_margin_m`). "4~5m 제한" 요청을
  soft/hard 두 단계 천장(ceiling fence)으로 해석함 - 바닥(하한)은 없음.
- `check_alt.cpp`: 기존 고도-홀드 모니터에 soft/hard 판정과 전이 로그,
  요약에 breach 샘플 카운트 추가. read-only 유지, 명령 전송 없음.
- `emergency.cpp` (신규): `hard_limit_m` 초과 시 GUIDED로 전환 후 비례
  하강 속도(`kGain`, `kMinDescent`~`descent_speed_mps` 클램프) 전송,
  `hard_limit_m - recovery_margin_m` 아래로 내려올 때까지 유지(경계
  채터링 방지).
- `yaml_settings.hpp/.cpp`에 `load_safety_settings()` 추가
  (`load_mavlink_settings()`와 동일한 실행파일-상대 경로 탐색),
  `drone_lib.hpp/.cpp`에 위임 함수 추가. `CMakeLists.txt`에 `emergency`
  타겟 등록. 전체 빌드 확인.

### 4. cycle_ms를 두 개로 분리: sense_cycle_ms / control_cycle_ms (완료)
- 기존엔 `target_track.cycle_ms`(50ms/20Hz) 하나를 `target_distance.cpp`
  (YOLO 폴링 + ALTITUDE 요청 주기)와 `control.cpp`(Pixhawk 전송 주기)가
  같이 읽어써서, YOLO 실측 처리량(~10fps)과 무관하게 감지 쪽도 20Hz로
  억지로 돌던 문제.
- `setting/MAVLink.yaml`: `sense_cycle_ms: 100`(target_distance.cpp),
  `control_cycle_ms: 50`(control.cpp)로 분리. control.cpp의 20Hz 송신은
  ArduPilot GUIDED가 꾸준한 setpoint 스트림을 원하기 때문에 그대로 유지.
- `target_distance.cpp`: `Args::cycle_ms` -> `sense_cycle_ms`로 개명,
  `--cycle-ms` 플래그도 `--sense-cycle-ms`로 변경. `control.cpp`는
  `control_cycle_ms` 읽도록 수정. 빌드 확인.

### 5. YOLO 타겟 연속-프레임 안정성 체크 (완료)
- 기존 `target_stale_ms`(400ms)는 "최근성"만 보고 "연속 N프레임 동일
  타겟" 확인이 없던 문제.
- `YOLO_MODEL/yolo_live.py`: `TARGET_CONFIRM_FRAMES = 5` (placeholder),
  `state["target_streak"]`(deque)로 `inferer()`에서 매 프레임 클래스명을
  누적, 연속 5프레임이 같은 클래스로 잡혀야 `/target` 응답에
  `"confirmed": true`. 검출 없음/클래스 변경 시 스트릭 리셋.
- `target_distance.cpp`: `found`뿐 아니라 `confirmed`도 요구하도록 수정
  (`json_bool(body, "confirmed")` 추가). 빌드 확인.

### 6. safety.yaml 안전 기준 확장 + emergency.cpp LOITER 인계 (부분 완료)
완료:
- `safety.yaml`에 `battery_limit`(min_percent 20%), `heartbeat_limit`
  (max_gap_sec 3초 - 최초 접속용 `heartbeat_timeout`과 별개로 비행 중
  링크 끊김 감지), `gps_limit`(min_fix_type 3, min_satellites 6),
  `vehicle_health_limit`(require_prearm_healthy, require_normal_state)
  추가. 전부 미검증 placeholder 값.
- `emergency.cpp`: 위 4개 중 하나라도 위반되면 `drone::set_mode("LOITER")`
  로 전환. GUIDED가 필요한 altitude 보정보다 우선순위를 높이고, health
  위반 중엔 altitude 보정을 억제해서 두 로직이 모드를 서로 뺏지 않게 함.

미완료 / 확인 필요:
- **EKF 상태확인 미구현.** `control/third_party/mavlink`에 벤더링된
  MAVLink 서브셋엔 `EKF_STATUS_REPORT` 메시지 디코더 자체가 없음(확인
  완료, 존재하지 않음). 지금은 `HEARTBEAT.system_status`가
  `MAV_STATE_CRITICAL/EMERGENCY`인지로 대체 중 - ArduPilot이 EKF
  failsafe 시 여기 반영하긴 하지만 EKF variance 직접 체크는 아님.
  정확히 하려면 mavlink 헤더 재생성 필요.
- "test_arm" 안전장치 해석: `SYS_STATUS.onboard_control_sensors_health`의
  `MAV_SYS_STATUS_PREARM_CHECK` 비트로 임의 해석해서 구현함 - 의도와
  맞는지 확인 필요.
- battery/heartbeat/gps 임계값 전부 실측 없이 넣은 placeholder.
- **"LOITER 전환 -> 타겟 재탐색 -> 접근 재개" 루프 미구현.** 대화로
  요구사항 확정: LOITER는 최소 3초 유지 -> 이후 건강상태 정상 +
  YOLO `confirmed` 재확인되면 다시 타겟 방향으로 이동. `control.cpp`/
  `target_link.cpp`에 있는 접근(추적) 로직을 `emergency.cpp`에서도 쓸 수
  있게 공용 함수로 분리하는 리팩터링이 선행되어야 함 - 아직 착수 전.
  (관련해서, 이 루프가 도는 동안 `control.cpp`를 별도 프로세스로 계속
  띄워둘지, `emergency.cpp` 자체가 두 역할을 다 흡수할지도 미정.)

### 7. 추가 안전장치 아이디어만 제시, 구현은 보류 (미구현)
- **control.cpp SIGINT(Ctrl+C) 핸들러**: 현재 `FinallyGuard`는 정상 함수
  종료 경로만 커버(속도 0). 시그널 수신 시 `land() -> 착지(armed=false)
  확인 폴링 -> 안되면 disarm` 순서로 안전 종료하는 핸들러 없음. 시그널
  핸들러 안에서 직접 MAVLink 전송은 안전하지 않으므로, 핸들러는
  `atomic<bool>` 플래그만 세우고 메인 루프가 처리하는 방식 제안.
- **거리값 이상치(sanity) 체크**: `pixel_to_meter`가 미검증 상수라
  오탐지로 픽셀 오프셋이 튀면 `distance_m`도 같이 튀어 그대로 속도
  명령에 들어감. 프레임간 최대 변화량(slew limit) + 절대 상한 clamp,
  벗어나면 스무딩 대신 그냥 `valid=0` 처리하는 방식 제안.
- **미션 타임아웃 에스컬레이션 분리**: 지금 `approach_duration_sec`
  초과는 `land()` 한 번뿐, 실패 시 재시도/RTL/최종 disarm 같은 단계적
  대응이 없음. SIGINT 핸들러와 같은 `land_and_confirm()`류 공용 헬퍼로
  묶어서 재사용하는 방식 제안.
- **안전 이벤트 전용 로그**: 지금은 `mav.tlog`(바이너리)뿐이라 어떤
  긴급 트리거가 언제 발동했는지 사후분석이 어려움. `check_alt.cpp`/
  `emergency.cpp`에 이미 있는 상태전이 지점마다 JSONL 한 줄씩 남기는
  방식 제안.

### 8. control.cpp / emergency.cpp 역할 분리 - "단일 명령 프로세스" 구조로 재설계 (완료)
문제: 6번에서 만든 `emergency.cpp`가 LOITER 전환 등 MAVLink 명령을 직접
보내는 구조였는데, 실비행에서는 `control.cpp`도 계속 떠서 자기 명령을
보내고 있을 것이므로 두 프로세스가 동시에 같은 기체에 명령을 보내
충돌할 수 있음. 요청에 따라 "명령은 control.cpp 하나만" 구조로 재설계:

- **`control/emergency_link.hpp` / `.cpp`** (신규): `target_link.hpp`와
  동일한 패턴의 루프백 UDP 구조체(`EmergencyMsg{seq, active, reason}`).
  `emergency.cpp -> control.cpp` 단방향, 매 사이클 상태를 그대로
  재전송(엣지 트리거 아님 - 패킷 유실/재시작에도 한 사이클 뒤엔 수렴).
  `drone_lib`에 소스 등록.
- **`emergency.cpp` 전면 수정**: `drone::connect()`/`set_mode()`/
  `send_velocity()` 등 명령 관련 코드 전부 제거, `open_connection()`으로
  읽기 전용 연결만 사용(check_alt.cpp와 동일 방식). altitude 관련 로직도
  전부 제거(→ control.cpp로 이동). battery/heartbeat/gps/vehicle_health
  체크는 유지하되, 위반 시 `EmergencySender`로 `{active, reason}`을 매
  틱마다 브로드캐스트만 함 - 기체에 아무 명령도 보내지 않음.
- **`control.cpp` 전면 수정**: (1) `altitude_limit`(soft/hard/천장) 판정 +
  GUIDED 하강 보정 로직을 `emergency.cpp`에서 옮겨와 `approach_target()`
  루프에 통합 - 이미 `target_distance.cpp`가 매 사이클 UDP로 보내주는
  `TargetRangeMsg.altitude_m`을 재사용해서 별도 MAVLink ALTITUDE 구독
  없이 처리. (2) `EmergencyReceiver`로 emergency.cpp 신호를 매 사이클
  폴링, `active=1`이면 `LOITER`로 전환하고 **최소 3초 유지**
  (`kMinLoiterHoldSec`, 채터링 방지), 신호가 꺼지고 최소 유지시간이
  지나면 `GUIDED`로 복귀해서 원래 타겟 추적 로직 재개.
- **`setting/safety.yaml`**: `emergency_link.udp_port: 15030` 섹션 추가
  (target_track과 다른 포트). `altitude_limit`/`vehicle_health_limit`
  등 주석에서 "emergency.cpp가 처리" -> "control.cpp가 처리, emergency.cpp
  는 감시만"으로 정정. `battery_limit`에 `min_voltage_v: 14.8` 추가
  (사용자가 알려준 4S 배터리 공칭전압 14.8V 기준 - 실측 후 조정 필요,
  자세한 근거는 야믈 주석 참고).
- `CMakeLists.txt`에 `emergency_link.cpp` 등록. 전체 빌드 확인
  (경고 없음), `control`/`emergency` 둘 다 `safety.yaml` 새 섹션을
  경고 없이 읽는 것까지 확인.
- **미구현으로 남은 부분**: LOITER 복귀 시 "YOLO가 타겟을 다시
  `confirmed`로 잡을 때까지 기다렸다가 재개"까지는 아직 안 넣음 - 지금은
  최소 3초 + 위험신호 해제만 조건이고, 복귀 직후 타겟이 없으면 기존
  `tracking=false` 경로(속도 0, 호버)로 자연스럽게 넘어가긴 하지만
  "확인된 재탐지"를 복귀 조건 자체에 넣는 건 아님.

## 2026-08-06

### 1. YOLO 좌표계 정리 (cam_sets.yaml)
- `setting/cam_sets.yaml`에 `coord_origin: center` 규약 추가.
- 화면 정가운데를 (0,0)으로, +x는 오른쪽, +y는 위쪽(드론 제어 기준, 이미지 기준 아래+ 아님).
- 해상도는 특정 값 하나로 고정하지 않고 범용 변환식만 주석으로 남김
  (`x = px - width/2`, `y = height/2 - py`) — camserver/fpslog(1280x720)와
  yolo_live.py(640x480)가 해상도가 서로 달라서.

### 2. check_alt.cpp - 실시간 고도 유지 체크 프로그램
- MAVLink `ALTITUDE.altitude_relative`(홈 기준 상대고도)를 스트리밍 받아서
  현재 고도, 목표와의 편차, HOLD/DRIFT 여부, min/max, 표준편차, 유지율(%)을
  실시간으로 출력.
- 기준 고도는 `--target` 미지정 시 첫 수신값을 자동 채택.
- `CMakeLists.txt`에 실행 타겟 등록, 빌드 확인 완료.

### 3. 타겟 접근(추적) 파이프라인 1차 구현
목표: YOLO가 타겟을 인식하면 화면 중심좌표와 비교한 픽셀 오프셋 + 고도를
피타고라스로 합쳐 거리를 구하고, 그 거리로 속도를 계산해서 타겟 쪽으로
비행하는 구조. 사이클을 세 단계로 명확히 분리:
**1) 거리계산 -> 2) 속도계산 -> 3) 픽스호크 전송**

- **YOLO_MODEL/yolo_live.py**: 최고 신뢰도 탐지의 박스 중심을 center-origin
  좌표로 변환해 `/target` HTTP 엔드포인트로 노출 (x_px, y_px, age_ms 등).
- **control/target_link.hpp / .cpp**: 두 프로그램을 잇는 로컬 UDP
  (`127.0.0.1`) 패킷 구조체(`TargetRangeMsg`)와 송/수신 클래스. 같은 장치,
  같은 컴파일러라 raw struct를 그대로 주고받음.
- **control/target_distance.cpp** (신규, 1단계=거리계산):
  Pixhawk에서 고도, yolo_live.py `/target`에서 픽셀 오프셋을 받아
  `ground_offset_m = hypot(x_px, y_px) * pixel_to_meter`,
  `distance_m = hypot(ground_offset_m, altitude_m)` 계산 후 UDP로 발행.
- **control/drone_lib.hpp / .cpp**: `send_velocity_body()` 추가
  (MAV_FRAME_BODY_OFFSET_NED). 기존 `send_velocity()`는 월드좌표계(NED)라
  기수가 북쪽을 볼 때만 "전진"이 맞아서, 카메라(기수) 기준 속도 명령이
  필요해 별도로 만듦.
- **control/control.cpp** (2/3단계=속도계산+전송): UDP로 최신 거리를 받아
  전진속도(거리 비례, 상한 有)·좌우속도(픽셀 오프셋 비례, 상한 有)를
  비례제어로 계산해서 `send_velocity_body()`로 송신. 타겟 놓치거나 UDP
  링크가 오래되면(`link_stale_ms`) 자동으로 속도 0(정지/호버).
- **setting/MAVLink.yaml**: `target_track` 섹션 신설
  (yolo_host/port, udp_port, pixel_to_meter, stale 타임아웃들,
  max_forward_speed, max_lateral_speed, stop_distance,
  approach_duration_sec, cycle_ms).
- `CMakeLists.txt`에 `target_distance` 실행 타겟 + `target_link.cpp`를
  `drone_lib`에 등록. 전체 빌드(-Wall -Wextra 경고 없음) 확인.

### 4. 사이클 주기 결정: 20Hz (cycle_ms: 50)
- 처음엔 YOLOv5n @ IMG_SIZE=320 젯슨 나노 실측 추론 속도(~10fps)에 맞춰
  100ms(10Hz)로 시작했다가, 요청에 따라 50ms(20Hz)로 변경.
- YOLO 추론 자체는 여전히 ~10fps라서 20Hz로 돌리면 절반의 사이클은 같은
  타겟값을 재전송하는 셈 — 그래도 GUIDED 속도 스트림은 계속 살아있는
  값을 원하므로 문제 없음. 더 빠르게 돌린다고 더 신선한 타겟 데이터가
  나오는 건 아니라는 점은 인지하고 있을 것.
- `target_distance.cpp` / `control.cpp` 모두 `cycle_ms`를 `MAVLink.yaml`에서
  런타임에 읽어가므로, 코드 수정 없이 설정값만 바꿔서 반영됨.

### 5. 속도 제한 조정
- 처음 기본값: `max_forward_speed 1.5 m/s`, `max_lateral_speed 1.0 m/s`.
- 요청에 따라 **둘 다 0.5 m/s로 하향** (`setting/MAVLink.yaml`).
  마찬가지로 코드 변경 없이 설정값만 수정.

### 6. control.cpp 변경 내역 정리 (질문 답변)
- 원래(이번 세션 이전) control.cpp: `connect → GUIDED → arm → takeoff →
  land` 뿐인 단순 데모, 이륙 직후 바로 착륙, 대기도 없었음.
- 지금: `approach_target()` 함수 추가 (파이프라인 2/3단계 — UDP로 받은
  거리값을 속도로 변환해 `send_velocity_body()`로 전송). 함수 종료 시
  무조건 속도 0을 보내는 `FinallyGuard` 포함.
- main() 흐름도 `이륙 → 10초 대기(고도 안정화) → approach_target() 호출
  (기본 60초) → land`로 변경.

### 7. GitHub push 충돌(rejected) 해결
- `Document` 브랜치에서 `git push`가 `[rejected] (fetch first)`로 실패.
- 원인: GitHub 쪽에 루트 `README.md`를 새로 추가하는 커밋(`2635dbe`,
  `Add initial content to README.md`)이 먼저 올라가 있었고, 로컬에서도
  별도로 2개 커밋(`README update 7-31`, `0806`)을 쌓아서 히스토리가
  갈라짐(diverged) — fast-forward가 안 되니 push가 그냥 거부된 것.
- 두 브랜치가 건드린 파일이 겹치지 않아(원격은 README.md만) 충돌 없이
  `git merge origin/Document`로 자동 병합 후 `git push origin Document`
  성공 (`2635dbe..9c04190`).
- 참고: 작업 트리에는 이 세션에서 만든 파일들 외에도 (camserver.py,
  fpslog.py, port.yaml, limit_speed.yaml 등) 세션과 무관해 보이는
  커밋 안 된 변경사항이 남아있음 — 이번엔 손대지 않음, 커밋은
  요청 시에만 진행.

### ⚠️ 실비행 전 확인할 것
- `pixel_to_meter`(기본 0.01)는 **실측 카메라 FOV/초점거리가 없어서 만든
  자리표시자 값**. 실제 타겟을 알려진 거리에 두고
  `target_distance.cpp` 출력(`dist=`)과 비교해서 캘리브레이션 필요.
- `max_forward_speed`, `max_lateral_speed`는 현재 0.5m/s로 낮춰놨지만
  실비행 조건 보며 재조정할 것. `stop_distance`(2.0m)도 마찬가지.
- 빌드 중 `control_sim.cpp` 파일이 디스크에서 사라져 있는 걸 발견함
  (`CMakeLists.txt`엔 여전히 등록돼 있어 `make all`이 거기서 실패).
  이번 작업에서 건드린 적 없음 — 확인 필요.
- (2026-08-08 추가) `setting/safety.yaml`의 `altitude_limit`,
  `battery_limit`, `heartbeat_limit`, `gps_limit`,
  `vehicle_health_limit`은 전부 실측 없이 넣은 placeholder 값. 실제
  배터리/운용 환경 기준으로 튜닝 필요.
- (2026-08-08 추가) `emergency.cpp`의 EKF 상태확인은 정식
  `EKF_STATUS_REPORT`가 아니라 `HEARTBEAT.system_status` 대체 지표
  (proxy)임. 정확한 EKF variance 체크가 필요하면
  `control/third_party/mavlink` 헤더를 `EKF_STATUS_REPORT` 포함해서
  재생성해야 함.
- (2026-08-08 추가) `emergency.cpp`의 "test_arm" 안전장치는
  `MAV_SYS_STATUS_PREARM_CHECK` 비트로 임의 해석해서 구현함 - 의도와
  맞는지 확인 필요.
- (2026-08-08 추가) 지금 픽스호크가 물리적으로 연결 안 돼 있어 이번
  세션의 `emergency.cpp` 변경분(LOITER 인계 포함)은 SITL/실기 연결 후
  검증 필요. "LOITER -> 재탐색 -> 접근 재개" 루프는 아직 미구현.
