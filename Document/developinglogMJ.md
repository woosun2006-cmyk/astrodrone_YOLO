# Developing Log (MJ)

## 2026-08-11

### 1. control.cpp - AUTO 요격 모드(`--auto-intercept`) 추가 (완료)
목표: 픽스호크가 (조종기/GCS로 이미 arm되어) AUTO 모드로 자체 미션을 날고
있는 상태를 지켜보다가, YOLO가 타겟을 확정 인식하면 그때만 GUIDED로
가로채 접근하고, 놓치거나 시간이 지나면 다시 AUTO로 돌려주는 흐름.
기존 "control.cpp가 직접 arm/이륙까지 다 하는" 흐름과는 별도 경로로
공존시킴(플래그로 선택).

- `apply_health_message()` (신규): 기존 `approach_target()` 안에 인라인
  으로 있던 SYS_STATUS/GPS_RAW_INT/EKF_STATUS_REPORT/HEARTBEAT 파싱을
  공용 함수로 분리 - 아래 `wait_for_lock()`도 같은 판정 기준을 써야
  하므로, 로직이 두 곳에서 갈라지면 위험한 안전 체크 특성상 하나로
  합침.
- `approach_target()`: 반환 타입을 `void` -> `ApproachOutcome{kLanded,
  kHandedBack}`로 변경, `resume_mode` 파라미터 추가(기본값 빈 문자열 -
  기존 자가이륙 흐름은 동작 100% 그대로). `resume_mode`가 채워져 있으면
  (요격 흐름) 타겟 로스트/시간초과 시 LAND 대신 그 모드(AUTO)로 복귀.
  heartbeat 유실/예상치 못한 disarm은 요격 여부와 무관하게 항상 LAND
  (하드 결함이라 미션 복귀가 의미 없음).
- `wait_for_auto_armed()` / `wait_for_lock()` / `run_auto_intercept()`
  (신규): 각각 "AUTO+armed 대기" / "그 상태에서 타겟이
  `lock_confirm_sec` 이상 안정적으로 잡히고 건강상태도 정상일 때까지
  감시" / 위 둘을 엮은 메인 루프(대기->락확인->GUIDED 전환->
  approach_target(resume_mode="AUTO")->복귀 후 쿨다운->재대기 반복).
- `main()`: `--auto-intercept` 인자로 분기. 이 경로에서는 GUIDED 전환/
  arm/takeoff를 전혀 하지 않음.
- `setting/MAVLink.yaml` `target_track`에 신규 키 3개: `lock_confirm_sec`
  (1.0), `intercept_approach_duration_sec`(30), `reacquire_cooldown_sec`
  (3.0).
- 빌드 확인(경고 없음). 실제 AUTO 미션으로는 아직 테스트 못함(아래
  확인할 것 참고).

### 2. scripts/ 신설 - health_check.sh / run_YOLO.sh / full_mission.sh (완료, 여러 차례 조정)
`control/` 밑에 실행 스크립트가 없어서 새로 만듦. 처음엔 프로그램 개수
만큼 스크립트를 만들었다가("왜 이렇게 많냐" 지적받고) 요청받은 3개로
정리:
- **`health_check.sh`**: `health-check/`(다른 세션에서 이미 `control/`의
  진단 프로그램 5개를 분리해놓은 상태였음 - `check_alt`/`check_link`/
  `emergency`/`get_gps_location`/`test_arm`)에서 자동 게이트에 쓸 것만
  선별 - `check_link`(연결/heartbeat/배터리/gps)와 `emergency`
  (safety.yaml 기준 BREACH 여부). `test_arm`은 실제로 모터에 arm 신호를
  보내는 프로그램이라 자동 스크립트에서 명시적으로 제외.
- **`run_YOLO.sh`**: YOLO만 단독 실행하는 수동 테스트용.
- **`full_mission.sh`**: 처음엔 health_check -> yolo -> target_distance
  -> control을 곧바로 순서대로 다 띄웠는데, "욜로가 아직 아무것도
  못 잡았는데 거리계산/픽스호크 연결까지 미리 켜놓을 필요 없다"는
  지적을 받아 구조 변경 - health_check 통과 후 욜로만 먼저 띄우고,
  `/target`의 `confirmed:true`를 가볍게 폴링하며 대기하다가, 확정되면
  그때 target_distance/control을 띄우는 구조로 수정. (자세한 최종
  순서는 4번 항목 참고 - 그 사이 mavlink_proxy/telem_sender가 추가됨.)

### 3. YOLO_MODEL/cpp/yolo_headless.cpp 신설 - 실비행용 (완료, 방향 한 번 정정)
`full_mission.sh`가 쓰던 `yolo_live.cpp`는 브라우저 대시보드+영상
스트리밍까지 포함된 프로그램이라, 매 프레임 박스 그리기+JPEG 인코딩이
아무도 안 보고 있어도 무조건 도는 구조 - 실비행 중 젯슨 리소스 낭비라는
지적으로 새로 분리.
- 처음 버전: `draw_detections()`/`cv::imencode()`/`/stream`/`/`(대시보드)
  전부 제거, `/target`(target_distance.cpp가 쓰는 것)만 유지.
- **방향 정정**: "헤드리스"는 "젯슨에 사람이 붙어서 보는 서버(대시보드
  페이지)가 필요 없다"는 뜻이었지, "영상 자체가 필요 없다"는 뜻이
  아니었음 - 노트북 쪽 GCS(`gcs/tools/gcs_bridge.py`/`dashboard.html`,
  아직 미완성)가 `/stream`을 직접 받아서 그리는 구조(`gcs/PLAN.md`:
  "화면을 그리는 것은 노트북에서 담당하고, jetson은 json형태의 파일로만
  줌")이므로 `/stream`은 다시 복원. 최종적으로 뺀 건 `/`(브라우저용
  대시보드 HTML/JS) 하나뿐 - `/target`, `/stream`, `/conf`, `/start`,
  `/stop`, `/data`는 `yolo_live.cpp`와 동일하게 유지.
- `YOLO_MODEL/cpp/run_yolo_headless.sh` 빌드/실행 스크립트,
  `CMakeLists.txt`에 `yolo_headless` 타겟 등록.
- 빌드 확인(경고 없음), 실제 실행해서 `/target`/`/stream` 둘 다 정상
  응답 확인(카메라+TensorRT 엔진 로드까지 성공, 워밍업에 ~20초 소요).

### 4. MAVLink 시리얼<->UDP 프록시 신설 + 포트 실측 수정 (완료)
GCS(`scripts/gcs.sh`) 단독 테스트 중 실제로 발견한 두 가지 실제 버그:

**4-1. 픽스호크 시리얼 주소 불일치.** `setting/MAVLink.yaml`
`real.serial.address`가 `/dev/ttyACM0`으로 고정돼 있었는데, 지금 연결된
4mini는 `/dev/ttyACM1`로 잡혀 있었음(USB enumeration 순서가 어떤 보드를
꽂았느냐에 따라 매번 바뀜 - `dmesg`/`udevadm`으로 확인). `/dev/ttyACM1`
로 수정, `check_link`로 heartbeat 수신 확인. **6cmini로 다시 바꾸면 포트
번호가 또 바뀔 수 있음 - 그때 다시 확인 필요.**

**4-2. MAVLink UDP 프록시가 실제로는 없었음.** `setting/port.yaml`이
예전부터 "mavproxy/mavlink-router가 외부에서 떠서 시리얼을 UDP
14550/14551로 fan-out 해준다"는 전제였는데, 이 저장소/환경엔 그게 실제로
없었음 - `target_distance.cpp`가 계속 heartbeat 타임아웃으로 죽던 근본
원인. 이 환경의 `mavproxy.py`는 실행하자마자 core dump(깨진 설치),
`mavlink-routerd`는 미설치, sudo 불가로 apt 설치도 불가능해서 직접
구현:
- **`control/mavlink_proxy.cpp`** (신규): 시리얼을 혼자 열고 UDP
  14550/14551/14552(`mavlink_control`/`mavlink_sensor`/`mavlink_gcs`,
  마지막은 이번에 `setting/port.yaml`에 새로 추가)로 양방향 릴레이.
  MAVLink를 파싱하지 않고 바이트만 그대로 중계(수신 측이 어차피
  프레이밍을 다시 맞춤). **1차 구현에 버그 있었음**: fan-out 포트를
  `mav_transport.cpp`의 `UdpTransport`(bind)로 열어서, 다운스트림
  프로그램들도 같은 포트를 bind하려다 충돌 - 아무도 못 붙는 상태였음.
  `connect()` 기반 UDP 클라이언트 소켓(`UdpClientPort`, 이 파일 안에
  직접 구현)으로 교체해서 해결 - `target_link.hpp`의
  `TargetRangeSender`와 같은 패턴.
- **`gcs/sender/telem_sender_main.cpp`**: 시리얼(`real.serial`) 직접
  열던 걸 이 프록시의 `mavlink_gcs`(14552) 포트로 변경 -
  `control.cpp`/`target_distance.cpp`와 시리얼 포트 경합하던 문제
  (파일 상단 KNOWN LIMITATION으로 이미 적혀 있던 것) 해소.
- **`scripts/gcs.sh`**: 원래 `telem_sender`만 띄웠는데, target 필드가
  항상 null이라는 지적을 받고 확인해보니 `target_distance`(+그게
  필요로 하는 yolo)가 같이 안 떠서였음 - mavlink_proxy+yolo_headless+
  target_distance+telem_sender 4개를 한 번에 띄우는 "GCS 단독 테스트"
  스크립트로 확장.
- **`scripts/full_mission.sh`**: mavlink_proxy/telem_sender를 앞단에
  추가(health_check 다음, yolo/게이트 이전). `gcs.sh`를 통째로 불러
  쓰면 yolo_headless/target_distance가 이 스크립트 몫과 중복 실행되므로,
  `gcs.sh`를 부르지 않고 mavlink_proxy/telem_sender만 직접 띄우는
  구조로 분리.
- **실측 테스트(45초)**: mavlink_proxy 정상 fan-out, telem_sender
  프록시로 정상 연결, **target_distance가 더 이상 죽지 않고 계속
  살아서 도는 것 확인**(이게 원래 막혀 있던 문제).

### ⚠️ 실비행 전 확인할 것 (추가분, 2026-08-11)
- `control --auto-intercept`는 아직 실제 AUTO 미션으로 테스트 못함 -
  SITL이든 실기든 AUTO 모드로 날리면서 GUIDED 요격/AUTO 복귀가 실제로
  되는지 검증 필요.
- `mavlink_proxy.cpp`는 직접 만든 대체품(원래 기대하던 외부
  mavlink-router가 없어서) - 45초 테스트에서 heartbeat/명령 왕복은
  확인했지만, `target_distance`가 요청한 ALTITUDE 스트림은 그 시간 안에
  안 들어왔음(원인 미확인 - 픽스호크 파라미터 문제일 수도 있음, 프록시
  버그인지 재확인 필요). 장시간 운용 시 안정성도 미검증(바이트 단위
  라운드로빈 릴레이라 진짜 mavlink-router보다 지연/처리량이 불리할 수
  있음).
- `yolo_headless`의 카메라+모델 워밍업이 ~20초 걸림 - `full_mission.sh`
  의 confirmed 폴링 게이트가 이 시간을 기다려주는지는 확인했지만, 실제
  비행 시나리오에서 이 지연이 문제 안 되는지 재확인.
- 4mini `/dev/ttyACM1` 고정은 지금 연결 기준 - 6cmini나 다른 보드로
  바꾸면 포트 번호가 다시 바뀔 수 있음(`/dev/serial/by-id/`의 보드별
  고유 심볼릭 링크를 쓰는 게 더 안정적 - 아직 안 바꿈).
- git: `control_program_test1` 브랜치에 1번 항목(auto-intercept)까지만
  커밋/푸시됨. 2~4번 항목(yolo_headless, mavlink_proxy, telem_sender
  수정, 포트/설정 변경)은 아직 로컬에만 있고 커밋 안 됨.

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

### 9. control.cpp / emergency.cpp 재설계 - emergency.cpp를 자동 대응 경로에서 제외 (완료)
8번에서 만든 `emergency.cpp -> UDP -> control.cpp` 구조를 다시 걷어냄.
요청에 따라 "명령을 보내는 건 control.cpp 하나뿐"으로 더 단순화:

- **`control/emergency_link.hpp`/`.cpp` 삭제.** `setting/safety.yaml`의
  `emergency_link.udp_port` 섹션도 제거. `CMakeLists.txt`에서
  `emergency_link.cpp` 등록 해제.
- **`control.cpp`**: battery/heartbeat/gps/vehicle_health 감시 로직을
  `emergency.cpp`에서 그대로 가져와 직접 흡수. `drone::require_connection()`
  으로 커넥션을 얻어 `SYS_STATUS`/`GPS_RAW_INT` 구독 요청을 자체적으로
  보내고, 매 사이클(`kHealthPollTimeoutSec = 0.005s`, 기존 20Hz 사이클을
  거의 방해하지 않는 짧은 논블로킹성 폴링) `HEARTBEAT`까지 같이
  `recv_match`로 확인. 위반 시 LOITER 전환 + 최소 3초 유지 로직은 그대로
  유지, 다만 신호를 UDP로 받는 대신 자체 계산.
- **`emergency.cpp`**: 순수 읽기 전용 진단 도구로 축소 - `check_alt.cpp`
  와 동일한 포지션. `EmergencySender`/UDP 송신 전부 제거, MAVLink 명령도
  전혀 안 보냄(`open_connection()`만 사용, `drone::connect()` 아님). 이제
  자동 안전 대응 경로에는 관여하지 않고, 터미널에서 사람이 직접 상태
  보는 용도로만 남김 - 파일 상단 주석에 명시.
- **부수 효과**: 지난 턴에 지적했던 "emergency.cpp가 안 떠 있으면
  안전장치가 조용히 사라짐" 문제가 이 구조에서는 자동으로 해소됨 -
  이제 안전 체크가 `control.cpp` 자기 자신 안에 있어서 별도 프로세스의
  생존 여부에 의존하지 않음.
- 전체 빌드 확인(경고 없음), `control`/`emergency` 둘 다 `safety.yaml`을
  경고 없이 읽는 것까지 확인.

### 10. test_arm 안전장치 재정의
"test_arm"이 의미하는 바를 사용자가 정정: prearm 체크 통과 여부(걸 수
있는 상태)뿐 아니라 **현재 시동이 걸려있는지 자체**도 포함. `safety.yaml`
`vehicle_health_limit`에 주석 추가, `control.cpp`/`emergency.cpp` 둘 다
`HEARTBEAT`에서 `is_armed_from_heartbeat()`로 armed 상태를 읽어 매 사이클
로그에 출력(`armed=Y/N`). 단, armed 자체는 비행 중 정상 상태라 그것만으론
위반(breach)으로 판정하지 않음 - 정보 표시 용도. (armed가 "예상과 다르게
꺼짐"을 위반으로 볼지는 별도 논의 필요 - 미결정.)

### 11. YOLO 다중 타겟 검출 제외 - yolo_live.py에서 처리 (완료)
어디서 처리할지(`yolo_live.py` vs `emergency.cpp`) 컴퓨팅 리소스 기준으로
결정: `yolo_live.py`의 `inferer()`는 이미 매 프레임 전체 탐지
데이터프레임(`df`)을 갖고 있어서 `len(df) > 1` 체크가 사실상 공짜.
`emergency.cpp`에서 처리하려면 지금 없는 `/target` HTTP 폴링 관계를 새로
만들어야 해서(→ `target_distance.cpp`가 하는 걸 중복) 오히려 리소스가 더
듦 - `yolo_live.py` 쪽으로 결정.
- `state["target"]`에 `"detections"`(개수), `"multi"`(2개 이상 여부)
  필드 추가.
- 다중 검출 프레임은 `target_streak`에 실제 이름 대신 `None`을 넣어
  스트릭을 강제로 끊음 - 검출은 됐어도(`found=true`) `confirmed=true`로
  올라가지 못하게 막음(비슷한 대상이 여러 개일 때 프레임마다 다른 걸
  가리킬 수 있어서, 연속성 보장이 안 됨).
- `python3 -m py_compile` 통과 확인.

### 12. RC 오버라이드(조종기로 제어권 뺏김) 감지 - **논의 필요, 미구현**
사용자 지적: 지금 긴급상황 목록에 "조종기로 제어권을 뺏겼을 때(RC 인식
안 될 경우 재시도 안 함)"에 대한 처리가 전혀 없음. 코드는 아직 작성하지
않음 - 설계가 먼저 필요:
- 기본 방향(잠정): `control.cpp` 운용 중 픽스호크의 모드가 우리가 명령한
  적 없는 값으로 바뀐 게 감지되면(`HEARTBEAT.custom_mode`로 감지 가능),
  자동으로 다시 GUIDED를 잡으려 시도하는("재캐치") 로직은 **아예 두지
  말고 꺼두자** - 파일럿이 RC로 모드를 바꿨다면 그건 의도적 개입일
  가능성이 높으므로, 소프트웨어가 자동으로 재장악을 시도하면 오히려
  위험할 수 있음.
- 미결정 사항: (a) 이걸 단순 "명령 전송 중단"으로 끝낼지, 아니면 우리
  쪽에서도 명시적으로 어떤 상태로 전이할지, (b) RC 신호 자체의
  유실(픽스호크 자체 FS_THR_ENABLE 등 펌웨어 레벨 failsafe와는 별개로,
  우리 앱이 이걸 알아야 할 이유가 있는지), (c) 이 로직이 들어갈 위치
  (`control.cpp` 안 vs 별도 감시).
- 사용자 승인 전까지 코드 변경 없음 - 다음 논의에서 결정.

### 13. astroquad/uav-onboard 참고 조사 - 관련 발견 사항 반영
`github.com/astroquad`의 `uav-onboard` 레포(`SafetyMonitor.cpp/hpp`,
`config/safety.toml`, `GridMission.hpp` 상태머신)를 조사해서 다음을
확인/반영함:
- `battery low_voltage_v`가 저희와 같은 4S/14.8V 배터리 기준으로 14.0V로
  설정돼 있음 (14.8은 무부하 공칭값이라 부하 시 바로 걸릴 위험 - 14번
  항목에서 반영).
- "operator takeover: mode changed from GUIDED"를 최우선순위(Abort)로
  처리 - 12번 항목의 RC 오버라이드 논의에 대한 참고 근거.
- `EmergencyLand`가 별도 프로세스가 아니라 같은 상태머신의 상태 하나 -
  9번에서 이미 잡은 "control.cpp 하나가 다 처리" 방향과 일치.
- 타겟/라인 유실 시 짧으면 호버, 길면 LAND로 에스컬레이션 - 저희는 아직
  무한 호버만 함(추후 논의 대상, 이번엔 미반영).
- EKF는 실제 `EKF_STATUS_REPORT`의 variance 값(`pos_horiz`, `velocity`
  각각 임계값 1.0)을 사용 - 14번 항목에서 동일하게 반영.

### 14. 배터리 임계값 하향, heartbeat 유실 시 LAND 에스컬레이션, 실제 EKF 체크, 비행 로그 (완료)
- **`battery_limit.min_voltage_v`: 14.8 → 그대로 두되(astroquad 근거로
  14.0 권장 자체는 텍스트로 안내), 실제 반영은 다음 세션 확인 필요** —
  주의: 이번엔 코드/설정 값 자체는 아직 14.8로 남아있음, 낮추는 걸
  명시적으로 요청받으면 반영할 것.
- **heartbeat 5초 이상 유실 시 LOITER 대신 LAND**: `safety.yaml`
  `heartbeat_limit.land_gap_sec: 5` 추가. `control.cpp`에서 매 사이클
  `heartbeat_gap`을 계산해 `land_gap_sec` 초과 시 일반 breach(LOITER)
  경로보다 우선해서 `drone::land()` 전송 후 `approach_target()` 루프
  종료(더 이상 손 쓸 게 없다고 판단 - 재개 시도 안 함).
- **EKF 체크를 astroquad 방식대로 실제 구현**: 벤더링된
  `control/third_party/mavlink`엔 `EKF_STATUS_REPORT`(id 193,
  ardupilotmega.xml 소속, common.xml엔 없음)가 없었던 문제를, 로컬에
  설치된 `pymavlink`의 mavgen(C 생성기)으로 ardupilotmega.xml에서 해당
  메시지 헤더만 정식 생성해서 해결 - CRC_EXTRA/길이 등을 손으로 추측하지
  않고 authoritative하게 가져옴.
  - `control/third_party/mavlink/common/mavlink_msg_ekf_status_report.h`
    추가 (출처 주석 포함), `common.h`에 include 라인 +
    `MAVLINK_MESSAGE_CRCS` 테이블에 `{193, 71, 22, 26, 0, 0, 0}` 추가 +
    `MAVLINK_MESSAGE_INFO` 테이블에도 등록.
  - `safety.yaml`에 `ekf_limit`(`pos_horiz_variance_max`,
    `velocity_variance_max`, 각 1.0 - astroquad와 동일값, ArduCopter가
    GUIDED를 받아들이는 실제 기준) 추가.
  - `control.cpp`/`emergency.cpp` 둘 다 `EKF_STATUS_REPORT` 구독 +
    변조/판정 로직 추가 (`control.cpp`만 실제 조치, `emergency.cpp`는
    출력만).
- **예상치 못한 disarm 시 긴급 LAND** (이전 세션 위험 요소 5번 항목에
  대한 결정): 비행 중 `armed: true -> false` 전이가 감지되면(이 루프는
  스스로 disarm하지 않으므로 이 전이는 항상 외부 요인) 즉시
  `drone::land()`를 best-effort로 보내고 `approach_target()` 루프 종료.
- **비행 로그 CSV 기록**: `Document/logs/flight_<타임스탬프>.csv`에
  매 틱 + 주요 이벤트(LOITER 진입/유지/탈출, ALT-LIMIT 시작/복귀,
  UNEXPECTED_DISARM, HEARTBEAT_LAND_ESCALATION)마다 한 줄씩 기록.
  타임스탬프, 경과시간, 모드, armed, tracking, 거리, vx/vy/vz, 고도,
  battery %/V, gps fix/sat, prearm_healthy, system_status,
  emergency_active/reason, event 컬럼. 매 줄 flush - 비정상 종료 시에도
  직전 상황이 남도록.
- **작업 중 이슈**: `control/emergency.cpp`가 이번 세션 중 디스크에서
  사라져 있는 걸 빌드 중 발견함 (git엔 마지막 커밋 기준 빈 파일로 존재,
  작업 트리에서만 없어짐 - 원인 불명, 제가 의도적으로 지운 적 없음).
  마지막으로 작성했던 내용(순수 읽기전용 진단 도구 버전)대로 다시
  작성해서 복구, EKF 필드까지 포함해서 최신 스키마에 맞춤.
- 빌드 확인(경고 없음), `control`/`emergency` 둘 다 `safety.yaml` 신규
  섹션(`ekf_limit`, `heartbeat_limit.land_gap_sec`)을 경고 없이 읽는 것
  까지 확인.

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
- (2026-08-08, 9번 항목 이후로 갱신) `setting/safety.yaml`의
  `altitude_limit`, `battery_limit`, `heartbeat_limit`, `gps_limit`,
  `vehicle_health_limit`, `ekf_limit`은 전부(`battery_limit.min_voltage_v`
  포함, astroquad 참고해 14.0 권장했으나 아직 14.8로 안 낮춤) 실측/실비행
  검증 없이 넣은 값. 실제 배터리/운용 환경 기준으로 튜닝 필요.
- (2026-08-08, 14번 항목에서 해결됨 - 취소선 대신 명시) ~~emergency.cpp의
  EKF 상태확인은 proxy~~ → 14번 항목에서 실제 `EKF_STATUS_REPORT`(id 193,
  ardupilotmega.xml에서 pymavlink mavgen으로 생성) 기반으로 교체 완료.
  `ekf_limit.pos_horiz_variance_max`/`velocity_variance_max`(각 1.0)도
  실비행 검증은 안 됨 - astroquad가 쓰는 값을 그대로 가져온 것.
- (2026-08-08) "test_arm" 안전장치는 사용자가 직접 정정: prearm 체크
  통과 여부 + 시동 상태 자체(armed 여부)를 의미 - 10번 항목에서 반영
  완료, 더 이상 확인 필요 항목 아님.
- (2026-08-08, 9번 항목 이후로 갱신) `emergency.cpp`는 더 이상 LOITER/LAND
  등 자동 대응을 하지 않음 - 순수 읽기 전용 진단 도구로 축소됐고, 실제
  대응(LOITER 3초 유지 후 복귀, heartbeat 5초 초과 시 LAND, 예상치 못한
  disarm 시 LAND)은 전부 `control.cpp` 안에 있음. 지금 픽스호크가 물리적
  으로 연결 안 돼 있어 이 대응 로직들은 전부 SITL/실기 연결 후 검증
  필요 - 아직 한 번도 실제 MAVLink 스트림으로 테스트 못 함.
- (2026-08-08) "LOITER -> 재탐색 -> 접근 재개" 루프는 여전히 미구현
  (12번 항목의 RC 오버라이드 논의와도 얽혀 있음 - 다음 논의 대상).

## 2026-08-12

### 1. Gazebo + ArduPilot SITL 시뮬레이션 환경 구축 (완료)
목표: 실기체/픽스호크 없이, 그리고 실내 GPS 불가 문제도 우회해서
`control`/`health-check`/`gcs`의 실제 바이너리를 그대로 검증할 수 있는
시뮬레이션 경로를 만드는 것.

- PC(Windows + WSL2 Ubuntu 24.04)에 구축: Gazebo **Jetty 10.5.0**(`gz-jetty`,
  `libgz-sim.so.10`), ArduPilot SITL(ArduCopter, frame `gazebo-iris`,
  model JSON), `ardupilot_gazebo` 플러그인 3종
  (`libArduPilotPlugin.so`/`libCameraZoomPlugin.so`/`libGstCameraPlugin.so`).
- 설치 중 걸린 것들(기록용):
  - `ardupilot_gazebo`의 `CMakeLists.txt`는 `$GZ_VERSION` 기본값이
    `harmonic`(gz-sim8)이라 Jetty 환경에서 그냥 빌드하면 실패.
    `GZ_VERSION=jetty` 필요. `libopencv-dev`, `rapidjson-dev`,
    gstreamer dev 패키지도 필요.
  - MAVProxy 1.8.74가 최신 setuptools에서 제거된 `pkg_resources`를
    요구 -> venv에 `setuptools<81` 고정.
  - `sim_vehicle.py`가 venv가 아닌 시스템 python3로 실행돼
    `pexpect` 없음으로 SITL이 안 뜸 -> 시스템에 `python3-pexpect` 설치.
- **AUTO 미션 완주 검증 완료**: 40m 사각 웨이포인트 4개 + RTL 업로드 ->
  AUTO 전환 -> 각 변 완주(위경도 변화가 계산값과 일치) -> 홈 복귀 ->
  자동 하강 -> `t=81.7s`에 disarm. Gazebo 물리엔진 <-> SITL JSON FDM 루프가
  실제로 닫혀 있음을 확인.
- ⚠️ **`FRAME_CLASS`/`FRAME_TYPE`이 안 실려 있으면 arm 자체가 안 됨**:
  PreArm이 `Motors: Check frame class and type`로 거부. `gazebo-iris` frame의
  기본 파라미터가 eeprom에 반영되지 않아서 발생. 이번엔 MAVLink로 직접
  써넣어 해결(PC쪽 SITL eeprom에만 남음, 저장소 무관).

### 2. 젯슨의 실제 바이너리를 PC의 SITL에 연결 (완료)
`control/mavlink_proxy.cpp`는 `setting/MAVLink.yaml`의
`real.serial.address`만 열고 SITL/네트워크 경로가 아예 없음(`sitl:` 섹션은
`health-check/test_arm.cpp`만 읽음). 그래서 코드 수정 없이 붙이기 위해
PTY(가상 시리얼)를 만들어 SITL로 중계하고, 그 주소 한 줄만 임시로 PTY를
가리키게 함(테스트 종료 시 백업에서 자동 원복).

연결 경로:
```
젯슨 mavlink_proxy -> /tmp/sitl_serial (PTY)
  -> tcp 192.168.0.34:25760 (droneVideo, LAN)
  -> socat 중계 -> droneVideo 127.0.0.1:25760
  -> ssh -R 역터널
  -> PC(WSL) 127.0.0.1:5762 = SITL SERIAL1
```
포트 선택은 전부 실측 근거가 있음(추측 아님):
- **5762(SERIAL1), 5760(SERIAL0) 아님**: SITL의 SERIAL0 TCP는 클라이언트를
  하나만 받고 MAVProxy가 점유. 두 번째 연결은 붙기만 하고 데이터가 안 옴
  (소켓이 `CLOSE-WAIT`). 마침 `setting/MAVLink.yaml`의 `sitl.local_tcp`가
  이미 5762로 선언돼 있음. 단 SITL은 **SERIAL0에 클라이언트가 붙은 뒤에야**
  5762/5763을 염.
- **25760, 15760 아님**: 젯슨 sshd가 이미 15760을 쓰고 있음.
- **마지막 구간이 socat TCP 중계인 이유**: 젯슨 sshd가 `allowtcpforwarding
  yes`인데도 역방향 포워딩을 거부함(`remote port forwarding failed`).

**결과: `scripts/health_check.sh`가 SITL 상대로 "비행 가능(SAFE)" 통과.**
`check_link`가 SITL을 픽스호크로 정상 인식(GPS `fix_type=6`, 위성 10개,
mode=AUTO). 실내 GPS 문제도 시뮬레이션 GPS로 해결됨.
`mavlink_proxy`도 정상 동작(SITL에서 519KB 수신 후 14550/14551/14552/14553
팬아웃 확인).

### 3. ⚠️ control/ 결함 2건 발견 (이번엔 수정하지 않음)
시뮬레이션을 실제로 돌려서 드러난 것들. 둘 다 **시뮬레이션 전용 문제가
아니라 실비행에서도 동일하게 발생**함.

**3-1. `drone_lib.cpp:36-37` - 수신하는 모든 메시지가 command target을 덮어씀**
```cpp
if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
    target_system_ = msg.sysid;      // 필터와 무관하게 무조건 실행
    target_component_ = msg.compid;
```
`recv_match()`의 필터(`msg_ids`)를 통과하는지와 상관없이, 파싱에 성공한
**모든** 메시지가 target을 갈아엎음. 링크에 다른 MAVLink 화자(GCS,
MAVProxy sysid 255 등)가 하나라도 있으면 명령이 엉뚱한 시스템으로 감.
`developinglogHJ.md` §5는 이걸 "검증된 최초 ArduPilot flight-controller
HEARTBEAT로만 설정하도록 수정했다"고 기록해뒀지만 **실제 코드에는 그 수정이
없음**. 같은 §5의 "target 확인 전 `send()` 차단"도 `send()`에 가드가 없어
마찬가지로 미반영.

**3-2. `control.cpp` `run_auto_intercept()` / `wait_for_lock()` - 락 단계에 영영 도달 못 함**
`run_auto_intercept()`가 바깥 루프를 돌 때마다 `HealthState health;`를
**새로 만들어** `have_armed=false`인 채로 `wait_for_lock()`에 넘기는데,
그 함수의 첫 검사가:
```cpp
if (!health.have_armed || !health.armed ||
    health.custom_mode != copter_mode_mapping().at("AUTO")) {
    return false;   // 메시지 없이 조용히 반환 (호출부가 "이탈" 출력)
}
```
이 두 지점 사이에는 `recv_match(..., 0.1초)` **한 번**뿐이고, `have_armed`를
세팅하는 건 HEARTBEAT뿐인데 HEARTBEAT는 1Hz. 즉 대부분 하트비트가 도착하기
전에 검사가 돌아 즉시 false로 빠짐.

실제 증상(기체 arm + AUTO 3.5m 비행 중, `/target`이 `confirmed:true` 반환,
`control`이 `system = 1`로 정상 연결된 상태):
```
[AUTO_INTERCEPT] AUTO + armed 확인됨 - 타겟 확정 대기.
[AUTO_INTERCEPT] AUTO/armed 상태 이탈 - 대기 상태로 복귀.
   (무한 반복)
```
결정적 증거: `wait_for_lock()` 루프 안에서 5초마다 찍히는
`[AUTO_INTERCEPT] AUTO 비행 중 - 타겟 대기 (tracking=... healthy=...)`가
**단 한 번도 출력되지 않음** -> 함수가 매번 수 밀리초 만에 빠져나온다는 뜻.

### 4. 카메라 인식 경로 (Gazebo 영상 기반, 동작 확인)
`YOLO_MODEL/cpp/yolo_headless.cpp`는 카메라를 **`nvarguscamerasrc` 하드코딩
파이프라인**(젯슨 CSI/Argus 전용)으로 열기 때문에 Gazebo 영상을 물리적으로
읽을 수 없고, 소스 선택 플래그도 없음. 그래서 이 파일은 건드리지 않고,
`target_distance.cpp`가 폴링하는 `/target` HTTP 계약을 그대로 만족하는
별도 프로그램을 둠.

- `gazeboSim/gazebo_camera_target.py` (**PC에서 실행** - 시뮬레이션 카메라는
  거기에만 있음): `ardupilot_gazebo`의 `GstCameraPlugin`이 쏘는 H.264 RTP
  (`udp 127.0.0.1:5600`)를 OpenCV로 받아 검출 -> `/target` 제공.
  **8fps로 실제 영상 수신 확인.**
  - 스트림 켜는 법: 짐벌 센서의 `.../image/enable_streaming` 토픽에
    `gz.msgs.Boolean data:true` publish.
  - ⚠️ **pip의 `opencv-python`은 GStreamer 없이 빌드돼 파이프라인을 못 엶.**
    apt `python3-opencv`를 써야 함.
  - PC의 8002 포트를 젯슨의 `127.0.0.1:8002`로 역터널하면
    `target_track.yolo_host/yolo_port`를 그대로 둔 채 동작함(설정 변경 불필요).
  - 검출은 HSV 임계값 + 최대 컨투어 방식이라 **YOLO 모델 자체를 검증하지는
    않음**. 모델 검증이 목적이면 `yolo_headless.cpp`에 소스 선택 분기를
    넣는 수밖에 없음(참고: astroquad/uav-onboard는 `--vision gazebo` /
    `--vision rpicam` 플래그로 이 문제를 해결함).
- `gazeboSim/fake_yolo_target.py`: 카메라 없이 돌릴 때 쓰는 정적 합성 타겟.
  인식은 검증 안 되고, 그 아래(`target_distance`의 거리 계산,
  `control`의 GUIDED 요격)만 실제 코드로 검증됨.

### 5. 추가된 파일 (실비행 경로와 분리)
- `gazeboSim/` (신규): `bridge_sitl.sh`, `fake_yolo_target.py`,
  `gazebo_camera_target.py`, `MAVLink.yaml.sim-backup`, `README.md`(전체 사용법)
- `scripts/sim_mission_no_camera.sh` (신규): 합성 타겟 버전
- `scripts/sim_mission_camera.sh` (신규): Gazebo 카메라 검출 버전
- 두 스크립트 모두 `setting/MAVLink.yaml`을 시작 시 백업하고 종료 시
  (Ctrl+C 포함) 자동 원복함. `control/`, `health-check/`, `gcs/`,
  `YOLO_MODEL/` 소스는 **하나도 수정하지 않음**.
- `scripts/health_check.sh`는 두 스크립트가 호출하지 않음: `check_link`가
  시리얼을 직접 열어 `mavlink_proxy`와 PTY를 두고 충돌하기 때문
  (`full_mission.sh`가 health_check를 proxy보다 **먼저** 돌리는 것과 같은 이유).

### ⚠️ 실비행 전 확인할 것 (2026-08-12 추가분)
- **3-1, 3-2의 `control/` 결함 2건은 수정되지 않은 채 남아 있음.** 특히 3-2는
  `--auto-intercept` 요격 흐름이 **어떤 조건에서도 동작하지 않는다**는
  뜻이므로, 실비행 전에 반드시 고쳐야 함. 3-1은 GCS를 하나라도 같이
  붙이는 순간 명령이 엉뚱한 시스템으로 갈 수 있음.
- `setting/safety.yaml`의 `altitude_limit.hard_limit_m`이 **5.0m**이라,
  시뮬레이션 미션 고도를 3.5m로 잡았음. 이보다 높게 날리면 `control.cpp`가
  요격 대신 강제 하강 명령만 계속 보냄 - 테스트 시나리오 짤 때 주의.
- SITL의 `SYS_STATUS`는 `voltage=0.00V, battery=-1%`로 나옴(배터리 모니터
  미설정). `safety.yaml`의 `battery_limit.min_voltage_v`(14.0)와의 상호작용은
  이번에 별도로 검증하지 못함 - 요격 흐름이 3-2에서 먼저 막혀서 건강 검사
  경로까지 도달하지 못했기 때문. SITL에 `BATT_MONITOR`/`SIM_BATT_VOLTAGE`를
  설정하고 재검증 필요.
- 이번 시뮬레이션에서 검증된 범위는 **연결성 + AUTO 비행 + health_check
  통과 + target_distance 폴링**까지임. **GUIDED 요격 전환, 접근 유도, AUTO
  복귀, 재탐지 쿨다운은 3-2 때문에 한 번도 실행되지 못했으므로 여전히
  미검증**임.
- 파이프라인 빌드 과정에서 `control/build`, `gcs/build`의 산출물이 갱신됨
  (같은 소스 재빌드). 이 저장소는 build 디렉터리를 git에 추적 중이라
  `git status`에 변경으로 나타남 - 소스 변경은 아님.
- 시뮬레이션 종료 후 `setting/MAVLink.yaml`은 md5 대조로 원복 확인함
  (`real.serial.address: /dev/ttyACM1`). 스크립트가 비정상 종료된 경우를
  대비해, 실비행 전 이 값이 실제 시리얼 경로인지 항상 확인할 것.

## 2026-08-12 (2) — AUTO 요격 완주. control/ 결함 6건 수정

Gazebo/SITL에서 `control --auto-intercept`를 실제로 돌려서 나온 결함들. 전부
**시뮬레이션 전용 문제가 아니라 실비행에서도 동일하게 발생**한다.
증상은 하나였다 — 요격이 절대 발동하지 않음 — 였지만 원인은 6개가 겹쳐 있었다.

### 최종 결과

```
[AUTO_INTERCEPT] AUTO + armed 확인됨 - 타겟 확정 대기.
[AUTO_INTERCEPT] AUTO 비행 중 - 타겟 대기 (tracking=Y healthy=Y)
[AUTO_INTERCEPT] 타겟 확정 (1s 유지) - GUIDED로 전환합니다.
타겟 접근 모드 시작 (UDP 15020, 주기 50ms, 최대 30초, 고도제한 4~5m)
t=0.05s  TRACK dist=4.03m vx=0.12 yaw_rate=0.36 alt=4.00m armed=Y
t=10.25s TRACK dist=4.09m vx=0.41 yaw_rate=0.09 alt=4.00m armed=Y
[AUTO_INTERCEPT] 타겟 로스트 10.0s > target_lost_land_sec 10.0s - AUTO 로 복귀
[AUTO_INTERCEPT] AUTO 로 복귀함 - 3.0s 후 재탐지 대기.
```
`TRACK` 샘플 857개. yaw-first 유도가 설계대로 동작 — 정렬 전 `yaw_rate` 0.36 /
`vx` 0.12에서, 정렬되며 `yaw_rate` 0.09 / `vx` 0.41로 전환.
`AUTO 감시 → 락 → GUIDED 요격 → 접근 → AUTO 복귀 → 재탐지`가 한 바퀴 이상 순환.

---

### 1. `wait_for_lock()` — HealthState 초기화 경쟁

`run_auto_intercept()`가 바깥 루프를 돌 때마다 `HealthState`를 새로 만들어
`have_armed=false`인 채 넘기는데, `wait_for_lock()`의 첫 검사가 바로 그걸 본다.
그 사이엔 `recv_match(..., 0.1초)` 한 번뿐이고 HEARTBEAT는 1Hz다.

**Before**
```cpp
wait_for_auto_armed(vehicle, health_limit.max_heartbeat_gap_sec);   // void
std::cout << "[AUTO_INTERCEPT] AUTO + armed 확인됨 ...";
HealthState health;                                  // have_armed = false
health.last_heartbeat = std::chrono::steady_clock::now();
if (!wait_for_lock(...)) { ... continue; }           // 즉시 false
```
**After** — `wait_for_auto_armed()`가 판정에 쓴 그 하트비트를 호출자에게 넘긴다
```cpp
HealthState health;
wait_for_auto_armed(vehicle, health_limit.max_heartbeat_gap_sec, health);
std::cout << "[AUTO_INTERCEPT] AUTO + armed 확인됨 ...";
if (!wait_for_lock(...)) { ... continue; }
```
```cpp
if (is_armed_from_heartbeat(hb) && hb.custom_mode == copter_mode_mapping().at("AUTO")) {
    health.last_heartbeat = now;
    health.have_armed = true;
    health.armed       = true;
    health.custom_mode = hb.custom_mode;
    return;
}
```
**검증**: `wait_for_lock()` 내부에서 5초마다 찍히는
`[AUTO_INTERCEPT] AUTO 비행 중 - 타겟 대기 (...)`가 출력되기 시작. 수정 전엔
단 한 번도 나온 적이 없었다 = 매번 수 ms 만에 빠져나왔다는 뜻.

### 2. `drone_lib.cpp` — 수신 메시지가 command target을 덮어씀

`recv_match()`가 `msg_ids` 필터와 무관하게 파싱된 **모든** 메시지로 target을 교체.
GCS는 sysid 255로 하트비트를 내고 ArduPilot이 이를 시리얼 포트 간에 중계하므로,
이 프로세스가 보내는 모든 명령이 존재하지 않는 시스템으로 갔다.
`developinglogHJ.md` §5가 "수정했다"고 적어둔 그 동작이 실제로는 없었다.

**Before**
```cpp
if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
    target_system_    = msg.sysid;      // 필터와 무관, 매 메시지
    target_component_ = msg.compid;
```
**After** — 최초 비행 컨트롤러 하트비트에서 한 번만 latch
```cpp
if (!have_target_ && msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(&msg, &hb);
    if (hb.autopilot != MAV_AUTOPILOT_INVALID && hb.type != MAV_TYPE_GCS) {
        target_system_ = msg.sysid; target_component_ = msg.compid;
        have_target_ = true;
    }
}
```
**검증**: `연결됨. system = 255` → `연결됨. system = 1`

### 3. `wait_heartbeat()` — GCS 하트비트에 반환

아무 하트비트에나 반환해서, GCS가 먼저 말하면 `target_system_ = 0`인 채 연결이
끝났다. 직후 나가는 명령(특히 `target_distance`의 ALTITUDE 요청)이 시스템 0으로
갔다.

**Before**
```cpp
if (!recv_match({MAVLINK_MSG_ID_HEARTBEAT}, msg, timeout_sec)) return false;
if (out) mavlink_msg_heartbeat_decode(&msg, out);
return true;
```
**After** — target이 latch될 때까지 계속 읽는다
```cpp
while (true) {
    if (now >= deadline) return false;
    if (!recv_match({MAVLINK_MSG_ID_HEARTBEAT}, msg, remaining)) return false;
    if (have_target_) { if (out) mavlink_msg_heartbeat_decode(&msg, out); return true; }
}
```

### 4. `target_distance.cpp` — 고도 소스가 영영 안 옴

ALTITUDE(msgid 141)만 읽는데 `MAV_CMD_SET_MESSAGE_INTERVAL` 요청이
**`MAV_RESULT_DENIED`**로 거부된다. 컴포넌트 0/1 양쪽, `SR1_*`/`SR2_*` 전 그룹
10Hz 상태에서도 동일. 전체 msgid 조사에서도 141은 한 번도 안 나왔다.
→ `altitude_valid`가 계속 0 → 모든 range가 invalid → `tracking=N`.

**Before**
```cpp
if (connection->recv_match({MAVLINK_MSG_ID_ALTITUDE}, msg, cycle_sec)) {
    mavlink_altitude_t alt; mavlink_msg_altitude_decode(&msg, &alt);
    altitude_m = alt.altitude_relative;
```
**After** — 같은 값을 담고 실제로 흐르는 `GLOBAL_POSITION_INT`를 대체 소스로
```cpp
if (connection->recv_match({MAVLINK_MSG_ID_ALTITUDE,
                             MAVLINK_MSG_ID_GLOBAL_POSITION_INT}, msg, cycle_sec)) {
    if (msg.msgid == MAVLINK_MSG_ID_ALTITUDE) { ... }
    else {
        mavlink_global_position_int_t gpos;
        mavlink_msg_global_position_int_decode(&msg, &gpos);
        altitude_m = gpos.relative_alt / 1000.0;   // mm -> m
    }
```
ALTITUDE가 오는 기체에서는 기존 동작 그대로.

### 5. `target_distance.cpp` — 컴패니언 채널이 아예 조용함

ArduPilot은 요청하지 않은 채널로 거의 아무것도 보내지 않는다. SERIAL0만 활발한
이유는 MAVProxy가 거기서 `REQUEST_DATA_STREAM`을 보내기 때문. 컴패니언 채널은
HEARTBEAT / MISSION_ITEM_REACHED / STATUSTEXT뿐이고 **위치 메시지가 0건**이라
4번의 대체 소스도 읽을 게 없었다.
(`simulation/README.md`가 반대편에서 같은 현상을 기록해뒀다: *"MAVProxy가 연결되고
REQUEST_DATA_STREAM을 보낸 뒤 최초 유효 위치는 약 3.21초에 도착했다."*)

**After** — 연결 직후 한 번 보낸다
```cpp
mavlink_msg_request_data_stream_pack(255, 0, &stream_req,
    connection->target_system(), connection->target_component(),
    MAV_DATA_STREAM_ALL, static_cast<uint16_t>(1000.0 / args.sense_cycle_ms), 1);
connection->send(stream_req);
```
**검증**: `alt=0.00m (valid=0)` → `alt=4.00m (valid=1) ... dist=4.01m valid=1`

### 6. 건강 상태가 다른 시스템의 하트비트에 오염 (2곳)

`apply_health_message()`가 보낸 주체를 확인하지 않는다. GCS는 sysid 255,
**disarmed**, `custom_mode=0`으로 초당 한 번 말한다.

- `wait_for_lock()` — `armed`가 1Hz로 뒤집혀 `AUTO/armed 상태 이탈` 무한 반복
- `approach_target()` — 요격 중 `armed: true -> false`로 보여
  **예상치 못한 disarm 감지 → 긴급 LAND**. 판정 로직은 옳고 입력이 틀렸다.

MAVProxy를 끄면 증상은 사라지지만 실비행에선 불가 — GCS는 있어야 하고, 끊으면
ArduPilot 자체 GCS failsafe가 걸린다.

**After** — 두 곳 모두 sysid로 거른다
```cpp
if (vehicle.recv_match({...}, hmsg, kHealthPollTimeoutSec) &&
    hmsg.sysid == vehicle.target_system()) {
```
`approach_target()`에서는 `was_armed`가 블록 **안**에서 계산되므로 `if` 조건
자체에 붙여야 한다. `apply_health_message()` 호출만 감싸면 foreign 메시지가 든
틱에서 `was_armed`만 갱신돼 비교가 어긋난다.

### 7. `approach_target()` — 같은 UDP 포트를 두 번 바인딩

`run_auto_intercept()`가 `TargetRangeReceiver(udp_port)`를 만들어 실행 내내 들고
있는데(`wait_for_lock()`이 이걸 폴링), `approach_target()`이 **또 하나**를 같은
포트에 만든다. 요격에 들어간 순간부터 접근 루프는 아무 패킷도 못 받는
소켓을 읽는다 — 버퍼에 남아있던 것만 잠깐 `TRACK`이었다가 계속 `LOST`.
그동안 `target_distance`는 `valid=1`을 정상 발행 중이었다.

**Before**
```cpp
ApproachOutcome approach_target(..., const std::string& resume_mode = "") {
    ...
    TargetRangeReceiver receiver(udp_port);     // 두 번째 bind
```
**After** — 호출자의 것을 재사용, 없을 때만 자체 생성(main()의 자가이륙 흐름 보존)
```cpp
ApproachOutcome approach_target(..., const std::string& resume_mode = "",
                                 TargetRangeReceiver* shared_receiver = nullptr) {
    std::unique_ptr<TargetRangeReceiver> owned_receiver;
    if (shared_receiver == nullptr) {
        owned_receiver.reset(new TargetRangeReceiver(udp_port));
        shared_receiver = owned_receiver.get();
    }
    TargetRangeReceiver& receiver = *shared_receiver;
```
호출부: `approach_target(..., kResumeMode, &receiver);`
**검증**: `ss -lunp | grep 15020` → 바인딩 프로세스가 `control` 하나뿐.

---

### 시뮬레이션 전용 설정 (코드 아님, 종료 시 원복)

- **배터리**: SITL은 `SYS_STATUS.voltage_battery = 0.00V`를 보고한다.
  `safety.yaml`의 `min_voltage_v: 14.0` 미만은 breach이고, breach가 하나라도
  있으면 `wait_for_lock()`이 요격을 시작하지 않는다 — SITL에서 통과 불가능한 검사.
  `BATT_MONITOR=4`로 켜봤지만 아날로그 백엔드가 인스턴스화되지 않아
  (`BATT_VOLT_PIN` 등이 아예 존재하지 않음) 전압은 그대로 0이고, 대신
  `Arm: Battery 1 unhealthy`로 prearm까지 막혔다. `BATT_MONITOR=0`으로 되돌리고
  시뮬레이션 동안만 `min_voltage_v: 0.0`으로 낮춘다.
  원본은 `gazeboSim/safety.yaml.sim-backup`.
- **재부팅**: armed 상태에서는 ArduPilot이 `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`을
  거부한다. 파라미터 반영이 안 되는 것처럼 보이면 먼저 disarm할 것.
- **`Arm: Leaning`**: Gazebo 월드 리셋이나 SITL 재시작 후 모델이 기울어 있으면
  arm이 거부된다. Gazebo부터 순서대로 재기동하면 해결
  (`gazeboSim/start_pc_sim.sh`).

### ⚠️ 실비행 전 확인할 것 (추가분)

- 위 7건은 전부 **실비행에도 해당**한다. 특히 6번(GCS 하트비트 오염)은 GCS를
  붙인 실비행에서 요격 중 **오탐 disarm으로 긴급 착륙**을 유발한다.
- 4번의 `GLOBAL_POSITION_INT` 대체는 실기체에서 ALTITUDE가 나오면 기존 경로를
  그대로 쓴다. 다만 실기체 픽스호크에서 ALTITUDE가 실제로 스트리밍되는지는
  별도 확인 필요.
- 5번의 `REQUEST_DATA_STREAM`은 실기체 시리얼 포트에도 필요하다. 컴패니언
  컴퓨터가 붙는 포트의 `SRn_*`가 0이면 젯슨은 아무 telemetry도 못 받는다.
- 배터리 게이트는 시뮬레이션에서만 완화했다. **실비행 전 `safety.yaml`이
  `min_voltage_v: 14.0`으로 돌아와 있는지 반드시 확인할 것.**
- 요격 접근은 `dist≈4m`에서 `vx` 0.4m/s까지만 관찰했다. `stop_distance`(2.0m)
  도달과 정지 거동, `intercept_approach_duration_sec`(30s) 만료 경로는 아직
  미검증.

## 2026-08-14 — control/new_algorithm 신설(좌표기반 접근 알고리즘), Gazebo PC 재구축, SITL 검증

### 1. 오늘 논의 정리 문서 2개 작성

- **`Document/algorithm-renewer.md`**: 오전 대화(Guided 모드 PID 문제 -> 위치/속도
  하이브리드 제어 -> ROI 기반 추적 전략) 원문 정리 + 참고 레포 2개
  (`Kimhyuntae9665/Capstone-2025-team-32` GPS 기반, `astroquad/uav-onboard`
  optical-flow 기반) 비교 + 그 차이에 대한 의견 섹션(내가 추가했다고 명시).
- **`Document/algorithm.md`**: 이륙부터 최종 접근까지 전체 흐름 서술. 아래
  2~4번 작업을 반영하며 여러 차례 갱신됨 - 최종 버전이 현재 상태.

### 2. `control/new_algorithm/hybrid_guidance` 신설 - 설계가 세 번 바뀜

목표: 기존 `control.cpp`/`target_distance.cpp`는 **하나도 수정하지 않고**,
새 접근 알고리즘만 별도 실행파일로 추가. 재사용 가능한 건 전부 재사용
(`drone_lib`, `target_link`, `pos_calculator`, `yaml_settings`).

**1차 설계 (폐기)**: 거리 구간별로 위치 제어(멀리)/속도 제어(가까이)를 나누는
하이브리드 - `far_max_forward_speed_mps` 등으로 `SET_POSITION_TARGET_LOCAL_NED`의
위치 필드를 매 사이클 조금씩 미는 방식.

**2차 설계 (현재)**: 사용자 피드백으로 전면 재설계.
- **좌표기반 사선(대각선) 이동**: 기존 `control.cpp`의 `approach_target()`은
  "몸을 돌리고 나서 전진"이라 사선 이동이 아님. 매 사이클 `distance_m`(빗변)과
  `x_px`에서 변환한 `lateral_offset_m`을 **피타고라스로 분해**해서 전진/좌우
  성분을 동시에 계산 (`forward_m = sqrt(distance_m² - lateral_m²)`).
  `drone::send_velocity_body(vx, vy, 0, yaw_rate)`로 전송 - 새 MAVLink 코드
  없이 기존 함수 그대로 사용.
- **거리 구간**: `shell_radius_m`(5m) 밖은 등속 `cruise_speed_mps`(0.5m/s),
  안쪽은 지수 감속 `v(d) = cruise_speed_mps * exp(-decel_rate_per_m *
  (shell_radius_m - d))`, `stop_radius_m`(1m) 이하는 하드 0 (지수함수는
  수학적으로 정확히 0에 안 닿으므로).
- **5m 셸 재검증 게이트**: `shell_radius_m` 안쪽으로 처음 들어온 순간 제자리
  정지, `tracking`이 `reverify_hold_sec`(1초) 연속 유지돼야 감속 접근 시작 -
  YOLO의 5프레임 확정 스트릭(1차 게이트)과는 별개인 2차 오검출 방지 게이트.
  (사용자가 설명한 "4분면 스캔 + 640x640 ROI 재탐지"는 Jetson 카메라/TensorRT
  가 있어야 검증 가능해서 오늘 범위에서 제외 - `algorithm.md` 6번에 명시.)
- **감속 곡선 상수**: "5m에서 0.5m/s, 2m에서 0.1m/s" 두 기준점으로
  `decel_rate_per_m = ln(0.5/0.1)/(5-2) ≈ 0.536` 역산. 실측 검증 필요.

**3차 설계 (오늘 중 추가 수정)**: 사용자가 시작 흐름 자체를 정정 -
"GUIDED로 자체 이륙"이 아니라 **"AUTO로 이미 비행 중인 미션을 감시하다가
타겟 확정되면 GUIDED로 가로채는" 구조**여야 함. `wait_for_auto_armed()`/
`wait_for_target_lock()` 추가 - `control.cpp`의 `run_auto_intercept()`/
`wait_for_lock()`과 같은 패턴이지만, 그 함수들이 `control.cpp` 익명
네임스페이스의 private 함수라 링크가 안 돼서 새로 작성함 (아래 5번 참고 -
"버그가 있어서 못 쓴다"고 처음에 잘못 적었다가 정정).
추가로 **1m 정지 후 `stop_hover_sec`(5초) 호버링 -> `drone::land()`** 도
반영 (`algorithm.md`의 "1m 정지 이후 동작 미정" 항목 해소).

- 파일: `control/new_algorithm/{hybrid_guidance.hpp,.cpp,
  hybrid_guidance_main.cpp, CMakeLists.txt, README.md}`,
  `setting/hybrid_guidance.yaml`(신규 설정 파일), `scripts/run_new_algorithm.sh`.
- `control/CMakeLists.txt`는 건드리지 않음 - `new_algorithm/`이 자체
  `CMakeLists.txt`로 `control/`의 기존 `.cpp`들을 상대경로로 다시 컴파일해
  독립 실행파일(`control/build_new_algorithm/hybrid_guidance`)을 만듦.
  실행파일 위치를 repo root 기준 `control/build/`와 같은 깊이(2단계 아래)에
  고정한 이유: `yaml_settings.cpp`의 설정파일 탐색이 실행파일 경로 기준
  상대경로(`/proc/self/exe`)라서.

### 3. `control.cpp`의 prearm 체크 허점 발견 - `hybrid_guidance`에서 수정

아래 4번 SITL 검증 중 GUIDED 전환 직후 `[EMERGENCY] health breach - LOITER`가
매번 발생하는 걸 보고 원인 추적. `SYS_STATUS`의 PREARM_CHECK 비트를 실측:

```
SYS_STATUS prearm present=False healthy=False batt=100% volt=12600mV
GPS_RAW_INT fix_type=6 satellites=10
```

**이 SITL 빌드는 PREARM_CHECK 비트를 `present`로 아예 안 냄** (배터리/GPS는
정상). 기존 `control.cpp`의 `evaluate_health_breach()`도 `h.have_sys_status`
만 보고 `present` 비트는 확인 안 해서 **똑같은 허점**을 갖고 있음 - 즉
`control.cpp`도 실기체가 이 비트를 안 낸다면 똑같이 오탐 LOITER에 걸릴 수
있음(이번 세션에서 `control.cpp`는 고치지 않음, `hybrid_guidance`만 수정).

**Before** (`hybrid_guidance_main.cpp`)
```cpp
health.prearm_healthy = (s.onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK) != 0;
...
if (!h.prearm_healthy) return true;
```
**After** - present 비트도 같이 보고, present일 때만 판정 (배터리/GPS 미보고
값을 안전/위험 어느 쪽으로도 안 보는 것과 같은 원칙)
```cpp
health.prearm_present = (s.onboard_control_sensors_present & MAV_SYS_STATUS_PREARM_CHECK) != 0;
health.prearm_healthy = (s.onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK) != 0;
...
if (h.prearm_present && !h.prearm_healthy) return true;
```

### 4. Gazebo 시뮬레이션 PC(`192.168.0.116`) - 근본 원인 다른 컴퓨터, 새로 구축

2026-08-12 로그의 PC(Windows+WSL2, Gazebo Jetty)와 오늘 접속한 PC가 다른
장비였음 - `astrohome@192.168.0.116`, Gazebo Sim **8.14.0 (Harmonic)**.
`gazeboSim/README.md`가 가정하는 경로(`~/ardupilot`, `~/venv-ardupilot`,
droneVideo 릴레이)도 이 PC엔 없어서 처음부터 다시 구축.

**4-1. `ardupilot_gazebo`가 Gazebo Classic용으로 빌드돼 있었음.** 이 PC의
`/hailo_work/astrohome-main/src/ardupilot_gazebo`(다른 프로젝트 - VTOL 비행기,
재난구조 드론 - 가 쓰던 체크아웃, 공유 자원이라 손대지 않음)는
`find_package(gazebo REQUIRED)`(Classic 전용) 프로젝트였음. 설치된 건
gz-sim(Harmonic)뿐이라 `libArduPilotPlugin.so`가 `GzPluginHook` 심볼을
export 못 해 로드 자체가 거부됨(`gz sim` 서버가 월드는 로드하지만
FDM 포트 9002가 절대 안 열림). **해결**: 진짜 gz-sim용 원본
(`github.com/ArduPilot/ardupilot_gazebo`)을 별도 폴더
(`/home/astrohome/astro-drone-gzsim-plugin/`)에 새로 clone+build - 공유
체크아웃은 전혀 안 건드림. 재빌드한 플러그인의 `iris_runway.sdf`/
`iris_with_gimbal` 등이 `gazeboSim/README.md`가 원래 가정하던 것과 정확히
일치 - README가 애초에 이 원본 프로젝트를 전제로 쓰여 있었던 것으로 보임.
- 중간에 이 PC의 `ardupilot_gazebo`(구버전) 월드가 Gazebo Classic 머티리얼
  문법(`<script><uri>file://media/...gazebo.material</uri></script>`)을
  써서 gz-sim이 월드 로드 자체를 거부하는 별개 문제도 만남 - 원본은 안
  건드리고 패치된 모델 사본을 `gazeboSim/models_patched/`에 만들어 우회
  (최종적으론 진짜 원본 저장소로 넘어가면서 이 우회 자체가 불필요해짐).

**4-2. Jetson -> PC 직접 연결이 방화벽에 막혀 있었음.** `gazeboSim/README.md`의
droneVideo 릴레이 체인이 이 환경엔 안 맞음. PC의 `ufw`가 기본 deny-incoming
(`22`/`433`/`3389`만 허용)이라 5762(SITL SERIAL1)가 막혀 있었던 것 -
`sudo ufw allow from 192.168.0.216 to any port 5762 proto tcp`로 **이
Jetson IP에서만** 좁게 허용해서 해결 (Anywhere로 안 열었음). ⚠️ 이 규칙은
아직 PC에 남아있음 - 되돌릴지 결정 필요.
- 이후 젯슨 쪽은 `gazeboSim/bridge_sitl.sh`를 그대로 쓰지 않고(하드코딩된
  droneVideo 주소라 이 토폴로지엔 안 맞음), 같은 기법(PTY <-> TCP socat
  릴레이)을 이 PC 주소로 직접 실행 - `bridge_sitl.sh` 자체는 안 건드림.

### 5. `hybrid_guidance` 실기(SITL) 검증 - AUTO 감시 ~ DECEL까지 확인

Jetson의 실제 `mavlink_proxy`/`target_distance`/`hybrid_guidance` 바이너리를
PC의 Gazebo+SITL에 붙여서 끝까지 돌림 (PC에서 `pymavlink`로 arm+더미
1-웨이포인트 미션 업로드+AUTO 전환).

```
[HYBRID_GUIDANCE] AUTO + armed 확인됨 - 타겟 확정 대기.
[HYBRID_GUIDANCE] AUTO 비행 중 - 타겟 대기 (tracking=Y)
[HYBRID_GUIDANCE] 타겟 확정 - GUIDED로 전환합니다.
GUIDED 모드 확인됨.
t=0.00s TRACK mode=REVERIFY dist=4.50m vx=0.00 vy=0.00 alt=4.50m armed=Y
... (1초 유지 후)
t=36.03s TRACK mode=DECEL dist=4.50m vx=0.38 vy=0.00 alt=4.50m armed=Y
```
`v(4.5) = 0.5*exp(-0.536*0.5) = 0.382` - 손으로 유도한 감속 공식과 실측
`vx=0.38`이 정확히 일치. `AUTO 감시 -> 타겟 확정 -> GUIDED 전환 -> REVERIFY
-> DECEL`까지 실제 SITL 텔레메트리로 검증됨(3번 항목의 prearm 버그를 여기서
발견/수정).

**미검증으로 남은 것**: `gazeboSim/fake_yolo_target.py`는 `x_px`/`y_px`가
고정값이라, 기체가 실제로 전진해도 `distance_m`이 안 줄어듦(고도만으로
`distance_m`이 정해지는 구조) - `sim_mission_no_camera.sh`도 같은 한계를
README에 이미 명시해둔 부분. STOPPED 진입/5초 호버/착륙까지 보려면 Gazebo에
타겟을 스폰하고 `gazebo_camera_target.py`(실제 카메라 인식)를 쓰는
`--camera` 경로가 필요 - 다음 단계로 남김.

추가로 이 Jetson의 `yolo_headless`가 이미 실제 서비스로 떠 있어서(8002 포트
점유) `fake_yolo_target.py`는 8100 포트로 대신 띄우고
`target_distance --yolo-port 8100`으로 넘김 - 실제 서비스는 안 건드림.

### 6. Jetson 로컬 `numpy`가 이 ARM CPU에서 죽어있음 (세션과 무관, 안 건드림)

`pymavlink`로 arm/모드 전환을 이 Jetson에서 직접 하려다 발견:
```
$ python3 -c "import numpy"
Illegal instruction (core dumped)
```
`numpy==1.19.5`가 이 aarch64 CPU와 안 맞는 것으로 보임 - YOLO 스택이 의존할
가능성이 있어 손대지 않고, 같은 작업을 PC의 `mavproxy` venv(`pymavlink`
포함, 정상 동작)에서 대신 실행함.

### 7. ⚠️ 문서 stale 발견 - `control/README.md`/`gazeboSim/README.md`

2번 항목에서 "`run_auto_intercept()`에 `HealthState` 버그가 있어서 못 쓴다"고
적었다가, 실제로는 **2026-08-12(2)에 이미 고쳐졌다는 걸 뒤늦게 확인**
(현재 `control.cpp` 923-924줄도 고친 패턴 그대로). `control/README.md`/
`gazeboSim/README.md`의 "Known blocker" 섹션이 그 수정 이후로 갱신이 안 돼서
생긴 착오 - 오늘 작성한 문서(`algorithm.md`, `control/new_algorithm/README.md`)
에서는 정정했지만, **두 README 자체는 이번 세션에서 안 고침** - 다음에 정리
필요.

### 8. `control.cpp`에도 같은 prearm `present` 비트 수정 반영 (완료)

3번에서 `hybrid_guidance`에만 넣었던 수정을 `control.cpp`의
`HealthState`/`evaluate_health_breach()`/`apply_health_message()`에도
동일하게 반영 (사용자 요청). 같은 세 지점, 같은 패턴:

```cpp
// HealthState에 필드 추가
bool prearm_present = false;
bool prearm_healthy = true;

// apply_health_message() - SYS_STATUS 케이스
health.prearm_present = (s.onboard_control_sensors_present & MAV_SYS_STATUS_PREARM_CHECK) != 0;
health.prearm_healthy = (s.onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK) != 0;

// evaluate_health_breach()
if (h.have_sys_status && limit.require_prearm_healthy && h.prearm_present && !h.prearm_healthy) {
    reasons.push_back("prearm check unhealthy");
}
```
빌드 확인(경고 없음). SITL로 재검증은 안 함(PC가 이미 꺼진 뒤 반영) - 로직은
`hybrid_guidance`에서 이미 검증된 것과 동일.

### 9. 포트 정리 + 시뮬레이션 PC 종료 (완료)

- `setting/port.yaml`에 `sim_hosts:` 섹션 신설 - `wsl_droneVideo_relay: 5762`
  (2026-08-12 세션의 WSL2 PC, `gazeboSim/README.md`가 가정하는 그 경로),
  `astrohome_192_168_0_116: 5764`(오늘 세션의 PC). 사용자 지적: "5762는 원래
  다른 컴퓨터랑 연결하던 포트 아니었냐" - 맞음, 두 개의 서로 다른 시뮬레이션
  PC가 같은 "5762"(SITL의 SERIAL1 고정 포트)를 쓰는 셈이라 프로젝트 기록상
  헷갈릴 수 있어서 PC별로 다른 번호를 문서화. 5763은 이미 그 PC의 SITL
  SERIAL2가 쓰고 있어서(`bind port 5763 for SERIAL2`, 4번 항목 로그 참고)
  제외, 5764로 지정. **지금은 실제로 붙어있는 연결이 아니라 예약/기록용** -
  다음에 그 PC의 SITL을 다시 켤 때 이 번호를 실제로 쓰려면 별도 릴레이/포트
  포워딩 설정이 필요함.
- PC(`192.168.0.116`)의 Gazebo/SITL/MAVProxy 전량 종료(`pkill`). `ufw` 규칙은
  사용자 지시로 그대로 둠("나머지는 냅둬").

### ⚠️ 실비행 전 확인할 것 (추가분, 2026-08-14)

- `control/README.md`/`gazeboSim/README.md`의 "Known blocker"(HealthState
  race) 섹션이 stale함 - 2026-08-12(2)에 고쳐진 걸 반영해서 갱신 필요.
- PC(`192.168.0.116`)의 `ufw` 규칙(이 Jetson IP에서 5762 허용)이 아직 남아
  있음 - 계속 열어둘지는 사용자가 나중에 결정하기로 함(2026-08-14 9번 항목).
- `decel_rate_per_m`(0.536)은 "5m/0.5m/s, 2m/0.1m/s" 두 점만으로 역산한 값 -
  실측 재조정 필요.
- `hybrid_guidance`의 STOPPED -> 5초 호버 -> 착륙 경로는 정적 fake 타겟
  한계로 아직 실행 검증 못함 - `--camera` 모드로 후속 검증 필요.
- (2026-08-14, 8번 항목에서 해결됨) ~~`hybrid_guidance`가 수정한 prearm
  `present` 비트 체크를 `control.cpp`에도 반영할지 결정 필요~~ →
  `control.cpp`에도 동일하게 반영 완료, SITL 재검증만 남음.
- `setting/port.yaml`의 `sim_hosts.astrohome_192_168_0_116: 5764`는 아직
  예약값뿐 - 실제로 그 PC를 다시 쓸 때 5764를 실제 릴레이 포트로 연결하는
  작업이 필요.

---

## 2026-08-15 — PC 단독 YOLO 시뮬 파이프라인 구축, 접근 시퀀스 첫 완주

이날의 시행별 상세 기록은 `Document/simulation-log/` 에 커밋 단위로 분리해
두었다(`08151210` / `08151240` / `08151300` / `08151700`). 여기에는 흐름과
결론만 남긴다.

### 0. 출발점 - 시뮬레이션에서 YOLO 는 한 번도 돈 적이 없었다

`gazeboSim/gazebo_camera_target.py` 는 HSV 색상 임계값이지 신경망이 아니다.
`YOLO_MODEL/cpp/yolo_headless.cpp` 는 GStreamer 파이프라인이
`nvarguscamerasrc`(Jetson L4T Argus)로 하드코딩돼 Gazebo 프레임을 못 읽는다.
그래서 지금까지 시뮬은 **제어 계층만** 검증해 왔고 인지 계층은 합성 타겟으로
대체돼 있었다.

### 1. `gazeboSim/gazebo_yolo_target.py` 신설 - PC 에서 실제 가중치 구동 (완료)

PC(WSL Ubuntu)에 `torch` / `torchvision` / `ultralytics` / `onnxruntime` 이 전부
없다. 대신 apt OpenCV 4.6.0 이 GStreamer 와 dnn 을 함께 갖고 있어
`cv2.dnn.readNetFromONNX` 로 **추가 설치 없이** 실제 가중치를 돌릴 수 있었다.

- 모델: `YOLO_MODEL/0812best.onnx` (Jetson 에서 PC 로 복사, md5
  `8b938b4690879b3d2622395570f15efd` 양쪽 일치 확인)
- `0812best.engine`(TensorRT)은 PC 에서 못 쓴다 - 엔진은 특정 GPU + TensorRT
  버전으로 직렬화돼 있다. `.onnx` 가 이식 가능한 산출물.
- PC 의 옛 체크아웃에 남아 있던 `prototype.onnx` / `prototype.pt` 는 쓰면 안 됨.
  Jetson 에는 이미 없는 파일이고(커밋 `35658ae` 에서 삭제) 320x320 로 export 된
  이전 모델이다.

`cv2.dnn` 으로 측정한 모델 형상(추정 아님):

```
입력   640x640 고정   ← 320/416/640x480 전부 model.24 Reshape 에서 실패
출력   1x25200x6      25200 = (80^2+40^2+20^2)x3,  6 = [cx,cy,w,h,obj,cls0]
클래스 1개 "astro-drone"
추론   CPU 30~42 ms (약 8 infer/s. 카메라가 ~8 fps 라 충분)
```

전처리는 `YOLO_MODEL/cpp/trt_engine.cpp` 를 읽고 실기와 동일하게 맞췄다 -
`letterbox()` + `copyMakeBorder(..., Scalar(114,114,114))` + RGB + `/255.0`,
카메라 해상도도 640x480 으로 같다. 확정 규칙(`DEFAULT_CONF=0.25`,
`TARGET_CONFIRM_FRAMES=5`, 다중검출 프레임 폐기)도 `yolo_live.py` 그대로 옮겼다.

### 2. `gazeboSim/run_sim_demo.sh` 신설 - 명령 한 줄로 전체 기동 (완료)

```powershell
wsl -e bash -lc "SIM_FX=205.47 ~/run_sim_demo.sh"
```

Gazebo GUI → 타겟 스폰 → 카메라 스트리밍 → 검출기 → 역터널/socat →
Jetson 스택(`run_new_algorithm.sh --sim --camera`) → 미션 비행 → 종료 시 전량
정리까지 7단계를 한 번에 한다. Jetson 로그는 `[JETSON]` 접두어로 같은 화면에
흘린다. 미션 본체는 `gazeboSim/sim_mission_core.py` 로 분리.

### 3. 흰색 바구니 타겟 반영 (완료)

사용자가 실측값을 줬다: **0.238 x 0.138 x 0.070 m, RGB(244,247,244) `#F4F7F4`**.

처음에는 STL 메시를 실물 크기로 줄이고 흰색을 칠했는데, astrohome
(`192.168.0.116`)의 `~/gazebo_yolo_bridge_0815/WHITE_BASKET_SETUP.md` 에 더 나은
해법이 이미 있었다 - **학습셋에서 잘라낸 실사 텍스처를 평면에 입히는 것**.
도메인 갭이 사라지므로 그쪽 모델을 그대로 가져왔다(텍스처 md5 `1ca0a3fd…`).

- `collision` 은 실측 그대로 `0.238 x 0.138 x 0.070`, `pose z=0.035`
- `visual` 은 실사 텍스처 평면. 실물보다 큰 이유는 "바구니를 키운 것"이 아니라
  저해상도 카메라에서도 학습 때와 같은 픽셀 크기로 보이게 하려는 것

평면 크기는 문서(3.10x2.07, 640x480 기준)와 SDF 주석(1.033x0.690, 1920x1080
기준)이 정확히 3배 차이였는데 **둘 다 맞는 값**이었다(해상도비가 3). 다만
하강 구간에서 갈렸다.

| 고도 | 1.033x0.690 | 3.10x2.07 |
|---:|---:|---:|
| 4.0 m | 0.777 | **0.959** |
| 2.0 m | 0.907 | 0.323 |
| 1.2 m | 0.733 | **미탐지** |

문서의 재현 조건은 고도 4 m 고정이라 그 구간에서만 최적이다. 우리 미션은
4 m → 1 m 로 하강하므로 **기본값은 SDF 원본 1.033x0.690** 으로 두고,
고정고도 진단이 필요할 때만 `PLANE_W`/`PLANE_H` 로 바꾸도록 했다.

### 4. ⚠️ `pixel_focal_length_px` 2.58배 오차 - 접근이 거의 수직으로 떨어졌다

`setting/MAVLink.yaml` 의 `pixel_focal_length_px: 530` 인데 이 시뮬 카메라의
실측값은 **205.47** 이다(`Document/algorithm.md` 기재값과 일치. 직접 재서
202.4 로 교차검증, 오차 1.5%).

`pos_calculator.cpp` 는 `ground_offset_m = pixel_offset / fx x altitude_m` 이라
fx 가 분모다. 2.58배 크게 잡으면 **수평 거리를 그만큼 작게 본다.** 3차원 사선
분해에서 수직 성분은 고도 그 자체라 영향이 없고 **수평 성분만 과소평가**되므로,
기체가 거의 수직으로 내려온다.

실비행 로그에 그대로 나왔다 - 사선 이동이면 일정해야 할 "수평오프셋/고도" 비가
접근 시작 0.53 에서 종료 시 1.16 으로 **커졌다.** 고도 1.12 m 에서 타겟이
1.31 m 옆에 남았는데 그 고도의 지상 반폭이 1.74 m 라 타겟이 프레임 밖으로
잘렸다 → 로스트 → 긴급착륙.

`run_sim_demo.sh` 에 `SIM_FX` 옵션을 넣었다(종료 시 `setting/MAVLink.yaml` 과
`gazeboSim/MAVLink.yaml.sim-backup` 양쪽을 530 으로 원복). **기본은 꺼져 있다** -
실비행 캘리브레이션 값이라 임의로 바꾸지 않았다.

### 5. ⚠️ 스크립트 결함 - 카메라 스트리밍을 켜고 검증하지 않았다 (수정)

`gz topic -p` 는 한 번 쏘고 즉시 종료한다. gz-transport 는 발행자가 구독자를
발견하는 데 시간이 걸리므로 발견 전에 죽으면 메시지가 유실된다. 실제로 한 번
유실되어 **프레임이 한 장도 안 왔는데 스크립트는 `[OK]` 를 찍고 진행**했고,
176 샘플 내내 `탐지=-` 인 채로 비행이 끝났다.

udp 5600 에 RTP 가 실제로 흐르는지 확인하고 흐를 때까지 최대 6회 재발행, 끝내
안 되면 진행하지 않고 중단하도록 고쳤다. 이전 실행들이 성공한 것은 운이었다.

### 6. ⚠️ 근접에서 타겟이 두 박스로 쪼개져 `confirmed` 가 영영 안 섰다 (수정)

가장 오래 걸린 결함. 정지 호버 대조군이 원인을 갈랐다(10 Hz 로 20초씩 폴링).

| 고도 | found | confirmed |
|---:|---:|---:|
| 2.45 m | 100% | 100% |
| **1.20 m** | **100%** | **0%** |

전송 지연이 아니다(age 중앙값 61~65 ms). `found` 는 완벽한데 `confirmed` 만 0%.
검출기의 `x_px` 가 -28 과 +26 을 왕복했다 - **타겟이 좌/우 두 조각으로 잡히고**,
"다중검출 프레임 = 미검출" 규칙이 이를 서로 다른 물체로 오인해 5프레임 스트릭이
매 프레임 리셋된 것이다. `target_distance.cpp` 는 `confirmed=false` 를
`found=false` 와 동일하게 취급하므로 그대로 로스트로 이어졌다.

두 박스가 약 55 px 떨어져 **겹치지 않으므로 NMS 로는 어떤 IoU 임계값으로도
합칠 수 없다.** 그래서 판정 기준을 겹침 비율에서 **가장자리 간격**으로 바꿨다.

```
한 물체의 조각   : 간격 -7, -3, -6 px  (붙어 있거나 겹침)   ← 실측 1.2/1.0/0.9 m
서로 다른 두 타겟 : 간격 +7 px          (떨어짐)            ← 4 m 에서 1 m 간격
```

`gazebo_yolo_target.py` 에 `MERGE_GAP_FRAC = 0.1` 기반 근접 병합을 넣었다.
군집에 들어온 박스는 경계를 합집합으로 넓혀 세 조각 이상도 사슬처럼 이어붙이고,
대표 위치를 **합집합 중심**으로 써서 `x_px` 진동(-28 ↔ +26)도 없앴다.
단위 9케이스 검증 - 서로 다른 두 타겟은 그대로 2개로 남는다.

결과: 1.20 m / 1.00 m 에서 `confirmed` **0% → 100%**, `x_px` -1.8~-3.3 로 안정.

### 7. 접근 시퀀스 첫 완주 (완료)

```
[JETSON] [HYBRID_GUIDANCE] 1m 정지 5.00초 호버링 완료 - 착륙.
  결과: 검출기=yolo  가로채기=성공  최저고도=0.21m
mode 분포 : DECEL 648 / REVERIFY 20 / STOPPED 100 / HOLD 16
TRACK 772 / LOST 16   (추적률 98%)
```

`Document/algorithm.md` 의 시퀀스 전체 - AUTO 감시 → 타겟 확정 → GUIDED
가로채기 → 5 m 셸 재검증 → 지수 감속 → **1 m 하드 정지** → 5초 호버 → 착륙 -
가 처음으로 끝까지 돌았다. 2026-08-14 항목에 "정적 fake 타겟 한계로 아직 실행
검증 못함"으로 남겨뒀던 STOPPED → 호버 → 착륙 경로가 이걸로 검증됐다.

시행별 추이:

| | 기준 | +SIM_FX | +NMS | +근접병합 |
|---|---:|---:|---:|---:|
| 추적률 | 42% | 72% | 20% | **98%** |
| 최소 `dist` | 1.47 m | 1.27 m | 2.48 m | **0.97 m** |
| `STOPPED` | 없음 | 없음 | 없음 | **100 샘플** |
| 종료 | 긴급착륙 | 긴급착륙 | 긴급착륙 | **정상 착륙** |

### 8. 고도 상한 5 m → 10 m (완료, 사용자 요청)

`setting/safety.yaml`: `soft_limit_m` 4.0→8.0, `hard_limit_m` 5.0→10.0.
`hybrid_guidance_main.cpp:321-322` 가 YAML 에서 읽는다(하드코딩 4.0/5.0 은 파일을
못 읽을 때의 폴백).

**단, 상한과 실사용 한계는 다르다.** 현재 타겟 평면(1.033x0.690)에서 잰 값:

| 고도 | 4 m | 5 m | 6 m | 7 m | 8 m | 10 m |
|---|---:|---:|---:|---:|---:|---:|
| 타겟 폭 | 53 px | 42 px | 35 px | 30 px | 27 px | 21 px |
| conf | 0.785 | 0.680 | 0.651 | 0.549 | 0.365 | **0.000** |

**실제 탐지 한계는 6~7 m.** 10 m 로 날리려면 평면을 약 2.6 m 로 키워야 한다
(`PLANE_W=2.6 PLANE_H=1.74`). 기본 비행고도는 4.0 m 유지.

### 9. 하향 카메라가 기체 자기 암을 보고 있었다 (수정, 사용자 요청)

사용자 지적: "스탠드오프는 실기에 없으니 시뮬에서도 빼라."

확인해 보니 **원인이 스탠드오프가 아니었다.** `ensamb_with_standoffs` 의 네
`*_leg_visual` 은 이미 `<transparency>1</transparency>` - 완전 투명이다. 화면에
찍히던 회색 막대는 **기체 본체 메시(`ensambFinal.STL`)의 암**이었다.

STL 바운딩박스를 계산해 원인을 특정했다(삼각형 245,214개).

```
메시 z 범위(scale+pose 적용) : -0.1026 ~ +0.1027 m
down_camera_link            : z = -0.021 m      ← 메시 안쪽에 박혀 있음
```

카메라가 프레임 **속에** 있어 메시가 카메라보다 0.0816 m 아래까지 내려온 것이
시야에 들어왔다. 카메라를 `z = -0.13`(메시 바닥보다 0.027 m 아래)으로 내렸다.

결과: 모서리의 회색 막대 완전 제거. 탐지 점수도 전 구간 상승
(3.0 m 0.756→0.823, 2.2 m 0.839→0.879). 전체 비행 회귀 통과(LOST 16→13).

`simulation/` 은 이 브랜치에서 추적되지 않는 다른 브랜치 자산이라 원본을
`model.sdf.orig` 로 백업하고 재현용 패치를
`gazeboSim/patches/ensamb_down_camera_below_frame.patch` 에 남겼다.

이 결함이 중요한 이유는, 2026-08-15 초반 실험에서 **모델이 타겟(0.016)보다 이
모서리 구조물(0.10~0.87)에 훨씬 강하게 반응**했기 때문이다. 게다가 자기 구조물은
매 프레임 같은 자리에 있어 5프레임 확정 스트릭을 **오히려 안정적으로 통과**한다.

### 10. ⚠️ 문서 오류 발견 - 카메라 FOV 계산

`down_camera` 의 SDF 는 `horizontal_fov: 2.7507`(157.6°)인데 실측 fx 는 205 px
(FOV 약 2.0 rad)다. `algorithm.md` 기재값과 일치하므로 **렌더링되는 실제 FOV 가
SDF 값과 다르다.**

이 때문에 `gazeboSim/README.md` 의 "at 4 m altitude ... about 16 px per metre"
계산이 틀렸다. 실제는 **51 px/m**. 원래 0.238 m 바구니가 4 px 가 아니라 약 12 px
로 보인다는 뜻이고, 기존 8배 스케일의 근거도 그만큼 흔들린다. 지금은 실사 텍스처
평면 방식이라 실무 영향은 없지만 문서는 고쳐야 한다.

### 11. Jetson 타임존 교정 + SSH 경고 정리 (완료)

- Jetson 이 `America/New_York`(EDT)로 잡혀 있어 로그 파일 시각이 PC 와 13시간
  어긋나 있었다. `timedatectl set-timezone Asia/Seoul` 로 교정. UTC 자체는
  정확했고(`systemd-timesyncd` 동기화 중) 표시만 틀렸다. **이 날짜 이전의 로그
  파일 mtime 을 볼 때 주의할 것.**
- `run_sim_demo.sh` 의 SSH 옵션에 `-o LogLevel=ERROR` 추가.
  `UserKnownHostsFile=/dev/null` 때문에 매 접속이 "처음 보는 호스트"가 되어
  `Warning: Permanently added ...` 가 반복 출력되던 것. 동작은 그대로.

### 12. 세션 중 Jetson 이 12:46~13:00 네트워크에서 사라짐

droneVideo 에서 ping 100% 손실, ARP `FAILED`(L2 무응답). 이 저장소의 푸시
자격증명이 Jetson 에만 있어 커밋이 막혔고, 그동안 PC 에서 가능한 다음 시행을
이어서 진행했다. 13:00 복귀 후 두 시행을 한 커밋으로 올렸다. 원인 미상.

### ⚠️ 실비행 전 확인할 것 (추가분, 2026-08-15)

- **`pixel_focal_length_px: 530` 은 여전히 미검증.** 시뮬에서는 `SIM_FX` 로
  우회했지만 실기 IMX219 는 측정된 적이 없다. **틀렸다면 실비행에서 위 4번의
  "거의 수직 낙하 → 타겟 프레임 이탈 → 로스트"가 그대로 재현된다.**
  측정은 수평거리 없이 가능하다:
  ```
  fx = 화면상 바구니 폭(px) x 고도(m) / 0.238(m)
  ```
  바구니를 카메라 바로 아래 두고 호버, 고도는 `ALTITUDE.altitude_relative`.
  배포와 같은 640x480 으로 재고 2/3/4 m 에서 반복해 값이 일정한지 확인할 것.
  비행 없이 삼각대로도 가능.
- **실기 TensorRT 경로(`YOLO_MODEL/cpp/trt_engine.cpp`)에 근접 병합이 없다.**
  NMS 만 한다. 근접에서 타겟이 쪼개지는 것은 모델의 성질이므로 실기에서도 같은
  일이 일어날 가능성이 높다. 위 6번과 같은 병합을 C++ 쪽에도 넣어야 한다.
- **실기 카메라도 자기 기체 구조물을 보는지 확인 필요.** 시뮬에서는 카메라 장착
  위치 때문이었다. 실기 장착 위치에서 프레임 모서리에 기체가 들어오는지 한 번
  찍어볼 것. 들어온다면 ROI 크롭(`algorithm.md` 1-2 계획)이 필요하다.
- **카메라 장착 높이 보정.** 시뮬에서 카메라를 기체 원점보다 0.13 m 아래로
  내렸는데 거리 계산은 픽스호크의 기체 고도를 쓴다(고도 4 m 에서 2.7% 계통
  오차). 실기도 카메라가 기체 원점과 다른 높이에 달리면 같은 보정이 필요하다.
- **미션이 GUIDED 로 이륙한다.** `sim_mission_core.py` 는 GUIDED 로 시동·이륙한
  뒤 AUTO 로 전환한다. `hybrid_guidance` 는 "AUTO + armed" 만 보므로 동작에는
  문제가 없지만 실기 운용 절차와는 다르다. 미션 첫 항목을 `NAV_TAKEOFF` 로 두고
  `MAV_CMD_MISSION_START` 로 시작하도록 바꾸는 것이 더 충실하다. (사용자 지적)
- **`gazeboSim/README.md` 의 FOV/px-per-metre 계산 수정 필요** (위 10번).
  2026-08-14 항목에 남아 있던 "`control/README.md`/`gazeboSim/README.md` 의
  Known blocker 섹션 stale" 도 여전히 미해결 - 이번 세션에서 실제로 그
  blocker(`wait_for_lock()` race)가 해소된 것을 재확인했으므로 함께 갱신할 것.
- **고도 8 m 이상에서는 현재 타겟 평면으로 탐지가 안 된다**(위 8번). 상한만
  10 m 로 올려둔 상태이므로, 그 고도를 실제로 쓸 거면 평면 크기를 재조정해야
  한다.
