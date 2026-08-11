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
