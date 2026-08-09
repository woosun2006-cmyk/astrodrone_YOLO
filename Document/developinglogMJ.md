# Developing Log (MJ)

## 2026-08-09

### 지금까지 완료 (빌드 확인됨)

1. 기수 방향 제어: control.cpp가 이제 픽셀 오프셋으로 yaw_rate를 계산해 기수를 타겟 쪽으로 돌리고, 정렬될수록 전진속도가 열리는 방식으로 바뀜 (좌우 게걸음 lateral 방식 제거)
2. GPS 검증 로깅: get_gps_location.cpp와 같은 방식으로 GPS_RAW_INT에서 위경도를 읽어, 접근 시작점 대비 실제 이동거리를 CSV/콘솔에 기록 (제어에는 관여 안 함, 검증용)
3. 포트 분리: control.cpp는 14550, target_distance.cpp/emergency.cpp는 14551 사용하도록 setting/port.yaml에서 읽어오게 변경 — 단, 외부에서 돌아가는 MAVLink 프록시(mavproxy 등)도 두 포트로 fan-out하도록 설정을 바꿔줘야 실제로 동작합니다 (이 저장소 안에는 프록시 실행 스크립트가 없어서 확인 못했어요)
4. 거리 계산식 교체: pos_calculator.cpp에 말씀하신 픽셀/530*고도 공식 구현, target_distance.cpp가 고도 무관 고정배율(pixel_to_meter) 대신 이걸 쓰도록 교체
5. 고도 제한이 YOLO 죽으면 같이 죽는" 문제의 원인은 정확히 짚으신 대로입니다: control.cpp는 자기 고도를 직접 안 받고 target_distance.cpp가 UDP로 넘겨주는 last.altitude_m만 보는데, 그마저 link_fresh(500ms 이내 수신) 조건 뒤에 있어서 target_distance.cpp가 죽으면 고도제한 블록 전체가 조용히 꺼집니다. → 해결책은 control.cpp가 check_alt.cpp처럼 자기 MAVLink 연결로 직접 ALTITUDE를 요청/수신하는 것 (별도 작업 리스트에 등록해뒀습니다).
6. yaw_rate 마스크 버그 수정 
	기존 : 0b0000111111000111 - 회전 속도 무시
	이후 :  0b0000011111000111 - 회전 속도 사용
7. target_distance.cpp 고도값 stale 처리: ALTITUDE 못 받아도 마지막 값을 계속 "신선한 척" 보내던 버그 수정 (TargetRangeMsg에 altitude_valid 필드 추가). altitude_stale_ms(기본 500ms) 이상 새 값이 안 오면 altitude_valid=0으로 보고하고, control.cpp는 이걸 받으면 (한 번이라도 유효한 값을 본 적 있다는 전제하에) LOITER로 전환하도록 함 - 고도 데이터가 끊긴 채로 조용히 고도제한 없이 나는 상태를 막음.
8. control.cpp 시동/모드전환 명령 확인: `set_mode("GUIDED")`, `arm_disarm(true)`, `takeoff()`가 전부 "보내고 끝"이던 걸, 실제 HEARTBEAT로 모드/armed 상태가 바뀌었는지 확인하도록 변경 (`wait_for_heartbeat()` 헬퍼 추가, 각 단계 5~10초 타임아웃). 이 확인 과정 중 heartbeat가 3초 이상 끊기면 즉시 예외로 중단.
9. 거리값 이상치 방어: target_distance.cpp에 절대 상한(`max_plausible_distance_m`, 기본 50m)과 직전 값 대비 급점프 감지(`max_distance_jump_m`, 기본 5m) 추가. 둘 중 하나라도 걸리면 그 사이클은 "타겟 미발견"으로 처리해서 튄 값이 속도 명령으로 안 들어가게 함.
10. 타겟 로스트 10초 이상 시 LAND: 기존엔 타겟 놓치면 GUIDED에서 속도 0으로 영원히 제자리 호버만 했음. `target_lost_land_sec`(기본 10초, setting/MAVLink.yaml) 넘게 연속으로 못 찾으면 heartbeat 유실 때와 동일하게 즉시 LAND 후 종료.
	- 5번 항목의 "고도 자기 직접 수신" 아키텍처 변경(control.cpp가 check_alt.cpp처럼 자기 MAVLink 연결로 직접 ALTITUDE 요청)은 이번에 안 함 - 7번으로 급한 증상(stale 값 방치)은 막았지만, target_distance.cpp 의존성 자체는 아직 남아있음.

### TensorRT 변환 + fps 관리

11. YOLO_MODEL/prototype.pt → TensorRT 엔진 변환. 처음 FP16으로 빌드(약 41fps, .pt 대비 2.1배)했다가, yolo_live.py에 이미 있던 주석(`HALF = False # info.txt: FP16 collapsed accuracy on the dev GTX 1660 SUPER (mAP50 0.995 -> 0.659). Unverified on this Jetson's GPU`)을 발견해서 FP32로 재빌드함 (정확도 리스크 회피, 그래도 약 37fps로 .pt(~19.6fps) 대비 1.9배). yolo_live.py의 WEIGHTS를 `prototype.engine`으로 교체 - DetectMultiBackend가 확장자로 자동 판별해서 기존 `torch.hub.load(..., "custom", ...)` 호출 코드는 안 바꿔도 됨.
	- YOLO_MODEL/cpp/에 이미 진행 중이던 별도 C++ 포팅(trt_engine.cpp 등)을 뒤늦게 발견함 - `prototype.onnx`를 그쪽이 직접 참조하고 있어서 삭제하면 안 됨 (TensorRT 빌드 중간 산출물처럼 보이지만 실제로는 의존성 있는 파일).
12. yolo_live.py의 inferer() fps 상한 + setting/rate.yaml 신설: inferer()가 속도 제한 없이 카메라 최대치(30fps)까지 무조건 돌던 걸 20fps로 캡 (target_distance.cpp가 10Hz로만 소비하는데 그 이상은 GPU/발열만 낭비). target_distance.cpp의 sense_cycle_ms(10Hz)와 이 infer_max_fps(20)가 각자 다른 파일(MAVLink.yaml / yolo_live.py 상수)에 따로 있던 걸 setting/rate.yaml로 통합해서 C++/Python 둘 다 여기서 읽도록 변경.

### git 저장소 손상 발견 (미해결)

13. `git status`/`git log`/`git commit`/`git fetch`가 전부 실패하는 상태 발견: Document 브랜치 HEAD가 가리키는 커밋 오브젝트(`81b6b6e0...`)가 0바이트로 깨져 있음. `.git/objects`에 총 26개의 0바이트 오브젝트가 있고 전부 생성 시각이 2026-08-08 09:58 두 시점(14초 간격)에 몰려있어, 그날 `git commit` 도중 뭔가에 의해 중단된 것으로 추정 (오늘 작업과는 무관 - 그 이전부터 있던 문제). reflog는 멀쩡해서 최근 커밋 이력(`0808(3)`까지)은 보이지만, "0809(1)"이라는 이름의 커밋은 로컬 어디에도 없음 - git commit 자체가 막혀 있어 정상적으로는 생성될 수 없는 상태. origin(GitHub) 원격은 연결돼 있으나 fetch도 로컬 손상 때문에 막혀서 원격에 있는지 확인 못함. 복구(마지막 정상 커밋 `0808(3)`으로 브랜치 되돌리기 등)는 아직 착수 안 함 - 사용자 확인 후 진행 예정.

### 고쳐야 할 문제점

비슷한 물체 두 개를 못 구분하는 문제

지금 yolo_live.py가 프레임마다 "타겟 하나"를 새로 골라서 /target에 보고하는 구조라면(픽셀 좌표+found+confirmed만 있고 지속 ID가 없어 보임), 진짜 원인은 대부분 두 가지 중 하나입니다.

1. 탐지 자체는 되는데 "어느 게 내 타겟인지" 매 프레임 새로 고르는 문제 — 이 경우가 훨씬 흔하고 고치기 쉬움. 예: 프레임 중심에 더 가까운 박스를 고르는 방식이면, 두 유사 물체가 화면에서 위치가 바뀔 때마다 타겟이 순간이동(ジャンプ)함. 해결: 최초 락온한 박스의 위치를 계속 추적해서, 다음 프레임에서도 "이전 위치와 가장 가까운/겹치는" 박스를 우선 선택 (IoU 매칭 또는 간단한 centroid+속도 예측 트래커, SORT류). 재학습 없이 yolo_live.py에 30줄 정도 트래커 로직만 추가하면 되는 수준.
2. 정말로 픽셀상 구분이 안 되는 두 물체(생김새가 동일) — 이건 detector가 클래스만 보고 개체는 구분 못 하는 근본적 한계라, 위 방법(1)으로 "먼저 잡은 애를 계속 따라가기"까지가 현실적 최선입니다. 그 이상 확실히 구분하려면 타겟에 물리적 마커(색 테이프, AprilTag) 부착하거나, 외형 임베딩 기반 재식별(작은 ReID 모델)을 추가해야 하는데 이건 작업량이 꽤 큽니다.

우선순위상 (1) 트래커 방식부터 넣어보는 걸 추천드려요. 지금 겪는 증상(두 개를 못 구분)이 "타겟이 자꾸 바뀐다"는 쪽이면 이걸로 대부분 해결됩니다.


### gcs 프로그램 제작 해야 함 !!


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
