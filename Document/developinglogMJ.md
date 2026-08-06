# Developing Log (MJ)

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
