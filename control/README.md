# control

제어 프로그램 저장소 (C/C++)

## 추가됨 (2026-08-11) - motor_test
- `motor_test`: 지상 모터 테스트 도구. `MAV_CMD_DO_MOTOR_TEST`로 모터 1개를
  골라 스로틀 5%로 돌리기 시작하고, 키보드로 스로틀을 올리고 내릴 수 있다.
  **프로펠러를 반드시 제거하고 사용할 것.**
  - 조작: `w`/`+` 증가, `s`/`-` 감소, `0` 즉시 정지, `1`-`--motor-count`
    숫자키로 해당 모터 즉시 전환(코드는 최대 9까지 받지만
    `--motor-count`보다 큰 숫자는 무시됨 - 기본 4모터 기체면 `5`-`9`는
    반응 없음), `n`/`p` 다음/이전 모터, `a` 전체 모터 자동 순환 켜기/끄기
    (`--dwell`초씩, 기본 3초), `q` 종료(정지 후 종료). 스텝은 `--step`
    (기본 5%). `--all`을 주면 시작부터 자동 순환 모드로 켜진다.
  - `t`(또는 `--together`)로 모터 1..motor_count를 동시에 돌리는 모드도
    켤 수 있다. `MAV_CMD_DO_MOTOR_TEST` 자체엔 "N개 동시" 파라미터가 없지만,
    ArduCopter 구현이 커맨드가 올 때마다 그 모터 채널에만 값을 쓰고 다른
    채널은 그대로 두면서 전체 세션의 데드맨 타이머만 갱신하는 점을 이용해,
    전체 모터에 대해 아주 짧은 간격으로 계속 돌려가며 보내 사실상 동시
    회전을 흉내낸다. 이건 ArduCopter 구현 세부사항에 기대는 방식이라
    펌웨어 버전에 따라 다르게 동작할 수 있음 - 동시에 안 도는 것처럼
    보이면 즉시 중단할 것. `a`(순차 자동 순환)와는 상호 배타적이고,
    `--all`+`--together`를 함께 주면 시작 시 거부한다. 동시 회전은 여러
    모터가 한꺼번에 돌아 합산 추력이 커지므로 고정을 더 단단히 해야 한다.
  - 시작 전 이미 armed 상태면 거부하고, 사람이 직접 "no prop"을 입력해야
    진행된다 (대소문자 무관). 영문 문구인 이유는 일부 터미널에서 한글
    입력이 먹지 않는 경우가 있어서.
  - 매 명령에 짧은 타임아웃(`MAV_CMD_DO_MOTOR_TEST`의 param4, 1초)을 실어
    반복 전송한다 - 프로그램이 죽거나 링크가 끊겨도 그 시간 안에
    ArduPilot이 스스로 모터를 멈추는 데드맨 스위치.
  - 접속 주소는 `setting/MAVLink.yaml`의 `real.proxy_udp.address` +
    `setting/port.yaml`의 `mavlink_motor_test`(14553) 포트를 기본값으로
    쓴다 - `mavlink_control`(control.cpp 전용)과 분리된 자체 포트라서 둘이
    동시에 떠 있어도 UDP bind 경합이 나지 않는다. `mavlink_proxy`가 이
    포트로도 fan-out해주므로, `motor_test` 실행 전에 `mavlink_proxy`가
    떠 있어야 한다. `--address`로 덮어쓸 수 있다.
  - 예: `./motor_test --motor 1 --motor-count 4 --step 5`
  - 예(전체 모터 순차 자동 순환): `./motor_test --all --dwell 3`
  - 예(전체 모터 동시 회전): `./motor_test --together`
  - `scripts/motor_test.sh`가 빌드+실행을 대신해준다 (인자는 그대로
    전달됨). `health_check.sh`/`full_mission.sh` 같은 자동 스크립트에는
    절대 넣지 않음 - 프로펠러 안전 확인 후 사람이 직접 돌리는 도구.

## 추가됨 (2026-08-11)
- `control --auto-intercept`: 픽스호크가 (외부에서 업로드된) AUTO 미션을
  자체적으로 날고 있는 상태를 지켜보다가, YOLO 타겟이 `lock_confirm_sec`
  이상 안정적으로 잡히면 GUIDED로 가로채 `approach_target()`으로 접근하고,
  타겟을 놓치거나 `intercept_approach_duration_sec`이 지나면 다시 AUTO로
  돌려주는 흐름. 이 프로그램은 arm/이륙을 직접 하지 않음 (기존
  플래그 없는 실행과 그 부분이 다름). `approach_target()`에
  `resume_mode` 파라미터를 추가해 구현 - 자세한 흐름은 control.cpp의
  `run_auto_intercept()` 주석 참고. `scripts/full_mission.sh`가 이 플래그로
  health_check + yolo + target_distance + control을 한 번에 띄운다.
  새 설정 키(`lock_confirm_sec`/`intercept_approach_duration_sec`/
  `reacquire_cooldown_sec`)는 `setting/MAVLink.yaml`의 `target_track`에.

## 해결됨 (2026-08-09)
- `target_distance.cpp`의 픽셀 오프셋 -> 실거리(`ground_offset_m`) 변환을
  고도 무관 고정 배율(`pixel_to_meter`) 방식에서, 고도를 반영하는 핀홀
  카메라 모델(`pos_calculator.cpp`의 `pixel_offset_to_ground_m()`,
  `target_track.pixel_focal_length_px`)로 교체함. `pixel_focal_length_px`
  기본값 530은 임시값이므로 실제 비행 전 알려진 거리/고도에서 캘리브레이션
  필요.
