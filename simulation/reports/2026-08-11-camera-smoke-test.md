# camera_smoke_test report

- 테스트 날짜: 2026-08-11 (Asia/Seoul)
- 저장소 Git commit: `25ab55158e9d2111e3d7c6b7caee377e9daeae4c`
- ArduPilot commit: `ceb710cc7557ef4bd226c7601a7c5eb7fedb4ea2`
- Gazebo/plugin commit: Gazebo `gz sim` 10.4.0 / ardupilot_gazebo
  `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`
- 설정: 외부 `iris_runway.sdf`, `iris_with_gimbal`, headless,
  Gazebo-only fixture, FDM UDP 9002, SITL/MAVProxy/control 미실행
- 외부 checkout 상태: 작업 전부터 `gimbal_small_3d/model.sdf`가 dirty이며,
  commit 기준 FOV 2.0 rad가 현재 사용 파일에서 2.7507 rad로 변경되어 있다.
  본 작업은 현재 사용 파일을 읽기만 했고 외부 checkout을 수정하지 않았다.
- 예상 결과: 실제 `gz.msgs.Image` topic 발견, publisher 존재, 정상 frame 수신,
  sample image 저장, 비검정 화면, tracked process/port 정리
- 실제 결과:
  - 조사한 include 체인: `iris_runway.sdf` → `iris_with_gimbal` →
    `iris_with_standoffs` + `gimbal_small_3d`; 지면은 `runway` model
  - camera sensor: `gimbal_small_3d`의 `pitch_link::camera`, frame id
    `pitch_link`
  - sensor pose: `0 0 0 -1.57 -1.57 0`; gimbal include pose:
    `0 -0.01 -0.124923 90 0 90` degrees
  - SDF에 pixel format은 명시되지 않았고 실제 message는 `RGB_INT8`
  - SDF 설정: 640x480, horizontal FOV 2.7507 rad(157.603501 deg),
    update rate 10 Hz
  - 실제 image topic:
    `/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image`
  - message type `gz.msgs.Image`, publisher 1
  - 최초 frame wall time `2026-08-11T16:35:29.802`, 구독 후 134.700 ms,
    message timestamp 6.200000000 s
  - 2.917초 수신 구간에서 15 frame, 측정 FPS 4.799
  - PPM 640x480 정상 open, pixel min/max/mean 0/255/117.122,
    전체 검정 아님
  - 시각 검사: 수평선이 화면 중앙 부근에 수평으로 놓이고, 위쪽은 하늘,
    아래쪽은 회색 활주로/지면과 기체 그림자이다. 기체/프로펠러 일부가 위쪽
    가장자리에 보인다. 90도 회전이나 상하 반전은 보이지 않는다.
  - optical axis: Gazebo camera의 +X optical axis가 sensor pose에 의해
    `pitch_link` +Z가 되고, gimbal include 회전과 합성하면 기체 +X 수평 전방이다.
    실제 영상의 하늘-수평선-지면 구도도 전방 카메라임을 확인한다.
  - camera smoke `PASS`; default-downward-camera-ready `FAIL` (3축 gimbal의
    명령 없는 기본 자세는 전방 방향)
  - vehicle-affecting MAVLink command 0건; MAVLink process 시작 0개
  - cleanup 후 tracked process/PID file/FDM port 잔여 0
- 판정: camera smoke `PASS`, default-downward-camera-ready `FAIL`
- 관련 로컬 결과:
  - `simulation/logs/camera_smoke_20260811_163508/`
  - sample: `simulation/logs/camera_smoke_20260811_163508/sample.ppm`

후속 runtime 조사에서 기존 `/gimbal/cmd_pitch` position controller에 +90도를
명령하면 실제 optical axis와 영상이 정확히 하향하는 것을 확인했다. 따라서
결정적인 기본 하향 자세가 반드시 필요한 경우에만 별도 고정 fixture가 필요하며,
Gazebo 시작 후 명시적인 gimbal 초기화 명령을 허용할 수 있으면 현재 model을
그대로 사용할 수 있다. 상세 근거는
`2026-08-11-gimbal-direction-investigation.md`에 기록한다.
