# gimbal runtime direction investigation

- 테스트 날짜: 2026-08-11 (Asia/Seoul)
- 저장소 Git commit: `25ab55158e9d2111e3d7c6b7caee377e9daeae4c`
- ArduPilot commit: `ceb710cc7557ef4bd226c7601a7c5eb7fedb4ea2`
- Gazebo/plugin commit: Gazebo 10.4.0 / ardupilot_gazebo
  `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`
- 결과 로그: `simulation/logs/gimbal_direction_20260811_171328/`

## 실행 구성 비교

`camera_smoke_test.sh`와 일반 `smoke_test.sh`는 `_common.sh`와 동일한 현재
로컬 env를 사용한다. 두 실행 모두 정확히 다음 파일과 model을 선택했다.

- world: `/home/hyojin/ardupilot_gazebo/worlds/iris_runway.sdf`
- model: `/home/hyojin/ardupilot_gazebo/models/iris_with_gimbal/model.sdf`
- include: `iris_with_standoffs`, `gimbal_small_3d`
- image topic:
  `/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image`

차이는 camera smoke가 Gazebo-only인 반면 일반 smoke는 이후 ArduPilot SITL,
MAVLink audit relay와 MAVProxy를 연결한다는 점이다. 이번 SITL 비교도 같은
연결 순서를 사용했지만 control/read-only telemetry executable은 시작하지 않았다.
MAVProxy가 보낸 것은 HEARTBEAT 25개와 REQUEST_DATA_STREAM 2개이며 audit의
vehicle-affecting command는 0건이다. 기체는 계속 disarmed였다.

외부 checkout의 `gimbal_small_3d/model.sdf`는 작업 전부터 dirty이며 FOV만
commit의 2.0 rad에서 현재 2.7507 rad로 바뀌어 있다. 외부 파일은 수정하지 않았다.

## joint와 제어 경로

Nested gimbal joint는 다음과 같다. 별도 initial position은 없으므로 명령 없는
초기 목표는 0 rad이고 runtime에서도 세 축이 거의 0으로 측정됐다.

| 기능 | joint | 축 | 물리 limit |
| --- | --- | --- | --- |
| yaw | `gimbal::yaw_joint` | Y | -π .. +π rad |
| roll | `gimbal::roll_joint` | Z | -π .. +π rad |
| pitch | `gimbal::pitch_joint` | X | -π .. +π rad |

외부 `iris_with_gimbal`의 `ArduPilotPlugin`은 channel 8/9/10을 각각
`/gimbal/cmd_roll`, `/gimbal/cmd_pitch`, `/gimbal/cmd_yaw`의
`gz.msgs.Double`로 publish하도록 구성한다. 각 topic은
`gz::sim::systems::JointPositionController`가 subscribe하며 P gain은 2이다.
Plugin source상 PWM이 0이면 `outputReady=false`라서 publish하지 않고, 유효 PWM이
들어오면 `(PWM-min)/(max-min)`에 offset과 multiplier를 적용해 COMMAND topic으로
내보낸다. 현재 SITL 실행에서는 세 COMMAND topic 모두 실제 message가 0개였다.

Outer `JointStatePublisher`는 nested joint가 아니라 고정 결합용 `gimbal_joint`만
내보낸다. 따라서 `dynamic_pose/info`의 gimbal/yaw/roll/pitch link quaternion을
Y→Z→X joint chain의 상대 회전으로 계산해 실제 joint angle을 얻었다.

## runtime 결과

| 조건 | roll | pitch | yaw | optical axis (body) | optical axis (world) |
| --- | ---: | ---: | ---: | --- | --- |
| Gazebo-only final | +0.072207° | -0.082062° | -0.000259° | `(0.9999987, 0.0007904, 0.0014333)` | `(-0.0007904, 0.9999987, 0.0014333)` |
| SITL 안정화 final | +0.021605° | -0.024595° | -0.000097° | `(0.9999996, 0.0007949, 0.0004296)` | `(-0.0007949, 0.9999996, 0.0004296)` |
| pitch +90° 명령 final | +0.061228° | +90.020228° | +0.001992° | `(-0.0003555, 0.0018635, -0.9999982)` | `(-0.0018636, -0.0003524, -0.9999982)` |

Camera link world pose final 값은 다음과 같다. Quaternion 표기는 `(x,y,z,w)`이다.

- Gazebo-only: position `(0.0099974,0.0000030,0.0700360)`, quaternion
  `(0.0004473,0.7066001,0.7076128,-0.0004437)`
- SITL 안정화: position `(0.0099992,0.0000009,0.0700647)`, quaternion
  `(0.0001340,0.7069550,0.7072585,-0.0001331)`
- pitch +90°: position `(0.0100331,0.0171994,0.0920672)`, quaternion
  `(0.0001787,0.9999998,-0.0001764,-0.0005336)`

## 실제 frame 판정

- Gazebo-only sample:
  `simulation/logs/gimbal_direction_20260811_171328/gazebo_only/sample.ppm`
  - 하늘과 수평선이 보이고 지면은 화면 아래쪽을 차지한다.
  - 기체 하부/landing gear와 그림자가 보이며 기본 전방 방향이다.
- SITL 안정화 sample:
  `simulation/logs/gimbal_direction_20260811_171328/sitl_connected/sample.ppm`
  - 하늘과 수평선이 선명하고 지면은 아래쪽이다.
  - Gazebo-only와 같은 전방 방향이며 SITL에 의한 pitch 변경은 없다.
- 기존 interface로 pitch +90°를 준 sample:
  `simulation/logs/gimbal_direction_20260811_171328/commanded_down/sample.ppm`
  - 하늘과 수평선이 없고 지면과 기체 그림자가 화면 전체를 차지한다.
  - optical axis의 body/world Z가 모두 약 -0.999998로 실제 하향이다.

하향 명령은 `2026-08-11T17:16:18.917047938+09:00`에 `gz topic`으로
`/gimbal/cmd_pitch`, `gz.msgs.Double`, `data=1.57079632679`를 한 번 발행했다.
Probe에는 시작 후 2.896480초에 같은 값 1개가 도착했다. 이를 받은 기존
`JointPositionController`가 pitch를 +90.020°로 이동시켰다. 외부 SDF/plugin은
수정하지 않았고 MAVLink 명령도 사용하지 않았다.

## 결론

1. 현재 model은 **고정 하향 카메라 model이 아니다**.
2. 명령 없는 기본 runtime 자세와 현재 SITL 안정화 자세는 모두 수평 전방이다.
3. 현재 관찰에서는 SITL/ArduPilotPlugin이 시작 과정에서 gimbal command를 한 번도
   publish하지 않았으므로 자동 하향 원인은 재현되지 않았다.
4. 하지만 같은 sensor는 **가변 gimbal을 runtime에 +90° pitch로 돌리면 실제로
   하향 촬영할 수 있다**. 사용자가 보았던 지면 전체 영상은 이 상태와 일치하며
   직접 command로 재현됐다.
5. 기존 보고서의 “명령 없는 기본 전방” 판정은 유지한다. 다만 이를 고정 전방
   카메라처럼 해석하거나 하향 사용에 반드시 새 model이 필요하다고 한 결론은
   수정한다.
6. 매 실행마다 결정적인 기본 하향 자세가 필요하면 저장소 소유의 고정 fixture가
   유용하다. 반대로 simulation 시작 절차에서 Gazebo gimbal position command를
   명시적으로 1회 보내고 실제 joint/axis를 검증할 수 있다면 별도 model은 필요 없다.
