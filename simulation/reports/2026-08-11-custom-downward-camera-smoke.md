# project custom downward-camera smoke report

> 후속 상태: 이 보고서 작성 당시 외부 checkout에만 있던 최소 custom 자산은
> 같은 날 `simulation/worlds`와 `simulation/models`로 바이트 그대로 이관됐다.
> 현재 재현성 검증과 hash는
> `2026-08-11-custom-assets-migration.md`를 기준으로 한다. 아래 외부 의존성
> 설명은 최초 원인 조사 당시의 기록이다.

- 테스트 날짜: 2026-08-11 (Asia/Seoul)
- 저장소 Git commit: `25ab55158e9d2111e3d7c6b7caee377e9daeae4c`
- ArduPilot commit: `ceb710cc7557ef4bd226c7601a7c5eb7fedb4ea2`
- Gazebo/plugin commit: Gazebo 10.4.0 / ardupilot_gazebo
  `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`
- scenario: `simulation/scenarios/custom_downward_camera_smoke.md`
- 결과 로그: `simulation/logs/camera_smoke_20260811_174558/`

## Scenario 분리

- upstream 연결 smoke: `iris_runway.sdf` → `iris_with_gimbal`; Gazebo + SITL +
  audit + MAVProxy + receive-only telemetry
- project custom camera smoke: `ensamb_iris_runway.sdf` →
  `ensamb_with_gimbal` → `ensamb_with_standoffs`; Gazebo-only

`_common.sh`의 upstream 기본값은 변경하지 않았다. `camera_smoke_test.sh`만
`CAMERA_SIM_WORLD=ensamb_iris_runway.sdf`,
`CAMERA_SIM_MODEL=ensamb_with_gimbal`을 자체 기본값으로 사용한다.

이전 camera test는 project용 `ensamb_iris_runway.sdf` / `ensamb_with_gimbal`이
아니라 upstream `iris_runway.sdf` / `iris_with_gimbal::gimbal_small_3d::camera`를
선택했으므로 project camera 기준으로는 잘못된 fixture였다.

## 전체 include와 resource chain

```text
/home/hyojin/ardupilot_gazebo/worlds/ensamb_iris_runway.sdf
├── inline axes, grass_ground, sun, sensors/physics systems
├── model://ensamb_with_gimbal
│   └── model://ensamb_with_standoffs
│       ├── meshes/ensambFinal.STL
│       ├── meshes/iris_prop_ccw.dae
│       ├── meshes/iris_prop_cw.dae
│       ├── imu_link::imu_sensor
│       └── down_camera_link::down_camera
│           ├── CameraZoomPlugin
│           └── GstCameraPlugin
└── model://target_basket
    └── meshes/basket.stl
```

World의 GUI `ImageDisplay`는 `/yolo/annotated`를 표시하도록 선언하지만 headless
camera smoke에서는 GUI와 YOLO publisher를 시작하지 않는다. `grass_ground`는
primitive plane과 색상만 사용하여 texture resource가 없다.

Runtime resource search path는 아직 다음 외부 경로다.

- `GZ_SIM_RESOURCE_PATH=/home/hyojin/ardupilot_gazebo/models:/home/hyojin/ardupilot_gazebo/worlds`
- `GZ_SIM_SYSTEM_PLUGIN_PATH=/home/hyojin/ardupilot_gazebo/build`
- plugin libraries: `libArduPilotPlugin.so`, `libCameraZoomPlugin.so`,
  `libGstCameraPlugin.so`와 Gazebo system plugin packages

`ensamb_with_gimbal/model.config`에는 `gimbal_small_2d` dependency가 남아 있지만
실제 `model.sdf` include에는 없고 runtime resource URI로도 사용되지 않는다.
저장소 이관 시 metadata를 현재 SDF와 일치시키는 검토가 필요하다.

## Camera identity와 runtime 결과

- 실제 생성 vehicle model: `ensamb_with_gimbal`
- camera owner model: `ensamb_with_standoffs`
- link/sensor: `down_camera_link::down_camera`
- SDF sensor pose: `0 0 0 0 1.57079632679 0`
- 실제 image topic:
  `/world/ensamb_iris_runway/model/ensamb_with_gimbal/model/ensamb_with_standoffs/link/down_camera_link/sensor/down_camera/image`
- actual publisher: 1, `gz.msgs.Image`; 다른 Image publisher 없음
- SDF/실제 frame: 640x480, FOV 2.7507 rad, 실제 `RGB_INT8`
- runtime camera link world position: `(0,0,0.038999172)`
- runtime camera link world quaternion `(x,y,z,w)`:
  `(0,0,0.707106781,0.707106781)`
- runtime sensor world quaternion `(x,y,z,w)`:
  `(-0.5,0.5,0.5,0.5)`
- optical axis body: `(0,0,-1)`
- optical axis world: `(0,-0.000000001,-1)`
- 수신: 15 frames / 2.807초, 4.988 FPS, 최초 frame 66.012 ms
- pixel min/max/mean: 34/116/58.533, 전체 검정 아님
- sample:
  `simulation/logs/camera_smoke_20260811_174558/sample.ppm`

직접 시각 검사에서 하늘과 수평선은 전혀 보이지 않았고 초록색 지면이 화면
전체를 차지했다. 지면 위에는 어두운 기체/그림자 형태가 보였다. SDF +90도
pitch가 optical +X를 body -Z로 돌린다는 계산과 runtime axis, 실제 frame이 모두
일치한다.

Topic 선택은 이름의 첫 match를 사용하지 않는다. 실제 topic 목록의 camera/image
후보마다 publisher type을 조회하고, 유일한 `gz.msgs.Image` publisher인지 확인한
뒤 SDF에서 도출한 world/model/link/sensor 소유 경로와 정확히 비교한다. Image
publisher가 0개 또는 여러 개거나 소유권이 다르면 후보 파일을 남기고 실패한다.

## 판정

- project custom downward-camera smoke: `PASS`
- upstream 기본 연결 smoke regression: `PASS`
  - 재실행 ID: `custom_camera_regression_20260811_1747`
  - world/model: `iris_runway.sdf` / `iris_with_gimbal`
  - telemetry overall PASS, armed_seen=false
  - MAVProxy→SITL: HEARTBEAT 32, REQUEST_DATA_STREAM 3
  - vehicle-affecting command 0
- 두 실행 모두 cleanup 후 관련 process, PID file, port 잔여 0

## 저장소 소유로 이관할 최소 목록

이번 작업에서는 복사하지 않았다. 승인된 후 다음 파일만 동일 상대 구조로
`simulation/` 아래에 가져오는 것이 현재 SDF를 재현하는 최소 계획이다.

```text
simulation/worlds/ensamb_iris_runway.sdf
simulation/models/ensamb_with_gimbal/model.config
simulation/models/ensamb_with_gimbal/model.sdf
simulation/models/ensamb_with_standoffs/model.config
simulation/models/ensamb_with_standoffs/model.sdf
simulation/models/ensamb_with_standoffs/meshes/ensambFinal.STL
simulation/models/ensamb_with_standoffs/meshes/iris_prop_ccw.dae
simulation/models/ensamb_with_standoffs/meshes/iris_prop_cw.dae
simulation/models/target_basket/model.config
simulation/models/target_basket/model.sdf
simulation/models/target_basket/meshes/basket.stl
```

현재 model SDF에서 참조하지 않는 `iris.dae`, `iris_collision.stl`,
`ensamb_final`, `ensamb_world.sdf`, runway texture 변경은 이 camera world의 최소
이관 목록에서 제외한다. DAE 두 파일은 외부 image/texture reference가 없다.

이관 후에는 `resolve_world_file`, `resolve_model_file`과
`GZ_SIM_RESOURCE_PATH`가 repository-owned custom 경로를 우선하도록 별도 변경하고,
모든 `model://` URI와 plugin resolution을 다시 검증해야 한다. Plugin source/build와
ArduPilot checkout은 복사 대상이 아니며 계속 명시적인 외부 build dependency로
남는다.

## 외부 checkout 상태

다음 custom 항목은 ardupilot_gazebo commit에 포함되지 않은 untracked 파일이다.

- `worlds/ensamb_iris_runway.sdf`
- `models/ensamb_with_gimbal/`
- `models/ensamb_with_standoffs/`
- `models/target_basket/`

따라서 현재 custom camera scenario는 외부 working tree에 의존하며 checkout을
clean/reclone하면 재현되지 않는다. 본 작업은 외부 파일을 읽기만 했고 삭제,
덮어쓰기, 복사를 수행하지 않았다.
