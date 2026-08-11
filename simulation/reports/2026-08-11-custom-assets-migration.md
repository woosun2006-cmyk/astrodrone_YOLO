# custom downward-camera assets migration

- 검증 날짜: 2026-08-11 (Asia/Seoul)
- 원본: `/home/hyojin/ardupilot_gazebo`의 Git 미추적 custom 자산
- 복사 위치: 저장소 `simulation/worlds`, `simulation/models`
- 외부 checkout 작업: 읽기와 복사 원본 사용만 수행; 수정·삭제·이동 없음

## 최종 include/resource chain

```text
simulation/worlds/ensamb_iris_runway.sdf
├── inline axes, grass_ground, sun, Gazebo systems
├── model://ensamb_with_gimbal
│   └── simulation/models/ensamb_with_gimbal/model.sdf
│       └── model://ensamb_with_standoffs
│           └── simulation/models/ensamb_with_standoffs/model.sdf
│               ├── meshes/ensambFinal.STL
│               ├── meshes/iris_prop_ccw.dae
│               ├── meshes/iris_prop_cw.dae
│               └── down_camera_link::down_camera
└── model://target_basket
    └── simulation/models/target_basket/model.sdf
        └── meshes/basket.stl
```

각 `model.config`는 Gazebo model package metadata로 함께 이관했다. 실제 runtime
SDF에는 `gimbal_small_2d` include/resource URI가 없으므로 model.config에 남은
dependency metadata만으로는 해당 model을 추가 복사하지 않았다. Propeller DAE는
외부 image/texture URI를 참조하지 않는다.

`GZ_SIM_RESOURCE_PATH`는 repository의 `simulation/models`와
`simulation/worlds`를 외부 ardupilot_gazebo보다 먼저 둔다. Camera smoke는
resolved world와 vehicle SDF의 `realpath`가 repository 내부의 기대 경로와
정확히 같고 symlink가 아님을 시작 전에 검사한다.

## 원본/복사본 동일성

아래 size와 SHA-256은 원본과 복사본에서 각각 계산했고 `cmp`도 모두 통과했다.
원본 world의 126행에는 공백만 있는 줄이 하나 있다. 원본과의 byte identity를
보존하기 위해 이를 고치지 않았으며, 따라서 이 untracked vendor SDF만 별도로
`git diff --no-index --check`하면 해당 원본 whitespace를 보고한다. 저장소의 tracked
diff와 새 script/tool/report는 `diff --check`를 통과했다.

| 상대 경로 | bytes | SHA-256 |
|---|---:|---|
| `worlds/ensamb_iris_runway.sdf` | 4,792 | `6d952ece7478d6ce0ff33bdee154e0d53ccd1f8a04ebfe60d2ac1633d2705dec` |
| `models/ensamb_with_gimbal/model.config` | 847 | `dd414f15fedc826adbc827c3d2e6d441679fce2773472ef019fffe17538b214c` |
| `models/ensamb_with_gimbal/model.sdf` | 9,254 | `41487820d6331ada966884b8cd528e2843885ce22e51b9482705756f423dd02b` |
| `models/ensamb_with_standoffs/model.config` | 637 | `8e10eadb3caa05555093b7bce80d6a824375a321ccceddf418df5014557f5e43` |
| `models/ensamb_with_standoffs/model.sdf` | 15,055 | `0466b025c329126a5c3d942a88e334572898d33f0a88b4cc47e93e5ad43b64fa` |
| `models/ensamb_with_standoffs/meshes/ensambFinal.STL` | 12,260,784 | `7861e2d84933688e7c46d01bb9f81a34047dd45f999c867022180da35e18a33b` |
| `models/ensamb_with_standoffs/meshes/iris_prop_ccw.dae` | 308,284 | `5f8b01668ee24a5b663ca8c4dd56e294fd6480db7eec1e5a7c11e45ba9004e99` |
| `models/ensamb_with_standoffs/meshes/iris_prop_cw.dae` | 354,209 | `6cbc686772dccd46253fb65ece000e7ffa6a74b9c097bda349884bd1e78cd879` |
| `models/target_basket/model.config` | 260 | `82e56312730f0023b71e1ca6b6f3498cae7f8b267e8de5e51aea2440cae74ce2` |
| `models/target_basket/model.sdf` | 841 | `c018267c2b37cca5d2a8ce5647709b0a7b1ca67674eaa473164e66efc4173c36` |
| `models/target_basket/meshes/basket.stl` | 314,084 | `9f295d07c9bbe095e8312f0cf73476dac6d2adf372dafc90e9d238c3a02ebe17` |

제외한 `iris.dae`, `iris_collision.stl`, `ensamb_final/`,
`ensamb_world.sdf`, runway texture 변경은 선택한 world/model의 SDF URI에서
참조되지 않는다. Plugin source/build와 ArduPilot checkout도 자산이 아니므로
복사하지 않았다.

## 실행 검증

### Project custom downward-camera smoke

- run ID: `local_assets_20260811_1800`
- 실제 world:
  `/mnt/c/Users/leems/OneDrive/Desktop/astrodrone_YOLO/simulation/worlds/ensamb_iris_runway.sdf`
- 실제 vehicle/body/target SDF: 모두 같은 저장소의
  `simulation/models/ensamb_with_gimbal`, `ensamb_with_standoffs`,
  `target_basket`
- 생성 model: `ensamb_with_gimbal`, `target_basket`
- 유일한 Image publisher:
  `/world/ensamb_iris_runway/model/ensamb_with_gimbal/model/ensamb_with_standoffs/link/down_camera_link/sensor/down_camera/image`
- publisher/message: 1 / `gz.msgs.Image`
- frame: 640x480, `RGB_INT8`, 16 frames, 5.018 FPS, 전체 검정 아님
- optical axis body/world: `(0,0,-1)` / `(0,-0.000000001,-1)`
- sample: `simulation/logs/camera_smoke_local_assets_20260811_1800/sample.ppm`
- 직접 시각 검사: 하늘과 수평선 없음; 초록색 지면과 지상 물체/그림자가
  화면 전체를 차지함
- 결과: PASS; cleanup process/PID/port 0; MAVLink process와
  vehicle-affecting command 0

### Upstream connection smoke regression

- run ID: `local_assets_upstream_regression_20260811_1810`
- 실제 world/model: 외부 기본 `worlds/iris_runway.sdf` /
  `models/iris_with_gimbal/model.sdf`
- telemetry: overall PASS, `armed_seen=false`
- packet audit: vehicle-affecting command 0
- 결과: PASS; cleanup process/PID/port 0

## 남은 외부 의존성

Custom SDF가 지정한 Gazebo plugin binary는 설치된
`/home/hyojin/ardupilot_gazebo/build`에서 읽는다:

- `libArduPilotPlugin.so`
- `libCameraZoomPlugin.so`
- `libGstCameraPlugin.so`

Upstream smoke는 의도적으로 외부 `iris_runway.sdf`와
`iris_with_gimbal`도 사용한다. Repository 내부에는 build/cache/runtime binary를
만들지 않았고, sample/log는 Git ignore된 `simulation/logs`에만 저장했다.
