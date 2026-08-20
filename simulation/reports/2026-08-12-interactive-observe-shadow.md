# Interactive observe / shadow 검증

## 1. 기준 상태

- branch: `Document`
- HEAD: `bf9d99fd259818bf2076944eb58375caf9706905`
- 로컬 remote-tracking `origin/Document`: 같은 commit
- `git fetch origin Document --prune`: DNS `Could not resolve host: github.com`로
  실패했으므로 origin의 서버 최신성은 확인하지 못했다.
- 작업 시작 status: 398줄, 종료 전 status: 405줄. 기존 대규모 CRLF/사용자
  변경은 정리하거나 덮어쓰지 않았다.
- `stash@{0}: On Document: target geometry shadow work`는 조회만 했고
  apply/pop/drop하지 않았다.

## 2. production 호출 경로

환경 의존 입력은 Python `YOLO_MODEL/yolo_live.py::grabber()` 또는 C++
`YOLO_MODEL/cpp/frame_source.cpp`이다. Python은 기존 IMX219
`nvarguscamerasrc`를 유지한다. C++ 기본 `Imx219FrameSource`도 기존 pipeline
문자열을 그대로 소유하며, 명시적 `--frame-source gazebo --gazebo-topic`일 때만
`GazeboFrameSource`가 `gz.msgs.Image`를 BGR frame으로 바꾼다.

알고리즘 경로는 다음과 같다.

1. Python: `torch.hub.load()` → `model(frame[:,:,::-1])` → yolov5 render/pandas,
   또는 C++: `YoloTrt::infer()`의 기존 letterbox/BGR→RGB/TensorRT decode/NMS.
2. 가장 confidence가 높은 detection을 고르고 center 기준 `+x right`, `+y up`으로
   복원한다. 복수 detection이 아니면서 같은 class가 5 frame 연속일 때
   `confirmed=true`가 된다.
3. HTTP `/target` JSON의 `found`, `confirmed`, `age_ms`, `x_px`, `y_px`를
   `target_distance.cpp`가 읽는다. 기본 freshness는 400 ms다.
4. `pixel_offset=hypot(x_px,y_px)`,
   `ground=pixel_offset_to_ground_m(pixel_offset,altitude,focal)`,
   `slant=hypot(ground,altitude)`를 계산해 `TargetRangeMsg`를 loopback UDP로 보낸다.
5. `TargetRangeReceiver` → production `approach_target()`가 slant distance에서
   `stop_distance`를 뺀 값, `k_forward=0.5`, `k_yaw_px=0.006`, alignment와 기존
   clamp/target-loss/altitude 정책을 적용한다.
6. `drone::send_velocity_body()`가 MAVLink msgid 84
   `SET_POSITION_TARGET_LOCAL_NED`, frame 9 `MAV_FRAME_BODY_OFFSET_NED`, type mask
   1991을 packing한다.

환경 adapter 외의 gain, threshold, confirmation, freshness, geometry, guidance,
target-loss와 command packing 파일은 수정하지 않았다. Baseline harness는 production
`approach_target()`과 `drone::send_velocity_body()`를 직접 호출하고 기존 unit
test의 in-memory `FakeTransport` factory override로 bytes를 받는다. 따라서
control main과 OS socket/serial write 없이 최종 packing 결과를 decode한다.

## 3. FrameSource / ShadowCommandSink

- `FrameSource`: `Imx219FrameSource`, `GazeboFrameSource`. 기본은 IMX219이며
  Gazebo topic은 자동 탐색하지 않는다.
- Gazebo validation: 정확히 640x480, `RGB_INT8`, stride 1920, payload
  `stride*height`, protobuf source timestamp를 요구한다. RGB를 BGR로 바꾼 뒤 기존
  production inference 입력에 준다. 상대 topic, 다른 소유권 topic, 잘못된
  format/dimension/stride/payload는 거부한다.
- `ShadowCommandSink`: production command bytes를 in-memory transport에서 decode해
  `shadow_commands.csv`에 기록한다. `allowed=false`,
  `block_reason=SHADOW_MODE`, external socket/serial write count 0이다.
- 향후 SITL flight 경계는 기존 `Transport` factory에 loopback transport를 넣는
  위치다. 이번 launcher의 `--mode flight`는 CTest gate가 닫혀 있어 항상 거부한다.

## 4. Gazebo / SITL 실측

- world: repository-owned
  `simulation/worlds/ensamb_iris_runway.sdf`
- vehicle: `simulation/models/ensamb_with_gimbal/model.sdf`
- camera body: `simulation/models/ensamb_with_standoffs/model.sdf`
- 생성 model: `axes`, `grass_ground`, `ensamb_with_gimbal`, `target_basket`
- topic:
  `/world/ensamb_iris_runway/model/ensamb_with_gimbal/model/ensamb_with_standoffs/link/down_camera_link/sensor/down_camera/image`
- publisher: 유일한 `gz.msgs.Image`
- frame: 640x480, RGB_INT8, stride 1920, source timestamp 13.8 s
- SDF camera pose: `0 0 0 0 1.57079632679 0`, horizontal FOV `2.7507`
- runtime optical axis body: `(0,-0,-1)`
- runtime optical axis world: `(0,-0.000000001,-1)`
- sample:
  `simulation/logs/interactive_shadow_validation_20260812_final/frame_source_sample.ppm`

Sample은 정상적으로 열렸고 640x480이며 min 34, max 116으로 검은 화면이 아니다.
시각 검사에서 하늘과 수평선은 없고 녹색 지면과 기체 하부 형상만 보였다.

## 5. YOLO runtime 현실성

- host: x86_64 WSL
- repository `YOLO_MODEL/cpp/build/yolo_live`: aarch64 ELF
- WSL: OpenCV 4.6은 있으나 CUDA toolkit/TensorRT, torch, ultralytics,
  onnxruntime은 없다. `nvarguscamerasrc`도 없다.
- `prototype.engine`은 이 host에서 deserialize하지 못했으며 production CMake는
  CUDA toolkit 탐색에서 중단됐다.
- `.onnx`와 `.pt`는 존재하지만 대체 runtime을 설치하거나 confidence를 낮추지
  않았다. 따라서 live TensorRT YOLO end-to-end는 **미검증**이다.
- `classes.txt`는 `astro-drone` 한 class다. `target_basket` mesh와 학습 class의
  시각적 대응은 확인되지 않았으므로 basket detection 성공을 주장하지 않는다.

Shadow PASS는 Gazebo frame adapter, production guidance/packing과 write 차단에
대한 결과이며 live YOLO detection PASS를 뜻하지 않는다.

## 6. Baseline 동일성

15 case 결과는 저장소의 고정
`simulation/fixtures/target_guidance_baseline_expected.csv` 및 이전 HEAD baseline과
byte-identical했다. 주요 결과는 다음과 같다.

| case | ground/slant m | clamped vx,vy,vz,yaw | loss |
|---|---:|---|---|
| center | 0 / 3 | 0.5,0,0,0 | TRACKING |
| left/right | 1.358491 / 3.293250 | 0,0,0,-0.6 / +0.6 | TRACKING |
| up/down | 1.018868 / 3.168295 | 0.5,0,0,0 | TRACKING |
| four corners | 2.264151 / 3.758508 | 0,0,0,±0.6 | TRACKING |
| missing | 0 / 0 | 0,0,0,0 | TARGET_MISSING |
| YOLO stale | 0 / 0 | 0,0,0,0 | TARGET_STALE_YOLO |
| link stale | 0.566038 / 3.052933 | 0,0,0,0 | TARGET_LINK_STALE |
| same px, altitude 1/3/5 m | 0.188679/0.566038/0.943396 ground | 0,0,0,+0.48 | TRACKING |

모든 row의 would-be message는 msgid 84, frame 9, type mask 1991이며 실제 external
write는 0이다. 기존 문제인 center altitude 3 m의 vx 0.5, vy 항상 0,
up/down 동일, slant distance 사용도 그대로 재현됐다.

## 7. Mission Planner

기존 설치는 `/home/hyojin/MissionPlanner/MissionPlanner.exe`, Mono 6.8이며 WSLg
환경은 `DISPLAY=:0`, `WAYLAND_DISPLAY=wayland-0`이다. 기존 로그에는
`Open port with UDP14550` 기록이 있다. launcher는 자동 GUI 실행 대신 다음 명령과
UDP port 14550을 출력한다.

```bash
cd /home/hyojin/MissionPlanner
DISPLAY=:0 WAYLAND_DISPLAY=wayland-0 mono MissionPlanner.exe
```

14550 telemetry output은 MAVProxy에 연결돼 있고 Mission Planner가 반환하는
packet은 같은 MAVProxy→audit→SITL 경로를 지난다. Profile/parameter는 변경하지
않았다. 자동 실행에서는 GUI를 띄우지 않았으므로 실제 GUI connection indicator는
수동 확인 항목이다.

## 8. 실행과 검증 결과

```bash
simulation/scripts/interactive_observe_shadow.sh --mode observe
simulation/scripts/interactive_observe_shadow.sh --mode shadow
```

- observe: PASS, HEARTBEAT 12, REQUEST_DATA_STREAM 1, vehicle-affecting 0
- shadow boundaries: PASS, live YOLO `NOT_RUN`, HEARTBEAT 116,
  REQUEST_DATA_STREAM 8, vehicle-affecting 0
- packet audit forbidden set: SET_MODE/DO_SET_MODE, ARM/DISARM, TAKEOFF, LAND,
  local/global setpoint, RC override, MANUAL_CONTROL, parameter write,
  mission write/set-current/start
- upstream `iris_runway` smoke: PASS, audit 0, cleanup 0
- custom world telemetry/down camera/target spawn: PASS
- frame adapter standalone build and relative-topic refusal: PASS
- frame contract test: RGB→BGR 동일 pixel, 640x480, stride, timestamp PASS;
  잘못된 size/format/stride 거부 PASS
- `--mode flight` refusal: PASS
- wrong camera ownership topic refusal: PASS
- control build: PASS
- CTest: 1/3 PASS; 기존 `mav_connection`, `command_characterization` FAIL 유지
- cleanup: tracked PID, relevant process/port 0
- 이번 작업 파일 scoped CRLF-aware `git diff --check`: PASS. 전체 기본
  `git diff --check`는 기존 CRLF와 기존 tracked build log의 trailing whitespace로
  FAIL했으며 해당 사용자 파일을 정리하지 않았다.
- repository 안에 이번 작업의 build/cache artifact 없음. 기존 `gcs/build`와
  `YOLO_MODEL/cpp/build` artifact는 작업 전부터 존재해 보존했다.

## 9. 남은 조건

Flight 활성화 전 최소 조건은 두 known CTest 수정/통과, loopback SITL transport
packet characterization, live production TensorRT-compatible runtime, model class와
simulation target 대응 확인, live observation sidecar timestamp/bbox 연결, shadow와
동일한 packet audit gate 통과다. 그 전까지 flight mode는 닫혀 있다.
