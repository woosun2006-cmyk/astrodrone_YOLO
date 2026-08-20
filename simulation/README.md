# ArduPilot SITL + Gazebo simulation

이 디렉터리는 WSL Ubuntu 24.04에서 외부 ArduPilot과
`ardupilot_gazebo` checkout을 참조하는 실행 기반이다. 두 외부 프로젝트나
Linux build 결과를 이 저장소로 복사하지 않는다. 실제 Pixhawk, serial 장치,
실제 기체, production control main은 사용하지 않는다.

## 조사 기준 환경

- 저장소: branch `Document`, commit
  `33a6877fe2862d5bde72822afaf5658d2bcf3604`
- WSL: Ubuntu 24.04.4 LTS, WSL2
- ArduPilot: `/home/hyojin/ardupilot`, commit
  `ceb710cc7557ef4bd226c7601a7c5eb7fedb4ea2`
- ardupilot_gazebo: `/home/hyojin/ardupilot_gazebo`, commit
  `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`
- Gazebo: `gz sim` 10.4.0, Gazebo Jetty packages
- plugin: `/home/hyojin/ardupilot_gazebo/build/libArduPilotPlugin.so`,
  `libgz-sim.so.10` 사용
- upstream 연결 smoke world/model: 제공 예제 `iris_runway.sdf` /
  `iris_with_gimbal`
- project camera smoke world/model: 저장소 소유 custom
  `simulation/worlds/ensamb_iris_runway.sdf` / `ensamb_with_gimbal` /
  `ensamb_with_standoffs::down_camera`
- SITL: ArduCopter frame `gazebo-iris`, model `JSON`

외부 checkout에는 조사 당시 사용자 변경과 미추적 파일이 있다. 이 환경은
그 파일을 수정하지 않는다. upstream 연결 smoke는 외부 기본 예제를 읽고,
project camera smoke는 `simulation/worlds`와 `simulation/models`로 이관한
저장소 소유 custom 자산을 읽는다.

## 설정 우선순위

스크립트는 다음 순서로 값을 선택한다.

1. 호출 전에 명시적으로 export한 환경변수
2. Git에서 제외된 `simulation/env/simulation.env`
3. `_common.sh`의 안전한 기본값

로컬 설정이 필요하면 다음처럼 만들고 필요한 값만 바꾼다.

```bash
cd /mnt/c/Users/leems/OneDrive/Desktop/astrodrone_YOLO
cp simulation/env/simulation.env.example simulation/env/simulation.env
```

`HOME`은 어느 스크립트에서도 재정의하지 않는다. 기본 build와 runtime은
각각 `/tmp/astrodrone-control-sim-build`,
`/tmp/astrodrone-simulation-runtime`을 사용한다.

## 실행

모든 명령은 WSL 터미널의 저장소 root에서 실행한다. 먼저 읽기 전용 검사를
수행한다.

```bash
simulation/scripts/check_environment.sh
simulation/scripts/build_control.sh
ctest --test-dir /tmp/astrodrone-control-sim-build/build --output-on-failure
```

전체 기본 smoke test는 다음 한 명령으로 실행한다. headless Gazebo, SITL,
MAVLink audit relay, MAVProxy, 수신 전용 진단을 순서대로 시작하고 추적한
프로세스만 종료한다.

```bash
simulation/scripts/smoke_test.sh
```

프로젝트의 고정 하향 Gazebo camera transport를 확인할 때는 다음 Gazebo-only
smoke를 사용한다. 이 script는 일반 smoke의 upstream 기본값을 변경하지 않고
자체적으로 저장소의 `ensamb_iris_runway.sdf`, `ensamb_with_gimbal`,
`ensamb_with_standoffs::down_camera_link::down_camera`를 선택한다.
SITL, MAVProxy, control, YOLO를 시작하거나 MAVLink packet을 송신하지 않는다.
실행 때마다 모든 topic의 publisher type을 조회하고, SDF include chain에서
도출한 model/link/sensor 소유권과 유일한 `gz.msgs.Image` publisher가 일치할
때만 첫 정상 frame을 PPM으로 저장한다. Image publisher가 0개 또는 여러 개면
임의 선택하지 않고 후보를 기록한 뒤 실패한다. runtime pose로 optical axis가
body -Z인지도 검증한다. 수신 도구는 설치된 `gz-transport`와 `gz-msgs`를
사용하며 `/tmp` 아래에만 build된다.

```bash
simulation/scripts/camera_smoke_test.sh
```

## Canonical simulation launcher

모든 표준 시뮬레이션은 `simulation/scripts/run_simulation.sh` 하나를 사용한다.
관찰은 `observe`, C++ YOLO와 제어 계산 검증은 `shadow`, loopback SITL 명령 검증은
명시적인 `sitl-flight` 프로파일로 구분한다. 대화형·고정 표적 전용 launcher는
표준 경로에서 제거했다.

```bash
simulation/scripts/run_simulation.sh \
  --profile sitl-flight \
  --simulation-profile diagonal-approach-validation \
  --status-monitor \
  --upload-mission \
  --arm
```

실제 YOLO 검증은 `scenario=default`에서 수행한다. `target-centered-hold`와
`--fixed-basket`는 기본 경로가 아니며, 고정 입력이 필요한 별도 단위 테스트에서만
명시적으로 사용할 수 있다.

Mission Planner는 telemetry 관찰에만 사용한다. ARM, mode 변경, mission
upload/start, parameter write, RC override를 하지 않는다. 두 public launcher 모두
`--mode flight`를 항상 거부한다. `mav_connection`과
`command_characterization` CTest가 통과하기 전에는 SITL command transport를
활성화하지 않는다.

Audit dependency 검사는 simulator를 시작하기 전에 동기적으로 한 번 수행한다.
기본 `MAVLINK_AUDIT_READY_TIMEOUT=60` 안에서 launcher는 audit wrapper 생존,
`AUDIT_LISTEN tcp:127.0.0.1:5770` 로그 marker와 실제 TCP LISTEN을 모두 확인한
뒤에만 MAVProxy를 시작한다. 실패는 dependency validation, process early exit,
bind/port collision, listen-log-without-socket, readiness timeout으로 구분된다.
단계별 시간은 각 run의 `startup_timing.log`와 `audit.log`에 기록된다.

Production C++ YOLO의 기본 source는 기존 IMX219 pipeline이다. 명시적으로
`--frame-source gazebo --gazebo-topic <정확한-topic>`을 준 build만 Gazebo
`gz.msgs.Image`를 받는다. Adapter는 640x480 RGB_INT8, stride 1920, payload와
source timestamp를 검사해 BGR 입력으로 변환한 뒤 기존 `YoloTrt::infer()`에
넘긴다. topic discovery나 첫 camera 자동 선택은 없다.

현재 shadow/sitl-flight 런처는 WSL x86_64에서 `yolo_live`를 로컬 빌드하고,
Gazebo topic과 `YOLO_MODEL/best_v5_wsl.engine`을 사용해 TensorRT inference를
실행한다. Jetson Nano의 aarch64/TensorRT runtime과 동일한 성능을 의미하지는
않지만, frame adapter부터 `/target`, target-distance, guidance까지의 실제
inference 경로를 검증한다. confidence를 낮추거나 ONNX/PT 대체 runtime으로
detection을 조작하지 않는다.
로그는 `simulation/logs/interactive_<mode>_<timestamp>/`에 저장된다.

camera scenario만 다른 외부 구성을 시험하려면 일반 `SIM_WORLD`/`SIM_MODEL`과
혼동하지 않도록 `CAMERA_SIM_WORLD`, `CAMERA_SIM_MODEL`, `CAMERA_BODY_MODEL`,
`CAMERA_LINK`, `CAMERA_SENSOR`를 명시한다. 기본 연결 smoke는 계속
`SIM_WORLD=iris_runway.sdf`, `SIM_MODEL=iris_with_gimbal`을 사용한다.

결과는 Git에서 제외된
`simulation/logs/camera_smoke_<timestamp>/`에 topic 목록, publisher 정보,
frame 통계, sample image와 cleanup 요약으로 저장된다.

## canonical launcher의 audit 사용 범위

`simulation/scripts/run_simulation.sh`는 프로필에 따라 audit 경로를 선택한다.

| 프로필 | SITL과 router 경로 | audit 사용 | 관찰·검증 결과 |
|---|---|---|---|
| `observe` | `tcp:127.0.0.1:5760` → MAVProxy/router → UDP 14550, 14551, 14552 | 사용하지 않음 | 수신 전용 `telemetry_probe.json`에서 control/telemetry HEARTBEAT를 측정 |
| `shadow` | SITL 5760 → audit 5770 → MAVProxy/router | 필수 | `packet_audit.json`, ShadowCommandSink, command 차단 결과를 검증 |
| `sitl-flight` | SITL 5760 → audit 5770 → MAVProxy/router | 필수 | loopback SITL 명령 packet과 CommandGate 결과를 검증 |

heartbeat 진단과 저부하 검증은 다음처럼 별도 simulation profile을 명시한다.

```bash
env -u DISPLAY -u WAYLAND_DISPLAY \
  simulation/scripts/run_simulation.sh \
    --profile observe --heartbeat-diagnostic \
    --simulation-profile control-validation --headless --duration 60
```

`control-validation`은 소스 SDF나 STL을 수정하지 않으며 production guidance의
`max_forward_speed=0.5m/s`와 `max_descent_speed=0.2m/s`를 그대로 사용한다. 실행 시 `/tmp`에
`ensamb_with_gimbal`과 `ensamb_with_standoffs`의 임시 overlay를 만들고 다음만
simulation 범위에서 바꾼다.

- IMU update rate: source `1000Hz` → validation `250Hz`
- ArduPilot Gazebo plugin: source `lock_step=1` → validation `lock_step=0`
- camera update rate: `10Hz` 유지
- world, model 이름, `ensambFinal.STL`, `down_camera` topic, `GstCameraPlugin` 유지
- headless에서는 `gz sim -s`를 사용해 GUI client와 `ImageDisplay`를 실행하지 않음

따라서 이 profile의 heartbeat/RTF 결과는 실제 기체의 IMU 주기나 lockstep 동작을
검증하는 값이 아니라, WSL에서 YOLO/control 경로를 안정적으로 검증하기 위한
simulation transport 결과다. 일반 fidelity 비교는 `--simulation-profile default`
로 별도 실행한다. `--heartbeat-diagnostic`은 observe에서만 audit relay를
추가하며, 일반 observe의 기본 직접 telemetry 경로는 바꾸지 않는다.

2026-08-16 검증에서는 `control-validation`으로 observe 60초를 실행해
SITL 입력과 UDP 14550/14551의 유효 ArduPilot HEARTBEAT 최대 간격을 각각
약 `1.004s`로 확인했다. shadow 30초에서는 TensorRT YOLO readiness와
target-distance/control 시작, ShadowCommandSink, vehicle command 0건을
확인했다. sitl-flight는 mission upload/AUTO/ARM 후 GUIDED와 velocity guidance를
수행했고, target loss 10초 후 LOITER 호버링 정책으로 control이 정상 종료했다.

`observe`의 telemetry probe는 UDP 14550과 14551에 바인드해 패킷을 받기만
하며 송신하지 않는다. ArduPilot HEARTBEAT는 `msgid=0`, `sysid=1`,
`compid=1`, `autopilot=ARDUPILOTMEGA`를 모두 만족하는 경우에만 집계한다.
GCS HEARTBEAT, telemetry 일반 메시지, `BAD_DATA`는 HEARTBEAT 통계에서
제외한다.

`shadow`와 `sitl-flight`의 audit relay는 `simulation/tests`에 있는 검증용
TCP 중계기다. production control이나 MAVLink transport의 필수 의존성이
아니며, relay가 실행되지 않거나 SITL 방향에서 유효한 ArduPilot HEARTBEAT를
파싱하지 못하면 해당 프로필은 성공으로 표시하지 않는다. 패킷 방향은 다음과
같이 기록된다.

```text
SITL_AUTOPILOT_TO_MAVPROXY
MAVPROXY_TO_SITL_AUTOPILOT
```

audit 결과는 `simulation/logs/run_<profile>_<timestamp>/packet_audit.json`,
observe 직접 측정 결과는 같은 디렉터리의 `telemetry_probe.json`에 저장된다.

Gazebo resource lookup은 항상 다음 순서다.

1. `simulation/models`, `simulation/worlds`
2. `$ARDUPILOT_GAZEBO_DIR/models`, `$ARDUPILOT_GAZEBO_DIR/worlds`
3. 호출 전에 존재한 추가 `GZ_SIM_RESOURCE_PATH`

따라서 외부 checkout에 같은 이름의 model이 있어도 project custom 자산이 먼저
선택된다. Camera smoke는 resolved world와 vehicle SDF가 실제 repository 내부
파일인지 시작 전에 검사하고, summary 첫 부분에 world, vehicle/camera/target SDF
절대경로를 기록한다.

Custom model은 외부 checkout의 build plugin을 계속 사용한다. 기본 설치 위치는
`$ARDUPILOT_GAZEBO_DIR/build`이며 `libArduPilotPlugin.so`,
`libCameraZoomPlugin.so`, `libGstCameraPlugin.so`가 필요하다. Script가 이 경로를
`GZ_SIM_SYSTEM_PLUGIN_PATH` 앞에 추가한다. Build 결과나 plugin binary는 이
저장소로 복사하지 않는다.

수동 foreground 실행 순서는 다음과 같다.

```bash
# 터미널 1 (GUI, WSLg가 없으면 GAZEBO_HEADLESS=1 추가)
simulation/scripts/start_gazebo.sh

# 터미널 2: SITL과 bundled MAVProxy를 함께 시작한다.
simulation/scripts/start_sitl.sh

# start_sitl.sh가 control=14550, telemetry=14551, GCS=14552를 연다.
# 별도 start_router.sh는 실행하지 않는다.

# 터미널 4
simulation/scripts/run_control_shadow.sh

# 종료 터미널
simulation/scripts/stop_simulation.sh
```

`run_control_shadow.sh`는 `simulation/tests/read_only_telemetry.cpp`로 만든
수신 전용 executable만 호출한다. 이 파일에는 네트워크/MAVLink 송신 코드가
없고 메시지별 수신 횟수, source, freshness, GPS fix와 전역/로컬 위치를 출력한다.
기존 `check_link`도 송신하지 않지만 위도/경도를 출력하지 않아 smoke의 위치
확인에는 이 전용 도구를 사용한다. `check_alt`, `get_gps_location`, `target_distance`,
`emergency`는 진단용이라는 설명과 달리 message interval command를 송신하므로
이번 smoke test에서 사용하지 않는다. `control`은 시작 직후 GUIDED 전환,
ARM, TAKEOFF를 거쳐 velocity와 LAND를 송신하므로 wrapper에서 호출할 수 없다.

## endpoint와 포트

```text
Gazebo ArduPilotPlugin  udp bind 127.0.0.1:9002
             ^ JSON FDM 자동 응답 peer
ArduCopter SITL        tcp server 127.0.0.1:5760 (instance 0)
             |
             +-- bundled MAVProxy --> udp 127.0.0.1:14551 telemetry
                                  \-> 14550 control / 14552 GCS
```

instance `N`의 SITL TCP port는 ArduPilot `sim_vehicle.py`가 사용하는
`5760 + 10*N`이다. SITL은 instance 0에서 SERIAL1 TCP 5762, SERIAL2 TCP
5763과 IRLock UDP 9005도 연다. 기존 `setting/MAVLink.yaml`의
`tcp:127.0.0.1:5762`는 이 SERIAL1 endpoint이며, MAVProxy source에는 SERIAL0
5760을 사용한다. 같은 파일의 `udp:0.0.0.0:14561` remote endpoint는
loopback-only 정책에 맞지 않아 사용하지 않는다. `setting/port.yaml`의 control
14550/sensor 14551은 유지하고 GCS는 별도 14552로 분리한다. UDP 소비자는 서로
다른 bind port를 사용한다. simulation
endpoint는 모두 loopback으로 검증되며 `/dev/tty*`, serial 문자열과 비-loopback
주소는 거부된다. read-only endpoint는 연결 점검용이며 실제 알고리즘 시험에서는
control consumer가 그 자리를 대신할 수 있다. GCS/Mission Planner는 UDP 14552를
사용한다. 패킷 감사 smoke처럼 외부 MAVProxy가 필요할 때만
`SITL_MAVPROXY_MODE=external`로 SITL을 시작한 뒤 `start_mavlink_audit.sh`와
`start_router.sh`를 별도로 실행한다.

## MAVProxy와 read-only의 의미

MAVProxy 1.8.74는 `/home/hyojin/venv-ardupilot/bin/mavproxy.py`에 설치되어 있다.
기존 검사는 PATH만 확인했기 때문에 이를 놓쳤다. 현재 탐지 순서는 명시적
`MAVPROXY_BIN`, PATH의 `mavproxy.py`, 알려진 venv 후보 순이며 executable,
version, 같은 Python의 `MAVProxy`/`pymavlink` import를 검증한다.
`mavlink-routerd`는 설치되지 않았고 기본 smoke에 필요하지 않다.

`read_only_telemetry`는 vehicle-affecting 명령을 포함해 어떤 packet도 송신하지
않는다. MAVProxy는 GCS heartbeat와 `REQUEST_DATA_STREAM`을 SITL로 보낼 수 있지만,
이번 최소 모듈 설정의 packet audit에서 SET_MODE, ARM/DISARM, TAKEOFF, LAND,
setpoint, RC/MANUAL_CONTROL, parameter/mission 변경은 모두 0건이었다.

최초 실패는 TCP 5760에 수신기만 연결하여 stream을 요청하지 않은 것이 원인이다.
또한 SITL은 SERIAL0 TCP client가 연결될 때까지 startup을 기다렸다. MAVProxy가
연결되고 `REQUEST_DATA_STREAM`을 보낸 뒤 최초 유효 위치는 약 3.21초에 도착했다.

## 파일과 로그 정책

- `simulation/logs/`, `simulation/videos/`, `simulation/runtime/`, 로컬 env는
  Git 제외 대상이다.
- 기본 PID/SITL state는 `/tmp/astrodrone-simulation-runtime`에 저장된다.
- build 결과는 `/tmp/astrodrone-control-sim-build/build`에만 생성된다.
- 작은 결과 요약만 `simulation/reports/`에 추가한다.
- `stop_simulation.sh`는 PID, `/proc` 시작 시각, 예상 command를 모두 확인한
  뒤 이 스크립트가 기록한 프로세스만 SIGTERM한다. `pkill`은 사용하지 않는다.

상세 smoke 절차와 아직 수행하지 않은 항목의 기록 상태는
[`scenarios/smoke_test.md`](scenarios/smoke_test.md)를 따른다.
