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
- world/model: 제공 예제 `iris_runway.sdf` / `iris_with_gimbal`
- SITL: ArduCopter frame `gazebo-iris`, model `JSON`

외부 checkout에는 조사 당시 사용자 변경과 미추적 파일이 있다. 이 환경은
그 파일을 수정하지 않으며, 기본 예제 world/model만 읽는다.

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

수동 foreground 실행 순서는 다음과 같다.

```bash
# 터미널 1 (GUI, WSLg가 없으면 GAZEBO_HEADLESS=1 추가)
simulation/scripts/start_gazebo.sh

# 터미널 2
simulation/scripts/start_sitl.sh

# 터미널 3: 설치된 MAVProxy를 자동 탐지한다.
simulation/scripts/start_router.sh

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
             +-- MAVProxy --> udp 127.0.0.1:14551  read-only telemetry
                         \-> 14550 control / 14552 GCS (선택, 기본 비활성)
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
control consumer가 그 자리를 대신할 수 있다. GCS/Mission Planner는 기본 smoke에
필수가 아니며, 여러 consumer가 동시에 필요할 때만 MAVProxy fan-out을 추가한다.

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
