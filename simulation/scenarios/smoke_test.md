# Smoke test: world, SITL, plugin, read-only telemetry

## 목적

`iris_runway.sdf`가 시작되고 ArduCopter SITL의 JSON backend가 Gazebo
ArduPilotPlugin에 연결되며, vehicle-affecting 송신 없이 HEARTBEAT와 mode/위치
telemetry를 수신하는지 확인한다.

## 사전조건

- WSL2 Ubuntu 24.04에서 실행
- `simulation/scripts/check_environment.sh`의 core check 통과
- `/home/hyojin/ardupilot/build/sitl/bin/arducopter` 존재
- `/home/hyojin/ardupilot_gazebo/build/libArduPilotPlugin.so` 존재
- `gz sim` 사용 가능
- `/home/hyojin/venv-ardupilot/bin/mavproxy.py` 1.8.74 사용 가능
- 필수 9002/5760/14551과 audit 5770 port가 다른 프로세스와 충돌하지 않음
- 실제 Pixhawk/serial 장치와 실제 기체에 연결하지 않음

## 고정한 버전

- repository: `33a6877fe2862d5bde72822afaf5658d2bcf3604`
- ArduPilot: `ceb710cc7557ef4bd226c7601a7c5eb7fedb4ea2`
- ardupilot_gazebo: `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`
- Gazebo: `gz sim` 10.4.0 (Jetty, gz-sim10)

## 실행 터미널 순서

권장 자동 실행은 아래 한 명령이다. SITL은 SERIAL0 client 연결 전 startup을
기다리므로 자동화는 Gazebo → SITL → audit → MAVProxy 순으로 시작한 뒤 plugin의
`JSON received`를 확인한다.

```bash
# 0. 환경과 build
simulation/scripts/check_environment.sh
simulation/scripts/build_control.sh
ctest --test-dir /tmp/astrodrone-control-sim-build/build --output-on-failure

# 1~10. 시작, 검사, packet audit, 종료/잔류 검사
simulation/scripts/smoke_test.sh
```

GUI가 필요한 경우 `GAZEBO_HEADLESS=0`을 사용한다. headless는 `gz sim -s`로
server만 실행한다.

## 사용 포트

| 용도 | protocol/address | 송수신 주체 |
|---|---|---|
| Gazebo JSON FDM | UDP `127.0.0.1:9002` | plugin bind, SITL peer 자동 감지 |
| router source | TCP `127.0.0.1:5760` | ArduCopter instance 0 server |
| SITL SERIAL1 | TCP `127.0.0.1:5762` | 기존 repository SITL local_tcp와 동일 |
| SITL SERIAL2 | TCP `127.0.0.1:5763` | ArduCopter instance 0 server |
| SITL IRLock | UDP `127.0.0.1:9005` | gazebo-iris 기본 parameter 기능 |
| read-only telemetry | UDP `127.0.0.1:14551` | `read_only_telemetry` bind |
| packet audit relay | TCP `127.0.0.1:5770` | smoke 검증 때만 MAVProxy master |
| control simulation | UDP `127.0.0.1:14550` | 선택, 기본 비활성/미사용 |
| Mission Planner/GCS | UDP `127.0.0.1:14552` | 선택, 기본 비활성/미사용 |

## 안전 금지 목록

ARM, TAKEOFF, GUIDED 전환, velocity/position setpoint, LAND, RC override,
parameter 변경, mission 업로드와 production `control` 실행을 하지 않는다.
`read_only_telemetry`는 MAVLink 송신을 전혀 하지 않는다. MAVProxy는 연결 유지용
GCS HEARTBEAT와 stream 요청을 송신할 수 있지만 비행 영향 명령은 보내지 않는다.
message interval command를
송신하는 `check_alt`, `get_gps_location`, `target_distance`, `emergency`도 이
smoke test에서는 사용하지 않는다.

## 예상 결과

1. Gazebo log에 `iris_with_gimbal`과 FDM `127.0.0.1:9002` plugin 초기화가 보인다.
2. SITL log에 JSON simulator 연결 및 정상적인 scheduler 진행이 보인다.
3. plugin/SITL log에 controller 연결 또는 JSON frame 교환이 보인다.
4. `read_only_telemetry`에 `HEARTBEAT ... custom_mode=... armed=false`가 보인다.
5. fresh하고 유효한 GLOBAL_POSITION_INT 또는 LOCAL_POSITION_NED가 보이며,
   GPS_RAW_INT의 fix 상태도 별도로 기록된다.
6. 어떤 log에도 ARM/TAKEOFF/GUIDED/setpoint/LAND/RC/parameter/mission 송신이 없다.

## 실제 결과 기록 방법

`simulation/reports/README.md`의 형식으로 작은 Markdown 보고서를 만든다.
각 terminal의 로컬 log 이름, plugin 연결 문구, heartbeat/mode, GPS/위치 수신
여부를 기록한다. 확인하지 못한 항목은 `미실행` 또는 `확인 불가`로 표시한다.

2026-08-11 검증 결과는 `PASS`다. MAVProxy가 3건의 `REQUEST_DATA_STREAM`을
보낸 후 최종 run의 첫 HEARTBEAT는 2.186초, 첫 GLOBAL_POSITION_INT와
LOCAL_POSITION_NED는 각각 4.000/4.002초에 도착했다. 30초 창에서 HEARTBEAT
10회, SYS_STATUS 32회, GPS_RAW_INT 32회, GLOBAL_POSITION_INT 32회,
LOCAL_POSITION_NED 32회, ATTITUDE 31회를 수신했다. GPS fix_type 6,
유효한 위경도, fresh 전역/로컬 위치,
armed=false를 확인했다. packet audit의 vehicle-affecting command는 0건이다.
상세 결과는 `../reports/2026-08-11-smoke-test.md`에 있다.

## 종료 방법

```bash
simulation/scripts/stop_simulation.sh
```

스크립트가 기록한 PID만 SIGTERM한다. 남아 있는 외부 프로세스가 있다면 PID와
명령을 직접 확인하고 별도로 판단한다. `pkill -f`를 사용하지 않는다.

## 실패 시 확인할 로그

- Gazebo terminal: plugin load 실패, resource URI, `127.0.0.1:9002` bind 충돌
- SITL terminal와 `$SIM_RUNTIME_DIR/sitl-0`: JSON backend 연결, TCP 5760, parameter/default load
- MAVProxy terminal: TCP master 연결, stream 요청, UDP 14551 output
- shadow terminal: heartbeat timeout, bind 충돌, mode/GPS 수신
- `ss -lntup`: 9002/5760/5770/14551 및 선택 포트 소유자
- `GZ_SIM_SYSTEM_PLUGIN_PATH`, `GZ_SIM_RESOURCE_PATH`
