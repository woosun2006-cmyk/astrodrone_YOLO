# smoke_test report

- 테스트 날짜: 2026-08-11 (Asia/Seoul)
- 저장소 Git commit: `33a6877fe2862d5bde72822afaf5658d2bcf3604`
- ArduPilot commit: `ceb710cc7557ef4bd226c7601a7c5eb7fedb4ea2`
- Gazebo/plugin commit: Gazebo `gz sim` 10.4.0 / ardupilot_gazebo
  `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`
- scenario: `simulation/scenarios/smoke_test.md`
- 설정: `iris_runway.sdf`, `iris_with_gimbal`, `gazebo-iris`, JSON,
  headless, instance 0, speedup 1, FDM UDP 9002, SITL TCP 5760,
  audit TCP 5770, read-only UDP 14551, timeout 30초
- 예상 결과: plugin JSON 연결, HEARTBEAT와 mode/위치 수신, armed=false 유지
- 실제 결과:
  - Gazebo plugin load와 UDP 9002 bind 확인
  - ArduCopter log에서 `JSON control interface set to 127.0.0.1:9002` 및
    `JSON received`(IMU/position/quaternion/velocity) 확인
  - MAVProxy 1.8.74를 `/home/hyojin/venv-ardupilot/bin/mavproxy.py`에서 탐지
  - 첫 HEARTBEAT 2.186초, 첫 GLOBAL_POSITION_INT 4.000초,
    GPS_RAW_INT/LOCAL_POSITION_NED 4.002초
  - 30초 창의 횟수: HEARTBEAT 10, SYS_STATUS 32, GPS_RAW_INT 32,
    GLOBAL_POSITION_INT 32, LOCAL_POSITION_NED 32, ATTITUDE 31
  - source sysid/compid 1/1, GPS fix_type 6, 유효 위경도
    (-35.363, 149.165), fresh global/local position, `armed=false`
  - MAVProxy→SITL audit: HEARTBEAT 32, REQUEST_DATA_STREAM 3,
    vehicle-affecting command 0
  - 수신 msgid 횟수: `0=10, 1=32, 2=32, 22=4, 24=32, 27=32,
    29=32, 30=31, 32=32, 33=32, 36=32, 42=32, 49=1, 62=32,
    65=32, 74=32, 111=1, 116=32, 125=32, 129=32, 133=4, 136=32,
    137=32, 193=32, 241=32, 242=1, 253=12`
- 판정: `PASS`
- 관련 로컬 로그 이름:
  - `simulation/logs/smoke_pass9_environment.log`
  - `simulation/logs/smoke_pass9_gazebo.log`
  - `simulation/logs/smoke_pass9_sitl.log`
  - `simulation/logs/smoke_pass9_mavproxy.log`
  - `simulation/logs/smoke_pass9_telemetry.log`
  - `simulation/logs/smoke_pass9_audit.json`
  - `/tmp/ArduCopter.log`
  - `/tmp/astrodrone-build-control.log`
  - `/tmp/astrodrone-ctest.log`

종료 후 관련 process와 9002/5760/5762/5763/5770/9005/14550/14551/14552 socket이 남지
않았음을 확인했다. SITL state는 기존 state를 삭제하지 않는 정책에 따라
`/tmp/astrodrone-simulation-runtime/sitl-0/eeprom.bin`에 유지된다.

## 원인 및 build 회귀 분리

최초 위치 실패는 GPS sensor나 EKF 이상이 아니었다. TCP SERIAL0의 GCS client가
없으면 SITL startup이 대기하며, 단순 수신 client는 stream-rate를 요청하지 않아
HEARTBEAT 외 위치 stream이 제공되지 않았다. MAVProxy 연결 후 ArduPilot Ready,
EKF3 IMU 초기화/tilt/yaw alignment, u-blox GPS 탐지를 확인했고,
`REQUEST_DATA_STREAM` 이후 유효 위치가 수신됐다. read-only parser는 위치를
놓치지 않았다.

외부 clean build는 `/tmp/astrodrone-control-sim-build`에서 성공했다. 기존 CTest는
3개 중 `target_json` 1개 PASS, `mav_connection`과
`command_characterization` 2개 FAIL이다. 전자는 기존 target discovery/stability와
coalesced/filter 보존 assertion 8건, 후자는 production target system assertion
1건의 알려진 MAVLink merge 회귀이며 이번 simulation 변경으로 수정·비활성화하지
않았다.
