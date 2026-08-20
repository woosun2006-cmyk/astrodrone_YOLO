# Interactive simulation / audited shadow 분리

## 결론

- 수동 경로는 `interactive_sim.sh`로 분리했다. `sim_vehicle.py`가 ArduCopter와
  MAVProxy를 함께 실행하며 audit relay 5770을 열지 않는다.
- 자동 경로는 `audited_shadow_test.sh`로 분리했다. 여기에서만 headless, 고정
  duration, audit relay 5770, packet audit, 자동 PASS/FAIL을 사용한다.
- production control, YOLO, target/guidance 수식과 MAVLink production 코드는
  변경하지 않았다.
- `--mode flight`는 두 public launcher에서 모두 실행 전에 거부된다.

요청된 Astroquad 참조 파일 세 개는 현재 WSL의 지정 경로와 `/home/hyojin`
전체 검색에서 발견되지 않았다. 따라서 line-by-line 비교는 할 수 없었고,
설명된 Astroquad 실행 방식과 설치된 ArduPilot `sim_vehicle.py`의 실제
`start_mavproxy()` / `--no-extra-ports` / `--mavproxy-args` 동작을 대조했다.

## 실행 구조 대응

수동 interactive:

```text
repository ensamb_iris_runway.sdf
  -> gz sim GUI/server
  -> sim_vehicle.py
       -> ArduCopter SITL (JSON, tcp:127.0.0.1:5760)
       -> bundled MAVProxy
            -> udpout:127.0.0.1:14552 (Mission Planner)
```

`--no-extra-ports`로 sim_vehicle의 WSL/Windows 자동 14550 output을 끈다.
14550은 onboard/control 예약 상태이며 이 launcher가 열지 않는다. integrated
MAVProxy는 background stdin EOF로 종료되지 않도록 `--non-interactive`지만,
Gazebo와 Mission Planner는 사람이 직접 조작한다.

자동 audited shadow:

```text
headless gz sim
  -> sim_vehicle.py --no-mavproxy
  -> transparent audit tcp:127.0.0.1:5770
  -> separate MAVProxy
  -> receive-only telemetry + FrameSource/pose probes + ShadowCommandSink
```

## 실행 명령

```bash
simulation/scripts/interactive_sim.sh
simulation/scripts/interactive_sim.sh --shadow
simulation/scripts/audited_shadow_test.sh
simulation/scripts/audited_shadow_test.sh --duration 10
```

Mission Planner는 별도 WSL terminal에서 기존 설치를 그대로 실행한다.

```bash
cd /home/hyojin/MissionPlanner
DISPLAY=:0 WAYLAND_DISPLAY=wayland-0 mono MissionPlanner.exe
```

Mission Planner에서는 UDP local port `14552`를 선택한다.

## 실제 검증

수동 run `simulation/logs/interactive_manual_20260812_021215/`:

- repository world, `ensamb_with_gimbal`, `target_basket`, down-camera topic 확인
- `sim_vehicle.py` 내장 MAVProxy가 `tcp:127.0.0.1:5760`에 연결
- MAVProxy가 vehicle `1:1` 탐지
- Gazebo-SITL `JSON received` 확인
- launcher 출력 `SITL_STATUS=READY MAVProxy=CONNECTED
  Gazebo_SITL_JSON=CONNECTED`
- control/YOLO 시작 0건, audit 5770 사용 안 함
- Ctrl+C 후 tracked PID file와 관련 port 0

현재 Codex 자동 PTY에서는 WSLg Mesa/Zink가 GLX context를 만들지 못해 Gazebo GUI
client가 `Unable to create the rendering window` 후 segmentation fault를 냈다.
GUI process와 world loading은 실제 시작됐고 한 run에서는 server/SITL/MAVProxy가
READY까지 진행했지만, GUI 화면 표시 자체는 PASS로 주장하지 않는다. 같은 현상은
`LIBGL_ALWAYS_SOFTWARE=1`에서도 재현됐다. launcher는 이 경우 audit/bind 오류가
아닌 GUI rendering failure로 보고한다.

자동 run `simulation/logs/interactive_shadow_20260812_021737/`:

- dependency validation: 1,377 ms
- audit readiness: 1,094 ms
- `AUDIT_LISTEN` marker와 실제 TCP LISTEN 모두 확인
- `SITL_CONNECTED`, MAVProxy vehicle 탐지, Gazebo-SITL JSON 연결 확인
- runtime optical axis body `(0, 0, -1)`, world `(0, 0, -1)`
- baseline 15 case byte-identical PASS
- ShadowCommandSink socket write 0, serial write 0
- MAVProxy -> SITL: HEARTBEAT 172, REQUEST_DATA_STREAM 12
- vehicle-affecting command 0
- cleanup 후 tracked PID/process/관련 port 0

2초 지연 listener 자동 테스트도 readiness 2,141 ms, false timeout 0으로 PASS했다.

## Cleanup

Gazebo GUI child가 SIGTERM을 무시하는 경우가 있어 `stop_simulation.sh`는 기록한
정확한 process group에만 TERM, INT, 최종 KILL을 단계적으로 적용한다. 이름 기반
`pkill`은 사용하지 않는다.

## Git / 안전

- stash apply/pop/drop 0건
- git add/commit/push 0건
- 외부 Astroquad, ArduPilot, ardupilot_gazebo 수정 0건
- scoped `git diff --check` PASS
- 저장소 전체 `git diff --check`는 기존 범위 밖 CRLF/trailing whitespace 변경
  때문에 FAIL하며 이번 작업에서 정리하지 않았다.
