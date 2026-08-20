# Interactive audit readiness 경합 조사

## 증상과 원인

실패 run은 `simulation/logs/interactive_observe_20260812_014153`이다.
`audit.err.log` 생성 시각과 `packet_audit.json`의 생성 시각/실행 duration을
대조하면 다음과 같다.

- background wrapper 로그 생성: epoch `1786466551.161558`
- Python audit proxy 실행 시작 추정: epoch `1786466571.233022`
- proxy 실행 전 지연: `20,071.464 ms`
- 기존 launcher readiness timeout: `20 s`
- proxy는 이후 실제로 bind하고 `AUDIT_LISTEN tcp:127.0.0.1:5770`을 출력함
- cleanup SIGTERM까지 proxy 실행: `753.175 ms`
- packet audit: vehicle-affecting command 0

따라서 bind 실패가 아니다. Background `start_mavlink_audit.sh` 안의 MAVProxy
탐색, version 실행, venv Python import 및 당시 시스템 I/O/process scheduling이
20초 경계까지 지연됐다. Launcher가 단순 port polling을 먼저 timeout 처리한 직후
proxy가 listen한 readiness 경합이다.

현재 warm 분리 계측은 다음과 같았다.

- `find_mavproxy`: 144 ms
- `mavproxy.py --version`: 1,031 ms (개별 첫 실행)
- `import MAVProxy,pymavlink`: 26 ms
- 합친 `verify_mavproxy`: 439 ms
- audit dialect import: 198 ms
- standalone wrapper: find 144 ms, verify 404 ms, exec-ready 559 ms
- Python process 내부 socket listen: 16.762 ms
- standalone 시작부터 두 readiness 조건 충족: 987 ms

Warm 결과만으로 20초 지연을 재현하지는 못했지만, 실패 로그에서 proxy 실행 전
20,071 ms가 직접 확인되며 timeout과 일치한다.

## 수정

1. `find_mavproxy`, `verify_mavproxy`, audit pymavlink dialect import를 simulator
   시작 전에 동기적으로 한 번 수행한다.
2. 검증된 venv Python entry point를 symlink 상태로 audit wrapper에 전달한다.
   `realpath`로 `/usr/bin/python`으로 풀지 않아 venv `pymavlink`를 보존한다.
3. `MAVLINK_AUDIT_READY_TIMEOUT`을 이름 있는 설정으로 두고 기본값을 60초로 했다.
4. Readiness는 다음 세 상태를 한 polling cycle에서 함께 확인한다.
   - background wrapper PID 생존
   - audit log의 정확한 `AUDIT_LISTEN tcp:127.0.0.1:5770`
   - `ss`에서 실제 TCP LISTEN
5. 두 readiness 조건을 모두 만족하기 전에는 MAVProxy를 시작하지 않는다.
6. 실패를 `DEPENDENCY_VALIDATION_FAILURE`, `AUDIT_PROCESS_EARLY_EXIT`,
   `BIND_FAILURE_OR_PORT_COLLISION`, `AUDIT_LISTEN_LOG_WITHOUT_SOCKET`,
   `AUDIT_READINESS_TIMEOUT`으로 구분하고 exit status와 stderr를 출력한다.
7. 성공 조건을 timeout 검사보다 먼저 평가하므로 timeout 경계에서 marker/socket이
   확인된 경우 거짓 timeout으로 처리하지 않는다.
8. MAVProxy disconnect의 정상 TCP reset은 traceback 대신 `AUDIT_PEER_CLOSED`로
   기록한다.
9. Cleanup 중 child와 wrapper가 동시에 PID 파일을 제거하거나 `/proc` entry가
   사라지는 경합을 idempotent stale cleanup으로 처리한다.

단순 sleep은 production readiness 수정에 추가하지 않았다. 100 ms polling은
process/log/socket 상태 재평가용이다.

## 느린 startup 자동 시험

`simulation/tests/audit_readiness_test.sh`는 test-only listener의 실제 bind를 2초
늦추고 wrapper 생존, log marker, TCP LISTEN을 검사한다.

- configured delay: 2,000 ms
- 관측 readiness: 3,178 ms
- 5초 timeout 안에 PASS
- false timeout: 0

## 최종 실제 실행

### Observe

명령:

```bash
simulation/scripts/interactive_observe_shadow.sh \
  --mode observe --headless --duration 5 --mission-planner none
```

최종 PASS run: `simulation/logs/interactive_observe_20260812_015855`

- dependency: find 382 ms, verify 414 ms, audit import 352 ms, 합계 1,148 ms
- background readiness: 896 ms
- wrapper exec-ready: 41 ms
- Python 내부 listen: 28.199 ms
- `AUDIT_LISTEN`: 확인
- 실제 TCP LISTEN: 확인
- `MAVPROXY_CONNECTED`: 확인
- `SITL_CONNECTED`: 확인
- Gazebo-SITL `JSON received`: 확인
- telemetry heartbeat/position freshness: PASS
- HEARTBEAT 37, REQUEST_DATA_STREAM 3
- vehicle-affecting command: 0
- audit stderr: empty

### Shadow

명령:

```bash
simulation/scripts/interactive_observe_shadow.sh \
  --mode shadow --headless --duration 5 --mission-planner none
```

최종 PASS run: `simulation/logs/interactive_shadow_20260812_020006`

- dependency: find 334 ms, verify 818 ms, audit import 441 ms, 합계 1,593 ms
- background readiness: 895 ms
- wrapper exec-ready: 33 ms
- Python 내부 listen: 25.471 ms
- `AUDIT_LISTEN`, 실제 TCP LISTEN, `MAVPROXY_CONNECTED`, `SITL_CONNECTED`: 확인
- Gazebo-SITL `JSON received`: 확인
- telemetry heartbeat/position freshness: PASS
- baseline fixture: PASS
- HEARTBEAT 129, REQUEST_DATA_STREAM 9
- vehicle-affecting command: 0
- audit stderr: empty

두 최종 실행 후 관련 process, tracked PID file, 관련 port는 모두 0이다.

