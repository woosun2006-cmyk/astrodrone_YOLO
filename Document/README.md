## 실행 환경

| 항목 | 값 |
|---|---|
| 장비 | Jetson (aarch64), 호스트명 `astro-desktop` |
| 접속 | `ssh astro@192.168.0.196` |
| Python | 3.6.9 |
| 의존성 | `pymavlink` 2.4.49, `MAVProxy`(링크 중계용) |
| 프로젝트 경로 | `~/astro-drone` |

```bash
pip3 install pymavlink MAVProxy
```

## 파일 구성

| 파일 | 역할 | 명령 전송 |
|---|---|---|
| `drone_lib.py` | 공용 MAVLink 래퍼. 아래 스크립트들이 공통으로 가져다 쓴다 | - |
| `check_link.py` | 링크/상태 점검 전용. **읽기 전용**이라 시동·모터 명령을 보내지 않는다 | ❌ |
| `test_arm.py` | ARM/DISARM 을 보내고 heartbeat 로 실제 반영됐는지 검증 | ⚠️ 시동 |
| `control.py` | GUIDED → 시동 → 이륙 → 착륙 자동 시퀀스 | ⚠️ 비행 |
| `control_sim.py` | 이륙 후 키보드(WASD)로 수동 조종 | ⚠️ 비행 |
| `mav.parm` | 기체 파라미터 덤프 | - |
| `mav.tlog`, `mav.tlog.raw` | MAVLink 텔레메트리 로그 | - |

### `drone_lib.py` 함수

전역 `master` 커넥션 하나를 모듈 안에 들고 있는 구조라, 다른 함수를 쓰기 전에
반드시 `connect()` 를 먼저 호출해야 한다 (안 하면 `require_connection()` 이 에러를 던진다).

- `connect(address, heartbeat_timeout=20)` — 연결 후 heartbeat 대기. 못 받으면 `TimeoutError`
- `set_mode(mode)` — `'GUIDED'`, `'AUTO'`, `'LAND'` 등. 지원하지 않는 모드면 `False` 반환
- `arm_disarm(arm_bool)` — 시동 켜기/끄기
- `takeoff(altitude)` — 지정 고도로 이륙
- `send_velocity(vx, vy, vz, yaw_rate=0)` — LOCAL_NED 속도 명령 (vz는 아래가 +)
- `land()` — 착륙

## 사용법

### 1. 링크 점검 (제일 먼저, 안전)

USB로 연결된 픽스호크의 heartbeat / 배터리 / GPS 상태만 8초간 읽는다.

```bash
python3 check_link.py                                  # 기본 /dev/ttyACM0, 115200
python3 check_link.py --address /dev/ttyACM0 --baud 921600
python3 check_link.py --address tcp:127.0.0.1:5762     # SITL
python3 check_link.py --listen 20                      # 20초 관찰
```

### 2. 시동 테스트

ARM 을 보내고 `COMMAND_ACK` + heartbeat 의 armed 플래그로 성공 여부를 판정한다.
기본 동작은 ARM → 3초 대기 → DISARM.

```bash
python3 test_arm.py                        # SITL(tcp:127.0.0.1:5762), GUIDED
python3 test_arm.py --address /dev/ttyACM0 # 실기체
python3 test_arm.py --no-mode              # 모드 변경 없이 현재 모드에서 시동
python3 test_arm.py --keep-armed           # DISARM 안 보내고 시동 유지
python3 test_arm.py --hold 10              # 10초 유지 후 DISARM
```

ARM 이 거부되면 출력되는 `STATUSTEXT: PreArm ...` 메시지가 원인이다.

### 3. 자동 이륙/착륙

```bash
python3 control.py
```

`udp:127.0.0.1:14550` 에 연결해 GUIDED → ARM → 10m 이륙 → 착륙까지 한 번에 실행한다.
연결 주소와 목표 고도(`target_alt`)는 파일 상단에 하드코딩되어 있다.

### 4. 키보드 수동 조종

```bash
python3 control_sim.py
```

`tcp:127.0.0.1:5762` 에 연결, GUIDED → ARM → 5m 이륙 후 키보드 조종으로 넘어간다.

| 키 | 동작 | 키 | 동작 |
|---|---|---|---|
| `W` / `S` | 전진 / 후진 | `Space` / `X` | 상승 / 하강 |
| `A` / `D` | 좌 / 우 이동 | `Q` / `E` | 좌회전 / 우회전 |
| `L` | 착륙 후 종료 | | |

수평 1.5 m/s, 회전 0.5 rad/s 고정. 키를 뗀 뒤에도 0.35초간 마지막 명령을 유지하다가
멈추며, 종료 시 `finally` 에서 속도 0을 보내고 터미널 설정을 되돌린다.

> `termios` / `tty` 를 쓰기 때문에 리눅스 터미널에서만 동작한다.

## 통신 구성

MAVLink 링크는 하나뿐인데 붙어야 할 프로그램은 여럿(스크립트, Mission Planner, MAVProxy 콘솔)이라,
**MAVProxy 를 허브로 두고 TCP 로 팬아웃**하는 구조를 쓴다.

```
[소스 하나만 선택]                  [허브]                     [클라이언트]

  실물 PH4-mini (USB)
    /dev/ttyACM0  ──┐
                    ├── mavproxy.py ──┬── tcpin:0.0.0.0:5760 ── Mission Planner 등
  Mission Planner   │    (--master)   └── tcpin:0.0.0.0:5762 ── control_sim.py
  SITL (UDP)      ──┘                                           test_arm.py
    udpin:0.0.0.0:14550                                         check_link.py
```

### 허브 띄우기

```bash
# (A) 실물 픽스호크를 소스로
mavproxy.py --master=/dev/ttyACM0 --baudrate=115200 \
            --out=tcpin:0.0.0.0:5760 --out=tcpin:0.0.0.0:5762

# (B) Mission Planner SITL 을 소스로 (MP 쪽에서 UDP 출력을 젯슨 IP:14550 으로 추가)
mavproxy.py --master=udpin:0.0.0.0:14550 \
            --out=tcpin:0.0.0.0:5760 --out=tcpin:0.0.0.0:5762
```

허브가 뜬 뒤 스크립트는 `tcp:127.0.0.1:5762` 로 붙는다.

### 포트 표

| 포트 / 장치 | 방향 | 용도 |
|---|---|---|
| `/dev/ttyACM0` | 시리얼 115200 | 실물 PH4-mini. MAVProxy `--master` |
| `udpin:0.0.0.0:14550` | 수신(bind) | Mission Planner SITL 이 쏴주는 MAVLink. MAVProxy `--master` |
| `udpin:0.0.0.0:14561` | 수신(bind) | 14550이 막혔을 때 쓴 대체 포트 |
| `tcpin:0.0.0.0:5760` | 서버 | GCS(Mission Planner)용 출력 |
| `tcpin:0.0.0.0:5762` | 서버 | 파이썬 스크립트용 출력 |

## MissionPlanner - Pixhawk 연결 도중 발생한 문제

### ① SITL 과 실물을 동시에 띄우면 포트가 충돌한다

초기에 SITL 용 MAVProxy 와 실물 픽스호크용 MAVProxy 를 **둘 다** 띄웠는데,
양쪽 다 출력이 `tcpin:5760` / `tcpin:5762` 로 같아서 나중에 뜬 쪽이
`Address already in use` 로 죽는다. 살아남은 쪽이 어느 소스인지도 구분이 안 된다.

→ **소스는 한 번에 하나만.** 굳이 동시에 띄워야 하면 출력 포트를 갈라준다
(예: 실물 5760/5762, SITL 5770/5772).

바꾸기 전에 뭐가 물고 있는지부터 확인:

```bash
sudo ss -lntup | grep -E "14550|14561|5760|5762"
ps -ef | grep mavproxy | grep -v grep
```

### ② `udp:` 는 보내는 게 아니라 **포트를 잡는다**

pymavlink 에서 `udp:HOST:PORT` 는 `udpin` 과 같은 동작이라 소켓을 bind 한다.
실제로 확인한 결과:

```
mavutil.mavlink_connection('udp:127.0.0.1:14599')
  → 타입: mavudp,  바인딩: ('127.0.0.1', 14599)
  → 같은 포트 재bind 시 [Errno 98] Address already in use
```

즉 `control.py` 의 `udp:127.0.0.1:14550` 은 14550 을 **직접 점유**한다.
MAVProxy 를 `--master=udpin:0.0.0.0:14550` 로 띄워 둔 상태면 둘 중 하나는 반드시 실패한다.
허브를 쓰는 지금 구조에서는 스크립트가 UDP 를 직접 잡을 이유가 없으므로
`control.py` 도 `tcp:127.0.0.1:5762` 로 붙는 게 맞다.

| 접두사 | 동작 |
|---|---|
| `udp:` / `udpin:` | 포트 bind (수신 대기) |
| `udpout:` | 지정 주소로 송신 |
| `tcp:` | TCP 클라이언트 (허브에 접속) ← 스크립트가 쓸 것 |
| `tcpin:` | TCP 서버 (허브가 여는 쪽) |

### ③ `ttyACM0` 과 `ttyACM1` 은 픽스호크 두 대가 아니다

둘 다 시리얼 번호가 같은 **하나의 PH4-mini** 이고, USB 인터페이스 번호만 00 / 02 로 다르다.

```
ID_SERIAL=ArduPilot_PH4-mini_350032000150315939353320   ← ACM0, ACM1 동일
ID_USB_INTERFACE_NUM=00 (ACM0) / 02 (ACM1)
```

MAVLink 텔레메트리는 `ttyACM0` 으로 나온다. `ttyACM1` 을 열면 heartbeat 가 안 잡힌다.
(`astro` 계정은 `dialout` 그룹에 있으므로 sudo 없이 접근 가능.)

## 유의사항

- `drone_lib.connect()` 의 타임아웃 에러 메시지는 포트 `14561` 을 안내하지만,
  `control.py` 는 실제로 `14550` 에 연결한다. 14561 은 14550 이 막혔을 때 임시로 쓴
  MAVProxy 마스터 포트라서, 지금 기준으로는 안내 문구가 맞지 않는다.
- `control.py` 만 아직 `udp:127.0.0.1:14550` 을 직접 bind 한다.
  나머지 스크립트처럼 `tcp:127.0.0.1:5762`(MAVProxy 출력)로 바꾸면 위 ②번 충돌이 사라진다.
- `control.py` 는 `takeoff()` 직후 대기 없이 `land()` 를 호출하기 때문에
  고도에 도달하기 전에 착륙 명령이 나간다. 고도 확인 루프 추가 필요.
