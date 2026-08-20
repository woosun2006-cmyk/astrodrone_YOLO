# astrodrone_YOLO

바구니 표적을 카메라로 찾고, 기체를 표적 근처로 유도하는 프로젝트입니다.

기본 데이터 흐름은 다음과 같습니다.

```text
카메라
  -> C++ TensorRT YOLO
  -> HTTP /target detection
  -> target-distance
  -> 픽셀·고도 기반 거리 계산
  -> guidance
  -> safety / CommandGate
  -> MAVLink
  -> Pixhawk 또는 ArduPilot SITL
```

## 빠른 실행

저장소 루트에서 다음 명령으로 SITL 비행 검증 프로파일을 실행합니다.

```bash
env -u DISPLAY -u WAYLAND_DISPLAY \
simulation/scripts/run_simulation.sh \
  --profile sitl-flight \
  --simulation-profile diagonal-approach-validation \
  --headless \
  --status-monitor \
  --upload-mission \
  --arm
```

이 명령은 실제 기체가 아니라 loopback SITL에만 명령을 보냅니다. GUI로 보려면
`--headless`를 빼고 실행 환경의 Gazebo GUI 조건을 확인해야 합니다.

`diagonal-approach-validation`은 사선 접근 검증용 SITL 설정입니다. 결과는
다음 항목을 서로 나누어 봐야 합니다.

- command-level 45도 경로각
- 실제 SITL 궤적 경로각
- GUIDED 전환
- `TargetCenteredHold` 진입과 유지
- 실제 장치 검증 여부

## 현재 상태

| 항목 | 상태 |
| --- | --- |
| SITL YOLO pipeline | 기존 실행 로그 기준 검증 기록 있음 |
| diagonal 45도 guidance | SITL command/trajectory 검증 기록 있음. 실제 기체와는 별도 |
| GUIDED / TargetCenteredHold | SITL 로그와 단위 테스트에서 확인 범위가 나뉨 |
| GCS video protocol | offline 검증 완료 |
| Jetson TensorRT | 실제 Jetson 미검증. WSL에서는 TensorRT 개발 의존성 확인 필요 |
| Pixhawk serial | 실제 장치 미검증 |
| 실기체 비행 | 미실시 |

## 폴더 한눈에 보기

| 폴더 | 역할 |
| --- | --- |
| `control/` | 거리 계산, guidance, safety, 상태머신, MAVLink 제어 |
| `YOLO_MODEL/cpp/` | camera adapter, TensorRT YOLO, `/target`, video publisher |
| `setting/` | MAVLink·카메라·안전·runtime 설정 |
| `scripts/` | 공통 C++/YOLO 실행 wrapper와 real launcher |
| `simulation/` | Gazebo world/model, SITL, simulation launcher |
| `gcs/` | telemetry/video bridge와 노트북 dashboard |
| `health-check/` | 읽기 전용 상태 모니터 |
| `control/tests/` | C++ 단위·안전·guidance 테스트 |

자세한 데이터 흐름은 [Document/ARCHITECTURE.md](./Document/ARCHITECTURE.md),
실행 순서와 장애 대응은 [Document/OPERATIONS.md](./Document/OPERATIONS.md)를
읽으세요.

## 실행 모드

| 모드 | 동작 | vehicle 명령 |
| --- | --- | --- |
| `observe-sim` | SITL telemetry 관찰 | 금지 |
| `shadow-sim` | YOLO·거리·guidance 계산 | 차단 |
| `flight-sim` | loopback SITL 운항 명령 | 명시적 옵션 필요 |
| `observe-real` | router UDP telemetry 관찰 | 금지 |
| `shadow-real` | 실제 camera·YOLO·guidance 계산 | 차단 |
| `flight-real` | 실기체 명령 경로 | 기본 거부, 이중 확인 필요 |

표의 `observe-sim`·`shadow-sim`·`flight-sim`은 설명용 이름입니다. 실제
`run_simulation.sh` 옵션은 각각 `--profile observe`, `--profile shadow`,
`--profile sitl-flight`입니다.

노트북 GCS는 telemetry와 영상을 표시하지만 ARM, mode, takeoff, LAND,
velocity 명령을 보내지 않습니다. Pixhawk serial은 실기체에서 external
MAVLink router가 단독으로 소유해야 합니다.

## 주의사항

- `observe`와 `shadow`는 vehicle 명령을 보내지 않습니다.
- `flight-sim`의 `--arm`은 SITL에서만 사용합니다.
- `flight-real`은 실제 장치에 연결하지 않은 상태에서도 기본적으로 거부됩니다.
- `--fixed-basket`은 YOLO를 우회하는 별도 SITL 시험 입력입니다. 실제 YOLO
  end-to-end 검증으로 사용하지 않습니다.
- generated 파일, build 디렉터리, 로그는 프로젝트의 canonical 소스 구조가
  아닙니다.
- 실제 Jetson, Pixhawk, serial, 외부 LAN 검증은 아직 하지 않았습니다.
