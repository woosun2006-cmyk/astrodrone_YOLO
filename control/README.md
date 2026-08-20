# control

제어 프로그램 저장소 (C/C++)

## 실행 흐름

`control`의 기본 동작과 `--auto-intercept`는 동일하다. 기체를 직접 ARM하거나
이륙시키지 않고, Mission Planner가 올린 임무가 `AUTO + ARMED` 상태가 될 때까지
기다린다. 건강한 표적 탐지가 `lock_confirm_sec` 동안 유지되면 `GUIDED`로
전환하여 접근한다. 표적을 오래 잃거나 한 번의 접근 제한 시간이 끝나면 `AUTO`로
복귀하고 다음 탐지를 기다린다. HEARTBEAT 단절이나 비행 중 예상치 못한 DISARM은
안전 종료/착륙 경로로 처리한다.

```bash
./build/control
./build/control --auto-intercept
```

독립 시험에서 프로그램이 직접 `GUIDED`, ARM, 4.5m 이륙, 접근, LAND까지 하게
하려면 다음 옵션을 사용한다.

```bash
./build/control --self-launch
```

현재 사선 접근 계산, `target_distance --fixed-target-ned` 고정 표적 시험,
MAVLink 수신 큐, CSV 로그는 그대로 유지된다.

## 해결됨 (2026-08-09)
- `target_distance.cpp`의 픽셀 오프셋 -> 실거리(`ground_offset_m`) 변환을
  고도 무관 고정 배율(`pixel_to_meter`) 방식에서, 고도를 반영하는 핀홀
  카메라 모델(`pos_calculator.cpp`의 `pixel_offset_to_ground_m()`,
  `target_track.pixel_focal_length_px`)로 교체함. `pixel_focal_length_px`
  기본값 530은 임시값이므로 실제 비행 전 알려진 거리/고도에서 캘리브레이션
  필요.

## SITL·실기체 transport 경계

런타임 transport는 `setting/runtime.sitl.yaml`과
`setting/runtime.real.yaml`에서 선택한다. 기본 target은 `sitl`이다.

- SITL에서는 `control`이 `udp:127.0.0.1:14550`, `target-distance`가
  `udp:127.0.0.1:14551`을 사용한다.
- 실기체에서는 외부 MAVLink router가 Pixhawk serial의 유일한 소유자다.
  router가 serial을 열고 `14550`(명령 입력)과 `14551`(telemetry fan-out)을
  loopback UDP로 제공해야 한다.
- 따라서 `target-distance`는 실기체에서 `/dev/tty*`를 직접 열지 않는다.
  두 프로세스가 같은 serial을 동시에 열지 않는 것이 이 경계의 핵심이다.
- `runtime.real.yaml`의 기본 `commands_enabled`는 `false`다. `--target real`,
  `--connect /dev/serial/by-id/...`, `--allow-arm`,
  `--confirm-real-flight`를 확인하는 별도 `flight-real` 런처가 추가되기
  전에는 실기체 명령 경로를 실행하지 않는다.

카메라 입력은 `FrameSource` 공통 인터페이스를 사용한다. Jetson은
`JetsonCameraFrameSource`, Gazebo는 `GazeboFrameSource`를 선택하며,
기존 `make_imx219_frame_source()` 이름은 호환용으로 남아 있다.
