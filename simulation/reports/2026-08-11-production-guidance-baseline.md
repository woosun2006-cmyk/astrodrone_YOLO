# production target-guidance baseline

- branch: `Document`
- HEAD: `bf9d99fd259818bf2076944eb58375caf9706905`
- HEAD tree: `49bca12e32475df6f20cbb73db466e295f3b6c53`
- fixture: `simulation/fixtures/target_guidance_baseline_inputs.csv`
- fixture SHA-256:
  `0aa7527a740336cea10c5738c4da6a20c42824e431c956342f959d71226023bd`
- baseline: `simulation/logs/target_guidance_baseline_head_bf9d99f_20260811_final/baseline.csv`
- baseline SHA-256:
  `c8c6d173a2bff593c84b1b2ac24e1dc741d0db18a43f8d89e517f752326472ca`
- control main/SITL/YOLO 실행: 없음
- 외부 MAVLink write: 0

작업 시작 시 working tree는 393개 modified entry가 있었고, 조사한
`control.cpp`, `target_distance.cpp`, `pos_calculator.cpp`, `target_link.cpp`와
관련 setting은 `--ignore-space-at-eol` 기준 HEAD와 일치했다. 즉 대량 표시는
알고리즘 변경이 아니라 기존 working-tree 줄바꿈 차이였다. Baseline script는 이
semantic HEAD guard가 실패하면 실행을 거부한다.

## 실제 호출 경계

Offline adapter는 기존 `test_command_characterization.cpp`와 같은 방식으로
production `control.cpp`를 번역단위에 포함하고 다음을 직접 호출했다.

- `pixel_offset_to_ground_m()`
- `TargetRangeSender::send()` / `TargetRangeReceiver::poll()`
- `approach_target()`
- `drone::send_velocity_body()`

`approach_target()`이 만든 MAVLink packet은 production encoder를 거치지만
`FakeTransport` 메모리에만 기록된다. OS MAVLink socket/serial transport는 만들지
않았다. `target_distance.cpp`의 JSON freshness/outlier 처리와 distance 결합은
무한 MAVLink/HTTP loop 안에 있어 독립 호출할 수 없다. Fixture adapter는 그 loop가
내보내는 `TargetRangeMsg` 경계를 만들며 실제 production pos calculator를 호출한다.
향후 개선 시 JSON→TargetRangeMsg도 순수 함수로 추출해야 전체 Stage 1을 복제 없이
unit-test할 수 있다.

## 현재 수식과 부호

YOLO 좌표는 frame center 원점, `+x=right`, `+y=up`이다.

```text
pixel_offset = hypot(x_px, y_px)
ground_offset = pixel_offset / 530 * altitude
slant_distance = hypot(ground_offset, altitude)

tracking = link_age <= 500ms && valid && found
approach_distance = max(0, slant_distance - 2.0)
raw_yaw_rate = 0.006 * x_px
yaw_rate = clamp(raw_yaw_rate, -0.6, 0.6)
align = clamp(1 - abs(x_px) / 80, 0, 1)
vx = min(0.5, 0.5 * approach_distance) * align
vy = 0
vz = 0                                when altitude <= 5m
vz = clamp(0.5 * (altitude - 5), 0.1, 0.5) when altitude > 5m
```

출력 frame은 `MAV_FRAME_BODY_OFFSET_NED`: `+vx=forward`, `+vy=right`,
`+vz=down`, positive yaw rate는 오른쪽/clockwise이다. 생성 packet은
`SET_POSITION_TARGET_LOCAL_NED`, 현재 type mask는 decimal `1991`이다.

## 핵심 결과

| case | ground/slant m | raw vx/yaw | captured vx/vy/vz/yaw |
|---|---|---|---|
| center, 3m | 0 / 3 | 0.5 / 0 | 0.5 / 0 / 0 / 0 |
| left, 3m | 1.358491 / 3.293250 | 0 / -1.44 | 0 / 0 / 0 / -0.6 |
| right, 3m | 1.358491 / 3.293250 | 0 / 1.44 | 0 / 0 / 0 / 0.6 |
| up, 3m | 1.018868 / 3.168295 | 0.584147 / 0 | 0.5 / 0 / 0 / 0 |
| down, 3m | 1.018868 / 3.168295 | 0.584147 / 0 | 0.5 / 0 / 0 / 0 |
| four corners, 3m | 2.264151 / 3.758508 | 0 / ±1.92 | 0 / 0 / 0 / ±0.6 |
| missing | 0 / 0 | 0 / 0 | 0 / 0 / 0 / 0 |
| YOLO stale | 0 / 0 | 0 / 0 | 0 / 0 / 0 / 0 |
| link stale | 0.566038 / 3.052933 retained | 0 / 0 | 0 / 0 / 0 / 0 |
| `(80,60)`, 1m | 0.188679 / 1.017644 | 0 / 0.48 | 0 / 0 / 0 / 0.48 |
| `(80,60)`, 3m | 0.566038 / 3.052933 | 0 / 0.48 | 0 / 0 / 0 / 0.48 |
| `(80,60)`, 5m | 0.943396 / 5.088222 | 0 / 0.48 | 0 / 0 / 0 / 0.48 |

Missing/stale observation tick도 zero-valued `SET_POSITION_TARGET_LOCAL_NED`를
생성한다. 연속 target loss가 10초를 넘으면 production loop는 LAND로 escalation
하지만 이 baseline은 명령을 만들지 않도록 짧은 관찰 구간만 실행했다.

## 재현된 문제

1. 하향 camera에서 화면 중앙은 ground offset 0인데 slant distance에 altitude가
   들어간다. 3m center가 distance 3m로 처리되어 body-forward `vx=0.5m/s`가 된다.
2. `y_px` 부호는 거리의 `hypot`에서 사라지고 guidance에는 사용되지 않는다.
   따라서 up/down이 완전히 같은 forward 명령을 만든다.
3. `vy`는 모든 입력에서 0이다. 하향 영상의 2D 지면 오차를 body XY translation으로
   바꾸지 않고 x 오차를 yaw로만 바꾼다.
4. 동일 픽셀에서 altitude가 증가하면 ground/slant distance는 증가하지만
   `(80,60)`은 yaw alignment 경계라 vx가 항상 0이고 yaw는 altitude와 무관하게
   0.48rad/s로 동일하다.

기존 CTest는 수정·비활성화하지 않았다. 결과도 기존 기록과 동일하게
`target_json` PASS, `mav_connection` 및 `command_characterization` FAIL이었다.

기존 custom Gazebo sample에는 YOLO detection 좌표/시각이 함께 기록돼 있지 않고,
이번 작업에서는 YOLO를 시작하지 않았으므로 실제-frame 행은 만들지 않았다.
근거 없는 픽셀 좌표를 실제 입력으로 표기하지 않고 모든 행을 `source=synthetic`으로
구분했다.

## 다음 stash 비교

Fixture와 CSV schema가 고정됐다. `stash@{0}`는 이번 검증에서 조회만 했으며
apply/pop/drop하지 않았다. 다음 단계는 별도 clean worktree/branch에 개선안을
적용하고 동일 script를 실행한 뒤 `case_id`별 ground/slant/raw/clamped 출력과
target-loss 상태를 비교하면 된다.
