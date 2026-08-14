# 비행 알고리즘 전체 흐름 (이륙 → 탐색 → 접근 → 정지)

작성 시각: 2026-08-14

`Document/algorithm-renewer.md`의 논의(위치/속도 하이브리드 제어, ROI 기반 추적 전략)를 바탕으로, 이후 대화에서 실제 세부 사양(감속 곡선, 좌표 기반 사선 이동, 재검증 게이트, ROI 크롭 계획)까지 구체화했다. 이 문서는 그 결과를 이륙부터 최종 정지까지 하나의 흐름으로 정리한 것이다.

`control/new_algorithm/`이 이 문서의 3~5단계(컨트롤 레이어)를 구현한다. 1~2단계(비전/거리계산)는 대부분 기존 코드를 그대로 재사용하고, 1단계의 ROI 크롭 확장만 아직 미구현(계획 단계)이다.

---

## 0. 전체 파이프라인 조감도

```
[1. 비전 인지]      YOLO_MODEL (Jetson 카메라)
                        │  HTTP GET /target  (found, confirmed, x_px, y_px, age_ms)
                        ▼
[2. 거리/좌표 계산]   control/target_distance.cpp
                        │  loopback UDP 15020, TargetRangeMsg
                        ▼
[3~5. 비행 제어]     control/new_algorithm/hybrid_guidance
                        │  MAVLink (SET_POSITION_TARGET_LOCAL_NED, 속도 필드)
                        ▼
                    픽스호크(Pixhawk) → 모터
```

세 구간은 완전히 독립된 프로세스이고, HTTP/UDP로만 통신한다. 하나가 죽어도 나머지는 죽지 않는다.

---

## 1. 비전 인지 단계 (YOLO_MODEL)

### 1-1. 현재 구현되어 있는 것

`YOLO_MODEL/cpp/yolo_headless.cpp` / `yolo_live.cpp`가 카메라 프레임마다 YOLO 추론을 돌리고, 결과를 `/target` HTTP 엔드포인트에 올린다.

**오검출 방지 (현재 구축된 것)**:

- **신뢰도 임계값**: `DEFAULT_CONF = 0.25` 미만인 박스는 애초에 탐지 리스트에 안 들어온다.
- **다중 탐지 시 그 프레임 폐기**: 한 프레임에 객체가 2개 이상 잡히면(`multi = dets.size() > 1`) 그 프레임은 확정용 스트릭에서 무조건 "미검출"로 취급된다.
- **5프레임 연속 동일 클래스 확정** (`TARGET_CONFIRM_FRAMES = 5`): 가장 최근 5개 추론 결과가 **전부** 같은 클래스여야 `confirmed = true`가 된다. `std::all_of`이므로 하나라도 어긋나면 즉시 탈락, 처음부터 다시 5개를 채워야 한다. 이건 정해진 시간(예: "2초")이 아니라 **프레임 개수** 기준이다 — `setting/rate.yaml`의 `infer_max_fps: 20`이 상한이므로 이상적으로는 5프레임 ≈ 0.25초, 실제 GPU 추론 속도가 느리면 더 걸린다.

즉 현재는 **"클래스 이름이 5프레임 연속 같았다"**는 것만 본다. 박스 위치/크기의 연속성, ROI 재검증 같은 건 없다.

### 1-2. 계획 중인 확장 (미구현)

타겟이 잡히면, 다음과 같은 방식으로 정밀도를 높일 계획이다:

1. 1280x720 전체 프레임을 4분면으로 나눠, 대상이 어느 분면에 있는지 파악한다.
2. 그 분면 기준으로 680x680 크기의 ROI를 잘라낸다.
3. **ROI 안에서 YOLO를 다시 돌려** 그 대상이 맞는지 재확인한다 (낚시하듯 좁혀 들어가는 방식 — 오검출 방지 1차 관문).
4. ROI의 중심 좌표를 화면(카메라) 중심과 비교해서, 그 방향으로 이동한다.

**이 부분은 실제 Jetson 카메라 + TensorRT 추론 파이프라인이 있어야 의미 있게 검증할 수 있어서, 오늘 작업(시뮬레이션 디버깅)에는 포함하지 않았다.** 시뮬레이션에서는 기존 `found`/`confirmed` 신호를 그대로 재사용해서 "1차 오검출 방지 게이트"로 삼고, 아래 3단계에서 설명할 "5m 셸 진입 시 재검증"을 2차 관문 자리에 배치해 구조만 맞춰두었다.

---

## 2. 거리/좌표 계산 단계 (target_distance.cpp)

100ms(`sense_cycle_ms`)마다 YOLO의 `/target`을 폴링하고, 동시에 MAVLink `ALTITUDE`를 받아서:

- **핀홀 카메라 모델**로 픽셀 오프셋을 실거리(m)로 변환한다 (`pos_calculator.cpp`):
  ```
  ground_offset_m = pixel_offset / pixel_focal_length_px × altitude_m
  ```
  같은 픽셀 오프셋이라도 고도가 높을수록 실제 지상 거리는 커진다는 걸 반영한 것.
- **피타고라스로 직선거리(슬랜트 레인지) 계산**:
  ```
  distance_m = hypot(ground_offset_m, altitude_m)
  ```
  타겟이 지면에 있고(z=0) 드론이 그 위 `altitude_m` 높이에 있다는 걸 전제로, 수평 오프셋과 고도를 빗변으로 합친 값이다. "5m 반경"을 원(2D)이 아니라 **구(3D, sphere)**로 정의하고 싶다는 요구사항과 정확히 맞아떨어지는 계산이라, 그대로 재사용한다.
- **이상치 제거**: 계산된 거리가 터무니없이 크거나(`max_plausible_distance_m`), 직전 값에서 갑자기 확 튀면(`max_distance_jump_m`) 그 사이클은 "타겟 없음"으로 처리한다.

결과(`x_px`, `y_px`, `distance_m`, `altitude_m`, `found`, `valid`)를 loopback UDP 15020번 포트로 쏜다. **여기까지는 기존 코드를 전혀 수정하지 않고 그대로 재사용한다.**

### 검증이 더 필요한 부분 (알려진 한계)

- `pixel_focal_length_px: 530`은 코드에 이미 "미검증 임시값"이라고 명시돼 있음 — 실비행 전 캘리브레이션 필요.
- x축/y축에 같은 초점거리 값 하나를 쓴다 (카메라 렌즈의 가로세로 화각이 다르면 오차 발생 가능).
- 타겟이 항상 지면(z=0)에 있다는 가정 — 공중 타겟이나 고도차가 있으면 안 맞음.

---

## 3. 컨트롤 레이어 — 전체 상태 머신

`control/new_algorithm/hybrid_guidance`가 담당한다. 흐름은 다음과 같다.

```
[시작] GUIDED 전환 → arm → 이륙(설정된 고도까지)
   │
   ▼
[SEARCH] 타겟 confirmed 대기 (정지 상태로 호버링)
   │  confirmed=true 확인되면
   ▼
[CRUISE] 등속 0.5 m/s로 타겟을 향해 사선(대각선) 이동
   │  distance_m ≤ 5m (shell_radius_m) 진입
   ▼
[REVERIFY] 제자리에서 정지, tracking이 1초(reverify_hold_sec) 연속 유지되는지 확인
   │  재확인 성공
   ▼
[DECEL] 지수함수로 감속하며 계속 사선 접근 (5m→1m 구간)
   │  distance_m ≤ 1m (stop_radius_m)
   ▼
[STOPPED] 속도 0으로 하드 정지, 제자리 호버링
```

각 단계를 구체적으로 설명한다.

### 3-1. 이륙까지

기존 `control.cpp`의 자체 실행 흐름(`main()`의 기본 경로 — `--auto-intercept`가 아닌 쪽)과 같은 패턴: `drone::set_mode("GUIDED")` → 모드 전환 확인(heartbeat 폴링) → `drone::arm_disarm(true)` → arm 확인 → `drone::takeoff(고도)`. 전부 기존 `drone_lib.hpp`의 함수를 그대로 호출한다.

`control.cpp`의 `run_auto_intercept()`/`wait_for_lock()` 경로는 쓰지 않는다 — `control/README.md`, `gazeboSim/README.md`에 기록된 `HealthState` 초기화 버그(매 루프 새 `HealthState`를 만들어서 `wait_for_lock()`이 armed 상태를 못 보고 즉시 빠져나가는 문제)가 있는 경로라, 아예 그 경로에 의존하지 않는 방식으로 짰다.

### 3-2. CRUISE — 좌표 기반 사선 이동 (5m 밖)

여기가 오늘 논의에서 가장 많이 다듬어진 부분이다.

**질문**: "사선으로 이동할 때 피타고라스 거리 계산이 필요한가?" → **필요하다.** 방향(벡터)과 속력 스케줄(스칼라)은 서로 다른 역할이고, 스칼라 "남은 거리"는 피타고라스 없이는 절대 못 구한다.

기존 `control.cpp`의 `approach_target()`은 **"일단 타겟 쪽으로 몸을 돌리고(yaw), 그다음에 전진(vx)"** 하는 방식이라 사선 이동이 아니다. 새 알고리즘은 **매 사이클마다 전진(forward)과 좌우(lateral) 성분을 동시에** 계산해서 진짜 대각선으로 움직인다.

방법: `target_distance.cpp`가 이미 주는 두 값 — 스칼라 `distance_m`(빗변)과 `x_px`에서 변환한 좌우 오프셋 `lateral_offset_m` — 을 직각삼각형으로 놓고, **피타고라스로 전진 성분을 역산**한다.

```
lateral_m = pixel_offset_to_ground_m(x_px, altitude_m, focal_length_px)   // 이미 다리 하나
forward_m = sqrt(distance_m² − lateral_m²)                                 // 나머지 다리를 피타고라스로 역산
```

이 `(forward_m, lateral_m)` 벡터를 정규화해서 원하는 속력(등속 구간이면 0.5m/s)만큼 곱한 뒤, `drone::send_velocity_body(vx, vy, 0, yaw_rate)`로 보낸다. `send_velocity_body`는 기존 `drone_lib.hpp`에 이미 있는 함수를 그대로 쓴다 — 새로 만든 MAVLink 전송 코드는 없다.

> **설계상 선택**: 타겟 좌표를 GPS/절대좌표로 한 번 "고정"해두고 그 점을 향해 항법하는 방식(위치 setpoint 고정)도 고려했지만, 채택하지 않았다. 그 방식은 픽스호크의 로컬 포지션(LOCAL_POSITION_NED) 텔레메트리를 새로 받아와야 하고, 기체 헤딩으로 좌표를 회전시키는 계산이 추가로 필요해서 검증 안 된 새 경로가 늘어난다. 대신 **매 사이클 최신 `x_px`/`distance_m`으로 계속 다시 조준**하는 방식을 택했다 — 이미 10Hz로 흘러들어오는 신뢰된 신호를 그대로 쓰고, 타겟이나 기체 위치 추정이 살짝 흔들려도 다음 사이클에 알아서 보정된다는 장점이 있다. 대신 타겟이 화면 밖으로 나가면(택배 상자 뒤에 가려지는 등) 그 사이클엔 명령이 안 나간다 — 어차피 기존 파이프라인도 똑같은 전제였다.

### 3-3. REVERIFY — 5m 셸 진입 시 재검증 (2차 오검출 방지 게이트)

`distance_m`이 처음으로 5m 밑으로 떨어지는 순간, 곧바로 접근을 계속하지 않는다. **제자리에 멈춰서(vx=vy=0), `tracking`(found && confirmed && 링크 신선함)이 `reverify_hold_sec`(기본 1초) 동안 끊기지 않고 유지되는지 확인**한다. 중간에 놓치면 타이머가 리셋되고 다시 1초를 채워야 한다.

이건 1단계에서 설명한 "1차 게이트(YOLO의 5프레임 스트릭)"와는 다른, **두 번째 독립적인 관문**이다. 멀리서 오검출로 5m까지 접근한 경우를 여기서 한 번 더 걸러낸다.

### 3-4. DECEL — 5m~1m 구간 지수 감속

재검증에 성공하면 다시 이동을 시작하되, 이번엔 등속이 아니라 지수함수로 감속한다.

```
v(d) = cruise_speed_mps × exp(−k × (shell_radius_m − d))
```

- `d = shell_radius_m`(5m)일 때: `v = cruise_speed_mps`(0.5 m/s) — 셸 경계에서는 등속 구간과 매끄럽게 이어짐.
- `k`는 "5m에서 0.5m/s, 2m에서 0.1m/s"라는 두 기준점으로 역산: `k = ln(0.5/0.1) / (5−2) ≈ 0.536`.
- 이 곡선의 모양은 **"멀리서는 거의 등속을 유지하다가, 목표 근처(마지막 1~2m)에서 속력이 훅 빠르게 떨어지는"** 형태다. 5m→2m 사이에 이미 0.5→0.1로 대부분 감속이 끝나 있고, 2m→1m 구간엔 (0.1 × 0.536 ≈) 0.06m/s 정도밖에 안 남는다.
- 방향 성분(forward/lateral)은 CRUISE와 똑같이 피타고라스 분해로 매 사이클 다시 계산한다 — 속력 크기만 이 지수함수로 바뀔 뿐, 사선 이동 방식 자체는 동일하다.

### 3-5. STOPPED — 1m 하드 정지

지수함수는 수학적으로 정확히 0에 도달하지 않고 한없이 가까워지기만 한다. 그래서 `distance_m ≤ stop_radius_m`(1m)이 되는 순간 위 수식과 상관없이 **속력을 무조건 0으로 못박는다**. 이게 실제로 기체를 멈추게 하는 지점이다.

---

## 4. 안전 계층 (기존 코드 스타일 재사용)

`hybrid_guidance_main.cpp`는 `control.cpp`의 안전 로직과 같은 값(`setting/safety.yaml`)을 기준으로, 같은 형태의 감시를 자체적으로 돌린다 (단, `control.cpp`의 private 함수들이라 직접 재사용은 못 하고 같은 패턴을 다시 짰다):

- **고도 상한**: `altitude_limit.hard_limit_m` 초과 시, 타겟 접근 명령을 무시하고 비례+최소값 방식으로 하강 명령을 보냄. `soft_limit_m` 초과는 경고만.
- **배터리/GPS/heartbeat/prearm 상태 감시**: 문제 생기면 `LOITER`로 전환, 최소 유지시간(3초) 지난 뒤 상태 해제되면 `GUIDED` 복귀.
- **heartbeat 완전 두절**(`land_gap_sec` 초과): 즉시 `LAND`.
- **타겟 장시간 로스트**(`target_lost_land_sec` 초과): 즉시 `LAND`.

---

## 5. 구현 위치 / 재사용 정리

| 구성 요소 | 상태 |
|---|---|
| YOLO 5프레임 확정 스트릭 (1차 오검출 게이트) | 기존 구현, 재사용 |
| YOLO 4분면 스캔 + 680x680 ROI 재탐지 | **미구현 — Jetson 카메라 필요, 향후 별도 작업** |
| 픽셀→미터 변환, 피타고라스 거리 계산 | 기존 구현(`pos_calculator.cpp`, `target_distance.cpp`), 재사용 |
| GUIDED 전환/arm/이륙 | 기존 `drone_lib.hpp` 함수 재사용 |
| 좌표기반 사선 이동 (피타고라스 분해) | **신규 — `control/new_algorithm/hybrid_guidance.cpp`** |
| 5m 셸 재검증 게이트 (2차 오검출 게이트) | **신규 — 위와 동일 파일** |
| 5m~1m 지수 감속, 1m 하드 정지 | **신규 — 위와 동일 파일** |
| 고도제한/배터리/GPS/heartbeat/타겟로스트 안전 감시 | 기존 로직과 같은 형태로 재구현 (`hybrid_guidance_main.cpp`) |

설정값은 `setting/hybrid_guidance.yaml`(신규 파일)에 모아뒀고, 기존 `setting/MAVLink.yaml`/`safety.yaml`/`port.yaml`은 읽기만 하고 전혀 수정하지 않았다.

---

## 6. 남은 미확정/미구현 항목

- **YOLO ROI 크롭(1단계 확장)**: Jetson 실기 카메라·TensorRT 파이프라인에서 별도로 구현·검증 필요.
- **1m 정지 이후 동작**: 현재는 그 자리에서 계속 호버링만 한다. 착륙할지, 다음 임무로 넘어갈지는 아직 정의 안 됨.
- **감속 곡선 상수(`decel_rate_per_m`)**: 5m/0.5m/s, 2m/0.1m/s 두 기준점으로만 역산한 값이라, 실비행/시뮬레이션에서 실제 느낌을 보고 재조정 필요.
