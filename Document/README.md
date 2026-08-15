## 하드웨어 장비
 - board :  NVIDIA Jetson Nano Developer Kit (ram : 4Gb)
 - mcu : pixhawk 4 mini
	** 확인 필요 ** `setting/safety.yaml`의 주석은 이 기체를 "Pixhawk 6C mini"로
	적고 있음(2026-08-11에 mavlink로 BATT_LOW_VOLT=14.0 읽으면서 기록). 둘 중
	하나가 틀렸으므로 실물 확인 후 한쪽으로 통일할 것.
 - camera : IMX219
	장치 node : /dev/video0
	센서 최대 스펙: 3264x2464 @21fps 등 (RG10 raw bayer)까지 지원
	실제 사용: 640x480 (`YOLO_MODEL/cpp/yolo_headless.cpp`)
	** 초점거리 미측정 ** 아래 "카메라 캘리브레이션" 참고
 - gps : pixhawk 내장 gps module
 - 수신기 : ... -> pixhawk - telem1

### 카메라 캘리브레이션 (⚠️ 실비행 전 필수)

`setting/MAVLink.yaml`의 `pixel_focal_length_px: 530`은 **실측값이 아니다.**
파일 주석도 "rough starting value, not derived from a measured focal length"라고
적고 있다.

이 값이 틀리면 `control/pos_calculator.cpp`의

```
ground_offset_m = pixel_offset / pixel_focal_length_px * altitude_m
```

가 수평 거리를 그만큼 잘못 본다. 수직 성분은 고도 그 자체라 영향이 없어서
**수평 성분만 어긋나고, 결과적으로 기체가 거의 수직으로 낙하한다.**
2026-08-15 시뮬레이션에서 실제로 재현됐다(`Document/developinglogMJ.md` 4번,
`Document/simulation-log/08151210.md`).

측정은 수평거리를 몰라도 된다. 바구니 실측 크기를 쓰면 된다.

```
fx = 화면상 바구니 폭(px) x 고도(m) / 0.238(m)
```

- 바구니를 카메라 바로 아래에 두고 호버(또는 삼각대 고정)
- 고도는 픽스호크 `ALTITUDE.altitude_relative`
- **배포와 같은 640x480으로 측정** (fx는 픽셀 단위라 해상도가 바뀌면 값도 바뀜)
- 2/3/4 m에서 반복해 값이 일정한지 확인

---

## 탐지 대상 (타겟)

흰색 플라스틱 바구니. 실측값:

| 항목 | 값 |
|---|---|
| 가로 x 세로 x 높이 | 0.238 x 0.138 x 0.070 m |
| 대표 색 | RGB(244, 247, 244) = `#F4F7F4` |

시뮬레이션 모델은 `gazeboSim/models/target_basket/`. collision은 위 실측값
그대로이고, visual은 학습셋에서 잘라낸 실사 텍스처를 입힌 평면이다. 평면이
실물보다 큰 것은 "바구니를 키운 것"이 아니라 저해상도 카메라에서도 모델이 학습
때와 같은 픽셀 크기로 보게 하려는 것 - 물리 크기로 해석하면 안 된다.

---

## Setting
~/astro-drone/setting 폴더 안에서 포트, 연결 서버 값 고정

### safety
- 고도 상한, 배터리/GPS/heartbeat/EKF 임계값. `control.cpp`와
  `control/new_algorithm/hybrid_guidance`가 읽는다.
- `altitude_limit`: soft 8.0 m / hard 10.0 m (2026-08-15에 4.0/5.0에서 상향).
  hard를 넘으면 강제 하강 명령이 나간다.

### port
- for open server in jetson, the server port number is recorded in here.
- it's open in only wifi-ASTRO5G

### MAVLink
- it's for mavlink port number setting.
- it's different for SITL/pixhawk.
- one port can connect one device.
- `target_track.pixel_focal_length_px` = 위 "카메라 캘리브레이션" 항목.

### rate / cam_sets / limit_speed / hybrid_guidance
- `rate.yaml` : `sense_cycle_ms`(10 Hz 폴링), `infer_max_fps`(추론 상한)
- `cam_sets.yaml` : 카메라 캡처 설정 + **좌표계 규약**
  (`coord_origin: center`, +x 오른쪽 / +y **위쪽**)
- `hybrid_guidance.yaml` : 신규 접근 알고리즘 튜닝값
  (셸 5 m, 정지 1 m, 감속상수, 호버 5초 등)

---

## control
제어 프로그램 저장소

### drone-lib.cpp
제어 프로그램에 필요한 함수 라이브러리 저장소

### test-arm.cpp
arm/disarm 시동걸리는지 유무만 판단

### check-link.cpp
 - 연결/상태 점검 도구
 - arm/모터 명령 없음
 - heartbeat, 배터리 전압, gps fix, 상태 메시지만 출력

### control.cpp
 - 자동 비행 프로그램.
 - 비행 로직 담당.
 - `--auto-intercept` : AUTO 미션을 지켜보다 타겟 확정 시 GUIDED로 가로채기

### target_distance.cpp / pos_calculator.cpp
 - YOLO의 `/target`을 폴링하고 MAVLink 고도를 받아 픽셀 오프셋을 실거리로 변환
 - 핀홀 카메라 모델 + 피타고라스로 슬랜트 레인지 계산
 - 결과를 loopback UDP 15020으로 송신
 - **`confirmed=false`를 `found=false`와 동일하게 취급한다** (중요)

### new_algorithm/hybrid_guidance
 - 2026-08-14 신설. `control.cpp` 대신 쓰는 좌표기반 접근 알고리즘.
 - AUTO 감시 → 타겟 확정 → GUIDED 가로채기 → 3차원 사선 접근 →
   5 m 셸 재검증 → 지수 감속 → 1 m 하드 정지 → 5초 호버 → 착륙
 - 설계 문서: `Document/algorithm.md`
 - arm/이륙을 스스로 하지 않는다. 이미 AUTO로 날고 있는 기체를 가로챈다.

### emergency.cpp
 - 긴급 상황 시, 드론 작동 메뉴얼에 관한 프로그램
 - '긴급 상황'이 어떤 상황인지는 ~/astro-drone/safety.yaml에서 정의
** this is an old file, so it's needed to remove **

---

## test-cam
** This is strictly for camera testing purposes. For actual flights, we won't host the server due to resource constraints. **

### camserver.py
- 카메라 송출을 파이썬으로 함.
- 실내에서 카메라 연결, 카메라 송출 화면 체크 시 사용

### fpslog.py
- 카메라 스트리밍 + fps, 화질, cpu/ram/쿨럭 등의 내용 표시
- 카메라 설정 할 때, 'fps/화질'을 결정 할 때 사용

### logs
- fpslog.py 로 기록한 로그 보관

---

## YOLO-MODEL

### 0812best.onnx / 0812best.engine  ← **현재 사용 모델**
 - 클래스 1개: `astro-drone` (`classes.txt`)
 - 입력 640x640 고정, 출력 1x25200x6 = [cx, cy, w, h, obj, cls0]
 - `.engine`은 Jetson의 GPU/TensorRT 버전으로 직렬화된 것이라 **다른 기계에서
   못 쓴다.** 이식할 때는 `.onnx`를 쓸 것.
 - 실기 추론 경로: `cpp/yolo_headless.cpp` + `cpp/trt_engine.cpp`
   (전처리: letterbox + pad 114 + RGB + /255.0)

### best.pt
 - 학습 산출물(PyTorch). 위 onnx/engine의 원본.

### ~~prototype.pt~~ (삭제됨)
 - 2026-08-12 커밋 `35658ae`에서 `0812best`로 교체되며 삭제됨.
 - PC의 오래된 체크아웃에 남아 있을 수 있으나 **쓰면 안 된다** (320x320로
   export된 이전 모델).

### yolo_live.py
 - YOLO 모델 단독 테스트 용
 - `DEFAULT_CONF` 0.25, `TARGET_CONFIRM_FRAMES` 5
   (5프레임 연속 같은 클래스여야 `confirmed`. 한 프레임에 2개 이상 검출되면
   그 프레임은 미검출로 처리)

---

## gazeboSim
Gazebo/SITL 시뮬레이션 전용. **실비행 경로와 분리되어 있다.**

### run_sim_demo.sh  ← 명령 한 줄로 전체 기동
```powershell
wsl -e bash -lc "SIM_FX=205.47 ~/run_sim_demo.sh"          # YOLO
wsl -e bash -lc "SIM_FX=205.47 ~/run_sim_demo.sh noyolo"   # HSV
```
Gazebo GUI → 타겟 스폰 → 카메라 스트리밍 → 검출기 → 역터널/socat →
Jetson 스택 → 미션 비행 → 종료 시 전량 정리.
환경변수: `ALT` `PLANE_W`/`PLANE_H` `SIM_FX` `CONF` `NORTH` `JETSON` `SIM_ONLY` `KEEP`

### gazebo_yolo_target.py  ← **실제 가중치를 PC에서 구동**
 - `0812best.onnx`를 OpenCV DNN으로 추론해 `/target` 서빙 (torch 불필요)
 - 실기와 동일한 전처리/확정 규칙 + 근접 병합(2026-08-15 추가)

### gazebo_camera_target.py
 - HSV 색상 임계값 검출기(신경망 아님). 제어 계층만 검증할 때 사용.

### fake_yolo_target.py
 - 정적 합성 타겟. 인지 검증 안 됨.

### bridge_sitl.sh / spawn_target.sh / start_pc_sim.sh
 - PTY 브리지 / 타겟 스폰 / PC측 기동(Gazebo→SITL→MAVProxy 순서)

### patches/
 - `simulation/`(다른 브랜치 자산)에 대한 재현용 패치 보관

---

## Document

### algorithm.md
 - 비행 알고리즘 설계 문서. 접근 시퀀스 전체 정의.

### Architecture.md
 - 시스템 구조.

### developinglogMJ.md / developinglogHJ.md
 - 개발 로그(날짜별).

### simulation-log/  ← 2026-08-15 신설
 - 시뮬레이션 시행 기록. 한 번의 **원인분석 → 디버깅 → 결과 테스트**
   사이클마다 문서 하나와 커밋 하나.
 - 파일명 = 커밋명 = `MMDDHHMM`(KST).

### logs/
 - 비행 로그(gitignore됨).
