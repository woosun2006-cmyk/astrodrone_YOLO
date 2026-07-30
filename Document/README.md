## 하드웨어 장비
 - board :  NVIDIA Jetson Nano Developer Kit (ram : 4Gb)
 - camera : IMX219
	장치 node : /dev/video0
	센서 최대 스펙: 3264x2464 @21fps 등 (RG10 raw bayer)까지 지원

## Setting
~/astro-drone/setting 폴더 안에서 포트, 연결 서버 값 고정

### safety

### port

### MAVLink

---

## control
제어 프로그램 저장소
현제 파이썬으로 되어 있는데, c/c++로 바꿔야 함.

### drone-lib.py
제어 프로그램에 필요한 함수 라이브러리 저장소

### test-arm.py
arm/disarm 시동걸리는지 유무만 판단

### check-link.py
 - 연결/상태 점검 도구
 - arm/모터 명령 없음
 - heartbeat, 배터리 전압, gps fix, 상태 메시지만 출력

### control-sim.py
 - 시뮬레이션 목적('gazebo/mission planner' 에서 tcp:5760 으로 SITL 포트 열어야함)
 - wasd등 키보드로 비행 조작

### control.py
 - 자동 비행 프로그램.
 - 비행 로직 담당.

### PID.py
 - ardupilot 등의 오픈소스 사용.
 - 비행 시, 드론이 수평을 자동으로 맞출 수 있도록 제어 해 주는 프로그램

### emergency.py
 - 긴급 상황 시, 드론 작동 메뉴얼에 관한 프로그램
 - '긴급 상황'이 어떤 상황인지는 ~/astro-drone/safety.yaml에서 정의

---

## test-cam

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

### yolo-live.py
 - YOLO 모델 단독 테스트 용
 - settings 변경해서 사용.
	DEFAULT_RECORD_SEC : log 시간 기본 값
	SAMPLE_INTERVAL_SEC : cpu/tempt/fps 띄우는 간격
	LIVE_WINDOW_SEC : 기록 중이 아닐 때, 화면이 몇초 일때를 보여 줄지에 대한 기본 값
	LOG_PREFIX : 기록 후 저장되는 텍스트 파일 이름 접두어
	PORT : 서버 송출 포트 (setting에서 관리)

### best.pt
실제로 사용 할 YOLO MODEL
단, 확장자를 바꾸는 등의 크기 축소가 필요 할 수 있음.

### prototype.pt
제어 코드 짤 때, 욜로 모델이 필요하긴 해서 존재하는 파일. 실제로 사용할 애는 아닌데, 카메라 -> 객체인식 -> 위치 파악 -> 이동 의 로직을 구현 할 때 사용.
 ** 완전 쓰레기 모델은 아님.. **
