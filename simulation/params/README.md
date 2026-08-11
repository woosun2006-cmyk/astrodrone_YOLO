# Simulation-only parameters

현재 추가 parameter file은 없다. `gazebo-iris` frame에 내장된 ArduPilot
default와 `default_params/gazebo-iris.parm`만 사용한다. 필요 시 simulation 전용
값만 이 디렉터리에 두고 `SITL_PARAM_FILE`로 명시한다. 실제 hardware dump,
`setting/mav.parm`, serial/board/CAN/GPS hardware parameter를 적용하지 않는다.
