# Local models

Upstream 연결 smoke는 외부 `iris_with_gimbal`을 사용한다. Project camera
smoke는 저장소 소유 `ensamb_with_gimbal`, `ensamb_with_standoffs`,
`target_basket`과 SDF가 직접 참조하는 최소 mesh만 사용한다.

`GZ_SIM_RESOURCE_PATH`는 이 디렉터리를 외부 ardupilot_gazebo model 경로보다
앞에 둔다. Model/plugin source나 build 결과는 복사하지 않으며
`ArduPilotPlugin`, `CameraZoomPlugin`, `GstCameraPlugin`은 외부 설치에 의존한다.
