# Local worlds

upstream 연결 smoke는 외부
`$ARDUPILOT_GAZEBO_DIR/worlds/iris_runway.sdf`를 읽는다. Project camera smoke는
저장소 소유 `ensamb_iris_runway.sdf`를 절대경로로 선택하며, 외부 checkout의
동명 파일로 fallback하면 실패한다. Custom world는 `ensamb_with_gimbal`과
`target_basket`을 include하고 지면은 inline primitive로 정의한다.
