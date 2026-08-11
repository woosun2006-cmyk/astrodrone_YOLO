# control

제어 프로그램 저장소 (C/C++)

## 추가됨 (2026-08-11)
- `control --auto-intercept`: 픽스호크가 (외부에서 업로드된) AUTO 미션을
  자체적으로 날고 있는 상태를 지켜보다가, YOLO 타겟이 `lock_confirm_sec`
  이상 안정적으로 잡히면 GUIDED로 가로채 `approach_target()`으로 접근하고,
  타겟을 놓치거나 `intercept_approach_duration_sec`이 지나면 다시 AUTO로
  돌려주는 흐름. 이 프로그램은 arm/이륙을 직접 하지 않음 (기존
  플래그 없는 실행과 그 부분이 다름). `approach_target()`에
  `resume_mode` 파라미터를 추가해 구현 - 자세한 흐름은 control.cpp의
  `run_auto_intercept()` 주석 참고. `scripts/full_mission.sh`가 이 플래그로
  health_check + yolo + target_distance + control을 한 번에 띄운다.
  새 설정 키(`lock_confirm_sec`/`intercept_approach_duration_sec`/
  `reacquire_cooldown_sec`)는 `setting/MAVLink.yaml`의 `target_track`에.

## 해결됨 (2026-08-09)
- `target_distance.cpp`의 픽셀 오프셋 -> 실거리(`ground_offset_m`) 변환을
  고도 무관 고정 배율(`pixel_to_meter`) 방식에서, 고도를 반영하는 핀홀
  카메라 모델(`pos_calculator.cpp`의 `pixel_offset_to_ground_m()`,
  `target_track.pixel_focal_length_px`)로 교체함. `pixel_focal_length_px`
  기본값 530은 임시값이므로 실제 비행 전 알려진 거리/고도에서 캘리브레이션
  필요.
