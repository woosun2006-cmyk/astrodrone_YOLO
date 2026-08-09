# control

제어 프로그램 저장소 (C/C++)

## 해결됨 (2026-08-09)
- `target_distance.cpp`의 픽셀 오프셋 -> 실거리(`ground_offset_m`) 변환을
  고도 무관 고정 배율(`pixel_to_meter`) 방식에서, 고도를 반영하는 핀홀
  카메라 모델(`pos_calculator.cpp`의 `pixel_offset_to_ground_m()`,
  `target_track.pixel_focal_length_px`)로 교체함. `pixel_focal_length_px`
  기본값 530은 임시값이므로 실제 비행 전 알려진 거리/고도에서 캘리브레이션
  필요.
