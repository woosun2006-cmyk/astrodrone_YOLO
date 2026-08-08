# control

제어 프로그램 저장소 (C/C++)

## 확인 필요 (2026-08-08)
- `target_distance.cpp`: 픽셀 오프셋 -> 실거리(`ground_offset_m`) 변환이 현재
  고도와 무관하게 고정 배율(`target_track.pixel_to_meter`)만 곱해서 계산되고
  있음. 실제로는 고도에 따라 카메라가 담는 지상 범위가 달라지므로, 같은
  픽셀 오프셋이라도 고도가 높을수록 실거리는 더 커져야 함 - 지금 방식은
  틀린 값을 낼 수 있어 수정 필요.
- 이 변환(고도 반영 픽셀->실거리 계산)은 `pos_calculator.cpp`가 담당해야
  함. 현재 `pos_calculator.cpp`는 빈 파일 상태.
