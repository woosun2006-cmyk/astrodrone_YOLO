# Simulation report format

대용량 log, 영상, binary 대신 다음 형식의 작은 Markdown 요약만 저장한다.
성공을 직접 확인하지 않은 항목은 `미실행` 또는 `확인 불가`로 적는다.

```markdown
# <scenario> report

- 테스트 날짜:
- 저장소 Git commit:
- ArduPilot commit:
- Gazebo/plugin commit:
- scenario:
- 설정: world, model, headless, instance, location, speedup, endpoint/port
- 예상 결과:
- 실제 결과:
- 판정: PASS / FAIL / BLOCKED / NOT RUN
- 관련 로컬 로그 이름: simulation/logs 아래 이름만 기록
```

`simulation/logs/`, `simulation/videos/`, `simulation/runtime/`은 Git에 넣지 않는다.
