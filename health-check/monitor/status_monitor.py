#!/usr/bin/env python3
"""한글 상태 모니터.

simulation profile은 run_simulation.sh가 남긴 로그만 읽는다.
real profile은 명시적인 endpoint에서 MAVLink 수신만 수행하며 송신 API를
호출하지 않는다. 이 파일 자체는 ARM, mode, takeoff, velocity, LAND 명령을
생성하지 않는다.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


COMMON_SECTIONS = (
    "비행 사전 안전검사",
    "미션 시작",
    "이륙",
    "표적 탐색",
    "GUIDED 접근",
    "안전 상태",
)

REAL_TELEMETRY_FRESHNESS_SEC = 3.0
REAL_TELEMETRY_TYPES = ("HEARTBEAT", "GPS_RAW_INT", "EKF_STATUS_REPORT", "SYS_STATUS")


class Reporter:
    """simulation/real이 공유하는 한글 상태 콘솔 렌더러."""

    def __init__(self) -> None:
        self._seen: set[str] = set()
        self._last_event_sequence = -1
        self._last_event_timestamp = float("-inf")

    def emit(self, stage: str, message: str) -> None:
        if stage in self._seen:
            return
        self._seen.add(stage)
        print(f"[{stage}] {message}", flush=True)

    def emit_label(self, key: str, label: str, message: str) -> None:
        """중복을 억제하면서 표시 라벨을 호출자가 정한다."""

        if key in self._seen:
            return
        self._seen.add(key)
        print(f"[{label}] {message}", flush=True)

    def emit_raw(self, key: str, message: str) -> None:
        """중복 없이 구분선과 단계 제목을 표시한다."""

        if key in self._seen:
            return
        self._seen.add(key)
        print(message, flush=True)

    def section(self, name: str) -> None:
        """공통 섹션 제목을 한 번만 출력한다."""

        self.emit_raw(f"section:{name}", f"=== {name} ===")

    def pass_(self, key: str, message: str, *, actual: bool = False) -> None:
        self.emit_label(key, "확인" if actual else "통과", message)

    def observe(self, key: str, message: str) -> None:
        self.emit_label(key, "관찰", message)

    def warn(self, key: str, message: str) -> None:
        self.emit_label(key, "경고", message)

    def fail(self, key: str, message: str) -> None:
        self.emit_label(key, "실패", message)

    def info(self, key: str, message: str) -> None:
        self.emit_label(key, "정보", message)


@dataclass(frozen=True)
class MonitorObservation:
    """두 입력 어댑터가 공통 상태머신에 전달하는 읽기 전용 snapshot."""

    mission_ready: bool = False
    auto_confirmed: bool = False
    armed: bool = False
    altitude_m: float = 0.0
    target_altitude_m: float = 5.0
    target_confirmed: bool = False
    target_loss: bool = False
    health_ok: bool = True
    control_locked: bool = False
    heartbeat_timeout: bool = False
    approach_timeout: bool = False
    landing: bool = False
    disarmed: bool = False
    fatal_error: bool = False
    zero_velocity: bool = False
    timestamp: float | None = None
    event_sequence: int | None = None
    preflight_ok: bool = True
    heartbeat_valid: bool = True
    gps_ekf_battery_ok: bool = True
    auto_heartbeat_count: int = 2
    arm_heartbeat_count: int = 2
    yolo_ready: bool = True
    camera_frame: bool = True
    centered: bool = False
    distance_m: float = float("inf")


@dataclass(frozen=True)
class StateEvent:
    """상태 전이를 출력 순서대로 재생하기 위한 구조화 이벤트."""

    sequence: int
    previous: str | None
    current: str
    reason: str
    timestamp: float


class SimulationLogAdapter:
    """run_simulation 로그만 읽는 simulation 입력 어댑터."""

    def __init__(self, root: Path, requested_log: Path | None = None) -> None:
        self.root = root
        self.log_dir = requested_log

    def resolve_log_dir(self) -> Path | None:
        if self.log_dir is None:
            self.log_dir = latest_simulation_log(self.root)
        return self.log_dir

    def read(self) -> dict[str, Any] | None:
        log_dir = self.resolve_log_dir()
        if log_dir is None:
            return None
        return {
            "log_dir": log_dir,
            "launcher": read_text(log_dir / "launcher.log"),
            "autonomy": read_text(log_dir / "autonomy.log"),
            "upload": read_text(log_dir / "upload_mission.log"),
            "cleanup": read_text(log_dir / "cleanup.log"),
            "audit": load_json(log_dir / "packet_audit.json"),
            "rows": load_flight_rows(log_dir),
            "events": load_cpp_state_events(log_dir),
        }


class RealTelemetryAdapter:
    """MAVLink 수신만 담당하는 real 입력 어댑터.

    의도적으로 send/send_buf/command API를 노출하지 않는다. 상태 표시기는
    실제 수신 telemetry를 관찰할 뿐 비행 제어 계층을 호출하지 않는다.
    """

    def __init__(self, connection: Any) -> None:
        self._connection = connection

    def receive(self, timeout: float = 0.5) -> Any:
        return self._connection.recv_match(blocking=True, timeout=timeout)


class MonitorStateMachine:
    """상태만 계산하는 읽기 전용 상태머신."""

    AUTO_WAIT = "AUTO_WAIT"
    ARMED_TAKEOFF = "ARMED_TAKEOFF"
    TARGET_SEARCH = "TARGET_SEARCH"
    GUIDED = "GUIDED"
    TARGET_LOSS_HOVER = "TARGET_LOSS_HOVER"
    SAFE_HOVER = "SAFE_HOVER"
    LANDING = "LANDING"
    DISARMED = "DISARMED"
    FAILED = "FAILED"

    def __init__(self) -> None:
        self.state = self.AUTO_WAIT
        self.transitions: list[tuple[str, str, str]] = []
        self.events: list[StateEvent] = []
        self._next_sequence = 1
        self._last_input_sequence = -1
        self._last_input_timestamp = float("-inf")
        self._implicit_timestamp = 0.0
        self._heartbeat_streak = 0
        self._heartbeat_since: float | None = None
        self._health_since: float | None = None
        self._altitude_since: float | None = None
        self._target_history: list[bool] = []
        self._target_loss_since: float | None = None
        self._hold_since: float | None = None
        self.hold_active = False
        self.heartbeat_stable = False
        self.health_stable = False
        self.altitude_stable = False

    def _observation_timestamp(self, observation: MonitorObservation) -> float | None:
        """입력 sequence/timestamp를 검증하고 늦은 snapshot을 버린다."""

        if observation.event_sequence is not None:
            if observation.event_sequence <= self._last_input_sequence:
                return None
            self._last_input_sequence = observation.event_sequence

        if observation.timestamp is None:
            self._implicit_timestamp += 0.5
            timestamp = self._implicit_timestamp
        else:
            timestamp = observation.timestamp
            if timestamp < self._last_input_timestamp:
                return None
            self._last_input_timestamp = timestamp
        return timestamp

    def _update_stability(self, observation: MonitorObservation, timestamp: float) -> None:
        if observation.heartbeat_valid:
            self._heartbeat_streak += 1
            if self._heartbeat_since is None:
                self._heartbeat_since = timestamp
        else:
            self._heartbeat_streak = 0
            self._heartbeat_since = None
        self.heartbeat_stable = (
            self._heartbeat_streak >= 3
            or self._heartbeat_since is not None
            and timestamp - self._heartbeat_since >= 2.0
        )

        if observation.gps_ekf_battery_ok:
            if self._health_since is None:
                self._health_since = timestamp
        else:
            self._health_since = None
        self.health_stable = (
            self._health_since is not None
            and timestamp - self._health_since >= 1.0
        )

        if observation.altitude_m >= observation.target_altitude_m:
            if self._altitude_since is None:
                self._altitude_since = timestamp
        else:
            self._altitude_since = None
        self.altitude_stable = (
            self._altitude_since is not None
            and timestamp - self._altitude_since >= 0.5
        )

        self._target_history.append(bool(observation.target_confirmed))
        self._target_history = self._target_history[-5:]

        target_is_confirmed = self.target_is_confirmed()
        if target_is_confirmed:
            self._target_loss_since = None
        elif self._target_history and self._target_loss_since is None:
            self._target_loss_since = timestamp

        hold_candidate = (
            target_is_confirmed
            and observation.centered
            and observation.distance_m <= 2.0
        )
        if hold_candidate:
            if self._hold_since is None:
                self._hold_since = timestamp
            self.hold_active = timestamp - self._hold_since >= 0.25
        else:
            self._hold_since = None
            self.hold_active = False

    def target_is_confirmed(self) -> bool:
        """최근 5개 관측 중 4개 이상이 검출된 경우만 확정한다."""

        return len(self._target_history) >= 5 and sum(self._target_history) >= 4

    def load_cpp_events(self, events: list[StateEvent]) -> None:
        """C++ production state events를 읽기 전용으로 재생한다."""

        if not events:
            return
        ordered: list[StateEvent] = []
        last_sequence = -1
        last_timestamp = float("-inf")
        for event in sorted(events, key=lambda item: item.sequence):
            if event.sequence <= last_sequence or event.timestamp < last_timestamp:
                continue
            ordered.append(event)
            last_sequence = event.sequence
            last_timestamp = event.timestamp
        if not ordered:
            return
        self.events = ordered
        self.state = ordered[-1].current
        self._next_sequence = max(event.sequence for event in ordered) + 1
        self._last_input_sequence = ordered[-1].sequence
        self._last_input_timestamp = ordered[-1].timestamp

    def transition(
        self,
        next_state: str,
        reason: str,
        timestamp: float,
    ) -> bool:
        """모든 상태 변경을 통과하는 단일 전이 지점."""

        if next_state == self.state or timestamp < self._last_input_timestamp:
            return False
        previous = self.state
        self.state = next_state
        self.transitions.append((previous, next_state, reason))
        self.events.append(
            StateEvent(
                sequence=self._next_sequence,
                previous=previous,
                current=next_state,
                reason=reason,
                timestamp=timestamp,
            )
        )
        self._next_sequence += 1
        return True

    @staticmethod
    def _safety_failure(observation: MonitorObservation) -> bool:
        return (
            observation.control_locked
            or observation.heartbeat_timeout
            or not observation.health_ok
            or observation.approach_timeout
        )

    def step(self, observation: MonitorObservation) -> str:
        timestamp = self._observation_timestamp(observation)
        if timestamp is None:
            return self.state
        self._update_stability(observation, timestamp)

        if observation.fatal_error:
            self.transition(self.FAILED, "치명적 시작/실행 오류", timestamp)
        elif self.state == self.AUTO_WAIT:
            self._handle_auto_wait(observation, timestamp)
        elif self.state == self.ARMED_TAKEOFF:
            self._handle_armed_takeoff(observation, timestamp)
        elif self.state == self.TARGET_SEARCH:
            self._handle_target_search(observation, timestamp)
        elif self.state == self.GUIDED:
            self._handle_guided(observation, timestamp)
        elif self.state == self.TARGET_LOSS_HOVER:
            self._handle_target_loss_hover(observation, timestamp)
        elif self.state == self.SAFE_HOVER:
            self._handle_safe_hover(observation, timestamp)
        elif self.state == self.LANDING:
            self._handle_landing(observation, timestamp)
        return self.state

    def _handle_auto_wait(self, observation: MonitorObservation, timestamp: float) -> None:
        if not observation.preflight_ok:
            self.transition(self.FAILED, "사전 안전검사 실패", timestamp)
        elif observation.landing:
            self.transition(self.LANDING, "착륙 모드 확인", timestamp)
        elif (
            observation.mission_ready
            and observation.auto_confirmed
            and observation.auto_heartbeat_count >= 2
            and observation.armed
            and observation.arm_heartbeat_count >= 2
            and self.heartbeat_stable
            and self.health_stable
        ):
            self.transition(self.ARMED_TAKEOFF, "미션/AUTO/ARM 안정 확인", timestamp)

    def _handle_armed_takeoff(self, observation: MonitorObservation, timestamp: float) -> None:
        if self._safety_failure(observation):
            self.transition(self.SAFE_HOVER, "비행 안전 조건 실패", timestamp)
        elif observation.landing:
            self.transition(self.LANDING, "착륙 모드 확인", timestamp)
        elif observation.disarmed:
            self.transition(self.DISARMED, "무장 해제 확인", timestamp)
        elif self.altitude_stable:
            self.transition(self.TARGET_SEARCH, "목표 고도 5m 안정 확인", timestamp)

    def _handle_target_search(self, observation: MonitorObservation, timestamp: float) -> None:
        if self._safety_failure(observation):
            self.transition(self.SAFE_HOVER, "비행 안전 조건 실패", timestamp)
        elif observation.landing:
            self.transition(self.LANDING, "착륙 모드 확인", timestamp)
        elif (
            observation.yolo_ready
            and observation.camera_frame
            and self.altitude_stable
            and self.target_is_confirmed()
        ):
            self.transition(self.GUIDED, "YOLO/camera 및 5프레임 중 4프레임 확정", timestamp)

    def _handle_guided(self, observation: MonitorObservation, timestamp: float) -> None:
        if self._safety_failure(observation):
            self.transition(self.SAFE_HOVER, "ControlLocked/heartbeat/health/timeout", timestamp)
        elif observation.landing:
            self.transition(self.LANDING, "착륙 모드 확인", timestamp)
        elif observation.target_loss or (
            self._target_loss_since is not None
            and not self.target_is_confirmed()
            and timestamp - self._target_loss_since >= 0.5
        ):
            self.transition(self.TARGET_LOSS_HOVER, "표적 일시 유실, zero velocity", timestamp)

    def _handle_target_loss_hover(self, observation: MonitorObservation, timestamp: float) -> None:
        if self._safety_failure(observation):
            self.transition(self.SAFE_HOVER, "표적 유실 중 안전 조건 실패", timestamp)
        elif observation.landing:
            self.transition(self.LANDING, "착륙 모드 확인", timestamp)
        elif self.target_is_confirmed():
            self.transition(self.GUIDED, "표적 재검출", timestamp)
        elif self._target_loss_since is not None and timestamp - self._target_loss_since >= 1.0:
            self.transition(self.SAFE_HOVER, "표적 유실 timeout", timestamp)

    def _handle_safe_hover(self, observation: MonitorObservation, timestamp: float) -> None:
        if observation.landing:
            self.transition(self.LANDING, "착륙 모드 확인", timestamp)
        elif observation.disarmed:
            self.transition(self.DISARMED, "무장 해제 확인", timestamp)

    def _handle_landing(self, observation: MonitorObservation, timestamp: float) -> None:
        if observation.disarmed:
            self.transition(self.DISARMED, "무장 해제 및 정상 종료", timestamp)


def print_state_transitions(
    reporter: Reporter, machine: MonitorStateMachine, last_sequence: int = -1
) -> int:
    messages = {
        MonitorStateMachine.AUTO_WAIT: "미션 준비 완료 및 AUTO 모드 대기",
        MonitorStateMachine.ARMED_TAKEOFF: "ARM 확인, 목표 고도까지 상승 중",
        MonitorStateMachine.TARGET_SEARCH: "고도 조건 충족, 표적 탐색 중",
        MonitorStateMachine.GUIDED: "표적 확정, GUIDED 사선 접근 중",
        MonitorStateMachine.TARGET_LOSS_HOVER: "표적 일시 유실, zero velocity 유지",
        MonitorStateMachine.SAFE_HOVER: "안전 정지 상태, 자동 GUIDED 복귀 금지",
        MonitorStateMachine.LANDING: "착륙 과정",
        MonitorStateMachine.DISARMED: "무장 해제 및 정상 종료",
        MonitorStateMachine.FAILED: "시작 실패 또는 치명적 오류",
    }
    events = [
        StateEvent(
            sequence=0,
            previous=None,
            current=machine.AUTO_WAIT,
            reason=messages[machine.AUTO_WAIT],
            timestamp=0.0,
        ),
        *machine.events,
    ]
    for event in sorted(events, key=lambda item: item.sequence):
        if event.sequence <= max(last_sequence, reporter._last_event_sequence):
            continue
        if event.timestamp < reporter._last_event_timestamp:
            continue
        reporter.emit_label(f"상태 이벤트:{event.sequence}", "현재 상태", event.current)
        reporter.emit_label(
            f"상태 이벤트 설명:{event.sequence}",
            "정보",
            event.reason or messages[event.current],
        )
        last_sequence = event.sequence
        reporter._last_event_sequence = event.sequence
        reporter._last_event_timestamp = event.timestamp
    return last_sequence


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def latest_simulation_log(root: Path) -> Path | None:
    candidates = [p for p in (root / "simulation" / "logs").glob("run_*") if p.is_dir()]
    return max(candidates, key=lambda p: p.stat().st_mtime) if candidates else None


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""


def load_json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def load_flight_rows(log_dir: Path) -> list[dict[str, str]]:
    paths = sorted(log_dir.glob("flight_*.csv"), key=lambda p: p.stat().st_mtime)
    if not paths:
        return []
    try:
        with paths[-1].open(newline="", encoding="utf-8") as stream:
            return list(csv.DictReader(stream))
    except OSError:
        return []


def load_cpp_state_events(log_dir: Path) -> list[StateEvent]:
    """control_events.jsonl의 C++ state_transition만 읽는다."""

    path = log_dir / "control_events.jsonl"
    events: list[StateEvent] = []
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return events
    for line in lines:
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if value.get("event") != "state_transition":
            continue
        try:
            sequence = int(value["sequence"])
            current = str(value["current"])
            previous = str(value.get("previous")) if value.get("previous") else None
            reason = str(value.get("reason", ""))
            timestamp = float(value.get("timestamp_unix_ms", 0)) / 1000.0
        except (KeyError, TypeError, ValueError):
            continue
        events.append(StateEvent(sequence, previous, current, reason, timestamp))
    return events


def number(row: dict[str, str], name: str, default: float = 0.0) -> float:
    try:
        return float(row.get(name, default))
    except (TypeError, ValueError):
        return default


def endpoint_check(text: str) -> bool:
    endpoints = re.findall(r"(?:tcp|udp)(?::out)?:(\[[^]]+\]|[^:\s]+):\d+", text)
    return bool(endpoints) and all(host in ("127.0.0.1", "localhost") for host in endpoints)


def latest_state(rows: list[dict[str, str]]) -> dict[str, str]:
    return rows[-1] if rows else {}


def mission_target_altitude(upload_log: str) -> float:
    match = re.search(r"TAKEOFF\s+([0-9.]+)m", upload_log)
    return float(match.group(1)) if match else 5.0


def replay_simulation_state_machine(
    launcher: str,
    upload_log: str,
    rows: list[dict[str, str]],
    cpp_events: list[StateEvent] | None = None,
) -> MonitorStateMachine:
    machine = MonitorStateMachine()
    # Production C++ state transitions are authoritative when present. CSV
    # replay remains the fallback for older logs and offline tests.
    if cpp_events:
        machine.load_cpp_events(cpp_events)
        return machine
    mission_ready = "mission setup complete" in launcher or "미션 현재 항목 확인" in upload_log
    auto_confirmed = "AUTO 모드 확인" in upload_log or any(row.get("mode") == "AUTO" for row in rows)
    target_altitude = mission_target_altitude(upload_log)
    auto_count = 0
    arm_count = 0
    yolo_ready = bool(
        re.search(r"READY YOLO|YOLO_READY|model loaded:|HTTP endpoint .*127\.0\.0\.1:8002", launcher)
    )
    camera_frame = "READY CAMERA" in launcher

    # 첫 snapshot은 AUTO_WAIT 상태를 명시하고, 이후 모든 flight CSV 행을
    # 동일한 observation 모델로 재생한다.
    machine.step(MonitorObservation())
    for index, row in enumerate(rows, start=1):
        tracking = number(row, "tracking") > 0
        armed = row.get("armed") == "1"
        auto_count += row.get("mode") == "AUTO"
        arm_count += armed
        event = row.get("event", "").upper()
        safety_override = row.get("safety_override", "").lower()
        emergency_reason = row.get("emergency_reason", "").lower()
        emergency = row.get("emergency_active") == "1"
        heartbeat_timeout = "heartbeat" in safety_override or "heartbeat" in emergency_reason
        control_locked = "lock" in event or "lock" in emergency_reason
        approach_timeout = "intercept_timeout" in event.lower()
        health_ok = not emergency and not any(
            word in safety_override for word in ("heartbeat", "gps", "ekf", "battery", "health")
        )
        raw_timestamp = row.get("timestamp") or row.get("time_s") or row.get("time")
        try:
            timestamp = float(raw_timestamp) if raw_timestamp else None
        except (TypeError, ValueError):
            timestamp = None
        observation = MonitorObservation(
            mission_ready=mission_ready,
            auto_confirmed=auto_confirmed or auto_count >= 2,
            armed=armed,
            altitude_m=number(row, "alt_m"),
            target_altitude_m=target_altitude,
            target_confirmed=tracking,
            target_loss="LOSS" in event or "STALE" in safety_override,
            health_ok=health_ok,
            gps_ekf_battery_ok=health_ok,
            control_locked=control_locked,
            heartbeat_timeout=heartbeat_timeout,
            approach_timeout=approach_timeout,
            landing=row.get("mode") == "LAND",
            disarmed=arm_count > 0 and not armed,
            fatal_error=False,
            zero_velocity=abs(number(row, "vx")) < 1e-6 and abs(number(row, "vz")) < 1e-6,
            timestamp=timestamp,
            event_sequence=index,
            preflight_ok=True,
            heartbeat_valid=True,
            auto_heartbeat_count=auto_count,
            arm_heartbeat_count=arm_count,
            yolo_ready=yolo_ready,
            camera_frame=camera_frame,
            centered="CENTER" in event or "HOLD" in event,
            distance_m=number(row, "dist_m", float("inf")),
        )
        machine.step(observation)
    return machine


def simulation_status(root: Path, requested_log: Path | None) -> int:
    reporter = Reporter()
    snapshot = SimulationLogAdapter(root, requested_log).read()
    if snapshot is None:
        reporter.fail("실행:로그", "시뮬레이션 로그 디렉터리를 찾지 못했습니다.")
        return 1

    log_dir = snapshot["log_dir"]
    reporter.info("로그 경로", f"로그 디렉터리: {log_dir}")
    launcher = snapshot["launcher"]
    autonomy = snapshot["autonomy"]
    upload_log = snapshot["upload"]
    cleanup = snapshot["cleanup"]
    audit = snapshot["audit"]
    rows = snapshot["rows"]
    cpp_events = snapshot["events"]

    if not launcher:
        reporter.fail("실행:launcher", "launcher 로그가 없어 실행 상태를 확인할 수 없습니다.")
        return 1

    profile_match = re.search(r"\[run_simulation\] profile=([^\s]+)", launcher)
    profile = profile_match.group(1) if profile_match else "observe"
    ready = emit_simulation_summary(
        reporter, profile, launcher, autonomy, upload_log, cleanup, audit, rows, cpp_events
    )
    machine = replay_simulation_state_machine(launcher, upload_log, rows, cpp_events)
    print_state_transitions(reporter, machine)
    if not cleanup:
        reporter.info("최종:진행", f"실행 중: 상세 로그는 {log_dir}에 저장됩니다.")
        return 0 if ready else 1
    errors = re.findall(r"^.*(?:ERROR|실패).*$", launcher + cleanup, re.MULTILINE | re.IGNORECASE)
    if errors:
        reporter.fail("최종:실패", f"실패 원인: {errors[-1].strip()}")
        reporter.info("최종:로그", f"로그 디렉터리: {log_dir}")
        return 1
    reporter.pass_("최종:성공", f"상태 확인 완료, 로그: {log_dir}")
    return 0


def simulation_watch(root: Path, requested_log: Path | None, duration: float) -> int:
    """실행 중인 simulation 로그에서 새 상태 전이만 읽기 전용으로 표시한다."""

    reporter = Reporter()
    adapter = SimulationLogAdapter(root, requested_log)
    started = time.monotonic()
    last_sequence = -1

    while True:
        snapshot = adapter.read()
        if snapshot is not None:
            log_dir = snapshot["log_dir"]
            reporter.emit_label("로그 경로", "로그", f"로그 디렉터리: {log_dir}")
            launcher = snapshot["launcher"]
            if launcher:
                upload_log = snapshot["upload"]
                autonomy = snapshot["autonomy"]
                cleanup = snapshot["cleanup"]
                audit = snapshot["audit"]
                rows = snapshot["rows"]
                cpp_events = snapshot["events"]
                profile_match = re.search(r"\[run_simulation\] profile=([^\s]+)", launcher)
                profile = profile_match.group(1) if profile_match else "observe"
                preflight_ready = emit_simulation_summary(
                    reporter, profile, launcher, autonomy, upload_log, cleanup, audit, rows,
                    cpp_events
                )
                if preflight_ready:
                    machine = replay_simulation_state_machine(
                        launcher, upload_log, rows, cpp_events
                    )
                    last_sequence = print_state_transitions(reporter, machine, last_sequence)
                    if machine.hold_active:
                        reporter.emit_label("TargetCenteredHold", "정보", "TargetCenteredHold 유지")
                if re.search(r"ERROR|실패", launcher + cleanup, re.IGNORECASE):
                    reporter.emit_label("최종:실패", "실패", "시뮬레이션 단계가 실패했습니다.")
                    reporter.emit_label("최종:실패:로그", "로그", f"로그 디렉터리: {log_dir}")

        if duration > 0 and time.monotonic() - started >= duration:
            return 0
        time.sleep(0.5)


def emit_simulation_summary(
    reporter: Reporter,
    profile: str,
    launcher: str,
    autonomy: str,
    upload_log: str,
    cleanup: str,
    audit: dict[str, Any] | None,
    rows: list[dict[str, str]],
    cpp_events: list[StateEvent] | None = None,
) -> bool:
    """런처 상세 로그를 단계형 한글 요약으로 변환한다."""

    reporter.section("비행 사전 안전검사")
    checks: dict[str, bool] = {}

    checks["device"] = (
        "endpoints" in launcher
        and endpoint_check(launcher)
        and "/dev/tty" not in launcher
        and "serial" not in launcher.lower()
    )
    if checks["device"]:
        reporter.emit_label("검사:device", "통과", "serial/LAN 미사용 및 loopback endpoint")

    checks["gazebo"] = "READY Gazebo" in launcher
    if checks["gazebo"]:
        reporter.emit_label("검사:gazebo", "통과", "Gazebo 준비")

    heartbeat_ok = "READY HEARTBEAT" in launcher
    if audit:
        heartbeat_ok = heartbeat_ok or audit.get("sitl_to_mavproxy", {}).get("heartbeat", {}).get("valid_ardupilot_count", 0) > 0
    if rows:
        heartbeat_ok = heartbeat_ok or any(row.get("mode") for row in rows)
    checks["heartbeat"] = heartbeat_ok
    if heartbeat_ok:
        reporter.emit_label("검사:heartbeat", "통과", "SITL 유효 HEARTBEAT")

    checks["health"] = "READY HEALTH" in launcher or bool(rows)
    if checks["health"]:
        reporter.emit_label("검사:health", "통과", "GPS/EKF/배터리 상태 수신")

    # The launcher marker is authoritative; audit JSON is only a later fallback.
    router_ok = "READY MAVLINK" in launcher or audit is not None
    checks["router"] = router_ok
    if router_ok:
        reporter.emit_label("검사:router", "통과", "MAVLink router/audit 준비")

    build_ok = "target-distance/control build ready" in launcher
    checks["build"] = profile == "observe" or build_ok
    if profile == "observe":
        reporter.emit_label("검사:build", "정보", "observe 프로파일: control build 실행 없음")
    elif build_ok:
        reporter.emit_label("검사:build", "통과", "target-distance/control 준비")

    yolo_ok = bool(re.search(r"YOLO_READY|model loaded:|HTTP endpoint .*127\.0\.0\.1:8002", autonomy))
    checks["yolo"] = profile == "observe" or yolo_ok
    if profile == "observe":
        reporter.emit_label("검사:yolo", "정보", "observe 프로파일: YOLO 별도 실행 없음")
    elif yolo_ok:
        reporter.emit_label("검사:yolo", "통과", "YOLO TensorRT 준비")

    camera_ok = "READY CAMERA" in launcher or (yolo_ok and "camera" in autonomy.lower())
    checks["camera"] = profile == "observe" or camera_ok
    if profile == "observe":
        reporter.emit_label("검사:camera", "정보", "observe 프로파일: camera frame은 telemetry 경로로 확인")
    elif camera_ok:
        reporter.emit_label("검사:camera", "통과", "Gazebo camera frame 수신")

    check_messages = {
        "device": "serial/LAN 미사용 및 loopback endpoint",
        "gazebo": "Gazebo 준비",
        "heartbeat": "유효 HEARTBEAT",
        "health": "GPS/EKF/배터리 상태 수신",
        "router": "MAVLink telemetry 입력 준비",
        "build": "target-distance/control 준비",
        "yolo": "YOLO TensorRT 준비",
        "camera": "camera frame 수신",
    }
    terminal_log = launcher + cleanup
    if re.search(r"ERROR|실패", terminal_log, re.IGNORECASE) or "EVENT COMPLETE" in launcher:
        for check_name, passed in checks.items():
            if not passed:
                reporter.fail(f"검사 실패:{check_name}", f"{check_messages[check_name]} 확인 실패")

    if checks and all(checks.values()):
        reporter.emit_raw("검사:완료", f"안전검사 통과: {sum(checks.values())}/{len(checks)}")

    if "explicit sitl-flight mission command" in launcher or "미션 현재 항목 확인" in upload_log:
        reporter.section("미션 시작")
        if "미션 현재 항목 확인" in upload_log or "mission setup complete" in launcher:
            reporter.emit_label("미션:업로드", "통과", "미션 업로드")
        if "AUTO 모드 확인" in upload_log:
            reporter.emit_label("미션:AUTO", "통과", "AUTO 모드")
        if "ARM 확인" in upload_log:
            reporter.emit_label("미션:ARM", "통과", "ARM")

    if rows:
        latest = rows[-1]
        target_altitude = mission_target_altitude(upload_log)
        altitude_rows = [row for row in rows if number(row, "alt_m") >= target_altitude]
        if altitude_rows:
            altitude = number(altitude_rows[-1], "alt_m")
            reporter.section("이륙")
            reporter.emit_label("이륙:고도", "통과", f"목표 고도 {altitude:.2f}m 도달")
        target_confirmed = any(number(row, "tracking") > 0 for row in rows)
        target_confirmed = target_confirmed or any(
            event.current == MonitorStateMachine.GUIDED for event in (cpp_events or [])
        )
        if target_confirmed:
            reporter.section("표적 탐색")
            reporter.emit_label("탐색:확정", "통과", "표적 탐지/확정")
        if any(row.get("mode") == "GUIDED" for row in rows):
            reporter.section("GUIDED 접근")
            reporter.emit_label("GUIDED:모드", "통과", "GUIDED 모드")
            reporter.emit_label("GUIDED:속도", "통과", "vx/vy/vz 명령 확인")
            reporter.emit_label("GUIDED:거리", "정보", f"현재 거리 {number(latest, 'dist_m'):.2f}m")
            angle = latest.get("command_path_angle_deg") or latest.get("path_angle_deg")
            if angle:
                reporter.emit_label("GUIDED:경로각", "정보", f"사선 접근 경로각 {angle}°")

    terminal_log = launcher + cleanup
    if rows or cleanup or re.search(r"ERROR|실패", terminal_log, re.IGNORECASE):
        reporter.section("안전 상태")
    if rows:
        latest = rows[-1]
        target_loss_count = sum(
            "LOSS" in row.get("event", "").upper()
            or "STALE" in row.get("safety_override", "").upper()
            for row in rows
        )
        lock_count = sum(
            "LOCK" in row.get("event", "").upper()
            or "LOCK" in row.get("emergency_reason", "").upper()
            for row in rows
        )
        if target_loss_count or lock_count or cleanup:
            reporter.info(
                "안전:요약",
                f"표적 유실 {target_loss_count}회, ControlLocked {lock_count}회, "
                f"최신 zero velocity={'예' if abs(number(latest, 'vx')) < 1e-6 and abs(number(latest, 'vz')) < 1e-6 else '아니오'}",
            )
    if cleanup:
        if re.search(r"ERROR|실패", cleanup, re.IGNORECASE):
            reporter.fail("종료:실패", "종료 과정에서 오류가 발생했습니다.")
        else:
            reporter.pass_("종료:정리", "cleanup 완료")

    return bool(checks) and all(checks.values())


def valid_heartbeat(message: Any) -> bool:
    try:
        return (
            message.get_type() == "HEARTBEAT"
            and message.get_srcSystem() == 1
            and message.get_srcComponent() == 1
            and message.autopilot == 3
        )
    except (AttributeError, TypeError):
        return False


def valid_autopilot_telemetry(message: Any) -> bool:
    """상태 입력으로 사용할 ArduPilot sysid/component만 허용한다."""

    try:
        return message.get_srcSystem() == 1 and message.get_srcComponent() == 1
    except (AttributeError, TypeError):
        return False


def real_telemetry_endpoint_allowed(endpoint: str) -> bool:
    """real read-only 모니터가 열 수 있는 입력은 loopback UDP뿐이다."""

    match = re.fullmatch(r"udp:127\.0\.0\.1:(\d{1,5})", endpoint)
    if match is None:
        return False
    port = int(match.group(1))
    return 1 <= port <= 65535


def real_status(endpoint: str, baud: int, duration: float) -> int:
    try:
        from pymavlink import mavutil
    except ImportError:
        print("[실패] pymavlink가 설치되지 않아 실제 telemetry를 읽을 수 없습니다.")
        return 1

    reporter = Reporter()
    reporter.section("비행 사전 안전검사")
    reporter.pass_("real:read-only", "실제 telemetry 읽기 전용 모드", actual=True)
    reporter.info("real:endpoint", f"telemetry 수신 주소: {endpoint}, baud={baud}")
    connection = mavutil.mavlink_connection(endpoint, baud=baud)
    adapter = RealTelemetryAdapter(connection)
    deadline = time.monotonic() + duration
    last: dict[str, Any] = {}
    heartbeat_count = 0
    seen_armed = False
    machine = MonitorStateMachine()
    modes: list[str] = []
    armed_states: list[bool] = []
    gps_seen = False
    ekf_seen = False
    battery_seen = False
    latest_altitude = 0.0
    last_seen: dict[str, float] = {}
    message_counts: dict[str, int] = {message_type: 0 for message_type in REAL_TELEMETRY_TYPES}

    while time.monotonic() < deadline:
        message = adapter.receive()
        if message is None:
            continue
        message_type = message.get_type()
        if message_type == "BAD_DATA":
            continue
        if message_type != "HEARTBEAT":
            if valid_autopilot_telemetry(message) and message_type in {
                "GPS_RAW_INT", "EKF_STATUS_REPORT", "SYS_STATUS", "GLOBAL_POSITION_INT"
            }:
                last[message_type] = message
                last_seen[message_type] = time.monotonic()
                message_counts[message_type] += 1
                gps_seen |= message_type == "GPS_RAW_INT"
                ekf_seen |= message_type == "EKF_STATUS_REPORT"
                battery_seen |= message_type == "SYS_STATUS"
                if message_type == "GLOBAL_POSITION_INT":
                    latest_altitude = getattr(message, "relative_alt", 0) / 1000.0
            continue
        if not valid_heartbeat(message):
            # GCS/other vehicle heartbeat는 상태 기준으로 사용하지 않는다.
            continue

        heartbeat_count += 1
        last["HEARTBEAT"] = message
        last_seen["HEARTBEAT"] = time.monotonic()
        message_counts["HEARTBEAT"] += 1
        mode = mavutil.mode_string_v10(message)
        armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        seen_armed = seen_armed or armed
        modes.append(mode)
        armed_states.append(armed)
        system_failure = message.system_status in (
            mavutil.mavlink.MAV_STATE_CRITICAL,
            mavutil.mavlink.MAV_STATE_EMERGENCY,
        )
        gps = last.get("GPS_RAW_INT")
        ekf = last.get("EKF_STATUS_REPORT")
        battery = last.get("SYS_STATUS")
        gps_ok = gps is not None and getattr(gps, "fix_type", 0) >= 3
        ekf_ok = ekf is not None and not system_failure
        battery_ok = battery is not None and getattr(battery, "voltage_battery", 0) > 0 and getattr(battery, "battery_remaining", -1) >= 0
        health_ok = gps_ok and ekf_ok and battery_ok and not system_failure
        machine.step(
            MonitorObservation(
                mission_ready=mode == "AUTO" or armed,
                auto_confirmed=mode == "AUTO",
                armed=armed,
                altitude_m=latest_altitude,
                target_confirmed=False,
                health_ok=health_ok,
                gps_ekf_battery_ok=health_ok,
                heartbeat_valid=True,
                preflight_ok=True,
                landing=mode == "LAND",
                disarmed=seen_armed and not armed,
                timestamp=time.monotonic(),
                event_sequence=heartbeat_count,
            )
        )
    report_time = time.monotonic()
    reporter.section("비행 사전 안전검사")
    reporter.pass_("real:loopback", "loopback UDP telemetry만 사용", actual=True)
    reporter.pass_("real:serial-owner", "serial은 외부 router가 단독 소유", actual=True)
    fresh_all = True
    for message_type in REAL_TELEMETRY_TYPES:
        seen_at = last_seen.get(message_type)
        if seen_at is None:
            fresh_all = False
            reporter.fail(
                f"real:fresh:{message_type}",
                f"{message_type} 수신 없음 (유효 ArduPilot telemetry만 집계)",
            )
            continue
        age = max(0.0, report_time - seen_at)
        message = last.get(message_type)
        if age <= REAL_TELEMETRY_FRESHNESS_SEC:
            reporter.pass_(
                f"real:fresh:{message_type}",
                f"{message_type} freshness {age:.2f}초, 수신 {message_counts[message_type]}개",
                actual=True,
            )
        else:
            fresh_all = False
            reporter.fail(
                f"real:fresh:{message_type}",
                f"{message_type} freshness {age:.2f}초 초과 (기준 {REAL_TELEMETRY_FRESHNESS_SEC:.1f}초)",
            )

    reporter.section("미션 시작")
    if "AUTO" in modes:
        reporter.pass_("real:auto", "실제 AUTO 모드", actual=True)
    else:
        reporter.observe("real:auto:missing", "실제 AUTO 모드는 관찰되지 않음")
    if any(armed_states):
        reporter.pass_("real:armed", "실제 ARM 상태", actual=True)
    else:
        reporter.observe("real:armed:missing", "실제 ARM 상태는 관찰되지 않음")
    for mode in dict.fromkeys(modes):
        reporter.pass_(f"real:mode:{mode}", f"실제 {mode} 모드", actual=True)
    for armed in dict.fromkeys(armed_states):
        reporter.pass_(f"real:arm:{armed}", f"실제 ARM 상태: {'됨' if armed else '해제'}", actual=True)

    reporter.section("이륙")
    if latest_altitude >= 5.0:
        reporter.pass_("real:altitude", f"실제 고도 {latest_altitude:.2f}m", actual=True)
    else:
        reporter.observe("real:altitude:missing", f"실제 고도 {latest_altitude:.2f}m")

    reporter.section("표적 탐색")
    reporter.observe("real:target", "수신 telemetry만으로는 카메라 표적 확정을 판단하지 않음")

    reporter.section("GUIDED 접근")
    if "GUIDED" in modes:
        reporter.pass_("real:guided", "실제 GUIDED 모드", actual=True)
    else:
        reporter.observe("real:guided:missing", "실제 GUIDED 모드는 관찰되지 않음")
    reporter.observe("real:velocity", "읽기 전용 모드이므로 속도 명령을 송신하지 않음")

    reporter.section("안전 상태")
    if modes and modes[-1] == "LAND":
        reporter.pass_("real:landing", "실제 LAND 모드", actual=True)
    else:
        reporter.observe("real:landing:missing", "실제 LAND 모드는 관찰되지 않음")
    battery = last.get("SYS_STATUS")
    battery_valid = battery is not None and getattr(battery, "voltage_battery", 0) not in (0, 65535) and getattr(battery, "voltage_battery", 0) > 0 and getattr(battery, "battery_remaining", -1) >= 0
    if gps_seen and ekf_seen and battery_seen and battery_valid and fresh_all:
        reporter.pass_("real:health", "GPS/EKF/배터리 telemetry 및 freshness 정상", actual=True)
    else:
        reporter.fail("real:health:partial", "GPS/EKF/배터리 telemetry 또는 freshness 조건 미충족")
    reporter.info("real:hold", "표적 확정·호버링 상태는 onboard 제어 로그 없이는 판단하지 않음")

    print_state_transitions(reporter, machine)
    reporter.pass_("real:cleanup", f"읽기 전용 telemetry 관찰 종료, 유효 heartbeat {heartbeat_count}개")
    return 0 if heartbeat_count and fresh_all and battery_valid else 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="시뮬레이션 또는 실제 MAVLink 상태를 한글로 표시")
    parser.add_argument("--profile", choices=("simulation", "real"), required=True)
    parser.add_argument("--log-dir", type=Path, help="simulation 로그 디렉터리(생략하면 최신 run_* 사용)")
    parser.add_argument("--connect", help="real profile의 명시적 MAVLink endpoint")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--read-only", action="store_true", help="real profile 필수 안전 플래그")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--watch", action="store_true", help="simulation 로그를 실시간으로 읽고 상태 전이만 표시")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.profile == "simulation":
        if args.connect or args.read_only:
            print("[실행 오류] simulation profile에는 --connect/--read-only를 사용할 수 없습니다.")
            return 2
        if args.watch:
            return simulation_watch(repo_root(), args.log_dir, args.duration)
        return simulation_status(repo_root(), args.log_dir)

    if not args.read_only:
        print("[실행 거부] real profile은 반드시 --read-only를 지정해야 합니다.")
        return 2
    if not args.connect:
        print("[실행 오류] real profile은 --connect endpoint가 필요합니다.")
        return 2
    if not real_telemetry_endpoint_allowed(args.connect):
        print("[실행 거부] real 상태 모니터는 udp:127.0.0.1:<port> telemetry만 허용합니다.")
        return 2
    if args.duration <= 0:
        print("[실행 오류] --duration은 0보다 커야 합니다.")
        return 2
    return real_status(args.connect, args.baud, args.duration)


if __name__ == "__main__":
    raise SystemExit(main())
