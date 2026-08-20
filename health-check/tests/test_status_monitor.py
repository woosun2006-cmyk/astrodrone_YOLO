#!/usr/bin/env python3
"""읽기 전용 상태머신 전이 테스트."""

import sys
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "monitor"))

from status_monitor import (  # noqa: E402
    COMMON_SECTIONS,
    MonitorObservation,
    MonitorStateMachine,
    Reporter,
    RealTelemetryAdapter,
    StateEvent,
    real_telemetry_endpoint_allowed,
    valid_autopilot_telemetry,
    valid_heartbeat,
    print_state_transitions,
)


class StatusMachineTest(unittest.TestCase):
    def to_guided(self) -> MonitorStateMachine:
        machine = MonitorStateMachine()
        machine.step(MonitorObservation(mission_ready=True, auto_confirmed=True, armed=True))
        machine.step(MonitorObservation(mission_ready=True, auto_confirmed=True, armed=True, altitude_m=5.0))
        machine.step(MonitorObservation(mission_ready=True, auto_confirmed=True, armed=True, altitude_m=5.0))
        for detected in (False, True, True, True, True):
            machine.step(
                MonitorObservation(
                    mission_ready=True,
                    auto_confirmed=True,
                    armed=True,
                    altitude_m=5.0,
                    target_confirmed=detected,
                    yolo_ready=True,
                    camera_frame=True,
                )
            )
        self.assertEqual(machine.state, MonitorStateMachine.GUIDED)
        return machine

    def test_normal_takeoff_and_guided_transition(self) -> None:
        machine = self.to_guided()
        states = [MonitorStateMachine.AUTO_WAIT] + [event.current for event in machine.events]
        self.assertEqual(
            states,
            [
                MonitorStateMachine.AUTO_WAIT,
                MonitorStateMachine.ARMED_TAKEOFF,
                MonitorStateMachine.TARGET_SEARCH,
                MonitorStateMachine.GUIDED,
            ],
        )

    def test_target_loss_hover_and_recovery(self) -> None:
        machine = self.to_guided()
        machine.step(MonitorObservation(target_loss=True, zero_velocity=True))
        self.assertEqual(machine.state, MonitorStateMachine.TARGET_LOSS_HOVER)
        for _ in range(4):
            machine.step(MonitorObservation(target_confirmed=True))
        self.assertEqual(machine.state, MonitorStateMachine.GUIDED)

    def test_safety_hover_is_latched(self) -> None:
        machine = self.to_guided()
        machine.step(MonitorObservation(control_locked=True, zero_velocity=True))
        self.assertEqual(machine.state, MonitorStateMachine.SAFE_HOVER)
        machine.step(MonitorObservation(target_confirmed=True, health_ok=True))
        self.assertEqual(machine.state, MonitorStateMachine.SAFE_HOVER)
        machine.step(MonitorObservation(landing=True))
        self.assertEqual(machine.state, MonitorStateMachine.LANDING)
        machine.step(MonitorObservation(disarmed=True))
        self.assertEqual(machine.state, MonitorStateMachine.DISARMED)

    def test_critical_start_error_fails_from_any_state(self) -> None:
        machine = MonitorStateMachine()
        machine.step(MonitorObservation(fatal_error=True))
        self.assertEqual(machine.state, MonitorStateMachine.FAILED)
        machine.step(MonitorObservation(mission_ready=True, auto_confirmed=True, armed=True))
        self.assertEqual(machine.state, MonitorStateMachine.FAILED)

    def test_landing_requires_disarm(self) -> None:
        machine = self.to_guided()
        machine.step(MonitorObservation(landing=True))
        self.assertEqual(machine.state, MonitorStateMachine.LANDING)
        machine.step(MonitorObservation(landing=True, disarmed=False))
        self.assertEqual(machine.state, MonitorStateMachine.LANDING)
        machine.step(MonitorObservation(disarmed=True))
        self.assertEqual(machine.state, MonitorStateMachine.DISARMED)

    def test_heartbeat_delay_holds_auto_wait(self) -> None:
        machine = MonitorStateMachine()
        for _ in range(2):
            machine.step(
                MonitorObservation(
                    mission_ready=True,
                    auto_confirmed=True,
                    armed=True,
                    heartbeat_valid=True,
                    gps_ekf_battery_ok=True,
                )
            )
        self.assertEqual(machine.state, MonitorStateMachine.AUTO_WAIT)
        machine.step(
            MonitorObservation(
                mission_ready=True,
                auto_confirmed=True,
                armed=True,
                heartbeat_valid=True,
                gps_ekf_battery_ok=True,
            )
        )
        self.assertEqual(machine.state, MonitorStateMachine.ARMED_TAKEOFF)

    def test_preflight_alone_keeps_auto_wait(self) -> None:
        machine = MonitorStateMachine()
        machine.step(
            MonitorObservation(
                preflight_ok=True,
                heartbeat_valid=True,
                gps_ekf_battery_ok=True,
            )
        )
        self.assertEqual(machine.state, MonitorStateMachine.AUTO_WAIT)

    def test_target_confirmation_waits_for_altitude(self) -> None:
        # Rebuild the relevant state without relying on a private state write:
        # a fresh altitude drop clears altitude stability while the machine
        # remains in the target-search phase only in a new run.
        machine = MonitorStateMachine()
        for _ in range(4):
            machine.step(
                MonitorObservation(
                    mission_ready=True,
                    auto_confirmed=True,
                    armed=True,
                    altitude_m=5.0,
                )
            )
        self.assertEqual(machine.state, MonitorStateMachine.TARGET_SEARCH)
        for detected in (True, True, True, True, True):
            machine.step(
                MonitorObservation(
                    mission_ready=True,
                    auto_confirmed=True,
                    armed=True,
                    altitude_m=0.0,
                    target_confirmed=detected,
                    yolo_ready=True,
                    camera_frame=True,
                )
            )
        self.assertEqual(machine.state, MonitorStateMachine.TARGET_SEARCH)

    def test_auto_and_arm_need_two_heartbeats(self) -> None:
        machine = MonitorStateMachine()
        for _ in range(3):
            machine.step(
                MonitorObservation(
                    mission_ready=True,
                    auto_confirmed=True,
                    armed=True,
                    auto_heartbeat_count=1,
                    arm_heartbeat_count=1,
                )
            )
        self.assertEqual(machine.state, MonitorStateMachine.AUTO_WAIT)
        machine.step(
            MonitorObservation(
                mission_ready=True,
                auto_confirmed=True,
                armed=True,
                auto_heartbeat_count=2,
                arm_heartbeat_count=2,
            )
        )
        self.assertEqual(machine.state, MonitorStateMachine.ARMED_TAKEOFF)

    def test_one_or_two_target_frames_missing_keeps_guided(self) -> None:
        machine = self.to_guided()
        machine.step(MonitorObservation(target_confirmed=False))
        machine.step(MonitorObservation(target_confirmed=False))
        self.assertEqual(machine.state, MonitorStateMachine.GUIDED)

    def test_target_loss_timeout_enters_safe_hover(self) -> None:
        machine = self.to_guided()
        for _ in range(3):
            machine.step(MonitorObservation(target_confirmed=False))
        self.assertEqual(machine.state, MonitorStateMachine.TARGET_LOSS_HOVER)
        for _ in range(3):
            machine.step(MonitorObservation(target_confirmed=False))
        self.assertEqual(machine.state, MonitorStateMachine.SAFE_HOVER)

    def test_late_sequence_is_ignored(self) -> None:
        machine = MonitorStateMachine()
        machine.step(MonitorObservation(preflight_ok=False, event_sequence=10))
        self.assertEqual(machine.state, MonitorStateMachine.FAILED)
        machine.step(
            MonitorObservation(
                preflight_ok=True,
                mission_ready=True,
                auto_confirmed=True,
                armed=True,
                event_sequence=9,
            )
        )
        self.assertEqual(machine.state, MonitorStateMachine.FAILED)

    def test_preflight_failure_blocks_mission_state(self) -> None:
        machine = MonitorStateMachine()
        machine.step(MonitorObservation(preflight_ok=False))
        self.assertEqual(machine.state, MonitorStateMachine.FAILED)

    def test_centered_hold_requires_distance_and_dwell(self) -> None:
        machine = self.to_guided()
        for _ in range(2):
            machine.step(
                MonitorObservation(
                    target_confirmed=True,
                    centered=True,
                    distance_m=2.0,
                )
            )
        self.assertTrue(machine.hold_active)

    def test_state_events_are_monotonic_and_printed_in_flight_order(self) -> None:
        machine = self.to_guided()
        machine.step(MonitorObservation(target_loss=True, zero_velocity=True))
        machine.step(MonitorObservation(landing=True, zero_velocity=True))
        machine.step(MonitorObservation(disarmed=True))

        states = [MonitorStateMachine.AUTO_WAIT] + [event.current for event in machine.events]
        self.assertEqual(
            states,
            [
                "AUTO_WAIT",
                "ARMED_TAKEOFF",
                "TARGET_SEARCH",
                "GUIDED",
                "TARGET_LOSS_HOVER",
                "LANDING",
                "DISARMED",
            ],
        )
        sequences = [event.sequence for event in machine.events]
        self.assertEqual(sequences, list(range(1, len(sequences) + 1)))

        output = StringIO()
        with redirect_stdout(output):
            print_state_transitions(Reporter(), machine)
        rendered = output.getvalue()
        positions = [rendered.index(f"[현재 상태] {state}") for state in states]
        self.assertEqual(positions, sorted(positions))
        self.assertNotIn("[상태:전이:", rendered)

    def test_common_sections_are_rendered_in_fixed_order(self) -> None:
        output = StringIO()
        reporter = Reporter()
        with redirect_stdout(output):
            for section in COMMON_SECTIONS:
                reporter.section(section)
        rendered = output.getvalue()
        positions = [rendered.index(f"=== {section} ===") for section in COMMON_SECTIONS]
        self.assertEqual(positions, sorted(positions))
        self.assertNotIn("[현재 상태]", rendered)

    def test_late_timestamp_event_is_ignored(self) -> None:
        machine = MonitorStateMachine()
        machine.load_cpp_events(
            [
                StateEvent(1, "AUTO_WAIT", "ARMED_TAKEOFF", "정상", 10.0),
                StateEvent(2, "ARMED_TAKEOFF", "TARGET_SEARCH", "늦은 이벤트", 9.0),
                StateEvent(3, "TARGET_SEARCH", "GUIDED", "정상", 11.0),
            ]
        )
        self.assertEqual([event.sequence for event in machine.events], [1, 3])
        self.assertEqual(machine.state, MonitorStateMachine.GUIDED)

    def test_real_adapter_only_receives(self) -> None:
        class FakeConnection:
            def __init__(self) -> None:
                self.received = 0
                self.sent = 0

            def recv_match(self, **_: object) -> str:
                self.received += 1
                return "telemetry"

            def send(self, *_: object) -> None:
                self.sent += 1

        connection = FakeConnection()
        adapter = RealTelemetryAdapter(connection)
        self.assertEqual(adapter.receive(), "telemetry")
        self.assertEqual(connection.sent, 0)
        self.assertFalse(hasattr(adapter, "send"))

    def test_real_endpoint_accepts_only_loopback_udp(self) -> None:
        self.assertTrue(real_telemetry_endpoint_allowed("udp:127.0.0.1:14551"))
        self.assertFalse(real_telemetry_endpoint_allowed("/dev/ttyACM0"))
        self.assertFalse(real_telemetry_endpoint_allowed("udp:192.168.0.10:14551"))
        self.assertFalse(real_telemetry_endpoint_allowed("tcp:127.0.0.1:14551"))

    def test_fake_udp_telemetry_fixture_filters_sources(self) -> None:
        class FakeMessage:
            def __init__(self, message_type: str, system: int, component: int, autopilot: int = 3) -> None:
                self._message_type = message_type
                self._system = system
                self._component = component
                self.autopilot = autopilot

            def get_type(self) -> str:
                return self._message_type

            def get_srcSystem(self) -> int:
                return self._system

            def get_srcComponent(self) -> int:
                return self._component

        ardupilot_heartbeat = FakeMessage("HEARTBEAT", 1, 1)
        gcs_heartbeat = FakeMessage("HEARTBEAT", 255, 190)
        gps = FakeMessage("GPS_RAW_INT", 1, 1)
        self.assertTrue(valid_heartbeat(ardupilot_heartbeat))
        self.assertFalse(valid_heartbeat(gcs_heartbeat))
        self.assertTrue(valid_autopilot_telemetry(gps))
        self.assertFalse(valid_autopilot_telemetry(gcs_heartbeat))


if __name__ == "__main__":
    unittest.main()
