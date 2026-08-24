#!/usr/bin/env python3
"""실기체 런타임의 읽기 전용 경계를 오프라인에서 검사한다."""

from __future__ import annotations

import json
import socket
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def real_fanout_endpoint_allowed(endpoint: str) -> bool:
    """The C++ health_check accepts only loopback UDP fan-out for this path."""
    prefix, separator, port = endpoint.rpartition(":")
    return separator == ":" and prefix == "udp:127.0.0.1" and port.isdigit() and 0 < int(port) <= 65535


class RealRuntimeOfflineTest(unittest.TestCase):
    def test_fake_udp_telemetry_is_loopback_and_receive_only(self) -> None:
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(1.0)
            port = receiver.getsockname()[1]
            self.assertTrue(real_fanout_endpoint_allowed(f"udp:127.0.0.1:{port}"))
            sender.sendto(b"FAKE_HEARTBEAT\nGPS_RAW_INT\nEKF_STATUS_REPORT\nSYS_STATUS", ("127.0.0.1", port))
            payload, address = receiver.recvfrom(4096)
            self.assertEqual(address[0], "127.0.0.1")
            self.assertIn(b"FAKE_HEARTBEAT", payload)
        finally:
            receiver.close()
            sender.close()

    def test_real_scripts_keep_serial_and_vehicle_writes_disabled(self) -> None:
        observe = (ROOT / "scripts" / "observe-real.sh").read_text(encoding="utf-8")
        shadow = (ROOT / "scripts" / "shadow-real.sh").read_text(encoding="utf-8")
        self.assertIn("ASTRODRONE_COMMAND_MODE=observe", observe)
        self.assertIn("ASTRODRONE_ALLOW_MAVLINK_WRITES=0", observe)
        self.assertIn("ASTRODRONE_ALLOW_VEHICLE_COMMANDS=0", observe)
        self.assertIn("ASTRODRONE_ALLOW_TELEMETRY_CONFIGURATION=0", observe)
        self.assertIn("--frame-source jetson", shadow)
        self.assertIn("ASTRODRONE_COMMAND_MODE=shadow", shadow)
        self.assertIn("ASTRODRONE_ALLOW_MAVLINK_WRITES=0", shadow)
        self.assertIn("ASTRODRONE_ALLOW_VEHICLE_COMMANDS=0", shadow)
        self.assertIn("health-check-cpp", observe)

    def test_shadow_audit_schema_distinguishes_blocked_from_sent(self) -> None:
        report = {
            "profile": "shadow-real",
            "decision_count": 4,
            "blocked_command_count": 4,
            "vehicle_affecting_command_count": 0,
            "direct_serial_write_count": 0,
            "set_message_interval_count": 0,
        }
        self.assertEqual(report["decision_count"], report["blocked_command_count"])
        self.assertEqual(report["vehicle_affecting_command_count"], 0)
        self.assertEqual(report["direct_serial_write_count"], 0)
        json.dumps(report)

    def test_flight_real_requires_explicit_guard_flags(self) -> None:
        flight = (ROOT / "scripts" / "flight-real.sh").read_text(encoding="utf-8")
        for flag in ("--target", "--allow-arm", "--confirm-real-flight", "--commands-enabled", "--serial-owner"):
            self.assertIn(flag, flight)
        self.assertIn("/dev/serial/by-id/", flight)
        self.assertIn('"$serial_owner" != onboard', flight)
        self.assertNotIn("router-owned", flight)
        self.assertNotIn("router_pid", flight)
        self.assertNotIn("--self-launch", flight)
        self.assertIn("AUTONOMY_MISSION_SETUP_REQUESTED=1", flight)

    def test_flight_real_starts_jetson_yolo_before_autonomy(self) -> None:
        flight = (ROOT / "scripts" / "flight-real.sh").read_text(encoding="utf-8")
        self.assertIn("run_yolo_live.sh", flight)
        self.assertIn("--frame-source jetson", flight)
        self.assertIn("YOLO_READY_TIMEOUT_SEC", flight)
        self.assertIn("yolo.stderr.log", flight)
        self.assertIn("autonomy-cpp", flight)
        self.assertLess(flight.index("run_yolo_live.sh"), flight.index("autonomy-cpp"))


if __name__ == "__main__":
    unittest.main()
