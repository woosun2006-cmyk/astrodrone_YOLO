#!/usr/bin/env python3
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


class RuntimeBoundaryTest(unittest.TestCase):
    def test_publisher_is_udp_only_and_does_not_configure_mavlink(self):
        source = (ROOT / "gcs" / "sender" / "telem_sender_main.cpp").read_text()
        self.assertIn("is_loopback_udp", source)
        self.assertIn("open_connection(args.telemetry_endpoint)", source)
        self.assertNotIn("MAV_CMD_SET_MESSAGE_INTERVAL", source)
        self.assertNotIn("request_message_interval", source)
        self.assertNotIn("open_connection(serial", source)
        self.assertIn("GCS_TELEMETRY_HOST", source)
        self.assertIn("GCS_TELEMETRY_PORT", source)

    def test_simulation_has_separate_gcs_telemetry_fanout(self):
        router = (ROOT / "simulation" / "scripts" / "start_router.sh").read_text()
        launcher = (ROOT / "simulation" / "scripts" / "run_simulation.sh").read_text()
        self.assertIn("ENABLE_GCS_TELEMETRY_OUTPUT", router)
        self.assertIn("GCS_TELEMETRY_UDP_PORT=14553", launcher)
        self.assertIn("GCS_telemetry=14553", launcher)

    def test_gcs_video_is_best_effort_and_control_is_not_started_by_viewer(self):
        source = (ROOT / "YOLO_MODEL" / "cpp" / "gcs_video_publisher.cpp").read_text()
        yolo = (ROOT / "YOLO_MODEL" / "cpp" / "yolo_live.cpp").read_text()
        launcher = (ROOT / "scripts" / "cpp-yolo-autonomy-sitl").read_text()
        self.assertIn("queue_.size() >= 2", source)
        self.assertIn("sendto", source)
        self.assertNotIn("MAVLink", source)
        self.assertIn("GCS_VIDEO_ENABLED", launcher)
        self.assertIn("video_publisher->submit(frame,", yolo)
        self.assertNotIn("video_publisher->submit(annotated", yolo)
        self.assertIn("GCS_VIDEO_ANNOTATED", yolo)
        self.assertIn("frame_width", yolo)
        for name in ("cpp-yolo-autonomy-sitl", "observe-real.sh", "shadow-real.sh"):
            self.assertIn('GCS_VIDEO_ANNOTATED="${GCS_VIDEO_ANNOTATED:-0}"',
                          (ROOT / "scripts" / name).read_text())

    def test_real_launcher_does_not_open_serial_for_publisher(self):
        for name in ("observe-real.sh", "shadow-real.sh", "flight-real.sh"):
            source = (ROOT / "scripts" / name).read_text()
            self.assertNotIn("/dev/tty", source)
            self.assertIn("gcs/run_publisher.sh", source)


if __name__ == "__main__":
    unittest.main()
