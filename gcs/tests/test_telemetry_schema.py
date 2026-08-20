#!/usr/bin/env python3
import json
import unittest


class TelemetrySchemaTest(unittest.TestCase):
    def test_additive_target_metadata_and_read_only_fields(self):
        payload = {
            "protocol": "astrodrone-gcs-v1", "seq": 1,
            "state": "GUIDED", "mode": "GUIDED", "armed": False,
            "target_found": True, "target_confirmed": True,
            "target_x_px": 320, "target_y_px": 240,
            "target_width_px": 50, "target_height_px": 60,
            "bbox_x_px": 295, "bbox_y_px": 210,
            "bbox_width_px": 50, "bbox_height_px": 60,
            "frame_width": 640, "frame_height": 480,
            "target_confidence": 0.9, "frame_sequence": 100,
            "frame_timestamp_ns": 123, "frame_source": "jetson",
            "vx": None, "vy": None, "vz": None,
            "actual_vx": 0.2, "actual_vy": 0.0, "actual_vz": -0.2,
        }
        encoded = json.dumps(payload)
        decoded = json.loads(encoded)
        self.assertEqual(decoded["protocol"], "astrodrone-gcs-v1")
        self.assertEqual(decoded["frame_source"], "jetson")
        self.assertEqual(decoded["frame_width"], 640)
        self.assertEqual(decoded["frame_height"], 480)
        self.assertIsNone(decoded["vx"])
        self.assertEqual(decoded["actual_vz"], -0.2)

    def test_old_payload_without_metadata_remains_readable(self):
        old = json.loads('{"seq":3,"mode":"AUTO","armed":false}')
        self.assertEqual(old["mode"], "AUTO")
        self.assertIsNone(old.get("target_confidence"))

    def test_dashboard_uses_source_dimensions_and_preserves_aspect_ratio(self):
        dashboard = (
            __import__("pathlib").Path(__file__).resolve().parents[1]
            / "tools" / "dashboard.html"
        ).read_text()
        self.assertIn("t.frame_width ?? t.width ?? 640", dashboard)
        self.assertIn("Math.min(canvas.width / frameWidth, canvas.height / frameHeight)", dashboard)
        self.assertNotIn("canvas.width / 640", dashboard)
        self.assertNotIn("canvas.height / 480", dashboard)


if __name__ == "__main__":
    unittest.main()
