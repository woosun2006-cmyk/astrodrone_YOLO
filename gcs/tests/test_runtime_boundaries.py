#!/usr/bin/env python3
import unittest
from pathlib import Path
import subprocess

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
        launcher = (ROOT / "simulation" / "scripts" / "run_simulation.sh").read_text()
        self.assertIn("GCS_TELEMETRY_UDP_PORT=14553", launcher)
        self.assertIn("health=udp:127.0.0.1:14554", launcher)
        self.assertIn('"router":false', launcher)
        self.assertNotIn("start_router.sh", launcher)
        self.assertNotIn("start_mavlink_audit.sh", launcher)

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
        self.assertIn("export GCS_VIDEO_ANNOTATED=1",
                      (ROOT / "scripts" / "cpp-yolo-autonomy-sitl").read_text())
        for name in ("observe-real.sh", "shadow-real.sh", "flight-real.sh"):
            self.assertIn('GCS_VIDEO_ANNOTATED="${GCS_VIDEO_ANNOTATED:-0}"',
                          (ROOT / "scripts" / name).read_text())

    def test_target_range_has_independent_gcs_fanout(self):
        target = (ROOT / "control" / "target_distance.cpp").read_text()
        sender = (ROOT / "gcs" / "sender" / "telem_sender_main.cpp").read_text()
        gcs = (ROOT / "setting" / "gcs.yaml").read_text()
        self.assertIn('gcs_udp_port', target)
        self.assertIn('flight_sender.send(out)', target)
        self.assertIn('gcs_sender.send(out)', target)
        self.assertIn('telem.get_long_or("target_udp_port", 15022)', sender)
        self.assertIn('target_udp_port: 15022', gcs)
        self.assertIn('command_state_udp_port: 15021', gcs)

    def test_real_launcher_does_not_open_serial_for_publisher(self):
        for name in ("observe-real.sh", "shadow-real.sh", "flight-real.sh"):
            source = (ROOT / "scripts" / name).read_text()
            self.assertNotIn("/dev/tty", source)
            self.assertIn("gcs/run_publisher.sh", source)

    def test_health_check_option_and_status_monitor_alias(self):
        launcher = ROOT / "simulation" / "scripts" / "run_simulation.sh"
        source = launcher.read_text()
        self.assertIn("--health-check|--status-monitor)", source)
        self.assertIn("scripts/health-check-cpp", source)
        self.assertNotIn("status_" + "monitor.py", source)

        for option in ("--health-check", "--status-monitor"):
            result = subprocess.run(
                [str(launcher), option, "--help"],
                cwd=ROOT,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("--health-check", result.stdout)
            self.assertIn("--status-monitor", result.stdout)

    def test_fly_gate_precedes_mission_commands_and_autonomy_is_terminal_mirrored(self):
        app_source = (ROOT / "control" / "app" / "flight_mission_app.cpp").read_text()
        launcher = (ROOT / "simulation" / "scripts" / "run_simulation.sh").read_text()
        approval = app_source.index("wait_for_fly_approval(std::cin, std::cout)")
        mission = app_source.index("mission_setup.run()")
        self.assertLess(approval, mission)
        self.assertIn('local autonomy_stdin=/dev/null', launcher)
        self.assertIn('autonomy_stdin=/dev/stdin', launcher)
        self.assertIn('> >(tee -a "$log_dir/$name.log") 2>&1 &', launcher)
        self.assertIn('setsid "$@" </dev/null >"$log_dir/$name.log" 2>&1 &', launcher)

    def test_preflight_detail_output_is_opt_in_and_vision_is_mandatory(self):
        source = (ROOT / "control" / "app" / "flight_mission_app.cpp").read_text()
        readiness = (ROOT / "control" / "app" / "preflight_readiness.cpp").read_text()
        self.assertIn('const bool verbose = env_flag_enabled("PREFLIGHT_VERBOSE");', source)
        self.assertIn('if (!force && !verbose) return;', source)
        self.assertIn('if (verbose) std::cerr << " 누적수신="', source)
        self.assertIn('preflight_policy.require_vision = true;', source)
        self.assertIn('YOLO_READY_FILE', source)
        self.assertIn('CAMERA_FRAME_READY_FILE', source)
        self.assertIn('CAMERA_SOURCE_READY_FILE', source)
        self.assertIn('http_endpoint_alive', source)
        self.assertIn('YOLO readiness marker 없음', readiness)
        self.assertIn('실제 frame 미수신', readiness)
        self.assertIn('카메라 topic 미수신', readiness)
        self.assertIn('frame stale', readiness)
        self.assertNotIn('YOLO/카메라 준비 필수 조건 아님', source)

    def test_real_launchers_mirror_flight_app_and_close_other_stdin(self):
        flight = (ROOT / "scripts" / "flight-real.sh").read_text()
        observe = (ROOT / "scripts" / "observe-real.sh").read_text()
        shadow = (ROOT / "scripts" / "shadow-real.sh").read_text()

        self.assertIn('> >(tee -a "$run_log/autonomy.log") 2>&1', flight)
        flight_app = flight[flight.index('"$SCRIPT_DIR/autonomy-cpp"'):]
        self.assertNotIn('</dev/null', flight_app)
        for source in (observe, shadow):
            self.assertIn('> >(tee -a "$run_log/autonomy.log") 2>&1 &', source)
            self.assertIn('</dev/null', source)
        self.assertIn('</dev/null > >(tee -a "$run_log/status-monitor.log")', observe)

        autonomy = (ROOT / "scripts" / "autonomy-cpp").read_text()
        yolo = (ROOT / "scripts" / "cpp-yolo-autonomy-sitl").read_text()
        launcher = (ROOT / "simulation" / "scripts" / "run_simulation.sh").read_text()
        self.assertIn('target-distance-cpp" "${target_args[@]}" </dev/null', autonomy)
        self.assertIn('run_yolo_live.sh"', yolo)
        self.assertIn('</dev/null >"$yolo_stdout_log"', yolo)
        self.assertIn('</dev/null > >(tee -a "$log_dir/status-monitor.log")', launcher)


if __name__ == "__main__":
    unittest.main()
