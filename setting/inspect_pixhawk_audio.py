#!/usr/bin/env python3
"""Read-only inspection of Pixhawk settings relevant to DShot motor tones."""

from pymavlink import mavutil


DEVICE = "/dev/pixhawk"
BAUD = 115200
PARAMS = (
    "MOT_PWM_TYPE",
    "SERVO_DSHOT_ESC",
    "NTF_BUZZ_TYPES",
    "BRD_IO_DSHOT",
    "SERVO1_FUNCTION",
    "SERVO2_FUNCTION",
    "SERVO3_FUNCTION",
    "SERVO4_FUNCTION",
    "SERVO9_FUNCTION",
    "SERVO10_FUNCTION",
    "SERVO11_FUNCTION",
    "SERVO12_FUNCTION",
)


def main() -> None:
    link = mavutil.mavlink_connection(DEVICE, baud=BAUD)
    heartbeat = link.wait_heartbeat(timeout=10)
    if heartbeat is None:
        raise SystemExit("No Pixhawk heartbeat received")

    armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    print(
        f"heartbeat system={link.target_system} component={link.target_component} "
        f"vehicle_type={heartbeat.type} autopilot={heartbeat.autopilot} armed={armed}"
    )

    link.mav.autopilot_version_request_send(link.target_system, link.target_component)
    version = link.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=3)
    if version:
        major = version.flight_sw_version >> 24
        minor = (version.flight_sw_version >> 16) & 0xFF
        patch = (version.flight_sw_version >> 8) & 0xFF
        print(f"firmware={major}.{minor}.{patch}")

    for name in PARAMS:
        link.mav.param_request_read_send(
            link.target_system,
            link.target_component,
            name.encode("ascii"),
            -1,
        )
        message = link.recv_match(
            type="PARAM_VALUE",
            condition=f"PARAM_VALUE.param_id == '{name}'",
            blocking=True,
            timeout=2,
        )
        print(f"{name}={message.param_value:g}" if message else f"{name}=unavailable")

    link.close()


if __name__ == "__main__":
    main()
