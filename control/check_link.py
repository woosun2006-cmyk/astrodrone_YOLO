import argparse
import time

from pymavlink import mavutil


ARM_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


def is_armed_from_heartbeat(msg):
    return bool(msg.base_mode & ARM_FLAG)


def main():
    parser = argparse.ArgumentParser(
        description="Read-only MAVLink link/status check. Sends no arm or motor commands."
    )
    parser.add_argument("--address", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--heartbeat-timeout", type=float, default=20)
    parser.add_argument("--listen", type=float, default=8)
    args = parser.parse_args()

    master = mavutil.mavlink_connection(args.address, baud=args.baud)
    print(
        f"MAVLink heartbeat waiting: {args.address}, "
        f"baud={args.baud}, timeout={args.heartbeat_timeout}s"
    )

    heartbeat = master.wait_heartbeat(timeout=args.heartbeat_timeout)
    if heartbeat is None:
        raise TimeoutError("No heartbeat received. Check cable, port, baud rate, and power.")

    mode = mavutil.mode_string_v10(heartbeat)
    armed = is_armed_from_heartbeat(heartbeat)
    print(
        f"Connected: system={master.target_system}, "
        f"component={master.target_component}, mode={mode}, armed={armed}"
    )

    deadline = time.time() + args.listen
    while time.time() < deadline:
        msg = master.recv_match(
            type=["HEARTBEAT", "SYS_STATUS", "GPS_RAW_INT", "STATUSTEXT"],
            blocking=True,
            timeout=1,
        )
        if msg is None:
            continue

        msg_type = msg.get_type()
        if msg_type == "HEARTBEAT":
            print(
                "HEARTBEAT: "
                f"mode={mavutil.mode_string_v10(msg)}, "
                f"armed={is_armed_from_heartbeat(msg)}"
            )
        elif msg_type == "SYS_STATUS":
            print(
                "SYS_STATUS: "
                f"voltage={msg.voltage_battery / 1000.0:.2f}V, "
                f"battery={msg.battery_remaining}%"
            )
        elif msg_type == "GPS_RAW_INT":
            print(
                "GPS_RAW_INT: "
                f"fix_type={msg.fix_type}, "
                f"satellites={msg.satellites_visible}"
            )
        elif msg_type == "STATUSTEXT":
            print(f"STATUSTEXT: {msg.text}")


if __name__ == "__main__":
    main()
