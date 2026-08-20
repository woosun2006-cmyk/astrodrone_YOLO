#!/usr/bin/env python3
"""Upload and optionally start a simple TAKEOFF + WAYPOINT mission."""

import argparse
import os
import sys
import time

from pymavlink import mavutil


def wait_message(master, message_type, timeout):
    message = master.recv_match(type=message_type, blocking=True, timeout=timeout)
    if message is None:
        raise TimeoutError(f"timed out waiting for {message_type}")
    return message


def mission_item(master, seq, command, lat, lon, altitude, current):
    master.mav.mission_item_int_send(
        master.target_system,
        master.target_component,
        seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        command,
        current,
        1,
        0,
        0,
        0,
        0,
        int(round(lat * 1e7)),
        int(round(lon * 1e7)),
        altitude,
    )


def upload_mission(master, home_lat, home_lon, waypoint_lat, waypoint_lon, altitude):
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    wait_message(master, "MISSION_ACK", 5)

    # ArduPilot reserves mission sequence 0 for HOME.  The first user
    # command must therefore be sequence 1; upload an explicit home item so
    # TAKEOFF is not silently replaced by the home slot.
    master.mav.mission_count_send(master.target_system, master.target_component, 3)

    for _ in range(3):
        request = wait_message(master, ["MISSION_REQUEST_INT", "MISSION_REQUEST"], 5)
        if request.seq == 0:
            mission_item(
                master,
                0,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                home_lat,
                home_lon,
                0.0,
                0,
            )
        elif request.seq == 1:
            mission_item(
                master,
                1,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                home_lat,
                home_lon,
                altitude,
                0,
            )
        elif request.seq == 2:
            mission_item(
                master,
                2,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                waypoint_lat,
                waypoint_lon,
                altitude,
                0,
            )
        else:
            raise RuntimeError(f"unexpected mission sequence {request.seq}")

    ack = wait_message(master, "MISSION_ACK", 5)
    if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError(f"mission rejected: {ack.type}")


def leave_auto_before_upload(master, timeout):
    """Force an AUTO mode reinitialization when replacing a stopped mission."""
    heartbeat = wait_message(master, "HEARTBEAT", timeout)
    armed_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    if heartbeat.base_mode & armed_flag:
        raise RuntimeError("기체가 ARM 상태입니다. 미션 업로드 전에 DISARM하세요")

    modes = master.mode_mapping()
    if "LOITER" not in modes:
        raise RuntimeError("AUTO 재초기화를 위한 LOITER 모드를 지원하지 않습니다")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        modes["LOITER"],
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if heartbeat is not None and mavutil.mode_string_v10(heartbeat) == "LOITER":
            print("미션 업로드 전 LOITER 전환 확인")
            return
    raise TimeoutError("미션 업로드 전 LOITER 전환을 확인하지 못했습니다")


def set_mission_current(master, seq, timeout):
    """Make the first navigation item explicit before entering AUTO."""
    master.mav.mission_set_current_send(
        master.target_system,
        master.target_component,
        seq,
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        current = master.recv_match(type="MISSION_CURRENT", blocking=True, timeout=1)
        if current is not None and current.seq == seq:
            print(f"미션 현재 항목 확인: seq={seq}")
            return
    raise TimeoutError(f"미션 현재 항목 {seq} 확인을 못했습니다")


def verify_uploaded_mission(master, timeout):
    """Read back the mission and reject a HOME/waypoint-only upload."""
    master.mav.mission_request_list_send(
        master.target_system,
        master.target_component,
    )
    count = wait_message(master, "MISSION_COUNT", timeout)
    if count.count < 3:
        raise RuntimeError(
            f"업로드된 미션 항목 수가 {count.count}개입니다. "
            "HOME + TAKEOFF + WAYPOINT 3개가 필요합니다"
        )

    items = {}
    for seq in range(count.count):
        master.mav.mission_request_int_send(
            master.target_system,
            master.target_component,
            seq,
        )
        item = wait_message(master, "MISSION_ITEM_INT", timeout)
        items[item.seq] = item.command

    takeoff = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    waypoint = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    if items.get(1) != takeoff or items.get(2) != waypoint:
        raise RuntimeError(
            "미션 검증 실패: "
            f"seq1={items.get(1)}(TAKEOFF={takeoff}), "
            f"seq2={items.get(2)}(WAYPOINT={waypoint})"
        )
    print("미션 검증 완료: HOME -> TAKEOFF -> WAYPOINT")


def set_auto(master, timeout):
    modes = master.mode_mapping()
    if "AUTO" not in modes:
        raise RuntimeError("AUTO 모드를 지원하지 않습니다")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        modes["AUTO"],
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if heartbeat is not None and mavutil.mode_string_v10(heartbeat) == "AUTO":
            print("AUTO 모드 확인")
            return
    raise TimeoutError("AUTO 모드 전환을 확인하지 못했습니다")


def wait_for_gps_ready(master, timeout):
    """Wait for the simulated GPS to become a valid 3D fix before arming."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        gps = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1)
        if gps is None:
            continue
        if (
            gps.fix_type >= mavutil.mavlink.GPS_FIX_TYPE_3D_FIX
            and gps.satellites_visible >= 6
        ):
            print(
                f"GPS 준비 확인: fix={gps.fix_type} "
                f"satellites={gps.satellites_visible}"
            )
            # The SITL JSON backend reports both IMUs before the first
            # consistency window has settled. Keep the single pre-arm check
            # after a short stable-GPS/IMU period; do not retry ARM.
            settle_until = time.time() + min(5.0, max(0.0, timeout))
            while time.time() < settle_until:
                master.recv_match(
                    type=["HEARTBEAT", "GPS_RAW_INT", "SYS_STATUS"],
                    blocking=True,
                    timeout=1,
                )
            return
    raise TimeoutError("GPS 3D fix 준비를 확인하지 못했습니다")


def run_prearm_checks(master, timeout):
    """Run ArduPilot's checks once and require a fresh passing SYS_STATUS."""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    deadline = time.time() + timeout
    last_status = None
    while time.time() < deadline:
        message = master.recv_match(
            type=["SYS_STATUS", "STATUSTEXT", "COMMAND_ACK"],
            blocking=True,
            timeout=1,
        )
        if message is None:
            continue
        message_type = message.get_type()
        if message_type == "STATUSTEXT":
            last_status = message.text
            print(f"pre-arm: {message.text}")
            continue
        if message_type == "COMMAND_ACK":
            if message.command == mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS:
                print(f"pre-arm check ACK: result={message.result}")
            continue
        if message.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK:
            print("pre-arm check 통과")
            return
    detail = f": {last_status}" if last_status else ""
    raise TimeoutError(f"pre-arm check 통과를 확인하지 못했습니다{detail}")


def arm(master, timeout):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    deadline = time.time() + timeout
    armed_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    while time.time() < deadline:
        heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if heartbeat is not None and heartbeat.base_mode & armed_flag:
            print("ARM 확인")
            return
    raise TimeoutError("ARM 상태를 확인하지 못했습니다")


def request_global_position(master):
    """Enable the position stream on the direct SITL SERIAL1 endpoint."""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        1_000_000,
        0,
        0,
        0,
        0,
        0,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--connect",
        default=os.environ.get(
            "MISSION_UPLOAD_ENDPOINT",
            os.environ.get("SITL_SERIAL1_ENDPOINT", "tcp:127.0.0.1:5762"),
        ),
        help=(
            "bidirectional MAVLink upload endpoint (default: "
            "MISSION_UPLOAD_ENDPOINT, SITL_SERIAL1_ENDPOINT, or tcp:127.0.0.1:5762)"
        ),
    )
    parser.add_argument("--waypoint-lat", type=float, default=-35.363172)
    parser.add_argument("--waypoint-lon", type=float, default=149.165237)
    parser.add_argument("--altitude", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--auto", action="store_true", help="upload 후 AUTO 모드로 전환")
    parser.add_argument("--arm", action="store_true", help="AUTO 전환 후 ARM")
    args = parser.parse_args()

    master = mavutil.mavlink_connection(args.connect)
    print(f"MAVLink 연결 대기: {args.connect}")
    master.wait_heartbeat(timeout=args.timeout)
    print(
        f"연결됨: system={master.target_system} "
        f"component={master.target_component}"
    )

    request_global_position(master)
    position = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=args.timeout)
    if position is None or not position.lat or not position.lon:
        raise RuntimeError("현재 home GPS 위치를 받지 못했습니다")

    home_lat = position.lat / 1e7
    home_lon = position.lon / 1e7
    print(f"home={home_lat:.7f}, {home_lon:.7f}")
    print(
        f"mission: TAKEOFF {args.altitude:.1f}m -> "
        f"WAYPOINT {args.waypoint_lat:.7f}, {args.waypoint_lon:.7f} "
        f"alt={args.altitude:.1f}m"
    )

    leave_auto_before_upload(master, args.timeout)
    upload_mission(
        master,
        home_lat,
        home_lon,
        args.waypoint_lat,
        args.waypoint_lon,
        args.altitude,
    )
    verify_uploaded_mission(master, args.timeout)
    set_mission_current(master, 0, args.timeout)
    if args.arm and not args.auto:
        raise RuntimeError("--arm은 --auto와 함께 사용해야 합니다")
    if args.auto:
        set_auto(master, args.timeout)
    if args.arm:
        wait_for_gps_ready(master, args.timeout)
        run_prearm_checks(master, args.timeout)
        arm(master, args.timeout)
    if not args.auto:
        print("미션 업로드 완료. ARM/AUTO 명령은 보내지 않았습니다.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, TimeoutError, OSError) as exc:
        print(f"오류: {exc}", file=sys.stderr)
        raise SystemExit(1)
