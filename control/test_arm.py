import argparse
import time

from pymavlink import mavutil

from drone_lib import load_mavlink_settings

ARM_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
ARM_COMMAND = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM


def result_name(result):
    enum = mavutil.mavlink.enums.get("MAV_RESULT", {})
    entry = enum.get(result)
    return entry.name if entry else str(result)


def is_armed_from_heartbeat(msg):
    return bool(msg.base_mode & ARM_FLAG)


def connect(address, timeout):
    master = mavutil.mavlink_connection(address)
    print(f"MAVLink heartbeat 대기 중: {address} ({timeout}초 제한)")
    heartbeat = master.wait_heartbeat(timeout=timeout)
    if heartbeat is None:
        raise TimeoutError("heartbeat를 받지 못했습니다. 연결 주소/포트를 확인하세요.")

    armed = is_armed_from_heartbeat(heartbeat)
    mode = mavutil.mode_string_v10(heartbeat)
    print(
        f"연결됨: system={master.target_system}, "
        f"component={master.target_component}, mode={mode}, "
        f"armed={armed}"
    )
    return master


def set_mode(master, mode, timeout):
    mapping = master.mode_mapping()
    if mode not in mapping:
        print(f"모드 변경 건너뜀: 지원하지 않는 모드 {mode}")
        return False

    print(f"{mode} 모드 변경 명령 전송")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode],
    )

    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print(f"STATUSTEXT: {msg.text}")
            continue

        current_mode = mavutil.mode_string_v10(msg)
        armed = is_armed_from_heartbeat(msg)
        print(f"현재 상태: mode={current_mode}, armed={armed}")
        if current_mode == mode:
            print(f"{mode} 모드 확인됨")
            return True

    print(f"{mode} 모드 확인 실패")
    return False


def send_arm(master, should_arm):
    value = 1 if should_arm else 0
    label = "ARM" if should_arm else "DISARM"
    print(f"{label} 명령 전송")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        ARM_COMMAND,
        0,
        value,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def wait_arm_result(master, expected_armed, timeout):
    deadline = time.time() + timeout
    ack_seen = False
    last_armed = None

    while time.time() < deadline:
        msg = master.recv_match(
            type=["COMMAND_ACK", "HEARTBEAT", "STATUSTEXT"],
            blocking=True,
            timeout=1,
        )
        if msg is None:
            continue

        msg_type = msg.get_type()
        if msg_type == "STATUSTEXT":
            print(f"STATUSTEXT: {msg.text}")
            continue

        if msg_type == "COMMAND_ACK" and msg.command == ARM_COMMAND:
            ack_seen = True
            print(f"ARM ACK: {result_name(msg.result)} ({msg.result})")
            continue

        if msg_type == "HEARTBEAT":
            last_armed = is_armed_from_heartbeat(msg)
            mode = mavutil.mode_string_v10(msg)
            print(f"현재 상태: mode={mode}, armed={last_armed}")
            if last_armed == expected_armed:
                return True, ack_seen, last_armed

    return False, ack_seen, last_armed


def main():
    parser = argparse.ArgumentParser(description="MAVLink ARM/DISARM test")
    settings = load_mavlink_settings()
    parser.add_argument("--address", default=settings["sitl"]["local_tcp"]["address"])
    parser.add_argument("--heartbeat-timeout", type=float, default=20)
    parser.add_argument("--mode", default="GUIDED")
    parser.add_argument("--no-mode", action="store_true")
    parser.add_argument("--arm-timeout", type=float, default=15)
    parser.add_argument("--hold", type=float, default=3)
    parser.add_argument("--keep-armed", action="store_true")
    args = parser.parse_args()

    master = connect(args.address, args.heartbeat_timeout)

    if not args.no_mode:
        set_mode(master, args.mode, timeout=8)

    send_arm(master, should_arm=True)
    armed_ok, ack_seen, last_armed = wait_arm_result(
        master,
        expected_armed=True,
        timeout=args.arm_timeout,
    )

    if armed_ok:
        print("ARM 성공: heartbeat에서 armed=True 확인")
    else:
        print(
            "ARM 실패 또는 확인 실패: "
            f"ack_seen={ack_seen}, last_armed={last_armed}"
        )

    if args.keep_armed:
        print("keep-armed 옵션 때문에 DISARM은 보내지 않습니다.")
        return

    if args.hold > 0:
        print(f"{args.hold}초 대기 후 DISARM")
        time.sleep(args.hold)

    send_arm(master, should_arm=False)
    disarmed_ok, disarm_ack_seen, last_armed = wait_arm_result(
        master,
        expected_armed=False,
        timeout=args.arm_timeout,
    )

    if disarmed_ok:
        print("DISARM 성공: heartbeat에서 armed=False 확인")
    else:
        print(
            "DISARM 실패 또는 확인 실패: "
            f"ack_seen={disarm_ack_seen}, last_armed={last_armed}"
        )


if __name__ == "__main__":
    main()
