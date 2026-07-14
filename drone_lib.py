from pymavlink import mavutil
import time

def connect(address):
    master = mavutil.mavlink_connection(address)
    master.wait_heartbeat()
    print(f"연결됨. system = {master.target_system}")
    return master

def set_mode(mode):
    if mode not in master.mode_mapping():
        print(f"지원하지 않는 모드입니다: {mode}")
        return False
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    return True

def arm_disarm(arm_bool):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1 if arm_bool else 0,
        0, 0, 0, 0, 0, 0
    )

def takeoff(altitude):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0,
        altitude
    )

def send_velocity(vx, vy, vz, yaw_rate=0):
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # 속도와 yaw_rate만 사용하도록 마스킹
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, yaw_rate
    )

def land():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
