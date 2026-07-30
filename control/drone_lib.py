from pymavlink import mavutil
import os
import time
import yaml

master = None

SETTING_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "setting")
MAVLINK_SETTINGS_PATH = os.path.join(SETTING_DIR, "MAVLink.yaml")


def load_mavlink_settings():
    with open(MAVLINK_SETTINGS_PATH, encoding="utf-8") as f:
        return yaml.safe_load(f)

def connect(address, heartbeat_timeout=20):
    global master
    master = mavutil.mavlink_connection(address)
    print(f"MAVLink heartbeat 대기 중: {address} ({heartbeat_timeout}초 제한)")
    heartbeat = master.wait_heartbeat(timeout=heartbeat_timeout)
    if heartbeat is None:
        raise TimeoutError(
            "MAVLink heartbeat를 받지 못했습니다. "
            "Mission Planner SITL의 MAVLink UDP 출력을 Jetson 192.168.0.196:14561로 추가하세요."
        )
    print(f"연결됨. system = {master.target_system}")
    return master

def require_connection():
    if master is None:
        raise RuntimeError("드론 연결이 없습니다. 먼저 connect(address)를 호출하세요.")
    return master

def set_mode(mode):
    vehicle = require_connection()
    if mode not in vehicle.mode_mapping():
        print(f"지원하지 않는 모드입니다: {mode}")
        return False
    mode_id = vehicle.mode_mapping()[mode]
    vehicle.mav.set_mode_send(
        vehicle.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    return True

def arm_disarm(arm_bool):
    vehicle = require_connection()
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1 if arm_bool else 0,
        0, 0, 0, 0, 0, 0
    )

def takeoff(altitude):
    vehicle = require_connection()
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0,
        altitude
    )

def send_velocity(vx, vy, vz, yaw_rate=0):
    vehicle = require_connection()
    vehicle.mav.set_position_target_local_ned_send(
        0,
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # 속도와 yaw_rate만 사용하도록 마스킹
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, yaw_rate
    )

def land():
    vehicle = require_connection()
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
