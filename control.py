from pymavlink import mavutil
from connect import connect
import time

master = connect()

def set_mode(name):
    mode_id = master.mode_mapping()[name]
    master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)

set_mode('GUIDED')
time.sleep(2)

master.arducopter_arm()
master.motors_armed_wait()
print("시동 완료.")

target_alt = 10
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, target_alt)
print(f"{target_alt}m 이륙 중...")

while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    alt = msg.relative_alt / 1000
    print(f"현재 고도 : {alt:.1f}m")
    if alt >= target_alt * 0.95:
        print("목표 고도 도달")
        break

set_mode('LAND')
print("착륙 중...")
