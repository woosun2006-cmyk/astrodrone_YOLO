from connect import connect
from pymavlink import mavutil
import time

master = connect()

def set_mode(name):
    mode_id = master.mode_mapping()[name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    print(f"{name} 모드 전환 명령 보냄")

set_mode('GUIDED')
time.sleep(3)
set_mode('STABILIZE')
