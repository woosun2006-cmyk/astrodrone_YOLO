from pymavlink import mavutil
from drone_lib import connect, set_mode, arm_disarm, takeoff, land, load_mavlink_settings
import time

settings = load_mavlink_settings()
connect(settings["real"]["proxy_udp"]["address"])
target_alt = 10

set_mode('GUIDED')
time.sleep(2)

arm_disarm(True)
print("시동 완료.")

takeoff(target_alt)

land()
print("착륙 중...")
