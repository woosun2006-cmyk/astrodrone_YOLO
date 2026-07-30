from pymavlink import mavutil
from drone_lib import connect, set_mode
import time

master = connect('udp:127.0.0.1:14550')
target_alt = 10

set_mode('GUIDED')
time.sleep(2)

arm_disarm(True)
print("시동 완료.")

takeoff(target_alt)

land()
print("착륙 중...")
