import time
import keyboard
from drone_lib import connect, set_mode, arm_disarm, takeoff, send_velocity, land

connection_address = "tcp:127.0.0.1:5762"
target_alt = 5.0

print("픽스호크 시뮬레이터 연결 중...")
connect(connection_address)

def change_mode():
    print("1. 모드 변경 확인")

    print("GUIDED 모드로 변경...")
    set_mode("GUIDED")
    time.sleep(5)

    print("AUTO 모드로 변경...")
    set_mode("AUTO")
    time.sleep(5)

    print("LAND 모드로 변경...")
    set_mode("LAND")
    time.sleep(5)


def try_arm():
    print("2. 시동 확인")

    print("시동 켜기...")
    arm_disarm(True)
    time.sleep(5)

    print("시동 끄기...")
    arm_disarm(False)
    time.sleep(5)

def take_off():
    print(f"3. 이륙(Takeoff) - 고도 {target_alt}m...")
    takeoff(target_alt)
    time.sleep(5)

def control_drone():
    print("\n=== 키보드 조종 시작 ===")
    print("[W/S] : 전진 / 후진")
    print("[A/D] : 좌 / 우 이동")
    print("[Space] : 상승 | [Shift] : 하강")
    print("[Q/E] : 좌회전 / 우회전")
    print("[L] : 착륙 후 종료")
    print("========================\n")

    speed = 1.5
    yaw_speed = 0.5

    while True:
        vx, vy, vz, yaw = 0.0, 0.0, 0.0, 0.0

        if keyboard.is_pressed('w'):
            vx = speed
        elif keyboard.is_pressed('s'):
            vx = -speed

        if keyboard.is_pressed('d'):
            vy = speed
        elif keyboard.is_pressed('a'):
            vy = -speed

        if keyboard.is_pressed('space'):
            vz = -speed
        elif keyboard.is_pressed('shift'):
            vz = speed

        if keyboard.is_pressed('q'):
            yaw = -yaw_speed
        elif keyboard.is_pressed('e'):
            yaw = yaw_speed

        if keyboard.is_pressed('l'):
            print("착륙 명령 수신. 조종을 종료합니다.")
            land()
            break

        send_velocity(vx, vy, vz, yaw)
        time.sleep(0.1)

set_mode("GUIDED")
time.sleep(3)
arm_disarm(True)
time.sleep(3)
take_off()
control_drone()
