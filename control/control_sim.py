import time
import select
import sys
import termios
import tty
from drone_lib import connect, set_mode, arm_disarm, takeoff, send_velocity, land, load_mavlink_settings

settings = load_mavlink_settings()
connection_address = settings["sitl"]["local_tcp"]["address"]
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
    print("[Space] : 상승 | [X] : 하강")
    print("[Q/E] : 좌회전 / 우회전")
    print("[L] : 착륙 후 종료")
    print("========================\n")

    speed = 1.5
    yaw_speed = 0.5
    command_hold_sec = 0.35
    last_command_time = 0.0
    last_command = (0.0, 0.0, 0.0, 0.0)
    settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            now = time.time()
            vx, vy, vz, yaw = 0.0, 0.0, 0.0, 0.0

            readable, _, _ = select.select([sys.stdin], [], [], 0.05)
            if readable:
                key = sys.stdin.read(1).lower()

                if key == 'w':
                    vx = speed
                elif key == 's':
                    vx = -speed
                elif key == 'd':
                    vy = speed
                elif key == 'a':
                    vy = -speed
                elif key == ' ':
                    vz = -speed
                elif key == 'x':
                    vz = speed
                elif key == 'q':
                    yaw = -yaw_speed
                elif key == 'e':
                    yaw = yaw_speed
                elif key == 'l':
                    print("착륙 명령 수신. 조종을 종료합니다.")
                    land()
                    break

                last_command = (vx, vy, vz, yaw)
                last_command_time = now
            elif now - last_command_time <= command_hold_sec:
                vx, vy, vz, yaw = last_command

            send_velocity(vx, vy, vz, yaw)
            time.sleep(0.05)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        send_velocity(0.0, 0.0, 0.0, 0.0)

set_mode("GUIDED")
time.sleep(3)
arm_disarm(True)
time.sleep(3)
take_off()
control_drone()
