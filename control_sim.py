import time
import keyboard
from drone_lib import connect, set_mode, arm_disarm, takeoff, send_velocity, land

def main():
    connection_address = "tcp:127.0.0.1:5762"
    
    print("픽스호크 시뮬레이터 연결 중...")
    master = connect(connection_address)
    
    print("1. GUIDED 모드로 변경...")
    set_mode("GUIDED")
    time.sleep(1)
    
    print("2. 시동(ARM) 명령...")
    arm_disarm(True)
    time.sleep(2)
    
    print("3. 이륙(Takeoff) - 고도 3m...")
    takeoff(3.0)
    time.sleep(5)

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
            land(master)
            break

        send_velocity(vx, vy, vz, yaw)
        time.sleep(0.1)

if __name__ == "__main__":
    main()
