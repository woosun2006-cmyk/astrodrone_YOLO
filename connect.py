from pymavlink import mavutil

def connect(address='udp:127.0.0.1:14550'):
    master = mavutil.mavlink_connection(address)
    master.wait_heartbeat()
    print(f"연결됨. system = {master.target_system}")
    return master
