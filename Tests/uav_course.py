from dronekit import connect
import math, time, math

connection_string = "tcp:127.0.0.1:5762" 

print("\nConnecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True)
vehicle.wait_ready('autopilot_version')

def get_flight_direction(vehicle, speed_threshold=0.5):
    """
    計算無人機實際飛行方向（地面軌跡方向）
    當速度小於 speed_threshold(m/s)時，返回 None
    ehicle.velocity  -> [vx, vy, vz] 
    vx: Velocity in the North direction (m/s)
    vy: Velocity in the East direction (m/s)
    vz: Velocity in the Down direction (m/s)
    """
    vx, vy, vz = vehicle.velocity  # m/s
    horizontal_speed = math.sqrt(vx**2 + vy**2)
    if horizontal_speed < speed_threshold:
        return None  # 無意義的方向（幾乎靜止）
    # 計算地面上的飛行方向角（從正北順時針）
    angle_rad = math.atan2(vy, vx)
    angle_deg = (math.degrees(angle_rad) + 360) % 360
    return angle_deg

try:
    while True:
        direction = get_flight_direction(vehicle)
        if direction is not None:
            print(f"目前飛行方向：{direction:.3f}°")
        else:
            print("無人機靜止中，無法判斷飛行方向")
        time.sleep(3)
except KeyboardInterrupt:
    print("Tracking stopped.")
    vehicle.close()