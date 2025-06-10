import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# --- PID 類別 ---
class PID:
    def __init__(self, kp, ki, kd, speed):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.integral = 0
        self.last_error = 0

    def reset(self):
        self.integral = 0
        self.last_error = 0

    '''
    #def compute(self, error, dt):
    =def compute(self, error, speed):
        #self.integral += error * dt
        derivative= speed
        #derivative = (error - self.last_error) / dt if dt > 0 else 0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        #self.last_error = error
        return output
    '''
'''
# --- Yaw 控制器 ---
class YawController:
    def __init__(self, vehicle, kp=0.8, ki=0.0, kd=0.2):
        self.vehicle = vehicle
        self.pid = PID(kp, ki, kd)
        self.is_relative=False

    def get_current_yaw(self):
        yaw = self.vehicle.attitude.yaw
        yaw_deg = (math.degrees(yaw) + 360) % 360
        return yaw_deg

    def send_yaw(self, heading): 
        #ardupilot 不支援 https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#set-attitude-target
              
        msg = self.vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        self.is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
        #直接給角度，不用YawController
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush() 

# --- 位置控制器 ---
class PositionController:
    def __init__(self, vehicle, kp=0.5, ki=0.0, kd=0.2):
        self.vehicle = vehicle
        self.pid_x = PID(kp, ki, kd)
        self.pid_y = PID(kp, ki, kd)
        self.pid_z = PID(kp, ki, kd)

    def reset(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()

    def send_velocity(self, vx, vy, vz):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def goto(self, target_location, tolerance=0.5, max_speed=3.0, timeout=20):
        self.reset()
        start = time.time()
        last_time = start

        while True:
            now = time.time()
            dt = now - last_time
            current = self.vehicle.location.global_relative_frame

            dx, dy = gps_to_local_offset_meters(current, target_location)
            dz = target_location.alt - current.alt

            if abs(dx) < tolerance and abs(dy) < tolerance and abs(dz) < tolerance:
                print("[Position] 抵達目標點")
                self.send_velocity(0, 0, 0)
                break

            vx = self.pid_x.compute(dx, dt)
            vy = self.pid_y.compute(dy, dt)
            vz = self.pid_z.compute(dz, dt)

            vx = max(min(vx, max_speed), -max_speed)
            vy = max(min(vy, max_speed), -max_speed)
            vz = max(min(vz, max_speed), -max_speed)

            print(f"[Position] dx={dx:.2f}, dy={dy:.2f}, dz={dz:.2f} → vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}")
            self.send_velocity(vx, vy, vz)

            if now - start > timeout:
                print("[Position] 超時停止")
                self.send_velocity(0, 0, 0)
                break

            last_time = now
            time.sleep(0.1)

# --- 綜合控制器 ---
class IntegratedController:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.yaw_ctrl = YawController(vehicle)
        self.pos_ctrl = PositionController(vehicle)

    def get_bearing_to_target(self, current_location, target_location):
        dLon = math.radians(target_location.lon - current_location.lon)
        lat1 = math.radians(current_location.lat)
        lat2 = math.radians(target_location.lat)
        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dLon)
        bearing = math.atan2(y, x)
        return (math.degrees(bearing) + 360) % 360

    def turn_and_fly(self, target_location, altitude=None):
        if altitude is None:
            altitude = target_location.alt

        current_location = self.vehicle.location.global_relative_frame
        bearing = self.get_bearing_to_target(current_location, target_location)
        print(f"[Integrated] 目標方位角: {bearing:.1f}°")

        self.yaw_ctrl.turn_to(bearing)
        # 目標點 altitude 可用參數或預設目標高度
        goal = LocationGlobalRelative(target_location.lat, target_location.lon, altitude)
        self.pos_ctrl.goto(goal)
    
    def safe_fly_to(self, target_location, all_vehicles, safe_distance=3.0, max_speed=3.0, timeout=30):
        """
        飛往目標位置，若其他無人機過近自動避讓（暫停），同時調整 yaw
        """
        self.yaw_ctrl.pid.reset()
        self.pos_ctrl.reset()

        start_time = time.time()
        last_time = time.time()

        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            # 狀態資料
            loc = self.vehicle.location.global_relative_frame
            yaw = self.vehicle.attitude.yaw
            yaw_deg = (math.degrees(yaw) + 360) % 360

            # 1. 與目標位置距離
            dx, dy = gps_to_local_offset_meters(loc, target_location)
            dz = target_location.alt - loc.alt
            dist = math.sqrt(dx**2 + dy**2 + dz**2)
            if dist < 0.5:
                self.pos_ctrl.send_velocity(0, 0, 0)
                print("[SafeFly] ✅ 已到達隊形位置")
                break

            # 2. 檢查附近有無其他無人機
            if self.detect_nearby_vehicles(all_vehicles, safe_distance):
                print("[SafeFly] ⚠️ 前方無人機過近，暫停")
                self.pos_ctrl.send_velocity(0, 0, 0)
                time.sleep(0.5)
                continue

            # 3. 同步轉頭
            bearing = self.get_bearing_to_target(loc, target_location)
            yaw_error = bearing - yaw_deg
            if yaw_error > 180: yaw_error -= 360
            if yaw_error < -180: yaw_error += 360
            yaw_rate = self.yaw_ctrl.pid.compute(yaw_error, dt)
            yaw_rate = max(min(yaw_rate, 45), -45)

            # 4. 飛行速度
            vx = self.pos_ctrl.pid_x.compute(dx, dt)
            vy = self.pos_ctrl.pid_y.compute(dy, dt)
            vz = self.pos_ctrl.pid_z.compute(dz, dt)
            vx = max(min(vx, max_speed), -max_speed)
            vy = max(min(vy, max_speed), -max_speed)
            vz = max(min(vz, max_speed), -max_speed)

            self.send_yaw_velocity(vx, vy, vz, yaw_rate)

            if now - start_time > timeout:
                print("[SafeFly] ⏱ 超時結束")
                break

            time.sleep(0.1)
    def detect_nearby_vehicles(self, all_vehicles, threshold=3.0):
        my_loc = self.vehicle.location.global_relative_frame
        for v in all_vehicles:
            if v == self.vehicle:
                continue
            other = v.location.global_relative_frame
            dx, dy = gps_to_local_offset_meters(my_loc, other)
            dist = math.hypot(dx, dy)
            if dist < threshold:
                return True
        return False
    



# --- 主程式範例 ---
if __name__ == "__main__":
    vehicle = connect('127.0.0.1:14550', wait_ready=True)
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(2)

    controller = IntegratedController(vehicle)

    target = LocationGlobalRelative(47.398036222362471, 8.5450146439425509, 10)
    controller.turn_and_fly(target)

    print("任務完成，關閉連線")
    vehicle.close()
'''
