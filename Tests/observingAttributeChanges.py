from dronekit import connect, VehicleMode
import time
import os, sys

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, parent_dir)

from Drone import Drone

connection_string = "tcp:127.0.0.1:5762"
drone=Drone(connection_string)

# 儲存之前的值（初始化）
drone.vehicle.mode = VehicleMode("GUIDED")
while not drone.vehicle.mode.name=='GUIDED':  #Wait until mode has changed
    print(" Waiting for mode change ...")
    time.sleep(1)

previous_mode = drone.vehicle.mode.name
# 定義 callback function
def mode_callback(self, attr_name, value):
    global previous_mode
    #new_rtl_alt = drone.vehicle.mode.name
    new_mode = value
    if new_mode != previous_mode:
        print(f"MODE 參數已改變：{previous_mode} ➜ {new_mode.name}")
        previous_mode = new_mode

# 加入監聽器，針對 "mode" 這個屬性
drone.vehicle.add_attribute_listener('mode', mode_callback)
time.sleep(2)
drone.vehicle.mode = VehicleMode("STABILIZE")

print("正在監聽 RTL_ALT 的變動...（10 秒）")

# 等待變化（這段期間你可以手動或程式內更改參數）
time.sleep(10)

# 移除監聽器並關閉連線
drone.vehicle.remove_attribute_listener('mode', mode_callback)
drone.closeConn()