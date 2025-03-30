from dronekit import connect
import time
import os, sys

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, parent_dir)

from Drone import Drone

connection_string = "tcp:127.0.0.1:5762"
drone=Drone(connection_string)


'''
Copter v4.5.7 Parameters:
https://ardupilot.org/copter/docs/parameters.html

'''
# 儲存之前的值（初始化）
previous_rtl_alt = drone.vehicle.parameters['RTL_ALT']

# 定義 callback function
def parameter_callback(self, attr_name, value):
    global previous_rtl_alt
    #new_rtl_alt = drone.vehicle.parameters['RTL_ALT']
    #new_rtl_alt = drone.vehicle.parameters[attr_name]
    new_rtl_alt = value
    if new_rtl_alt != previous_rtl_alt:
        print(f"RTL_ALT 參數已改變：{previous_rtl_alt} ➜ {new_rtl_alt}")
        previous_rtl_alt = new_rtl_alt

# 加入監聽器，針對 "parameters" 這個屬性
drone.vehicle.parameters.add_attribute_listener('RTL_ALT', parameter_callback)
time.sleep(2)
drone.vehicle.parameters['RTL_ALT']=3000

print("正在監聽 RTL_ALT 的變動...（10 秒）")

# 等待變化（這段期間你可以手動或程式內更改參數）
time.sleep(15)

# 移除監聽器並關閉連線
drone.vehicle.parameters.remove_attribute_listener('RTL_ALT', parameter_callback)
drone.closeConn()