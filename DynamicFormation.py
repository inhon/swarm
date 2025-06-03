'''
動態轉向隊形跟隨:
1. 隊長（中心機）持續向一個目標 GPS 飛行
2. 隊形成員根據隊長的機頭方向 + 實時位置，不斷更新目標點
3. 隊形持續保持相對位置與朝向，像戰機編隊一樣「轉向同步」
系統設計概念:
每架機都有：
1.各自的 Controller
2.各自的「隊形偏移」（如左後、右後）
3.定時（每 0.2 秒）更新目標位置並追蹤飛行
控制流程:
[中心機]
1.計算當前位置 + 朝向
2.對所有隊形 offset 套用 yaw 旋轉，得到目標 GPS
3.發送給隊員 → 各機轉頭 + 飛行
'''

import threading
import math
import time
from dronekit import connect, LocationGlobalRelative, VehicleMode
from Controller import IntegratedController

# 編隊偏移（每台無人機的位置，單位：公尺）
FORMATION_V_SHAPE = [
    (0, 0, 0),        # 領頭機
    (-3, -3, 0),      # 左後
    (3, -3, 0),       # 右後
    (-6, -6, 0),      # 左更後
    (6, -6, 0),       # 右更後
]

# 用 yaw 旋轉後套用偏移
def offset_to_rotated_location(origin, offset, yaw_deg):
    x, y, z = offset
    yaw_rad = math.radians(yaw_deg)
    x_rot = x * math.cos(yaw_rad) - y * math.sin(yaw_rad)
    y_rot = x * math.sin(yaw_rad) + y * math.cos(yaw_rad)
    dlat = y_rot / 110540
    dlon = x_rot / (111320 * math.cos(math.radians(origin.lat)))
    return LocationGlobalRelative(origin.lat + dlat, origin.lon + dlon, origin.alt + z)

def get_current_yaw(vehicle):
    yaw = vehicle.attitude.yaw
    return (math.degrees(yaw) + 360) % 360

def get_leader_state(): #TODO
    loc = leader.location.global_relative_frame
    yaw = get_current_yaw(leader)
    return loc, yaw


# 控制隊員持續跟隨
def follower_loop(vehicle, offset, leader_state_getter, index):
        """
        ctrl 是該無人機對應的 IntegratedController 控制器實例，它整合了：
        1.Yaw PID 控制器（機頭轉向）
        2.Position PID 控制器（位置導航）
        3.功能：turn_and_fly()：先轉頭再飛向目標
        ctrl.turn_and_fly(target) 的流程是：
        呼叫 Yaw 控制器 → 讓無人機轉頭對準目標
        呼叫 Position 控制器 → 精確控制無人機飛到該位置
        這確保了飛行過程中：
        1.無人機「面對」目標方向
        2.保持穩定隊形偏移
        3.飛行行為一致、美觀
        """
    ctrl= IntegratedController(vehicle)
    while True:
        leader_loc, leader_yaw = leader_state_getter()
        target = offset_to_rotated_location(leader_loc, offset, leader_yaw)
        ctrl.turn_and_fly(target)
        print(f"[Drone {index}] 跟隨隊形位置：{target.lat:.6f}, {target.lon:.6f}")
        time.sleep(0.2)  # 反應頻率

'''

def follower_loop(vehicle, offset, get_leader_state, all_vehicles, index):
    ctrl = IntegratedController(vehicle)
    while True:
        leader_loc, leader_yaw = get_leader_state()
        target = offset_to_rotated_location(leader_loc, offset, leader_yaw)
        ctrl.safe_fly_to(target, all_vehicles)
        time.sleep(0.1)

'''

# 主控制程序
def main():
    connection_strings = [
        'udp:127.0.0.1:14550',  # 主機
        'udp:127.0.0.1:14551',
        'udp:127.0.0.1:14552',
        'udp:127.0.0.1:14553',
        'udp:127.0.0.1:14554',
    ]

    # 連線 + 初始化所有無人機
    vehicles = [connect(conn, wait_ready=True) for conn in connection_strings]
    for v in vehicles:
        v.mode = VehicleMode("GUIDED")
    time.sleep(2)

    leader = vehicles[0]
    leader_ctrl = IntegratedController(leader)

    # 領頭機的終極目標
    goal = LocationGlobalRelative(47.398139, 8.545246, 10)

    # 🧠 提供隊長目前狀態給成員
    def get_leader_state():
        loc = leader.location.global_relative_frame
        yaw = get_current_yaw(leader)
        return loc, yaw

    # 啟動所有成員的跟隨控制
    threads = []
    for i in range(1, len(vehicles)):
        t = threading.Thread(
            target=follower_loop,
            args=(vehicles[i], FORMATION_V_SHAPE[i], get_leader_state, i)
        )
        t.start()
        threads.append(t)

    # 主機開始往目標前進
    print("🚁 領頭機飛往目標點，隊形動態跟隨...")
    leader_ctrl.turn_and_fly(goal)

    for t in threads:
        t.join()

    print("✅ 任務完成")
    for v in vehicles:
        v.close()

if __name__ == "__main__":
    main()
