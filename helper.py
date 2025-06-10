import numpy as np
import math
from geopy.distance import geodesic
import setting

# Within Accepted Delay (in sec) the received data will be considered valid

def timeIsValid(recvTime, curTime):
    if(curTime >= recvTime):
        if(curTime-recvTime < setting.ACCEPTED_DELAY): return True
        else: return False
    else:
        if(curTime+60 -recvTime < setting.ACCEPTED_DELAY): return True
        else: return False
    
    return True

def getDistanceMetres(aLocation1, aLocation2):
    """
    使用 geopy 計算兩個地點之間的地面距離（單位：公尺）
    aLocation1, aLocation2 必須具有 .lat 和 .lon 屬性（例如 LocationGlobal 或 Waypoint）
    """
    point1 = (aLocation1.lat, aLocation1.lon)
    point2 = (aLocation2.lat, aLocation2.lon)
    return geodesic(point1, point2).meters

def getUnitVector(p1, p2):
    """
    使用 geopy 計算從 p1 指向 p2 的水平單位向量 (North, East)。
    適用於局部區域內（近似平地）。
    """
    lat1, lon1 = p1.lat, p1.lon
    lat2, lon2 = p2.lat, p2.lon

    # 分開計算南北和東西方向的距離（以公尺為單位）
    d_north = geodesic((lat1, lon1), (lat2, lon1)).meters
    d_east = geodesic((lat1, lon1), (lat1, lon2)).meters

    # 根據緯度和經度的相對關係調整方向符號
    if lat2 < lat1:
        d_north = -d_north
    if lon2 < lon1:
        d_east = -d_east

    # 建立水平位移向量並正規化
    direction_vector = np.array([d_north, d_east])
    unit_vector = direction_vector / np.linalg.norm(direction_vector)

    return unit_vector[0], unit_vector[1]  # vn, ve

def calculate_formation_velocity(i, uav_i_pos):
    """計算避免碰撞的速度向量（作用於 UAV i）"""

    collision_threshold = 10  # 設定碰撞避距閾值（公尺）
    repulsion_strength = 50   # 避碰推力強度係數
    v_formation = np.array([0.0, 0.0])

    for j in range(self.num_uavs):
        if i == j:
            continue  # 不跟自己比

        uav_j_pos = self.vehicles[j].read_global_position()

        # 計算 UAV i 和 UAV j 的地理距離（單位：公尺）
        distance_ij = geodesic(
            (uav_i_pos.lat, uav_i_pos.lon),
            (uav_j_pos.lat, uav_j_pos.lon)
        ).meters

        # 只有在小於閾值時才採取避碰行動
        if distance_ij < collision_threshold:
            print(f"[警告] Collision Risk between UAV {i} and UAV {j} — Distance: {distance_ij:.2f} m")

            # 滲透深度（越深表示越接近或重疊）
            penetration = collision_threshold - distance_ij
            weight = penetration / distance_ij  # 正值，越近推力越大

            # 從 j 指向 i 的向量（表示 i 應往反方向逃離）
            direction_vector = np.array([
                uav_i_pos.lat - uav_j_pos.lat,
                uav_i_pos.lon - uav_j_pos.lon
            ])
            normalized_direction = direction_vector / np.linalg.norm(direction_vector)

            # 推開速度向量
            repulsion_velocity = repulsion_strength * weight * normalized_direction
            v_formation += repulsion_velocity

            print(f"    -> UAV {i} Repulsion Vector from UAV {j}: {repulsion_velocity}")

    return v_formation

