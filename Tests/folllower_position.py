import math

def follower_position_backward(lat1_deg, lon1_deg, bearing_deg, distance_m):
    #提供leader的經緯度，飛行方向(bearing_deg)，計算反飛行方向，距離leader  distance_m 的經緯度
    R = 6371000  # 地球半徑（公尺）

    # 轉成弧度
    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    bearing = math.radians((bearing_deg + 180) % 360)  # 反方向

    # 計算新的座標
    lat2 = math.asin(math.sin(lat1) * math.cos(distance_m / R) +
                     math.cos(lat1) * math.sin(distance_m / R) * math.cos(bearing))

    lon2 = lon1 + math.atan2(math.sin(bearing) * math.sin(distance_m / R) * math.cos(lat1),
                             math.cos(distance_m / R) - math.sin(lat1) * math.sin(lat2))

    # 轉回十進制度
    lat2_deg = math.degrees(lat2)
    lon2_deg = math.degrees(lon2)

    return lat2_deg, lon2_deg

def destination_point(lat_deg, lon_deg, bearing_deg, distance_m):
    '''
    bearing_deg follower 的方向
    # 後方偏左 45 度
    bearing_left = (bearing + 180 + 45) % 360
    lat_left, lon_left = destination_point(lat, lon, bearing_left, distance)

    # 後方偏右 45 度
    bearing_right = (bearing + 180 - 45) % 360
    lat_right, lon_right = destination_point(lat, lon, bearing_right, distance)
    '''

    R = 6371000  # 地球半徑 (公尺)
    lat1 = math.radians(lat_deg)
    lon1 = math.radians(lon_deg)

    bearing = math.radians(bearing_deg)

    lat2 = math.asin(math.sin(lat1) * math.cos(distance_m / R) +
                     math.cos(lat1) * math.sin(distance_m / R) * math.cos(bearing))

    lon2 = lon1 + math.atan2(math.sin(bearing) * math.sin(distance_m / R) * math.cos(lat1),
                             math.cos(distance_m / R) - math.sin(lat1) * math.sin(lat2))

    return math.degrees(lat2), math.degrees(lon2)

'''
# 範例：從 (25.033, 121.565) 出發，方向 45°，後方 10 公尺
new_lat, new_lon = move_backward(25.033, 121.565, 45, 10)
print(f"後方10公尺的新座標為：({new_lat}, {new_lon})")

lat = 25.0330
lon = 121.5650
bearing = 70
distance = 10

# 後方偏左 45 度
bearing_left = (bearing + 180 + 45) % 360
lat_left, lon_left = destination_point(lat, lon, bearing_left, distance)

# 後方偏右 45 度
bearing_right = (bearing + 180 - 45) % 360
lat_right, lon_right = destination_point(lat, lon, bearing_right, distance)

print(f"後方偏左45度 10m 處座標：({lat_left:.6f}, {lon_left:.6f})")
print(f"後方偏右45度 10m 處座標：({lat_right:.6f}, {lon_right:.6f})")
'''
