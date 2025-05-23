import numpy as np
import math
# 地球參數（WGS84）
a = 6378137.0               # 半長軸（赤道半徑）
f = 1 / 298.257223563       # 扁率
e2 = f * (2 - f)            # 離心率平方

# Within Accepted Delay (in sec) the received data will be considered valid
ACCEPTED_DELAY = 3 

def timeIsValid(recvTime, curTime):
    if(curTime >= recvTime):
        if(curTime-recvTime < ACCEPTED_DELAY): return True
        else: return False
    else:
        if(curTime+60 -recvTime < ACCEPTED_DELAY): return True
        else: return False
    
    return True

def getDistanceMetres(aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.
        This method is an approximation, and will not be accurate over large distances and close to the 
        earth's poles.
        """
        # 0.00000898 difference => 1 meter
        dlat = float(aLocation2.lat) - float(aLocation1.lat)
        dlong = float(aLocation2.lon) - float(aLocation1.lon)
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5 
    
def geodetic_to_ecef(lat, lon, h=0):
    """將經緯度轉換為 ECEF 座標"""
    lat = np.radians(lat)
    lon = np.radians(lon)
    N = a / np.sqrt(1 - e2 * np.sin(lat)**2)
    x = (N + h) * np.cos(lat) * np.cos(lon)
    y = (N + h) * np.cos(lat) * np.sin(lon)
    z = (N * (1 - e2) + h) * np.sin(lat)
    return np.array([x, y, z])

def ecef_to_ned_matrix(lat, lon):
    """建立 ECEF 到 NED 的旋轉矩陣"""
    lat = np.radians(lat)
    lon = np.radians(lon)
    R = np.array([
        [-np.sin(lat)*np.cos(lon), -np.sin(lat)*np.sin(lon),  np.cos(lat)],
        [-np.sin(lon),              np.cos(lon),              0],
        [-np.cos(lat)*np.cos(lon), -np.cos(lat)*np.sin(lon), -np.sin(lat)]
    ])
    return R

def getUnitVector(p1, p2): 
    """
    計算從 p1 指向 p2 的向量在 NED 座標系下的單位向量。
    p1, p2是 global_relative_frame
    point1, point2: (lat, lon) in degrees
    return: 單位向量 (north, east), 
    """
    point1=(p1.lat, p1.lon)
    point2=(p2.lat, p2.lon)
    ecef1 = geodetic_to_ecef(*point1)#轉換為ECEF座標系
    ecef2 = geodetic_to_ecef(*point2)
    delta_ecef = ecef2 - ecef1
    R = ecef_to_ned_matrix(*point1)#建立 p1 所在點的 NED 坐標系旋轉矩陣
    delta_ned = R @ delta_ecef #將 ECEF 向量轉換為 NED 座標系
    unit_vector_ned = delta_ned / np.linalg.norm(delta_ned) #正規化為單位向量
    vn=unit_vector_ned[0]
    vd=unit_vector_ned[1]
    return vn, vd