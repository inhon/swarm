from dronekit import connect, VehicleMode, LocationGlobalRelative
import dronekit
import json
import time
import sys
import math
from threading import Timer
from RepeatTimer import RepeatTimer
from datetime import datetime

class formation:
    '''
    計算隊形的位置:
    隊形包括:
    1. Line: rover 跟在 base 飛行方向的後方
    2. Wedge: 倒 V 型，base 帶領rover
    3. Square: base在中間，rover在方形的四個角上
    '''
    def __init__(self) -> None:
        pass
    def getHeading(self,vehicle):
        yaw = vehicle.attitude.yaw  # 弧度值
        heading = (math.degrees(yaw) + 360) % 360  # 轉成 0~360 度
        return heading # int
        '''
        vx, vy, vz = vehicle.velocity  # m/s
        horizontalSpeed = math.sqrt(vx**2 + vy**2)
        if horizontalSpeed < speedTreshold:
            return None  # 無意義的方向（幾乎靜止）
        # 計算地面上的飛行方向角（從正北順時針）
        angleRad = math.atan2(vy, vx)
        angleDeg = (math.degrees(angleRad) + 360) % 360 #將bearing 轉為正數
        '''
        return angleDeg
        '''
    def getFollowerPosition(self, vehicle, distMeter=10): 

    def getFollowerPosition(self, vehicle, distMeter=10): 
        '''
        1.使用leader的經緯度與飛行方向(bearing),計算於反飛行方向,距離leader distMeter 的經緯度
        2.再來要計算2台follower位於leader後方,左右各45度的情況(TODO)
        '''
        R = 6371000  # 地球半徑（公尺）
        # 轉成弧度
        lat1 = math.radians(vehicle.location.global_frame.lat)
        lon1 = math.radians(vehicle.location.global_frame.lon)
        bearing=self.getBearing(vehicle) #degree
        if bearing is not None:
            reverseBearing = math.radians((bearing + 180) % 360)  # 反方向
            # 計算新的座標
            lat2 = math.asin(math.sin(lat1) * math.cos(distMeter / R) +
                   math.cos(lat1) * math.sin(distMeter / R) * math.cos(reverseBearing))
            lon2 = lon1 + math.atan2(math.sin(reverseBearing) * math.sin(distMeter / R) * math.cos(lat1),
                             math.cos(distMeter / R) - math.sin(lat1) * math.sin(lat2))
            # 轉回十進制度
            lat2_deg = math.degrees(lat2)
            lon2_deg = math.degrees(lon2)
            return lat2_deg, lon2_deg
        else:
            print("無人機靜止中，無法得到飛行方向以計算follower 位置")
            return None, None 