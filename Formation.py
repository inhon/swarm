from dronekit import connect, VehicleMode, LocationGlobalRelative
import setting
import json
import time
import sys
import math
from threading import Timer
from RepeatTimer import RepeatTimer
from datetime import datetime
from geopy.distance import distance
from geopy.point import Point


class formation:
    '''
    計算隊形的位置:
    隊形(formation)包括:
    0. Line: 1台rover 跟在 base 飛行方向的後方
    1. Wedge: 倒 V 型，base 帶領2台rover, 
    2. Square: base在中間，4台rover在方形的四個角上
    '''
    def __init__(self):
        #self.formation=formation
        self.offset=[(-10,0,0),   #隊形0 [後方]
                     (-20,-20,0),(-20,20,0), #隊形1 [左後, 右後]
                     (-20,-20,0),(-20,20,0),(20,20,0),(20,-20,0)] #隊形2 [左後 左上 右上 右下]
        #offset 是在FRD座標系統下，各台rover的位置偏移量
    
    def getHeading(self,vehicle): #base 的機頭方向， base 的機頭方向為飛行方向
        yaw = vehicle.attitude.yaw  # 弧度值
        heading = (math.degrees(yaw) + 360) % 360  # 轉成 0~360 度
        return heading # int
   
    def getRoverPosition_0(self, vehicle):
        lat, lon, alt = (
            vehicle.location.global_relative_frame.lat,
            vehicle.location.global_relative_frame.lon,
            vehicle.location.global_relative_frame.alt
        )
        
        #vx, vy, vz = vehicle.velocity  # m/s
        #horizontalSpeed = math.sqrt(vx**2 + vy**2)
        #if horizontalSpeed < setting.SPEED_THRESHOLD:
        #    return (None, None)  # base 速度慢，視為靜止
        f,r,d=self.offset[0]
        headingRad = vehicle.attitude.yaw
        roverPos=[]
        # FRD 轉 NED 偏移量（進行平面旋轉）
        n = f * math.cos(headingRad) - r * math.sin(headingRad) 
        e = f * math.sin(headingRad) + r * math.cos(headingRad)
        origin = Point(lat, lon)
        # 先向北方向偏移
        pointNorth = distance(meters=n).destination(origin, bearing=0)
        # 再向東方向偏移
        finalPoint = distance(meters=e).destination(pointNorth, bearing=90)
        roverPos.append((finalPoint.latitude, finalPoint.longitude))
        #return finalPoint.latitude, finalPoint.longitude
        return roverPos[0] 
    
    def getRoverPosition_1(self, vehicle):
        lat, lon, alt = (
            vehicle.location.global_relative_frame.lat,
            vehicle.location.global_relative_frame.lon,
            vehicle.location.global_relative_frame.alt
        )
        origin = Point(lat, lon)
        headingRad = vehicle.attitude.yaw
        roverPos=[]
        for i in range(1,3):
            f,r,d=self.offset[i]
            n = f * math.cos(headingRad) - r * math.sin(headingRad) 
            e = f * math.sin(headingRad) + r * math.cos(headingRad)
            # 先向北方向偏移
            pointNorth = distance(meters=n).destination(origin, bearing=0)
            # 再向東方向偏移
            finalPoint = distance(meters=e).destination(pointNorth, bearing=90)
            roverPos.append((finalPoint.latitude, finalPoint.longitude))
        return roverPos #roverPos=[(lat1, lon1), (lat2, lon2)]

    def getRoverPosition_2(self, vehicle):
         
        lat, lon, alt = (
            vehicle.location.global_relative_frame.lat,
            vehicle.location.global_relative_frame.lon,
            vehicle.location.global_relative_frame.alt,
        )
        origin = Point(lat, lon)
        headingRad = vehicle.attitude.yaw
        roverPos=[]
        for i in range(3,7):
            f,r,d=self.offset[i]
            n = f * math.cos(headingRad) - r * math.sin(headingRad) 
            e = f * math.sin(headingRad) + r * math.cos(headingRad)
            # 先向北方向偏移
            pointNorth = distance(meters=n).destination(origin, bearing=0)
            # 再向東方向偏移
            finalPoint = distance(meters=e).destination(pointNorth, bearing=90)
            roverPos.append((finalPoint.latitude, finalPoint.longitude))
        return roverPos  #roverPos=[(lat1, lon1), (lat2, lon2), (lat3, lon3), (lat4, lon4)] 