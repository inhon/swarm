from dronekit import connect, VehicleMode, LocationGlobalRelative
import dronekit
import json
import time
import sys
import math
from threading import Timer
from RepeatTimer import RepeatTimer
from datetime import datetime


class Protocol():
    '''
    0   latitude(:011.8f), 
        longitude(:012.8f),
        altitude(:06.2f), 
        time(minute+second, 4 chars)  34 chars (including "0" in the beginning)
    1   "TAKEOFF"                     1 char (only "1")
    2   "TOOKOFF"                     1 char (only "2")
    3   "LAND"                        1 char (only "3")
    4   "LANDED"                      1 char (only "4")
    '''
    def __init__(self) -> None:
        pass

    def sendMsg(self, client, msgName, vehicle=None): 
        if msgName == "COORDINATES":
            #傳入leader的vehicle，由leader的速度計算bearing，再由leader的位置計算follower的位置後傳給follower
            #follower的位置以leader的飛行方向為軸進行計算，如無法計算出來，則經緯度設為0，follower如收到緯度為0，
            #則直接忽略
            lat, lon = self.getFollowerPosition(vehicle, 10)
            #lat = float(vehicle.location.global_frame.lat)
            #lon = float(vehicle.location.global_frame.lon)
            alt = float(vehicle.location.global_relative_frame.alt)
            if (alt<0): alt=0.0
            if lat is not None:  #如base是靜止，則lat 和 lon 是none
                current_time = datetime.now().strftime("%M%S")     # This will turn the time into minute and second format, something like 0835 (08:35)
            # assert(lat <= 90 and lat >= -90)              
            # assert(lon <= 180 and lon >= -180)      
            # assert(alt < 100)    # Assumes altitude below 100, if higher the message format requires adaptation
                TCP_msg = "0"+ str("{:011.8f}".format(lat)) + str("{:012.8f}".format(lon)) + str("{:06.2f}".format(alt)) + str(current_time)
            else:
                current_time = datetime.now().strftime("%M%S") 
                lat=0.0
                lon=0.0
                TCP_msg = "0"+ str("{:011.8f}".format(lat)) + str("{:012.8f}".format(lon)) + str("{:06.2f}".format(alt)) + str(current_time)
            
        elif msgName == "TAKEOFF":
            TCP_msg = "1"
        elif msgName == "TOOKOFF":
            TCP_msg = "2"
        elif msgName == "LAND":
            TCP_msg = "3"
        elif msgName == "LANDED":
            TCP_msg = "4"
        else:
            print("ERROR: Wrong Message Name:", msgName)
            return
        
        client.send(TCP_msg.encode())
        #print("Sent",msgName)

    def recvMsg(self, client):
        msgName = client.recv(1).decode()
        # print("Received Msg Name", msgName)
        if(msgName == "0"):
            msg = client.recv(33).decode()
            # print("Received:",msg)
            lat = float(msg[0:11])
            lon = float(msg[11:23])
            alt = float(msg[23:29])
            recvTime = int(msg[31:33])
            # assert(lat <= 90 and lat >= -90)               
            # assert(lon <= 180 and lon >= -180)             
            # assert(alt < 100)                  

            return lat, lon, alt, recvTime
        elif(msgName == "1"):
            return ("TAKEOFF",)
        elif(msgName == "2"):
            return ("TOOKOFF",)
        elif(msgName == "3"):
            return ("LAND",)
        elif(msgName == "4"):
            return ("LANDED",)
    
    def getBearing(self,vehicle,speedTreshold=3):
        """
        計算無人機實際飛行方向 bearing,北方為0,順時針
        當速度小於 speed_threshold(m/s)時，返回 None
        ehicle.velocity  -> [vx, vy, vz] 
        vx: Velocity in the North direction (m/s)
        vy: Velocity in the East direction (m/s)
        vz: Velocity in the Down direction (m/s)
        
        """
        vx, vy, vz = vehicle.velocity  # m/s
        horizontalSpeed = math.sqrt(vx**2 + vy**2)
        if horizontalSpeed < speedTreshold:
            return None  # 無意義的方向（幾乎靜止）
        # 計算地面上的飛行方向角（從正北順時針）
        angleRad = math.atan2(vy, vx)
        angleDeg = (math.degrees(angleRad) + 360) % 360 #將bearing 轉為正數
        return angleDeg
    
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