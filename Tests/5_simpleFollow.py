''' 
This test is designed for a simple check to see if follower drone actually follows the leader drone. 
'''

'''
argv[] = [<"base" or "rover">, <base's IP>, <port number>]
'''
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import sys
import math
import socket
import os

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, parent_dir)

from Drone import Drone
from RepeatTimer import RepeatTimer, sendMsg
import helper
import setting


baseVehicleIP="tcp:127.0.0.1:5762"
roverVehicleIP="tcp:127.0.0.1:5772"

if(len(sys.argv) <4): 
    print("Should have 3 arguments: argv[] = [<'base' or 'rover'>, <base's IP>, <port number>]")
    # python 5_simpleFollow.py base 127.0.0.1 12345
    sys.exit()

if(sys.argv[1] == "base"):
    baseDrone = Drone(baseVehicleIP)
    print("=====BASE=====")
    baseDrone.vehicle.airspeed=setting.BASE_SPEED
    ''' Setting up server '''
    ip = sys.argv[2]
    port = int(sys.argv[3])
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((ip,port))
    server.listen(5)
    client, address = server.accept()
    print("Base Connection established")
    baseDrone.takeoff(25) #Waiting for manual confirmation for takeoff. blocking
    sendMsgTimer = RepeatTimer(setting.SEND_INTERVAL,sendMsg, args=(baseDrone, client,))
    '''
    def sendMsg(drone, client): #定義在RepeaterTimer.py，用來傳送rover要追隨的座標
        drone.sendInfo(client, "COORDINATES") 
    
    def sendInfo(self, client, msgName): #
        self.protocol.sendMsg(client, msgName, self.vehicle) 
    1. self.vehicle 用來取得base無人機位置
    2. 使用msgName已發送對應訊息如"COORDINATES"、 "TAKEOFF"、 "TOOKOFF"...
    3. 如是base發送座標，則在sendMsg內計算follower的位置進行發送
    '''
    sendMsgTimer.start()
   
    try:
       while(1):
            time.sleep(1)
    except KeyboardInterrupt:
        baseDrone.vehicle.mode=VehicleMode("RTL")
        baseDrone.closeConn()

elif(sys.argv[1] == "rover"):
    roverDrone = Drone(roverVehicleIP)
    print("=====ROVER=====")

    ''' Setting up client '''
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ip = sys.argv[2]
    port = int(sys.argv[3])
    client.connect((ip,port))
    print("Rover Connection Established")
    roverDrone.takeoff(15) #Waiting for manual confirmation for takeoff. blocking
    
    counter=0
    numInvalidMsg = 0
    ''' 
    We only want our iterations be counted when the drone actually goes to the point, 
    so only increment counter when the received message is valid. 
    numInvalidMsg is a safety measure that makes sure if the rover forever receives outdated (invalid) message, 
    we will break from the loop and land.
    '''
    #while(numInvalidMsg < 5 and counter<5):
    try:
        while True:
            #print("Enter Iteration",counter)
            targetPoint = roverDrone.receiveInfo(client)
        
            if(type(targetPoint) == LocationGlobalRelative): 
                targetPoint.alt = 15
                #print("Received target:",targetPoint)
                roverDrone.flyToPointVelocity_vector(targetPoint) #控制rover飛向目標點
                #roverDrone.flyToPoint(targetPoint, 3) #blocking
                #roverDrone.flyToPointNonBlocking(targetPoint, 5)
                #roverDrone.flyToPointVelocity(targetPoint)
                counter = counter+1
                numInvalidMsg = 0
            else:
                numInvalidMsg = numInvalidMsg + 1
            time.sleep(setting.SLEEP_LENGTH)
    except KeyboardInterrupt:
        roverDrone.vehicle.mode=VehicleMode("RTL")
        #roverDrone.land()
        roverDrone.closeConn()
    # Terminate program and socket 
else:
    print("Please specify which drone it is")
    