'''
This test checks if a pair of base and rover can communicate through TCP by setting up the 'stateReport' in Drone class.

argv[] = [<"base" or "rover">, <base's IP>, <port number>]
'''
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import sys
import math
import socket
import os

#sys.path.append("..")
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, parent_dir)

from Drone import Drone
from RepeatTimer import RepeatTimer, sendMsg
from Internet import checkInternetConnection


if(len(sys.argv) <4): 
    print("Should have 3 arguments: argv[] = [<'base' or 'rover'>, <base's IP>, <port number>]")
    sys.exit()

connection_strings = ["tcp:127.0.0.1:5762"]
#connection_strings = ["tcp:127.0.0.1:5762","tcp:127.0.0.1:5772"]
#connection_string = "tcp:127.0.0.1:5762"
#connection_string = "/dev/tty.usbmodem14101"

''' Connect to vehicle '''
for connection_string in connection_strings:
    drone = Drone(connection_string)
    if(drone.connected): break
drone.setStateReport(3)

''' Setting up a checker to see if internet connection works, otherwise land the vehicle'''
checkConnectTimer = RepeatTimer(10,checkInternetConnection,args=(drone,))
checkConnectTimer.start()
print("Check Connect Timer Set")

if(sys.argv[1] == "base"):
    print("=====BASE=====")
    ''' Setting up server '''
    #ip = "172.20.10.8"
    ip = sys.argv[2]
    port = int(sys.argv[3])
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((ip,port))
    server.listen(5)
    client, address = server.accept()
    print("Base Connection established")
    
    sendMsgTimer = RepeatTimer(1,sendMsg, args=(drone, client,))
    sendMsgTimer.start()
    while(1):
        print("Base in while loop")
        time.sleep(1)

elif(sys.argv[1] == "rover"):
    print("=====ROVER=====")

    ''' Setting up client '''
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ip = sys.argv[2]
    port = int(sys.argv[3])
    client.connect((ip,port))
    print("Rover Connection Established")
    
    counter=0
    while(counter<5):
        print("Enter Iteration",counter)
        targetPoint = drone.receiveInfo(client)
        
        if(type(targetPoint) == LocationGlobalRelative):
            targetPoint.alt = 3
            print("Received target:",targetPoint)
        else:
            print("Received message of type",type(targetPoint))

        counter = counter+1
        time.sleep(0.7)
    


else:
    print("Please specify which drone it is")