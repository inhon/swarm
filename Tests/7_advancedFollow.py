''' 
In this testcase we will fly two drones, base(leader) and rover(follower) at the same time, and to make sure
that they won't collide in the air, base will need to fly at a higher altitude, takeoff first, and land after rover.
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
from Drone import Drone, get_distance_metres
from RepeatTimer import RepeatTimer, sendMsg
from Internet import checkInternetConnection

baseVehicleIP="tcp:127.0.0.1:5762"
roverVehicleIP="tcp:127.0.0.1:5772"

SEND_INTERVAL = 1
SLEEP_LENGTH = 0.5
BASE_ALT = 25
ROVER_ALT = 15

if(len(sys.argv) <4): 
    print("Should have 3 arguments: argv[] = [<'base' or 'rover'>, <base's IP>, <port number>]")
    sys.exit()
if(sys.argv[1] == "base"): #create base drone and TCP server 
    baseDrone = Drone(baseVehicleIP)
    print("=====BASE=====")
    ''' Setting up server '''
    # ip = "127.0.0.1"
    ip = sys.argv[2]
    port = int(sys.argv[3])
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
    server.bind((ip,port))
    server.listen(5)
    client, address = server.accept()  #
    print("Base Connection established")
    
    points = list()
    #diff = 0.00000898
    points.append(LocationGlobalRelative(22.9052290, 120.2723712,BASE_ALT))   # 面對圖書館右側
    points.append(LocationGlobalRelative(22.9052043,120.2719259,BASE_ALT))   # 面對圖書館左側側
    points.append(LocationGlobalRelative(22.9048930,120.2719527,BASE_ALT))   # 圖書館對面右側
    #points.append(LocationGlobalRelative(24.7891822,120.9951456,BASE_ALT))
   
    baseDrone.takeoff(BASE_ALT)

    # Tell rover to take off
    # client.send("TAKEOFF".encode())
    baseDrone.sendInfo(client,"TAKEOFF")
    print("Sent TAKEOFF")

    # Wait for "TOOKOFF" from rover
    msg = baseDrone.receiveInfo(client)
    if(msg != "TOOKOFF"):
        print("Received incorrect message from rover:", msg)
        sys.exit()
    print("Received TOOKOFF")

    # Start sending the coordinates
    sendMsgTimer = RepeatTimer(SEND_INTERVAL, sendMsg, args=(baseDrone, client,))
    sendMsgTimer.start()

    # Start going to the pre-determined points
    for point in points:
        # 1. go to a pre-determined coordinate
        baseDrone.flyToPoint(point, 2)
        time.sleep(5)

    # Stop sending coordinates
    sendMsgTimer.cancel()

    # Tell rover to land
    baseDrone.sendInfo(client, "LAND")
    print("Sent LAND")

    # Wait for "LANDED" from rover
    msg = baseDrone.receiveInfo(client)
    if(msg != "LANDED"):
        print("Received incorrect message from rover:", msg)
        sys.exit()
    print("Received LANDED")

    # Land the base drone
    baseDrone.land()

elif(sys.argv[1] == "rover"):
    roverDrone = Drone(roverVehicleIP)
    print("=====ROVER=====")

    ''' Setting up client '''
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # ip = "127.0.0.1"
    ip = sys.argv[2]
    port = int(sys.argv[3])
    client.connect((ip,port))
    print("Rover Connection Established")

    # Waiting for "TAKEOFF" from base
    msg = roverDrone.receiveInfo(client)
    if(msg != "TAKEOFF"):
        print("Received incorrect message from base:", msg)
        sys.exit()
    print("Received TAKEOFF")

    roverDrone.takeoff(ROVER_ALT)

    # Tell base that rover has tookoff
    roverDrone.sendInfo(client, "TOOKOFF")
    print("Sent TOOKOFF")
    
    # Follow the base drone
    while(1):
        msg = roverDrone.receiveInfo(client)
        if(msg == "LAND"):
            print("Received LAND")
            break
        elif(type(msg) == LocationGlobalRelative):
            msg.alt = ROVER_ALT
            print("Received target:",msg)
            # vehicle.flyToPoint(targetPoint, 2)
            roverDrone.flyToPointNonBlocking(msg, 2)
            # time.sleep(SLEEP_LENGTH)
    
    # Landing the rover drone
    roverDrone.land()

    # Make sure the vehicle is landed (cause land() is aync) and disarmed
    while(roverDrone.vehicle.armed):
        time.sleep(1)

    time.sleep(1)
    print("Rover landed")

    # Tell base that rover has landed
    roverDrone.sendInfo(client, "LANDED")
    print("Sent LANDED")

else:
    print("Please specify which drone it is")