''' 
In this testcase we will fly two drones, base(leader) and rover(follower) at the same time, and to make sure
that they won't collide in the air, base will need to fly at a higher altitude, takeoff first, and land after rover.
'''

'''
argv[] = [<"base" or "rover">, <base's IP>, <port number (for rover 1)>, <port number 2(for rover 2, only base required)>]
'''
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import sys
import math
import socket

sys.path.append("..")
from Drone import Drone, get_distance_metres
from RepeatTimer import RepeatTimer, sendMsg
from Internet import checkInternetConnection

SEND_INTERVAL = 1
SLEEP_LENGTH = 3
BASE_ALT = 30
ROVER1_ALT = 25
ROVER2_ALT = 20


baseVehicleIP="tcp:127.0.0.1:5762"
rover1VehicleIP="tcp:127.0.0.1:5772"
rover2VehicleIP="tcp:127.0.0.1:5782"
# connection_string = "COM"

if(len(sys.argv) <4): 
    print("Should have 3 arguments: argv[] = [<'base' or 'rover1' or 'rover2'>, <base's IP>, <port number>]")
    sys.exit()

if(sys.argv[1] == "base"):
    if(len(sys.argv) < 5):
        print("Should have 4 arguments: argv[] = [<'base' or 'rover1' or 'rover2'>, <base's IP>, <port number 1>, <port number 2>]")
        sys.exit()

if(sys.argv[1] == "base"):
    baseDrone = Drone(baseVehicleIP)
    print("=====BASE=====")
    ''' Setting up server '''
    ip = sys.argv[2]
    port1 = int(sys.argv[3])
    server1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server1.bind((ip,port1))
    server1.listen(5)
    
    port2 = int(sys.argv[4])
    server2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server2.bind((ip,port2))
    server2.listen(5)

    client1, address1 = server1.accept()
    print("Base Connection 1 established")
    client2, address2 = server2.accept()
    print("Base Connection 2 established")

    points = list()
    diff = 0.00000898
    # 圖書館前草皮繞一圈
    points.append(LocationGlobalRelative(22.9052191,120.2724221,BASE_ALT))
    points.append(LocationGlobalRelative(22.9051722,120.2719045,BASE_ALT))
    points.append(LocationGlobalRelative(22.9048238,20.2719581,BASE_ALT))
    points.append(LocationGlobalRelative(22.9049078,120.2724597,BASE_ALT))
    
    baseDrone.takeoff(BASE_ALT)

    # Tell rover 1 to take off
    baseDrone.sendInfo(client1, "TAKEOFF")
    print("Sent TAKEOFF to rover 1")

    # Wait for "TOOKOFF" from rover
    msg = baseDronevehicle.receiveInfo(client1)
    if(msg != "TOOKOFF"):
        print("Received incorrect message from rover 1:", msg)
        sys.exit()
    print("Received TOOKOFF from rover 1")


    # Tell rover 2 to take off
    vbaseDrone.sendInfo(client2, "TAKEOFF")
    print("Sent TAKEOFF to rover 2")

    # Wait for "TOOKOFF" from rover
    msg = baseDrone.receiveInfo(client2)
    if(msg != "TOOKOFF"):
        print("Received incorrect message from rover 2:", msg)
        sys.exit()
    print("Received TOOKOFF from rover 2")

    # Start sending the coordinates
    sendMsgTimer1 = RepeatTimer(SEND_INTERVAL,sendMsg, args=(baseDrone,client1,))
    sendMsgTimer1.start()
    sendMsgTimer2 = RepeatTimer(SEND_INTERVAL,sendMsg, args=(baseDrone,client2,))
    sendMsgTimer2.start()

    # Start going to the pre-determined points
    for point in points:
        # 1. go to a pre-determined coordinate
        baseDrone.flyToPoint(point, 2)
        time.sleep(SLEEP_LENGTH)

    # Stop sending coordinates
    sendMsgTimer1.cancel()
    sendMsgTimer2.cancel()

    # Tell rover 2 to land
    vehbaseDrone.sendInfo(client2, "LAND")
    print("Sent LAND to rover 2")

    # Wait for "LANDED" from rover 2
    msg = vehicle.receiveInfo(client2)
    if(msg != "LANDED"):
        print("Received incorrect message from rover 2:", msg)
        sys.exit()
    print("Received LANDED from rover 2")

    # Tell rover 1 to land
    baseDrone.sendInfo(client1, "LAND")
    print("Sent LAND to rover 1")

    # Wait for "LANDED" from rover 1
    msg = vehicle.receiveInfo(client1)
    if(msg != "LANDED"):
        print("Received incorrect message from rover 1:", msg)
        sys.exit()
    print("Received LANDED from rover 1")

    # Land the base drone
    baseDrone.land()

elif(sys.argv[1][0:5] == "rover"):
    if(sys.argv[1][-1] == "1"):
        rover1Drone = Drone(rover1VehicleIP)
        print("=====ROVER 1=====")
        ROVER_ALT = ROVER1_ALT
    elif(sys.argv[1][-1] == "2"):
        rover2Drone = Drone(rover2VehicleIP)
        print("=====ROVER 2=====")
        ROVER_ALT = ROVER2_ALT
    else:
        print("Incorrect Sequence Number of Rover")
####################################################### 0331
    ''' Setting up client '''
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ip = sys.argv[2]
    port = int(sys.argv[3])
    client.connect((ip,port))
    print("Rover Connection Established")

    # Waiting for "TAKEOFF" from base
    msg = vehicle.receiveInfo(client)
    if(msg != "TAKEOFF"):
        print("Received incorrect message from base:", msg)
        sys.exit()
    print("Received TAKEOFF")

    vehicle.takeoff(ROVER_ALT)

    # Tell base that rover has tookoff
    vehicle.sendInfo(client, "TOOKOFF")
    print("Sent TOOKOFF")

    
    # Follow the base drone
    while(1):
        msg = vehicle.receiveInfo(client)

        if(msg == "LAND"):
            print("Received LAND")
            break
        elif(type(msg) == LocationGlobalRelative):
            msg.alt = ROVER_ALT
            print("Received target:",msg)
            vehicle.flyToPointNonBlocking(msg, 2)
    
    # Landing the rover drone
    vehicle.land()

    # Make sure the vehicle is landed (cause land() is aync) and disarmed
    while(vehicle.vehicle.armed):
        time.sleep(1)

    time.sleep(1)
    print("Rover landed")

    # Tell base that rover has landed
    vehicle.sendInfo(client, "LANDED")
    print("Sent LANDED")



else:
    print("Please specify which drone it is")