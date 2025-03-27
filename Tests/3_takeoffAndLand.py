'''
This test lets a drone to take off, hold for a few seconds in air, and land, checking if the drone is physically ready for flight.
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
from RepeatTimer import RepeatTimer
from Internet import checkInternetConnection

connection_strings = ["tcp:127.0.0.1:5762"]
#connection_strings = ["tcp:127.0.0.1:5762","tcp:127.0.0.1:5772"]
#connection_string = "tcp:127.0.0.1:5762"
#connection_string = "/dev/tty.usbmodem14101"

''' Connect to vehicle '''
for connection_string in connection_strings:
    drone = Drone(connection_string)
    if(drone.connected): break
drone.setStateReport(5)

''' Setting up a checker to see if internet connection works, otherwise land the vehicle'''
#checkConnectTimer = RepeatTimer(10,checkInternetConnection,args=(drone,))
#checkConnectTimer.start()
#print("Check Connect Timer Set")

drone.takeoff(10)
time.sleep(10)
drone.land()

try:
    while True:
        time.sleep(1)
        #print("main loop")
except KeyboardInterrupt:
    drone.cancelStateReport()
    #checkConnectTimer.cancel()
    #time.sleep(1)
    drone.vehicle.close()
    

