'''
Check if the state report feature in 'Drone' class works
'''
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import sys
import os
import math
import socket

#sys.path.append("..")
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, parent_dir)

from Drone import Drone
from RepeatTimer import RepeatTimer
from Internet import checkInternetConnection

connection_strings = ["tcp:127.0.0.1:5762"]
#connection_string = "/dev/tty.usbmodem14101"


''' Connect to vehicle '''
for connection_string in connection_strings:
    drone = Drone(connection_string)
    if(drone.connected): break
drone.setStateReport(3)
try:
    while True:
        time.sleep(1)
        #print("main loop")
except KeyboardInterrupt:
    drone.cancelStateReport()
    #time.sleep(1)
    drone.vehicle.close()
    #drone.close()
