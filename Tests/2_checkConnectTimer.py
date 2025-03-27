'''
This test makes use of the connection checker feature and test if it works.
'''
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import sys
import os
import math
import socket
from datetime import datetime

#sys.path.append("..")
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, parent_dir)

from Drone import Drone
from RepeatTimer import RepeatTimer
from Internet import checkInternetConnection

f = open("connectiontest.txt", "w")
f.write("START\n")

connection_strings = ["tcp:127.0.0.1:5762"]

''' Connect to vehicle '''
for connection_string in connection_strings:
    drone = Drone(connection_string)
    if(drone.connected): break
drone.setStateReport(5)

''' Setting up a checker to see if internet connection works, otherwise land the vehicle'''
checkConnectTimer = RepeatTimer(3,checkInternetConnection,args=(drone,f,))
checkConnectTimer.start()
print("Check Connect Timer Set")
f.write("At time "+datetime.now().strftime("%H%M%S")+" Connect Timer set.\n")

try:
    while True:
        time.sleep(1)
        print("main loop")
except KeyboardInterrupt:
    drone.cancelStateReport()
    checkConnectTimer.cancel()
    #time.sleep(1)
    drone.vehicle.close()
    f.close()

''' 
Here you can check if it will try to land the vehicle after disconnecting wifi/Internet
'''

