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
    #vehicle = connect(connection_string, wait_ready=True)
    if(drone.connected): break

# Get all original channel values (before override)
print("Channel values from RC Tx:", drone.vehicle.channels)
#for channel in vehicle.channels:
#    print(f"Channel {channel}: {vehicle.channels[channel]}")

# Access channels individually
print("Read channels individually:")
print(" Ch1: %s" % drone.vehicle.channels['1'])
print(" Ch2: %s" % drone.vehicle.channels['2'])
print(" Ch3: %s" % drone.vehicle.channels['3'])
print(" Ch4: %s" % drone.vehicle.channels['4'])
print(" Ch5: %s" % drone.vehicle.channels['5'])
print(" Ch6: %s" % drone.vehicle.channels['6'])
print(" Ch7: %s" % drone.vehicle.channels['7'])
print(" Ch8: %s" % drone.vehicle.channels['8'])
print("Number of channels: %s" % len(drone.vehicle.channels))


# Override channels
print("\nChannel overrides: %s" % drone.vehicle.channels.overrides)

print("Set Ch2 override to 200 (indexing syntax)")
drone.vehicle.channels.overrides['2'] = 200
print(" Channel overrides: %s" % drone.vehicle.channels.overrides)
print(" Ch2 override: %s" % drone.vehicle.channels.overrides['2'])

print("Set Ch3 override to 300 (dictionary syntax)")
drone.vehicle.channels.overrides = {'3':300}
print(" Channel overrides: %s" % drone.vehicle.channels.overrides)

print("Set Ch1-Ch8 overrides to 110-810 respectively")
drone.vehicle.channels.overrides = {'1': 110, '2': 210,'3': 310,'4':4100, '5':510,'6':610,'7':710,'8':810}
print(" Channel overrides: %s" % drone.vehicle.channels.overrides) 


# Clear override by setting channels to None
print("\nCancel Ch2 override (indexing syntax)")
drone.vehicle.channels.overrides['2'] = None
print(" Channel overrides: %s" % drone.vehicle.channels.overrides) 

print("Clear Ch3 override (del syntax)")
del drone.vehicle.channels.overrides['3']
print(" Channel overrides: %s" % drone.vehicle.channels.overrides) 

print("Clear Ch5, Ch6 override and set channel 3 to 500 (dictionary syntax)")
drone.vehicle.channels.overrides = {'5':None, '6':None,'3':500}
print(" Channel overrides: %s" % drone.vehicle.channels.overrides) 

print("Clear all overrides")
drone.vehicle.channels.overrides = {}
print(" Channel overrides: %s" % drone.vehicle.channels.overrides) 

#Close vehicle object before exiting script
print("\nClose vehicle object")
drone.vehicle.close()