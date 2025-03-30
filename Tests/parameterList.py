from dronekit import connect, VehicleMode
import time
import os, sys

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, parent_dir)

from Drone import Drone

connection_string = "tcp:127.0.0.1:5762"
drone=Drone(connection_string)
#vehicle = connect(connection_string, wait_ready=True)
#vehicle.wait_ready('autopilot_version')
print ("FORMAT_VERSION: %s \n" % drone.vehicle.parameters['FORMAT_VERSION'])
print ("FSYSID_THISMAV: %s \n" % drone.vehicle.parameters['SYSID_THISMAV'])
print ("SCR_ENABLE: %s \n" % drone.vehicle.parameters['SCR_ENABLE'])
print ("RTL_ALT: %s \n" % drone.vehicle.parameters['RTL_ALT'])
print ("RTL_SPEED: %s \n" % drone.vehicle.parameters['RTL_SPEED'])
drone.vehicle.parameters['RTL_SPEED']=2.0
print ("RTL_SPEED: %s \n" % drone.vehicle.parameters['RTL_SPEED'])

'''
print ("\nPrint all parameters (iterate `vehicle.parameters`):")
for key, value in vehicle.parameters.items():
    print (" Key:%s Value:%s" % (key,value))
'''
drone.closeConn()