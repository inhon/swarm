from dronekit import connect, VehicleMode
import time

connection_string = "tcp:127.0.0.1:5762"
vehicle = connect(connection_string, wait_ready=True)
vehicle.wait_ready('autopilot_version')
print ("\nPrint all parameters (iterate `vehicle.parameters`):")
for key, value in vehicle.parameters.items():
    print (" Key:%s Value:%s" % (key,value))
vehicle.close()