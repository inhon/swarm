## Overview
This project aimed to let multiple drones fly in a swarm automatically without human intervention by letting the drones communicate with one another. Further details as follows:
- Dronekit-Python library to operate the drone
- TCP for inter-drone communication
- A protocol to ensure no collision during takeoff and landing

In this project, one or more follower drones will follow a leader drone. The follower drones were also called 'Rover' while the leader drone was called 'Base', so don't get confused with the terms. 

## Protocol
The drones will fly at different altitudes to address the GNSS inaccuracy, but during takeoff and landing they can still collide, so we set up a protocol to tackle this. We let the drone that flies at the highest altitude take off first, and the second highest drone take off and so on. The landing sequence is the opposite, where the lowest drone lands first. To faciliate the protocol, we let the Base drone be the commander, which flies at the highest altitude, and command each Rover to take off and land according to the protocol. 

### Packages
- Drone.py: A class that contains basically all the needed functions to operate a drone.
- RepeatTimer.py: Contains a class that will create a thread which will periodically execute a designated function. It is used to periodically return the status of the drone (Drone.setStateReport) and check the internet connection (Internet.py)
- Protocol.py: A class to allow the TCP communication between a Base and a Rover.


