# this python script is used for building a basic connection between raspberry pi and Flight contoller.
# using dronkit library:
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# connecting the raspberry pi to arducopter :
print("connecting to autumn1")
autumn1 = connect('/dev/ttyACM0', baud=115200, wait_ready=True)
print("successfully connected to autumn1", autumn1.version)

-----------------
# using pymavlink library;
from pymavlink import mavutil

# connecting to RPi:
autumn_connect = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
autumn_connect.wait_heartbeat()
print("MAVLINK connection successfully established.")
