# connecting by using dronekit:
#libs: 
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# connecting the raspberry pi to arducopter :
print("connecting to autumn1")
autumn1 = connect('/dev/ttyACM0', baud=115200, wait_ready=True)
print("successfully connected to autumn1", autumn1.version)


