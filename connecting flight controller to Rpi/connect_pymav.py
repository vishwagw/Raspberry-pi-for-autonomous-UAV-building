# connectin using the pymavlink
from pymavlink import mavutil

# connecting to RPi:
autumn_connect = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
autumn_connect.wait_heartbeat()
print("MAVLINK connection successfully established.")

