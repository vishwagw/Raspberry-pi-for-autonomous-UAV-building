import sys
#import pymavlink
from dronekit import connect
import drone

connect_drone = '/dev/ttyACM0'
baud_rate = 115200

print("Connecting to Autumn..")
autumn1 = connect(connect_drone, baud=baud_rate, wait_ready=True)
print("Succesfully connected to Autumn..")

current_cordinate = autumn1.get_location()
current = (current_cordinate.lat, current_cordinate.lon)

print(current)

drone.disconnect_drone()

