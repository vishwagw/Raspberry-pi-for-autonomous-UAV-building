# using dronekit:
from dronekit import connect, VehicleMode
import time

# connection:
autumn1 = connect('/dev/ttyACM0', baud=115200, wait_ready=True)

# function for arming and taking off the vehicle:
def arm_takeoff(t_alt):
    print("Arming motors..")
    autumn1.mode = VehicleMode("GUIDED")
    autumn1.armed = True
    while not autumn1.armed:
        time.sleep(1)

    print("Taking off the autumn-1...")
    autumn1.simple_takeoff(t_alt)
    while True:
        print(f"Altitude: {autumn1.location.global_relative_frame.alt}")
        if autumn1.location.global_relative_frame.alt >= t_alt * 0.95:
            print("Autumn-1 has Reached target altitude")
            break
        time.sleep(1)

# initialize the function:    
arm_takeoff(5)
# landing back to same location by changing vehicle mode:
autumn1.mode = VehicleMode("LAND")
autumn1.close()


