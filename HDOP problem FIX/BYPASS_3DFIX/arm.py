# using pymavlink:
from pymavlink import mavutil
from dronekit import connect, VehicleMode
import time

# connect to the vehicle:
autumn1 = connect('/dev/ttyACM0', baud=115200, wait_ready=True)

def force_arm_takeoff(target_altitude):
    # Set mode to GUIDED
    autumn1.mode = VehicleMode("GUIDED")
    time.sleep(2)

    # Force arm using MAVLink command (bypasses 3D fix requirement)
    print("Forcing arming without 3D fix...")
    autumn1.armed = True

    # Send MAVLink command to force arming
    msg = autumn1.message_factory.command_long_encode(
        0, 0,                         # target system, component
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
        0,                            # confirmation
        1, 0, 0, 0, 0, 0, 0           # param1=1 (arm), rest are unused
    )
    autumn1.send_mavlink(msg)
    autumn1.flush()

     # Wait until armed
    while not autumn1.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Armed! Taking off...")
    autumn1.simple_takeoff(target_altitude)

     # Monitor altitude
    while True:
        print(f"Altitude: {autumn1.location.global_relative_frame.alt}")
        if autumn1.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Target altitude reached")
            break
        time.sleep(1)

# Adjust parameters (optional safety overrides)
autumn1.parameters['ARMING_CHECK'] = 0
autumn1.parameters['GPS_HDOP_GOOD'] = 5.0

# Force arm and take off
# test altitude - 2m
force_arm_takeoff(2)

# Land after hovering
time.sleep(10)
print("Landing...")
autumn1.mode = VehicleMode("LAND")

# Close connection
autumn1.close()

