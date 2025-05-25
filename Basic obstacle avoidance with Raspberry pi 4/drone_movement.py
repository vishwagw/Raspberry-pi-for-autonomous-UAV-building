# using dronekit to control movements:
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# connecting to vehicle:
vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)

# Arm and take off
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
vehicle.simple_takeoff(altitude)

# Adjust velocity based on obstacle data
def set_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # Control mask
        0, 0, 0, vx, vy, vz,
        0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

set_velocity(1, 0, 0)  # Move forward with 1 m/s

