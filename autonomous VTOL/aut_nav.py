from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# Connect to vehicle
vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)

# Takeoff Function
def arm_and_takeoff(target_altitude):
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)

    print("Taking off...")
    vehicle.simple_takeoff(target_altitude)
    while True:
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# Move to a GPS waypoint
def go_to_gps_waypoint(lat, lon, alt):
    waypoint = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(waypoint)

# Main execution
arm_and_takeoff(10)  # Take off to 10m height
go_to_gps_waypoint(37.7749, -122.4194, 10)  # Example coordinates

# Land after mission
vehicle.mode = VehicleMode("LAND")
vehicle.close()
