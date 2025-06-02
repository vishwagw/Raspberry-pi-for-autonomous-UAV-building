from dronekit import connect, VehicleMode
import time

# Connect to the vehicle (adjust the connection string as needed)
autumn1 = connect('/dev/ttyACM0', baud=115200, wait_ready=True)

# Function to arm and takeoff
def arm_and_takeoff(target_altitude):
    print("Arming motors...")
    autumn1.mode = VehicleMode("GUIDED")
    autumn1.armed = True

    # Wait until the vehicle is armed
    while not autumn1.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    autumn1.simple_takeoff(target_altitude)

    # Wait until the vehicle reaches the target altitude
    while True:
        print(f"Altitude: {autumn1.location.global_relative_frame.alt}")
        if autumn1.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Target altitude reached")
            break
        time.sleep(1)

# Ignore GPS HDOP
autumn1.parameters['GPS_HDOP_GOOD'] = 5.0  # Set a high HDOP threshold
autumn1.parameters['ARMING_CHECK'] = 0     # Disable all arming checks (not recommended for safety)

# Start the takeoff process
arm_and_takeoff(10)  # Takeoff to 10 meters

# Land after some time
time.sleep(10)
print("Landing...")
# autonomous landing
autumn1.mode = VehicleMode("LAND")

# Close the connection
autumn1.close()



    