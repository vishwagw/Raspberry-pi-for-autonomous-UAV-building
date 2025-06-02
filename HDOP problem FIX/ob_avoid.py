from dronekit import connect, VehicleMode
import time
import cv2
import numpy as np

# Connect to the vehicle
autumn1 = connect('/dev/ttyAMA0', baud=57600, wait_ready=True)

# Camera Initialization (adjust index if needed)
cap = cv2.VideoCapture(0)  # 0 is the default camera

# Function to detect obstacles
def detect_obstacle(frame, threshold=1500):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > threshold:  # Obstacle detected based on area size
            return True
    return False

# Arm and Takeoff with Obstacle Avoidance
def arm_and_takeoff(target_altitude):
    print("Arming motors...")
    autumn1.mode = VehicleMode("GUIDED")
    autumn1.armed = True

    # Wait until the vehicle is armed
    while not autumn1.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Checking for obstacles before takeoff...")
    # Check for obstacles before takeoff
    ret, frame = cap.read()
    if ret and detect_obstacle(frame):
        print("Obstacle detected! Aborting takeoff.")
        autumn1.armed = False
        return

    print("No obstacles detected. Taking off!")
    autumn1.simple_takeoff(target_altitude)

    # Monitor altitude and obstacles during ascent
    while True:
        alt = autumn1.location.global_relative_frame.alt
        print(f"Altitude: {alt:.2f} m")

        # Check for obstacles mid-ascent
        ret, frame = cap.read()
        if ret and detect_obstacle(frame):
            print("Obstacle detected during ascent! Hovering...")
            autumn1.mode = VehicleMode("LOITER")
            time.sleep(5)  # Hover and reassess
            autumn1.mode = VehicleMode("GUIDED")
        
        if alt >= target_altitude * 0.95:
            print("Target altitude reached")
            break
        time.sleep(1)

# Adjust Parameters to Ignore GPS HDOP
autumn1.parameters['GPS_HDOP_GOOD'] = 5.0
autumn1.parameters['ARMING_CHECK'] = 0  # Disables arming checks (use cautiously)

# Start Takeoff Process
arm_and_takeoff(10)  # Target: 10 meters

# Hold Position, Then Land
print("Hovering for observation...")
time.sleep(10)
print("Landing...")
autumn1.mode = VehicleMode("LAND")

# Release Resources
cap.release()
autumn1.close()

