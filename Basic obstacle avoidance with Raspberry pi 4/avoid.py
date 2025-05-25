# using the ultrasonic sensors for the prototype model
import RPi.GPIO as GPIO
import time

#pins in GPIO (General Purpose Input Output)
TRIG = 18
ECHO = 24

# GPIO setup:
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# determine the safe distance:
SAFE_DISTANCE = 50

# function for getting the distance:
def get_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        start_time = time.time()

    while GPIO.input(ECHO) == 1:
        end_time = time.time()

    
    # Calculate distance (speed of sound = 34300 cm/s)
    duration = end_time - start_time
    distance = (duration * 34300) / 2
    return distance

try:
    while True:
        dist = get_distance()
        print(f"Distance: {dist:.2f} cm")
        time.sleep(0.1)

except KeyboardInterrupt:
    GPIO.cleanup()

# function for obstacle avoidance:
def avoid_obstacle():
    distance = get_distance()
    if distance < SAFE_DISTANCE:
        print("Obstacle detected! Taking action...")
        # Send command to the flight controller to change direction
        # For example, stop or move backward
    else:
        print("Path is clear. Proceeding...")

