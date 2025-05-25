# using ultrasonic sensors:

import RPi.GPIO as GPIO
import time

# GPIO setup
GPIO.setmode(GPIO.BCM)
TRIG = 23
ECHO = 24
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# measuring the distance to objects:
def measure_dist():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        start_time = time.time()
    while GPIO.input(ECHO) == 1:
        end_time = time.time()

    distance = (end_time - start_time) * 34300 / 2  # cm
    return distance

try:
    while True:
        dist = measure_dist()
        print(f"Distance: {dist} cm")
        time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()


