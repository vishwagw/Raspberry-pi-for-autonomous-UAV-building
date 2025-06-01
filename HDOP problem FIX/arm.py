# arming the vehicle without gps connection:
# for ignore High gps HDOP fix

from dronekit import connect, VehicleMode
import time

# connecting:
autumn1 = connect('/dev/ttyACM0', baud=115200, wait_ready=True)

# function for ignoring gps checkup first:
def disable_gps_check():
    print("Disabling GPS arming checks...")
    autumn1.parameters['ARMING_CHECK'] = 0  # Disable all arming checks
    # Alternatively, set GPS_HDOP_GOOD to a higher threshold
    autumn1.parameters['GPS_HDOP_GOOD'] = 5.0

# now ariming function:
def arming():
    print("Arming the vehicle...")
    autumn1.mode = VehicleMode("GUIDED")
    while not autumn1.mode.name == 'GUIDED':
        print("Waiting for GUIDED mode...")
        time.sleep(1)

    autumn1.armed = True
    while not autumn1.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Vehicle is armed!")

# Main execution
disable_gps_check()
arming()

# Clean up
autumn1.close()
