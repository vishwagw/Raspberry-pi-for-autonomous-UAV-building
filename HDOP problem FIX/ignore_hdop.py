# arming the vehicle without gps connection:
# for ignore High gps HDOP fix

from dronekit import connect, VehicleMode
import time

# connecting:
autumn1 = connect('/dev/ttyACM0', baud=115200, wait_ready=True)

# set the HDOP threshold:
# acceptalbe HDOP limit is 2
HDOP_threshold = 2.0

# function for gps fixing:
def check_gps_quality():
    if autumn1.gps_0.fix_type < 3:
        print("Warning, no reliable 3D GPS FIX")
        return False
    
    hdop = autumn1.gps_0.eph
    if hdop is None:
        print("HDOP data unavailable.")
        return False
    
    print(f"Current HDOP Data: {hdop}")
    if hdop > HDOP_threshold:
        print("HDOP Error detected. Ignoring GPS data.")
        return False
    return True

# building the loop:
try:
    while True:
        gps_ok = check_gps_quality()

        if not gps_ok:
            # switch to GUIDED mode and hold the position
            autumn1.mode = VehicleMode("BRAKE")
            print("Vehicle is on brake mode due to poor GPS.")
        
        time.sleep(1)

except KeyboardInterrupt:
    print("Script terminated by vehicle user.")

finally:
    autumn1.close()

    



