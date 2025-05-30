import drone
import sys

sys.path.insert(1, 'modules')

drone.connect_drone('/dev/ttyACM0')

current_cordinate = drone.get_location()
current = (current_cordinate.lat,current_cordinate.lon)

print(current)

drone.disconnect_drone()

