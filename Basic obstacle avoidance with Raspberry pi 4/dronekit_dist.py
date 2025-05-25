from dronekit import connect, VehicleMode

# connecting to vehicles:
vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)

# function to change direction:
def change_direction():
    # Example of setting the pitch to move the drone backward
    vehicle.channels.overrides = {'2': 1500, '3': 1000}  # Modify as needed

