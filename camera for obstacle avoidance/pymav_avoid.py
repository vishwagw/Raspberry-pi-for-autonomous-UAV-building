from pymavlink import mavutil
import cv2
import torch
import numpy as np

# loading the yolo model:
obj_model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

# initializing the connection to vehicle:
autumn1 = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
autumn1.wait_heartbeat()
print("Connection initialized successfully.")

# open camerafeed:
cap = cv2.VideoCapture(0)

# defining the abstacle avoidance threshold:
OBSTACLE_THRESHOLD = 200

# function for sending mavlink commands:
# this function will send commands to adjust the drone moments
# to avoid obstacles:
def send_mavlink_command(x_offset):
    if x_offset < -50: # when object is on left side of vehicle direction
        print("Obstacle detected on the left, moving right")
        autumn1.mav.set_position_target_local_ned_send(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000, 1, 0, 0, 0, 0, 0, 0, 0, 0
        )

    elif x_offset > 50: # when object is on right side
        print("Obstacle detected on the right, moving left")
        autumn1.mav.set_position_target_local_ned_send(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000, -1, 0, 0, 0, 0, 0, 0, 0, 0
        )

    else:
        print("Moving forward")
        autumn1.mav.set_position_target_local_ned_send(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000, 0, 1, 0, 0, 0, 0, 0, 0, 0
        )

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # runing the object detection program
    results = obj_model(frame)
    #extracting the detections:
    detection = results.xyxy[0].numpy()

    obstacle_detected = False
    x_offset = 0
    
    for *xyxy, conf, cls in detection:
        x_min, y_min, x_max, y_max = map(int, xyxy)
        width = x_max - x_min

        if width > OBSTACLE_THRESHOLD:
            obstacle_detected = True
            x_offset = (x_min + x_max) // 2 - frame.shape[1] // 2

            # Draw bounding box
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)

    if obstacle_detected:
        send_mavlink_command(x_offset)

    # Display the frame
    cv2.imshow("Obstacle Avoidance", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()





