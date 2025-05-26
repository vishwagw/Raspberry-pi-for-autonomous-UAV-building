# Using Raspberry Pi for autonomous drone building & programming.

Autonomous vehicle are the next big evolution in transportation industry. Big companies including Tesla, BYD are consitantly working on building state-of-the-art autonomous vehicle systems and as a result, technology is being advanced in a rapid speed. But, not only manned vehicles but also unmanned vehicles can be build with autonomous capabilties. 

Building an autonomous vehicles require a complex interlinked connection between software and hardware. one of the major hardware equipment is edge computin devices. They are generally known as single-board computers.

## Single-board computers:
They are simply the computers which integrated into a single circuit board. Unlike traditional desktop computers, which use separate components like a motherboard, CPU, and memory, an SBC combines all essential elements onto one board. This includes the processor, memory, input/output functions, and other critical features. 

## raspberrry pi:
The Raspberry Pi was originally created to help teach computer science in schools, but gained popularity for many other uses due to its low cost, compact size, and flexibility. It is now used in areas such as industrial automation, robotics, home automation, IoT devices, and hobbyist projects. Raspberry pi is probably the most common single-board computer unit for general purposes. The raspberry pi computers have their own operating system called raspberry pi OS. It was previously known as Raspbian OS. It is an open source OS based on linux kernel. There are several raspbery pi models such as model 2, model 3, model 4 and most advanced model 5. But for this project I highly recommend model 5. 

## Raspberry Pi for autonomous drone control:
As mentioned before, to build an autonomous drone the onboard edge computing unit is essential. Raspberry pi is viable solution as an ege computing unit. 

Building an autonomous drone using a Raspberry Pi is a complex but rewarding project that combines hardware integration, software development, and control systems.

### autonomous drone requirement overview:
An autonomous drone must perform tasks such as navigation, obstacle avoidance, and flight stabilization without human intervention. The Raspberry Pi, a compact single-board computer, serves as the drone’s brain, handling high-level tasks like processing sensor data, running algorithms for autonomy, and communicating with ground stations. Other components, like flight controllers, handle low-level tasks such as motor control and stabilization.

Key requirements for an autonomous drone include:

* Flight control: Stabilizing the drone and executing flight commands.
* Sensors: For navigation, obstacle detection, and environmental awareness.
* Processing: Running algorithms for path planning, computer vision, or GPS navigation.
* Communication: Sending/receiving data to/from a ground station or remote control.
* Power management: Ensuring all components are powered efficiently.

### Hardware Assembly

1. Frame Setup:
Assemble the quadcopter frame according to the manufacturer’s instructions.
Mount the motors securely to the frame arms, ensuring proper alignment.

2. Install ESCs and Motors:
Connect each motor to its corresponding ESC. Solder ESC power lines to the power distribution board (PDB).
Connect ESC signal lines to the flight controller’s motor outputs.
3. Mount Flight Controller:
Secure the flight controller (e.g., Pixhawk or Navio2) to the frame using vibration-dampening mounts to reduce noise in sensor data.
Connect the flight controller to the Raspberry Pi via UART (serial), SPI, or USB, depending on the controller’s requirements.
4. Attach Raspberry Pi:
Mount the Raspberry Pi in a protective case on the frame, ensuring access to USB, HDMI, and GPIO pins.
Power the Pi using a 5V BEC (Battery Eliminator Circuit) connected to the LiPo battery via the PDB.

5. Install Sensors:
Connect the GPS module to the flight controller (usually via UART or I2C).
Mount the Raspberry Pi Camera facing forward for computer vision tasks or downward for optical flow.
Attach ultrasonic or LiDAR sensors for obstacle detection, connected to the Pi’s GPIO or the flight controller.
6. Wiring and Power:
Use a PDB to distribute power from the LiPo battery to the ESCs, flight controller, and Raspberry Pi.
Double-check connections for polarity and secure all wires to avoid interference with propellers.

7. Propeller Installation:
Attach propellers after all testing to avoid accidents. Ensure correct propeller orientation (CW/CCW) for each motor.

### Software integration

#### Raspberry Pi Configuration:

1. Operating System:
Install Raspberry Pi OS (Lite version for better performance) on the MicroSD card using Raspberry Pi Imager.
Enable SSH and Wi-Fi for remote access.

2. Dependencies:
Install Python (pre-installed on Raspberry Pi OS), pip, and libraries like opencv-python (for computer vision), pymavlink (for communication with the flight controller), and dronekit (for high-level drone control).
Example command: sudo pip install opencv-python pymavlink dronekit.

3. Networking:
Configure the Pi as a Wi-Fi access point or connect it to a ground station via Wi-Fi or telemetry radio.
Flight Controller Firmware:

4. Choose Firmware:
Ardupilot: Open-source, feature-rich, supports advanced autonomy features like waypoint navigation.
PX4: Lightweight, suitable for custom applications, integrates well with ROS (Robot Operating System).
Betaflight: For high-performance drones, less focus on autonomy.
5. Install Firmware:
Use software like Mission Planner (Ardupilot) or QGroundControl (PX4) to flash the firmware onto the flight controller.
Configure parameters like frame type (quadcopter), motor layout, and sensor calibration.

6. Connect to Raspberry Pi:
Use MAVLink protocol for communication between the Raspberry Pi and flight controller.
Example: Connect via serial (/dev/ttyS0 or /dev/ttyUSB0) and test using a simple Python script with pymavlink.

### Programming for Autonomy

#### Key Autonomous Features:

1. Waypoint Navigation:
Use GPS coordinates to define a flight path.
Implement using DroneKit or ROS to send waypoints to the flight controller.

2. Obstacle Avoidance:
Process camera or LiDAR data to detect obstacles.
Use algorithms like SLAM (Simultaneous Localization and Mapping) or simple threshold-based avoidance.
Example: Stop or reroute if an obstacle is detected within 2 meters using LiDAR data.

3. Position Hold:
Use GPS, optical flow, or IMU data to maintain a stable hover.
Configure the flight controller for “Loiter” mode (Ardupilot) or “Position” mode (PX4).

4. Object Tracking:
Use computer vision to track a target (e.g., a person or marker).
Implement with OpenCV and send position updates to the flight controller.

#### Control Loop:

High-Level Control (Raspberry Pi): Processes sensor data, runs path-planning algorithms, and sends high-level commands (e.g., “go to waypoint”).
Low-Level Control (Flight Controller): Handles motor control, PID loops for stabilization, and sensor fusion (IMU, GPS).

#### Challenges:
* Power Management: The Raspberry Pi consumes significant power, reducing flight time. Optimize code and use a high-capacity LiPo battery.
* Processing Power: Real-time computer vision on a Raspberry Pi can be slow. Consider offloading tasks to a companion computer or optimizing algorithms.
* Weight: Adding sensors and the Pi increases payload, affecting flight dynamics. Choose lightweight components.
* Latency: Communication delays between the Pi and flight controller can cause instability. Use efficient protocols like MAVLink.

This project repository is a collection of several sub-porjects. I will try to create seperate readme file for each sub-project. 

