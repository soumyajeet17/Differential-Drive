ROS Differential Drive Robot with ESP32 and LiDAR
A ROS-based differential drive robot using an ESP32, L298N motor driver, encoder feedback, and a YDLidar X2. The robot communicates over Wi-Fi via rosserial (TCP mode) and supports closed-loop PID control and obstacle avoidance.

Features
Differential Drive control with encoder feedback.

360° LiDAR-based obstacle detection using YDLidar X2.

Closed-loop PID motor control for precise movement.

Wi-Fi Communication between ESP32 and a ROS master computer using rosserial_tcp.

Full ROS Integration with standard message types (geometry_msgs, sensor_msgs).

System Architecture
The ESP32 serves as the low-level hardware controller. It reads sensor data (encoders, LiDAR) and controls the motors. It communicates all information over Wi-Fi to the ROS Master running on a separate computer, which handles high-level logic.

Code snippet

graph LR
    A[Computer / ROS Master] -- Motor Commands (Twist) --> B[ESP32];
    B -- Encoder Ticks & LiDAR Scans --> A;
Hardware Components
Microcontroller: ESP32 DevKit V1

Motor Driver: L298N H-Bridge

Motors: 2x DC Geared Motors with Quadrature Encoders

LiDAR: YDLidar X2

Power: 12V Battery Pack / LiPo Battery

Chassis: A simple 2-wheel drive chassis with a front caster wheel.

Software & ROS Packages
ROS Version: ROS Noetic (Recommended) or Melodic

ROS Packages:

rosserial: For communication between ROS and the ESP32.

ydlidar_ros: Official driver for the YDLidar.

sensor_msgs: For LiDAR scan data.

geometry_msgs: For velocity commands (Twist).

tf: For publishing robot frame transformations.

⚙️ Setup and Installation
1. Prerequisites
A computer with Ubuntu 20.04 and ROS Noetic installed.

PlatformIO or Arduino IDE installed and configured for the ESP32.

All necessary hardware components assembled and wired. (You can add a wiring diagram to the docs/ folder).

2. Firmware Setup (ESP32)
Open the firmware/ directory in PlatformIO or Arduino IDE.

Install the required libraries (e.g., Rosserial Arduino Library, ESP32...).

Update the config.h file with your Wi-Fi credentials and the IP address of your ROS Master computer.

Upload the firmware to your ESP32.

3. ROS Workspace Setup
Clone this repository into your catkin workspace's src folder:

Bash

cd ~/catkin_ws/src
git clone https://github.com/yourusername/diff-drive-ros.git
Install any missing ROS dependencies using rosdep:

Bash

cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
Build the workspace:

Bash

catkin_make
Source the new environment:

Bash

source devel/setup.bash
🚀 Running the Robot
Start the ROS Serial Server: Open a new terminal and run the rosserial TCP socket node. The ESP32 will connect to this.

Bash

rosrun rosserial_server socket_node
Note: By default, it listens on port 11411. Ensure your ESP32 firmware is configured for this port.

Launch the Robot: Open a second terminal, source your workspace, and run the main launch file. This will start the LiDAR driver and other core nodes.

Bash

source ~/catkin_ws/devel/setup.bash
roslaunch diff_drive_robot bringup.launch
Control the Robot (Optional): You can now send commands to the robot, for example, using a teleop node in a third terminal:

Bash

rosrun teleop_twist_keyboard teleop_twist_keyboard.py
Demo Video
(Click the thumbnail to watch on LinkedIn)

Note: Save a screenshot from your video as docs/demo.png in this repository for the thumbnail to appear.

Project Structure
├── firmware/         # ESP32 PlatformIO/Arduino code
├── src/              # ROS Python/C++ nodes
├── launch/           # ROS launch files
├── urdf/             # Robot description files (optional)
├── docs/             # Images and documentation
├── README.md         # This file
Author
Soumyajeet Mahapatra

LinkedIn

GitHub
