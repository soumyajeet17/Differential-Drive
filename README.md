# ROS Differential Drive Robot

A ROS-based differential drive robot using **ESP32**, **L298N motor driver**, **encoder feedback**, and **YDLidar X2**. The robot communicates over Wi-Fi via `rosserial` (TCP mode) and supports closed-loop PID control for straight-line motion and obstacle avoidance.

## Features
- **Differential Drive** with encoder feedback
- **LiDAR-based obstacle detection** using YDLidar X2
- **Closed-loop PID motor control**
- **Wi-Fi communication** between ESP32 and ROS using `rosserial_server`
- **ROS Integration** with custom topics for sensor data and motor commands

## System Architecture
```mermaid
graph LR
A[ESP32 #1] -->|LiDAR Data| B[ROS Master]
A -->|Encoder Feedback| B
B -->|Motor Commands| A
Hardware Used

    ESP32 DevKit

    L298N Motor Driver

    DC Motors + Encoders

    YDLidar X2

    12V Battery Pack

    Chassis with 2 wheels + caster

Software & ROS Packages

    ROS Melodic / Noetic

    rosserial_server (TCP)

    sensor_msgs

    geometry_msgs

    Custom Python nodes for PID and obstacle detection

Setup Instructions

    Clone the Repository

git clone https://github.com/yourusername/diff-drive-ros.git
cd diff-drive-ros

Start ROS Serial Server

rosrun rosserial_server socket_node tcp

Launch Robot Nodes

roslaunch diff_drive_robot bringup.launch

Run Control Node

    rosrun diff_drive_robot pid_control.py

Demo Video

(Click the thumbnail to watch on LinkedIn)

    📌 Note: Save a screenshot from your video as docs/demo.png in this repository for the thumbnail to appear.

Project Structure

├── firmware/          # ESP32 Arduino code
├── src/               # ROS Python nodes
├── launch/            # Launch files
├── urdf/              # Robot description
├── docs/              # Images and documentation
├── README.md          # This file

Author

Soumyajeet Mahapatra

    LinkedIn

    GitHub
