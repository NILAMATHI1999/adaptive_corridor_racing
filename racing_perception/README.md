Adaptive Corridor-Aware Speed & Steering Control (ROS2 Humble)

A full perception → behavior → control pipeline for autonomous racing simulation.

🚀 Overview

This project implements a complete LiDAR-based autonomy pipeline using ROS2 Humble:

Fake LiDAR → publishes synthetic /scan data

Perception Node → estimates corridor width, detects obstacles, assigns risk (SAFE/NARROW/CRITICAL)

Behavior Planner → selects CRUISE / SHIFT_LEFT / SHIFT_RIGHT / STOP

Steering Controller → generates angular steering commands

Speed Controller → computes forward velocity

Logger Node → records all data to CSV

Analysis Module → plots performance graphs

This setup demonstrates core concepts used in racing autonomy, navigation, and robot behaviour planning, making it ideal for learning and project applications.

🛠 ROS2 Nodes
1️⃣ fake_lidar_node

Publishes synthetic LaserScan messages

Corridor width changes over time

Simulates SAFE → NARROW → CRITICAL transitions

2️⃣ racing_perception_node

Subscribes to /scan

Computes:

front obstacle distance

left & right free space

corridor width

risk level

recommended speed

steering (-1 left, +1 right)

high-level behavior

Publishes:

/racing/corridor_width
/racing/risk_level
/racing/safe_speed
/racing/steering_cmd
/racing/behavior

3️⃣ speed_controller_node

Converts recommended speed + steering into /cmd_vel

Smooth acceleration/deceleration

Labels CRUISE / SLOW / STOP states

4️⃣ data_logger_node

Logs all system outputs automatically to CSV

Used for analysis & graphs

5️⃣ Analysis (Python script)

Reads CSV logs

Generates plots:

Corridor width

Speed profile

Steering

Control signals

📊 Results
SAFE Zone

Corridor ≈ 10 m

Behavior: CRUISE

Steering: 0

Speed ≈ 0.6 → 1.2 m/s

NARROW Zone

Corridor ≈ 2 m

Behavior: SHIFT_LEFT / SHIFT_RIGHT

Steering: ±1

Speed: Moderate

CRITICAL Zone

Corridor ≈ 0.8 m

Behavior: STOP

Steering: 0

Speed ≈ 0

📈 Graphs

(place your .png images here once uploaded to GitHub)

▶️ How to Run
Build:
cd ~/ros2_ws
colcon build
source install/setup.bash

Launch full system:
ros2 launch racing_perception racing_perception_launch.py

Check topics:
ros2 topic list

View logs:

Located in:

ros2_ws/install/racing_perception/lib/python3.10/site-packages/racing_perception/logs/

📦 Folder Structure
racing_perception/
 ├── racing_perception/
 │    ├── perception_node.py
 │    ├── speed_controller_node.py
 │    ├── fake_lidar_node.py
 │    ├── data_logger_node.py
 │    └── __init__.py
 ├── analysis/
 │    ├── plot_logs.py
 │    └── *.png
 ├── launch/
 │    └── racing_perception_launch.py
 ├── package.xml
 ├── setup.py
 └── README.md
