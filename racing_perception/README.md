# Adaptive Corridor-Aware Racing (ROS2 Humble)

A complete **perception → behavior → control** pipeline for a simplified autonomous racing scenario using **ROS2 Humble**.  
The robot uses simulated LiDAR to estimate corridor width, classify driving risk, decide behaviour, and generate safe speed & steering commands.

---

## 🚀 Features

### 🟢 Fake LiDAR Node
- Publishes `/scan` with varying corridor widths  
- Simulates transitions: **SAFE → NARROW → CRITICAL**

### 🔵 Perception Node (`racing_perception_node`)
Computes:
- front distance  
- left & right free space  
- corridor width  
- risk level (SAFE / NARROW / CRITICAL)  
- steering command (−1 left, +1 right)  
- recommended safe speed  
- behaviour: CRUISE / SHIFT_LEFT / SHIFT_RIGHT / STOP

Publishes topics:
- `/racing/corridor_width`
- `/racing/risk_level`
- `/racing/safe_speed`
- `/racing/steering_cmd`
- `/racing/behavior`

### 🔴 Speed Controller Node (`speed_controller_node`)
- Converts safe speed + steering into `/cmd_vel`
- Smooth acceleration/deceleration
- Behaviour-based output (STOP / SLOW / CRUISE)

### 🟡 Data Logger Node
Logs:
- corridor width  
- risk level  
- behaviour  
- steering  
- safe speed  
- final `/cmd_vel` (linear & angular)

Saves logs as CSV files for analysis.

### 🧪 Analysis Tools (`analysis/`)
Generates automatic plots:
- `corridor_width.png`
- `safe_speed.png`
- `steering_cmd.png`
- `linear_speed.png`
- `angular_speed.png`

---

## 🏗 System Architecture

```

Fake LiDAR (/scan)
↓
Perception Node
↓   (corridor width, risk, speed, steering, behaviour)
Speed Controller Node
↓
/cmd_vel
↓
Robot / Simulation

```

Logger subscribes to all topics and records results.

---

## 📦 Project Structure

```

racing_perception/
├── README.md
├── package.xml
├── setup.py
├── launch/
│   └── racing_perception_launch.py
├── racing_perception/
│   ├── **init**.py
│   ├── fake_lidar_node.py
│   ├── perception_node.py
│   ├── speed_controller_node.py
│   └── data_logger_node.py
└── analysis/
├── plot_logs.py
├── corridor_width.png
├── safe_speed.png
├── steering_cmd.png
├── linear_speed.png
└── angular_speed.png

````

---

## ▶️ How to Run

### 1. Build the workspace
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
````

### 2. Launch the full system

```bash
ros2 launch racing_perception racing_perception_launch.py
```

---

## 📊 Log File Location

CSV logs are saved in:

```
ros2_ws/install/racing_perception/lib/python3.10/site-packages/racing_perception/logs/
```

---

## 🎯 Why this project is relevant

This project demonstrates:

* corridor-based risk estimation
* autonomous racing behaviour selection
* real-time ROS2 control
* logging + offline evaluation
* clean modular design for racing/autonomy tasks

A strong portfolio project for **Autonomous Vehicles / Robotics / RoboRacer** positions.
