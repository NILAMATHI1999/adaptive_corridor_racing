Okay Nila — here is the **correct README content** you should paste.

Since your file is already open in `nano`, do this:

👉 **Copy EVERYTHING below**
👉 Paste it into your empty README.md
👉 Then tell me **“pasted”** and I will tell you how to save + push it.

---

# 📄 **PASTE THIS INTO README.md**

```
# Adaptive Corridor-Aware Racing (ROS2 Humble)

A complete **perception → behavior → control** pipeline for a simplified autonomous racing scenario using **ROS2 Humble**.  
This project shows how a robot can use LiDAR to estimate free corridor space, classify risk (SAFE / NARROW / CRITICAL), select behavior, and generate smooth speed + steering commands.

---

## 🚀 Features

### 🟢 Fake LiDAR Node
- Publishes `/scan` with changing corridor widths.
- Simulates SAFE → NARROW → CRITICAL transitions.

### 🔵 Perception Node (`racing_perception_node`)
Computes:
- front obstacle distance  
- left & right free space  
- corridor width  
- risk level (SAFE / NARROW / CRITICAL)  
- steering command (−1 left, +1 right)  
- recommended safe speed  

Publishes:
- `/racing/corridor_width`
- `/racing/risk_level`
- `/racing/safe_speed`
- `/racing/steering_cmd`
- `/racing/behavior`

### 🔴 Speed Controller Node (`speed_controller_node`)
- Converts safe speed + steering into actual `/cmd_vel`
- Smooth acceleration and deceleration
- Publishes state: STOP / SLOW / CRUISE

### 🟡 Data Logger Node
Logs:
- corridor width  
- risk level  
- safe speed  
- steering  
- behavior  
- cmd_vel  

Saves data as CSV for analysis.

### 🧪 Analysis Script (`analysis/plot_logs.py`)
Automatically generates:
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
Perception → corridor, risk, steering, safe_speed, behavior
↓
Speed Controller → /cmd_vel
↓
Robot / Simulation

```

Logger subscribes to everything and creates CSV logs.

---

## 📦 Package Structure

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

### Build:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
````

### Launch everything:

```bash
ros2 launch racing_perception racing_perception_launch.py
```

---

## 📊 Log File Location

```
ros2_ws/install/racing_perception/lib/python3.10/site-packages/racing_perception/logs/
```

---

## 🎯 Relevance to RoboRacer / Autonomous Racing

* Demonstrates a **ROS2 racing pipeline**: perception → decision → control
* Implements **risk-aware corridor estimation**
* Includes behavior planning (CRUISE, SHIFT_LEFT, SHIFT_RIGHT, STOP)
* Full **logging + analysis** system for evaluating robot performance

This project shows strong understanding of **autonomy fundamentals**, making it suitable as a portfolio project for robotics, autonomous driving, and RoboRacer applications.

```

---
