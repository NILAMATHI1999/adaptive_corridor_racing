# Adaptive Corridor-Aware Racing (ROS2 Humble)

A complete **perception → behaviour → control → SLAM → navigation** pipeline for autonomous racing and indoor robot navigation using **ROS2 Humble**, **Gazebo**, **RViz**, **SLAM (Cartographer)**, and **Navigation2 (Nav2)**.

The robot can:
- simulate LiDAR-based corridor racing,
- explore and map a simulated environment,
- save the generated map,
- reload it for Navigation2,
- navigate autonomously to user-defined goals in Gazebo.

---

# 🚀 NEW: Stage 1 – SLAM + Navigation2 (Robot Autonomy)

## 🟢 Gazebo Simulation
- TurtleBot3 Burger in `turtlebot3_world`
- LiDAR enabled
- `/scan` used for mapping and planning

## 🔵 SLAM Mapping (Cartographer)
- Ran SLAM to create a 2D occupancy grid map  
- Saved to:
  - `maps/tb3_map.pgm`
  - `maps/tb3_map.yaml`

## 🟡 Autonomous Exploration Node (`tb3_auto_explore`)
A custom Python ROS2 node that drives the robot automatically:
- Moves forward for a fixed period  
- Rotates to scan new areas  
- Repeats cycle  
- Enables automated SLAM mapping without teleop

## 🔴 Navigation2 (Nav2)
Using the saved map:
- Loaded map via Map Server  
- Set initial pose in RViz2  
- Planned and executed full navigation goals  
- Robot successfully moved in **Gazebo** following Nav2 global path

Screenshots available in:
```

/docs/screenshots/

```

These include:
- SLAM mapping  
- Nav2 global/local costmaps  
- Navigation goal execution  
- Gazebo robot motion  

---

# 📸 Evidence (Screenshots)

**Folder:** `docs/screenshots/`

Contains:
- `rviz_map_loaded.png`
- `rviz_nav2_path.png`
- `rviz_costmap_view.png`
- `rviz_navigation_complete.png`
- `gazebo_environment.png`

These verify that the robot:
- maps the environment,
- loads the map,
- localizes,
- plans a path,
- and navigates successfully in simulation.

---

# 🏁 Stage 2 – Corridor Racing (Perception → Behaviour → Control)

## 🟢 Fake LiDAR Node
- Publishes `/scan` with variable corridor widths  
- Simulates transitions: **SAFE → NARROW → CRITICAL**

## 🔵 Perception Node (`racing_perception_node`)
Computes:
- front distance  
- corridor width  
- risk level  
- steering  
- behaviour (CRUISE / SHIFT_LEFT / SHIFT_RIGHT / STOP)

Publishes:
- `/racing/corridor_width`
- `/racing/risk_level`
- `/racing/safe_speed`
- `/racing/steering_cmd`
- `/racing/behavior`

## 🔴 Speed Controller Node
Converts behaviour + safe speed → `/cmd_vel`

## 🟡 Data Logger Node
Logs behaviour, speed, steering, and `/cmd_vel` to CSV.

---

# 🏗 System Architecture

```

Gazebo ↔ SLAM ↔ Saved Map ↔ Nav2
↓
Perception Node
↓
Speed Controller
↓
/cmd_vel
↓
Robot

```

---

# 📦 Project Structure

```

adaptive_corridor_racing/
│
├── racing_perception/          # Corridor perception & behaviour
│
├── maps/                       # Saved SLAM maps (pgm + yaml)
│   ├── tb3_map.pgm
│   └── tb3_map.yaml
│
├── docs/
│   └── screenshots/            # RViz & Gazebo evidence
│
└── tb3_auto_explore/ (optional if added next)

````

---

# ▶️ How to Run

## 1. Build workspace
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
````

## 2. Run Gazebo with SLAM (mapping)

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py use_sim_time:=True
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

## 3. Run Autonomous Explore Node

```bash
ros2 run tb3_auto_explore auto_explore
```

## 4. Save Map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/tb3_map
```

## 5. Run Navigation2 with Saved Map

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/tb3_map.yaml
```

---

# 🎯 Why this project is powerful for robotics/research

This project now demonstrates:

### ✔ SLAM mapping

### ✔ Map saving + reuse

### ✔ Nav2 navigation

### ✔ Gazebo simulation

### ✔ RViz visualization

### ✔ Autonomous behaviour design

### ✔ Perception → Control pipeline

### ✔ ROS2 modular coding

### ✔ Real robot navigation logic

This is a **full robotics autonomy system**, suitable for internships, research labs, and RoboRace-style challenges.

````

