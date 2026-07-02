
---

# Author

Aayush Anil Mishra 
Undergraduate Mechatronics Engineering Student

---

````markdown
# VAMR_MSD
## Vision-Assisted Autonomous Mobile Robot with Mission-Level Task Scheduling and Docking in ROS2 Humble

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)]()
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420.svg)]()
[![Gazebo](https://img.shields.io/badge/Gazebo-11-orange.svg)]()
[![License](https://img.shields.io/badge/License-MIT-green.svg)]()

---

## Project Overview

VAMR_MSD (Vision-Assisted Autonomous Mobile Robot with Mission-Level Task Scheduling and Docking) is a modular autonomous mobile robotics platform developed in ROS2 Humble for research and educational applications in autonomous navigation, mission execution, and intelligent robot behavior.

The platform integrates:

- Vision-assisted object tracking
- SLAM-based environment mapping
- Localization using Adaptive Monte Carlo Localization (AMCL)
- Autonomous navigation using Nav2
- Mission-level multi-goal task scheduling
- Autonomous docking capability
- Simulation-first development workflow using Gazebo

Unlike conventional ROS mobile robot demonstrations which only showcase single-point navigation, VAMR_MSD focuses on complete mission execution pipelines including perception, planning, task scheduling, and energy-aware behavior.

---

# System Architecture

```text
                    +----------------+
                    | Vision System  |
                    | Camera + CV    |
                    +--------+-------+
                             |
                             v
+-----------+       +------------------+       +----------------+
| Lidar     | ----> | Localization &   | ----> | Nav2 Planner   |
| Sensor    |       | Mapping Stack    |       | Controller     |
+-----------+       +------------------+       +--------+-------+
                                                       |
                                                       v
                                          +------------------------+
                                          | Goal Queue Scheduler   |
                                          | Mission Management     |
                                          +-----------+------------+
                                                      |
                                                      v
                                           +----------------------+
                                           | Differential Drive   |
                                           | Mobile Platform      |
                                           +----------------------+
````

---

# Objectives

The project aims to develop a mobile robot capable of:

* Generating occupancy maps using SLAM
* Localizing itself within a previously generated map
* Performing autonomous navigation to target locations
* Executing multiple goals sequentially
* Tracking visual targets using onboard vision
* Returning to a charging dock autonomously
* Recording mission-level performance statistics

---

# Implemented Features

## Core Mobile Robotics Stack

### Differential Drive Mobile Robot

* Custom URDF/Xacro robot model
* ROS2 Control integration
* Differential drive controller

### Sensor Suite

* 2D LiDAR
* RGB Camera
* Depth Camera support
* Wheel Odometry

### Simulation Environment

* Gazebo Classic simulation
* Empty world testing environment
* Obstacle-rich navigation environment

---

## Mapping and Localization

### SLAM Toolbox

Supports online asynchronous SLAM for map generation.

Features:

* Real-time occupancy grid generation
* Loop closure support
* Persistent map saving

Launch:

```bash
ros2 launch vamr_msd online_async_launch.py
```

---

### Adaptive Monte Carlo Localization (AMCL)

Provides probabilistic localization using:

* Laser scan matching
* Particle filter localization
* Map-based pose estimation

Launch:

```bash
ros2 launch vamr_msd localization_launch.py
```

---

## Autonomous Navigation

Navigation is implemented using Nav2.

Capabilities:

* Global path planning
* Local obstacle avoidance
* Recovery behaviors
* Goal reaching

Launch:

```bash
ros2 launch vamr_msd navigation_launch.py
```

---

## Vision Assisted Navigation

The robot uses camera input for target detection and tracking.

Current implementation includes:

* Color-based object tracking
* Visual servoing
* Camera-guided motion commands

Launch:

```bash
ros2 launch vamr_msd ball_tracker.launch.py
```

Future upgrades include:

* YOLO object detection
* Semantic navigation
* Dynamic object following

---

## Mission-Level Goal Queue System

Traditional Nav2 navigation executes only one goal at a time.

VAMR_MSD introduces:

* FIFO mission queue
* Multiple waypoint execution
* Goal retry mechanism
* Pause and resume support
* Mission cancellation
* YAML mission file support

Example mission:

```yaml
goals:
  - x: 1.0
    y: 2.0
    yaw: 0.0

  - x: 3.5
    y: 1.2
    yaw: 1.57
```

---

## Autonomous Docking

The robot can automatically return to a docking station after mission completion.

Planned docking workflow:

```text
Mission Complete
        ↓
Battery Threshold Check
        ↓
Dock Pose Selection
        ↓
Visual Dock Detection
        ↓
Fine Alignment
        ↓
Charging State
```

---

## Mission Statistics

Mission execution metrics include:

* Total mission duration
* Distance travelled
* Average navigation speed
* Number of goals completed
* Number of retries
* Docking success rate

---

# Repository Structure

```text
vamr_msd/
│
├── CMakeLists.txt
├── package.xml
│
├── config/
│
├── description/
│
├── launch/
│
└── worlds/
```

---

## Configuration Directory

Contains parameter files for all subsystems.

| File                            | Purpose                                     |
| ------------------------------- | ------------------------------------------- |
| ball_tracker_params_robot.yaml  | Ball tracking parameters for physical robot |
| ball_tracker_params_sim.yaml    | Ball tracking parameters for simulation     |
| gazebo_params.yaml              | Gazebo simulation parameters                |
| joystick.yaml                   | Joystick configuration                      |
| mapper_params_online_async.yaml | SLAM Toolbox configuration                  |
| nav2_params.yaml                | Nav2 navigation parameters                  |
| twist_mux.yaml                  | Velocity command arbitration                |

---

## Description Directory

Contains robot description files.

| File               | Purpose              |
| ------------------ | -------------------- |
| robot.urdf.xacro   | Main robot model     |
| robot_core.xacro   | Base chassis         |
| lidar.xacro        | LiDAR sensor         |
| camera.xacro       | RGB camera           |
| depth_camera.xacro | Depth sensor         |
| ros2_control.xacro | Controller interface |

---

## Launch Directory

Contains launch files for individual subsystems.

| Launch File            | Purpose                |
| ---------------------- | ---------------------- |
| launch_sim.launch.py   | Full simulation launch |
| launch_robot.launch.py | Physical robot launch  |
| online_async_launch.py | SLAM launch            |
| localization_launch.py | AMCL launch            |
| navigation_launch.py   | Nav2 launch            |
| ball_tracker.launch.py | Vision tracking launch |
| joystick.launch.py     | Teleoperation          |
| rsp.launch.py          | Robot State Publisher  |

---

## Worlds Directory

| World           | Purpose            |
| --------------- | ------------------ |
| empty.world     | Basic validation   |
| obstacles.world | Navigation testing |

---

# Software Dependencies

## Operating System

* Ubuntu 22.04 LTS

## ROS Distribution

* ROS2 Humble Hawksbill

## Major Packages

* Nav2
* SLAM Toolbox
* Gazebo ROS
* robot_state_publisher
* ros2_control
* diff_drive_controller
* joint_state_broadcaster
* twist_mux

---

# Installation

## Create Workspace

```bash
mkdir -p ~/robot_project/src
cd ~/robot_project/src
```

Clone repository:

```bash
git clone https://github.com/<username>/vamr_msd.git
```

---

## Install Dependencies

```bash
cd ~/robot_project

rosdep install \
    --from-paths src \
    --ignore-src \
    -r \
    -y
```

---

## Build Workspace

```bash
colcon build --symlink-install
```

---

## Source Environment

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

# Running the Simulation

## Launch Gazebo

```bash
ros2 launch vamr_msd launch_sim.launch.py
```

---

## Launch SLAM

```bash
ros2 launch vamr_msd online_async_launch.py
```

---

## Launch Localization

```bash
ros2 launch vamr_msd localization_launch.py
```

---

## Launch Navigation

```bash
ros2 launch vamr_msd navigation_launch.py
```

---

# Mapping Workflow

```text
Launch Simulation
        ↓
Launch SLAM
        ↓
Drive Robot
        ↓
Generate Occupancy Map
        ↓
Save Map
```

Save map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/vamr_map
```

---

# Localization Workflow

```text
Load Existing Map
        ↓
Launch AMCL
        ↓
Set Initial Pose
        ↓
Receive Laser Scans
        ↓
Estimate Robot Pose
```

---

# Future Work

## Perception

* YOLOv8 object detection
* Semantic segmentation
* Dynamic obstacle prediction

## Navigation

* Multi-floor navigation
* Dynamic replanning
* Semantic navigation

## Fleet Management

* Multi-robot coordination
* Shared map server
* Distributed task allocation

## Cloud Robotics

* Remote mission upload
* Telemetry dashboard
* Cloud mission scheduling

---

# Research Contributions

Potential publication topics include:

* Vision-assisted mission scheduling
* Hybrid visual and LiDAR localization
* Autonomous docking strategies
* Lightweight mobile robot autonomy stack

---

# License

MIT License

```

---


```
