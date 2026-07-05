## Author

**Aayush Anil Mishra** - 
Mechatronics @ MIT Manipal

## VAMR_MSD: Vision-Assisted Autonomous Mobile Robot with Mission-Level Task Scheduling and Docking in ROS2 Humble

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)]()
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420.svg)]()
[![Gazebo](https://img.shields.io/badge/Gazebo-11-orange.svg)]()
[![License](https://img.shields.io/badge/License-MIT-green.svg)]()


![Video](/img/vamr_msd.mp4)


### Project Overview

VAMR_MSD (Vision-Assisted Autonomous Mobile Robot with Mission-Level Task Scheduling and Docking) is a modular autonomous mobile robotics platform developed in ROS2 Humble for research and educational applications in autonomous navigation, mission execution, and intelligent robot behavior.

The Platform Integrates:

- Vision-assisted Object Tracking
- SLAM-based Environment Mapping
- Localization using Adaptive Monte Carlo Localization (AMCL)
- Autonomous Navigation using Nav2
- Mission-level Multi-goal Task Scheduling
- Autonomous Docking Capability
- Simulation-first Development Workflow using Gazebo

Unlike conventional ROS mobile robot demonstrations which only showcase single-point navigation, VAMR_MSD focuses on complete mission execution pipelines including perception, planning, task scheduling, and energy-aware behavior.

````markdown
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


# Objectives

The Project Aims to Develop a Mobile Robot Capable of:

* Generating Occupancy Maps using SLAM
* Localizing Itself within a Previously Generated Map
* Performing Autonomous Navigation to Target Locations
* Executing Multiple Goals Sequentially
* Tracking Visual Targets using Onboard Vision
* Returning to a Charging Dock Autonomously
* Recording Mission-level Performance Statistics

# Implemented Features

## Core Mobile Robotics Stack

### Differential Drive Mobile Robot

* Custom URDF/Xacro Robot Model
* ROS2 Control Integration
* Differential Drive Controller

### Sensor Suite

* 2D LiDAR
* RGB Camera
* Depth Camera Support
* Wheel Odometry

### Simulation Environment

* Gazebo Classic Simulation
* Empty World Testing Environment
* Obstacle-rich Navigation Environment

## Mapping and Localization

### SLAM Toolbox

Supports Online Asynchronous SLAM for Map Generation.

Features:

* Real-time Occupancy Grid Generation
* Loop Closure Support
* Persistent Map Saving

Launch:

```bash
ros2 launch vamr_msd online_async_launch.py
```

### Adaptive Monte Carlo Localization (AMCL)

Provides Probabilistic Localization using:

* Laser Scan Matching
* Particle Filter Localization
* Map-based Pose Estimation

Launch:

```bash
ros2 launch vamr_msd localization_launch.py
```

## Autonomous Navigation

Navigation is Implemented using Nav2.

Capabilities:

* Global Path Planning
* Local Obstacle Avoidance
* Recovery Behaviors
* Goal Reaching

Launch:

```bash
ros2 launch vamr_msd navigation_launch.py
```

## Vision Assisted Navigation

The Robot uses Camera Input for Target Detection and Tracking.

Current implementation includes:

* Color-Based Object Tracking
* Visual Servoing
* Camera-Guided Motion Commands

Launch:

```bash
ros2 launch vamr_msd ball_tracker.launch.py
```

Future upgrades include:

* YOLO Object Detection
* Semantic Navigation
* Dynamic Object Following

## Mission-Level Goal Queue System

Traditional Nav2 Navigation Executes only 1 Goal at a Time.

VAMR_MSD introduces:

* FIFO Mission Queue
* Multiple Waypoint Execution
* Goal Retry Mechanism
* Pause and Resume Support
* Mission Cancellation
* YAML Mission File Support

Example Mission:

```yaml
goals:
  - x: 1.0
    y: 2.0
    yaw: 0.0

  - x: 3.5
    y: 1.2
    yaw: 1.57
```

## Autonomous Docking

The Robot can Automatically Return to a Docking Station after Mission Completion.

Planned Docking Workflow:

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

## Mission Statistics

Mission Execution Metrics Include:

* Total Mission Duration
* Distance Travelled
* Average Navigation Speed
* Number of Goals Completed
* Number of Retries
* Docking Success Rate

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

## Configuration Directory

Contains Parameter Files for All Subsystems.

| File                            | Purpose                                     |
| ------------------------------- | ------------------------------------------- |
| ball_tracker_params_robot.yaml  | Ball Tracking Parameters for Physical Robot |
| ball_tracker_params_sim.yaml    | Ball Tracking Parameters for Simulation     |
| gazebo_params.yaml              | Gazebo Simulation Parameters                |
| joystick.yaml                   | Joystick Configuration                      |
| mapper_params_online_async.yaml | SLAM Toolbox Configuration                  |
| nav2_params.yaml                | Nav2 Navigation Parameters                  |
| twist_mux.yaml                  | Velocity Command Arbitration                |

## Description Directory

Contains Robot Description Files.

| File               | Purpose              |
| ------------------ | -------------------- |
| robot.urdf.xacro   | Main Robot Model     |
| robot_core.xacro   | Base Chassis         |
| lidar.xacro        | LiDAR Sensor         |
| camera.xacro       | RGB Camera           |
| depth_camera.xacro | Depth Sensor         |
| ros2_control.xacro | Controller Interface |

## Launch Directory

Contains launch files for individual subsystems.

| Launch File            | Purpose                |
| ---------------------- | ---------------------- |
| launch_sim.launch.py   | Full Simulation Launch |
| launch_robot.launch.py | Physical Robot Launch  |
| online_async_launch.py | SLAM Launch            |
| localization_launch.py | AMCL Launch            |
| navigation_launch.py   | Nav2 Launch            |
| ball_tracker.launch.py | Vision Tracking Launch |
| joystick.launch.py     | Teleoperation          |
| rsp.launch.py          | Robot State Publisher  |

## Worlds Directory

| World           | Purpose            |
| --------------- | ------------------ |
| empty.world     | Basic Validation   |
| obstacles.world | Navigation Testing |

# Software Dependencies

### Operating System

* Ubuntu 22.04 LTS

### ROS Distribution

* ROS2 Humble Hawksbill

### Major Packages

* Nav2
* SLAM Toolbox
* Gazebo ROS
* robot_state_publisher
* ros2_control
* diff_drive_controller
* joint_state_broadcaster
* twist_mux

# Installation

### Create Workspace

```bash
mkdir -p ~/robot_project/src
cd ~/robot_project/src
```

Clone Repository:

```bash
git clone https://github.com/<username>/vamr_msd.git
```

### Install Dependencies

```bash
cd ~/robot_project

rosdep install \
    --from-paths src \
    --ignore-src \
    -r \
    -y
```

### Build Workspace

```bash
colcon build --symlink-install
```

### Source Environment

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

# Running the Simulation

### Launch Gazebo

```bash
ros2 launch vamr_msd launch_sim.launch.py
```

### Launch SLAM

```bash
ros2 launch vamr_msd online_async_launch.py
```

### Launch Localization

```bash
ros2 launch vamr_msd localization_launch.py
```

### Launch Navigation

```bash
ros2 launch vamr_msd navigation_launch.py
```

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

Save Map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/vamr_map
```

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

# Future Work

### Perception

* YOLOv8 Object Detection
* Semantic Segmentation
* Dynamic Obstacle Prediction

### Navigation

* Multi-Floor Navigation
* Dynamic Replanning
* Semantic Navigation

### Fleet Management

* Multi-Robot Coordination
* Shared Map Server
* Distributed Task Allocation

### Cloud Robotics

* Remote Mission Upload
* Telemetry Dashboard
* Cloud Mission Scheduling

# Research Contributions

Potential Publication Topics Include:

* Vision-Assisted Mission Scheduling
* Hybrid Visual and LiDAR Localization
* Autonomous Docking Strategies
* Lightweight Mobile Robot Autonomy Stack

# License

MIT License
