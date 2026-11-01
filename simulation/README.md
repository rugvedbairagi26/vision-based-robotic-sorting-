# Robotic Arm Simulation and Motion Planning

## Overview

This module focuses on developing and simulating a robotic arm for pick-and-place operations using ROS 2 and MoveIt 2.

The robotic arm is modeled using URDF/Xacro and visualized in RViz. MoveIt 2 is used for motion planning, inverse kinematics configuration, and collision-aware trajectory generation.

The primary objective is to develop a simulated robotic manipulation environment where the arm can interact with objects and perform color-based sorting demonstrations.

## Technologies Used

- Ubuntu 24.04
- ROS 2 Jazzy
- MoveIt 2
- RViz
- Python
- URDF / Xacro
- OMPL
- KDL
- ros2_control

## Repository Structure

```text
simulation/
│
├── README.md
│
├── robot_arm_description/
│   ├── urdf/
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── LICENSE
│
├── robot_arm_moveit_config/
│   ├── config/
│   ├── launch/
│   ├── CMakeLists.txt
│   └── package.xml
│
├── robot_arm_practice/
│   ├── config/
│   ├── scripts/
│   ├── CMakeLists.txt
│   └── package.xml
│
├── scripts/
│   └── restore_sorting_scene.py
│
└── videos/
```

## 1. Robot Description

The robotic arm is modeled using URDF and Xacro files.

The model consists of five revolute joints for arm positioning and a prismatic joint representing the gripper in the ROS model.

The robot description defines:

- Robot links and joints.
- Joint limits.
- Visual and collision geometries.
- Robot kinematic structure.
- Gripper configuration.

## 2. MoveIt 2 Configuration

MoveIt 2 is used to configure the robotic arm for motion planning and manipulation.

The configuration includes:

- Semantic robot description using SRDF.
- Joint limits.
- Kinematics configuration.
- Motion planning configuration.
- Controller configuration.
- RViz visualization settings.

OMPL is used as the motion planning framework, while KDL is configured for kinematics calculations.

## 3. Motion Planning and Simulation

The robotic arm is configured to perform joint-space motion planning and simulated trajectory execution.

The MoveIt planning scene represents the robot's working environment and includes collision objects such as:

- Red cube
- Green cube
- Yellow cube
- Virtual table
- Other objects used during testing

Collision objects are added to the planning scene to help the motion planner account for obstacles during trajectory generation.

## 4. Pick-and-Place Operations

Separate Python scripts are used to demonstrate simulated pick-and-place operations for colored cubes.

The demonstrations include:

- Red cube pick-and-place.
- Green cube pick-and-place.
- Yellow cube pick-and-place.

These scripts are used to test robot movement and object-specific sorting sequences in the simulated environment.

## 5. Running the Simulation

### Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy
- MoveIt 2
- Required workspace packages

### Launch MoveIt 2

```bash
cd ~/master_ros2_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch robot_arm_moveit_config demo.launch.py
```

### Restore the Sorting Scene

In a separate terminal:

```bash
cd ~/master_ros2_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 ~/master_ros2_ws/scripts/restore_sorting_scene.py
```

The robot and planning scene can then be inspected in RViz.

## 6. Demonstrations

The `videos/` directory contains recordings of simulated robotic arm movements and colored cube pick-and-place operations.

These demonstrations show the motion planning and simulated sorting workflow.

## 7. Future Improvements

- Implement and verify Cartesian path planning.
- Improve grasp planning and object manipulation.
- Integrate camera-based object localization.
- Implement camera-to-robot coordinate transformations.
- Connect the perception module with MoveIt 2.
- Transfer the simulated manipulation workflow to the physical robotic arm.

## Project Status

The robotic arm description, MoveIt 2 configuration, planning scene, collision object integration, and simulated pick-and-place demonstrations have been developed.

The next stage is to integrate the perception system with the robotic manipulation pipeline and validate the complete workflow on the physical robotic arm.


