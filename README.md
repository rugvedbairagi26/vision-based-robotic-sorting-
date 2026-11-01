# Vision-Based Robotic Sorting System

## Overview

The **Vision-Based Robotic Sorting System** is a robotics project that combines computer vision, robotic manipulation, and motion planning to develop a color-based object sorting system.

The project focuses on detecting colored cubes using a YOLO-based object detection model and developing a robotic arm system capable of performing pick-and-place operations.

The system is divided into three major components:

- Robotic Simulation
- Computer Vision and Perception
- Real-World Robotic Implementation

The main objective is to identify objects based on their colors and use the detected information to perform sorting operations.

## Project Objectives

- Develop a 6-DOF robotic arm model for simulation and manipulation.
- Implement robotic motion planning using ROS 2 and MoveIt 2.
- Develop a computer vision system for detecting red, green, and yellow cubes.
- Train and evaluate a YOLO object detection model.
- Implement and test pick-and-place operations in simulation.
- Work toward integrating visual perception with the physical robotic arm.

## System Architecture

### 1. Simulation

Uses ROS 2, MoveIt 2, and RViz to model the robotic arm, plan its movements, and test pick-and-place operations.

### 2. Perception

Uses YOLO-based object detection to identify and classify red, green, and yellow cubes from images.

### 3. Real-World Robot

Focuses on the physical robotic arm, servo control, camera setup, and eventual integration of the perception and manipulation systems.

## Repository Structure

```text
vision-based-robotic-sorting-/
│
├── README.md
├── LICENSE
├── .gitignore
│
├── simulation/
│   ├── README.md
│   ├── robot_arm_description/
│   ├── robot_arm_moveit_config/
│   ├── robot_arm_practice/
│   ├── scripts/
│   └── videos/
│
├── perception/
│   ├── README.md
│   ├── dataset/
│   ├── notebooks/
│   ├── src/
│   ├── weights/
│   └── results/
│
└── real_world_robot/
    └── README.md
```

## Technologies Used

| Category | Technologies |
|---|---|
| Operating System | Ubuntu 24.04 |
| Robotics Middleware | ROS 2 Jazzy |
| Motion Planning | MoveIt 2, OMPL |
| Visualization | RViz |
| Programming | Python |
| Robot Description | URDF, Xacro |
| Computer Vision | OpenCV |
| Object Detection | YOLOv8, Ultralytics |
| Numerical Computing | NumPy |
| Model Training | Google Colab |
| Hardware Control | ESP32, PCA9685 |

## Current Progress

- Physical robotic arm assembly.
- Basic servo testing and control.
- Robotic arm description using URDF/Xacro.
- MoveIt 2 configuration and motion planning.
- Collision object integration in the planning scene.
- Simulated pick-and-place operations for colored cubes.
- Custom colored cube dataset preparation and annotation.
- YOLO model training and evaluation.

## Future Work

- Integrate live camera-based object detection with the robotic arm.
- Convert detected object positions into robot coordinate frames.
- Implement visual feedback for object localization.
- Connect perception outputs with robotic motion planning.
- Perform complete physical pick-and-place and color-based sorting.
- Improve system accuracy and reliability under different lighting conditions.

## Project Status

The simulation and perception components have been developed independently. The next major step is integrating these components with the physical robotic arm to achieve an end-to-end autonomous sorting workflow.

## Author

**Computer Engineering Student**

Robotics | Computer Vision | Artificial Intelligence

