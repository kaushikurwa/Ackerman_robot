# Ackerman Robot

A ROS2-based Ackerman steering robot implementation for simulation, perception, and navigation.

---

## Overview

This project implements an Ackerman steering model used in autonomous vehicles, enabling realistic car-like motion control and navigation. It integrates perception and control to simulate a complete robotics pipeline.

---

## Objective

Design and implement a complete ROS 2 perception-to-action pipeline. The system reconstructs a partially defined simulation architecture, applies computer vision to identify a target, and controls an Ackerman-steered robot to navigate toward it autonomously.

---

## Features

* Ackerman steering kinematics
* ROS2-based modular architecture
* Perception-to-control pipeline
* Simulation-ready setup
* Target detection using computer vision

---

## System Pipeline

1. **Perception**

   * Capture sensor/camera data
   * Detect and localize the target using computer vision

2. **Planning**

   * Compute path or direction toward the target

3. **Control**

   * Generate steering and velocity commands
   * Apply Ackerman kinematics for motion

---

## Project Structure

```
Ackerman_robot/
 ├── src/
    ├── my_robot_description/
      ├── launch/
        display.launch.xml
      ├── urdf/
          my_robot.urdf.xacro
          common_properties.xacro      
          ackermann_base.xacro         
          ackermann_gazebo.xacro
    ├── my_robot_bringup/       
      ├── launch/
          gazebo.launch.py    
      ├── config/
          bridge.yaml
      ├── worlds/
          shapes.sdf
      ├── vision_node/
          vision_node.py       
          control_node.py      
```

---

## How to Run

### 1. Launch Simulation

```bash
cd ~/Ackerman_robot
source install/setup.bash
ros2 launch my_robot_bringup gazebo.launch.py
```

### 2. Start Vision Node

```bash
ros2 run vision_node vision_node
```

### 3. Start Control Node

```bash
ros2 run vision_node control_node
```
```

---

## Future Improvements

* Path planning algorithms (A*, RRT)
* SLAM integration
* Real-world deployment on hardware
* Sensor fusion (LiDAR + Camera)

---

## Applications

* Autonomous vehicles
* Mobile robotics research
* Simulation-based testing
* Robotics education and prototyping

---
