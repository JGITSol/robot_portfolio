# Robot Arm Module

## Overview
This module provides a robotic arm simulation framework using PyBullet. It includes implementations for different robot arms, grippers, and production line simulations.

## Core Components

### RobotArm Class
- **Location**: `robot.py`
- **Purpose**: Base class for robotic arm simulation
- **Features**:
  - Multi-link robot arm with configurable parameters
  - Physics-based simulation
  - Collision detection
  - Support for different gripper types

### Grippers
- **Location**: `grippers.py`
- **Types**:
  1. `ParallelGripper`: Two-finger gripper for boxes and cylinders
  2. `SuctionGripper`: For flat surfaces
- **Base Class**: `Gripper` provides common interface

### ProductionLine
- **Location**: `production_line.py`
- **Purpose**: Simulates a production line environment

## Demos
- `demo.py`: Basic robot arm demonstration
- `demo_conveyor.py`: Conveyor belt simulation
- `demo_pendant.py`: Virtual pendant interface
- `download_models.py`: Utility for downloading 3D models

## Usage
```python
from robot_arm import RobotArm, create_gripper, ProductionLine

# Create robot with parallel gripper
robot = RobotArm(physics_client, position=[0, 0, 0])
gripper = create_gripper("parallel", physics_client, [0, 0, 0])
robot.attach_gripper(gripper)

# Control the robot
robot.move_to_pose([0.5, 0, 0.5])
gripper.close()
```

## Dependencies
- PyBullet
- NumPy
- Standard Python libraries

## Examples
See the `examples/` directory for complete usage examples.
