# Robot Arm Implementation

## Overview
The `RobotArm` class provides a flexible implementation of a robotic arm for PyBullet simulations, supporting various configurations and gripper types.

## Class: `RobotArm`

### Initialization
```python
__init__(self, physics_client_id, position: List[float], 
         gripper_type: str = "parallel", name: str = "robot")
```

**Parameters**:
- `physics_client_id`: PyBullet physics client ID
- `position`: [x, y, z] base position of the robot
- `gripper_type`: Type of gripper (default: "parallel")
- `name`: Identifier for the robot instance

### Core Methods

#### `move_to_pose(target_position, target_orientation=None)`
Moves the end effector to the specified position and orientation.

**Parameters**:
- `target_position`: Target [x, y, z] position
- `target_orientation`: Optional quaternion [x, y, z, w]

**Returns**:
- `bool`: True if movement was successful

#### `get_joint_positions()`
Gets current joint angles.

**Returns**:
- `List[float]`: Current joint positions in radians

#### `set_joint_positions(positions)`
Sets target joint positions.

**Parameters**:
- `positions`: List of target joint angles

#### `attach_gripper(gripper)`
Attaches a gripper to the robot's end effector.

**Parameters**:
- `gripper`: Gripper instance to attach

### Gripper Control
- `open_gripper()`: Opens the attached gripper
- `close_gripper()`: Closes the attached gripper
- `gripper_action(action)`: Performs a gripper action

### Kinematics
- `inverse_kinematics(target_position, target_orientation)`: Solves IK for target pose
- `forward_kinematics(joint_angles)`: Computes end effector pose from joint angles

### Properties
- `joint_limits`: Get joint position limits
- `end_effector_pose`: Current end effector [position, orientation]
- `is_gripper_attached`: Check if gripper is attached

## Usage Example
```python
import pybullet as p
from robot_arm import RobotArm

# Initialize simulation
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

# Create robot
robot = RobotArm(p, position=[0, 0, 0])

# Move to position
robot.move_to_pose([0.5, 0, 0.5])

# Close gripper
robot.close_gripper()
```

## Implementation Notes
- Uses PyBullet for physics simulation
- Supports both position and velocity control
- Includes collision detection
- Thread-safe for multi-robot simulations
