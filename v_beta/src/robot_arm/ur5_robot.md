# UR5 Robot Implementation

## Overview
The `UR5Robot` class implements a UR5 robotic arm with configurable grippers using PyBullet for simulation.

## Class: `UR5Robot`

### Initialization
```python
__init__(self, 
         physics_client: Any,
         base_position: List[float],
         urdf_path: str,
         gripper_type: str,
         gripper_urdf: str,
         ee_link_name: str,
         gripper_mount_offset: List[float] = None,
         color: List[float] = None,
         label: str = "UR5",
         physics_client_id: int = 0)
```

### Key Features
- **URDF Loading**: Supports custom URDF models
- **Gripper Integration**: Configurable gripper attachment
- **Joint Control**: Precise control over revolute and prismatic joints
- **Collision Handling**: Built-in self-collision detection

### Core Methods

#### `load_robot()`
Loads the URDF model and initializes joint control.

#### `attach_gripper(gripper_urdf: str)`
Attaches a gripper to the end effector.

#### `move_to_joint_positions(positions: List[float])`
Moves robot to specified joint positions.

#### `get_end_effector_pose()`
Returns current end effector [position, orientation].

#### `inverse_kinematics(target_position, target_orientation=None)`
Solves IK for target end effector pose.

### Configuration
- **Joint Limits**: Automatically extracted from URDF
- **Control Modes**: Position and velocity control
- **Visualization**: Customizable colors and labels

## Usage Example
```python
import pybullet as p

# Initialize simulation
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

# Create UR5 instance
robot = UR5Robot(
    physics_client=p,
    base_position=[0, 0, 0],
    urdf_path="ur5/ur5.urdf",
    gripper_type="parallel",
    gripper_urdf="gripper/parallel.urdf",
    ee_link_name="wrist_3_link"
)

# Move to target position
robot.move_to_joint_positions([0, -1.57, 0, -1.57, 0, 0])
```

## Dependencies
- PyBullet
- NumPy
- Standard Python libraries

## Implementation Notes
- Uses URDF for robot description
- Supports custom gripper models
- Thread-safe for multi-robot simulations
- Includes error handling for model loading
