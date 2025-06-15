# Grippers Module

## Overview
Implements various gripper types for robotic arms in the simulation.

## Base Class
### `Gripper`
- **Purpose**: Abstract base class for all gripper implementations
- **Location**: `grippers.py`
- **Key Methods**:
  - `load()`: Loads the gripper model
  - `open()`: Opens the gripper
  - `close()`: Closes the gripper
  - `attach_to_robot(robot_id, link_index)`: Attaches gripper to a robot

## Implementations

### 1. `ParallelGripper`
- **Description**: Two-finger parallel jaw gripper
- **Best For**: Boxes and cylindrical objects
- **Methods**:
  - `load()`: Loads the parallel gripper URDF
  - `open()`: Opens the gripper jaws
  - `close()`: Closes the gripper jaws

### 2. `SuctionGripper`
- **Description**: Suction cup gripper
- **Best For**: Flat surfaces
- **Methods**:
  - `load()`: Creates a suction cup model
  - `activate()`: Activates suction
  - `deactivate()`: Deactivates suction

## Factory Function
```python
def create_gripper(gripper_type: str, *args, **kwargs) -> Gripper:
    """Creates a gripper of the specified type.
    
    Args:
        gripper_type: Type of gripper ('parallel' or 'suction')
        *args, **kwargs: Passed to gripper constructor
        
    Returns:
        Configured Gripper instance
    """
```

## Usage Example
```python
from robot_arm.grippers import create_gripper

# Create a parallel gripper
gripper = create_gripper('parallel', physics_client, [0, 0, 0])
gripper.load()
gripper.attach_to_robot(robot_id, -1)  # -1 for base link

gripper.close()  # Close gripper
gripper.open()   # Open gripper
```
