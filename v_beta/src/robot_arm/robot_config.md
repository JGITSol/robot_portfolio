# Robot Configuration

## Overview
This module provides configuration and visualization utilities for robot arms in the simulation. It defines robot presets and helper functions for visual customization.

## Configuration Structure

### Robot Presets
Predefined robot configurations stored in `ROBOT_CONFIGS` dictionary with the following structure:

```python
{
    "robot_name": {
        "base_position": [x, y, z],
        "urdf_path": "path/to/model.urdf",
        "gripper_type": "gripper_id",
        "gripper_urdf": "path/to/gripper.urdf",
        "ee_link_name": "end_effector_link",
        "gripper_mount_offset": [x, y, z],
        "color": [r, g, b, a],
        "label": "Display Name"
    }
}
```

### Default Configurations

#### KUKA iiwa (3 instances)
- **Base Model**: KUKA LBR iiwa
- **Positions**: Spaced 1.5m apart on X-axis
- **Colors**: Red, Green, Blue
- **Labels**: KUKA-1, KUKA-2, KUKA-3

## API Reference

### `get_robot_configs() -> Dict[str, Dict[str, Any]]`
Returns the complete dictionary of robot configurations.

### `setup_robot_visuals(robot_id: int, color: List[float])`
Applies visual properties to a robot.
- `robot_id`: PyBullet body ID
- `color`: RGBA color values [0-1]

### `add_robot_label(position: List[float], label: str, color: List[float])`
Adds a text label above a robot (stub implementation).
- `position`: [x, y, z] world coordinates
- `label`: Text to display
- `color`: RGBA color values [0-1]

## Usage Example

```python
import pybullet as p
from robot_arm.robot_config import get_robot_configs, setup_robot_visuals

# Get configurations
configs = get_robot_configs()

# Setup a robot with visual customization
robot_id = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[0, 0, 0])
setup_robot_visuals(robot_id, color=[1, 0, 0, 1])  # Red robot
```

## Visual Customization
- Base and links are colored differently for better visibility
- Link colors are automatically lightened for contrast
- Supports alpha channel for transparency

## Notes
- Currently uses KUKA iiwa model from pybullet_data
- UR5 and other models can be added to configurations
- Default gripper is KUKA's built-in end effector
- Labels are currently printed to console (extend for 3D text)
