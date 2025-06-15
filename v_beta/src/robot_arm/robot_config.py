"""Configuration for industrial robots and their grippers."""
from typing import Dict, Any, List, Tuple
import pybullet as p

# Robot configurations
# NOTE: UR5 and other advanced robot models will be supported in the future. For now, all robots use the built-in KUKA iiwa model from pybullet_data.
ROBOT_CONFIGS: Dict[str, Dict[str, Any]] = {
    "KUKA_iiwa_1": {
        "base_position": [-1.5, 0, 0],
        "urdf_path": "kuka_iiwa/model.urdf",  # From pybullet_data
        "gripper_type": "kuka_default",
        "gripper_urdf": None,
        "ee_link_name": "lbr_iiwa_link_7",
        "gripper_mount_offset": [0, 0, 0],
        "color": [0.8, 0.2, 0.2, 1.0],  # Red
        "label": "KUKA-1"
    },
    "KUKA_iiwa_2": {
        "base_position": [0, 0, 0],
        "urdf_path": "kuka_iiwa/model.urdf",  # From pybullet_data
        "gripper_type": "kuka_default",
        "gripper_urdf": None,
        "ee_link_name": "lbr_iiwa_link_7",
        "gripper_mount_offset": [0, 0, 0],
        "color": [0.2, 0.8, 0.2, 1.0],  # Green
        "label": "KUKA-2"
    },
    "KUKA_iiwa_3": {
        "base_position": [1.5, 0, 0],
        "urdf_path": "kuka_iiwa/model.urdf",  # From pybullet_data
        "gripper_type": "kuka_default",
        "gripper_urdf": None,
        "ee_link_name": "lbr_iiwa_link_7",
        "gripper_mount_offset": [0, 0, 0],
        "color": [0.2, 0.2, 0.8, 1.0],  # Blue
        "label": "KUKA-3"
    }
}

def get_robot_configs() -> Dict[str, Dict[str, Any]]:
    """Return the robot configurations."""
    return ROBOT_CONFIGS

def setup_robot_visuals(robot_id: int, color: List[float]):
    """Apply visual properties to the robot."""
    # Set base color
    p.changeVisualShape(robot_id, -1, rgbaColor=color)
    
    # Set link colors (slightly lighter than base)
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        link_color = [min(1.0, c * 1.2) for c in color[:3]] + [1.0]  # Lighter color
        p.changeVisualShape(robot_id, i, rgbaColor=link_color)

def add_robot_label(position: List[float], label: str, color: List[float]):
    """Add a text label above the robot."""
    # This is a placeholder - in a real implementation, you might use:
    # 1. PyBullet's debug text rendering
    # 2. A 3D text object
    # 3. A billboard with text texture
    # For now, we'll just print the label
    print(f"Adding label '{label}' at position {position}")
    # In a real implementation, you would use addUserDebugText or similar
