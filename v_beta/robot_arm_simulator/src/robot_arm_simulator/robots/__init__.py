"""Robot arm implementations and factory functions."""
from typing import Dict, Type, Any, List, Optional

from .base_robot import BaseRobot
from .ur5_robot import UR5Robot

# Registry of available robot types
ROBOT_REGISTRY: Dict[str, Type[BaseRobot]] = {
    'ur5': UR5Robot,
    # Add other robot types here as they are implemented
}

def create_robot(
    robot_type: str,
    physics_client: Any,
    base_position: List[float],
    base_orientation: Optional[List[float]] = None,
    name: str = "robot",
    **kwargs
) -> BaseRobot:
    """Create a robot of the specified type.
    
    Args:
        robot_type: Type of robot to create (e.g., 'ur5')
        physics_client: PyBullet physics client
        base_position: [x, y, z] position of the robot base
        base_orientation: [x, y, z, w] quaternion orientation of the base
        name: Name of the robot for identification
        **kwargs: Additional arguments specific to the robot type
        
    Returns:
        An instance of the specified robot type
        
    Raises:
        ValueError: If the specified robot type is not found
    """
    robot_cls = ROBOT_REGISTRY.get(robot_type.lower())
    if robot_cls is None:
        available = ", ".join(f"'{t}'" for t in ROBOT_REGISTRY.keys())
        raise ValueError(
            f"Unknown robot type: '{robot_type}'. "
            f"Available types are: {available}"
        )
        
    return robot_cls(
        physics_client=physics_client,
        base_position=base_position,
        base_orientation=base_orientation,
        name=name,
        **kwargs
    )

__all__ = [
    'BaseRobot',
    'UR5Robot',
    'create_robot',
]