"""Gripper implementations for robot end effectors."""
from typing import Dict, Type, Any, List, Optional

from .base_gripper import BaseGripper
from .parallel_gripper import ParallelGripper

# Registry of available gripper types
GRIPPER_REGISTRY: Dict[str, Type[BaseGripper]] = {
    'parallel': ParallelGripper,
    # Add other gripper types here as they are implemented
}

def create_gripper(
    gripper_type: str,
    physics_client: Any,
    position: List[float],
    orientation: Optional[List[float]] = None,
    name: str = "gripper",
    **kwargs
) -> BaseGripper:
    """Create a gripper of the specified type.
    
    Args:
        gripper_type: Type of gripper to create (e.g., 'parallel')
        physics_client: PyBullet physics client
        position: [x, y, z] position of the gripper
        orientation: [x, y, z, w] quaternion orientation of the gripper
        name: Name of the gripper for identification
        **kwargs: Additional arguments specific to the gripper type
        
    Returns:
        An instance of the specified gripper type
        
    Raises:
        ValueError: If the specified gripper type is not found
    """
    gripper_cls = GRIPPER_REGISTRY.get(gripper_type.lower())
    if gripper_cls is None:
        available = ", ".join(f"'{t}'" for t in GRIPPER_REGISTRY.keys())
        raise ValueError(
            f"Unknown gripper type: '{gripper_type}'. "
            f"Available types are: {available}"
        )
        
    return gripper_cls(
        physics_client=physics_client,
        position=position,
        orientation=orientation,
        name=name,
        **kwargs
    )

__all__ = [
    'BaseGripper',
    'ParallelGripper',
    'create_gripper',
]