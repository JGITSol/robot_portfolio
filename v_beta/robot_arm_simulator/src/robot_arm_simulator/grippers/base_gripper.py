"""Base gripper class for robot end effectors."""
from typing import List, Optional, Dict, Any, Tuple
import numpy as np
import pybullet as p

class BaseGripper:
    """Base class for all robot grippers."""
    
    def __init__(self, 
                 physics_client: Any,
                 position: List[float],
                 orientation: Optional[List[float]] = None,
                 name: str = "gripper",
                 **kwargs):
        """Initialize the base gripper.
        
        Args:
            physics_client: PyBullet physics client
            position: [x, y, z] position of the gripper
            orientation: [x, y, z, w] quaternion orientation of the gripper
            name: Name of the gripper for identification
        """
        self.physics_client = physics_client
        self.position = np.array(position, dtype=np.float32)
        self.orientation = orientation or [0, 0, 0, 1]
        self.name = name
        self.gripper_id = None
        self.attached_object = None
        self.robot_id = None
        self.ee_link_index = -1
        self.is_active = False
        
    def load(self) -> int:
        """Load the gripper model into the simulation.
        
        Returns:
            int: The ID of the loaded gripper in the simulation
        """
        raise NotImplementedError("Subclasses must implement load()")
        
    def attach_to_robot(self, robot_id: int, ee_link_index: int):
        """Attach the gripper to a robot's end effector.
        
        Args:
            robot_id: The ID of the robot in the simulation
            ee_link_index: The link index of the robot's end effector
        """
        self.robot_id = robot_id
        self.ee_link_index = ee_link_index
        
        if self.gripper_id is None:
            self.load()
            
        # Create a fixed constraint between the robot's end effector and the gripper
        self.constraint_id = self.physics_client.createConstraint(
            parentBodyUniqueId=robot_id,
            parentLinkIndex=ee_link_index,
            childBodyUniqueId=self.gripper_id,
            childLinkIndex=-1,  # Base of the gripper
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0],
            parentFrameOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
    
    def detach(self):
        """Detach the gripper from the robot."""
        if self.constraint_id is not None:
            self.physics_client.removeConstraint(self.constraint_id)
            self.constraint_id = None
        self.robot_id = None
        self.ee_link_index = -1
    
    def activate(self, activate: bool = True):
        """Activate or deactivate the gripper.
        
        Args:
            activate: Whether to activate (True) or deactivate (False) the gripper
        """
        self.is_active = activate
    
    def open(self):
        """Open the gripper (alias for deactivate)."""
        self.activate(False)
    
    def close(self):
        """Close the gripper (alias for activate)."""
        self.activate(True)
    
    def attach_object(self, object_id: int):
        """Attach an object to the gripper.
        
        Args:
            object_id: The ID of the object to attach in the simulation
        """
        if self.attached_object is not None:
            self.detach_object()
            
        self.attached_object = object_id
        
        # Create a fixed constraint between the gripper and the object
        self.object_constraint_id = self.physics_client.createConstraint(
            parentBodyUniqueId=self.gripper_id,
            parentLinkIndex=-1,  # Base of the gripper
            childBodyUniqueId=object_id,
            childLinkIndex=-1,  # Base of the object
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=self.physics_client.getBasePositionAndOrientation(object_id)[0],
            parentFrameOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
    
    def detach_object(self):
        """Detach any attached object."""
        if self.attached_object is not None and hasattr(self, 'object_constraint_id'):
            self.physics_client.removeConstraint(self.object_constraint_id)
            del self.object_constraint_id
            
            # Apply a small impulse to help separate the object
            obj_pos, obj_orn = self.physics_client.getBasePositionAndOrientation(self.attached_object)
            self.physics_client.applyExternalForce(
                objectUniqueId=self.attached_object,
                linkIndex=-1,
                forceObj=[0, 0, 0.1],  # Small upward force
                posObj=obj_pos,
                flags=p.WORLD_FRAME
            )
            
        self.attached_object = None
    
    def reset(self):
        """Reset the gripper to its initial state."""
        self.is_active = False
        if self.attached_object is not None:
            self.detach_object()
    
    def update(self):
        """Update the gripper state (to be called in the simulation loop)."""
        pass
    
    def get_state(self) -> Dict[str, Any]:
        """Get the current state of the gripper.
        
        Returns:
            Dict containing the gripper's state
        """
        return {
            'name': self.name,
            'position': self.position.tolist() if hasattr(self.position, 'tolist') else self.position,
            'orientation': self.orientation,
            'is_active': self.is_active,
            'has_object': self.attached_object is not None,
            'object_id': self.attached_object
        }
