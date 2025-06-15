"""Gripper implementations for robotic arms."""
import pybullet as p
import pybullet_data
from typing import List, Optional
import numpy as np

class Gripper:
    """Base gripper class."""
    def __init__(self, physics_client, position: List[float], 
                 orientation: Optional[List[float]] = None):
        self.physics_client = physics_client
        self.position = list(position)
        self.orientation = orientation or [0, 0, 0, 1]
        self.gripper_id = None
        self.joint_indices = []
        self.joint_limits = []
        self.attached_object = None
        
    def load(self) -> int:
        """Load the gripper model and return its ID."""
        raise NotImplementedError("Subclasses must implement load()")
        
    def open(self):
        """Open the gripper."""
        pass
        
    def close(self):
        """Close the gripper."""
        pass
        
    def attach_to_robot(self, robot_id: int, link_index: int):
        """Attach gripper to a robot's end effector."""
        if self.gripper_id is None:
            self.load()
            
        self.constraint_id = self.physics_client.createConstraint(
            parentBodyUniqueId=robot_id,
            parentLinkIndex=link_index,
            childBodyUniqueId=self.gripper_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0],
            parentFrameOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
        return self.gripper_id


class ParallelGripper(Gripper):
    """Parallel jaw gripper, good for boxes and cylinders."""
    def load(self) -> int:
        gripper_urdf = "gripper/wsg50_one_motor_gripper_new_free_base.urdf"
        self.gripper_id = self.physics_client.loadURDF(
            gripper_urdf,
            basePosition=self.position,
            baseOrientation=self.orientation,
            useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION
        )
        # Get joint information
        self.joint_indices = [i for i in range(self.physics_client.getNumJoints(self.gripper_id))
                            if self.physics_client.getJointInfo(self.gripper_id, i)[2] == p.JOINT_PRISMATIC]
        return self.gripper_id
        
    def open(self):
        for joint_index in self.joint_indices:
            self.physics_client.setJointMotorControl2(
                self.gripper_id,
                joint_index,
                p.POSITION_CONTROL,
                targetPosition=0.04,  # Open position
                force=100
            )
            
    def close(self):
        for joint_index in self.joint_indices:
            self.physics_client.setJointMotorControl2(
                self.gripper_id,
                joint_index,
                p.POSITION_CONTROL,
                targetPosition=0.0,  # Closed position
                force=100
            )


class SuctionGripper(Gripper):
    """Suction cup gripper, good for flat surfaces."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.is_active = False
    
    def load(self) -> int:
        # Create a simple suction cup
        visual_shape_id = self.physics_client.createVisualShape(
            shapeType=p.GEOM_CYLINDER,
            radius=0.05,
            length=0.02,
            rgbaColor=[0.3, 0.3, 0.9, 0.7]
        )
        collision_shape_id = self.physics_client.createCollisionShape(
            shapeType=p.GEOM_CYLINDER,
            radius=0.05,
            height=0.02
        )
        
        self.gripper_id = self.physics_client.createMultiBody(
            baseMass=0.1,
            baseCollisionShapeIndex=collision_shape_id,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=self.position,
            baseOrientation=self.orientation
        )
        return self.gripper_id
        
    def activate(self, activate: bool = True):
        """Activate or deactivate suction."""
        self.is_active = activate
        if activate:
            p.changeVisualShape(self.gripper_id, -1, rgbaColor=[0.8, 0.8, 1.0, 0.9])
            # In a real implementation, create constraints here
        else:
            p.changeVisualShape(self.gripper_id, -1, rgbaColor=[0.3, 0.3, 0.9, 0.7])
            # In a real implementation, remove constraints here
    
    def open(self):
        """Alias for deactivate."""
        self.activate(False)
    
    def close(self):
        """Alias for activate."""
        self.activate(True)


def create_gripper(gripper_type: str, *args, **kwargs) -> Gripper:
    """Factory function to create grippers by type."""
    grippers = {
        'parallel': ParallelGripper,
        'suction': SuctionGripper,
    }
    gripper_class = grippers.get(gripper_type.lower())
    if gripper_class is None:
        raise ValueError(f"Unknown gripper type: {gripper_type}")
    return gripper_class(*args, **kwargs)
