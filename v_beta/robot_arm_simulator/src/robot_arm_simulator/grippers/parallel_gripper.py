"""Parallel jaw gripper implementation."""
from typing import List, Optional, Dict, Any, Tuple
import numpy as np
import pybullet as p
from .base_gripper import BaseGripper

class ParallelGripper(BaseGripper):
    """Parallel jaw gripper with two fingers."""
    
    def __init__(self, 
                 physics_client: Any,
                 position: List[float],
                 orientation: Optional[List[float]] = None,
                 name: str = "parallel_gripper",
                 max_force: float = 10.0,
                 finger_width: float = 0.02,
                 finger_length: float = 0.05,
                 finger_thickness: float = 0.01,
                 palm_thickness: float = 0.02,
                 **kwargs):
        """Initialize the parallel gripper.
        
        Args:
            physics_client: PyBullet physics client
            position: [x, y, z] position of the gripper
            orientation: [x, y, z, w] quaternion orientation of the gripper
            name: Name of the gripper for identification
            max_force: Maximum force applied by the gripper (N)
            finger_width: Width of each finger (m)
            finger_length: Length of each finger (m)
            finger_thickness: Thickness of each finger (m)
            palm_thickness: Thickness of the palm (m)
        """
        super().__init__(
            physics_client=physics_client,
            position=position,
            orientation=orientation,
            name=name,
            **kwargs
        )
        
        self.max_force = max_force
        self.finger_width = finger_width
        self.finger_length = finger_length
        self.finger_thickness = finger_thickness
        self.palm_thickness = palm_thickness
        
        # Finger joint positions (0.0 = open, 1.0 = closed)
        self.finger_positions = [0.0, 0.0]  # Left and right fingers
        self.target_positions = [0.0, 0.0]   # Target positions for control
        
        # Will be set during load()
        self.palm_id = None
        self.finger_ids = []
        self.finger_joint_indices = []
        self.finger_constraints = []
        
    def load(self) -> int:
        """Load the gripper model into the simulation."""
        # Create the gripper components
        self._create_gripper_geometry()
        
        # The gripper ID is the palm ID
        self.gripper_id = self.palm_id
        return self.gripper_id
    
    def _create_gripper_geometry(self):
        """Create the gripper's visual and collision geometry."""
        # Create the palm (base of the gripper)
        palm_collision_shape = self.physics_client.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[
                self.finger_width * 1.5,  # Width
                self.palm_thickness / 2,   # Thickness
                self.finger_width * 0.8    # Height
            ]
        )
        
        palm_visual_shape = self.physics_client.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[
                self.finger_width * 1.5,
                self.palm_thickness / 2,
                self.finger_width * 0.8
            ],
            rgbaColor=[0.3, 0.3, 0.3, 1.0]  # Dark gray
        )
        
        self.palm_id = self.physics_client.createMultiBody(
            baseMass=0.1,  # 100g
            baseCollisionShapeIndex=palm_collision_shape,
            baseVisualShapeIndex=palm_visual_shape,
            basePosition=self.position,
            baseOrientation=self.orientation
        )
        
        # Create the fingers
        finger_half_extents = [
            self.finger_thickness / 2,
            self.finger_width / 2,
            self.finger_length / 2
        ]
        
        finger_collision_shape = self.physics_client.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=finger_half_extents
        )
        
        finger_visual_shape = self.physics_client.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=finger_half_extents,
            rgbaColor=[0.5, 0.5, 0.5, 1.0]  # Light gray
        )
        
        # Finger positions relative to the palm
        finger_positions = [
            [0, self.palm_thickness / 2 + self.finger_thickness / 2, 0],  # Right finger
            [0, -self.palm_thickness / 2 - self.finger_thickness / 2, 0]   # Left finger
        ]
        
        # Create the fingers as multibodies connected to the palm with prismatic joints
        for i, rel_pos in enumerate(finger_positions):
            # Create the finger
            finger_id = self.physics_client.createMultiBody(
                baseMass=0.05,  # 50g
                baseCollisionShapeIndex=finger_collision_shape,
                baseVisualShapeIndex=finger_visual_shape,
                basePosition=[p + o for p, o in zip(self.position, rel_pos)],
                baseOrientation=self.orientation
            )
            
            # Create a constraint to connect the finger to the palm
            constraint_id = self.physics_client.createConstraint(
                parentBodyUniqueId=self.palm_id,
                parentLinkIndex=-1,  # Base of the palm
                childBodyUniqueId=finger_id,
                childLinkIndex=-1,   # Base of the finger
                jointType=p.JOINT_PRISMATIC,
                jointAxis=[0, 1, 0],  # Move along y-axis
                parentFramePosition=rel_pos,
                childFramePosition=[0, 0, 0],
                parentFrameOrientation=[0, 0, 0, 1]
            )
            
            # Store the finger and constraint information
            self.finger_ids.append(finger_id)
            self.finger_constraints.append(constraint_id)
            
            # Enable motor control for the constraint
            self.physics_client.changeConstraint(
                constraint_id,
                maxForce=self.max_force,
                erp=0.9,
                jointMaxVelocity=0.05
            )
    
    def activate(self, activate: bool = True):
        """Activate or deactivate the gripper."""
        super().activate(activate)
        
        # Set target positions based on activation state
        if activate:
            self.target_positions = [1.0, 1.0]  # Close gripper
        else:
            self.target_positions = [0.0, 0.0]  # Open gripper
    
    def set_finger_positions(self, positions: List[float]):
        """Set the target positions for the gripper fingers.
        
        Args:
            positions: List of target positions for each finger [left, right]
                     0.0 = fully open, 1.0 = fully closed
        """
        if len(positions) != 2:
            raise ValueError("Must provide exactly 2 finger positions")
            
        self.target_positions = [max(0.0, min(1.0, p)) for p in positions]
    
    def get_finger_positions(self) -> List[float]:
        """Get the current positions of the gripper fingers.
        
        Returns:
            List of current positions for each finger [left, right]
            0.0 = fully open, 1.0 = fully closed
        """
        return self.finger_positions.copy()
    
    def update(self):
        """Update the gripper state (to be called in the simulation loop)."""
        if not hasattr(self, 'finger_constraints') or not self.finger_constraints:
            return
            
        # Update each finger position based on the target
        for i, constraint_id in enumerate(self.finger_constraints):
            # Get the current position of the finger
            finger_pos = self.physics_client.getConstraintState(constraint_id)
            
            # Calculate the target position in world coordinates
            target_pos = self.target_positions[i] * self.finger_width * 0.9  # 90% of full width
            
            # Move the finger towards the target position
            self.physics_client.changeConstraint(
                constraint_id,
                jointChildPivot=[0, target_pos if i == 0 else -target_pos, 0],
                maxForce=self.max_force,
                erp=0.9,
                jointMaxVelocity=0.05
            )
            
            # Update the current finger position (normalized to [0, 1])
            self.finger_positions[i] = min(1.0, max(0.0, abs(finger_pos) / (self.finger_width * 0.9)))
    
    def reset(self):
        """Reset the gripper to its initial state."""
        super().reset()
        self.finger_positions = [0.0, 0.0]
        self.target_positions = [0.0, 0.0]
        
        # Reset finger positions
        for i, constraint_id in enumerate(getattr(self, 'finger_constraints', [])):
            self.physics_client.changeConstraint(
                constraint_id,
                jointChildPivot=[0, 0, 0],
                maxForce=self.max_force,
                erp=0.9,
                jointMaxVelocity=0.05
            )
    
    def get_state(self) -> Dict[str, Any]:
        """Get the current state of the gripper."""
        state = super().get_state()
        state.update({
            'type': 'parallel',
            'finger_positions': self.finger_positions.copy(),
            'target_positions': self.target_positions.copy(),
            'max_force': self.max_force,
            'finger_width': self.finger_width,
            'finger_length': self.finger_length,
            'finger_thickness': self.finger_thickness,
            'palm_thickness': self.palm_thickness
        })
        return state
