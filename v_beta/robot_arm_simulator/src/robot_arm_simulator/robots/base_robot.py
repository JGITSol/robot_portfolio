"""Base class for robot arms with common functionality."""
from typing import List, Optional, Dict, Any, Tuple
import numpy as np
import pybullet as p

class BaseRobot:
    """Base class for all robot arms in the simulation."""
    
    def __init__(self, 
                 physics_client: Any,
                 base_position: List[float],
                 base_orientation: Optional[List[float]] = None,
                 name: str = "robot",
                 **kwargs):
        """Initialize the base robot.
        
        Args:
            physics_client: PyBullet physics client
            base_position: [x, y, z] position of the robot base
            base_orientation: [x, y, z, w] quaternion orientation of the base
            name: Name of the robot for identification
        """
        self.physics_client = physics_client
        self.base_position = np.array(base_position, dtype=np.float32)
        self.base_orientation = base_orientation or [0, 0, 0, 1]
        self.name = name
        self.robot_id = None
        self.joint_indices = []
        self.joint_limits = {}
        self.gripper = None
        self.attached_object = None
        
    def load_urdf(self, urdf_path: str, **kwargs) -> int:
        """Load the robot URDF into the simulation."""
        self.robot_id = self.physics_client.loadURDF(
            urdf_path,
            basePosition=self.base_position,
            baseOrientation=self.base_orientation,
            useFixedBase=True,
            **kwargs
        )
        return self.robot_id
    
    def get_joint_info(self):
        """Get information about all joints in the robot."""
        if self.robot_id is None:
            raise RuntimeError("Robot not loaded in simulation")
            
        num_joints = self.physics_client.getNumJoints(self.robot_id)
        joint_info = []
        
        for i in range(num_joints):
            info = self.physics_client.getJointInfo(self.robot_id, i)
            joint_info.append({
                'jointIndex': info[0],
                'jointName': info[1].decode('utf-8'),
                'jointType': info[2],
                'qIndex': info[3],
                'uIndex': info[4],
                'flags': info[5],
                'jointDamping': info[6],
                'jointFriction': info[7],
                'jointLowerLimit': info[8],
                'jointUpperLimit': info[9],
                'jointMaxForce': info[10],
                'jointMaxVelocity': info[11],
                'linkName': info[12].decode('utf-8'),
                'jointAxis': info[13],
                'parentFramePos': info[14],
                'parentFrameOrn': info[15],
                'parentIndex': info[16]
            })
            
            # Store joint limits for later use
            if info[2] == self.physics_client.JOINT_REVOLUTE or info[2] == self.physics_client.JOINT_PRISMATIC:
                self.joint_limits[info[0]] = (info[8], info[9])
                
        return joint_info
    
    def get_joint_positions(self) -> List[float]:
        """Get current joint positions."""
        if self.robot_id is None:
            raise RuntimeError("Robot not loaded in simulation")
            
        joint_states = self.physics_client.getJointStates(
            self.robot_id, 
            self.joint_indices
        )
        return [state[0] for state in joint_states]
    
    def set_joint_positions(self, positions: List[float]):
        """Set target joint positions."""
        if self.robot_id is None:
            raise RuntimeError("Robot not loaded in simulation")
            
        for i, joint_idx in enumerate(self.joint_indices):
            if i < len(positions):
                self.physics_client.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=joint_idx,
                    controlMode=self.physics_client.POSITION_CONTROL,
                    targetPosition=positions[i],
                    maxVelocity=1.0,
                    force=100.0
                )
    
    def get_end_effector_pose(self) -> Tuple[List[float], List[float]]:
        """Get the current end-effector position and orientation."""
        if self.robot_id is None:
            raise RuntimeError("Robot not loaded in simulation")
            
        # This method should be implemented by subclasses
        raise NotImplementedError("Subclasses must implement get_end_effector_pose()")
    
    def move_to_pose(self, 
                   target_position: List[float], 
                   target_orientation: Optional[List[float]] = None):
        """Move end-effector to target pose using inverse kinematics."""
        # This method should be implemented by subclasses
        raise NotImplementedError("Subclasses must implement move_to_pose()")
    
    def attach_gripper(self, gripper):
        """Attach a gripper to the robot's end effector."""
        self.gripper = gripper
        # Implementation depends on the robot and gripper type
        
    def detach_gripper(self):
        """Detach the current gripper."""
        if self.gripper:
            self.gripper.detach()
            self.gripper = None
    
    def attach_object(self, object_id: int):
        """Attach an object to the robot's gripper."""
        if self.gripper:
            self.gripper.attach_object(object_id)
            self.attached_object = object_id
    
    def detach_object(self):
        """Detach any attached object."""
        if self.gripper and self.attached_object is not None:
            self.gripper.detach_object()
            self.attached_object = None
    
    def update(self):
        """Update the robot state (e.g., gripper control)."""
        if self.gripper:
            self.gripper.update()
    
    def reset(self):
        """Reset the robot to its initial state."""
        if self.robot_id is not None:
            # Reset all joints to zero position
            for joint_idx in self.joint_indices:
                self.physics_client.resetJointState(
                    self.robot_id, 
                    joint_idx, 
                    targetValue=0.0
                )
        
        if self.gripper:
            self.gripper.reset()
