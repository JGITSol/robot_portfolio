"""UR5 Robot Arm implementation with configurable grippers."""
import os
import pybullet as p
import numpy as np
from typing import List, Dict, Any, Optional, Tuple

class UR5Robot:
    """UR5 Robot Arm with configurable gripper."""
    
    def __init__(self, 
                 physics_client: Any,
                 base_position: List[float],
                 urdf_path: str,
                 gripper_type: str,
                 gripper_urdf: str,
                 ee_link_name: str,
                 gripper_mount_offset: List[float] = None,
                 color: List[float] = None,
                 label: str = "UR5",
                 physics_client_id: int = 0):
        """Initialize the UR5 robot with a gripper.
        
        Args:
            physics_client: PyBullet physics client
            base_position: [x, y, z] position of the robot base
            urdf_path: Path to the URDF file
            gripper_type: Type of gripper
            gripper_urdf: Path to the gripper URDF
            ee_link_name: Name of the end effector link
            gripper_mount_offset: [x, y, z] offset for gripper mounting
            color: [r, g, b, a] color for visualization
            label: Display label for the robot
            physics_client_id: Physics client ID
        """
        self.p = physics_client
        self.physics_client_id = physics_client_id
        self.base_position = base_position
        self.urdf_path = urdf_path
        self.gripper_type = gripper_type
        self.label = label
        self.attached_object = None
        
        # Default mount offset if not provided
        if gripper_mount_offset is None:
            gripper_mount_offset = [0, 0, 0]
        self.gripper_mount_offset = gripper_mount_offset
        
        # Default color if not provided
        if color is None:
            color = [0.5, 0.5, 0.5, 1.0]  # Gray
        self.color = color
        
        # Load the UR5 robot
        # Loading robot arm (KUKA fallback or other)
        print(f"Loading robot arm from {urdf_path}")
        try:
            self.robot_id = self.p.loadURDF(
                urdf_path,
                basePosition=base_position,
                baseOrientation=self.p.getQuaternionFromEuler([0, 0, 0]),
                useFixedBase=True,
                flags=self.p.URDF_USE_SELF_COLLISION,
                physicsClientId=physics_client_id
            )
        except Exception as e:
            print(f"ERROR: Could not load robot arm URDF: {urdf_path}")
            raise
        
        # Get joint information
        self.num_joints = self.p.getNumJoints(self.robot_id, physics_client_id)
        self.joint_indices = list(range(self.num_joints))
        self.joint_limits = []
        self.joint_rest_positions = []
        
        # Get joint info and set up control
        for i in range(self.num_joints):
            joint_info = self.p.getJointInfo(self.robot_id, i, physics_client_id)
            joint_name = joint_info[1].decode('utf-8')
            joint_type = joint_info[2]
            
            # Only consider revolute and prismatic joints
            if joint_type in [self.p.JOINT_REVOLUTE, self.p.JOINT_PRISMATIC]:
                self.joint_limits.append((joint_info[8], joint_info[9]))  # lower, upper limits
                self.joint_rest_positions.append(0.0)  # Default rest position
                
                # Enable motor control
                self.p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=i,
                    controlMode=self.p.VELOCITY_CONTROL,
                    targetVelocity=0,
                    force=100,
                    physicsClientId=physics_client_id
                )
        
        # Find the end effector link index
        self.ee_link_idx = -1
        for i in range(self.num_joints):
            joint_info = self.p.getJointInfo(self.robot_id, i, physics_client_id)
            if joint_info[12].decode('utf-8') == ee_link_name:
                self.ee_link_idx = i
                break
        
        if self.ee_link_idx == -1:
            print(f"Warning: Could not find end effector link '{ee_link_name}'")
            self.ee_link_idx = self.num_joints - 1  # Default to last link
        
        # Load the gripper
        self.gripper = self._setup_gripper(gripper_urdf, gripper_type)
        
        # Apply visual properties
        self._apply_visuals()
        
        print(f"Initialized {label} with {self.num_joints} joints")

    
    def _setup_gripper(self, gripper_urdf: str, gripper_type: str) -> Dict[str, Any]:
        """Set up the gripper."""
        # In a real implementation, this would load and attach the gripper
        # For now, return a simple dictionary with gripper properties
        return {
            'type': gripper_type,
            'urdf': gripper_urdf,
            'mounted': True,
            'width': 0.085 if '2f_85' in gripper_type else 0.14  # Default widths in meters
        }
    
    def _apply_visuals(self):
        """Apply visual properties to the robot."""
        # Set base color
        self.p.changeVisualShape(
            self.robot_id, 
            -1,  # Base link
            rgbaColor=self.color,
            physicsClientId=self.physics_client_id
        )
        
        # Set link colors (slightly lighter than base)
        for i in range(self.num_joints):
            link_color = [min(1.0, c * 1.2) for c in self.color[:3]] + [1.0]  # Lighter color
            self.p.changeVisualShape(
                self.robot_id, 
                i, 
                rgbaColor=link_color,
                physicsClientId=self.physics_client_id
            )
    
    def get_joint_positions(self) -> List[float]:
        """Get current joint positions in radians."""
        positions = []
        for i in range(self.num_joints):
            joint_state = self.p.getJointState(
                self.robot_id, 
                i,
                physicsClientId=self.physics_client_id
            )
            positions.append(joint_state[0])
        return positions
    
    def set_joint_positions(self, target_positions: List[float], duration: float = 1.0):
        """Move to target joint positions."""
        # Simple position control - in a real implementation, you'd want to interpolate
        for i, pos in enumerate(target_positions):
            if i < self.num_joints:
                self.p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=i,
                    controlMode=self.p.POSITION_CONTROL,
                    targetPosition=pos,
                    targetVelocity=0,
                    force=100,
                    maxVelocity=1.0,
                    physicsClientId=self.physics_client_id
                )
    
    def move_to_pose(self, target_position: List[float], target_orientation: List[float] = None):
        """Move end effector to target pose using inverse kinematics."""
        if target_orientation is None:
            target_orientation = self.p.getQuaternionFromEuler([0, -np.pi, 0])  # Default orientation
            
        # Use PyBullet's IK solver
        joint_poses = self.p.calculateInverseKinematics(
            self.robot_id,
            self.ee_link_idx,
            target_position,
            targetOrientation=target_orientation,
            maxNumIterations=100,
            residualThreshold=1e-4,
            physicsClientId=self.physics_client_id
        )
        
        # Apply the joint positions
        self.set_joint_positions(joint_poses)
    
    def get_end_effector_pose(self) -> Tuple[List[float], List[float]]:
        """Get end effector position and orientation."""
        link_state = self.p.getLinkState(
            self.robot_id,
            self.ee_link_idx,
            computeForwardKinematics=True,
            physicsClientId=self.physics_client_id
        )
        return link_state[4], link_state[5]  # Position and orientation
    
    def close_gripper(self):
        """Close the gripper."""
        # In a real implementation, this would control the gripper
        print(f"{self.label} closing {self.gripper['type']} gripper")
    
    def open_gripper(self):
        """Open the gripper."""
        # In a real implementation, this would control the gripper
        print(f"{self.label} opening {self.gripper['type']} gripper")
    
    def pick(self, target_position: List[float]) -> bool:
        """Pick an object at the target position."""
        # Move above the target
        approach = target_position.copy()
        approach[2] += 0.1  # 10cm above
        
        # Move to approach position
        self.move_to_pose(approach)
        
        # Move down to target
        self.move_to_pose(target_position)
        
        # Close gripper
        self.close_gripper()
        
        # Move back up
        self.move_to_pose(approach)
        
        return True
    
    def place(self, target_position: List[float]) -> bool:
        """Place the held object at the target position."""
        # Similar to pick but opens the gripper at the end
        approach = target_position.copy()
        approach[2] += 0.1  # 10cm above
        
        # Move to approach position
        self.move_to_pose(approach)
        
        # Move down to target
        self.move_to_pose(target_position)
        
        # Open gripper
        self.open_gripper()
        
        # Move back up
        self.move_to_pose(approach)
        
        return True
    
    def move_to_rest_pose(self):
        """Move the robot to its rest position."""
        rest_positions = [0.0] * len(self.joint_rest_positions)
        self.set_joint_positions(rest_positions)
    
    def update(self):
        """Update the robot state."""
        # For any per-frame updates
        pass
    
    def get_status(self) -> Dict[str, Any]:
        """Get the current status of the robot."""
        return {
            'joint_positions': self.get_joint_positions(),
            'ee_pose': self.get_end_effector_pose(),
            'gripper': self.gripper,
            'label': self.label
        }
