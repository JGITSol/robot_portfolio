"""UR5 robot arm implementation."""
import os
from typing import List, Dict, Any, Optional, Tuple
import numpy as np
import pybullet as p
from .base_robot import BaseRobot

class UR5Robot(BaseRobot):
    """UR5 robot arm implementation."""
    
    def __init__(self, 
                 physics_client: Any,
                 base_position: List[float],
                 urdf_path: str,
                 gripper_type: str = "parallel",
                 gripper_urdf: Optional[str] = None,
                 ee_link_name: str = "tool0",
                 gripper_mount_offset: Optional[List[float]] = None,
                 color: Optional[List[float]] = None,
                 name: str = "UR5",
                 **kwargs):
        """Initialize the UR5 robot.
        
        Args:
            physics_client: PyBullet physics client
            base_position: [x, y, z] position of the robot base
            urdf_path: Path to the URDF file
            gripper_type: Type of gripper to attach
            gripper_urdf: Optional path to gripper URDF
            ee_link_name: Name of the end effector link
            gripper_mount_offset: [x, y, z] offset for gripper mounting
            color: [r, g, b, a] color for visualization
            name: Name of the robot for identification
        """
        super().__init__(
            physics_client=physics_client,
            base_position=base_position,
            name=name,
            **kwargs
        )
        
        self.urdf_path = urdf_path
        self.gripper_type = gripper_type
        self.gripper_urdf = gripper_urdf
        self.ee_link_name = ee_link_name
        self.gripper_mount_offset = np.array(gripper_mount_offset or [0, 0, 0], dtype=np.float32)
        self.color = color or [0.5, 0.5, 0.5, 1.0]  # Default gray
        
        # Will be set during load()
        self.ee_link_index = -1
        self.joint_indices = []
        self.joint_limits = {}
        
    def load(self) -> int:
        """Load the robot model into the simulation."""
        # Load the URDF
        self.robot_id = self.physics_client.loadURDF(
            self.urdf_path,
            basePosition=self.base_position,
            baseOrientation=self.base_orientation,
            useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION
        )
        
        # Get joint information
        joint_info = self.get_joint_info()
        
        # Find the end effector link index
        self.ee_link_index = -1
        for i, info in enumerate(joint_info):
            if info['linkName'] == self.ee_link_name:
                self.ee_link_index = i
                break
                
        if self.ee_link_index == -1:
            raise ValueError(f"End effector link '{self.ee_link_name}' not found in URDF")
        
        # Collect all revolute joint indices
        self.joint_indices = [
            info['jointIndex'] 
            for info in joint_info 
            if info['jointType'] == p.JOINT_REVOLUTE
        ]
        
        # Set joint limits
        for info in joint_info:
            if info['jointType'] == p.JOINT_REVOLUTE:
                self.joint_limits[info['jointIndex']] = (
                    info['jointLowerLimit'], 
                    info['jointUpperLimit']
                )
        
        # Apply color if specified
        if self.color:
            self._apply_color()
            
        return self.robot_id
    
    def _apply_color(self):
        """Apply color to the robot links."""
        if self.robot_id is None:
            return
            
        # Set base color
        self.physics_client.changeVisualShape(
            self.robot_id, 
            -1,  # Base link
            rgbaColor=self.color
        )
        
        # Set link colors (slightly lighter than base)
        num_joints = self.physics_client.getNumJoints(self.robot_id)
        for i in range(num_joints):
            link_color = [min(1.0, c * 1.2) for c in self.color[:3]] + [1.0]  # Lighter color
            self.physics_client.changeVisualShape(
                self.robot_id, 
                i, 
                rgbaColor=link_color
            )
    
    def get_end_effector_pose(self) -> Tuple[List[float], List[float]]:
        """Get the current end-effector position and orientation."""
        if self.robot_id is None:
            raise RuntimeError("Robot not loaded in simulation")
            
        # Get the current state of the end effector
        link_state = self.physics_client.getLinkState(
            self.robot_id,
            self.ee_link_index,
            computeForwardKinematics=True
        )
        
        # Position and orientation of the end effector
        position = link_state[4]  # World position of the URDF link frame
        orientation = link_state[5]  # World orientation of the URDF link frame
        
        # Apply gripper offset if any
        if self.gripper_mount_offset.any():
            # Transform the offset by the end effector orientation
            rotation_matrix = np.array(
                self.physics_client.getMatrixFromQuaternion(orientation)
            ).reshape(3, 3)
            offset_world = rotation_matrix @ self.gripper_mount_offset
            position = [p + o for p, o in zip(position, offset_world)]
            
        return position, orientation
    
    def move_to_pose(self, 
                   target_position: List[float], 
                   target_orientation: Optional[List[float]] = None,
                   **kwargs):
        """Move end-effector to target pose using inverse kinematics.
        
        Args:
            target_position: [x, y, z] target position in world coordinates
            target_orientation: [x, y, z, w] target orientation as quaternion
            **kwargs: Additional arguments for inverse kinematics
        """
        if self.robot_id is None:
            raise RuntimeError("Robot not loaded in simulation")
            
        # Default orientation if not provided
        if target_orientation is None:
            # Keep current orientation if target not specified
            _, current_orientation = self.get_end_effector_pose()
            target_orientation = current_orientation
            
        # Calculate inverse kinematics
        joint_positions = self.physics_client.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=self.ee_link_index,
            targetPosition=target_position,
            targetOrientation=target_orientation,
            **kwargs
        )
        
        # Set joint positions (only for the revolute joints)
        self.set_joint_positions(joint_positions[:len(self.joint_indices)])
        
        return joint_positions
    
    def get_jacobian(self, joint_positions: Optional[List[float]] = None) -> np.ndarray:
        """Compute the Jacobian matrix for the current configuration.
        
        Args:
            joint_positions: Optional joint positions. If None, use current positions.
            
        Returns:
            J: 6xN Jacobian matrix where N is the number of joints
        """
        if self.robot_id is None:
            raise RuntimeError("Robot not loaded in simulation")
            
        if joint_positions is None:
            joint_positions = self.get_joint_positions()
            
        # Get the current end-effector state
        link_state = self.physics_client.getLinkState(
            self.robot_id,
            self.ee_link_index,
            computeLinkVelocity=1,
            computeForwardKinematics=1
        )
        
        # Get the Jacobian
        linear_jacobian, angular_jacobian = self.physics_client.calculateJacobian(
            self.robot_id,
            self.ee_link_index,
            localPosition=[0, 0, 0],  # Position relative to the link frame
            objPositions=joint_positions,
            objVelocities=[0] * len(joint_positions),
            objAccelerations=[0] * len(joint_positions)
        )
        
        # Combine linear and angular Jacobians
        J = np.vstack((linear_jacobian, angular_jacobian))
        return J
    
    def is_pose_reachable(self, 
                         target_position: List[float], 
                         target_orientation: Optional[List[float]] = None,
                         tolerance: float = 0.01) -> bool:
        """Check if a pose is reachable by the robot.
        
        Args:
            target_position: [x, y, z] target position
            target_orientation: [x, y, z, w] target orientation as quaternion
            tolerance: Position tolerance for considering the pose reached
            
        Returns:
            bool: True if the pose is reachable, False otherwise
        """
        try:
            # Try to compute IK
            joint_positions = self.move_to_pose(
                target_position,
                target_orientation,
                maxNumIterations=100,
                residualThreshold=1e-4
            )
            
            # Check if the solution brings the end effector close enough
            self.set_joint_positions(joint_positions)
            
            # Step the simulation to update the robot state
            for _ in range(10):  # Small number of steps
                self.physics_client.stepSimulation()
                
            # Get the actual end-effector position after moving
            actual_position, actual_orientation = self.get_end_effector_pose()
            
            # Check position error
            position_error = np.linalg.norm(
                np.array(target_position) - np.array(actual_position)
            )
            
            # Check orientation error (quaternion distance)
            if target_orientation is not None:
                q1 = np.array(target_orientation)
                q2 = np.array(actual_orientation)
                orientation_error = 1.0 - np.abs(np.dot(q1, q2))  # 1 - |q1.q2|
            else:
                orientation_error = 0.0
                
            return position_error <= tolerance and orientation_error <= 0.1
            
        except Exception as e:
            print(f"IK failed: {e}")
            return False
