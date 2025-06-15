"""Robot arm implementation with gripper support."""
import pybullet as p
import pybullet_data
import time
from typing import List, Optional, Dict, Any, Tuple
import numpy as np
from .grippers import create_gripper, Gripper

class RobotArm:
    """Robot arm with configurable gripper for object manipulation."""
    
    def __init__(self, physics_client_id, position: List[float], 
                 gripper_type: str = "parallel", name: str = "robot"):
        """Initialize the robotic arm with a specific gripper.
        
        Args:
            physics_client_id: PyBullet physics client ID (from p.connect())
            position: [x, y, z] position of the robot base
            gripper_type: Type of gripper ('parallel' or 'suction')
            name: Name of the robot for identification
        """
        self.physics_client_id = physics_client_id
        self.p = p  # Direct reference to pybullet module
        self.base_position = list(position)
        self.name = name
        self.gripper_type = gripper_type
        
        print(f"Loading robot URDF for {name}...")
        
        # Create a simple robot arm using a single multi-body with multiple links
        try:
            print(f"Creating simple robot arm for {name}...")
            
            # Create a list to hold link masses, shapes, and visual shapes
            link_masses = [0.0]  # Base mass (0 = fixed)
            link_collision_indices = []
            link_visual_indices = []
            link_positions = [[0, 0, 0]]  # Base position
            link_orientations = [p.getQuaternionFromEuler([0, 0, 0])]  # Base orientation
            
            # Joint properties
            num_links = 3
            link_radii = [0.2, 0.15, 0.1]  # Radius of each link
            link_lengths = [0.5, 0.4, 0.3]  # Length of each link
            
            # Colors for visualization
            colors = [
                [0.8, 0.2, 0.2, 1],  # Red
                [0.2, 0.8, 0.2, 1],  # Green
                [0.2, 0.2, 0.8, 1]   # Blue
            ]
            
            # Create base link (fixed)
            base_shape = p.createCollisionShape(
                p.GEOM_CYLINDER,
                radius=link_radii[0],
                height=0.1
            )
            base_visual = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=link_radii[0],
                length=0.1,
                rgbaColor=[0.3, 0.3, 0.3, 1]
            )
            
            link_collision_indices.append(base_shape)
            link_visual_indices.append(base_visual)
            
            # Create links
            for i in range(num_links):
                # Create link collision and visual shapes
                link_shape = p.createCollisionShape(
                    p.GEOM_CYLINDER,
                    radius=link_radii[i],
                    height=link_lengths[i]
                )
                
                link_visual = p.createVisualShape(
                    p.GEOM_CYLINDER,
                    radius=link_radii[i],
                    length=link_lengths[i],
                    rgbaColor=colors[i % len(colors)]
                )
                
                link_masses.append(1.0)  # Mass of each link
                link_collision_indices.append(link_shape)
                link_visual_indices.append(link_visual)
                
                # Position each link at the end of the previous one
                if i == 0:  # First link
                    link_positions.append([0, 0, 0.05 + link_lengths[i]/2])
                else:
                    link_positions.append([0, 0, link_lengths[i-1] + link_lengths[i]/2])
                
                link_orientations.append(p.getQuaternionFromEuler([0, 0, 0]))
            
            # Create the multi-body with all links
            self.robot_id = p.createMultiBody(
                baseMass=0,  # Fixed base
                baseCollisionShapeIndex=link_collision_indices[0],
                baseVisualShapeIndex=link_visual_indices[0],
                basePosition=self.base_position,
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                linkMasses=link_masses[1:],
                linkCollisionShapeIndices=link_collision_indices[1:],
                linkVisualShapeIndices=link_visual_indices[1:],
                linkPositions=link_positions[1:],
                linkOrientations=link_orientations[1:],
                linkInertialFramePositions=[[0, 0, 0]] * num_links,
                linkInertialFrameOrientations=[p.getQuaternionFromEuler([0, 0, 0])] * num_links,
                linkParentIndices=list(range(num_links)),
                linkJointTypes=[p.JOINT_REVOLUTE] * num_links,
                linkJointAxis=[[0, 1, 0]] * num_links  # Rotate around Y axis
            )
            
            print(f"Created robot with ID {self.robot_id} and {num_links} links")
            
            # Store joint information
            self.num_joints = num_links
            self.joint_indices = list(range(self.num_joints))
            
            # Enable motor control for each joint
            for i in range(self.num_joints):
                p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=i,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=0,
                    force=100,
                    physicsClientId=self.physics_client_id
                )
                
                # Set joint limits
                p.changeDynamics(
                    bodyUniqueId=self.robot_id,
                    linkIndex=i,
                    jointLowerLimit=-3.14,  # -180 degrees
                    jointUpperLimit=3.14,    # 180 degrees
                    physicsClientId=self.physics_client_id
                )
                
                print(f"Configured joint {i} for robot {self.robot_id}")
            
            print(f"Created {self.num_joints} revolute joints")
            
        except Exception as e:
            print(f"Error initializing robot {name}: {str(e)}")
            import traceback
            traceback.print_exc()
            raise
        
        # Store joint limits and rest positions
        self.joint_limits = []
        self.joint_rest_positions = []
        for i in self.joint_indices:
            joint_info = self.p.getJointInfo(self.robot_id, i)
            self.joint_limits.append((joint_info[8], joint_info[9]))  # lower, upper limits
            self.joint_rest_positions.append(0.0)  # Default rest position
        
        # Add gripper
        self.gripper = self._setup_gripper(gripper_type)
        self.attached_object = None
        
        # Set default control parameters
        self.control_mode = p.POSITION_CONTROL
        self.max_velocity = 0.5  # rad/s
        self.max_force = 200.0   # N
        
    def _setup_gripper(self, gripper_type: str) -> Gripper:
        """Create and attach a gripper to the robot."""
        # Get end effector position and orientation
        ee_state = self.p.getLinkState(
            self.robot_id, 
            self.joint_indices[-1],
            physicsClientId=self.physics_client_id
        )
        ee_pos, ee_orn = ee_state[0], ee_state[1]
        
        # Create gripper
        gripper = create_gripper(
            gripper_type,
            physics_client=self.p,
            position=ee_pos,
            orientation=ee_orn,
            physics_client_id=self.physics_client_id
        )
        
        # Load and attach gripper
        gripper.load()
        gripper.attach_to_robot(self.robot_id, self.joint_indices[-1])
        
        return gripper
    
    def get_joint_positions(self) -> List[float]:
        """Get current joint positions in radians."""
        return [self.physics_client.getJointState(self.robot_id, i)[0] 
                for i in self.joint_indices]
    
    def set_joint_positions(self, 
                          target_positions: List[float], 
                          duration: float = 1.0) -> bool:
        """
        Move to target joint positions smoothly.
        
        Args:
            target_positions: Target joint angles in radians
            duration: Time in seconds to reach the target
            
        Returns:
            bool: True if movement completed successfully
        """
        current_pos = self.get_joint_positions()
        start_time = time.time()
        end_time = start_time + duration
        
        while time.time() < end_time:
            # Calculate interpolation factor (0 to 1)
            alpha = (time.time() - start_time) / duration
            alpha = np.clip(alpha, 0, 1)
            
            # Interpolate positions
            interp_pos = [
                current + alpha * (target - current)
                for current, target in zip(current_pos, target_positions)
            ]
            
            # Apply position control
            for i, pos in enumerate(self.joint_indices):
                self.physics_client.setJointMotorControl2(
                    bodyIndex=self.robot_id,
                    jointIndex=pos,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=interp_pos[i],
                    targetVelocity=0,
                    positionGain=0.5,
                    velocityGain=1.0,
                    force=self.max_force
                )
            
            # Step simulation
            self.physics_client.stepSimulation()
            time.sleep(1./240.)  # Assume 240 Hz simulation
        
        return True
    
    def move_to_pose(self, 
                    target_position: List[float],
                    target_orientation: Optional[List[float]] = None,
                    duration: float = 2.0) -> bool:
        """
        Move end effector to target pose using inverse kinematics.
        
        Args:
            target_position: [x, y, z] target position in world coordinates
            target_orientation: [x, y, z, w] target orientation as quaternion
            duration: Time in seconds to reach the target
            
        Returns:
            bool: True if movement completed successfully
        """
        if target_orientation is None:
            # Default orientation (pointing forward)
            target_orientation = p.getQuaternionFromEuler([0, -1.57, 0])
        
        # Calculate inverse kinematics
        joint_positions = self.physics_client.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=self.joint_indices[-1],
            targetPosition=target_position,
            targetOrientation=target_orientation,
            maxNumIterations=100,
            residualThreshold=1e-5
        )
        
        # Set joint positions (first n joints where n is number of revolute joints)
        joint_positions = joint_positions[:len(self.joint_indices)]
        return self.set_joint_positions(joint_positions, duration)
    
    def pick(self, target_position: List[float]) -> bool:
        """
        Pick up an object at the target position.
        
        Args:
            target_position: [x, y, z] position to pick from
            
        Returns:
            bool: True if pick was successful
        """
        # Move above the object
        above_pos = target_position.copy()
        above_pos[2] += 0.1  # 10cm above
        
        # Move to approach position
        if not self.move_to_pose(above_pos):
            return False
        
        # Move down to pick
        if not self.move_to_pose(target_position):
            return False
        
        # Close gripper
        self.gripper.close()
        
        # Move back up
        if not self.move_to_pose(above_pos):
            return False
            
        return True
    
    def place(self, target_position: List[float]) -> bool:
        """
        Place the held object at the target position.
        
        Args:
            target_position: [x, y, z] position to place at
            
        Returns:
            bool: True if place was successful
        """
        # Move above the target
        above_pos = target_position.copy()
        above_pos[2] += 0.1  # 10cm above
        
        # Move to approach position
        if not self.move_to_pose(above_pos):
            return False
        
        # Move down to place
        if not self.move_to_pose(target_position):
            return False
        
        # Open gripper
        self.gripper.open()
        
        # Move back up
        if not self.move_to_pose(above_pos):
            return False
            
        return True
    
    def move_to_rest_pose(self) -> bool:
        """Move the robot to its rest position."""
        return self.set_joint_positions(self.joint_rest_positions)
    
    def get_end_effector_pose(self) -> Tuple[List[float], List[float]]:
        """Get end effector position and orientation."""
        return p.getLinkState(self.robot_id, self.joint_indices[-1])[4:6]
    
    def get_joint_velocities(self) -> List[float]:
        """Get current joint velocities in rad/s."""
        return [self.physics_client.getJointState(self.robot_id, i)[1] 
                for i in self.joint_indices]
    
    def get_joint_torques(self) -> List[float]:
        """Get current joint torques in Nm."""
        return [self.physics_client.getJointState(self.robot_id, i)[3] 
                for i in self.joint_indices]
    
    def is_pose_reachable(self, 
                         target_position: List[float], 
                         target_orientation: Optional[List[float]] = None) -> bool:
        """Check if a pose is reachable by the robot."""
        try:
            joint_positions = self.physics_client.calculateInverseKinematics(
                bodyUniqueId=self.robot_id,
                endEffectorLinkIndex=self.joint_indices[-1],
                targetPosition=target_position,
                targetOrientation=target_orientation or [0, 0, 0, 1],
                maxNumIterations=100,
                residualThreshold=1e-5
            )
            
            # Check if all joint positions are within limits
            for i, pos in enumerate(joint_positions[:len(self.joint_indices)]):
                if pos < self.joint_limits[i][0] or pos > self.joint_limits[i][1]:
                    return False
            return True
        except:
            return False
    
    def close(self):
        """Clean up resources."""
        # Remove constraints and clean up
        if hasattr(self, 'constraint_id'):
            self.physics_client.removeConstraint(self.constraint_id)
        # Note: Don't disconnect physics client here as it might be shared
