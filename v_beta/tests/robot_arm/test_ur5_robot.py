"""Tests for the UR5 robot implementation."""
import pytest
import numpy as np
from unittest.mock import MagicMock, patch

class TestUR5Robot:
    """Test cases for the UR5Robot class."""
    
    @pytest.fixture
    def mock_physics_client(self):
        """Create a mock PyBullet physics client."""
        with patch('pybullet') as mock_pb:
            # Common PyBullet return values
            mock_pb.getQuaternionFromEuler.return_value = [0, 0, 0, 1]
            mock_pb.getEulerFromQuaternion.return_value = [0, 0, 0]
            mock_pb.getNumJoints.return_value = 6  # UR5 has 6 joints
            
            # Mock joint info for UR5
            mock_pb.getJointInfo.side_effect = [
                (0, b'shoulder_pan_joint', 0, -1, -1, 0, [0], [0], [0], [0], [0], [0, 0, 0], [0, 0, 0]),
                (1, b'shoulder_lift_joint', 0, -1, -1, 0, [0], [0], [0], [0], [0], [0, 0, 0], [0, 0, 0]),
                (2, b'elbow_joint', 0, -1, -1, 0, [0], [0], [0], [0], [0], [0, 0, 0], [0, 0, 0]),
                (3, b'wrist_1_joint', 0, -1, -1, 0, [0], [0], [0], [0], [0], [0, 0, 0], [0, 0, 0]),
                (4, b'wrist_2_joint', 0, -1, -1, 0, [0], [0], [0], [0], [0], [0, 0, 0], [0, 0, 0]),
                (5, b'wrist_3_joint', 0, -1, -1, 0, [0], [0], [0], [0], [0], [0, 0, 0], [0, 0, 0]),
            ]
            
            # Mock joint states
            mock_pb.getJointState.return_value = ([0.1] * 6, [0.0] * 6)  # pos, vel
            
            # Mock IK
            mock_pb.calculateInverseKinematics.return_value = [0.1] * 6
            
            yield mock_pb
    
    @pytest.fixture
    def ur5_robot(self, mock_physics_client):
        """Create a UR5Robot instance for testing."""
        from robot_arm.ur5_robot import UR5Robot
        return UR5Robot(
            physics_client=mock_physics_client,
            base_position=[0, 0, 0],
            base_orientation=[0, 0, 0, 1]
        )
    
    def test_initialization(self, ur5_robot, mock_physics_client):
        """Test UR5 robot initialization."""
        assert ur5_robot.num_joints == 6
        assert ur5_robot.joint_names[0] == 'shoulder_pan_joint'
        assert ur5_robot.joint_limits[0] == (-np.pi, np.pi)  # Default limits
        
        # Verify URDF was loaded with correct parameters
        mock_physics_client.loadURDF.assert_called_once()
        args, kwargs = mock_physics_client.loadURDF.call_args
        assert 'ur5/ur5.urdf' in args[0]  # Check URDF path
        
    def test_forward_kinematics(self, ur5_robot, mock_physics_client):
        """Test forward kinematics calculation."""
        # Test position
        joint_positions = [0.1] * 6
        position, orientation = ur5_robot.forward_kinematics(joint_positions)
        
        # Should return position and orientation
        assert len(position) == 3
        assert len(orientation) == 4  # Quaternion
        
        # Verify PyBullet was called with correct parameters
        mock_physics_client.calculateForwardKinematics.assert_called_once()
        
    def test_inverse_kinematics(self, ur5_robot, mock_physics_client):
        """Test inverse kinematics calculation."""
        target_position = [0.5, 0, 0.5]
        target_orientation = [0, 0, 0, 1]
        
        joint_positions = ur5_robot.inverse_kinematics(
            target_position=target_position,
            target_orientation=target_orientation
        )
        
        # Should return joint positions
        assert len(joint_positions) == 6
        
        # Verify PyBullet was called with correct parameters
        mock_physics_client.calculateInverseKinematics.assert_called_once()
        
    def test_move_to_joint_positions(self, ur5_robot, mock_physics_client):
        """Test moving to joint positions."""
        target_positions = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
        ur5_robot.move_to_joint_positions(target_positions, duration=1.0)
        
        # Verify PyBullet was called with correct parameters
        mock_physics_client.setJointMotorControlArray.assert_called_once()
        args, kwargs = mock_physics_client.setJointMotorControlArray.call_args
        
        # Check target positions match
        assert kwargs['targetPositions'] == target_positions
        assert kwargs['controlMode'] == mock_physics_client.POSITION_CONTROL
        
    def test_move_to_pose(self, ur5_robot, mock_physics_client):
        """Test moving to a cartesian pose."""
        target_position = [0.5, 0, 0.5]
        target_orientation = [0, 0, 0, 1]
        
        ur5_robot.move_to_pose(
            target_position=target_position,
            target_orientation=target_orientation,
            duration=1.0
        )
        
        # Verify IK was called
        mock_physics_client.calculateInverseKinematics.assert_called_once()
        
        # Verify move_to_joint_positions was called with IK result
        mock_physics_client.setJointMotorControlArray.assert_called_once()
        
    def test_get_end_effector_pose(self, ur5_robot, mock_physics_client):
        """Test getting end effector pose."""
        # Mock link state
        mock_physics_client.getLinkState.return_value = (
            [0.5, 0, 0.5],  # Position
            [0, 0, 0, 1],    # Orientation
            None, None, None, None, None
        )
        
        position, orientation = ur5_robot.get_end_effector_pose()
        
        # Should return position and orientation
        assert position == [0.5, 0, 0.5]
        assert orientation == [0, 0, 0, 1]
        
        # Verify PyBullet was called with correct parameters
        mock_physics_client.getLinkState.assert_called_once_with(
            ur5_robot.robot_id,
            ur5_robot.end_effector_link_index
        )
    
    def test_set_joint_positions(self, ur5_robot, mock_physics_client):
        """Test setting joint positions directly."""
        target_positions = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
        ur5_robot.set_joint_positions(target_positions)
        
        # Verify PyBullet was called with correct parameters
        mock_physics_client.resetJointState.assert_has_calls([
            call(ur5_robot.robot_id, i, pos)
            for i, pos in enumerate(target_positions)
        ])
    
    def test_get_joint_positions(self, ur5_robot, mock_physics_client):
        """Test getting current joint positions."""
        # Mock joint states
        mock_physics_client.getJointState.side_effect = [
            ([pos], None) for pos in [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
        ]
        
        positions = ur5_robot.get_joint_positions()
        
        # Should return joint positions
        assert len(positions) == 6
        assert positions == [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
        
        # Verify PyBullet was called for each joint
        assert mock_physics_client.getJointState.call_count == 6
    
    def test_get_joint_velocities(self, ur5_robot, mock_physics_client):
        """Test getting current joint velocities."""
        # Mock joint states
        mock_physics_client.getJointState.side_effect = [
            (None, [vel]) for vel in [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
        ]
        
        velocities = ur5_robot.get_joint_velocities()
        
        # Should return joint velocities
        assert len(velocities) == 6
        assert velocities == [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
        
        # Verify PyBullet was called for each joint
        assert mock_physics_client.getJointState.call_count == 6
