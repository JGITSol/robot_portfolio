"""Tests for the main robotic arm module."""
import pytest
import numpy as np
from unittest.mock import MagicMock, patch, ANY
import pybullet as p

class TestRoboticArm:
    """Test cases for the RoboticArm class."""
    
    @pytest.fixture
    def mock_pybullet(self):
        """Create a mock PyBullet module."""
        with patch('pybullet') as mock_pb:
            # Mock common PyBullet functions
            mock_pb.GUI = 1
            mock_pb.DIRECT = 2
            mock_pb.JOINT_REVOLUTE = 0
            
            # Mock joint info
            mock_joint_info = [
                (0, b'joint_1', 0, -1, -1, 0, [0], [0], [0], [0], [0], [0, 0, 0], [0, 0, 0]),
                (1, b'joint_2', 0, -1, -1, 0, [0], [0], [0], [0], [0], [0, 0, 0], [0, 0, 0]),
                (2, b'joint_3', 0, -1, -1, 0, [0], [0], [0], [0], [0], [0, 0, 0], [0, 0, 0]),
                (3, b'joint_4', 0, -1, -1, 0, [0], [0], [0], [0], [0], [0, 0, 0], [0, 0, 0]),
                (4, b'joint_5', 0, -1, -1, 0, [0], [0], [0], [0], [0], [0, 0, 0], [0, 0, 0]),
                (5, b'joint_6', 0, -1, -1, 0, [0], [0], [0], [0], [0], [0, 0, 0], [0, 0, 0]),
            ]
            
            # Mock getNumJoints and getJointInfo
            mock_pb.getNumJoints.return_value = 6
            mock_pb.getJointInfo.side_effect = mock_joint_info
            
            # Mock getJointState
            mock_pb.getJointState.return_value = (0.0, 0.0)  # pos, vel
            
            # Mock other PyBullet functions
            mock_pb.getQuaternionFromEuler.return_value = [0, 0, 0, 1]
            mock_pb.calculateInverseKinematics.return_value = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
            
            yield mock_pb
    
    @pytest.fixture
    def robotic_arm(self, mock_pybullet):
        """Create a RoboticArm instance for testing."""
        from robot_arm.main import RoboticArm
        return RoboticArm(gui=False, timestep=1/240.0)
    
    def test_initialization(self, robotic_arm, mock_pybullet):
        """Test RoboticArm initialization."""
        assert robotic_arm.num_joints == 6
        assert len(robotic_arm.joint_indices) == 6
        assert len(robotic_arm.joint_limits) == 6
        
        # Verify PyBullet was called with correct parameters
        mock_pybullet.connect.assert_called_with(mock_pybullet.DIRECT)
        mock_pybullet.setGravity.assert_called_with(0, 0, -9.81)
        mock_pybullet.setTimeStep.assert_called_with(1/240.0)
    
    def test_get_joint_positions(self, robotic_arm, mock_pybullet):
        """Test getting joint positions."""
        # Setup mock joint states
        mock_states = [(i * 0.1, 0.0) for i in range(6)]  # pos, vel
        mock_pybullet.getJointState.side_effect = mock_states
        
        positions = robotic_arm.get_joint_positions()
        
        assert len(positions) == 6
        assert positions == [i * 0.1 for i in range(6)]
    
    def test_move_to_rest_pose(self, robotic_arm, mock_pybullet):
        """Test moving to rest pose."""
        robotic_arm.move_to_rest_pose()
        
        # Verify setJointMotorControlArray was called with correct parameters
        mock_pybullet.setJointMotorControlArray.assert_called_once()
        args, kwargs = mock_pybullet.setJointMotorControlArray.call_args
        
        assert args[0] == robotic_arm.robot_id
        assert kwargs['controlMode'] == mock_pybullet.POSITION_CONTROL
        assert kwargs['targetPositions'] == [0.0] * 6
    
    def test_set_joint_positions(self, robotic_arm, mock_pybullet):
        """Test setting joint positions."""
        target_positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        robotic_arm.set_joint_positions(target_positions)
        
        # Verify setJointMotorControlArray was called with correct parameters
        mock_pybullet.setJointMotorControlArray.assert_called_once()
        args, kwargs = mock_pybullet.setJointMotorControlArray.call_args
        
        assert args[0] == robotic_arm.robot_id
        assert kwargs['controlMode'] == mock_pybullet.POSITION_CONTROL
        assert kwargs['targetPositions'] == target_positions
    
    def test_get_end_effector_pose(self, robotic_arm, mock_pybullet):
        """Test getting end effector pose."""
        # Setup mock link state
        mock_pybullet.getLinkState.return_value = (
            [0.4, 0.2, 0.3],  # position
            [0, 0, 0, 1],     # orientation
            None, None, None, None, None, None
        )
        
        position, orientation = robotic_arm.get_end_effector_pose()
        
        assert position == [0.4, 0.2, 0.3]
        assert orientation == [0, 0, 0, 1]
        mock_pybullet.getLinkState.assert_called_once_with(
            robotic_arm.robot_id, robotic_arm.num_joints - 1
        )
    
    def test_move_to_pose(self, robotic_arm, mock_pybullet):
        """Test moving to a target pose."""
        target_position = [0.5, 0.2, 0.3]
        target_orientation = [0, 0, 0, 1]
        
        # Mock IK solution
        mock_ik_solution = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        mock_pybullet.calculateInverseKinematics.return_value = mock_ik_solution
        
        robotic_arm.move_to_pose(target_position, target_orientation)
        
        # Verify IK was called with correct parameters
        mock_pybullet.calculateInverseKinematics.assert_called_once()
        
        # Verify set_joint_positions was called with IK solution
        mock_pybullet.setJointMotorControlArray.assert_called()
        args, kwargs = mock_pybullet.setJointMotorControlArray.call_args
        assert kwargs['targetPositions'] == mock_ik_solution


def test_main():
    """Test the main function."""
    with patch('robot_arm.main.RoboticArm') as mock_robotic_arm:
        # Import here to avoid loading the module during test collection
        from robot_arm.main import main
        
        # Call the main function
        main()
        
        # Verify RoboticArm was instantiated
        mock_robotic_arm.assert_called_once()
