"""Tests for the robot_arm module."""
import math
import pytest
import numpy as np
from unittest.mock import MagicMock, patch, call

class TestRobotArm:
    """Test cases for the RobotArm class."""
    
    @pytest.fixture
    def mock_physics_client(self):
        """Create a mock PyBullet physics client."""
        with patch('pybullet') as mock_pb:
            # Common PyBullet return values
            mock_pb.getQuaternionFromEuler.return_value = [0, 0, 0, 1]
            mock_pb.getEulerFromQuaternion.return_value = [0, 0, 0]
            mock_pb.getNumJoints.return_value = 6  # Typical for UR5
            mock_pb.getJointInfo.return_value = (0, b'joint', 0, -1, -1, 0, [0], [0], [0], [0], [0], [0, 0, 0], [0, 0, 0])
            mock_pb.getJointState.return_value = ([0.1] * 6, [0.0] * 6)  # pos, vel
            mock_pb.calculateInverseKinematics.return_value = [0.1] * 6
            yield mock_pb
    
    @pytest.fixture
    def robot(self, mock_physics_client):
        """Create a RobotArm instance for testing."""
        from robot_arm.robot import RobotArm
        return RobotArm(
            physics_client=mock_physics_client,
            base_position=[0, 0, 0],
            base_orientation=[0, 0, 0, 1],
            urdf_path="dummy.urdf",
            ee_link_name="ee_link"
        )
    
    def test_initialization(self, robot, mock_physics_client):
        """Test robot initialization."""
        assert robot.base_position == [0, 0, 0]
        assert robot.base_orientation == [0, 0, 0, 1]
        mock_physics_client.loadURDF.assert_called_once()
    
    def test_move_to_joint_positions(self, robot, mock_physics_client):
        """Test moving to joint positions."""
        target_positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        robot.move_to_joint_positions(target_positions, duration=1.0)
        
        # Verify PyBullet calls
        mock_physics_client.setJointMotorControlArray.assert_called()
        args, kwargs = mock_physics_client.setJointMotorControlArray.call_args
        assert kwargs['targetPositions'] == target_positions
        
    def test_get_joint_positions(self, robot, mock_physics_client):
        """Test getting current joint positions."""
        # Setup mock return values for getJointState
        mock_joint_states = [
            ([0.1], [0.0]),  # pos, vel for joint 0
            ([0.2], [0.0]),  # pos, vel for joint 1
            ([0.3], [0.0]),  # pos, vel for joint 2
            ([0.4], [0.0]),  # pos, vel for joint 3
            ([0.5], [0.0]),  # pos, vel for joint 4
            ([0.6], [0.0]),  # pos, vel for joint 5
        ]
        mock_physics_client.getJointState.side_effect = mock_joint_states
        
        positions = robot.get_joint_positions()
        assert positions == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        
    def test_move_to_pose(self, robot, mock_physics_client):
        """Test moving to a target pose."""
        target_position = [0.5, 0.2, 0.3]
        target_orientation = [0, 0, 0, 1]
        
        # Mock IK solution
        mock_ik_solution = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        mock_physics_client.calculateInverseKinematics.return_value = mock_ik_solution
        
        robot.move_to_pose(
            target_position=target_position,
            target_orientation=target_orientation,
            duration=1.0
        )
        
        # Verify IK was called with correct parameters
        mock_physics_client.calculateInverseKinematics.assert_called_once()
        
        # Verify move_to_joint_positions was called with IK solution
        mock_physics_client.setJointMotorControlArray.assert_called()
        
    def test_attach_gripper(self, robot, mock_physics_client):
        """Test attaching a gripper to the robot."""
        mock_gripper = MagicMock()
        mock_gripper.attach_to_robot.return_value = 1  # gripper_id
        
        robot.attach_gripper(mock_gripper)
        
        # Verify gripper's attach_to_robot was called
        mock_gripper.attach_to_robot.assert_called_once_with(
            robot.robot_id, robot.ee_link_index
        )
        
    def test_detach_gripper(self, robot, mock_physics_client):
        """Test detaching a gripper from the robot."""
        # First attach a gripper
        mock_gripper = MagicMock()
        robot.attach_gripper(mock_gripper)
        
        # Now detach it
        robot.detach_gripper()
        
        # Verify gripper's detach method was called
        mock_gripper.detach.assert_called_once()
        
    def test_set_joint_velocities(self, robot, mock_physics_client):
        """Test setting joint velocities."""
        velocities = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        robot.set_joint_velocities(velocities)
        
        # Verify PyBullet call
        mock_physics_client.setJointMotorControlArray.assert_called_once()
        args, kwargs = mock_physics_client.setJointMotorControlArray.call_args
        assert kwargs['targetVelocities'] == velocities
        
    def test_get_end_effector_pose(self, robot, mock_physics_client):
        """Test getting end effector pose."""
        # Setup mock return values
        mock_position = [0.5, 0.2, 0.3]
        mock_orientation = [0, 0, 0, 1]
        mock_physics_client.getLinkState.return_value = (
            mock_position, mock_orientation, None, None, None, None, None, None
        )
        
        position, orientation = robot.get_end_effector_pose()
        
        assert position == mock_position
        assert orientation == mock_orientation
        mock_physics_client.getLinkState.assert_called_once_with(
            robot.robot_id, robot.ee_link_index
        )
        
    def test_apply_joint_torques(self, robot, mock_physics_client):
        """Test applying joint torques."""
        torques = [1.0, 0.5, 0.3, 0.2, 0.1, 0.0]
        robot.apply_joint_torques(torques)
        
        # Verify PyBullet call
        mock_physics_client.setJointMotorControlArray.assert_called_once()
        args, kwargs = mock_physics_client.setJointMotorControlArray.call_args
        assert kwargs['forces'] == torques
    
    def test_get_joint_positions(self, robot, mock_physics_client):
        """Test getting current joint positions."""
        positions = robot.get_joint_positions()
        assert len(positions) == 6  # Should match num_joints
        mock_physics_client.getJointState.assert_called()
    
    @pytest.mark.parametrize("position,orientation,expected_calls", [
        ([0.5, 0, 0.5], [0, 0, 0, 1], 1),  # Valid position
        ([10, 10, 10], [0, 0, 0, 1], 1),    # Potentially unreachable
    ])
    def test_move_to_pose(self, robot, mock_physics_client, position, orientation, expected_calls):
        """Test moving to a cartesian pose."""
        robot.move_to_pose(position, orientation, duration=1.0)
        
        # Verify IK was called
        assert mock_physics_client.calculateInverseKinematics.call_count == expected_calls
    
    def test_gripper_operations(self, robot):
        """Test gripper operations."""
        # Test with no gripper attached
        robot.gripper = None
        with pytest.raises(AttributeError):
            robot.gripper.open()
        
        # Test with mock gripper
        mock_gripper = MagicMock()
        robot.gripper = mock_gripper
        
        robot.gripper.open()
        mock_gripper.open.assert_called_once()
        
        robot.gripper.close()
        mock_gripper.close.assert_called_once()
    
    def test_safety_limits(self, robot, mock_physics_client):
        """Test joint limit enforcement."""
        # Set up joint limits
        robot.joint_limits = [(-math.pi, math.pi)] * 6
        
        # Test within limits
        valid_positions = [0.1] * 6
        robot._enforce_joint_limits(valid_positions)
        
        # Test exceeding limits
        invalid_positions = [10.0] * 6
        with pytest.raises(ValueError, match="Joint limit exceeded"):
            robot._enforce_joint_limits(invalid_positions)
    
    def test_kinematics(self, robot, mock_physics_client):
        """Test forward and inverse kinematics."""
        # Test forward kinematics
        joint_positions = [0.1] * 6
        pose = robot.forward_kinematics(joint_positions)
        assert len(pose[0]) == 3  # Position
        assert len(pose[1]) == 4  # Orientation (quaternion)
        
        # Test inverse kinematics
        position = [0.5, 0, 0.5]
        orientation = [0, 0, 0, 1]
        joint_pos = robot.inverse_kinematics(position, orientation)
        assert len(joint_pos) == 6
