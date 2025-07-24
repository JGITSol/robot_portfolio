"""Tests for robot arm implementation."""

import pytest
import numpy as np
from unittest.mock import Mock, patch, MagicMock
from pathlib import Path

from robotics_suite.core.robot import RobotArm, RobotConfig, JointConfig


class TestJointConfig:
    """Test cases for JointConfig."""
    
    def test_joint_config_creation(self):
        """Test joint configuration creation."""
        config = JointConfig(
            name="joint1",
            joint_type="revolute",
            lower_limit=-3.14,
            upper_limit=3.14,
            max_force=1000.0,
            max_velocity=2.0
        )
        
        assert config.name == "joint1"
        assert config.joint_type == "revolute"
        assert config.lower_limit == -3.14
        assert config.upper_limit == 3.14
        assert config.max_force == 1000.0
        assert config.max_velocity == 2.0
        
    def test_joint_config_defaults(self):
        """Test joint configuration with defaults."""
        config = JointConfig(
            name="joint1",
            joint_type="revolute",
            lower_limit=-1.0,
            upper_limit=1.0
        )
        
        assert config.max_force == 500.0
        assert config.max_velocity == 1.0


class TestRobotConfig:
    """Test cases for RobotConfig."""
    
    def test_robot_config_creation(self):
        """Test robot configuration creation."""
        config = RobotConfig(
            urdf_path="test.urdf",
            base_position=[1, 2, 3],
            base_orientation=[0, 0, 0, 1],
            end_effector_link=5
        )
        
        assert config.urdf_path == "test.urdf"
        assert config.base_position == [1, 2, 3]
        assert config.base_orientation == [0, 0, 0, 1]
        assert config.end_effector_link == 5
        
    def test_robot_config_defaults(self):
        """Test robot configuration with defaults."""
        config = RobotConfig(urdf_path="test.urdf")
        
        assert config.base_position == [0, 0, 0]
        assert config.base_orientation == [0, 0, 0, 1]
        assert config.joints == []
        assert config.end_effector_link == -1


class TestRobotArm:
    """Test cases for RobotArm."""
    
    def test_robot_initialization(self):
        """Test robot arm initialization."""
        config = RobotConfig(urdf_path="test.urdf")
        robot = RobotArm(config)
        
        assert robot.config == config
        assert robot.robot_id is None
        assert robot.physics_client is None
        assert robot.joint_indices == []
        assert len(robot.current_joint_positions) == 0
        assert len(robot.target_joint_positions) == 0
        assert not robot._is_loaded
        
    @patch('pybullet.loadURDF')
    @patch('pybullet.getNumJoints')
    @patch('pybullet.getJointInfo')
    @patch('pathlib.Path.exists')
    def test_load_into_simulation_success(self, mock_exists, mock_joint_info, mock_num_joints, mock_load_urdf):
        """Test successful robot loading."""
        # Setup mocks
        mock_exists.return_value = True
        mock_load_urdf.return_value = 1
        mock_num_joints.return_value = 3
        
        # Mock joint info: (name, type, joint_type, ...)
        mock_joint_info.side_effect = [
            ("joint1", None, 0, None, None, None, None, None, None, None, None, None, None, None),  # REVOLUTE
            ("joint2", None, 1, None, None, None, None, None, None, None, None, None, None, None),  # PRISMATIC
            ("joint3", None, 4, None, None, None, None, None, None, None, None, None, None, None),  # FIXED
        ]
        
        config = RobotConfig(urdf_path="test.urdf")
        robot = RobotArm(config)
        
        robot.load_into_simulation(physics_client=0)
        
        assert robot.robot_id == 1
        assert robot.physics_client == 0
        assert robot.joint_indices == [0, 1]  # Only revolute and prismatic
        assert len(robot.current_joint_positions) == 2
        assert len(robot.target_joint_positions) == 2
        assert robot._is_loaded
        
        mock_load_urdf.assert_called_once()
        
    @patch('pathlib.Path.exists')
    def test_load_into_simulation_file_not_found(self, mock_exists):
        """Test robot loading with missing URDF file."""
        mock_exists.return_value = False
        
        config = RobotConfig(urdf_path="missing.urdf")
        robot = RobotArm(config)
        
        with pytest.raises(FileNotFoundError):
            robot.load_into_simulation(physics_client=0)
            
    @patch('pybullet.setJointMotorControl2')
    def test_set_joint_positions_success(self, mock_motor_control):
        """Test setting joint positions."""
        config = RobotConfig(urdf_path="test.urdf")
        robot = RobotArm(config)
        
        # Simulate loaded state
        robot._is_loaded = True
        robot.physics_client = 0
        robot.robot_id = 1
        robot.joint_indices = [0, 1, 2]
        robot.current_joint_positions = np.zeros(3)
        robot.target_joint_positions = np.zeros(3)
        
        positions = [0.5, -0.3, 1.2]
        robot.set_joint_positions(positions)
        
        np.testing.assert_array_equal(robot.target_joint_positions, positions)
        assert mock_motor_control.call_count == 3
        
    def test_set_joint_positions_not_loaded(self):
        """Test setting joint positions when robot not loaded."""
        config = RobotConfig(urdf_path="test.urdf")
        robot = RobotArm(config)
        
        with pytest.raises(RuntimeError, match="Robot not loaded"):
            robot.set_joint_positions([0.5, -0.3, 1.2])
            
    def test_set_joint_positions_wrong_count(self):
        """Test setting joint positions with wrong count."""
        config = RobotConfig(urdf_path="test.urdf")
        robot = RobotArm(config)
        
        # Simulate loaded state
        robot._is_loaded = True
        robot.joint_indices = [0, 1, 2]
        
        with pytest.raises(ValueError, match="Expected 3 positions"):
            robot.set_joint_positions([0.5, -0.3])  # Wrong count
            
    @patch('pybullet.getJointState')
    def test_get_joint_positions_success(self, mock_joint_state):
        """Test getting joint positions."""
        config = RobotConfig(urdf_path="test.urdf")
        robot = RobotArm(config)
        
        # Simulate loaded state
        robot._is_loaded = True
        robot.physics_client = 0
        robot.robot_id = 1
        robot.joint_indices = [0, 1]
        
        # Mock joint states: (position, velocity, ...)
        mock_joint_state.side_effect = [
            (0.5, 0.1, None, None),
            (-0.3, 0.2, None, None)
        ]
        
        positions = robot.get_joint_positions()
        
        np.testing.assert_array_almost_equal(positions, [0.5, -0.3])
        assert mock_joint_state.call_count == 2
        
    def test_get_joint_positions_not_loaded(self):
        """Test getting joint positions when robot not loaded."""
        config = RobotConfig(urdf_path="test.urdf")
        robot = RobotArm(config)
        
        with pytest.raises(RuntimeError, match="Robot not loaded"):
            robot.get_joint_positions()
            
    @patch('pybullet.getLinkState')
    def test_get_end_effector_pose_success(self, mock_link_state):
        """Test getting end effector pose."""
        config = RobotConfig(urdf_path="test.urdf", end_effector_link=5)
        robot = RobotArm(config)
        
        # Simulate loaded state
        robot._is_loaded = True
        robot.physics_client = 0
        robot.robot_id = 1
        
        # Mock link state: (position, orientation, ...)
        mock_link_state.return_value = (
            [1.0, 2.0, 3.0],  # position
            [0.0, 0.0, 0.0, 1.0],  # orientation
            None, None, None, None, None, None
        )
        
        position, orientation = robot.get_end_effector_pose()
        
        np.testing.assert_array_equal(position, [1.0, 2.0, 3.0])
        np.testing.assert_array_equal(orientation, [0.0, 0.0, 0.0, 1.0])
        mock_link_state.assert_called_once_with(1, 5, physicsClientId=0)
        
    def test_get_end_effector_pose_not_loaded(self):
        """Test getting end effector pose when robot not loaded."""
        config = RobotConfig(urdf_path="test.urdf")
        robot = RobotArm(config)
        
        with pytest.raises(RuntimeError, match="Robot not loaded"):
            robot.get_end_effector_pose()
            
    @patch('pybullet.calculateInverseKinematics')
    def test_move_to_pose_position_only(self, mock_ik):
        """Test moving to pose with position only."""
        config = RobotConfig(urdf_path="test.urdf", end_effector_link=5)
        robot = RobotArm(config)
        
        # Simulate loaded state
        robot._is_loaded = True
        robot.physics_client = 0
        robot.robot_id = 1
        robot.joint_indices = [0, 1, 2]
        robot.current_joint_positions = np.zeros(3)
        robot.target_joint_positions = np.zeros(3)
        
        # Mock IK solution
        mock_ik.return_value = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        
        with patch.object(robot, 'set_joint_positions') as mock_set_joints:
            result = robot.move_to_pose([1.0, 2.0, 3.0])
            
            assert result is True
            mock_ik.assert_called_once_with(1, 5, [1.0, 2.0, 3.0], physicsClientId=0)
            mock_set_joints.assert_called_once_with([0.1, 0.2, 0.3])
            
    @patch('pybullet.calculateInverseKinematics')
    def test_move_to_pose_with_orientation(self, mock_ik):
        """Test moving to pose with position and orientation."""
        config = RobotConfig(urdf_path="test.urdf", end_effector_link=5)
        robot = RobotArm(config)
        
        # Simulate loaded state
        robot._is_loaded = True
        robot.physics_client = 0
        robot.robot_id = 1
        robot.joint_indices = [0, 1, 2]
        
        # Mock IK solution
        mock_ik.return_value = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        
        with patch.object(robot, 'set_joint_positions') as mock_set_joints:
            result = robot.move_to_pose([1.0, 2.0, 3.0], [0, 0, 0, 1])
            
            assert result is True
            mock_ik.assert_called_once_with(1, 5, [1.0, 2.0, 3.0], [0, 0, 0, 1], physicsClientId=0)
            
    def test_move_to_pose_not_loaded(self):
        """Test moving to pose when robot not loaded."""
        config = RobotConfig(urdf_path="test.urdf")
        robot = RobotArm(config)
        
        with pytest.raises(RuntimeError, match="Robot not loaded"):
            robot.move_to_pose([1.0, 2.0, 3.0])
            
    @patch('pybullet.calculateInverseKinematics')
    def test_move_to_pose_ik_failure(self, mock_ik):
        """Test moving to pose with IK failure."""
        config = RobotConfig(urdf_path="test.urdf", end_effector_link=5)
        robot = RobotArm(config)
        
        # Simulate loaded state
        robot._is_loaded = True
        robot.physics_client = 0
        robot.robot_id = 1
        robot.joint_indices = [0, 1, 2]
        
        # Mock IK failure
        mock_ik.side_effect = Exception("IK failed")
        
        result = robot.move_to_pose([1.0, 2.0, 3.0])
        assert result is False
        
    def test_update_loaded(self):
        """Test update when robot is loaded."""
        config = RobotConfig(urdf_path="test.urdf")
        robot = RobotArm(config)
        
        # Simulate loaded state
        robot._is_loaded = True
        
        with patch.object(robot, 'get_joint_positions') as mock_get_positions:
            robot.update()
            mock_get_positions.assert_called_once()
            
    def test_update_not_loaded(self):
        """Test update when robot is not loaded."""
        config = RobotConfig(urdf_path="test.urdf")
        robot = RobotArm(config)
        
        # Should not raise exception
        robot.update()
        
    def test_reset_loaded(self):
        """Test reset when robot is loaded."""
        config = RobotConfig(urdf_path="test.urdf")
        robot = RobotArm(config)
        
        # Simulate loaded state
        robot._is_loaded = True
        robot.joint_indices = [0, 1, 2]
        
        with patch.object(robot, 'set_joint_positions') as mock_set_positions:
            robot.reset()
            mock_set_positions.assert_called_once_with([0.0, 0.0, 0.0])
            
    def test_reset_not_loaded(self):
        """Test reset when robot is not loaded."""
        config = RobotConfig(urdf_path="test.urdf")
        robot = RobotArm(config)
        
        # Should not raise exception
        robot.reset()
        
    def test_get_metrics_not_loaded(self):
        """Test getting metrics when robot not loaded."""
        config = RobotConfig(urdf_path="test.urdf")
        robot = RobotArm(config)
        
        metrics = robot.get_metrics()
        assert metrics == {}
        
    def test_get_metrics_loaded(self):
        """Test getting metrics when robot is loaded."""
        config = RobotConfig(urdf_path="test.urdf")
        robot = RobotArm(config)
        
        # Simulate loaded state
        robot._is_loaded = True
        robot.current_joint_positions = np.array([0.1, 0.2, 0.3])
        robot.target_joint_positions = np.array([0.15, 0.25, 0.35])
        
        with patch.object(robot, 'get_joint_positions', return_value=robot.current_joint_positions):
            metrics = robot.get_metrics()
            
            assert "joint_positions" in metrics
            assert "target_positions" in metrics
            assert "position_error" in metrics
            assert "is_loaded" in metrics
            
            assert metrics["is_loaded"] is True
            assert len(metrics["joint_positions"]) == 3
            assert len(metrics["target_positions"]) == 3
            assert metrics["position_error"] > 0