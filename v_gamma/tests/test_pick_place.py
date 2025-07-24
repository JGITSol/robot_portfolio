"""Tests for pick and place scenario."""

import pytest
import time
import numpy as np
from unittest.mock import Mock, patch, MagicMock

from robotics_suite.scenarios.pick_place import PickPlaceScenario, PickPlaceConfig
from robotics_suite.scenarios.base import ScenarioConfig


class TestPickPlaceConfig:
    """Test cases for PickPlaceConfig."""
    
    def test_pick_place_config_creation(self):
        """Test pick place configuration creation."""
        config = PickPlaceConfig(
            name="test_pick_place",
            description="Test scenario",
            robot_urdf="custom.urdf",
            pick_positions=[[1, 2, 3], [4, 5, 6]],
            place_positions=[[7, 8, 9], [10, 11, 12]],
            approach_height=0.3,
            grip_delay=2.0
        )
        
        assert config.name == "test_pick_place"
        assert config.description == "Test scenario"
        assert config.robot_urdf == "custom.urdf"
        assert config.pick_positions == [[1, 2, 3], [4, 5, 6]]
        assert config.place_positions == [[7, 8, 9], [10, 11, 12]]
        assert config.approach_height == 0.3
        assert config.grip_delay == 2.0
        
    def test_pick_place_config_defaults(self):
        """Test pick place configuration with defaults."""
        config = PickPlaceConfig(
            name="test",
            description="test"
        )
        
        assert config.robot_urdf == "models/ur5/ur5.urdf"
        assert config.pick_positions == [[0.5, 0.3, 0.1]]
        assert config.place_positions == [[0.5, -0.3, 0.1]]
        assert config.approach_height == 0.2
        assert config.grip_delay == 1.0


class TestPickPlaceScenario:
    """Test cases for PickPlaceScenario."""
    
    def test_initialization(self):
        """Test scenario initialization."""
        config = PickPlaceConfig(
            name="test_scenario",
            description="Test pick and place"
        )
        
        scenario = PickPlaceScenario(config)
        
        assert scenario.config == config
        assert scenario.robot is None
        assert scenario.environment is None
        assert scenario.current_step == 0
        assert scenario.total_cycles == 0
        
    @patch('robotics_suite.scenarios.pick_place.RobotArm')
    @patch('robotics_suite.scenarios.pick_place.Environment')
    def test_setup_success(self, mock_env_class, mock_robot_class):
        """Test successful scenario setup."""
        config = PickPlaceConfig(
            name="test_scenario",
            description="Test pick and place",
            pick_positions=[[0.5, 0.3, 0.1], [0.3, 0.5, 0.1]],
            place_positions=[[0.5, -0.3, 0.1]]
        )
        
        scenario = PickPlaceScenario(config)
        
        # Mock engine
        mock_engine = Mock()
        scenario.set_engine(mock_engine)
        
        # Mock robot and environment instances
        mock_robot = Mock()
        mock_robot_class.return_value = mock_robot
        
        mock_environment = Mock()
        mock_env_class.return_value = mock_environment
        
        scenario.setup()
        
        assert scenario.robot == mock_robot
        assert scenario.environment == mock_environment
        
        mock_engine.add_robot.assert_called_once_with("main_robot", mock_robot)
        mock_engine.set_environment.assert_called_once_with(mock_environment)
        
        # Check metrics
        assert scenario.metrics["setup_complete"] is True
        assert scenario.metrics["objects_count"] == 2  # Two pick positions
        
    @patch('robotics_suite.scenarios.pick_place.time.sleep')
    def test_execute_success(self, mock_sleep):
        """Test successful scenario execution."""
        config = PickPlaceConfig(
            name="test_scenario",
            description="Test pick and place",
            pick_positions=[[0.5, 0.3, 0.1]],
            place_positions=[[0.5, -0.3, 0.1]]
        )
        
        scenario = PickPlaceScenario(config)
        scenario.is_running = True
        
        with patch.object(scenario, '_execute_pick_place_cycle', return_value=True) as mock_cycle:
            scenario.execute()
            
            mock_cycle.assert_called_once_with([0.5, 0.3, 0.1], [0.5, -0.3, 0.1])
            assert scenario.total_cycles == 1
            assert scenario.metrics["total_cycles"] == 1
            assert scenario.metrics["success_rate"] == 1.0
            
    @patch('robotics_suite.scenarios.pick_place.time.sleep')
    def test_execute_with_failure(self, mock_sleep):
        """Test scenario execution with failure."""
        config = PickPlaceConfig(
            name="test_scenario",
            description="Test pick and place",
            pick_positions=[[0.5, 0.3, 0.1]],
            place_positions=[[0.5, -0.3, 0.1]]
        )
        
        scenario = PickPlaceScenario(config)
        scenario.is_running = True
        
        with patch.object(scenario, '_execute_pick_place_cycle', return_value=False) as mock_cycle:
            scenario.execute()
            
            mock_cycle.assert_called_once()
            assert scenario.total_cycles == 0
            
    @patch('robotics_suite.scenarios.pick_place.time.sleep')
    def test_execute_stopped(self, mock_sleep):
        """Test scenario execution when stopped."""
        config = PickPlaceConfig(
            name="test_scenario",
            description="Test pick and place",
            pick_positions=[[0.5, 0.3, 0.1]],
            place_positions=[[0.5, -0.3, 0.1]]
        )
        
        scenario = PickPlaceScenario(config)
        scenario.is_running = False  # Stopped
        
        with patch.object(scenario, '_execute_pick_place_cycle') as mock_cycle:
            scenario.execute()
            
            mock_cycle.assert_not_called()
            
    def test_execute_with_exception(self):
        """Test scenario execution with exception."""
        config = PickPlaceConfig(
            name="test_scenario",
            description="Test pick and place"
        )
        
        scenario = PickPlaceScenario(config)
        scenario.is_running = True
        
        with patch.object(scenario, '_execute_pick_place_cycle', side_effect=Exception("Test error")):
            scenario.execute()
            
            assert "error" in scenario.metrics
            assert scenario.metrics["error"] == "Test error"
            
    @patch('robotics_suite.scenarios.pick_place.time.sleep')
    def test_execute_pick_place_cycle_success(self, mock_sleep):
        """Test successful pick and place cycle."""
        config = PickPlaceConfig(
            name="test_scenario",
            description="Test pick and place",
            approach_height=0.2,
            grip_delay=0.1
        )
        
        scenario = PickPlaceScenario(config)
        scenario.is_running = True
        
        # Mock robot
        mock_robot = Mock()
        mock_robot.move_to_pose.return_value = True
        scenario.robot = mock_robot
        
        with patch.object(scenario, '_wait_for_movement') as mock_wait:
            result = scenario._execute_pick_place_cycle([0.5, 0.3, 0.1], [0.5, -0.3, 0.1])
            
            assert result is True
            assert mock_robot.move_to_pose.call_count == 6  # 6 movements in cycle
            assert mock_wait.call_count == 6  # 6 waits in cycle (updated count)
            assert scenario.current_step == 0  # Reset at end
            
    def test_execute_pick_place_cycle_robot_failure(self):
        """Test pick and place cycle with robot failure."""
        config = PickPlaceConfig(
            name="test_scenario",
            description="Test pick and place"
        )
        
        scenario = PickPlaceScenario(config)
        scenario.is_running = True
        
        # Mock robot that fails
        mock_robot = Mock()
        mock_robot.move_to_pose.return_value = False
        scenario.robot = mock_robot
        
        result = scenario._execute_pick_place_cycle([0.5, 0.3, 0.1], [0.5, -0.3, 0.1])
        
        assert result is False
        
    def test_execute_pick_place_cycle_stopped(self):
        """Test pick and place cycle when stopped."""
        config = PickPlaceConfig(
            name="test_scenario",
            description="Test pick and place"
        )
        
        scenario = PickPlaceScenario(config)
        scenario.is_running = False  # Set to stopped before calling
        
        mock_robot = Mock()
        mock_robot.move_to_pose.return_value = True
        scenario.robot = mock_robot
        
        result = scenario._execute_pick_place_cycle([0.5, 0.3, 0.1], [0.5, -0.3, 0.1])
        
        assert result is False
        
    def test_execute_pick_place_cycle_exception(self):
        """Test pick and place cycle with exception."""
        config = PickPlaceConfig(
            name="test_scenario",
            description="Test pick and place"
        )
        
        scenario = PickPlaceScenario(config)
        scenario.is_running = True
        
        # Mock robot that raises exception
        mock_robot = Mock()
        mock_robot.move_to_pose.side_effect = Exception("Robot error")
        scenario.robot = mock_robot
        
        result = scenario._execute_pick_place_cycle([0.5, 0.3, 0.1], [0.5, -0.3, 0.1])
        
        assert result is False
        
    @patch('robotics_suite.scenarios.pick_place.time.time')
    @patch('robotics_suite.scenarios.pick_place.time.sleep')
    def test_wait_for_movement_success(self, mock_sleep, mock_time):
        """Test successful wait for movement."""
        config = PickPlaceConfig(name="test", description="test")
        scenario = PickPlaceScenario(config)
        scenario.is_running = True
        
        # Mock robot with converged positions
        mock_robot = Mock()
        mock_robot.get_joint_positions.return_value = np.array([0.1, 0.2, 0.3])
        mock_robot.target_joint_positions = np.array([0.1, 0.2, 0.3])  # Same as current
        scenario.robot = mock_robot
        
        # Mock time progression
        mock_time.side_effect = [0, 0.1]  # Start and end times
        
        scenario._wait_for_movement(timeout=5.0)
        
        mock_robot.get_joint_positions.assert_called()
        
    @patch('robotics_suite.scenarios.pick_place.time.time')
    @patch('robotics_suite.scenarios.pick_place.time.sleep')
    def test_wait_for_movement_timeout(self, mock_sleep, mock_time):
        """Test wait for movement with timeout."""
        config = PickPlaceConfig(name="test", description="test")
        scenario = PickPlaceScenario(config)
        scenario.is_running = True
        
        # Mock robot with non-converged positions
        mock_robot = Mock()
        mock_robot.get_joint_positions.return_value = np.array([0.1, 0.2, 0.3])
        mock_robot.target_joint_positions = np.array([1.0, 2.0, 3.0])  # Different
        scenario.robot = mock_robot
        
        # Mock time progression to exceed timeout
        mock_time.side_effect = [0, 6.0]  # Exceeds 5.0 timeout
        
        scenario._wait_for_movement(timeout=5.0)
        
        # Should exit due to timeout
        
    @patch('robotics_suite.scenarios.pick_place.time.time')
    @patch('robotics_suite.scenarios.pick_place.time.sleep')
    def test_wait_for_movement_stopped(self, mock_sleep, mock_time):
        """Test wait for movement when stopped."""
        config = PickPlaceConfig(name="test", description="test")
        scenario = PickPlaceScenario(config)
        scenario.is_running = False  # Stopped
        
        mock_robot = Mock()
        scenario.robot = mock_robot
        
        mock_time.side_effect = [0, 0.1]
        
        scenario._wait_for_movement(timeout=5.0)
        
        # Should exit immediately due to not running
        
    @patch('robotics_suite.scenarios.pick_place.time.time')
    @patch('robotics_suite.scenarios.pick_place.time.sleep')
    def test_wait_for_movement_empty_positions(self, mock_sleep, mock_time):
        """Test wait for movement with empty positions."""
        config = PickPlaceConfig(name="test", description="test")
        scenario = PickPlaceScenario(config)
        scenario.is_running = True
        
        # Mock robot with empty positions
        mock_robot = Mock()
        mock_robot.get_joint_positions.return_value = np.array([])
        mock_robot.target_joint_positions = np.array([])
        scenario.robot = mock_robot
        
        mock_time.side_effect = [0, 6.0]  # Exceeds timeout
        
        scenario._wait_for_movement(timeout=5.0)
        
        # Should exit due to timeout (empty arrays)
        
    def test_cleanup(self):
        """Test scenario cleanup."""
        config = PickPlaceConfig(name="test", description="test")
        scenario = PickPlaceScenario(config)
        
        scenario.current_step = 5
        scenario.total_cycles = 3
        
        scenario.cleanup()
        
        assert scenario.current_step == 0
        assert scenario.metrics["cleanup_complete"] is True
        assert scenario.metrics["final_cycles"] == 3