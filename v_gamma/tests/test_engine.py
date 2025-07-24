"""Tests for simulation engine."""

import pytest
from unittest.mock import patch, Mock

from robotics_suite.core.engine import SimulationEngine, SimulationConfig
from robotics_suite.core.robot import RobotArm, RobotConfig


class TestSimulationEngine:
    """Test cases for SimulationEngine."""
    
    def test_initialization(self, mock_pybullet, simulation_config):
        """Test engine initialization."""
        engine = SimulationEngine(simulation_config)
        engine.initialize()
        
        assert engine.physics_client == 1
        assert not engine.is_running
        assert engine._step_count == 0
        
        mock_pybullet['connect'].assert_called_once()
        mock_pybullet['setGravity'].assert_called_once()
        mock_pybullet['setTimeStep'].assert_called_once()
        
        engine.cleanup()
        
    def test_add_robot(self, simulation_engine):
        """Test adding robot to simulation."""
        robot_config = RobotConfig(urdf_path="test.urdf")
        robot = Mock(spec=RobotArm)
        
        simulation_engine.add_robot("test_robot", robot)
        
        assert "test_robot" in simulation_engine.robots
        robot.load_into_simulation.assert_called_once_with(1)
        
    def test_step_simulation(self, simulation_engine, mock_pybullet):
        """Test simulation step."""
        # Add a mock robot
        robot = Mock()
        simulation_engine.robots["test"] = robot
        
        # Ensure physics client is set (from initialization)
        assert simulation_engine.physics_client == 1
        
        simulation_engine.step()
        
        assert simulation_engine._step_count == 1
        robot.update.assert_called_once()
        mock_pybullet['stepSimulation'].assert_called_once()
        
    @patch('pybullet.resetSimulation')
    def test_reset_simulation(self, mock_reset, simulation_engine):
        """Test simulation reset."""
        # Add mock robot and environment
        robot = Mock()
        environment = Mock()
        simulation_engine.robots["test"] = robot
        simulation_engine.environment = environment
        
        simulation_engine.reset()
        
        mock_reset.assert_called_once()
        robot.reset.assert_called_once()
        environment.reset.assert_called_once()
        assert simulation_engine._step_count == 0
            
    def test_get_metrics(self, simulation_engine):
        """Test metrics collection."""
        metrics = simulation_engine.get_metrics()
        
        assert "step_count" in metrics
        assert "robot_count" in metrics
        assert "is_running" in metrics
        assert "config" in metrics
        
        assert metrics["step_count"] == 0
        assert metrics["robot_count"] == 0
        assert not metrics["is_running"]
        
    @patch('pybullet.disconnect')
    def test_context_manager(self, mock_disconnect, mock_pybullet, simulation_config):
        """Test engine as context manager."""
        with SimulationEngine(simulation_config) as engine:
            assert engine.physics_client == 1
            
        mock_disconnect.assert_called_once_with(1)