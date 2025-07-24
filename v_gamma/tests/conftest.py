"""Pytest configuration and fixtures."""

import pytest
from unittest.mock import Mock, patch
import pybullet as p

from robotics_suite.core.engine import SimulationEngine, SimulationConfig
from robotics_suite.scenarios.manager import ScenarioManager


@pytest.fixture
def mock_pybullet():
    """Mock PyBullet for testing without GUI."""
    with patch('pybullet.connect') as mock_connect, \
         patch('pybullet.setGravity') as mock_gravity, \
         patch('pybullet.setTimeStep') as mock_timestep, \
         patch('pybullet.setRealTimeSimulation') as mock_realtime, \
         patch('pybullet.stepSimulation') as mock_step, \
         patch('pybullet.disconnect') as mock_disconnect, \
         patch('pybullet.resetSimulation') as mock_reset:
        
        mock_connect.return_value = 1  # Mock physics client ID (non-zero)
        
        yield {
            'connect': mock_connect,
            'setGravity': mock_gravity,
            'setTimeStep': mock_timestep,
            'setRealTimeSimulation': mock_realtime,
            'stepSimulation': mock_step,
            'disconnect': mock_disconnect,
            'resetSimulation': mock_reset,
        }


@pytest.fixture
def simulation_config():
    """Create a test simulation configuration."""
    return SimulationConfig(
        gui_enabled=False,
        real_time=False,
        debug_mode=True
    )


@pytest.fixture
def simulation_engine(mock_pybullet, simulation_config):
    """Create a test simulation engine."""
    engine = SimulationEngine(simulation_config)
    engine.initialize()
    yield engine
    engine.cleanup()


@pytest.fixture
def scenario_manager(simulation_engine):
    """Create a test scenario manager."""
    return ScenarioManager(simulation_engine)