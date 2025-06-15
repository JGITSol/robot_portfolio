"""Shared pytest fixtures and configuration."""
import os
import sys
from pathlib import Path
from typing import Generator

import pytest

# Add the src directory to Python path
src_path = str(Path(__file__).parent.parent / "src")
sys.path.insert(0, src_path)

# Common fixtures

@pytest.fixture(scope="session")
def test_data_dir() -> Path:
    """Return the path to the test data directory."""
    return Path(__file__).parent / "data"

@pytest.fixture(scope="function")
def mock_physics_client(mocker):
    """Create a mock PyBullet physics client."""
    mock_pb = mocker.patch('pybullet', autospec=True)
    
    # Common PyBullet return values
    mock_pb.getQuaternionFromEuler.return_value = [0, 0, 0, 1]
    mock_pb.getEulerFromQuaternion.return_value = [0, 0, 0]
    mock_pb.getNumJoints.return_value = 6  # Typical for UR5
    
    return mock_pb

@pytest.fixture(scope="function")
def mock_ur5_robot(mocker, mock_physics_client):
    """Create a mock UR5 robot instance."""
    from robot_arm.ur5_robot import UR5Robot
    
    robot = UR5Robot(physics_client=mock_physics_client, base_position=[0, 0, 0])
    
    # Mock robot properties
    robot.get_joint_positions = mocker.Mock(return_value=[0.1]*6)  # 6 joints
    robot.get_joint_velocities = mocker.Mock(return_value=[0.0]*6)
    robot.get_end_effector_pose = mocker.Mock(return_value=([0.5, 0, 0.5], [0, 0, 0, 1]))
    
    return robot

# Add markers for test categories
def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "integration: mark test as an integration test (dependencies may be mocked)",
    )
    config.addinivalue_line(
        "markers",
        "e2e: mark test as an end-to-end test (runs with real dependencies)",
    )
