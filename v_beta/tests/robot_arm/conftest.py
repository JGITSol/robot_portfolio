"""Test configuration for robot_arm tests."""
import pytest
import numpy as np
from unittest.mock import MagicMock, patch

@pytest.fixture(scope="module")
def mock_pybullet():
    """Mock the PyBullet module for testing."""
    with patch('pybullet') as mock_pb:
        # Mock commonly used PyBullet functions
        mock_pb.getQuaternionFromEuler.return_value = [0, 0, 0, 1]
        mock_pb.getEulerFromQuaternion.return_value = [0, 0, 0]
        mock_pb.JOINT_FIXED = 4
        mock_pb.GEOM_BOX = 2
        mock_pb.GEOM_SPHERE = 3
        mock_pb.GEOM_CAPSULE = 4
        mock_pb.GEOM_CYLINDER = 5
        mock_pb.GEOM_MESH = 6
        mock_pb.URDF_USE_SELF_COLLISION = 1
        
        # Create a mock for the physics client
        mock_client = MagicMock()
        mock_pb.connect.return_value = 0
        mock_pb.DIRECT = 1
        mock_pb.GUI = 2
        
        # Mock the physics client methods
        mock_client.getNumJoints.return_value = 0
        mock_client.getJointInfo.return_value = None
        mock_client.createMultiBody.return_value = 1
        mock_client.createConstraint.return_value = 42
        mock_client.getQuaternionFromEuler.return_value = [0, 0, 0, 1]
        mock_client.addUserDebugText.return_value = 0
        
        yield mock_pb

@pytest.fixture
def mock_physics_client(mock_pybullet):
    """Return a mocked PyBullet physics client."""
    return mock_pybullet
