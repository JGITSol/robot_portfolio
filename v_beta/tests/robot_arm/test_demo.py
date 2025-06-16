"""Tests for the robot arm demo script."""
import pytest
import os
import sys
from unittest.mock import MagicMock, patch, ANY
import pybullet as p
from pathlib import Path

# Import the demo module
import robot_arm.demo as demo

class TestDemoFunctions:
    """Test cases for demo module functions."""
    
    @pytest.fixture
    def mock_pybullet(self):
        """Create a mock PyBullet module."""
        with patch('robot_arm.demo.p') as mock_pb:
            # Mock common PyBullet constants
            mock_pb.GUI = 1
            mock_pb.DIRECT = 2
            mock_pb.URDF_USE_SELF_COLLISION = 1
            mock_pb.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS = 2
            mock_pb.URDF_USE_INERTIA_FROM_FILE = 4
            
            # Mock PyBullet functions
            mock_pb.getQuaternionFromEuler.return_value = [0, 0, 0, 1]
            mock_pb.getNumJoints.return_value = 10
            
            # Mock joint info
            mock_joint_info = [
                (i, f'joint_{i}'.encode('utf-8'), 0, -1, -1, 0, [0], [0], [0], 
                 [0], [0], [0, 0, 0], [0, 0, 0]) 
                for i in range(10)
            ]
            mock_pb.getJointInfo.side_effect = mock_joint_info
            
            yield mock_pb
    
    def test_setup_environment(self, mock_pybullet):
        """Test environment setup function."""
        # Call with GUI
        client_id = demo.setup_environment(mock_pybullet, gui=True)
        
        # Verify PyBullet was initialized correctly
        mock_pybullet.connect.assert_called_with(mock_pybullet.GUI)
        mock_pybullet.setGravity.assert_called_with(0, 0, -9.81)
        mock_pybullet.setPhysicsEngineParameter.assert_called()
        mock_pybullet.setRealTimeSimulation.assert_called_with(0)
    
    @patch('robot_arm.demo.p')
    def test_create_work_objects(self, mock_pb):
        """Test creation of work objects."""
        # Setup mock return values
        mock_pb.createCollisionShape.return_value = 1
        mock_pb.createMultiBody.return_value = 1
        
        # Call the function
        demo.create_work_objects(mock_pb)
        
        # Verify objects were created
        assert mock_pb.createMultiBody.call_count >= 3  # At least 3 objects
    
    @patch('robot_arm.demo.setup_environment')
    @patch('robot_arm.demo.create_work_objects')
    @patch('robot_arm.ur5_robot.UR5Robot')
    @patch('robot_arm.robot_config.get_robot_configs')
    def test_main(self, mock_get_configs, mock_ur5_robot, mock_create_objects, mock_setup_env, mock_pybullet):
        """Test the main function."""
        # Setup mocks
        mock_setup_env.return_value = 1
        
        # Mock robot configs
        mock_configs = {
            'ur5_2f85': {'position': [0, 0, 0], 'gripper_type': 'parallel'},
            'ur5_2f140': {'position': [1, 0, 0], 'gripper_type': 'parallel'},
            'ur5_epick': {'position': [2, 0, 0], 'gripper_type': 'suction'}
        }
        mock_get_configs.return_value = mock_configs
        
        # Mock UR5Robot instance
        mock_robot = MagicMock()
        mock_ur5_robot.return_value = mock_robot
        
        # Mock stepSimulation to run only once
        def mock_step_simulation():
            mock_pybullet.getKeyboardEvents.side_effect = [
                {ord('q'): 1}  # Simulate 'q' key press to quit
            ]
        
        mock_pybullet.stepSimulation.side_effect = mock_step_simulation
        
        # Call the main function
        demo.main()
        
        # Verify robots were created
        assert mock_ur5_robot.call_count == 3
        
        # Verify simulation steps were performed
        mock_pybullet.stepSimulation.assert_called()
        
        # Verify cleanup
        mock_pybullet.disconnect.assert_called_once()


class TestDemoScript:
    """Test the demo script can be imported and run."""
    
    def test_import_demo(self):
        """Test that the demo module can be imported."""
        import importlib
        import robot_arm.demo
        importlib.reload(robot_arm.demo)
        assert hasattr(robot_arm.demo, 'main')
    
    @patch('robot_arm.demo.p')
    @patch('robot_arm.demo.setup_environment')
    @patch('robot_arm.demo.create_work_objects')
    @patch('robot_arm.ur5_robot.UR5Robot')
    @patch('robot_arm.robot_config.get_robot_configs')
    def test_demo_run(self, mock_get_configs, mock_ur5_robot, mock_create_objects, mock_setup_env, mock_pb):
        """Test running the demo with early exit."""
        # Setup mocks
        mock_setup_env.return_value = 1
        mock_get_configs.return_value = {}
        
        # Mock keyboard event to exit immediately
        mock_pb.getKeyboardEvents.return_value = {ord('q'): 1}
        
        # Import and run the demo
        import robot_arm.demo as demo_module
        demo_module.main()
        
        # Verify cleanup
        mock_pb.disconnect.assert_called_once()
