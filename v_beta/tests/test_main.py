"""Tests for the main application."""
import pytest
from unittest.mock import MagicMock, patch

class TestMainApplication:
    """Test cases for the main application."""
    
    @pytest.fixture
    def mock_imports(self):
        """Mock imports for testing the main application."""
        with patch('pybullet') as mock_pb, \
             patch('numpy') as mock_np, \
             patch('robot_arm.robot.RobotArm') as mock_robot, \
             patch('robot_arm.grippers.create_gripper') as mock_gripper:
            
            # Configure mocks
            mock_pb.getQuaternionFromEuler.return_value = [0, 0, 0, 1]
            mock_pb.getEulerFromQuaternion.return_value = [0, 0, 0]
            mock_pb.getNumJoints.return_value = 6
            
            # Mock robot and gripper
            mock_robot.return_value.get_joint_positions.return_value = [0.1] * 6
            mock_robot.return_value.get_joint_velocities.return_value = [0.0] * 6
            mock_robot.return_value.get_end_effector_pose.return_value = ([0.5, 0, 0.5], [0, 0, 0, 1])
            
            mock_gripper.return_value.is_closed.return_value = False
            
            yield {
                'pybullet': mock_pb,
                'numpy': mock_np,
                'RobotArm': mock_robot,
                'create_gripper': mock_gripper
            }
    
    def test_main_menu_selection(self, mock_imports):
        """Test main menu selection handling."""
        from robot_arm.main import main
        
        # Mock user input
        with patch('builtins.input', side_effect=['1', '0']), \
             patch('robot_arm.main.move_to_rest_position') as mock_rest:
            
            # Mock exit condition
            with pytest.raises(SystemExit):
                main()
            
            # Verify rest position was called for option 1
            mock_rest.assert_called_once()
    
    def test_move_to_rest_position(self, mock_imports):
        """Test moving to rest position."""
        from robot_arm.main import move_to_rest_position
        
        # Create a mock robot
        mock_robot = mock_imports['RobotArm'].return_value
        
        # Call the function
        move_to_rest_position(mock_robot)
        
        # Verify robot was commanded to move to rest position
        mock_robot.move_to_joint_positions.assert_called_once()
        args, _ = mock_robot.move_to_joint_positions.call_args
        assert len(args[0]) == 6  # 6 joint positions
    
    def test_move_to_example_position(self, mock_imports):
        """Test moving to example position."""
        from robot_arm.main import move_to_example_position
        
        # Create a mock robot
        mock_robot = mock_imports['RobotArm'].return_value
        
        # Call the function
        move_to_example_position(mock_robot)
        
        # Verify robot was commanded to move to example position
        mock_robot.move_to_pose.assert_called_once()
        
    def test_get_joint_positions(self, mock_imports, capsys):
        """Test getting and displaying joint positions."""
        from robot_arm.main import get_joint_positions
        
        # Create a mock robot
        mock_robot = mock_imports['RobotArm'].return_value
        mock_robot.get_joint_positions.return_value = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        
        # Call the function
        get_joint_positions(mock_robot)
        
        # Verify output
        captured = capsys.readouterr()
        assert "Joint positions:" in captured.out
        assert "0.1" in captured.out  # First joint position
        
    def test_gripper_control(self, mock_imports):
        """Test gripper open/close functionality."""
        from robot_arm.main import open_gripper, close_gripper
        
        # Create a mock robot with a mock gripper
        mock_robot = mock_imports['RobotArm'].return_value
        mock_gripper = MagicMock()
        mock_robot.gripper = mock_gripper
        
        # Test open gripper
        open_gripper(mock_robot)
        mock_gripper.open.assert_called_once()
        
        # Reset mock
        mock_gripper.reset_mock()
        
        # Test close gripper
        close_gripper(mock_robot)
        mock_gripper.close.assert_called_once()
    
    def test_invalid_menu_selection(self, mock_imports, capsys):
        """Test handling of invalid menu selection."""
        from robot_arm.main import main
        
        # Mock user input with invalid selection followed by exit
        with patch('builtins.input', side_effect=['999', '0']):
            # Mock exit condition
            with pytest.raises(SystemExit):
                main()
        
        # Verify error message was printed
        captured = capsys.readouterr()
        assert "Invalid selection" in captured.out
