"""Tests for demo functionality."""
import pytest
from unittest.mock import MagicMock, patch

class TestDemos:
    """Test cases for demo functionality."""
    
    @pytest.fixture
    def mock_imports(self):
        """Mock imports for testing demos."""
        with patch('pybullet') as mock_pb, \
             patch('numpy') as mock_np, \
             patch('robot_arm.robot.RobotArm') as mock_robot, \
             patch('robot_arm.grippers.create_gripper') as mock_gripper, \
             patch('robot_arm.production_line.ProductionLine') as mock_production_line:
            
            # Configure mocks
            mock_pb.getQuaternionFromEuler.return_value = [0, 0, 0, 1]
            mock_pb.getEulerFromQuaternion.return_value = [0, 0, 0]
            mock_pb.getNumJoints.return_value = 6
            
            # Mock robot and gripper
            mock_robot.return_value.get_joint_positions.return_value = [0.1] * 6
            mock_robot.return_value.get_joint_velocities.return_value = [0.0] * 6
            mock_robot.return_value.get_end_effector_pose.return_value = ([0.5, 0, 0.5], [0, 0, 0, 1])
            
            # Mock gripper
            mock_gripper.return_value.is_closed.return_value = False
            
            yield {
                'pybullet': mock_pb,
                'numpy': mock_np,
                'RobotArm': mock_robot,
                'create_gripper': mock_gripper,
                'ProductionLine': mock_production_line
            }
    
    def test_demo_setup(self, mock_imports):
        """Test basic demo setup."""
        from robot_arm.demo import setup_demo_environment
        
        # Call the function
        robot, gripper = setup_demo_environment()
        
        # Verify robot and gripper were created
        assert robot is not None
        assert gripper is not None
        
        # Verify PyBullet was initialized
        mock_imports['pybullet'].connect.assert_called_once()
        
    def test_demo_pick_and_place(self, mock_imports):
        """Test pick and place demo."""
        from robot_arm.demo import run_pick_and_place_demo
        
        # Setup mock robot and gripper
        mock_robot = mock_imports['RobotArm'].return_value
        mock_gripper = mock_imports['create_gripper'].return_value
        
        # Run the demo
        run_pick_and_place_demo(mock_robot, mock_gripper, num_cycles=1)
        
        # Verify robot was commanded to move
        assert mock_robot.move_to_pose.call_count >= 2  # At least pick and place
        
        # Verify gripper was opened and closed
        mock_gripper.open.assert_called_once()
        mock_gripper.close.assert_called_once()
    
    def test_demo_conveyor(self, mock_imports):
        """Test conveyor demo."""
        from robot_arm.demo_conveyor import run_conveyor_demo
        
        # Setup mock production line
        mock_pl = mock_imports['ProductionLine'].return_value
        
        # Mock user input to exit after first cycle
        with patch('builtins.input', return_value='q'):
            run_conveyor_demo()
        
        # Verify production line was created and updated
        mock_pl.add_conveyor.assert_called()
        mock_pl.run_simulation_step.assert_called()
    
    def test_demo_pendant(self, mock_imports):
        """Test pendant control demo."""
        from robot_arm.demo_pendant import PendantControl
        
        # Create a mock robot
        mock_robot = mock_imports['RobotArm'].return_value
        
        # Create pendant control
        pendant = PendantControl(mock_robot)
        
        # Test joint movement
        with patch('robot_arm.demo_pendant.get_key', side_effect=['1', 'q']):
            pendant.run()
        
        # Verify joint position was updated
        mock_robot.move_to_joint_positions.assert_called()
    
    @patch('time.sleep')  # Patch sleep to speed up tests
    def test_demo_ur5(self, mock_sleep, mock_imports):
        """Test UR5 demo."""
        from robot_arm.ur5_robot import UR5Robot
        
        # Create a mock UR5 robot
        mock_ur5 = MagicMock(spec=UR5Robot)
        mock_ur5.get_joint_positions.return_value = [0.1] * 6
        
        # Mock the UR5Robot class to return our mock
        with patch('robot_arm.ur5_robot.UR5Robot', return_value=mock_ur5):
            from robot_arm.demo import run_ur5_demo
            
            # Run the demo with minimal cycles
            run_ur5_demo(num_cycles=1)
            
            # Verify robot was commanded to move
            assert mock_ur5.move_to_pose.call_count >= 1
