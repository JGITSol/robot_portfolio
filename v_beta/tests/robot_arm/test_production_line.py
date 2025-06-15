"""Tests for the production_line module."""
import pytest
import numpy as np
from unittest.mock import MagicMock, patch, call

class TestProductionLine:
    """Test cases for the ProductionLine class."""
    
    @pytest.fixture
    def mock_physics_client(self):
        """Create a mock PyBullet physics client."""
        with patch('pybullet') as mock_pb:
            mock_pb.getQuaternionFromEuler.return_value = [0, 0, 0, 1]
            mock_pb.getEulerFromQuaternion.return_value = [0, 0, 0]
            yield mock_pb
    
    @pytest.fixture
    def mock_robot(self):
        """Create a mock robot."""
        robot = MagicMock()
        robot.get_joint_positions.return_value = [0.1] * 6
        robot.get_joint_velocities.return_value = [0.0] * 6
        robot.get_end_effector_pose.return_value = ([0.5, 0, 0.5], [0, 0, 0, 1])
        return robot
    
    @pytest.fixture
    def production_line(self, mock_physics_client, mock_robot):
        """Create a ProductionLine instance for testing."""
        with patch('robot_arm.robot.RobotArm', return_value=mock_robot):
            from robot_arm.production_line import ProductionLine
            
            line = ProductionLine(
                physics_client=mock_physics_client,
                table_position=[0, 0, 0],
                table_orientation=[0, 0, 0, 1]
            )
            
            # Add a robot to the production line
            line.add_robot(
                robot_type="ur5",
                position=[0, 0.5, 0],
                gripper_type="parallel"
            )
            
            return line
    
    def test_add_robot(self, production_line, mock_robot):
        """Test adding a robot to the production line."""
        assert len(production_line.robots) == 1
        assert production_line.robots[0] == mock_robot
    
    def test_add_conveyor(self, production_line, mock_physics_client):
        """Test adding a conveyor to the production line."""
        production_line.add_conveyor(
            start_pos=[0, 0, 0],
            end_pos=[1, 0, 0],
            width=0.5,
            speed=0.1
        )
        
        assert len(production_line.conveyors) == 1
        mock_physics_client.loadURDF.assert_called()
    
    def test_spawn_object(self, production_line, mock_physics_client):
        """Test spawning an object on the production line."""
        obj = production_line.spawn_object(
            obj_type="box",
            position=[0.5, 0, 0.5],
            size=[0.1, 0.1, 0.1],
            mass=1.0
        )
        
        assert obj is not None
        assert len(production_line.objects) == 1
        mock_physics_client.loadURDF.assert_called()
    
    def test_update_objects_on_conveyor(self, production_line, mock_physics_client):
        """Test updating objects on the conveyor."""
        # Add a conveyor and an object on it
        production_line.add_conveyor(
            start_pos=[0, 0, 0],
            end_pos=[1, 0, 0],
            width=0.5,
            speed=0.1
        )
        
        obj = MagicMock()
        obj.position = np.array([0.5, 0, 0.5])
        obj.conveyor_id = 0
        production_line.objects = [obj]
        
        # Mock the conveyor's update method
        production_line.conveyors[0].update = MagicMock()
        
        production_line._update_objects_on_conveyor(dt=0.1)
        
        # Verify the conveyor's update method was called
        production_line.conveyors[0].update.assert_called_once_with([obj], 0.1)
    
    def test_pick_object(self, production_line, mock_robot):
        """Test picking up an object with a robot."""
        # Add an object to the scene
        obj = MagicMock()
        obj.position = np.array([0.5, 0, 0.5])
        production_line.objects = [obj]
        
        # Test successful pick
        mock_robot.gripper.is_closed.return_value = True
        result = production_line.pick_object(robot_idx=0, obj_idx=0)
        
        assert result is True
        mock_robot.move_to_pose.assert_called()
        mock_robot.gripper.close.assert_called_once()
    
    def test_place_object(self, production_line, mock_robot):
        """Test placing an object with a robot."""
        # Add an object being held by the robot
        obj = MagicMock()
        obj.position = np.array([0.5, 0, 0.5])
        obj.held_by = 0
        production_line.objects = [obj]
        
        # Configure the robot to have a gripper with an object
        mock_robot.gripper.is_closed.return_value = True
        mock_robot.gripper.has_object.return_value = True
        
        # Test successful place
        target_position = [0.8, 0, 0.5]
        result = production_line.place_object(robot_idx=0, position=target_position)
        
        assert result is True
        mock_robot.move_to_pose.assert_called()
        mock_robot.gripper.open.assert_called_once()
    
    def test_run_simulation_step(self, production_line, mock_physics_client):
        """Test running a single simulation step."""
        # Add a conveyor and an object
        production_line.add_conveyor(
            start_pos=[0, 0, 0],
            end_pos=[1, 0, 0],
            width=0.5,
            speed=0.1
        )
        
        obj = MagicMock()
        obj.position = np.array([0.5, 0, 0.5])
        obj.conveyor_id = 0
        production_line.objects = [obj]
        
        # Run a simulation step
        production_line.run_simulation_step(dt=0.1)
        
        # Verify physics step was called
        mock_physics_client.stepSimulation.assert_called_once()
    
    def test_object_detection(self, production_line, mock_physics_client):
        """Test object detection in the scene."""
        # Add an object to the scene
        obj = MagicMock()
        obj.position = np.array([0.5, 0, 0.5])
        production_line.objects = [obj]
        
        # Test detection by position
        detected = production_line.detect_objects(position=[0.5, 0, 0.5], radius=0.1)
        assert len(detected) == 1
        
        # Test detection with no matches
        detected = production_line.detect_objects(position=[2.0, 2.0, 2.0], radius=0.1)
        assert len(detected) == 0
