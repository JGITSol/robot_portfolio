"""Tests for the grippers module."""
import pytest
import numpy as np
from unittest.mock import MagicMock, patch, ANY
from typing import List, Dict, Any, Optional, Tuple

class TestGripper:
    """Test cases for the base Gripper class."""
    
    @pytest.fixture
    def gripper(self):
        """Create a Gripper instance for testing."""
        from robot_arm.grippers import Gripper
        mock_physics = MagicMock()
        return Gripper(physics_client=mock_physics, position=[0, 0, 0])
    
    def test_initialization(self, gripper):
        """Test gripper initialization."""
        assert gripper.position == [0, 0, 0]
        assert gripper.orientation == [0, 0, 0, 1]
        assert gripper.gripper_id is None
        assert gripper.joint_indices == []
        assert gripper.joint_limits == []
        assert gripper.attached_object is None
    
    def test_attach_to_robot(self, gripper):
        """Test attaching gripper to a robot."""
        # Mock the load method
        gripper.load = MagicMock(return_value=42)
        gripper.physics_client.createConstraint.return_value = 123
        
        # Call the method
        result = gripper.attach_to_robot(robot_id=1, link_index=2)
        
        # Verify the result
        assert result == 42
        gripper.load.assert_called_once()
        gripper.physics_client.createConstraint.assert_called_once_with(
            parentBodyUniqueId=1,
            parentLinkIndex=2,
            childBodyUniqueId=42,
            childLinkIndex=-1,
            jointType=ANY,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0],
            parentFrameOrientation=ANY
        )
    
    def test_abstract_methods(self):
        """Test that abstract methods raise NotImplementedError."""
        from robot_arm.grippers import Gripper
        gripper = Gripper(physics_client=MagicMock(), position=[0, 0, 0])
        
        with pytest.raises(NotImplementedError):
            gripper.load()


class TestParallelGripper:
    """Test cases for the ParallelGripper class."""
    
    @pytest.fixture
    def gripper(self):
        """Create a ParallelGripper instance for testing."""
        from robot_arm.grippers import ParallelGripper
        mock_physics = MagicMock()
        gripper = ParallelGripper(
            physics_client=mock_physics,
            position=[0, 0, 0.5]
        )
        gripper.gripper_id = 1  # Simulate loaded gripper
        return gripper
    
    def test_load(self, gripper):
        """Test loading the parallel gripper model."""
        # Mock the URDF loading
        gripper.physics_client.getNumJoints.return_value = 3
        gripper.physics_client.getJointInfo.side_effect = [
            (0, b'finger_joint', 0, -1, -1, 0, [0], [0], [0], [0], [0], [0, 0, 0], [0, 0, 0]),
            (1, b'left_finger_joint', 0, -1, -1, 0, [0], [0], [0], [0], [0], [0, 0, 0], [0, 0, 0]),
            (2, b'right_finger_joint', 0, -1, -1, 0, [0], [0], [0], [0], [0], [0, 0, 0], [0, 0, 0])
        ]
        
        gripper_id = gripper.load()
        
        assert gripper_id == 1
        gripper.physics_client.loadURDF.assert_called_once()
        assert len(gripper.joint_indices) > 0
    
    def test_open(self, gripper):
        """Test opening the parallel gripper."""
        gripper.joint_indices = [1, 2]  # Simulate loaded joints
        gripper.open()
        
        # Verify setJointMotorControl2 was called for each joint
        assert gripper.physics_client.setJointMotorControl2.call_count == 2
        
        # Check the first call (for first joint)
        args, kwargs = gripper.physics_client.setJointMotorControl2.call_args_list[0]
        assert args[0] == gripper.gripper_id
        assert args[1] in gripper.joint_indices
        assert kwargs['controlMode'] == gripper.physics.POSITION_CONTROL
        assert kwargs['targetPosition'] == 0.04  # Open position
        assert kwargs['force'] == 100
        
        # Check the second call (for second joint)
        args, kwargs = gripper.physics_client.setJointMotorControl2.call_args_list[1]
        assert args[0] == gripper.gripper_id
        assert args[1] in gripper.joint_indices
        assert kwargs['controlMode'] == gripper.physics.POSITION_CONTROL
        assert kwargs['targetPosition'] == 0.04  # Open position
        assert kwargs['force'] == 100
    
    def test_close(self, gripper):
        """Test closing the parallel gripper."""
        gripper.close()
        gripper.physics.setJointMotorControl2.assert_any_call(
            gripper.gripper_id,
            gripper.left_finger_joint,
            gripper.physics.POSITION_CONTROL,
            targetPosition=gripper.close_position,
            force=gripper.max_effort
        )
        gripper.physics.setJointMotorControl2.assert_any_call(
            gripper.gripper_id,
            gripper.right_finger_joint,
            gripper.physics.POSITION_CONTROL,
            targetPosition=gripper.close_position,
            force=gripper.max_effort
        )
    
    @pytest.mark.parametrize("left_pos,right_pos,expected", [
        (0.0, 0.0, True),     # Fully closed
        (0.0, 0.1, False),    # Partially open
        (0.1, 0.1, False),    # Fully open
    ])
    def test_is_closed(self, gripper, left_pos, right_pos, expected):
        """Test checking if the gripper is closed."""
        # Mock getJointState to return specific positions
        gripper.physics.getJointState.side_effect = [
            ([left_pos], None),    # Left finger
            ([right_pos], None)    # Right finger
        ]
        
        assert gripper.is_closed() == expected


class TestSuctionGripper:
    """Test cases for the SuctionGripper class."""
    
    @pytest.fixture
    def gripper(self):
        """Create a SuctionGripper instance for testing."""
        from robot_arm.grippers import SuctionGripper
        mock_physics = MagicMock()
        gripper = SuctionGripper(
            physics_client=mock_physics,
            position=[0, 0, 0.5]
        )
        gripper.gripper_id = 1
        return gripper
    
    def test_initialization(self, gripper):
        """Test SuctionGripper initialization."""
        assert gripper.is_active is False
    
    def test_load(self, gripper):
        """Test loading the suction gripper model."""
        # Mock the visual and collision shape creation
        gripper.physics_client.createVisualShape.return_value = 1
        gripper.physics_client.createCollisionShape.return_value = 2
        gripper.physics_client.createMultiBody.return_value = 1
        
        gripper_id = gripper.load()
        
        assert gripper_id == 1
        gripper.physics_client.createVisualShape.assert_called_once()
        gripper.physics_client.createCollisionShape.assert_called_once()
        gripper.physics_client.createMultiBody.assert_called_once()
    
    def test_activate(self, gripper):
        """Test activating the suction gripper."""
        gripper.activate(True)
        assert gripper.is_active is True
        gripper.physics_client.changeVisualShape.assert_called_once()
    
    def test_deactivate(self, gripper):
        """Test deactivating the suction gripper."""
        gripper.activate(False)
        assert gripper.is_active is False
        gripper.physics_client.changeVisualShape.assert_called_once()
    
    def test_open_close_aliases(self, gripper):
        """Test the open/close alias methods."""
        gripper.open()
        assert gripper.is_active is False
        
        gripper.close()
        assert gripper.is_active is True
    
    def test_is_active(self, gripper):
        """Test checking if the suction gripper is active."""
        # Mock getJointState to return specific velocity
        gripper.physics.getJointState.return_value = (None, [0.0])
        
        assert gripper.is_active() is False


def test_create_gripper():
    """Test the create_gripper factory function."""
    mock_physics = MagicMock()
    position = [0.5, 0.2, 0.3]
    
    # Test creating a parallel gripper
    from robot_arm.grippers import create_gripper, ParallelGripper
    parallel_gripper = create_gripper('parallel', mock_physics, position)
    assert isinstance(parallel_gripper, ParallelGripper)
    assert parallel_gripper.position == position
    
    # Test creating a suction gripper
    from robot_arm.grippers import SuctionGripper
    suction_gripper = create_gripper('suction', mock_physics, position)
    assert isinstance(suction_gripper, SuctionGripper)
    assert suction_gripper.position == position
    
    # Test case insensitivity
    suction_gripper2 = create_gripper('SUCTION', mock_physics, position)
    assert isinstance(suction_gripper2, SuctionGripper)
    
    # Test invalid gripper type
    with pytest.raises(ValueError) as excinfo:
        create_gripper('invalid', mock_physics, position)
    assert "Unknown gripper type" in str(excinfo.value)
