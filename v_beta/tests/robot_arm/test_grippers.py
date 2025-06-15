"""Tests for the grippers module."""
import pytest
from unittest.mock import MagicMock, patch

class TestGripperBase:
    """Test cases for the GripperBase class."""
    
    @pytest.fixture
    def gripper(self):
        """Create a GripperBase instance for testing."""
        from robot_arm.grippers import GripperBase
        mock_physics = MagicMock()
        return GripperBase(physics_client=mock_physics, gripper_id=1)
    
    def test_abstract_methods(self, gripper):
        """Test that abstract methods raise NotImplementedError."""
        with pytest.raises(NotImplementedError):
            gripper.open()
        
        with pytest.raises(NotImplementedError):
            gripper.close()
        
        with pytest.raises(NotImplementedError):
            gripper.is_closed()


class TestParallelGripper:
    """Test cases for the ParallelGripper class."""
    
    @pytest.fixture
    def gripper(self):
        """Create a ParallelGripper instance for testing."""
        from robot_arm.grippers import ParallelGripper
        mock_physics = MagicMock()
        return ParallelGripper(
            physics_client=mock_physics,
            gripper_id=1,
            left_finger_joint=2,
            right_finger_joint=3,
            max_effort=100.0
        )
    
    def test_open(self, gripper):
        """Test opening the parallel gripper."""
        gripper.open()
        gripper.physics.setJointMotorControl2.assert_any_call(
            gripper.gripper_id,
            gripper.left_finger_joint,
            gripper.physics.POSITION_CONTROL,
            targetPosition=gripper.open_position,
            force=gripper.max_effort
        )
        gripper.physics.setJointMotorControl2.assert_any_call(
            gripper.gripper_id,
            gripper.right_finger_joint,
            gripper.physics.POSITION_CONTROL,
            targetPosition=gripper.open_position,
            force=gripper.max_effort
        )
    
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
        return SuctionGripper(
            physics_client=mock_physics,
            gripper_id=1,
            suction_tip_link=2,
            max_vacuum=100.0
        )
    
    def test_activate(self, gripper):
        """Test activating the suction gripper."""
        gripper.activate()
        gripper.physics.setJointMotorControl2.assert_called_once_with(
            gripper.gripper_id,
            gripper.suction_tip_link,
            gripper.physics.VELOCITY_CONTROL,
            targetVelocity=0,
            force=gripper.max_vacuum
        )
    
    def test_deactivate(self, gripper):
        """Test deactivating the suction gripper."""
        gripper.deactivate()
        gripper.physics.setJointMotorControl2.assert_called_once_with(
            gripper.gripper_id,
            gripper.suction_tip_link,
            gripper.physics.VELOCITY_CONTROL,
            targetVelocity=0,
            force=0
        )
    
    @pytest.mark.parametrize("velocity,expected", [
        (0.0, False),   # Not moving, not sucking
        (1.0, False),   # Moving, but not sucking
        (-1.0, True),   # Sucking
    ])
    def test_is_active(self, gripper, velocity, expected):
        """Test checking if the suction gripper is active."""
        # Mock getJointState to return specific velocity
        gripper.physics.getJointState.return_value = (None, [velocity])
        
        assert gripper.is_active() == expected


def test_create_gripper():
    """Test the create_gripper factory function."""
    from robot_arm.grippers import create_gripper
    
    mock_physics = MagicMock()
    
    # Test creating a parallel gripper
    parallel_gripper = create_gripper(
        physics_client=mock_physics,
        gripper_type="parallel",
        gripper_id=1,
        left_finger_joint=2,
        right_finger_joint=3
    )
    assert parallel_gripper.__class__.__name__ == "ParallelGripper"
    
    # Test creating a suction gripper
    suction_gripper = create_gripper(
        physics_client=mock_physics,
        gripper_type="suction",
        gripper_id=1,
        suction_tip_link=2
    )
    assert suction_gripper.__class__.__name__ == "SuctionGripper"
    
    # Test invalid gripper type
    with pytest.raises(ValueError):
        create_gripper(mock_physics, "invalid_type")
