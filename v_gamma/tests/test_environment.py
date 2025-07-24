"""Tests for environment implementation."""

import pytest
from unittest.mock import Mock, patch, MagicMock
from pathlib import Path

from robotics_suite.core.environment import Environment, EnvironmentConfig, ObjectConfig


class TestObjectConfig:
    """Test cases for ObjectConfig."""
    
    def test_object_config_creation(self):
        """Test object configuration creation."""
        config = ObjectConfig(
            name="test_box",
            type="box",
            position=[1, 2, 3],
            orientation=[0, 0, 0, 1],
            scale=[0.5, 0.5, 0.5],
            color=[1, 0, 0, 1],
            mass=2.0,
            friction=0.8,
            mesh_path="test.obj"
        )
        
        assert config.name == "test_box"
        assert config.type == "box"
        assert config.position == [1, 2, 3]
        assert config.orientation == [0, 0, 0, 1]
        assert config.scale == [0.5, 0.5, 0.5]
        assert config.color == [1, 0, 0, 1]
        assert config.mass == 2.0
        assert config.friction == 0.8
        assert config.mesh_path == "test.obj"
        
    def test_object_config_defaults(self):
        """Test object configuration with defaults."""
        config = ObjectConfig(
            name="test_obj",
            type="sphere",
            position=[0, 0, 0]
        )
        
        assert config.orientation == [0, 0, 0, 1]
        assert config.scale == [1, 1, 1]
        assert config.color == [0.7, 0.7, 0.7, 1.0]
        assert config.mass == 1.0
        assert config.friction == 0.5
        assert config.mesh_path is None


class TestEnvironmentConfig:
    """Test cases for EnvironmentConfig."""
    
    def test_environment_config_creation(self):
        """Test environment configuration creation."""
        obj_config = ObjectConfig(name="obj1", type="box", position=[0, 0, 0])
        
        config = EnvironmentConfig(
            name="test_env",
            description="Test environment",
            objects=[obj_config],
            lighting={"ambient": 0.5},
            camera_position=[5, 5, 5],
            camera_target=[1, 1, 1]
        )
        
        assert config.name == "test_env"
        assert config.description == "Test environment"
        assert len(config.objects) == 1
        assert config.lighting == {"ambient": 0.5}
        assert config.camera_position == [5, 5, 5]
        assert config.camera_target == [1, 1, 1]
        
    def test_environment_config_defaults(self):
        """Test environment configuration with defaults."""
        config = EnvironmentConfig(
            name="test_env",
            description="Test environment"
        )
        
        assert config.objects == []
        assert config.lighting == {}
        assert config.camera_position == [2, 2, 2]
        assert config.camera_target == [0, 0, 0]


class TestEnvironment:
    """Test cases for Environment."""
    
    def test_environment_initialization(self):
        """Test environment initialization."""
        config = EnvironmentConfig(
            name="test_env",
            description="Test environment"
        )
        
        env = Environment(config)
        
        assert env.config == config
        assert env.physics_client is None
        assert env.object_ids == {}
        assert not env._is_loaded
        
    @patch('pybullet.loadURDF')
    @patch('pybullet.resetDebugVisualizerCamera')
    def test_load_into_simulation_empty(self, mock_camera, mock_load_urdf):
        """Test loading empty environment."""
        config = EnvironmentConfig(
            name="test_env",
            description="Test environment"
        )
        
        env = Environment(config)
        mock_load_urdf.return_value = 0  # Ground plane ID
        
        env.load_into_simulation(physics_client=1)
        
        assert env.physics_client == 1
        assert env._is_loaded
        assert "ground" in env.object_ids
        assert env.object_ids["ground"] == 0
        
        mock_load_urdf.assert_called_once_with("plane.urdf", physicsClientId=1)
        mock_camera.assert_called_once()
        
    @patch('pybullet.loadURDF')
    @patch('pybullet.createCollisionShape')
    @patch('pybullet.createVisualShape')
    @patch('pybullet.createMultiBody')
    @patch('pybullet.changeDynamics')
    @patch('pybullet.resetDebugVisualizerCamera')
    def test_load_into_simulation_with_objects(self, mock_camera, mock_dynamics, 
                                             mock_multibody, mock_visual, mock_collision, mock_load_urdf):
        """Test loading environment with objects."""
        obj_config = ObjectConfig(
            name="test_box",
            type="box",
            position=[1, 2, 3],
            scale=[0.5, 0.5, 0.5]
        )
        
        config = EnvironmentConfig(
            name="test_env",
            description="Test environment",
            objects=[obj_config]
        )
        
        env = Environment(config)
        
        # Setup mocks
        mock_load_urdf.return_value = 0  # Ground plane
        mock_collision.return_value = 1
        mock_visual.return_value = 2
        mock_multibody.return_value = 3
        
        env.load_into_simulation(physics_client=1)
        
        assert env._is_loaded
        assert "ground" in env.object_ids
        assert "test_box" in env.object_ids
        assert env.object_ids["test_box"] == 3
        
        mock_collision.assert_called_once()
        mock_visual.assert_called_once()
        mock_multibody.assert_called_once()
        mock_dynamics.assert_called_once()
        
    @patch('pybullet.createCollisionShape')
    @patch('pybullet.createVisualShape')
    @patch('pybullet.createMultiBody')
    @patch('pybullet.changeDynamics')
    def test_create_object_box(self, mock_dynamics, mock_multibody, mock_visual, mock_collision):
        """Test creating box object."""
        config = EnvironmentConfig(name="test", description="test")
        env = Environment(config)
        env.physics_client = 1
        
        obj_config = ObjectConfig(
            name="test_box",
            type="box",
            position=[1, 2, 3],
            scale=[0.5, 0.5, 0.5]
        )
        
        mock_collision.return_value = 1
        mock_visual.return_value = 2
        mock_multibody.return_value = 3
        
        obj_id = env._create_object(obj_config)
        
        assert obj_id == 3
        mock_collision.assert_called_once()
        mock_visual.assert_called_once()
        mock_multibody.assert_called_once()
        mock_dynamics.assert_called_once()
        
    @patch('pybullet.createCollisionShape')
    @patch('pybullet.createVisualShape')
    @patch('pybullet.createMultiBody')
    @patch('pybullet.changeDynamics')
    def test_create_object_sphere(self, mock_dynamics, mock_multibody, mock_visual, mock_collision):
        """Test creating sphere object."""
        config = EnvironmentConfig(name="test", description="test")
        env = Environment(config)
        env.physics_client = 1
        
        obj_config = ObjectConfig(
            name="test_sphere",
            type="sphere",
            position=[1, 2, 3],
            scale=[0.5, 0.5, 0.5]
        )
        
        mock_collision.return_value = 1
        mock_visual.return_value = 2
        mock_multibody.return_value = 3
        
        obj_id = env._create_object(obj_config)
        
        assert obj_id == 3
        
    @patch('pybullet.createCollisionShape')
    @patch('pybullet.createVisualShape')
    @patch('pybullet.createMultiBody')
    @patch('pybullet.changeDynamics')
    def test_create_object_cylinder(self, mock_dynamics, mock_multibody, mock_visual, mock_collision):
        """Test creating cylinder object."""
        config = EnvironmentConfig(name="test", description="test")
        env = Environment(config)
        env.physics_client = 1
        
        obj_config = ObjectConfig(
            name="test_cylinder",
            type="cylinder",
            position=[1, 2, 3],
            scale=[0.5, 1.0, 0.5]  # radius, height, radius
        )
        
        mock_collision.return_value = 1
        mock_visual.return_value = 2
        mock_multibody.return_value = 3
        
        obj_id = env._create_object(obj_config)
        
        assert obj_id == 3
        
    @patch('pybullet.createCollisionShape')
    @patch('pybullet.createVisualShape')
    @patch('pybullet.createMultiBody')
    @patch('pybullet.changeDynamics')
    @patch('pathlib.Path.exists')
    def test_create_object_mesh(self, mock_exists, mock_dynamics, mock_multibody, mock_visual, mock_collision):
        """Test creating mesh object."""
        config = EnvironmentConfig(name="test", description="test")
        env = Environment(config)
        env.physics_client = 1
        
        obj_config = ObjectConfig(
            name="test_mesh",
            type="mesh",
            position=[1, 2, 3],
            mesh_path="test.obj"
        )
        
        mock_exists.return_value = True
        mock_collision.return_value = 1
        mock_visual.return_value = 2
        mock_multibody.return_value = 3
        
        obj_id = env._create_object(obj_config)
        
        assert obj_id == 3
        
    @patch('pathlib.Path.exists')
    def test_create_object_mesh_not_found(self, mock_exists):
        """Test creating mesh object with missing file."""
        config = EnvironmentConfig(name="test", description="test")
        env = Environment(config)
        env.physics_client = 1
        
        obj_config = ObjectConfig(
            name="test_mesh",
            type="mesh",
            position=[1, 2, 3],
            mesh_path="missing.obj"
        )
        
        mock_exists.return_value = False
        
        with pytest.raises(FileNotFoundError):
            env._create_object(obj_config)
            
    def test_create_object_unsupported_type(self):
        """Test creating object with unsupported type."""
        config = EnvironmentConfig(name="test", description="test")
        env = Environment(config)
        env.physics_client = 1
        
        obj_config = ObjectConfig(
            name="test_obj",
            type="unsupported",
            position=[1, 2, 3]
        )
        
        with pytest.raises(ValueError, match="Unsupported object type"):
            env._create_object(obj_config)
            
    @patch('pybullet.getBasePositionAndOrientation')
    def test_get_object_pose_success(self, mock_get_pose):
        """Test getting object pose."""
        config = EnvironmentConfig(name="test", description="test")
        env = Environment(config)
        env.physics_client = 1
        env.object_ids = {"test_obj": 5}
        
        mock_get_pose.return_value = ([1, 2, 3], [0, 0, 0, 1])
        
        position, orientation = env.get_object_pose("test_obj")
        
        assert position == [1, 2, 3]
        assert orientation == [0, 0, 0, 1]
        mock_get_pose.assert_called_once_with(5, physicsClientId=1)
        
    def test_get_object_pose_not_found(self):
        """Test getting pose of non-existent object."""
        config = EnvironmentConfig(name="test", description="test")
        env = Environment(config)
        
        with pytest.raises(ValueError, match="Object 'missing' not found"):
            env.get_object_pose("missing")
            
    @patch('pybullet.resetBasePositionAndOrientation')
    def test_set_object_pose_with_orientation(self, mock_reset_pose):
        """Test setting object pose with orientation."""
        config = EnvironmentConfig(name="test", description="test")
        env = Environment(config)
        env.physics_client = 1
        env.object_ids = {"test_obj": 5}
        
        env.set_object_pose("test_obj", [1, 2, 3], [0, 0, 0, 1])
        
        mock_reset_pose.assert_called_once_with(5, [1, 2, 3], [0, 0, 0, 1], physicsClientId=1)
        
    @patch('pybullet.resetBasePositionAndOrientation')
    def test_set_object_pose_without_orientation(self, mock_reset_pose):
        """Test setting object pose without orientation."""
        config = EnvironmentConfig(name="test", description="test")
        env = Environment(config)
        env.physics_client = 1
        env.object_ids = {"test_obj": 5}
        
        env.set_object_pose("test_obj", [1, 2, 3])
        
        mock_reset_pose.assert_called_once_with(5, [1, 2, 3], [0, 0, 0, 1], physicsClientId=1)
        
    def test_set_object_pose_not_found(self):
        """Test setting pose of non-existent object."""
        config = EnvironmentConfig(name="test", description="test")
        env = Environment(config)
        
        with pytest.raises(ValueError, match="Object 'missing' not found"):
            env.set_object_pose("missing", [1, 2, 3])
            
    def test_reset_not_loaded(self):
        """Test reset when environment not loaded."""
        config = EnvironmentConfig(name="test", description="test")
        env = Environment(config)
        
        # Should not raise exception
        env.reset()
        
    def test_reset_loaded(self):
        """Test reset when environment is loaded."""
        obj_config = ObjectConfig(
            name="test_obj",
            type="box",
            position=[1, 2, 3],
            orientation=[0, 0, 0, 1]
        )
        
        config = EnvironmentConfig(
            name="test",
            description="test",
            objects=[obj_config]
        )
        
        env = Environment(config)
        env._is_loaded = True
        env.object_ids = {"test_obj": 5}
        
        with patch.object(env, 'set_object_pose') as mock_set_pose:
            env.reset()
            mock_set_pose.assert_called_once_with("test_obj", [1, 2, 3], [0, 0, 0, 1])
            
    def test_get_metrics(self):
        """Test getting environment metrics."""
        config = EnvironmentConfig(name="test_env", description="test")
        env = Environment(config)
        env._is_loaded = True
        env.object_ids = {"obj1": 1, "obj2": 2}
        
        metrics = env.get_metrics()
        
        assert metrics["name"] == "test_env"
        assert metrics["object_count"] == 2
        assert metrics["is_loaded"] is True