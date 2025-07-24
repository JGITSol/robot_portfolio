"""Tests for scenario management."""

import pytest
from unittest.mock import Mock, patch

from robotics_suite.scenarios.manager import ScenarioManager
from robotics_suite.scenarios.pick_place import PickPlaceScenario, PickPlaceConfig
from robotics_suite.scenarios.production_line import ProductionLineScenario, ProductionLineConfig
from robotics_suite.scenarios.base import BaseScenario, ScenarioConfig


class MockScenario(BaseScenario):
    """Mock scenario for testing."""
    
    def setup(self):
        self.setup_called = True
        
    def execute(self):
        self.execute_called = True
        
    def cleanup(self):
        self.cleanup_called = True


class TestScenarioManager:
    """Test cases for ScenarioManager."""
    
    def test_register_scenario(self, scenario_manager):
        """Test scenario registration."""
        scenario_manager.register_scenario("mock", MockScenario)
        
        assert "mock" in scenario_manager.scenario_classes
        assert scenario_manager.scenario_classes["mock"] == MockScenario
        
    def test_create_scenario(self, scenario_manager):
        """Test scenario creation."""
        scenario_manager.register_scenario("mock", MockScenario)
        
        config = ScenarioConfig(name="test_scenario", description="Test")
        scenario = scenario_manager.create_scenario("mock", config)
        
        assert isinstance(scenario, MockScenario)
        assert scenario.config.name == "test_scenario"
        assert "test_scenario" in scenario_manager.scenarios
        
    def test_start_scenario(self, scenario_manager):
        """Test scenario startup."""
        scenario_manager.register_scenario("mock", MockScenario)
        
        config = ScenarioConfig(name="test_scenario", description="Test")
        scenario = scenario_manager.create_scenario("mock", config)
        
        scenario_manager.start_scenario("test_scenario")
        
        assert scenario_manager.current_scenario == scenario
        assert scenario.is_running
        assert hasattr(scenario, 'setup_called')
        
    def test_stop_scenario(self, scenario_manager):
        """Test scenario stopping."""
        scenario_manager.register_scenario("mock", MockScenario)
        
        config = ScenarioConfig(name="test_scenario", description="Test")
        scenario = scenario_manager.create_scenario("mock", config)
        
        scenario_manager.start_scenario("test_scenario")
        scenario_manager.stop_current_scenario()
        
        assert not scenario.is_running
        assert hasattr(scenario, 'cleanup_called')
        
    def test_list_scenarios(self, scenario_manager):
        """Test scenario listing."""
        scenario_manager.register_scenario("mock", MockScenario)
        
        config1 = ScenarioConfig(name="scenario1", description="Test 1")
        config2 = ScenarioConfig(name="scenario2", description="Test 2")
        
        scenario_manager.create_scenario("mock", config1)
        scenario_manager.create_scenario("mock", config2)
        
        scenarios = scenario_manager.list_scenarios()
        
        assert "scenario1" in scenarios
        assert "scenario2" in scenarios
        assert len(scenarios) == 2
        
    def test_get_metrics(self, scenario_manager):
        """Test metrics collection."""
        scenario_manager.register_scenario("mock", MockScenario)
        
        config = ScenarioConfig(name="test_scenario", description="Test")
        scenario = scenario_manager.create_scenario("mock", config)
        
        metrics = scenario_manager.get_scenario_metrics("test_scenario")
        
        assert "name" in metrics
        assert "is_running" in metrics
        assert metrics["name"] == "test_scenario"
        
    def test_get_metrics_current_scenario(self, scenario_manager):
        """Test getting metrics for current scenario."""
        scenario_manager.register_scenario("mock", MockScenario)
        
        config = ScenarioConfig(name="test_scenario", description="Test")
        scenario = scenario_manager.create_scenario("mock", config)
        scenario_manager.start_scenario("test_scenario")
        
        # Get metrics without specifying scenario name
        metrics = scenario_manager.get_scenario_metrics()
        
        assert "name" in metrics
        assert metrics["name"] == "test_scenario"
        
    def test_get_metrics_no_current_scenario(self, scenario_manager):
        """Test getting metrics when no current scenario."""
        metrics = scenario_manager.get_scenario_metrics()
        assert metrics == {}
        
    def test_get_metrics_scenario_not_found(self, scenario_manager):
        """Test getting metrics for non-existent scenario."""
        with pytest.raises(ValueError, match="Scenario not found"):
            scenario_manager.get_scenario_metrics("non_existent")
            
    def test_get_all_metrics(self, scenario_manager):
        """Test getting all scenario metrics."""
        scenario_manager.register_scenario("mock", MockScenario)
        
        config1 = ScenarioConfig(name="scenario1", description="Test 1")
        config2 = ScenarioConfig(name="scenario2", description="Test 2")
        
        scenario_manager.create_scenario("mock", config1)
        scenario_manager.create_scenario("mock", config2)
        
        all_metrics = scenario_manager.get_all_metrics()
        
        assert "scenario1" in all_metrics
        assert "scenario2" in all_metrics
        assert len(all_metrics) == 2
        
    def test_cleanup(self, scenario_manager):
        """Test scenario manager cleanup."""
        scenario_manager.register_scenario("mock", MockScenario)
        
        config = ScenarioConfig(name="test_scenario", description="Test")
        scenario = scenario_manager.create_scenario("mock", config)
        scenario_manager.start_scenario("test_scenario")
        
        scenario_manager.cleanup()
        
        assert scenario_manager.current_scenario is None
        assert len(scenario_manager.scenarios) == 0
        
    def test_load_scenario_from_config_not_implemented(self, scenario_manager):
        """Test loading scenario from config file (not implemented)."""
        # Use a simple approach - create a file in the current directory
        import os
        test_file = "test_config.yaml"
        
        try:
            # Create test file
            with open(test_file, 'w') as f:
                f.write("test: config")
                
            with pytest.raises(NotImplementedError):
                scenario_manager.load_scenario_from_config(test_file)
        finally:
            # Clean up
            if os.path.exists(test_file):
                try:
                    os.remove(test_file)
                except (OSError, PermissionError):
                    pass  # Ignore cleanup errors
            
    def test_load_scenario_from_config_file_not_found(self, scenario_manager):
        """Test loading scenario from non-existent config file."""
        with pytest.raises(FileNotFoundError):
            scenario_manager.load_scenario_from_config("non_existent.yaml")
            
    def test_register_scenario_invalid_class(self, scenario_manager):
        """Test registering invalid scenario class."""
        class InvalidScenario:
            pass
            
        with pytest.raises(ValueError, match="must inherit from BaseScenario"):
            scenario_manager.register_scenario("invalid", InvalidScenario)
            
    def test_create_scenario_unknown_type(self, scenario_manager):
        """Test creating scenario with unknown type."""
        config = ScenarioConfig(name="test", description="test")
        
        with pytest.raises(ValueError, match="Unknown scenario type"):
            scenario_manager.create_scenario("unknown_type", config)
            
    def test_start_scenario_not_found(self, scenario_manager):
        """Test starting non-existent scenario."""
        with pytest.raises(ValueError, match="Scenario not found"):
            scenario_manager.start_scenario("non_existent")
            
    def test_builtin_scenarios_registered(self, scenario_manager):
        """Test that built-in scenarios are registered."""
        scenario_types = scenario_manager.list_scenario_types()
        
        assert "pick_place" in scenario_types
        assert "production_line" in scenario_types
        
    def test_builtin_scenarios_import_failure(self, simulation_engine):
        """Test scenario manager with import failure."""
        with patch('robotics_suite.scenarios.manager.logger') as mock_logger:
            with patch.dict('sys.modules', {'robotics_suite.scenarios.pick_place': None}):
                # This should trigger the import error handling
                manager = ScenarioManager(simulation_engine)
                # Should still work, just with warning logged


class TestPickPlaceScenario:
    """Test cases for PickPlaceScenario."""
    
    @patch('robotics_suite.scenarios.pick_place.RobotArm')
    @patch('robotics_suite.scenarios.pick_place.Environment')
    def test_setup(self, mock_env_class, mock_robot_class, simulation_engine):
        """Test pick and place setup."""
        config = PickPlaceConfig(
            name="test_pick_place",
            description="Test scenario",
            pick_positions=[[0.5, 0.3, 0.1]],
            place_positions=[[0.5, -0.3, 0.1]]
        )
        
        scenario = PickPlaceScenario(config)
        scenario.set_engine(simulation_engine)
        
        # Mock the robot and environment instances
        mock_robot = Mock()
        mock_robot_class.return_value = mock_robot
        
        mock_environment = Mock()
        mock_env_class.return_value = mock_environment
        
        scenario.setup()
        
        assert scenario.robot == mock_robot
        assert scenario.environment == mock_environment
        assert scenario.metrics["setup_complete"]
        assert scenario.metrics["objects_count"] == 1
        assert scenario.metrics["setup_complete"]
        assert scenario.metrics["objects_count"] == 1
        
    def test_metrics_update(self, simulation_engine):
        """Test metrics updating."""
        config = PickPlaceConfig(
            name="test_pick_place",
            description="Test scenario"
        )
        
        scenario = PickPlaceScenario(config)
        scenario.set_engine(simulation_engine)
        
        scenario.update_metrics({"test_metric": 42})
        
        metrics = scenario.get_metrics()
        assert "test_metric" in metrics
        assert metrics["test_metric"] == 42