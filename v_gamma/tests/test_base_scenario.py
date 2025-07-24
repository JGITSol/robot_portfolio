"""Tests for base scenario implementation."""

import pytest
from unittest.mock import Mock

from robotics_suite.scenarios.base import BaseScenario, ScenarioConfig
from robotics_suite.core.engine import SimulationEngine


class TestScenarioConfig:
    """Test cases for ScenarioConfig."""
    
    def test_scenario_config_creation(self):
        """Test scenario configuration creation."""
        config = ScenarioConfig(
            name="test_scenario",
            description="Test scenario description",
            duration=30.0,
            auto_start=False,
            metrics_enabled=False
        )
        
        assert config.name == "test_scenario"
        assert config.description == "Test scenario description"
        assert config.duration == 30.0
        assert config.auto_start is False
        assert config.metrics_enabled is False
        
    def test_scenario_config_defaults(self):
        """Test scenario configuration with defaults."""
        config = ScenarioConfig(
            name="test",
            description="test"
        )
        
        assert config.duration is None
        assert config.auto_start is True
        assert config.metrics_enabled is True


class ConcreteScenario(BaseScenario):
    """Concrete implementation of BaseScenario for testing."""
    
    def __init__(self, config: ScenarioConfig):
        super().__init__(config)
        self.setup_called = False
        self.execute_called = False
        self.cleanup_called = False
        
    def setup(self):
        self.setup_called = True
        
    def execute(self):
        self.execute_called = True
        
    def cleanup(self):
        self.cleanup_called = True


class TestBaseScenario:
    """Test cases for BaseScenario."""
    
    def test_initialization(self):
        """Test base scenario initialization."""
        config = ScenarioConfig(name="test", description="test")
        scenario = ConcreteScenario(config)
        
        assert scenario.config == config
        assert scenario.engine is None
        assert not scenario.is_running
        assert scenario.metrics == {}
        
    def test_set_engine(self):
        """Test setting simulation engine."""
        config = ScenarioConfig(name="test", description="test")
        scenario = ConcreteScenario(config)
        
        mock_engine = Mock(spec=SimulationEngine)
        scenario.set_engine(mock_engine)
        
        assert scenario.engine == mock_engine
        
    def test_start_success(self):
        """Test successful scenario start."""
        config = ScenarioConfig(
            name="test",
            description="test",
            auto_start=True
        )
        scenario = ConcreteScenario(config)
        
        mock_engine = Mock(spec=SimulationEngine)
        scenario.set_engine(mock_engine)
        
        scenario.start()
        
        assert scenario.is_running
        assert scenario.setup_called
        assert scenario.execute_called
        
    def test_start_no_auto_start(self):
        """Test scenario start without auto execution."""
        config = ScenarioConfig(
            name="test",
            description="test",
            auto_start=False
        )
        scenario = ConcreteScenario(config)
        
        mock_engine = Mock(spec=SimulationEngine)
        scenario.set_engine(mock_engine)
        
        scenario.start()
        
        assert scenario.is_running
        assert scenario.setup_called
        assert not scenario.execute_called
        
    def test_start_no_engine(self):
        """Test scenario start without engine."""
        config = ScenarioConfig(name="test", description="test")
        scenario = ConcreteScenario(config)
        
        with pytest.raises(RuntimeError, match="Simulation engine not set"):
            scenario.start()
            
    def test_start_setup_exception(self):
        """Test scenario start with setup exception."""
        config = ScenarioConfig(name="test", description="test")
        scenario = ConcreteScenario(config)
        
        mock_engine = Mock(spec=SimulationEngine)
        scenario.set_engine(mock_engine)
        
        # Make setup raise exception
        def failing_setup():
            raise Exception("Setup failed")
            
        scenario.setup = failing_setup
        
        with pytest.raises(Exception, match="Setup failed"):
            scenario.start()
            
        assert not scenario.is_running
        
    def test_stop(self):
        """Test scenario stop."""
        config = ScenarioConfig(name="test", description="test")
        scenario = ConcreteScenario(config)
        
        mock_engine = Mock(spec=SimulationEngine)
        scenario.set_engine(mock_engine)
        
        scenario.start()
        assert scenario.is_running
        
        scenario.stop()
        
        assert not scenario.is_running
        assert scenario.cleanup_called
        
    def test_get_metrics_with_engine(self):
        """Test getting metrics with engine."""
        config = ScenarioConfig(name="test_scenario", description="test")
        scenario = ConcreteScenario(config)
        
        mock_engine = Mock(spec=SimulationEngine)
        mock_engine.get_metrics.return_value = {"engine_metric": "value"}
        scenario.set_engine(mock_engine)
        
        scenario.update_metrics({"custom_metric": 42})
        
        metrics = scenario.get_metrics()
        
        assert metrics["name"] == "test_scenario"
        assert metrics["is_running"] is False
        assert "config" in metrics
        assert "engine_metrics" in metrics
        assert metrics["engine_metrics"]["engine_metric"] == "value"
        assert metrics["custom_metric"] == 42
        
    def test_get_metrics_without_engine(self):
        """Test getting metrics without engine."""
        config = ScenarioConfig(name="test_scenario", description="test")
        scenario = ConcreteScenario(config)
        
        scenario.update_metrics({"custom_metric": 42})
        
        metrics = scenario.get_metrics()
        
        assert metrics["name"] == "test_scenario"
        assert metrics["is_running"] is False
        assert "config" in metrics
        assert "engine_metrics" not in metrics
        assert metrics["custom_metric"] == 42
        
    def test_update_metrics_enabled(self):
        """Test updating metrics when enabled."""
        config = ScenarioConfig(
            name="test",
            description="test",
            metrics_enabled=True
        )
        scenario = ConcreteScenario(config)
        
        scenario.update_metrics({"test_metric": "value"})
        
        assert scenario.metrics["test_metric"] == "value"
        
    def test_update_metrics_disabled(self):
        """Test updating metrics when disabled."""
        config = ScenarioConfig(
            name="test",
            description="test",
            metrics_enabled=False
        )
        scenario = ConcreteScenario(config)
        
        scenario.update_metrics({"test_metric": "value"})
        
        # Should not update metrics when disabled
        assert "test_metric" not in scenario.metrics
        
    def test_config_in_metrics(self):
        """Test that config is properly included in metrics."""
        config = ScenarioConfig(
            name="test_scenario",
            description="test description",
            duration=30.0,
            auto_start=False,
            metrics_enabled=True
        )
        scenario = ConcreteScenario(config)
        
        metrics = scenario.get_metrics()
        
        config_dict = metrics["config"]
        assert config_dict["name"] == "test_scenario"
        assert config_dict["description"] == "test description"
        assert config_dict["duration"] == 30.0
        assert config_dict["auto_start"] is False
        assert config_dict["metrics_enabled"] is True