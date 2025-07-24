"""Tests for production line scenario."""

import pytest
import time
from unittest.mock import Mock, patch, MagicMock
from enum import Enum

from robotics_suite.scenarios.production_line import (
    ProductionLineScenario, 
    ProductionLineConfig, 
    StationState
)


class TestStationState:
    """Test cases for StationState enum."""
    
    def test_station_state_values(self):
        """Test station state enum values."""
        assert StationState.IDLE.value == "idle"
        assert StationState.PROCESSING.value == "processing"
        assert StationState.WAITING.value == "waiting"
        assert StationState.ERROR.value == "error"


class TestProductionLineConfig:
    """Test cases for ProductionLineConfig."""
    
    def test_production_line_config_creation(self):
        """Test production line configuration creation."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line",
            robot_configs=[{"type": "ur5"}],
            conveyor_speed=0.2,
            station_count=4,
            products_per_cycle=10,
            cycle_time=60.0,
            quality_check_enabled=False
        )
        
        assert config.name == "test_production"
        assert config.description == "Test production line"
        assert config.robot_configs == [{"type": "ur5"}]
        assert config.conveyor_speed == 0.2
        assert config.station_count == 4
        assert config.products_per_cycle == 10
        assert config.cycle_time == 60.0
        assert config.quality_check_enabled is False
        
    def test_production_line_config_defaults(self):
        """Test production line configuration with defaults."""
        config = ProductionLineConfig(
            name="test",
            description="test"
        )
        
        assert config.robot_configs == []
        assert config.conveyor_speed == 0.1
        assert config.station_count == 3
        assert config.products_per_cycle == 5
        assert config.cycle_time == 30.0
        assert config.quality_check_enabled is True


class TestProductionLineScenario:
    """Test cases for ProductionLineScenario."""
    
    def test_initialization(self):
        """Test scenario initialization."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line"
        )
        
        scenario = ProductionLineScenario(config)
        
        assert scenario.config == config
        assert scenario.robots == {}
        assert scenario.environment is None
        assert scenario.station_states == {}
        assert scenario.products_processed == 0
        assert scenario.quality_passed == 0
        assert scenario.cycle_start_time == 0
        
    @patch('robotics_suite.scenarios.production_line.RobotArm')
    @patch('robotics_suite.scenarios.production_line.Environment')
    def test_setup_success(self, mock_env_class, mock_robot_class):
        """Test successful scenario setup."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line",
            station_count=2,
            products_per_cycle=3
        )
        
        scenario = ProductionLineScenario(config)
        
        # Mock engine
        mock_engine = Mock()
        scenario.set_engine(mock_engine)
        
        # Mock robot and environment instances
        mock_robot = Mock()
        mock_robot_class.return_value = mock_robot
        
        mock_environment = Mock()
        mock_env_class.return_value = mock_environment
        
        scenario.setup()
        
        # Check robots created
        assert len(scenario.robots) == 2
        assert "station_0_robot" in scenario.robots
        assert "station_1_robot" in scenario.robots
        
        # Check station states
        assert len(scenario.station_states) == 2
        assert scenario.station_states["station_0"] == StationState.IDLE
        assert scenario.station_states["station_1"] == StationState.IDLE
        
        # Check engine calls
        assert mock_engine.add_robot.call_count == 2
        mock_engine.set_environment.assert_called_once_with(mock_environment)
        
        # Check metrics
        assert scenario.metrics["setup_complete"] is True
        assert scenario.metrics["station_count"] == 2
        assert scenario.metrics["products_total"] == 3
        
    @patch('robotics_suite.scenarios.production_line.time.time')
    @patch('robotics_suite.scenarios.production_line.time.sleep')
    def test_execute_success(self, mock_sleep, mock_time):
        """Test successful scenario execution."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line",
            products_per_cycle=2,
            quality_check_enabled=True
        )
        
        scenario = ProductionLineScenario(config)
        scenario.is_running = True
        scenario.cycle_start_time = 100.0
        
        with patch.object(scenario, '_process_product', return_value=True) as mock_process, \
             patch.object(scenario, '_quality_check', return_value=True) as mock_quality, \
             patch.object(scenario, '_update_production_metrics') as mock_update, \
             patch.object(scenario, '_finalize_production_metrics') as mock_finalize:
            
            mock_time.return_value = 130.0  # 30 seconds later
            
            scenario.execute()
            
            assert mock_process.call_count == 2
            assert mock_quality.call_count == 2
            assert scenario.products_processed == 2
            assert scenario.quality_passed == 2
            mock_finalize.assert_called_once()
            
    @patch('robotics_suite.scenarios.production_line.time.sleep')
    def test_execute_with_failure(self, mock_sleep):
        """Test scenario execution with product failure."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line",
            products_per_cycle=2
        )
        
        scenario = ProductionLineScenario(config)
        scenario.is_running = True
        
        with patch.object(scenario, '_process_product', return_value=False) as mock_process, \
             patch.object(scenario, '_update_production_metrics') as mock_update, \
             patch.object(scenario, '_finalize_production_metrics') as mock_finalize:
            
            scenario.execute()
            
            assert scenario.products_processed == 0  # No successful products
            mock_finalize.assert_called_once()
            
    @patch('robotics_suite.scenarios.production_line.time.sleep')
    def test_execute_stopped(self, mock_sleep):
        """Test scenario execution when stopped."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line",
            products_per_cycle=2
        )
        
        scenario = ProductionLineScenario(config)
        scenario.is_running = False  # Stopped
        
        with patch.object(scenario, '_process_product') as mock_process:
            scenario.execute()
            
            mock_process.assert_not_called()
            
    def test_execute_with_exception(self):
        """Test scenario execution with exception."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line"
        )
        
        scenario = ProductionLineScenario(config)
        scenario.is_running = True
        
        with patch.object(scenario, '_process_product', side_effect=Exception("Test error")):
            scenario.execute()
            
            assert "error" in scenario.metrics
            assert scenario.metrics["error"] == "Test error"
            
    @patch('robotics_suite.scenarios.production_line.time.sleep')
    def test_process_product_success(self, mock_sleep):
        """Test successful product processing."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line",
            station_count=2
        )
        
        scenario = ProductionLineScenario(config)
        scenario.is_running = True
        scenario.station_states = {
            "station_0": StationState.IDLE,
            "station_1": StationState.IDLE
        }
        
        with patch.object(scenario, '_operate_station', return_value=True) as mock_operate, \
             patch.object(scenario, '_move_product_on_conveyor') as mock_move:
            
            result = scenario._process_product(0)
            
            assert result is True
            assert mock_operate.call_count == 2
            assert mock_move.call_count == 2
            
            # Check station states reset to idle
            assert scenario.station_states["station_0"] == StationState.IDLE
            assert scenario.station_states["station_1"] == StationState.IDLE
            
    def test_process_product_station_failure(self):
        """Test product processing with station failure."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line",
            station_count=2
        )
        
        scenario = ProductionLineScenario(config)
        scenario.is_running = True
        scenario.station_states = {
            "station_0": StationState.IDLE,
            "station_1": StationState.IDLE
        }
        
        with patch.object(scenario, '_operate_station', return_value=False) as mock_operate:
            
            result = scenario._process_product(0)
            
            assert result is False
            assert scenario.station_states["station_0"] == StationState.ERROR
            
    def test_process_product_stopped(self):
        """Test product processing when stopped."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line",
            station_count=1
        )
        
        scenario = ProductionLineScenario(config)
        scenario.is_running = False  # Stopped
        
        result = scenario._process_product(0)
        
        assert result is False
        
    def test_process_product_exception(self):
        """Test product processing with exception."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line",
            station_count=1
        )
        
        scenario = ProductionLineScenario(config)
        scenario.is_running = True
        scenario.station_states = {"station_0": StationState.IDLE}
        
        with patch.object(scenario, '_operate_station', side_effect=Exception("Station error")):
            
            result = scenario._process_product(0)
            
            assert result is False
            
    @patch('robotics_suite.scenarios.production_line.time.sleep')
    def test_operate_station_success(self, mock_sleep):
        """Test successful station operation."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line"
        )
        
        scenario = ProductionLineScenario(config)
        scenario.is_running = True
        
        # Mock robot
        mock_robot = Mock()
        mock_robot.move_to_pose.return_value = True
        scenario.robots = {"station_0_robot": mock_robot}
        
        result = scenario._operate_station("station_0_robot", "product_0", 0)
        
        assert result is True
        assert mock_robot.move_to_pose.call_count >= 3  # Multiple movements
        
    def test_operate_station_different_stations(self):
        """Test station operations for different station types."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line"
        )
        
        scenario = ProductionLineScenario(config)
        scenario.is_running = True
        
        # Mock robot
        mock_robot = Mock()
        mock_robot.move_to_pose.return_value = True
        scenario.robots = {"station_robot": mock_robot}
        
        # Test different station IDs (0, 1, 2+)
        for station_id in [0, 1, 2]:
            result = scenario._operate_station("station_robot", "product", station_id)
            assert result is True
            
    def test_operate_station_exception(self):
        """Test station operation with exception."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line"
        )
        
        scenario = ProductionLineScenario(config)
        scenario.is_running = True
        
        # Mock robot that raises exception
        mock_robot = Mock()
        mock_robot.move_to_pose.side_effect = Exception("Robot error")
        scenario.robots = {"station_0_robot": mock_robot}
        
        result = scenario._operate_station("station_0_robot", "product_0", 0)
        
        assert result is False
        
    def test_move_product_on_conveyor_success(self):
        """Test successful product movement on conveyor."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line"
        )
        
        scenario = ProductionLineScenario(config)
        
        # Mock environment
        mock_environment = Mock()
        scenario.environment = mock_environment
        
        scenario._move_product_on_conveyor("product_0", 1)
        
        mock_environment.set_object_pose.assert_called_once_with(
            "product_0", 
            [4.0, 0, 0.1]  # (1+1) * 2.0 = 4.0
        )
        
    def test_move_product_on_conveyor_exception(self):
        """Test product movement with exception."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line"
        )
        
        scenario = ProductionLineScenario(config)
        
        # Mock environment that raises exception
        mock_environment = Mock()
        mock_environment.set_object_pose.side_effect = Exception("Move error")
        scenario.environment = mock_environment
        
        # Should not raise exception (warning logged)
        scenario._move_product_on_conveyor("product_0", 1)
        
    def test_quality_check_pass(self):
        """Test quality check pass."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line"
        )
        
        scenario = ProductionLineScenario(config)
        
        # Test that the method can return True (run multiple times to ensure it can pass)
        results = [scenario._quality_check(0) for _ in range(50)]
        # With 90% pass rate, at least one should pass
        assert any(results), "Quality check should pass at least once with 90% pass rate"
        
    def test_quality_check_fail(self):
        """Test quality check fail."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line"
        )
        
        scenario = ProductionLineScenario(config)
        
        # Test that the method can return False (run multiple times to ensure it can fail)
        results = [scenario._quality_check(0) for _ in range(50)]
        # With 10% fail rate, at least one should fail
        assert not all(results), "Quality check should fail at least once with 10% fail rate"
        
    @patch('robotics_suite.scenarios.production_line.time.time')
    def test_update_production_metrics(self, mock_time):
        """Test production metrics update."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line",
            products_per_cycle=10
        )
        
        scenario = ProductionLineScenario(config)
        scenario.cycle_start_time = 100.0
        scenario.products_processed = 5
        scenario.quality_passed = 4
        scenario.station_states = {
            "station_0": StationState.PROCESSING,
            "station_1": StationState.IDLE
        }
        
        mock_time.return_value = 160.0  # 60 seconds later
        
        scenario._update_production_metrics()
        
        assert scenario.metrics["products_processed"] == 5
        assert scenario.metrics["quality_passed"] == 4
        assert scenario.metrics["efficiency"] == 50.0  # 5/10 * 100
        assert scenario.metrics["quality_rate"] == 80.0  # 4/5 * 100
        assert scenario.metrics["throughput"] == 5.0  # 5 products per minute
        assert scenario.metrics["elapsed_time"] == 60.0
        
        # Check station states
        assert scenario.metrics["station_states"]["station_0"] == "processing"
        assert scenario.metrics["station_states"]["station_1"] == "idle"
        
    @patch('robotics_suite.scenarios.production_line.time.time')
    def test_update_production_metrics_no_products(self, mock_time):
        """Test production metrics update with no products processed."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line"
        )
        
        scenario = ProductionLineScenario(config)
        scenario.cycle_start_time = 100.0
        scenario.products_processed = 0
        scenario.quality_passed = 0
        
        mock_time.return_value = 160.0
        
        scenario._update_production_metrics()
        
        # Should handle division by zero - metrics won't be updated if no products processed
        # The method only updates metrics if products_processed > 0
        
    @patch('robotics_suite.scenarios.production_line.time.time')
    def test_finalize_production_metrics(self, mock_time):
        """Test production metrics finalization."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line",
            products_per_cycle=8
        )
        
        scenario = ProductionLineScenario(config)
        scenario.cycle_start_time = 100.0
        scenario.products_processed = 6
        scenario.quality_passed = 5
        
        mock_time.return_value = 180.0  # 80 seconds later
        
        scenario._finalize_production_metrics()
        
        assert scenario.metrics["cycle_complete"] is True
        assert scenario.metrics["total_time"] == 80.0
        assert scenario.metrics["final_efficiency"] == 75.0  # 6/8 * 100
        assert scenario.metrics["final_quality_rate"] == 83.33333333333334  # 5/6 * 100
        assert scenario.metrics["average_cycle_time"] == 80.0 / 6
        
    @patch('robotics_suite.scenarios.production_line.time.time')
    def test_finalize_production_metrics_no_products(self, mock_time):
        """Test production metrics finalization with no products."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line"
        )
        
        scenario = ProductionLineScenario(config)
        scenario.cycle_start_time = 100.0
        scenario.products_processed = 0
        scenario.quality_passed = 0
        
        mock_time.return_value = 180.0
        
        scenario._finalize_production_metrics()
        
        # Should handle division by zero
        assert scenario.metrics["final_quality_rate"] == 0
        assert scenario.metrics["average_cycle_time"] == 0
        
    def test_cleanup(self):
        """Test scenario cleanup."""
        config = ProductionLineConfig(
            name="test_production",
            description="Test production line"
        )
        
        scenario = ProductionLineScenario(config)
        scenario.station_states = {
            "station_0": StationState.PROCESSING,
            "station_1": StationState.ERROR
        }
        scenario.products_processed = 5
        scenario.quality_passed = 4
        
        scenario.cleanup()
        
        # Check all stations reset to idle
        assert scenario.station_states["station_0"] == StationState.IDLE
        assert scenario.station_states["station_1"] == StationState.IDLE
        
        # Check final metrics
        assert scenario.metrics["cleanup_complete"] is True
        assert scenario.metrics["final_products_processed"] == 5
        assert scenario.metrics["final_quality_passed"] == 4