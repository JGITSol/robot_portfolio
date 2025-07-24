"""Tests for package initialization."""

import pytest

def test_package_imports():
    """Test that main package imports work correctly."""
    from robotics_suite import SimulationEngine, RobotArm, ScenarioManager
    
    # Should be able to import main classes
    assert SimulationEngine is not None
    assert RobotArm is not None
    assert ScenarioManager is not None
    
def test_package_version():
    """Test package version is available."""
    import robotics_suite
    
    assert hasattr(robotics_suite, '__version__')
    assert robotics_suite.__version__ == "1.0.0"
    
def test_package_author():
    """Test package author is available."""
    import robotics_suite
    
    assert hasattr(robotics_suite, '__author__')
    assert robotics_suite.__author__ == "Robotics Team"
    
def test_core_module_imports():
    """Test core module imports."""
    from robotics_suite.core import SimulationEngine, RobotArm, Environment
    
    assert SimulationEngine is not None
    assert RobotArm is not None
    assert Environment is not None
    
def test_scenarios_module_imports():
    """Test scenarios module imports."""
    from robotics_suite.scenarios import ScenarioManager, BaseScenario, PickPlaceScenario, ProductionLineScenario
    
    assert ScenarioManager is not None
    assert BaseScenario is not None
    assert PickPlaceScenario is not None
    assert ProductionLineScenario is not None
    
def test_utils_module_imports():
    """Test utils module imports."""
    from robotics_suite.utils import get_logger, load_config, save_config
    
    assert get_logger is not None
    assert load_config is not None
    assert save_config is not None
    
def test_cli_module_imports():
    """Test CLI module imports."""
    from robotics_suite.cli import main
    
    assert main is not None