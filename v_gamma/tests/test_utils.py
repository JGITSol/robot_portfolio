"""Tests for utility modules."""

import pytest
import json
import yaml
import tempfile
from pathlib import Path
from unittest.mock import patch, mock_open
from pydantic import BaseModel

from robotics_suite.utils.logger import get_logger, set_log_level
from robotics_suite.utils.config import load_config, save_config, validate_config


class TestLogger:
    """Test cases for logger utilities."""
    
    def test_get_logger_with_name(self):
        """Test getting logger with specific name."""
        logger = get_logger("test_module")
        assert logger.name == "test_module"
        
    def test_get_logger_without_name(self):
        """Test getting logger without name."""
        logger = get_logger()
        assert logger.name == "robotics_suite"
        
    def test_get_logger_none_name(self):
        """Test getting logger with None name."""
        logger = get_logger(None)
        assert logger.name == "robotics_suite"
        
    def test_set_log_level_valid(self):
        """Test setting valid log level."""
        # Should not raise exception
        set_log_level("DEBUG")
        set_log_level("INFO")
        set_log_level("WARNING")
        set_log_level("ERROR")
        set_log_level("CRITICAL")
        
    def test_set_log_level_invalid(self):
        """Test setting invalid log level."""
        with pytest.raises(ValueError, match="Invalid log level"):
            set_log_level("INVALID")
            
    def test_set_log_level_case_insensitive(self):
        """Test setting log level is case insensitive."""
        # Should not raise exception
        set_log_level("debug")
        set_log_level("info")


class TestConfig:
    """Test cases for configuration utilities."""
    
    def test_load_config_yaml(self):
        """Test loading YAML configuration."""
        yaml_content = """
        name: test_config
        value: 42
        nested:
          key: value
        """
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(yaml_content)
            f.flush()
            
            config = load_config(f.name)
            
            assert config["name"] == "test_config"
            assert config["value"] == 42
            assert config["nested"]["key"] == "value"
            
        Path(f.name).unlink()  # Clean up
        
    def test_load_config_yml(self):
        """Test loading YML configuration."""
        yml_content = """
        name: test_config
        value: 42
        """
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yml', delete=False) as f:
            f.write(yml_content)
            f.flush()
            
            config = load_config(f.name)
            
            assert config["name"] == "test_config"
            assert config["value"] == 42
            
        Path(f.name).unlink()  # Clean up
        
    def test_load_config_json(self):
        """Test loading JSON configuration."""
        json_content = {
            "name": "test_config",
            "value": 42,
            "nested": {"key": "value"}
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            json.dump(json_content, f)
            f.flush()
            
            config = load_config(f.name)
            
            assert config["name"] == "test_config"
            assert config["value"] == 42
            assert config["nested"]["key"] == "value"
            
        Path(f.name).unlink()  # Clean up
        
    def test_load_config_file_not_found(self):
        """Test loading non-existent configuration file."""
        with pytest.raises(FileNotFoundError):
            load_config("non_existent.yaml")
            
    def test_load_config_unsupported_format(self):
        """Test loading unsupported configuration format."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as f:
            f.write("test content")
            f.flush()
            
            with pytest.raises(ValueError, match="Unsupported config format"):
                load_config(f.name)
                
        Path(f.name).unlink()  # Clean up
        
    def test_load_config_invalid_yaml(self):
        """Test loading invalid YAML."""
        invalid_yaml = "invalid: yaml: content: ["
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(invalid_yaml)
            f.flush()
            
            with pytest.raises(Exception):  # YAML parsing error
                load_config(f.name)
                
        Path(f.name).unlink()  # Clean up
        
    def test_load_config_invalid_json(self):
        """Test loading invalid JSON."""
        invalid_json = '{"invalid": json content}'
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            f.write(invalid_json)
            f.flush()
            
            with pytest.raises(Exception):  # JSON parsing error
                load_config(f.name)
                
        Path(f.name).unlink()  # Clean up
        
    def test_save_config_dict_yaml(self):
        """Test saving dictionary configuration as YAML."""
        config_dict = {
            "name": "test_config",
            "value": 42,
            "nested": {"key": "value"}
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            save_config(config_dict, f.name)
            
            # Verify saved content
            loaded_config = load_config(f.name)
            assert loaded_config == config_dict
            
        Path(f.name).unlink()  # Clean up
        
    def test_save_config_dict_json(self):
        """Test saving dictionary configuration as JSON."""
        config_dict = {
            "name": "test_config",
            "value": 42,
            "nested": {"key": "value"}
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            save_config(config_dict, f.name)
            
            # Verify saved content
            loaded_config = load_config(f.name)
            assert loaded_config == config_dict
            
        Path(f.name).unlink()  # Clean up
        
    def test_save_config_pydantic_model(self):
        """Test saving Pydantic model configuration."""
        class TestConfig(BaseModel):
            name: str
            value: int
            
        config_model = TestConfig(name="test_config", value=42)
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            save_config(config_model, f.name)
            
            # Verify saved content
            loaded_config = load_config(f.name)
            assert loaded_config["name"] == "test_config"
            assert loaded_config["value"] == 42
            
        Path(f.name).unlink()  # Clean up
        
    def test_save_config_unsupported_format(self):
        """Test saving configuration with unsupported format."""
        config_dict = {"name": "test"}
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as f:
            with pytest.raises(ValueError, match="Unsupported config format"):
                save_config(config_dict, f.name)
                
        Path(f.name).unlink()  # Clean up
        
    def test_save_config_creates_directories(self):
        """Test that save_config creates parent directories."""
        config_dict = {"name": "test"}
        
        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = Path(temp_dir) / "subdir" / "config.yaml"
            
            save_config(config_dict, config_path)
            
            assert config_path.exists()
            loaded_config = load_config(config_path)
            assert loaded_config == config_dict
            
    def test_save_config_write_error(self):
        """Test save_config with write error."""
        config_dict = {"name": "test"}
        
        # Try to write to a directory (should fail)
        with tempfile.TemporaryDirectory() as temp_dir:
            with pytest.raises(Exception):
                save_config(config_dict, temp_dir)  # Directory, not file
                
    def test_validate_config_success(self):
        """Test successful configuration validation."""
        class TestConfig(BaseModel):
            name: str
            value: int
            
        config_dict = {"name": "test", "value": 42}
        
        validated = validate_config(config_dict, TestConfig)
        
        assert isinstance(validated, TestConfig)
        assert validated.name == "test"
        assert validated.value == 42
        
    def test_validate_config_failure(self):
        """Test configuration validation failure."""
        class TestConfig(BaseModel):
            name: str
            value: int
            
        config_dict = {"name": "test", "value": "not_an_int"}
        
        with pytest.raises(Exception):  # Pydantic validation error
            validate_config(config_dict, TestConfig)
            
    def test_validate_config_missing_field(self):
        """Test configuration validation with missing field."""
        class TestConfig(BaseModel):
            name: str
            value: int
            
        config_dict = {"name": "test"}  # Missing 'value'
        
        with pytest.raises(Exception):  # Pydantic validation error
            validate_config(config_dict, TestConfig)