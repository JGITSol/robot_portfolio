# Robotics Suite Examples

This directory contains example scripts demonstrating various robotics simulation scenarios.

## Available Examples

### Basic Demo (`basic_demo.py`)
Simple pick and place demonstration with a single robot arm.

**Features:**
- Single UR5 robot arm
- Multiple pick and place positions
- Basic trajectory planning
- Performance metrics

**Usage:**
```bash
python examples/basic_demo.py
```

### Production Line Demo (`production_demo.py`)
Advanced multi-robot production line simulation.

**Features:**
- Multiple robot stations
- Conveyor belt system
- Quality control checks
- Production metrics and KPIs
- Station state monitoring

**Usage:**
```bash
python examples/production_demo.py
```

## Running Examples

### Prerequisites
Make sure you have installed the robotics suite:

```bash
pip install -e .
```

### Command Line Interface
You can also run scenarios using the CLI:

```bash
# Basic pick and place
robotics-demo --scenario pick_place --duration 30

# Production line
robotics-demo --scenario production_line --duration 60

# List available scenarios
robotics-demo list-scenarios

# Validate setup
robotics-demo validate-setup
```

### Customization

Each example can be customized by modifying the configuration parameters:

**Pick and Place Configuration:**
- `pick_positions`: List of [x, y, z] coordinates for pick locations
- `place_positions`: List of [x, y, z] coordinates for place locations
- `approach_height`: Height above objects for safe approach
- `grip_delay`: Time to simulate gripper operation

**Production Line Configuration:**
- `station_count`: Number of processing stations
- `products_per_cycle`: Number of products to process
- `conveyor_speed`: Speed of conveyor belt movement
- `quality_check_enabled`: Enable/disable quality control

## Creating Custom Scenarios

To create your own scenario:

1. Inherit from `BaseScenario`
2. Implement required methods: `setup()`, `execute()`, `cleanup()`
3. Register with `ScenarioManager`

Example:
```python
from robotics_suite.scenarios.base import BaseScenario, ScenarioConfig

class CustomScenario(BaseScenario):
    def setup(self):
        # Setup robots and environment
        pass
        
    def execute(self):
        # Run scenario logic
        pass
        
    def cleanup(self):
        # Clean up resources
        pass
```

## Troubleshooting

### Common Issues

1. **PyBullet GUI not showing**: Ensure you have a display available and GUI is enabled in config
2. **URDF files not found**: Check that robot model files are in the correct path
3. **Performance issues**: Try disabling GUI or reducing simulation complexity

### Debug Mode
Enable debug mode for verbose logging:

```python
config = SimulationConfig(debug_mode=True)
```

### Log Levels
Adjust logging verbosity:

```python
from robotics_suite.utils.logger import set_log_level
set_log_level("DEBUG")  # or INFO, WARNING, ERROR
```