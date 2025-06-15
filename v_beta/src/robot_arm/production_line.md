# Production Line Simulation

## Overview
The `ProductionLine` class simulates an industrial production line with multiple robot arms working in coordination. It includes a conveyor belt system, object spawning, and multiple robot arms with different gripper types.

## Class: `ProductionLine`

### Initialization
```python
__init__(self, gui: bool = True)
```

### Key Components

#### 1. Conveyor System
- **Setup**: `setup_conveyor()`
  - Creates a moving conveyor belt
  - Configurable length, width, and speed
  - Visual and collision properties

#### 2. Robot Arms
- **Setup**: `setup_robots()`
  - Creates multiple robot arms at specified positions
  - Supports different gripper types (parallel, suction)
  - Each robot has a dedicated work area

#### 3. Object Handling
- **Object Types**:
  - Boxes
  - Cylinders
  - Spheres
- **Spawning**:
  - Automatic object generation on conveyor
  - Configurable spawn intervals
  - Random object types and properties

### Main Simulation Loop
```python
def run_simulation(self, duration: float = 60.0):
    """Run the production line simulation."""
    # Implementation details...
```

### Robot Tasks
1. **Pick and Place**:
   - Detect objects on conveyor
   - Pick up objects using appropriate gripper
   - Place in designated bins

2. **Coordination**:
   - Robot work zones to prevent collisions
   - Task assignment based on object type
   - Error recovery

## Usage Example
```python
# Create and run production line simulation
production_line = ProductionLine(gui=True)
production_line.run_simulation(duration=120)  # Run for 2 minutes
```

## Visualization
- **3D View**: Real-time visualization of robots and conveyor
- **Debugging**: Visual markers for object detection
- **Status**: Console output for simulation events

## Configuration
- Adjustable parameters:
  - Conveyor speed
  - Object spawn rate
  - Robot positions and types
  - Simulation speed

## Dependencies
- PyBullet
- NumPy
- Standard Python libraries

## Implementation Notes
- Uses PyBullet's physics engine
- Thread-safe for multi-robot operations
- Includes collision detection
- Supports both GUI and headless modes
