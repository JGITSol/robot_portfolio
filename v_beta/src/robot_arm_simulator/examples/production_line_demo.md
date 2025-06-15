# Production Line Simulation Demo

## Overview
This demo showcases a complete production line with multiple robotic arms working in coordination with a conveyor system. It demonstrates object tracking, task distribution, and synchronized robot operations in a manufacturing-like environment.

## Key Components

### 1. Robot Roles
- **PICKER**: Retrieves objects from the conveyor
- **PROCESSOR**: Performs operations on the objects
- **PACKER**: Handles final packaging

### 2. Production Objects
- **Tracking**: Position, orientation, velocity
- **State Management**: On conveyor, being processed, packed
- **Stability Detection**: Monitors object movement

### 3. Conveyor System
- **Object Transport**: Moves items between workstations
- **Collision Handling**: Manages object interactions
- **Speed Control**: Adjustable transport speed

## Features
- Multi-robot coordination
- Real-time object tracking
- Task queuing and prioritization
- Physics-based simulation
- Visual feedback and monitoring
- Error handling and recovery

## Simulation Workflow
1. Objects spawn on the input conveyor
2. Picker robot detects and picks objects
3. Objects move to processing station
4. Processor robot performs operations
5. Packer robot packages finished products
6. Packages exit via output conveyor

## Code Structure

### ProductionObject Class
- **Attributes**: ID, type, position, velocity, state
- **Methods**:
  - `update()`: Sync with physics simulation
  - `is_stable()`: Check if object is at rest

### ProductionLineSimulation Class
- **Initialization**: Setup environment, robots, conveyors
- **Main Loop**:
  ```python
  while running:
      self._spawn_objects()
      self._update_objects()
      self._assign_tasks()
      self._update_robots()
      self._step_simulation()
  ```

### Robot Control
- **State Machines**: Each robot follows a defined workflow
- **Task Queues**: Prioritized task assignment
- **Motion Planning**: Collision-free trajectory generation

## Usage
```python
# Create and run the simulation
sim = ProductionLineSimulation(gui=True, realtime=True)
sim.run()
```

### Configuration Options
- `gui`: Enable/disable visualization
- `realtime`: Run at real-time speed
- `spawn_rate`: Control object generation frequency
- `conveyor_speed`: Adjust transport speed

## Dependencies
- PyBullet
- NumPy
- Python standard libraries

## Monitoring
- Object counts by state
- Robot utilization
- Processing times
- Error rates

## Extending the Simulation
1. Add new robot roles
2. Implement custom object types
3. Define new processing steps
4. Add quality control checks
5. Integrate with external systems

## Troubleshooting
- **Objects Falling Off Conveyor**: Check collision shapes
- **Robots Missing Objects**: Verify grasp positions
- **Simulation Jitter**: Adjust time step and physics parameters
