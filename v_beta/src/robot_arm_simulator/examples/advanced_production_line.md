# Advanced Production Line Simulation

## Overview
This simulation demonstrates an industrial production line with multiple robotic arms, conveyor belts, and sensors. It showcases advanced features like task scheduling, object tracking, and quality control.

## Key Components

### 1. Simulation Environment
- **SimulationEnvironment**: Manages the PyBullet physics simulation
- **ConveyorBelt**: Handles object transportation
- **Sensors**: Proximity and camera sensors for object detection

### 2. Production Objects
- **ProductionObject**: Represents items moving through the production line
- **States**: Tracks object state (INCOMING, PICKED, UNDER_INSPECTION, etc.)
- **Quality Control**: Includes inspection and quality scoring

### 3. Robot Roles
- **PICKER**: Handles initial object pickup
- **INSPECTOR**: Performs quality checks
- **ASSEMBLER**: Handles assembly operations
- **PACKER**: Packages finished products

### 4. Task Management
- **Task Class**: Defines work items for robots
- **Priority System**: Ensures critical tasks are handled first
- **Task Scheduling**: Manages task assignment and completion

## Features
- Multiple synchronized conveyor belts
- Object tracking and state management
- Quality inspection system
- Task prioritization and scheduling
- Real-time monitoring and statistics
- Collision detection and avoidance
- Visual feedback for simulation state

## Usage
```python
# Create and run the simulation
sim = AdvancedProductionLine(gui=True, realtime=True)
sim.run()
```

### Configuration Options
- `gui`: Enable/disable graphical interface
- `realtime`: Run in real-time or as fast as possible
- `spawn_interval`: Time between spawning new objects (default: 5.0s)

## Simulation Workflow
1. Objects spawn on the input conveyor
2. Picker robot transfers objects to inspection
3. Inspector robot performs quality checks
4. Passed objects move to assembly
5. Assembler robot performs assembly tasks
6. Packer robot packages finished products
7. Packages exit via output conveyor

## Dependencies
- PyBullet
- NumPy
- Python standard libraries

## Statistics Tracking
The simulation maintains statistics on:
- Objects created
- Objects inspected
- Objects assembled
- Objects packed
- Processing times
- Error rates

## Customization
- Add new robot roles by extending the `RobotRole` enum
- Modify object types in `available_types`
- Adjust conveyor speeds and robot parameters
- Extend the task system with new task types
