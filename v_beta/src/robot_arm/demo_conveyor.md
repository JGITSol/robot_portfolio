# Conveyor Demo

## Overview
This demo showcases a conveyor belt system with three KUKA iiwa robot arms for object manipulation. The conveyor can be controlled in real-time using keyboard inputs.

## Features
- Interactive conveyor belt with start/stop and reverse functionality
- Three KUKA iiwa robots placed alongside the conveyor
- Dynamic marble spawning system
- Real-time physics simulation

## Controls
- **'s' or NUMPAD_5**: Toggle conveyor start/stop
- **'r' or NUMPAD_8**: Reverse conveyor direction
- **'q'**: Quit the demo

## Components

### Conveyor System
- **Dimensions**: 6.0m (length) × 0.5m (width)
- **Speed**: Configurable (default: 0.5 m/s)
- **Visual**: Checkered texture for better visibility

### Robot Arms
- **Model**: KUKA iiwa (URDF)
- **Quantity**: 3 units
- **Placement**: Evenly spaced along the conveyor
- **Visual**: Labeled (KUKA-1, KUKA-2, KUKA-3)

### Marble Objects
- **Shape**: Spheres
- **Radius**: 0.05m
- **Mass**: 0.02kg
- **Spawn Rate**: Every 2 seconds (configurable)

## Implementation Details

### Physics Setup
- **Gravity**: -9.81 m/s² (Z-axis)
- **Time Step**: 1/240 seconds
- **Collision Detection**: Enabled

### Keyboard Handling
- Uses PyBullet's `getKeyboardEvents()`
- Supports both main keyboard and numpad
- Debounced input handling

## Usage
```bash
# Run the demo
python -m robot_arm.demo_conveyor
```

## Dependencies
- PyBullet
- PyBullet Data (for URDF models)
- Standard Python libraries

## Notes
- The demo runs in real-time simulation
- Physics accuracy depends on system performance
- Adjust conveyor speed and marble spawn rate as needed
