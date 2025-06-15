# Robot Arm Pendant Control Demo

## Overview
This demo provides a keyboard-based pendant interface for manually controlling a KUKA iiwa robot arm's joints in real-time using PyBullet simulation.

## Features
- Interactive joint-by-joint control
- Adjustable movement step size
- Real-time position feedback
- Smooth joint interpolation

## Controls

### Joint Selection
- **1-7**: Select joint to control (KUKA iiwa has 7 joints)

### Joint Movement
- **Arrow Keys** or **WASD**:
  - **Left/Right** or **A/D**: Rotate joint counter-clockwise/clockwise
  - **Up/Down** or **W/S**: Rotate joint in positive/negative direction

### Configuration
- **+**: Increase movement step size by 20%
- **-**: Decrease movement step size by 20%
- **q**: Quit the application

## Technical Details

### Simulation Setup
- **Physics Engine**: PyBullet
- **Time Step**: 1/240 seconds
- **Gravity**: -9.81 m/s² (Z-axis)
- **Robot Model**: KUKA iiwa (URDF)

### Joint Control
- **Position Control**: Direct joint angle commands
- **Smoothing**: Interpolated movement for smooth transitions
- **Limits**: Respects joint limits defined in URDF

## Usage
```bash
# Run the pendant control demo
python -m robot_arm.demo_pendant
```

## Status Display
The console shows:
- Currently selected joint
- Current step size
- Current joint positions (radians)

## Dependencies
- PyBullet
- PyBullet Data (for URDF models)
- Standard Python libraries

## Notes
- Start with small step sizes for precise control
- Joint limits are automatically enforced
- The simulation runs in real-time by default
