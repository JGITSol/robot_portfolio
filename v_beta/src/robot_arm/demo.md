# UR5 Robot Demo

## Overview
This demo showcases three UR5 robot arms, each equipped with a different type of gripper, performing coordinated pick-and-place operations in a simulated environment.

## Features

### Robot Configurations
1. **Robot 1**: UR5 with Robotiq 2F-85 (85mm parallel gripper)
2. **Robot 2**: UR5 with Robotiq 2F-140 (140mm parallel gripper)
3. **Robot 3**: UR5 with Robotiq EPick (vacuum gripper)

### Environment Setup
- **Simulation Engine**: PyBullet
- **Table**: Standard URDF table model
- **Work Objects**: Colored blocks for manipulation
- **Visual Differentiation**: Unique colors and labels for each robot

## Key Components

### `setup_environment(physics_client, gui=True)`
Initializes the PyBullet simulation environment.
- Configures camera view
- Sets up physics parameters
- Loads the ground plane and table models
- Returns IDs of created objects

### `create_work_objects(physics_client)`
Creates interactive objects for the robots to manipulate.
- Generates colored blocks
- Configures physical properties
- Returns list of object IDs

## Controls
- **Left-click + drag**: Rotate view
- **Right-click + drag**: Pan view
- **Scroll**: Zoom in/out
- **'q'**: Quit simulation

## Usage
```bash
# Run the demo
python -m robot_arm.demo
```

## Dependencies
- PyBullet
- NumPy
- Standard Python libraries

## Implementation Notes
- Uses PyBullet's GUI for visualization
- Implements basic physics simulation
- Includes error handling for model loading
- Supports both GUI and headless modes

## Demo Workflow
1. Initialize three UR5 robots with different grippers
2. Create work objects on the table
3. Each robot performs pick-and-place operations
4. Visual feedback of robot status and actions
5. Clean up on exit
