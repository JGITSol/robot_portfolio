# Main Simulation Script

## Overview
`main.py` provides an interactive simulation of a 6-DOF robotic arm using PyBullet. It serves as both a demonstration and a testbed for the robotic arm's basic functionalities.

## Core Components

### Class: `RoboticArm`
A simplified 6-DOF robotic arm implementation using the KUKA IIWA model.

#### Key Methods
- `__init__(gui=True, timestep=1/240)`: Initialize the simulation
- `get_joint_positions()`: Get current joint angles
- `move_to_rest_pose()`: Move arm to default position
- `set_joint_positions(positions)`: Command joints to specified angles
- `get_end_effector_pose()`: Get current end effector position/orientation
- `move_to_pose(target_position, target_orientation)`: Move end effector to target

## Interactive Menu

### Options
1. **Move to rest position**
   - Returns all joints to 0 radians

2. **Move to example position**
   - Moves to a predefined joint configuration
   - Example: `[0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]`

3. **Get current joint positions**
   - Displays all joint angles in radians

4. **Exit**
   - Cleanly shuts down the simulation

## Usage
```bash
# Run the simulation
python src/robot_arm/main.py
```

## Simulation Controls
- **GUI Controls**:
  - WASD: Move camera
  - QE: Zoom in/out
  - Mouse drag: Rotate view
  - Right-click + drag: Pan view

## Dependencies
- PyBullet
- NumPy
- Standard Python libraries

## Implementation Notes
- Uses PyBullet's position control for joint movements
- Fixed timestep of 1/240 seconds for stable simulation
- Includes basic error handling for user inputs
- Automatically cleans up resources on exit
