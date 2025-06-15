# Basic Robot Arm Demo

## Overview
This demo showcases the fundamental capabilities of the robot arm simulation framework, including environment setup, robot control, and basic pick-and-place operations.

## Features Demonstrated
- Simulation environment initialization
- UR5 robot with parallel gripper
- Object manipulation (pick and place)
- Basic robot motion control
- Gripper operations

## Components Used

### 1. Simulation Environment
- **SimulationEnvironment**: Manages the physics simulation
- **GUI Mode**: Visual feedback of the simulation
- **Realtime Mode**: Controls simulation speed

### 2. Robot
- **UR5**: 6-DOF robotic arm
- **Parallel Gripper**: For object manipulation
- **Pose Control**: Move to specific positions/orientations

### 3. Objects
- **Table**: Static surface for placing objects
- **Cube**: Dynamic object for manipulation

## Code Walkthrough

### 1. Environment Setup
```python
with SimulationEnvironment(gui=True, realtime=True) as env:
    # Environment is automatically initialized and cleaned up
```

### 2. Add Robot
```python
robot_name = env.add_robot(
    robot_type="ur5",
    position=[0, 0, 0],
    orientation=p.getQuaternionFromEuler([0, 0, 0]),
    urdf_path="path/to/ur5.urdf",
    gripper_type="parallel",
    name="ur5_robot"
)
```

### 3. Add Objects
```python
# Add table
table_id = env.add_object(
    urdf_path="table/table.urdf",
    position=[0.5, 0, 0],
    fixed_base=True
)

# Add cube
cube_id = env.add_object(
    urdf_path="cube_small.urdf",
    position=[0.5, 0, 0.7],
    fixed_base=False
)
```

### 4. Robot Control
```python
# Get robot instance
robot = env.get_robot(robot_name)

# Move to position
robot.move_to_pose(
    target_position=[x, y, z],
    target_orientation=quaternion
)

# Control gripper
robot.gripper.open()
robot.gripper.close()
```

## Running the Demo
```bash
python -m robot_arm_simulator.examples.basic_demo
```

## Expected Behavior
1. The simulation starts with a UR5 robot, table, and cube
2. The robot moves to a pre-grasp position
3. The gripper opens
4. The robot moves down to grasp the cube
5. The gripper closes to grasp the cube
6. The robot lifts the cube
7. The robot moves to a new position
8. The gripper releases the cube

## Dependencies
- PyBullet
- NumPy
- Python standard libraries

## Troubleshooting
- **URDF Not Found**: Ensure the URDF paths are correct
- **Simulation Crashes**: Check for collision issues or invalid positions
- **Gripper Not Working**: Verify gripper_type is supported
