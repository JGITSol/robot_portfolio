# Robotic Arm Production Line Simulation

![Robotic Arm Simulation](docs/images/robotic_arm_simulation.png)

A PyBullet-based simulation of a production line with multiple robotic arms, each equipped with different grippers to handle various objects on a conveyor belt.

## Features

- **Multiple Robot Arms**: Three 6-DOF robotic arms working in parallel
- **Specialized Grippers**: Different gripper types (parallel, suction) for different object types
- **Object Variety**: Handles decahedrons, tetrahedrons, and cylinders
- **Conveyor System**: Realistic conveyor belt simulation
- **Pick and Place**: Automated object detection and manipulation
- **Visualization**: Real-time 3D visualization of the simulation

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/robotic-arm-simulation.git
   cd robotic-arm-simulation
   ```

2. Create a virtual environment (recommended):
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install the package in development mode:
   ```bash
   pip install -e .
   ```

## Usage

### Running the Demo

```bash
python -m robot_arm.demo
```

Or use the console script:

```bash
robot-arm-demo
```

### Controls

- `q`: Quit the simulation
- Mouse drag: Rotate view
- Mouse wheel: Zoom in/out
- Mouse right-click drag: Pan view

## Project Structure

```
src/
└── robot_arm/
    ├── __init__.py
    ├── grippers.py      # Gripper implementations
    ├── robot.py         # Robot arm class
    ├── production_line.py  # Main simulation
    └── demo.py          # Demo script
```

## Requirements

- Python 3.8+
- PyBullet
- NumPy
- PyYAML

## License

MIT License - see [LICENSE](LICENSE) for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
