# Robot Arm Simulator

A professional-grade robot arm simulation framework for industrial applications, supporting multiple robot models and gripper types.

## Features

- Support for multiple industrial robot arms (UR5, KUKA iiwa, etc.)
- Configurable grippers (parallel, suction, specialized)
- Physics-based simulation using PyBullet
- Predefined industrial tasks and scenarios
- Easy-to-use API for custom simulations
- Visualization and debugging tools

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/robot-arm-simulator.git
   cd robot-arm-simulator
   ```

2. Create and activate a virtual environment (recommended):
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install the package in development mode:
   ```bash
   pip install -e .[dev]
   ```

## Quick Start

```python
import pybullet as p
from robot_arm_simulator.robots import UR5Robot
from robot_arm_simulator.grippers import create_gripper

# Initialize simulation
physics_client = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Create a robot instance
robot = UR5Robot(
    base_position=[0, 0, 0],
    gripper_type="parallel"
)

# Run simulation
while True:
    p.stepSimulation()
    # Add your control logic here
```

## Project Structure

```
robot_arm_simulator/
├── src/
│   └── robot_arm_simulator/
│       ├── __init__.py
│       ├── config/           # Configuration files
│       ├── models/           # 3D models and URDFs
│       ├── grippers/         # Gripper implementations
│       ├── robots/           # Robot arm implementations
│       ├── simulations/      # Predefined simulation scenarios
│       └── utils/            # Utility functions
├── tests/                   # Unit and integration tests
├── docs/                    # Documentation
└── examples/                # Example scripts
```

## Contributing

1. Fork the repository
2. Create a new branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
