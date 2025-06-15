import os
from pathlib import Path

def create_project_structure(base_path: str):
    """Create the project directory structure."""
    # Main directories
    dirs = [
        "robot_arm_simulator/src/robot_arm_simulator/config",
        "robot_arm_simulator/src/robot_arm_simulator/models",
        "robot_arm_simulator/src/robot_arm_simulator/grippers",
        "robot_arm_simulator/src/robot_arm_simulator/robots",
        "robot_arm_simulator/src/robot_arm_simulator/simulations",
        "robot_arm_simulator/src/robot_arm_simulator/utils",
        "robot_arm_simulator/tests",
        "robot_arm_simulator/docs",
        "robot_arm_simulator/models/grippers",
        "robot_arm_simulator/models/robots"
    ]
    
    # Create __init__.py files
    init_files = [
        "robot_arm_simulator/src/robot_arm_simulator/__init__.py",
        "robot_arm_simulator/src/robot_arm_simulator/config/__init__.py",
        "robot_arm_simulator/src/robot_arm_simulator/models/__init__.py",
        "robot_arm_simulator/src/robot_arm_simulator/grippers/__init__.py",
        "robot_arm_simulator/src/robot_arm_simulator/robots/__init__.py",
        "robot_arm_simulator/src/robot_arm_simulator/simulations/__init__.py",
        "robot_arm_simulator/src/robot_arm_simulator/utils/__init__.py",
        "robot_arm_simulator/tests/__init__.py"
    ]
    
    # Create all directories
    for directory in dirs:
        full_path = os.path.join(base_path, directory)
        os.makedirs(full_path, exist_ok=True)
        print(f"Created directory: {full_path}")
    
    # Create __init__.py files
    for init_file in init_files:
        full_path = os.path.join(base_path, init_file)
        with open(full_path, 'w') as f:
            f.write("")
        print(f"Created file: {full_path}")

if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.abspath(__file__))
    create_project_structure(base_dir)
    print("\nProject structure created successfully!")
