"""Test script to verify package imports."""
import sys
import os

# Add src directory to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

try:
    # Try importing the robot_arm package
    import robot_arm
    print("Successfully imported robot_arm package")
    
    # List available modules
    print("\nAvailable modules in robot_arm:")
    for name in dir(robot_arm):
        if not name.startswith('_'):
            print(f"- {name}")
    
    # Try importing specific modules
    try:
        from robot_arm import robot, ur5_robot, grippers
        print("\nSuccessfully imported robot, ur5_robot, and grippers modules")
    except ImportError as e:
        print(f"\nError importing modules: {e}")
    
except ImportError as e:
    print(f"Error importing robot_arm: {e}")
    print("\nPython path:")
    for path in sys.path:
        print(f"- {path}")
    
    print("\nCurrent working directory:")
    print(os.getcwd())
    
    print("\nDirectory contents:")
    for item in os.listdir():
        print(f"- {item}" + (" (directory)" if os.path.isdir(item) else ""))
