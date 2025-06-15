print("Python environment test script")
print("=" * 30)

# Test basic Python functionality
try:
    import sys
    import os
    print(f"Python version: {sys.version}")
    print(f"Current directory: {os.getcwd()}")
    print("\nPython path:")
    for p in sys.path:
        print(f"- {p}")
except Exception as e:
    print(f"Error: {e}")

# Test PyBullet import
try:
    import pybullet as p
    print("\nPyBullet imported successfully!")
    print(f"PyBullet version: {p.getAPIVersion()}")
except ImportError as e:
    print(f"\nError importing PyBullet: {e}")

# Test robot_arm import
try:
    import robot_arm
    print("\nrobot_arm package imported successfully!")
    print("Available attributes:", [attr for attr in dir(robot_arm) if not attr.startswith('_')])
except ImportError as e:
    print(f"\nError importing robot_arm package: {e}")
    print("\nTroubleshooting steps:")
    print("1. Make sure you're in the project root directory")
    print("2. Run 'pip install -e .' to install the package in development mode")
    print("3. Check that the 'src' directory is in the Python path")
