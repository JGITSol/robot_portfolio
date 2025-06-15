"""Script to check Python imports and package structure."""
import sys
import os

def print_section(title):
    """Print a section header for better output readability."""
    print(f"\n{'='*50}")
    print(f"{title.upper()}")
    print(f"{'='*50}")

def main():
    # Print Python version and paths
    print_section("Python Information")
    print(f"Python version: {sys.version}")
    print(f"Python executable: {sys.executable}")
    
    # Print current working directory
    cwd = os.getcwd()
    print_section("Current Working Directory")
    print(cwd)
    
    # Print Python path
    print_section("Python Path")
    for i, path in enumerate(sys.path, 1):
        print(f"{i}. {path}")
    
    # Check if src is in the path
    src_path = os.path.abspath(os.path.join(cwd, 'src'))
    print_section("Checking Source Directory")
    print(f"Looking for src directory at: {src_path}")
    print(f"Directory exists: {os.path.exists(src_path)}")
    
    if os.path.exists(src_path):
        print("\nContents of src directory:")
        for item in os.listdir(src_path):
            item_path = os.path.join(src_path, item)
            print(f"- {item} (directory)" if os.path.isdir(item_path) else f"- {item}")
    
    # Try to import the robot_arm package
    print_section("Testing Imports")
    try:
        # Add src to path if not already there
        if src_path not in sys.path:
            sys.path.insert(0, src_path)
            print(f"Added {src_path} to Python path")
        
        # Try importing robot_arm
        print("\nAttempting to import robot_arm...")
        import robot_arm
        print("Successfully imported robot_arm package!")
        
        # List available modules
        print("\nAvailable modules in robot_arm:")
        for name in dir(robot_arm):
            if not name.startswith('_'):
                print(f"- {name}")
        
    except ImportError as e:
        print(f"\nError importing robot_arm: {e}")
        print("\nTroubleshooting steps:")
        print("1. Make sure you're running this script from the project root directory")
        print("2. Verify that the 'src' directory contains the 'robot_arm' package")
        print("3. Check that the package has an __init__.py file")
        print("4. Ensure all dependencies are installed")

if __name__ == "__main__":
    main()
