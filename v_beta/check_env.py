"""Script to check Python environment and paths."""
import sys
import os
import pytest

def main():
    print("Python Environment Check")
    print("======================")
    
    # Print Python version
    print(f"Python Version: {sys.version}")
    print(f"Python Executable: {sys.executable}")
    
    # Print Python path
    print("\nPython Path:")
    for path in sys.path:
        print(f"  {path}")
    
    # Check pytest version
    print(f"\npytest Version: {pytest.__version__}")
    
    # Check if tests directory exists
    tests_dir = os.path.join(os.getcwd(), 'tests')
    print(f"\nTests directory exists: {os.path.exists(tests_dir)}")
    if os.path.exists(tests_dir):
        print("Test files found:")
        for f in os.listdir(tests_dir):
            if f.startswith('test_') and f.endswith('.py'):
                print(f"  {f}")

if __name__ == "__main__":
    main()
