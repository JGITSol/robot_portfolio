"""Script to run tests with detailed output."""
import sys
import subprocess
import os
from pathlib import Path

def run_command(cmd, cwd=None):
    """Run a command and return its output."""
    print(f"Running: {' '.join(cmd)}")
    try:
        result = subprocess.run(
            cmd,
            cwd=cwd,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        return result.stdout, result.stderr, result.returncode
    except subprocess.CalledProcessError as e:
        return e.stdout, e.stderr, e.returncode

def main():
    print("Running tests with detailed output...\n")
    
    # Get the project root directory
    project_root = Path(__file__).parent.absolute()
    
    # Activate virtual environment
    if os.name == 'nt':  # Windows
        activate_script = project_root / 'venv' / 'Scripts' / 'activate.bat'
        python_exec = project_root / 'venv' / 'Scripts' / 'python.exe'
    else:  # Unix/Linux/Mac
        activate_script = project_root / 'venv' / 'bin' / 'activate'
        python_exec = project_root / 'venv' / 'bin' / 'python'
    
    # Check if virtual environment exists
    if not python_exec.exists():
        print(f"Error: Virtual environment not found at {python_exec}")
        print("Please create it first with: python -m venv venv")
        return 1
    
    # Run pytest directly with the virtual environment's Python
    print(f"Using Python: {python_exec}\n")
    
    # Run pytest with verbose output and no capture
    cmd = [str(python_exec), "-m", "pytest", "tests/", "-v", "-s"]
    stdout, stderr, returncode = run_command(cmd, cwd=project_root)
    
    print("\n=== Test Output ===")
    print(stdout)
    
    if stderr:
        print("\n=== Errors ===")
        print(stderr)
    
    print(f"\nExit code: {returncode}")
    return returncode

if __name__ == "__main__":
    sys.exit(main())
