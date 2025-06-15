"""Script to verify pytest installation and basic functionality."""
import sys
import subprocess

def run_command(cmd):
    """Run a command and return its output."""
    try:
        result = subprocess.run(
            cmd,
            shell=True,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        return result.stdout, result.stderr, result.returncode
    except subprocess.CalledProcessError as e:
        return e.stdout, e.stderr, e.returncode

def main():
    print("Testing pytest installation...\n")
    
    # 1. Check Python version
    print("1. Python version:")
    print(f"   {sys.version}\n")
    
    # 2. Check pytest version
    print("2. Checking pytest version:")
    out, err, code = run_command("python -m pytest --version")
    if code == 0:
        print(f"   {out.strip()}")
    else:
        print(f"   Error: {err.strip()}")
    print()
    
    # 3. Create a temporary test file
    test_content = """
    def test_example():
        assert 1 + 1 == 2
    
    def test_another():
        assert "hello".upper() == "HELLO"
    """
    
    test_file = "temp_test_file.py"
    with open(test_file, "w") as f:
        f.write(test_content)
    
    print(f"3. Created temporary test file: {test_file}")
    
    # 4. Run the test file
    print("4. Running tests...\n")
    out, err, code = run_command(f"python -m pytest {test_file} -v")
    
    print("Test output:")
    print("-" * 50)
    print(out)
    if err:
        print("Errors:")
        print("-" * 50)
        print(err)
    print("-" * 50)
    
    # 5. Clean up
    import os
    if os.path.exists(test_file):
        os.remove(test_file)
        print(f"\nCleaned up: Removed {test_file}")

if __name__ == "__main__":
    main()
