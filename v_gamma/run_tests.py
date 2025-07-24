#!/usr/bin/env python3
"""Comprehensive test runner for robotics suite with 100% coverage target."""

import sys
import subprocess
import os
from pathlib import Path

def run_command(cmd, description):
    """Run a command and handle errors."""
    print(f"\n{'='*60}")
    print(f"Running: {description}")
    print(f"Command: {' '.join(cmd)}")
    print('='*60)
    
    try:
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        print(result.stdout)
        if result.stderr:
            print("STDERR:", result.stderr)
        return True
    except subprocess.CalledProcessError as e:
        print(f"ERROR: {description} failed!")
        print(f"Return code: {e.returncode}")
        print(f"STDOUT: {e.stdout}")
        print(f"STDERR: {e.stderr}")
        return False

def main():
    """Run comprehensive test suite."""
    print("🤖 Robotics Suite - Comprehensive Test Runner")
    print("Target: 100% Code Coverage")
    
    # Change to project directory
    project_root = Path(__file__).parent
    os.chdir(project_root)
    
    # Test commands to run
    test_commands = [
        # Install package in development mode
        (["pip", "install", "-e", ".[dev]"], "Installing package in development mode"),
        
        # Run linting
        (["flake8", "src/", "tests/", "--max-line-length=88", "--extend-ignore=E203,W503"], "Code linting with flake8"),
        
        # Run type checking
        (["mypy", "src/robotics_suite/", "--ignore-missing-imports"], "Type checking with mypy"),
        
        # Run tests with coverage
        (["pytest", "tests/", "-v", "--cov=src/robotics_suite", "--cov-report=term-missing", "--cov-report=html", "--cov-report=xml", "--cov-fail-under=95"], "Running tests with coverage"),
        
        # Run specific test modules for detailed coverage
        (["pytest", "tests/test_init.py", "-v"], "Testing package initialization"),
        (["pytest", "tests/test_engine.py", "-v"], "Testing simulation engine"),
        (["pytest", "tests/test_robot.py", "-v"], "Testing robot implementation"),
        (["pytest", "tests/test_environment.py", "-v"], "Testing environment"),
        (["pytest", "tests/test_base_scenario.py", "-v"], "Testing base scenario"),
        (["pytest", "tests/test_scenarios.py", "-v"], "Testing scenario manager"),
        (["pytest", "tests/test_pick_place.py", "-v"], "Testing pick and place scenario"),
        (["pytest", "tests/test_production_line.py", "-v"], "Testing production line scenario"),
        (["pytest", "tests/test_utils.py", "-v"], "Testing utilities"),
        (["pytest", "tests/test_cli.py", "-v"], "Testing CLI"),
    ]
    
    # Track results
    passed = 0
    failed = 0
    
    for cmd, description in test_commands:
        if run_command(cmd, description):
            passed += 1
            print(f"✅ {description} - PASSED")
        else:
            failed += 1
            print(f"❌ {description} - FAILED")
    
    # Summary
    print(f"\n{'='*60}")
    print("TEST SUMMARY")
    print('='*60)
    print(f"✅ Passed: {passed}")
    print(f"❌ Failed: {failed}")
    print(f"📊 Total: {passed + failed}")
    
    if failed == 0:
        print("\n🎉 ALL TESTS PASSED! 100% SUCCESS RATE!")
        print("📈 Coverage report generated in htmlcov/index.html")
        return 0
    else:
        print(f"\n⚠️  {failed} test(s) failed. Please review and fix.")
        return 1

if __name__ == "__main__":
    sys.exit(main())