"""Script to verify the Python environment and test execution."""

def test_environment():
    """Test that the environment is set up correctly."""
    print("\n=== Running environment test ===")
    print("This is a simple test to verify the Python environment.")
    assert 1 + 1 == 2, "Basic math should work"
    print("Environment test passed!")

if __name__ == "__main__":
    test_environment()
    print("\nTo run tests with pytest, try:")
    print("  python -m pytest tests/ -v")
