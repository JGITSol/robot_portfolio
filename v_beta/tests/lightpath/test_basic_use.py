"""Tests for the basic_use module."""
import io
import sys
from unittest.mock import patch

def test_main_output():
    """Test that main() produces the expected output."""
    # Redirect stdout to capture print statements
    captured_output = io.StringIO()
    sys.stdout = captured_output
    
    # Import and run main
    from lightpath.basic_use import main
    main()
    
    # Reset stdout
    sys.stdout = sys.__stdout__
    
    # Check the output
    output = captured_output.getvalue().strip()
    assert "Optimal route: A -> C -> D" in output or "Optimal route: A -> B -> C -> D" in output

def test_import():
    """Test that the module can be imported and has the expected attributes."""
    from lightpath import basic_use
    assert hasattr(basic_use, 'main')
    assert callable(basic_use.main)
