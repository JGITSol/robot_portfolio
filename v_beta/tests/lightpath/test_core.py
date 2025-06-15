"""Tests for the lightpath module."""
import pytest
from hypothesis import given, strategies as st

class TestLightPathEngine:
    """Test cases for LightPathEngine class."""
    
    @pytest.fixture(autouse=True)
    def setup_engine(self):
        """Create a fresh LightPathEngine instance for each test."""
        from lightpath.core import LightPathEngine
        self.engine = LightPathEngine()
        
        # Add test locations
        self.locations = {
            'A': (0, 0),
            'B': (1, 1),
            'C': (2, 0),
            'D': (3, 1)
        }
        
        for loc, pos in self.locations.items():
            self.engine.add_location(loc, pos)
            
        # Add connections
        self.connections = [
            ('A', 'B', 1.4),  # Diagonal
            ('B', 'C', 1.4),
            ('C', 'D', 1.4),
            ('A', 'C', 2.0),  # Direct connection
        ]
        
        for src, dest, dist in self.connections:
            self.engine.add_connection(src, dest, dist)
    
    def test_add_location(self):
        """Test adding a new location."""
        self.engine.add_location('E', (4, 0))
        assert 'E' in self.engine.locations
        assert self.engine.locations['E'] == (4, 0)
    
    def test_add_connection(self):
        """Test adding a connection between locations."""
        self.engine.add_connection('B', 'D', 2.0)
        assert ('B', 'D') in self.engine.graph.edges()
        assert self.engine.graph['B']['D']['weight'] == 2.0
    
    @pytest.mark.parametrize("algorithm", ["dijkstra", "astar"])
    def test_find_optimal_route(self, algorithm):
        """Test finding the optimal route between two points."""
        from lightpath.core import RouteRequest
        
        request = RouteRequest(origin='A', destination='D')
        route = self.engine.find_optimal_route(request, algorithm=algorithm)
        
        # Should take the direct path A -> C -> D (distance 3.4)
        # Instead of A -> B -> C -> D (distance 4.2)
        assert route == ['A', 'C', 'D']
    
    def test_invalid_route(self):
        """Test route finding with invalid locations."""
        from lightpath.core import RouteRequest
        
        request = RouteRequest(origin='A', destination='Z')
        with pytest.raises(ValueError, match="No path exists"):
            self.engine.find_optimal_route(request)
    
    @given(
        x1=st.floats(0, 100),
        y1=st.floats(0, 100),
        x2=st.floats(0, 100),
        y2=st.floats(0, 100)
    )
    def test_euclidean_distance(self, x1, y1, x2, y2):
        """Test Euclidean distance calculation with property-based testing."""
        from lightpath.core import LightPathEngine
        
        # Calculate expected distance using the formula
        expected = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        
        # Test the engine's distance calculation
        engine = LightPathEngine()
        engine.add_location('A', (x1, y1))
        engine.add_location('B', (x2, y2))
        engine.add_connection('A', 'B')
        
        # The distance should match within floating point precision
        assert abs(engine.graph['A']['B']['weight'] - expected) < 1e-10
