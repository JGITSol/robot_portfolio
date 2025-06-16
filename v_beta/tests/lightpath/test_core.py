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
    
    def test_unsupported_algorithm(self):
        """Test that an unsupported algorithm raises a ValueError."""
        from lightpath.core import RouteRequest
        
        request = RouteRequest(origin='A', destination='D')
        with pytest.raises(ValueError, match="Unsupported algorithm"):
            self.engine.find_optimal_route(request, algorithm='invalid_algorithm')
    
    def test_route_with_missing_node(self):
        """Test that a route with a missing node raises ValueError."""
        from lightpath.core import RouteRequest
        
        # Test with missing origin
        request = RouteRequest(origin='Z', destination='A')
        with pytest.raises(ValueError, match="No path exists"):
            self.engine.find_optimal_route(request)
        
        # Test with missing destination
        request = RouteRequest(origin='A', destination='Z')
        with pytest.raises(ValueError, match="No path exists"):
            self.engine.find_optimal_route(request)
    
    def test_connection_without_weight(self):
        """Test adding a connection without explicit weight."""
        # Add two new locations
        self.engine.add_location('E', (4, 0))
        self.engine.add_location('F', (4, 1))
        
        # Add connection without weight (should use Euclidean distance)
        self.engine.add_connection('E', 'F')
        
        # The weight should be the Euclidean distance between (4,0) and (4,1) = 1.0
        assert abs(self.engine.graph['E']['F']['weight'] - 1.0) < 1e-10
    
    def test_astar_with_heuristic(self):
        """Test A* algorithm with the Euclidean heuristic."""
        from lightpath.core import RouteRequest
        
        # Add some more locations to make the heuristic useful
        self.engine.add_location('E', (4, 0))
        self.engine.add_location('F', (5, 0))
        self.engine.add_connection('D', 'E', 1.0)
        self.engine.add_connection('E', 'F', 1.0)
        
        # This should prefer the direct path A->C->D->E->F over A->B->C->D->E->F
        # because of the Euclidean heuristic
        request = RouteRequest(origin='A', destination='F')
        route = self.engine.find_optimal_route(request, algorithm='astar')
        assert route == ['A', 'C', 'D', 'E', 'F']
    
    def test_dijkstra_with_disconnected_graph(self):
        """Test Dijkstra's algorithm with a disconnected graph."""
        from lightpath.core import RouteRequest
        
        # Create a new engine with a disconnected graph
        engine = LightPathEngine()
        engine.add_location('X', (0, 0))
        engine.add_location('Y', (1, 1))
        engine.add_connection('X', 'Y', 1.0)
        
        # Add a disconnected node
        engine.add_location('Z', (10, 10))
        
        # Should raise ValueError when no path exists
        request = RouteRequest(origin='X', destination='Z')
        with pytest.raises(ValueError, match="No path exists"):
            engine.find_optimal_route(request, algorithm='dijkstra')
    
    def test_astar_with_identical_points(self):
        """Test A* algorithm with identical start and end points."""
        from lightpath.core import RouteRequest
        
        # Test with identical points (should return just the single point)
        request = RouteRequest(origin='A', destination='A')
        route = self.engine.find_optimal_route(request, algorithm='astar')
        assert route == ['A']
    
    def test_route_with_constraints(self):
        """Test that constraints can be passed to the RouteRequest."""
        from lightpath.core import RouteRequest
        
        # Test that constraints can be passed (even if not used in the current implementation)
        constraints = {'max_distance': 10.0}
        request = RouteRequest(origin='A', destination='D', constraints=constraints)
        route = self.engine.find_optimal_route(request, algorithm='dijkstra')
        assert len(route) > 0  # Just verify it returns a valid route
    
    def test_add_location_with_attributes(self):
        """Test adding a location with additional attributes."""
        # Add location with additional attributes
        self.engine.add_location('E', (4, 0), color='red', size=10)
        
        # Check that the attributes were added to the node
        assert self.engine.graph.nodes['E'].get('color') == 'red'
        assert self.engine.graph.nodes['E'].get('size') == 10
    
    def test_add_connection_with_attributes(self):
        """Test adding a connection with additional attributes."""
        # Add connection with additional attributes
        self.engine.add_connection('A', 'D', 5.0, color='blue', bidirectional=True)
        
        # Check that the attributes were added to the edge
        edge_data = self.engine.graph.get_edge_data('A', 'D')
        assert edge_data['weight'] == 5.0
        assert edge_data['color'] == 'blue'
        assert edge_data['bidirectional'] is True
