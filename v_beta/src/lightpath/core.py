import numpy as np
import networkx as nx
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass

@dataclass
class RouteRequest:
    """Container for route optimization requests."""
    origin: str
    destination: str
    constraints: Optional[Dict] = None

class LightPathEngine:
    """Core route optimization engine with multiple algorithm support."""
    
    def __init__(self):
        self.graph = nx.Graph()
        self.locations = {}
        self.algorithms = {
            'dijkstra': self._dijkstra_shortest_path,
            'astar': self._astar_path,
        }
    
    def add_location(self, node_id: str, coords: Tuple[float, float], **attrs):
        """Add a location node to the graph."""
        self.locations[node_id] = coords
        self.graph.add_node(node_id, pos=coords, **attrs)
        return self
    
    def add_connection(self, node1: str, node2: str, weight: float = None, **attrs):
        """Add a connection between two nodes.
        
        If weight is not provided, it will be calculated as the Euclidean distance
        between the nodes' positions.
        """
        if weight is None:
            # Calculate Euclidean distance if weight is not provided
            pos1 = self.graph.nodes[node1]['pos']
            pos2 = self.graph.nodes[node2]['pos']
            weight = ((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2) ** 0.5
            
        self.graph.add_edge(node1, node2, weight=weight, **attrs)
        return self
    
    def find_optimal_route(self, request: RouteRequest, algorithm: str = 'astar') -> List[str]:
        """Find the optimal route using the specified algorithm."""
        if request.origin not in self.graph or request.destination not in self.graph:
            raise ValueError("No path exists")
            
        if algorithm not in self.algorithms:
            raise ValueError(f"Unsupported algorithm: {algorithm}")
            
        try:
            return self.algorithms[algorithm](request)
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            raise ValueError("No path exists")
    
    def _dijkstra_shortest_path(self, request: RouteRequest) -> List[str]:
        """Find shortest path using Dijkstra's algorithm."""
        try:
            path = nx.dijkstra_path(self.graph, request.origin, request.destination, weight='weight')
            return path
        except nx.NetworkXNoPath:
            raise ValueError("No path exists")
    
    def _astar_path(self, request: RouteRequest) -> List[str]:
        """Find path using A* algorithm with heuristic."""
        def euclidean_heuristic(u, v):
            pos_u = self.graph.nodes[u]['pos']
            pos_v = self.graph.nodes[v]['pos']
            return ((pos_v[0] - pos_u[0])**2 + (pos_v[1] - pos_u[1])**2) ** 0.5
        
        try:
            path = nx.astar_path(self.graph, request.origin, request.destination, 
                               heuristic=euclidean_heuristic, weight='weight')
            return path
        except nx.NetworkXNoPath:
            raise ValueError("No path exists")