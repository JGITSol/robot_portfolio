# LightPath Basic Usage Example

## Overview
This example demonstrates how to use the `LightPathEngine` to find optimal routes between locations in a graph-based pathfinding system.

## Key Concepts

### LightPathEngine
- Manages locations (nodes) and connections (edges)
- Supports different pathfinding algorithms
- Maintains the graph structure

### RouteRequest
- Defines a pathfinding request
- Contains origin and destination nodes
- Can include additional constraints (not shown in basic example)

## Example Code Walkthrough

### 1. Import Required Modules
```python
from lightpath.core import LightPathEngine, RouteRequest
```

### 2. Initialize the Engine
```python
engine = LightPathEngine()
```

### 3. Add Locations (Nodes)
```python
engine.add_location("A", (0, 0))  # Node ID and (x,y) coordinates
engine.add_location("B", (1, 2))
engine.add_location("C", (3, 1))
engine.add_location("D", (4, 3))
```

### 4. Add Connections (Edges)
```python
# Add bidirectional connections with distances
engine.add_connection("A", "B", 2.5)  # A <-> B, distance 2.5
engine.add_connection("B", "C", 2.8)  # B <-> C, distance 2.8
engine.add_connection("A", "C", 3.2)  # A <-> C, distance 3.2
engine.add_connection("C", "D", 1.8)  # C <-> D, distance 1.8
```

### 5. Find Optimal Route
```python
# Create route request from A to D
request = RouteRequest(origin="A", destination="D")

# Find path using A* algorithm
route = engine.find_optimal_route(request, algorithm='astar')

# Print result
print(f"Optimal route: {' -> '.join(route)}")
# Example output: Optimal route: A -> C -> D
```

## Available Pathfinding Algorithms
- `'dijkstra'`: Dijkstra's algorithm (guarantees shortest path)
- `'astar'`: A* algorithm (generally faster with good heuristic)

## Expected Output
```
Optimal route: A -> C -> D
```

## Extending the Example
- Add more locations and connections
- Implement custom cost functions
- Add obstacles or constraints
- Visualize the graph and path

## Dependencies
- `lightpath.core` module
- Python standard library
