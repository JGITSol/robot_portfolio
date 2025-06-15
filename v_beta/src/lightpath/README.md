# LightPath Module

## Overview
The `lightpath` module provides route optimization and path finding capabilities for robotic systems using graph-based algorithms.

## Core Components

### LightPathEngine
- **Location**: `core.py`
- **Purpose**: Core route optimization engine with multiple algorithm support
- **Features**:
  - Multiple path-finding algorithms
  - Extensible architecture
  - Support for weighted graphs

### RouteRequest
- **Location**: `core.py`
- **Purpose**: Data container for route optimization requests
- **Attributes**:
  - `origin`: Starting coordinates (x, y)
  - `destination`: Target coordinates (x, y)
  - `constraints`: Optional dictionary of constraints

## Available Algorithms
1. **Dijkstra's Algorithm**
   - Finds shortest path in weighted graphs
   - Guarantees optimality
   - Good for static environments

2. **A* Algorithm**
   - Uses heuristic for faster search
   - Optimal with admissible heuristics
   - Good for known target locations

## Usage Example
```python
from lightpath import LightPathEngine, RouteRequest

# Initialize engine
engine = LightPathEngine()

# Add locations (nodes)
engine.add_location('A', (0, 0))
engine.add_location('B', (1, 1))
engine.add_location('C', (2, 0))

# Add connections (edges)
engine.add_connection('A', 'B', 1.4)  # weight = 1.4
engine.add_connection('B', 'C', 1.0)
engine.add_connection('A', 'C', 2.0)

# Find optimal route
request = RouteRequest(
    origin=(0, 0),
    destination=(2, 0)
)
path = engine.find_optimal_route(request, algorithm='astar')
print(f"Optimal path: {path}")
```

## Dependencies
- NetworkX
- NumPy
- Standard Python libraries

## Implementation Notes
- Uses NetworkX for graph operations
- Supports custom weight functions
- Thread-safe implementation
- Extensible for new algorithms
