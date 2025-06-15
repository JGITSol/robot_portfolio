from lightpath.core import LightPathEngine, RouteRequest

def main():
    # Initialize the engine
    engine = LightPathEngine()
    
    # Add locations (nodes)
    engine.add_location("A", (0, 0))
    engine.add_location("B", (1, 2))
    engine.add_location("C", (3, 1))
    engine.add_location("D", (4, 3))
    
    # Add connections (edges)
    engine.add_connection("A", "B", 2.5)
    engine.add_connection("B", "C", 2.8)
    engine.add_connection("A", "C", 3.2)
    engine.add_connection("C", "D", 1.8)
    
    # Find optimal route
    request = RouteRequest(origin="A", destination="D")
    route = engine.find_optimal_route(request, algorithm='astar')
    
    print(f"Optimal route: {' -> '.join(route)}")

if __name__ == "__main__":
    main()