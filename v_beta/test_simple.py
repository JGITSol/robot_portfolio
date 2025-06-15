"""
Simple test script to verify PyBullet installation and basic functionality.
This creates a simple scene with a plane and a box to test if PyBullet is working.
"""
import pybullet as p
import pybullet_data
import time

def main():
    print("Starting simple PyBullet test...")
    
    # Connect to the physics server
    physics_client = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optional
    
    # Configure the camera
    p.resetDebugVisualizerCamera(
        cameraDistance=3,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0]
    )
    
    # Set gravity and time step
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1./240.)
    
    # Load a plane and a simple object
    print("Loading plane and box...")
    plane_id = p.loadURDF("plane.urdf")
    
    # Create a simple box
    box_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])
    box_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1], rgbaColor=[1, 0, 0, 1])
    box_body = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=box_id, baseVisualShapeIndex=box_visual, basePosition=[0, 0, 1])
    
    print("Simulation running. Close the PyBullet window to exit.")
    
    # Run the simulation
    try:
        for i in range(1000):  # Run for 1000 steps or until window is closed
            p.stepSimulation()
            time.sleep(1./240.)
            
            # Check if the window is still open
            if not p.isConnected():
                break
                
    except KeyboardInterrupt:
        print("\nSimulation stopped by user")
    finally:
        # Clean up
        p.disconnect()
        print("Simulation finished")

if __name__ == "__main__":
    main()
