"""
UR5 Robot Arms with Different Grippers Demo

This script demonstrates three robot arms, each equipped with a different type of gripper:
1. robot arm with Robotiq 2F-85 (parallel gripper, 85mm)
2. robot arm with Robotiq 2F-140 (wider parallel gripper, 140mm)
3. robot arm with Robotiq EPick (vacuum gripper)

Key Features:
- Three robot arms with industry-standard grippers
- Each robot has a different color and label
- Demonstration of pick and place operations
- Visual differentiation between robots

Controls:
- Press 'q' to quit the simulation
- Left-click and drag to rotate view
- Right-click and drag to pan
- Scroll to zoom
"""

import os
import sys
import time
import math
from pathlib import Path

# Add the src directory to the path
current_dir = Path(__file__).parent
src_dir = current_dir.parent
if str(src_dir) not in sys.path:
    sys.path.insert(0, str(src_dir))

import pybullet as p
import pybullet_data
import numpy as np

# Import our modules
from robot_arm.ur5_robot import UR5Robot
from robot_arm.robot_config import get_robot_configs, setup_robot_visuals, add_robot_label

def setup_environment(physics_client, gui=True):
    """Set up the PyBullet simulation environment."""
    # Connect to the physics server
    if gui:
        physics_client.connect(p.GUI)
    else:
        physics_client.connect(p.DIRECT)
    
    # Configure debug visualizer
    physics_client.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    physics_client.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
    
    # Set up the camera
    physics_client.resetDebugVisualizerCamera(
        cameraDistance=3.0,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.5]
    )
    
    # Set gravity and time step
    physics_client.setGravity(0, 0, -9.81)
    physics_client.setTimeStep(1.0/240.0)
    
    # Load plane
    physics_client.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = physics_client.loadURDF("plane.urdf")
    
    # Load a table
    table_urdf = os.path.join(pybullet_data.getDataPath(), "table/table.urdf")
    table_id = physics_client.loadURDF(table_urdf, [0, 0, -0.63])
    
    return plane_id, table_id

def create_work_objects(physics_client):
    """Create some objects for the robots to manipulate."""
    # Create a few colored blocks
    block_size = 0.05
    colors = [
        [1, 0, 0, 1],  # Red
        [0, 1, 0, 1],  # Green
        [0, 0, 1, 1],  # Blue
    ]
    
    block_ids = []
    for i, color in enumerate(colors):
        visual_shape_id = physics_client.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[block_size/2, block_size/2, block_size/2],
            rgbaColor=color
        )
        
        collision_shape_id = physics_client.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[block_size/2, block_size/2, block_size/2]
        )
        
        block_id = physics_client.createMultiBody(
            baseMass=0.1,  # 100g
            baseCollisionShapeIndex=collision_shape_id,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=[-1.0 + i * 0.1, 0.5, block_size/2],
            baseOrientation=physics_client.getQuaternionFromEuler([0, 0, 0])
        )
        block_ids.append(block_id)
    
    return block_ids

def main():
    print("Starting Robot Arm Simulation with Multiple KUKA iiwa Arms")
    print("=" * 60)
    print("This simulation demonstrates:")
    print("- Three robot arms (KUKA iiwa) with different colors and labels")
    print("- Each robot has a different color and label")
    print("- Demonstration of pick and place operations")
    print("\nPress 'q' in the simulation window to quit\n")
    
    # Set up the simulation environment
    physics_client = p
    plane_id, table_id = setup_environment(physics_client, gui=True)
    
    # Get robot configurations
    robot_configs = get_robot_configs()
    
    # Create robots
    robots = {}
    for name, config in robot_configs.items():
        print(f"\n=== Setting up {name} ===")
        print(f"Position: {config['base_position']}")
        print(f"Gripper: {config['gripper_type']}")
        
        # Create the robot
        robot = UR5Robot(
            physics_client=physics_client,
            base_position=config['base_position'],
            urdf_path=config['urdf_path'],
            gripper_type=config['gripper_type'],
            gripper_urdf=config['gripper_urdf'],
            ee_link_name=config['ee_link_name'],
            gripper_mount_offset=config['gripper_mount_offset'],
            color=config['color'],
            label=config['label'],
            physics_client_id=0
        )
        
        # Add label above the robot
        label_pos = config['base_position'].copy()
        label_pos[2] += 1.0  # Position above the robot
        add_robot_label(label_pos, config['label'], config['color'])
        
        robots[name] = robot
    
    # Create some objects to manipulate
    block_ids = create_work_objects(physics_client)
    
    # Main simulation loop
    print("\n=== Starting simulation ===")
    print("Press 'q' to quit")
    
    # Move to initial positions
    for robot in robots.values():
        robot.move_to_rest_pose()
    
    # Simulation loop
    try:
        step = 0
        while True:
            # Step the simulation
            physics_client.stepSimulation()
            
            # Simple demo: make each robot move in a small circle
            for i, (name, robot) in enumerate(robots.items()):
                t = step * 0.01
                angle = t + i * (2 * math.pi / 3)  # Offset each robot
                
                # Target position in a circle
                radius = 0.2
                target_pos = [
                    robot.base_position[0] + math.cos(angle) * radius,
                    robot.base_position[1] + 0.3 + math.sin(angle) * radius * 0.5,
                    0.1
                ]
                
                # Update robot target
                robot.move_to_pose(target_pos)
                
                # Update robot state
                robot.update()
            
            # Check for exit
            keys = p.getKeyboardEvents()
            if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
                break
                
            step += 1
            time.sleep(1.0/240.0)
            
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    finally:
        print("\nSimulation finished. Cleaning up...")
        # Only disconnect if connected
        try:
            if hasattr(physics_client, 'getConnectionInfo'):
                info = physics_client.getConnectionInfo()
                if info.get('isConnected', 0):
                    physics_client.disconnect()
        except Exception:
            pass

if __name__ == "__main__":
    main()
