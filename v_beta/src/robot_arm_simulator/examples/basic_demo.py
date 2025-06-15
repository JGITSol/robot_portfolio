"""
Basic demo of the robot arm simulation framework.

This script demonstrates how to:
1. Create a simulation environment
2. Add a UR5 robot to the simulation
3. Add a parallel gripper to the robot
4. Add objects to the scene
5. Control the robot to pick and place objects
"""
import os
import time
import pybullet as p
import pybullet_data
import numpy as np

# Add the parent directory to the Python path to allow importing the package
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_arm_simulator.simulation.environment import SimulationEnvironment

def main():
    # Create a simulation environment with GUI enabled
    with SimulationEnvironment(gui=True, realtime=True) as env:
        print("Simulation environment created")
        
        # Add a UR5 robot
        robot_name = env.add_robot(
            robot_type="ur5",
            position=[0, 0, 0],
            orientation=p.getQuaternionFromEuler([0, 0, 0]),
            urdf_path=os.path.join("models", "ur5", "ur5_description", "ur_description", "urdf", "ur5_robot.urdf"),
            gripper_type="parallel",
            name="ur5_robot"
        )
        print(f"Added robot: {robot_name}")
        
        # Add a table
        table_id = env.add_object(
            urdf_path="table/table.urdf",
            position=[0.5, 0, 0],
            orientation=[0, 0, 0, 1],
            fixed_base=True,
            name="table"
        )
        print(f"Added table: {table_id}")
        
        # Add a small cube to pick up
        cube_id = env.add_object(
            urdf_path="cube_small.urdf",
            position=[0.5, 0, 0.7],
            orientation=[0, 0, 0, 1],
            fixed_base=False,
            name="cube"
        )
        print(f"Added cube: {cube_id}")
        
        # Get the robot instance
        robot = env.get_robot(robot_name)
        
        # Move to a pre-grasp position
        print("Moving to pre-grasp position...")
        robot.move_to_pose(
            target_position=[0.5, 0, 0.8],
            target_orientation=p.getQuaternionFromEuler([np.pi, 0, 0])
        )
        
        # Wait for the robot to reach the position
        for _ in range(100):
            env.step()
            time.sleep(0.01)
        
        # Open the gripper
        if hasattr(robot, 'gripper'):
            robot.gripper.open()
        
        # Move down to grasp position
        print("Moving to grasp position...")
        robot.move_to_pose(
            target_position=[0.5, 0, 0.6],
            target_orientation=p.getQuaternionFromEuler([np.pi, 0, 0])
        )
        
        # Wait for the robot to reach the position
        for _ in range(100):
            env.step()
            time.sleep(0.01)
        
        # Close the gripper to grasp the cube
        if hasattr(robot, 'gripper'):
            robot.gripper.close()
        
        # Wait for the gripper to close
        for _ in range(50):
            env.step()
            time.sleep(0.01)
        
        # Lift the cube
        print("Lifting the cube...")
        robot.move_to_pose(
            target_position=[0.5, 0, 1.0],
            target_orientation=p.getQuaternionFromEuler([np.pi, 0, 0])
        )
        
        # Wait for the robot to lift the cube
        for _ in range(100):
            env.step()
            time.sleep(0.01)
        
        # Move to a new position
        print("Moving to target position...")
        robot.move_to_pose(
            target_position=[-0.5, 0, 1.0],
            target_orientation=p.getQuaternionFromEuler([np.pi, 0, 0])
        )
        
        # Wait for the robot to reach the target position
        for _ in range(100):
            env.step()
            time.sleep(0.01)
        
        # Lower the cube
        robot.move_to_pose(
            target_position=[-0.5, 0, 0.6],
            target_orientation=p.getQuaternionFromEuler([np.pi, 0, 0])
        )
        
        # Wait for the robot to lower the cube
        for _ in range(100):
            env.step()
            time.sleep(0.01)
        
        # Open the gripper to release the cube
        if hasattr(robot, 'gripper'):
            robot.gripper.open()
        
        # Wait for the gripper to open
        for _ in range(50):
            env.step()
            time.sleep(0.01)
        
        # Move back up
        robot.move_to_pose(
            target_position=[-0.5, 0, 1.0],
            target_orientation=p.getQuaternionFromEuler([np.pi, 0, 0])
        )
        
        # Run the simulation for a while
        print("Simulation running. Press Ctrl+C to exit.")
        try:
            while True:
                env.step()
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("Simulation stopped by user")

if __name__ == "__main__":
    main()
