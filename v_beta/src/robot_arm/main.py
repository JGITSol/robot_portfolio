import pybullet as p
import pybullet_data
import time
import numpy as np
from typing import List, Tuple, Optional, Dict, Any
import math

class RoboticArm:
    """6-DOF robotic arm simulation using PyBullet."""
    
    def __init__(self, gui: bool = True, timestep: float = 1./240.):
        """Initialize the robotic arm simulation."""
        # Physics client setup
        self.physics_client = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Load plane and robot
        self.plane_id = p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])
        
        # Get joint information
        self.num_joints = p.getNumJoints(self.robot_id)
        self.joint_indices = [i for i in range(self.num_joints) 
                            if p.getJointInfo(self.robot_id, i)[2] == p.JOINT_REVOLUTE]
        
        # Simulation parameters
        self.timestep = timestep
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(timestep)
        
        # Store joint limits and rest pose
        self.joint_limits = []
        for i in self.joint_indices:
            joint_info = p.getJointInfo(self.robot_id, i)
            self.joint_limits.append((
                joint_info[8],  # lower limit
                joint_info[9]   # upper limit
            ))
        
        # Move to default position
        self.move_to_rest_pose()
    
    def get_joint_positions(self) -> List[float]:
        """Get current joint positions in radians."""
        return [p.getJointState(self.robot_id, i)[0] for i in self.joint_indices]
        
    def move_to_rest_pose(self) -> None:
        """Move the robot to its rest position (all joints at 0)."""
        rest_pose = [0.0] * len(self.joint_indices)
        self.set_joint_positions(rest_pose)
        # Step simulation to apply the movement
        for _ in range(100):
            p.stepSimulation()
            time.sleep(1./240.)
    
    def set_joint_positions(self, target_positions: List[float]) -> None:
        """Set target joint positions."""
        for i, pos in zip(self.joint_indices, target_positions):
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=pos,
                targetVelocity=0,
                force=500,
                positionGain=0.1,
                velocityGain=0.5
            )
    
    def step_simulation(self, steps: int = 1) -> None:
        """Step the simulation."""
        for _ in range(steps):
            p.stepSimulation()
    
    def get_end_effector_pose(self) -> Tuple[List[float], List[float]]:
        """Get end effector position and orientation."""
        return p.getLinkState(self.robot_id, self.joint_indices[-1])[4:6]
    
    def move_to_pose(self, target_position: List[float], 
                    target_orientation: Optional[List[float]] = None) -> bool:
        """Move end effector to target pose."""
        # Implementation of inverse kinematics
        pass
    
    def close(self) -> None:
        """Clean up the simulation."""
        p.disconnect(self.physics_client)

def main():
    print("Starting Robotic Arm Simulation...")
    robot = None
    
    try:
        # Initialize the robotic arm with GUI
        robot = RoboticArm(gui=True)
        print("Robot initialized. Press Ctrl+C to exit.")
        
        # Move to rest position first
        print("Moving to rest position...")
        robot.move_to_rest_pose()
        
        # Simple interactive loop
        while True:
            print("\nOptions:")
            print("1. Move to rest position")
            print("2. Move to example position")
            print("3. Get current joint positions")
            print("4. Exit")
            
            choice = input("Select an option (1-4): ")
            
            if choice == '1':
                print("Moving to rest position...")
                robot.move_to_rest_pose()
            elif choice == '2':
                # Example position (adjust values as needed)
                print("Moving to example position...")
                example_pos = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
                robot.set_joint_positions(example_pos)
                # Step simulation to show movement
                for _ in range(100):
                    p.stepSimulation()
                    time.sleep(1./240.)
            elif choice == '3':
                positions = robot.get_joint_positions()
                print("\nCurrent joint positions (radians):")
                for i, pos in enumerate(positions, 1):
                    print(f"Joint {i}: {pos:.4f}")
            elif choice == '4':
                print("Exiting...")
                break
            else:
                print("Invalid choice. Please select a number between 1-4.")
    
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        if robot:
            print("Cleaning up...")
            robot.close()

if __name__ == "__main__":
    main()