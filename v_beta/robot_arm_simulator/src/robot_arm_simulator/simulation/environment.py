"""Simulation environment for robot arms."""
from typing import Dict, List, Optional, Tuple, Any, Union
import numpy as np
import pybullet as p
import pybullet_data
import time
import os

from ..robots import create_robot
from ..grippers import create_gripper

class SimulationEnvironment:
    """Main simulation environment for robot arm simulations."""
    
    def __init__(self, 
                 gui: bool = True, 
                 realtime: bool = False,
                 time_step: float = 1.0/240.0,
                 gravity: List[float] = None,
                 **kwargs):
        """Initialize the simulation environment.
        
        Args:
            gui: Whether to show the PyBullet GUI
            realtime: Whether to run in real-time (slower than simulation time)
            time_step: Physics simulation time step (seconds)
            gravity: [x, y, z] gravity vector (m/s²)
        """
        self.gui = gui
        self.realtime = realtime
        self.time_step = time_step
        self.gravity = gravity or [0, 0, -9.81]
        self.physics_client = None
        self.robots = {}
        self.objects = {}
        self._setup_physics()
        
    def _setup_physics(self):
        """Set up the physics simulation."""
        # Connect to the physics server
        if self.gui:
            self.physics_client = p.connect(p.GUI)
            # Configure debug visualizer
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
            p.resetDebugVisualizerCamera(
                cameraDistance=1.5,
                cameraYaw=45,
                cameraPitch=-30,
                cameraTargetPosition=[0, 0, 0.5]
            )
        else:
            self.physics_client = p.connect(p.DIRECT)
            
        # Configure physics engine
        p.setGravity(*self.gravity)
        p.setTimeStep(self.time_step)
        p.setPhysicsEngineParameter(
            fixedTimeStep=self.time_step,
            numSolverIterations=100,
            numSubSteps=4,
            enableConeFriction=1,
            enableFileCaching=0
        )
        
        # Add PyBullet data path for loading default objects
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
    def add_robot(self, 
                 robot_type: str,
                 position: List[float],
                 orientation: Optional[List[float]] = None,
                 name: Optional[str] = None,
                 **robot_kwargs) -> str:
        """Add a robot to the simulation.
        
        Args:
            robot_type: Type of robot to create (e.g., 'ur5')
            position: [x, y, z] position of the robot base
            orientation: [x, y, z, w] quaternion orientation of the base
            name: Optional name for the robot. If None, will be auto-generated.
            **robot_kwargs: Additional arguments to pass to the robot constructor
            
        Returns:
            str: The name of the created robot
        """
        if name is None:
            name = f"{robot_type}_{len([n for n in self.robots if n.startswith(robot_type)])}"
            
        if name in self.robots:
            raise ValueError(f"Robot with name '{name}' already exists")
            
        robot = create_robot(
            robot_type=robot_type,
            physics_client=self.physics_client,
            base_position=position,
            base_orientation=orientation,
            name=name,
            **robot_kwargs
        )
        
        # Load the robot model
        robot.load()
        
        self.robots[name] = robot
        return name
    
    def add_gripper(self, 
                   robot_name: str,
                   gripper_type: str,
                   **gripper_kwargs) -> bool:
        """Add a gripper to a robot.
        
        Args:
            robot_name: Name of the robot to attach the gripper to
            gripper_type: Type of gripper to create
            **gripper_kwargs: Additional arguments to pass to the gripper constructor
            
        Returns:
            bool: True if gripper was attached successfully, False otherwise
        """
        if robot_name not in self.robots:
            print(f"Error: Robot '{robot_name}' not found")
            return False
            
        robot = self.robots[robot_name]
        
        # Create and attach the gripper
        gripper = create_gripper(
            gripper_type=gripper_type,
            physics_client=self.physics_client,
            position=[0, 0, 0],  # Will be positioned by the robot
            **gripper_kwargs
        )
        
        # Attach the gripper to the robot's end effector
        if hasattr(robot, 'attach_gripper'):
            robot.attach_gripper(gripper)
            return True
        else:
            print(f"Error: Robot '{robot_name}' does not support gripper attachment")
            return False
    
    def add_object(self, 
                  urdf_path: str,
                  position: List[float],
                  orientation: Optional[List[float]] = None,
                  name: Optional[str] = None,
                  fixed_base: bool = False,
                  **kwargs) -> str:
        """Add an object to the simulation.
        
        Args:
            urdf_path: Path to the URDF file, or name of a built-in PyBullet object
            position: [x, y, z] position of the object
            orientation: [x, y, z, w] quaternion orientation of the object
            name: Optional name for the object. If None, will be auto-generated.
            fixed_base: Whether the object should be fixed in place
            **kwargs: Additional arguments to pass to loadURDF
            
        Returns:
            str: The name of the created object
        """
        if orientation is None:
            orientation = [0, 0, 0, 1]
            
        if name is None:
            # Generate a name based on the URDF filename
            base_name = os.path.splitext(os.path.basename(urdf_path))[0]
            name = f"{base_name}_{len([n for n in self.objects if n.startswith(base_name)])}"
            
        if name in self.objects:
            raise ValueError(f"Object with name '{name}' already exists")
        
        # Check if this is a built-in PyBullet object
        if not os.path.exists(urdf_path) and not os.path.isabs(urdf_path):
            # Try to load from pybullet_data
            try:
                object_id = p.loadURDF(
                    fileName=urdf_path,
                    basePosition=position,
                    baseOrientation=orientation,
                    useFixedBase=fixed_base,
                    **kwargs
                )
            except:
                raise ValueError(f"Could not find URDF file or built-in object: {urdf_path}")
        else:
            # Load from file
            object_id = p.loadURDF(
                fileName=urdf_path,
                basePosition=position,
                baseOrientation=orientation,
                useFixedBase=fixed_base,
                **kwargs
            )
            
        self.objects[name] = {
            'id': object_id,
            'position': position,
            'orientation': orientation,
            'fixed_base': fixed_base,
            'urdf_path': urdf_path
        }
        
        return name
    
    def get_robot(self, name: str):
        """Get a robot by name."""
        return self.robots.get(name)
    
    def get_object(self, name: str) -> Optional[Dict[str, Any]]:
        """Get an object by name."""
        return self.objects.get(name)
    
    def remove_robot(self, name: str) -> bool:
        """Remove a robot from the simulation."""
        if name not in self.robots:
            return False
            
        # Remove the robot's body from the simulation
        robot = self.robots.pop(name)
        p.removeBody(robot.robot_id)
        return True
    
    def remove_object(self, name: str) -> bool:
        """Remove an object from the simulation."""
        if name not in self.objects:
            return False
            
        # Remove the object's body from the simulation
        obj = self.objects.pop(name)
        p.removeBody(obj['id'])
        return True
    
    def reset(self):
        """Reset the simulation to its initial state."""
        # Reset all robots
        for robot in self.robots.values():
            if hasattr(robot, 'reset'):
                robot.reset()
        
        # Reset all objects (reload them)
        objects = list(self.objects.items())  # Create a copy to avoid modifying during iteration
        for name, obj in objects:
            self.remove_object(name)
            self.add_object(
                urdf_path=obj['urdf_path'],
                position=obj['position'],
                orientation=obj['orientation'],
                name=name,
                fixed_base=obj['fixed_base']
            )
    
    def step(self, sleep_time: Optional[float] = None):
        """Step the simulation forward by one time step.
        
        Args:
            sleep_time: If not None, sleep for this amount of time after stepping.
                       If None and realtime=True, sleep to maintain real-time simulation.
        """
        # Update all robots
        for robot in self.robots.values():
            if hasattr(robot, 'update'):
                robot.update()
        
        # Step the simulation
        p.stepSimulation()
        
        # Sleep if needed
        if sleep_time is not None:
            time.sleep(sleep_time)
        elif self.realtime:
            time.sleep(self.time_step)
    
    def run(self, steps: Optional[int] = None, duration: Optional[float] = None):
        """Run the simulation for a number of steps or a duration.
        
        Args:
            steps: Number of simulation steps to run. If None, run indefinitely.
            duration: Duration to run the simulation for (seconds). If None, run for 'steps' steps.
        """
        start_time = time.time()
        step_count = 0
        
        try:
            while True:
                # Check if we've reached the desired duration
                if duration is not None and (time.time() - start_time) >= duration:
                    break
                    
                # Check if we've reached the desired number of steps
                if steps is not None and step_count >= steps:
                    break
                
                # Step the simulation
                self.step()
                step_count += 1
                
        except KeyboardInterrupt:
            print("Simulation stopped by user")
        
        print(f"Simulation completed. Ran for {step_count} steps ({(time.time() - start_time):.2f} seconds)")
    
    def close(self):
        """Close the simulation and clean up resources."""
        if self.physics_client is not None:
            p.disconnect(self.physics_client)
            self.physics_client = None
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()
        
    def get_robot_state(self, name: str) -> Dict[str, Any]:
        """Get the current state of a robot."""
        if name not in self.robots:
            raise ValueError(f"Robot '{name}' not found")
            
        robot = self.robots[name]
        state = {
            'name': name,
            'position': robot.base_position.tolist() if hasattr(robot.base_position, 'tolist') else robot.base_position,
            'orientation': robot.base_orientation,
            'joint_positions': robot.get_joint_positions() if hasattr(robot, 'get_joint_positions') else [],
            'end_effector_pose': robot.get_end_effector_pose() if hasattr(robot, 'get_end_effector_pose') else (None, None)
        }
        
        if hasattr(robot, 'gripper') and robot.gripper is not None:
            state['gripper'] = robot.gripper.get_state()
            
        return state
    
    def get_object_state(self, name: str) -> Dict[str, Any]:
        """Get the current state of an object."""
        if name not in self.objects:
            raise ValueError(f"Object '{name}' not found")
            
        obj = self.objects[name]
        pos, orn = p.getBasePositionAndOrientation(obj['id'])
        lin_vel, ang_vel = p.getBaseVelocity(obj['id'])
        
        return {
            'name': name,
            'position': pos,
            'orientation': orn,
            'linear_velocity': lin_vel,
            'angular_velocity': ang_vel,
            'fixed_base': obj['fixed_base']
        }
    
    def get_state(self) -> Dict[str, Any]:
        """Get the current state of the entire simulation."""
        return {
            'robots': {name: self.get_robot_state(name) for name in self.robots},
            'objects': {name: self.get_object_state(name) for name in self.objects}
        }
