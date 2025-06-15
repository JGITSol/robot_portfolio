"""
Production Line Simulation with Multiple Robot Arms

This demo showcases a realistic production line with:
- A conveyor belt system
- Three robot arms with different roles
- Object tracking and management
- Coordinated task execution
- Real-time monitoring and control
"""
import os
import sys
import time
import math
import numpy as np
import pybullet as p
from enum import Enum
from typing import Dict, List, Optional, Tuple, Any

# Add the parent directory to the Python path to allow importing the package
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_arm_simulator.simulation.environment import SimulationEnvironment
from robot_arm_simulator.simulation.conveyor import ConveyorBelt
from robot_arm_simulator.robots import create_robot
from robot_arm_simulator.grippers import create_gripper

# Constants
CONFIG_DIR = os.path.join(os.path.dirname(__file__), '..', 'config')
MODELS_DIR = os.path.join(os.path.dirname(__file__), '..', 'models')

class RobotRole(Enum):
    """Define roles for different robots in the production line."""
    PICKER = 1      # Picks objects from the conveyor
    PROCESSOR = 2   # Processes the objects (e.g., assembly, inspection)
    PACKER = 3      # Packs the finished products

class ProductionObject:
    """Represents an object in the production line."""
    
    def __init__(self, 
                 obj_id: int, 
                 obj_type: str, 
                 position: List[float],
                 orientation: List[float] = None):
        """Initialize a production object.
        
        Args:
            obj_id: PyBullet object ID
            obj_type: Type of the object (e.g., 'box', 'bottle')
            position: [x, y, z] position in world coordinates
            orientation: [x, y, z, w] quaternion orientation
        """
        self.obj_id = obj_id
        self.obj_type = obj_type
        self.position = np.array(position, dtype=np.float32)
        self.orientation = np.array(orientation if orientation is not None else [0, 0, 0, 1], dtype=np.float32)
        self.velocity = np.zeros(3, dtype=np.float32)
        self.angular_velocity = np.zeros(3, dtype=np.float32)
        self.conveyor_id = None  # ID of the conveyor this object is on
        self.robot_id = None      # ID of the robot currently holding this object
        self.processed = False    # Whether the object has been processed
        self.timestamp = time.time()  # Creation time
    
    def update(self, physics_client):
        """Update the object's state from the physics simulation."""
        pos, orn = physics_client.getBasePositionAndOrientation(self.obj_id)
        lin_vel, ang_vel = physics_client.getBaseVelocity(self.obj_id)
        
        self.position = np.array(pos, dtype=np.float32)
        self.orientation = np.array(orn, dtype=np.float32)
        self.velocity = np.array(lin_vel, dtype=np.float32)
        self.angular_velocity = np.array(ang_vel, dtype=np.float32)
    
    def is_stable(self, threshold: float = 0.01) -> bool:
        """Check if the object is stable (not moving much).
        
        Args:
            threshold: Maximum velocity magnitude to be considered stable
            
        Returns:
            bool: True if the object is stable
        """
        return np.linalg.norm(self.velocity) < threshold and \
               np.linalg.norm(self.angular_velocity) < threshold
    
    def __str__(self) -> str:
        return f"{self.obj_type.capitalize()}(id={self.obj_id}, pos={self.position[:2]}, " \
               f"on_conveyor={self.conveyor_id is not None}, held_by_robot={self.robot_id is not None})"

class ProductionLineSimulation:
    """Main class for the production line simulation."""
    
    def __init__(self, gui: bool = True, realtime: bool = True):
        """Initialize the production line simulation.
        
        Args:
            gui: Whether to show the GUI
            realtime: Whether to run in real-time
        """
        self.gui = gui
        self.realtime = realtime
        self.env = None
        self.conveyor = None
        self.robots = {}
        self.objects = {}  # obj_id -> ProductionObject
        self.object_counter = 0
        self.running = False
        self.spawn_timer = 0
        self.spawn_interval = 3.0  # seconds
        self.available_types = ['box', 'bottle', 'gear']
        
        # Task management
        self.tasks = []
        self.current_task = None
        self.task_start_time = 0
        
        # Statistics
        self.stats = {
            'objects_created': 0,
            'objects_processed': 0,
            'objects_packed': 0,
            'processing_time': 0.0,
            'avg_processing_time': 0.0
        }
    
    def initialize(self):
        """Initialize the simulation environment and components."""
        print("Initializing production line simulation...")
        
        # Create simulation environment
        self.env = SimulationEnvironment(gui=self.gui, realtime=self.realtime)
        
        # Set up the scene
        self._setup_scene()
        
        # Add robots
        self._add_robots()
        
        # Add conveyor
        self._add_conveyor()
        
        # Add UI elements
        if self.gui:
            self._setup_ui()
        
        print("Production line simulation initialized.")
    
    def _setup_scene(self):
        """Set up the scene with floor, lighting, etc."""
        # Load plane
        self.env.add_plane()
        
        # Add some basic lighting
        if self.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            p.resetDebugVisualizerCamera(
                cameraDistance=3.0,
                cameraYaw=45,
                cameraPitch=-30,
                cameraTargetPosition=[0, 0, 0.5]
            )
    
    def _add_robots(self):
        """Add robots to the simulation."""
        # Robot configurations
        robot_configs = [
            {
                'name': 'picker_robot',
                'type': 'ur5',
                'position': [-1.5, -0.5, 0],
                'orientation': [0, 0, 0, 1],
                'gripper': 'parallel',
                'role': RobotRole.PICKER
            },
            {
                'name': 'processor_robot',
                'type': 'ur5',
                'position': [0, -0.5, 0],
                'orientation': [0, 0, 0, 1],
                'gripper': 'suction',
                'role': RobotRole.PROCESSOR
            },
            {
                'name': 'packer_robot',
                'type': 'ur5',
                'position': [1.5, -0.5, 0],
                'orientation': [0, 0, 0, 1],
                'gripper': 'parallel',
                'role': RobotRole.PACKER
            }
        ]
        
        # Add each robot
        for config in robot_configs:
            # Create robot
            robot = self.env.add_robot(
                robot_type=config['type'],
                name=config['name'],
                position=config['position'],
                orientation=config['orientation']
            )
            
            # Add gripper
            if 'gripper' in config:
                self.env.add_gripper(
                    robot_name=config['name'],
                    gripper_type=config['gripper']
                )
            
            # Store robot with role
            self.robots[config['name']] = {
                'robot': robot,
                'role': config['role'],
                'busy': False,
                'current_task': None,
                'last_update': time.time()
            }
    
    def _add_conveyor(self):
        """Add a conveyor belt to the simulation."""
        self.conveyor = ConveyorBelt(
            physics_client=self.env.physics_client,
            position=[0, 0.5, 0],
            orientation=p.getQuaternionFromEuler([0, 0, 0]),
            length=6.0,
            width=0.5,
            height=0.1,
            speed=0.3,
            max_speed=1.0,
            texture=os.path.join(pybullet_data.getDataPath(), "checker_blue.png")
        )
        
        # Start the conveyor
        self.conveyor.start()
    
    def _setup_ui(self):
        """Set up the user interface."""
        # Add debug text for status
        self.status_text_id = p.addUserDebugText(
            "Production Line Simulation\nStatus: Initializing...",
            textPosition=[-1, 0, 1.5],
            textColorRGB=[1, 1, 1],
            textSize=1.2,
            lifeTime=0
        )
        
        # Add controls help
        self.help_text_id = p.addUserDebugText(
            "Controls:\n"
            "  SPACE: Toggle simulation\n"
            "  C: Spawn object\n"
            "  S: Toggle conveyor\n"
            "  R: Reverse conveyor\n"
            "  Q: Quit",
            textPosition=[-1, 0, 0.5],
            textColorRGB=[0.8, 0.8, 0.8],
            textSize=1.0,
            lifeTime=0
        )
    
    def spawn_object(self, obj_type: str = None, position: List[float] = None):
        """Spawn a new object on the conveyor.
        
        Args:
            obj_type: Type of object to spawn. If None, a random type is chosen.
            position: [x, y, z] position to spawn the object. If None, uses default spawn position.
        """
        if obj_type is None:
            obj_type = np.random.choice(self.available_types)
        
        if position is None:
            # Default spawn position at the start of the conveyor
            position = [-2.5, 0.5, 0.2]
        
        # Create the object
        if obj_type == 'box':
            obj_id = p.loadURDF(
                os.path.join(pybullet_data.getDataPath(), "cube.urdf"),
                basePosition=position,
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                globalScaling=0.1
            )
        elif obj_type == 'bottle':
            obj_id = p.loadURDF(
                os.path.join(pybullet_data.getDataPath(), "lego/lego.urdf"),
                basePosition=position,
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                globalScaling=0.5
            )
        elif obj_type == 'gear':
            obj_id = p.loadURDF(
                os.path.join(pybullet_data.getDataPath(), "duck_vhacd.urdf"),
                basePosition=position,
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                globalScaling=0.1
            )
        else:
            # Default to a simple box
            obj_id = p.loadURDF(
                os.path.join(pybullet_data.getDataPath(), "cube.urdf"),
                basePosition=position,
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                globalScaling=0.1
            )
        
        # Create and track the object
        obj = ProductionObject(obj_id, obj_type, position)
        self.objects[obj_id] = obj
        self.conveyor.add_object(obj_id)
        
        # Update statistics
        self.stats['objects_created'] += 1
        self.object_counter += 1
        
        print(f"Spawned {obj_type} with ID {obj_id}")
        return obj_id
    
    def update(self, time_step: float):
        """Update the simulation state.
        
        Args:
            time_step: Time step in seconds
        """
        if not self.running:
            return
        
        # Update conveyor
        self.conveyor.update(time_step)
        
        # Update object states
        self._update_objects()
        
        # Spawn new objects at intervals
        self._handle_object_spawning(time_step)
        
        # Update robot tasks
        self._update_robots(time_step)
        
        # Update UI
        self._update_ui()
    
    def _update_objects(self):
        """Update the state of all objects."""
        # Remove objects that no longer exist
        for obj_id in list(self.objects.keys()):
            if not (0 <= obj_id < p.getNumBodies()):
                if obj_id in self.conveyor.tracked_objects:
                    self.conveyor.remove_object(obj_id)
                del self.objects[obj_id]
                continue
            
            # Update object state
            obj = self.objects[obj_id]
            obj.update(self.env.physics_client)
            
            # Check if object fell off the conveyor
            if not self.conveyor.is_object_on_conveyor(obj_id):
                self.conveyor.remove_object(obj_id)
    
    def _handle_object_spawning(self, time_step: float):
        """Handle automatic object spawning."""
        self.spawn_timer += time_step
        if self.spawn_timer >= self.spawn_interval:
            self.spawn_object()
            self.spawn_timer = 0
    
    def _update_robots(self, time_step: float):
        """Update robot states and execute tasks."""
        current_time = time.time()
        
        for robot_name, robot_data in self.robots.items():
            robot = robot_data['robot']
            
            # Skip if robot is busy
            if robot_data['busy']:
                # Check if current task is complete
                if self._is_task_complete(robot_data['current_task'], robot_data):
                    robot_data['busy'] = False
                    robot_data['current_task'] = None
                    print(f"{robot_name} completed task")
                continue
            
            # Find a new task for the robot
            task = self._find_task_for_robot(robot_name, robot_data['role'])
            if task:
                robot_data['current_task'] = task
                robot_data['busy'] = True
                robot_data['last_update'] = current_time
                print(f"{robot_name} started task: {task['type']}")
    
    def _is_task_complete(self, task: Dict[str, Any], robot_data: Dict[str, Any]) -> bool:
        """Check if a robot's current task is complete."""
        if task is None:
            return True
            
        # Simple timeout for now
        return (time.time() - robot_data['last_update']) > 5.0  # 5 second tasks
    
    def _find_task_for_robot(self, robot_name: str, role: RobotRole) -> Optional[Dict[str, Any]]:
        """Find an appropriate task for a robot based on its role."""
        # This is a simplified task assignment logic
        # In a real implementation, this would be more sophisticated
        
        # For the picker robot: pick objects from the conveyor
        if role == RobotRole.PICKER:
            # Find an object on the conveyor that's not being handled
            for obj_id, obj in self.objects.items():
                if obj.conveyor_id is not None and obj.robot_id is None:
                    return {
                        'type': 'pick',
                        'object_id': obj_id,
                        'target_position': [0, -0.5, 0.2],  # Place on worktable
                        'timeout': 10.0
                    }
        
        # For the processor robot: process objects on the worktable
        elif role == RobotRole.PROCESSOR:
            # Find an unprocessed object on the worktable
            for obj_id, obj in self.objects.items():
                if obj.robot_id is None and not obj.processed and \
                   obj.position[1] < -0.3:  # On the worktable
                    return {
                        'type': 'process',
                        'object_id': obj_id,
                        'timeout': 8.0
                    }
        
        # For the packer robot: pack processed objects
        elif role == RobotRole.PACKER:
            # Find a processed object on the worktable
            for obj_id, obj in self.objects.items():
                if obj.robot_id is None and obj.processed and \
                   obj.position[1] < -0.3:  # On the worktable
                    return {
                        'type': 'pack',
                        'object_id': obj_id,
                        'target_position': [1.5, 0.5, 0.1],  # Place on output conveyor
                        'timeout': 12.0
                    }
        
        return None
    
    def _update_ui(self):
        """Update the user interface."""
        if not self.gui:
            return
        
        # Update status text
        status_text = (
            f"Production Line Simulation\n"
            f"Status: {'Running' if self.running else 'Paused'}\n"
            f"Objects: {len(self.objects)} created, {self.stats['objects_processed']} processed\n"
            f"Conveyor: {'ON' if self.conveyor.running else 'OFF'} "
            f"({abs(self.conveyor.speed):.1f} m/s {'→' if self.conveyor.direction > 0 else '←'})\n"
            f"Robots: {sum(1 for r in self.robots.values() if not r['busy'])}/{len(self.robots)} idle"
        )
        
        p.addUserDebugText(
            status_text,
            textPosition=[-1, 0, 1.5],
            textColorRGB=[1, 1, 1],
            textSize=1.0,
            lifeTime=0,
            replaceItemUniqueId=self.status_text_id
        )
    
    def run(self):
        """Run the simulation."""
        if self.env is None:
            self.initialize()
        
        self.running = True
        print("Starting production line simulation. Press SPACE to pause/resume, Q to quit.")
        
        # Main simulation loop
        try:
            while True:
                # Handle keyboard input
                keys = p.getKeyboardEvents()
                
                # Toggle simulation with SPACE
                if ord(' ') in keys and keys[ord(' ')] & p.KEY_WAS_TRIGGERED:
                    self.running = not self.running
                    print(f"Simulation {'paused' if not self.running else 'resumed'}")
                
                # Spawn object with C
                if ord('c') in keys and keys[ord('c')] & p.KEY_WAS_TRIGGERED:
                    self.spawn_object()
                
                # Toggle conveyor with S
                if ord('s') in keys and keys[ord('s')] & p.KEY_WAS_TRIGGERED:
                    if self.conveyor.running:
                        self.conveyor.stop()
                    else:
                        self.conveyor.start()
                
                # Reverse conveyor with R
                if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
                    self.conveyor.toggle_direction()
                
                # Quit with Q
                if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
                    print("Quitting simulation...")
                    break
                
                # Step the simulation
                self.env.step()
                
                # Update the production line state
                self.update(self.env.time_step)
                
                # Sleep to maintain real-time simulation
                if self.realtime:
                    time.sleep(self.env.time_step)
        
        except KeyboardInterrupt:
            print("Simulation stopped by user")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        if self.env:
            self.env.cleanup()
        print("Simulation cleaned up.")

if __name__ == "__main__":
    # Create and run the production line simulation
    sim = ProductionLineSimulation(gui=True, realtime=True)
    sim.run()
