"""Production line simulation with multiple robot arms."""
import pybullet as p
import pybullet_data
import time
import numpy as np
from typing import List, Dict, Any, Optional, Tuple
import random
import os
import sys

# Add the src directory to the path
current_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.dirname(current_dir)
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)

from robot_arm.robot import RobotArm
from robot_arm.grippers import create_gripper

class ProductionLine:
    """Simulation of a production line with multiple robot arms."""
    
    def __init__(self, gui: bool = True):
        """Initialize the production line simulation."""
        # Physics client setup
        self.physics_client_id = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1./240.)
        
        # Store the pybullet module reference
        self.p = p
        
        # Load environment
        print("Loading plane...")
        self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.physics_client_id)
        print(f"Plane loaded with ID {self.plane_id}")
        
        # Setup conveyor
        self.conveyor_length = 6.0
        self.conveyor_width = 0.5
        self.conveyor_speed = 0.5  # m/s
        self.setup_conveyor()
        
        # Setup robots with different grippers
        self.robots = self.setup_robots()
        
        # Setup objects to be handled
        self.setup_objects()
        
        # Track objects on conveyor
        self.objects = []
        self.spawn_timer = 0
        self.spawn_interval = 3.0  # seconds
        
        # Setup bins for each robot
        self.setup_bins()
    
    def setup_conveyor(self):
        """Create a conveyor belt in the simulation."""
        conveyor_height = 0.1
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[self.conveyor_length/2, self.conveyor_width/2, 0.05],
            rgbaColor=[0.4, 0.4, 0.4, 1]
        )
        collision_shape_id = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[self.conveyor_length/2, self.conveyor_width/2, 0.05]
        )
        self.conveyor_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape_id,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=[0, 0, conveyor_height/2]
        )
        
        # Add texture for better visualization
        p.changeVisualShape(
            self.conveyor_id, -1,
            textureUniqueId=p.loadTexture("checker_blue.png"),
            rgbaColor=[1, 1, 1, 1]
        )
    
    def setup_robots(self) -> List[RobotArm]:
        """Initialize robot arms with different grippers."""
        print("\n=== Setting up robots ===")
        robots = []
        robot_positions = [
            (-2, 0, 0),  # Robot 1: Left
            (0, 0, 0),   # Robot 2: Middle
            (2, 0, 0)    # Robot 3: Right
        ]
        
        # Different gripper types for different robots
        gripper_types = ["parallel", "suction", "parallel"]
        
        for i, (pos, gripper_type) in enumerate(zip(robot_positions, gripper_types)):
            try:
                print(f"\n=== Initializing Robot {i+1} ===")
                print(f"Position: {pos}")
                print(f"Gripper type: {gripper_type}")
                
                robot = RobotArm(
                    physics_client_id=self.physics_client_id,
                    position=list(pos),
                    gripper_type=gripper_type,
                    name=f"robot_{i+1}"
                )
                
                # Move to initial position
                print("Moving to rest pose...")
                robot.move_to_rest_pose()
                robots.append(robot)
                print(f"=== Robot {i+1} initialized successfully ===")
                
            except Exception as e:
                print(f"ERROR: Failed to initialize robot {i+1}")
                print(f"Error details: {str(e)}")
                import traceback
                traceback.print_exc()
                raise
                
        print("\nAll robots initialized")
        return robots
            
        return robots
    
    def setup_objects(self):
        """Define object types and their properties."""
        self.object_types = {
            "decahedron": {
                "color": [1, 0, 0, 1],  # Red
                "robot_idx": 0,  # Handled by first robot
                "size": 0.1,
                "shape": p.GEOM_MESH,
                "mesh": "dodecahedron.obj",
                "mass": 0.1
            },
            "tetrahedron": {
                "color": [0, 1, 0, 1],  # Green
                "robot_idx": 1,  # Handled by second robot
                "size": 0.1,
                "shape": p.GEOM_MESH,
                "mesh": "tetrahedron.obj",
                "mass": 0.08
            },
            "cylinder": {
                "color": [0, 0, 1, 1],  # Blue
                "robot_idx": 2,  # Handled by third robot
                "size": 0.1,
                "shape": p.GEOM_CYLINDER,
                "radius": 0.05,
                "height": 0.1,
                "mass": 0.05
            }
        }
    
    def setup_bins(self):
        """Create bins for each robot to place objects."""
        self.bins = []
        bin_size = 0.3
        
        for i, robot in enumerate(self.robots):
            # Position bin behind the robot
            bin_pos = [robot.base_position[0], robot.base_position[1] - 0.8, bin_size/2]
            
            # Create bin
            bin_id = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[bin_size, bin_size, bin_size/2]
            )
            bin_vis = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[bin_size, bin_size, bin_size/2],
                rgba=[0.8, 0.8, 0.8, 0.7]  # Semi-transparent
            )
            bin_body = p.createMultiBody(
                baseMass=0,  # Static
                baseCollisionShapeIndex=bin_id,
                baseVisualShapeIndex=bin_vis,
                basePosition=bin_pos
            )
            
            # Add label
            p.addUserDebugText(
                f"Bin {i+1}",
                textPosition=[bin_pos[0], bin_pos[1], bin_pos[2] + bin_size/2 + 0.1],
                textColorRGB=[0, 0, 0],
                textSize=1.0
            )
            
            self.bins.append({
                "id": bin_body,
                "position": bin_pos,
                "robot_idx": i
            })
    
    def spawn_object(self, obj_type: str):
        """Spawn an object of the specified type on the conveyor."""
        try:
            obj_info = self.object_types[obj_type]
            start_pos = [-self.conveyor_length/2 + 0.5, 0, 0.2]  # Slightly above conveyor
            
            print(f"Spawning {obj_type} at {start_pos}")
            
            # For simplicity, we'll use simple shapes instead of meshes
            if obj_type == "decahedron":
                # Use a box as a stand-in for decahedron
                col_id = p.createCollisionShape(
                    p.GEOM_BOX,
                    halfExtents=[0.05, 0.05, 0.05]
                )
                vis_id = p.createVisualShape(
                    p.GEOM_BOX,
                    halfExtents=[0.05, 0.05, 0.05],
                    rgbaColor=obj_info["color"]
                )
            elif obj_type == "tetrahedron":
                # Use a smaller box for tetrahedron
                col_id = p.createCollisionShape(
                    p.GEOM_BOX,
                    halfExtents=[0.04, 0.04, 0.04]
                )
                vis_id = p.createVisualShape(
                    p.GEOM_BOX,
                    halfExtents=[0.04, 0.04, 0.04],
                    rgbaColor=obj_info["color"]
                )
            else:  # CYLINDER
                col_id = p.createCollisionShape(
                    p.GEOM_CYLINDER,
                    radius=0.03,
                    height=0.1
                )
                vis_id = p.createVisualShape(
                    p.GEOM_CYLINDER,
                    radius=0.03,
                    length=0.1,
                    rgbaColor=obj_info["color"]
                )
            
            # Create the object
            obj_id = p.createMultiBody(
                baseMass=obj_info["mass"],
                baseCollisionShapeIndex=col_id,
                baseVisualShapeIndex=vis_id,
                basePosition=start_pos
            )
            
            # Store object info
            self.objects.append({
                "id": obj_id,
                "type": obj_type,
                "picked": False,
                "in_bin": False,
                "robot_idx": obj_info["robot_idx"]
            })
            
            print(f"Spawned {obj_type} with ID {obj_id}")
            
        except Exception as e:
            print(f"Error spawning object: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def update_conveyor(self, direction=1):
        """Update positions of objects on the conveyor."""
        for obj in self.objects:
            if not obj["picked"] and not obj["in_bin"]:
                pos, orn = p.getBasePositionAndOrientation(obj["id"])
                new_pos = [
                    pos[0] + self.conveyor_speed * (1./240.) * direction,  # 240 Hz simulation
                    pos[1],
                    pos[2]
                ]
                p.resetBasePositionAndOrientation(obj["id"], new_pos, orn)
    
    def step_simulation(self, conveyor_running=True, conveyor_direction=1):
        """Step the simulation, optionally moving the conveyor."""
        # Spawn new objects at intervals
        current_time = time.time()
        if not hasattr(self, '_last_spawn'):
            self._last_spawn = current_time
        if not hasattr(self, '_processed_objects'):
            self._processed_objects = 0
        if (current_time - self._last_spawn > self.spawn_interval and 
            len([o for o in self.objects if not o["in_bin"]]) < 5):
            obj_type = random.choice(list(self.object_types.keys()))
            self.spawn_object(obj_type)
            self._last_spawn = current_time
            self._processed_objects += 1
        # Update conveyor if running
        if conveyor_running:
            self.update_conveyor(direction=conveyor_direction)
        # Robots pick/place as in run()
        for i, robot in enumerate(self.robots):
            # Find objects this robot should pick
            target_obj = None
            for obj in self.objects:
                if (not obj["picked"] and not obj["in_bin"] and 
                    obj["robot_idx"] == i):
                    pos, _ = p.getBasePositionAndOrientation(obj["id"])
                    # Check if object is in front of this robot
                    if abs(pos[0] - robot.base_position[0]) < 0.3:
                        target_obj = obj
                        break
            if target_obj:
                # Get object position and bin position
                obj_pos, _ = p.getBasePositionAndOrientation(target_obj["id"])
                bin_pos = [
                    self.bins[i]["position"][0],
                    self.bins[i]["position"][1],
                    self.bins[i]["position"][2] + 0.1  # Slightly above bin
                ]
                # Pick and place the object
                if robot.pick(obj_pos):
                    target_obj["picked"] = True
                    # Small delay to simulate grasping
                    for _ in range(50):  # ~0.2s at 240Hz
                        p.stepSimulation()
                        time.sleep(1./240.)
                    if robot.place(bin_pos):
                        target_obj["in_bin"] = True
                        # Remove the object after placing
                        p.removeBody(target_obj["id"])
        # Step simulation
        p.stepSimulation()
        time.sleep(1./240.)

def main():
    """Run the production line simulation."""
    # Create and run the production line
    production_line = ProductionLine(gui=True)
    production_line.run()

if __name__ == "__main__":
    main()
