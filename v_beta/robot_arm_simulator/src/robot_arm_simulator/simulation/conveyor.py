"""Conveyor belt implementation for production line simulations."""
import numpy as np
import pybullet as p
from typing import List, Dict, Any, Optional, Tuple, Set
import time

class ConveyorBelt:
    """A conveyor belt that can transport objects with configurable properties."""
    
    def __init__(self,
                 physics_client: Any,
                 position: List[float] = (0, 0, 0),
                 orientation: List[float] = (0, 0, 0, 1),
                 length: float = 6.0,
                 width: float = 0.5,
                 height: float = 0.1,
                 speed: float = 0.5,
                 max_speed: float = 2.0,
                 texture: Optional[str] = None):
        """Initialize the conveyor belt.
        
        Args:
            physics_client: The PyBullet physics client
            position: [x, y, z] position of the conveyor center
            orientation: [x, y, z, w] quaternion orientation
            length: Length of the conveyor belt (x-axis)
            width: Width of the conveyor belt (y-axis)
            height: Height/thickness of the conveyor belt
            speed: Initial speed of the conveyor (m/s)
            max_speed: Maximum allowed speed (m/s)
            texture: Path to texture file for the conveyor surface
        """
        self.physics_client = physics_client
        self.position = np.array(position, dtype=np.float32)
        self.orientation = np.array(orientation, dtype=np.float32)
        self.length = float(length)
        self.width = float(width)
        self.height = float(height)
        self.speed = float(speed)
        self.max_speed = float(max_speed)
        self.running = False
        self.direction = 1  # 1 for forward, -1 for reverse
        self.texture = texture
        self.conveyor_id = None
        self.texture_id = None
        self.tracked_objects = set()  # Set of object IDs on the conveyor
        self.object_positions = {}  # Last known positions of objects
        self.object_velocities = {}  # Current velocities of objects
        
        # Create the conveyor belt
        self._create_conveyor()
    
    def _create_conveyor(self):
        """Create the conveyor belt in the physics simulation."""
        # Create collision and visual shapes
        collision_shape = self.physics_client.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[self.length/2, self.width/2, self.height/2]
        )
        
        visual_shape = self.physics_client.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[self.length/2, self.width/2, self.height/2],
            rgbaColor=[0.3, 0.3, 0.3, 1.0]  # Default gray color
        )
        
        # Create the conveyor body
        self.conveyor_id = self.physics_client.createMultiBody(
            baseMass=0,  # Fixed body
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=self.position,
            baseOrientation=self.orientation
        )
        
        # Apply texture if provided
        if self.texture:
            self.texture_id = self.physics_client.loadTexture(self.texture)
            self.physics_client.changeVisualShape(
                self.conveyor_id, 
                -1,  # All links
                textureUniqueId=self.texture_id
            )
    
    def add_object(self, object_id: int):
        """Add an object to be tracked by the conveyor.
        
        Args:
            object_id: The PyBullet object ID to track
        """
        self.tracked_objects.add(object_id)
        pos, _ = self.physics_client.getBasePositionAndOrientation(object_id)
        self.object_positions[object_id] = np.array(pos)
        self.object_velocities[object_id] = np.zeros(3)
    
    def remove_object(self, object_id: int):
        """Stop tracking an object.
        
        Args:
            object_id: The PyBullet object ID to stop tracking
        """
        if object_id in self.tracked_objects:
            self.tracked_objects.remove(object_id)
            if object_id in self.object_positions:
                del self.object_positions[object_id]
            if object_id in self.object_velocities:
                del self.object_velocities[object_id]
    
    def set_speed(self, speed: float):
        """Set the conveyor speed.
        
        Args:
            speed: Speed in m/s (positive or negative)
        """
        self.speed = max(-self.max_speed, min(self.max_speed, speed))
        self.direction = 1 if self.speed >= 0 else -1
    
    def start(self):
        """Start the conveyor."""
        self.running = True
    
    def stop(self):
        """Stop the conveyor."""
        self.running = False
    
    def toggle_direction(self):
        """Toggle the direction of the conveyor."""
        self.direction *= -1
        self.speed = abs(self.speed) * self.direction
    
    def is_object_on_conveyor(self, object_id: int) -> bool:
        """Check if an object is on the conveyor.
        
        Args:
            object_id: The PyBullet object ID to check
            
        Returns:
            bool: True if the object is on the conveyor
        """
        if object_id not in self.tracked_objects:
            return False
            
        # Get object position
        pos, _ = self.physics_client.getBasePositionAndOrientation(object_id)
        pos = np.array(pos)
        
        # Transform to local conveyor coordinates
        inv_pos, inv_orn = self.physics_client.invertTransform(
            self.position, self.orientation)
        local_pos, _ = self.physics_client.multiplyTransforms(
            inv_pos, inv_orn, pos, [0, 0, 0, 1])
        
        # Check if within conveyor bounds
        half_length = self.length / 2
        half_width = self.width / 2
        
        return (abs(local_pos[0]) <= half_length and 
                abs(local_pos[1]) <= half_width and
                abs(local_pos[2] - self.height/2) <= 0.05)  # Small tolerance for height
    
    def update(self, time_step: float):
        """Update the conveyor state.
        
        Args:
            time_step: Time step in seconds
        """
        if not self.running or self.speed == 0:
            return
            
        # Get the conveyor's forward direction in world coordinates
        rot_matrix = np.array(self.physics_client.getMatrixFromQuaternion(self.orientation)).reshape(3, 3)
        forward = rot_matrix @ np.array([1, 0, 0])  # Conveyor's forward direction
        
        # Apply forces to objects on the conveyor
        for obj_id in list(self.tracked_objects):
            if not self.is_object_on_conveyor(obj_id):
                self.remove_object(obj_id)
                continue
                
            # Get object properties
            mass = self.physics_client.getDynamicsInfo(obj_id, -1)[0]
            
            # Calculate target velocity (conveyor speed in world coordinates)
            target_velocity = forward * self.speed
            
            # Get current velocity
            current_velocity, _ = self.physics_client.getBaseVelocity(obj_id)
            current_velocity = np.array(current_velocity)
            
            # Calculate velocity error
            velocity_error = target_velocity - current_velocity
            
            # Apply force to reach target velocity (simple PD control)
            kp = 10.0  # Proportional gain
            kd = 2.0   # Derivative gain
            
            # Calculate acceleration needed
            acceleration = kp * velocity_error - kd * current_velocity
            
            # Apply force (F = m*a)
            force = mass * acceleration
            
            # Apply the force at the object's center of mass
            self.physics_client.applyExternalForce(
                obj_id, 
                -1,  # Apply to base
                forceObj=force.tolist(),
                posObj=[0, 0, 0],
                flags=p.WORLD_FRAME
            )
            
            # Update object tracking
            pos, _ = self.physics_client.getBasePositionAndOrientation(obj_id)
            self.object_positions[obj_id] = np.array(pos)
            self.object_velocities[obj_id] = np.array(current_velocity)
    
    def get_state(self) -> Dict[str, Any]:
        """Get the current state of the conveyor.
        
        Returns:
            Dict containing conveyor state
        """
        return {
            'running': self.running,
            'speed': self.speed,
            'direction': self.direction,
            'tracked_objects': list(self.tracked_objects),
            'position': self.position.tolist(),
            'orientation': self.orientation.tolist()
        }
    
    def reset(self):
        """Reset the conveyor to its initial state."""
        self.stop()
        self.speed = abs(self.speed)  # Reset to positive speed
        self.direction = 1
        self.tracked_objects.clear()
        self.object_positions.clear()
        self.object_velocities.clear()
    
    def __del__(self):
        """Clean up resources."""
        if hasattr(self, 'conveyor_id') and self.conveyor_id is not None:
            self.physics_client.removeBody(self.conveyor_id)
