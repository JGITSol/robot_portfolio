"""Sensor implementations for robotic simulation."""
import numpy as np
import pybullet as p
from typing import List, Dict, Any, Optional, Tuple

class Sensor:
    """Base class for sensors in the robotic simulation."""
    
    def __init__(self, name: str, update_rate: float = 10.0):
        """Initialize the sensor.
        
        Args:
            name: Name of the sensor
            update_rate: Frequency of sensor updates in Hz
        """
        self.name = name
        self.update_rate = update_rate
        self.update_interval = 1.0 / update_rate if update_rate > 0 else 0
        self.last_update = 0.0
        self.data = None
        self.enabled = True
    
    def update(self, current_time: float, physics_client: Any) -> bool:
        """Update sensor readings if necessary.
        
        Args:
            current_time: Current simulation time in seconds
            physics_client: PyBullet physics client
            
        Returns:
            bool: True if sensor data was updated
        """
        if not self.enabled:
            return False
            
        if current_time - self.last_update >= self.update_interval:
            self._read_data(physics_client)
            self.last_update = current_time
            return True
        return False
    
    def _read_data(self, physics_client: Any):
        """Read sensor data. To be implemented by subclasses."""
        pass
    
    def get_data(self) -> Any:
        """Get the latest sensor data.
        
        Returns:
            The latest sensor readings
        """
        return self.data
    
    def enable(self):
        """Enable the sensor."""
        self.enabled = True
    
    def disable(self):
        """Disable the sensor."""
        self.enabled = False

class ProximitySensor(Sensor):
    """Simulates a proximity sensor for detecting objects within a range."""
    
    def __init__(self, 
                 name: str, 
                 body_id: int, 
                 link_id: int, 
                 position: List[float],
                 direction: List[float],
                 max_range: float = 1.0,
                 min_range: float = 0.05,
                 fov: float = 30.0,
                 update_rate: float = 20.0):
        """Initialize the proximity sensor.
        
        Args:
            name: Name of the sensor
            body_id: PyBullet body ID to which the sensor is attached
            link_id: Link ID on the body where sensor is attached (-1 for base)
            position: Local position of sensor on the link [x, y, z]
            direction: Direction vector of sensor in local frame [x, y, z]
            max_range: Maximum detection range in meters
            min_range: Minimum detection range in meters
            fov: Field of view in degrees
            update_rate: Frequency of sensor updates in Hz
        """
        super().__init__(name, update_rate)
        self.body_id = body_id
        self.link_id = link_id
        self.position = np.array(position, dtype=np.float32)
        self.direction = np.array(direction, dtype=np.float32) / np.linalg.norm(direction)
        self.max_range = max_range
        self.min_range = min_range
        self.fov = np.deg2rad(fov)
        self.data = {'distance': None, 'object_id': None}
    
    def _read_data(self, physics_client: Any):
        """Read proximity sensor data using ray casting."""
        # Get sensor position and orientation in world frame
        if self.link_id >= 0:
            link_state = physics_client.getLinkState(self.body_id, self.link_id)
            link_pos = np.array(link_state[4])
            link_orn = np.array(link_state[5])
            # Transform sensor position from local to world
            sensor_pos = link_pos + np.array(physics_client.multiplyTransforms(
                [0, 0, 0], link_orn, self.position, [0, 0, 0, 1])[0])
            # Transform direction vector to world frame
            sensor_dir = np.array(physics_client.multiplyTransforms(
                [0, 0, 0], link_orn, self.direction, [0, 0, 0, 1])[0])
        else:
            base_pos, base_orn = physics_client.getBasePositionAndOrientation(self.body_id)
            sensor_pos = np.array(base_pos) + np.array(physics_client.multiplyTransforms(
                [0, 0, 0], base_orn, self.position, [0, 0, 0, 1])[0])
            sensor_dir = np.array(physics_client.multiplyTransforms(
                [0, 0, 0], base_orn, self.direction, [0, 0, 0, 1])[0])
        
        # Compute end point for ray
        ray_end = sensor_pos + sensor_dir * self.max_range
        
        # Cast ray
        ray_result = physics_client.rayTest(sensor_pos, ray_end)
        
        if ray_result[0] != -1:  # Hit something
            hit_object = ray_result[0]
            hit_fraction = ray_result[2]
            hit_distance = hit_fraction * self.max_range
            
            if hit_distance >= self.min_range:
                # Check if within field of view (for conical sensors)
                hit_pos = np.array(ray_result[3])
                to_hit = hit_pos - sensor_pos
                hit_dir = to_hit / np.linalg.norm(to_hit)
                angle = np.arccos(np.clip(np.dot(sensor_dir, hit_dir), -1.0, 1.0))
                
                if angle <= self.fov / 2.0:
                    self.data = {'distance': hit_distance, 'object_id': hit_object}
                    return
        
        # No valid detection
        self.data = {'distance': None, 'object_id': None}

class CameraSensor(Sensor):
    """Simulates a camera sensor for object detection and image processing."""
    
    def __init__(self,
                 name: str,
                 body_id: int,
                 link_id: int,
                 position: List[float],
                 orientation: List[float],
                 width: int = 640,
                 height: int = 480,
                 fov: float = 60.0,
                 near: float = 0.1,
                 far: float = 10.0,
                 update_rate: float = 5.0):
        """Initialize the camera sensor.
        
        Args:
            name: Name of the sensor
            body_id: PyBullet body ID to which the camera is attached
            link_id: Link ID on the body where camera is attached (-1 for base)
            position: Local position of camera on the link [x, y, z]
            orientation: Local orientation as Euler angles [roll, pitch, yaw]
            width: Image width in pixels
            height: Image height in pixels
            fov: Field of view in degrees
            near: Near clipping plane distance
            far: Far clipping plane distance
            update_rate: Frequency of camera updates in Hz
        """
        super().__init__(name, update_rate)
        self.body_id = body_id
        self.link_id = link_id
        self.position = np.array(position, dtype=np.float32)
        self.orientation = np.array(orientation, dtype=np.float32)
        self.width = width
        self.height = height
        self.fov = fov
        self.aspect = width / height
        self.near = near
        self.far = far
        self.projection_matrix = p.computeProjectionMatrixFOV(
            fov, self.aspect, near, far)
        self.data = {
            'rgb': None,
            'depth': None,
            'segmentation': None,
            'objects': []
        }
    
    def _read_data(self, physics_client: Any):
        """Read camera data and perform basic object detection."""
        # Get camera position and orientation in world frame
        if self.link_id >= 0:
            link_state = physics_client.getLinkState(self.body_id, self.link_id)
            link_pos = np.array(link_state[4])
            link_orn = np.array(link_state[5])
            # Transform camera position from local to world
            cam_pos = link_pos + np.array(physics_client.multiplyTransforms(
                [0, 0, 0], link_orn, self.position, [0, 0, 0, 1])[0])
            # Compute camera orientation
            local_orn = p.getQuaternionFromEuler(self.orientation)
            cam_orn = np.array(physics_client.multiplyTransforms(
                [0, 0, 0], link_orn, [0, 0, 0], local_orn)[1])
        else:
            base_pos, base_orn = physics_client.getBasePositionAndOrientation(self.body_id)
            cam_pos = np.array(base_pos) + np.array(physics_client.multiplyTransforms(
                [0, 0, 0], base_orn, self.position, [0, 0, 0, 1])[0])
            local_orn = p.getQuaternionFromEuler(self.orientation)
            cam_orn = np.array(physics_client.multiplyTransforms(
                [0, 0, 0], base_orn, [0, 0, 0], local_orn)[1])
        
        # Compute view matrix
        cam_rot_matrix = np.array(p.getMatrixFromQuaternion(cam_orn)).reshape(3, 3)
        cam_target = cam_pos + cam_rot_matrix[:, 0]  # Assume camera looks along x-axis after orientation
        cam_up = cam_rot_matrix[:, 2]  # Up vector along z-axis after orientation
        view_matrix = p.computeViewMatrix(cam_pos, cam_target, cam_up)
        
        # Capture images
        images = physics_client.getCameraImage(
            self.width,
            self.height,
            view_matrix,
            self.projection_matrix,
            shadow=True,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        
        # Store image data
        self.data['rgb'] = np.array(images[2]).reshape(self.height, self.width, 4)
        self.data['depth'] = np.array(images[3]).reshape(self.height, self.width)
        self.data['segmentation'] = np.array(images[4]).reshape(self.height, self.width)
        
        # Perform basic object detection by analyzing segmentation mask
        unique_ids = np.unique(self.data['segmentation'])
        detected_objects = []
        for obj_id in unique_ids:
            if obj_id >= 0:  # Ignore background (-1)
                # Calculate centroid in image coordinates
                mask = (self.data['segmentation'] == obj_id)
                if np.sum(mask) > 100:  # Ignore very small detections
                    y_coords, x_coords = np.where(mask)
                    cx = int(np.mean(x_coords))
                    cy = int(np.mean(y_coords))
                    depth = self.data['depth'][cy, cx]
                    detected_objects.append({
                        'id': obj_id,
                        'center': [cx, cy],
                        'depth': depth,
                        'size': np.sum(mask)
                    })
        
        self.data['objects'] = detected_objects

class ForceTorqueSensor(Sensor):
    """Simulates a force-torque sensor for measuring forces and torques at a joint or link."""
    
    def __init__(self, 
                 name: str, 
                 body_id: int, 
                 joint_id: int,
                 update_rate: float = 100.0):
        """Initialize the force-torque sensor.
        
        Args:
            name: Name of the sensor
            body_id: PyBullet body ID to which the sensor is attached
            joint_id: Joint ID where forces and torques are measured
            update_rate: Frequency of sensor updates in Hz
        """
        super().__init__(name, update_rate)
        self.body_id = body_id
        self.joint_id = joint_id
        self.data = {
            'force': [0.0, 0.0, 0.0],
            'torque': [0.0, 0.0, 0.0]
        }
    
    def _read_data(self, physics_client: Any):
        """Read force and torque data from the joint."""
        # Enable joint feedback to get forces and torques
        physics_client.enableJointForceTorqueSensor(
            self.body_id, self.joint_id, enableSensor=1)
        
        # Get joint reaction forces
        joint_state = physics_client.getJointState(self.body_id, self.joint_id)
        
        # Joint reaction forces are in the joint frame
        self.data = {
            'force': joint_state[2][:3],   # Reaction force
            'torque': joint_state[2][3:]    # Reaction torque
        }
