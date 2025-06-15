"""Advanced trajectory planning for robotic arms."""
import numpy as np
from typing import List, Dict, Any, Optional, Tuple
import time

class TrajectoryPlanner:
    """Advanced trajectory planner for smooth and optimized robot movements."""
    
    def __init__(self, 
                 num_joints: int,
                 max_velocity: float = 1.0,
                 max_acceleration: float = 2.0,
                 time_step: float = 1.0/240.0,
                 planning_horizon: float = 2.0):
        """Initialize the trajectory planner.
        
        Args:
            num_joints: Number of robot joints
            max_velocity: Maximum joint velocity (rad/s)
            max_acceleration: Maximum joint acceleration (rad/s^2)
            time_step: Simulation time step in seconds
            planning_horizon: Time horizon for planning in seconds
        """
        self.num_joints = num_joints
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.time_step = time_step
        self.planning_horizon = planning_horizon
        self.num_steps = int(planning_horizon / time_step)
        
        # Current trajectory
        self.current_trajectory = None
        self.trajectory_start_time = 0.0
        self.trajectory_index = 0
    
    def plan_trajectory(self, 
                        start_positions: List[float],
                        target_positions: List[float],
                        start_velocities: List[float] = None,
                        target_velocities: List[float] = None,
                        current_time: float = 0.0) -> Dict[str, Any]:
        """Plan a smooth trajectory from start to target positions.
        
        Args:
            start_positions: Starting joint positions (radians)
            target_positions: Target joint positions (radians)
            start_velocities: Starting joint velocities (rad/s), default zero
            target_velocities: Target joint velocities (rad/s), default zero
            current_time: Current simulation time
            
        Returns:
            Dictionary containing the planned trajectory
        """
        if start_velocities is None:
            start_velocities = [0.0] * self.num_joints
        if target_velocities is None:
            target_velocities = [0.0] * self.num_joints
        
        # Convert to numpy arrays
        q0 = np.array(start_positions)
        qf = np.array(target_positions)
        v0 = np.array(start_velocities)
        vf = np.array(target_velocities)
        
        # Estimate minimum time required based on velocity and acceleration limits
        delta_q = np.abs(qf - q0)
        # Time to accelerate to max velocity
        t_acc = self.max_velocity / self.max_acceleration
        # Distance covered during acceleration
        d_acc = 0.5 * self.max_acceleration * t_acc * t_acc
        # For each joint, compute required time
        times = []
        for i in range(self.num_joints):
            if delta_q[i] <= 2 * d_acc:
                # Triangular profile (never reaches max velocity)
                t = np.sqrt(2 * delta_q[i] / self.max_acceleration)
            else:
                # Trapezoidal profile
                d_cruise = delta_q[i] - 2 * d_acc
                t_cruise = d_cruise / self.max_velocity
                t = 2 * t_acc + t_cruise
            times.append(t)
        
        # Use the longest time among all joints
        duration = max(times) if times else self.planning_horizon
        duration = min(self.planning_horizon, max(0.5, duration))  # Limit duration
        num_steps = int(duration / self.time_step)
        
        # Generate quintic polynomial trajectories for smooth motion
        positions = np.zeros((num_steps, self.num_joints))
        velocities = np.zeros((num_steps, self.num_joints))
        accelerations = np.zeros((num_steps, self.num_joints))
        
        t = np.linspace(0, duration, num_steps)
        
        for i in range(self.num_joints):
            # Coefficients for quintic polynomial
            # q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
            T = duration
            a0 = q0[i]
            a1 = v0[i]
            a2 = 0.0  # Assume zero initial acceleration
            a3 = (20 * (qf[i] - q0[i]) - (8 * vf[i] + 12 * v0[i]) * T + (a2 * 3) * T * T) / (2 * T * T * T)
            a4 = (-30 * (qf[i] - q0[i]) + (14 * vf[i] + 16 * v0[i]) * T - (a2 * 3) * T * T) / (2 * T * T * T * T)
            a5 = (12 * (qf[i] - q0[i]) - (6 * vf[i] + 6 * v0[i]) * T + (a2) * T * T) / (2 * T * T * T * T * T)
            
            # Compute positions, velocities, and accelerations
            positions[:, i] = a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
            velocities[:, i] = a1 + 2 * a2 * t + 3 * a3 * t**2 + 4 * a4 * t**3 + 5 * a5 * t**4
            accelerations[:, i] = 2 * a2 + 6 * a3 * t + 12 * a4 * t**2 + 20 * a5 * t**3
            
            # Limit velocities and accelerations
            velocities[:, i] = np.clip(velocities[:, i], -self.max_velocity, self.max_velocity)
            accelerations[:, i] = np.clip(accelerations[:, i], -self.max_acceleration, self.max_acceleration)
        
        trajectory = {
            'positions': positions,
            'velocities': velocities,
            'accelerations': accelerations,
            'time_step': self.time_step,
            'duration': duration,
            'start_time': current_time,
            'num_steps': num_steps
        }
        
        self.current_trajectory = trajectory
        self.trajectory_start_time = current_time
        self.trajectory_index = 0
        
        return trajectory
    
    def get_setpoint(self, current_time: float) -> Optional[Dict[str, List[float]]]:
        """Get the trajectory setpoint for the current time.
        
        Args:
            current_time: Current simulation time
            
        Returns:
            Dict with positions, velocities, and accelerations, or None if no active trajectory
        """
        if self.current_trajectory is None:
            return None
            
        elapsed_time = current_time - self.trajectory_start_time
        index = int(elapsed_time / self.time_step)
        
        if index >= self.current_trajectory['num_steps']:
            # Trajectory completed
            self.current_trajectory = None
            self.trajectory_index = 0
            return None
            
        self.trajectory_index = index
        return {
            'positions': self.current_trajectory['positions'][index].tolist(),
            'velocities': self.current_trajectory['velocities'][index].tolist(),
            'accelerations': self.current_trajectory['accelerations'][index].tolist(),
            'time': self.trajectory_start_time + index * self.time_step
        }
    
    def is_trajectory_active(self) -> bool:
        """Check if a trajectory is currently active.
        
        Returns:
            bool: True if a trajectory is active
        """
        return self.current_trajectory is not None
    
    def get_remaining_trajectory(self) -> Optional[Dict[str, Any]]:
        """Get the remaining portion of the current trajectory.
        
        Returns:
            Dict with remaining trajectory data, or None if no active trajectory
        """
        if self.current_trajectory is None:
            return None
            
        index = self.trajectory_index
        remaining_steps = self.current_trajectory['num_steps'] - index
        if remaining_steps <= 0:
            return None
            
        return {
            'positions': self.current_trajectory['positions'][index:],
            'velocities': self.current_trajectory['velocities'][index:],
            'accelerations': self.current_trajectory['accelerations'][index:],
            'time_step': self.time_step,
            'duration': remaining_steps * self.time_step,
            'start_time': self.trajectory_start_time + index * self.time_step,
            'num_steps': remaining_steps
        }
    
    def cancel_trajectory(self):
        """Cancel the current trajectory."""
        self.current_trajectory = None
        self.trajectory_index = 0
        self.trajectory_start_time = 0.0

class ObstacleAvoidancePlanner:
    """Planner for obstacle avoidance using potential fields."""
    
    def __init__(self, 
                 num_joints: int,
                 workspace_limits: List[Tuple[float, float]],
                 attractive_gain: float = 1.0,
                 repulsive_gain: float = 0.5,
                 influence_distance: float = 0.3):
        """Initialize the obstacle avoidance planner.
        
        Args:
            num_joints: Number of robot joints
            workspace_limits: List of (min, max) limits for each joint
            attractive_gain: Gain for attractive force to goal
            repulsive_gain: Gain for repulsive force from obstacles
            influence_distance: Distance within which obstacles influence the robot
        """
        self.num_joints = num_joints
        self.workspace_limits = [(min_val, max_val) for min_val, max_val in workspace_limits]
        self.attractive_gain = attractive_gain
        self.repulsive_gain = repulsive_gain
        self.influence_distance = influence_distance
        self.obstacles = []  # List of obstacle positions in Cartesian space
    
    def add_obstacle(self, position: List[float]):
        """Add an obstacle to avoid.
        
        Args:
            position: [x, y, z] position of the obstacle
        """
        self.obstacles.append(np.array(position, dtype=np.float32))
    
    def clear_obstacles(self):
        """Clear all obstacles."""
        self.obstacles = []
    
    def compute_potential_field(self, 
                               current_position: List[float], 
                               goal_position: List[float],
                               ee_position: List[float]) -> np.ndarray:
        """Compute potential field forces for obstacle avoidance.
        
        Args:
            current_position: Current joint positions
            goal_position: Goal joint positions
            ee_position: Current end-effector position in Cartesian space
            
        Returns:
            Array of joint velocities for obstacle avoidance
        """
        ee_pos = np.array(ee_position, dtype=np.float32)
        goal_pos = np.array(goal_position, dtype=np.float32)
        
        # Attractive force to goal
        attractive_force = self.attractive_gain * (goal_pos - ee_pos)
        
        # Repulsive forces from obstacles
        repulsive_force = np.zeros(3, dtype=np.float32)
        for obs in self.obstacles:
            dist_vec = ee_pos - obs
            dist = np.linalg.norm(dist_vec)
            if dist < self.influence_distance and dist > 0.001:  # Avoid division by zero
                repulsive_force += self.repulsive_gain * (1.0/dist - 1.0/self.influence_distance) * (dist_vec/dist) / (dist*dist)
        
        # Total force
        total_force = attractive_force + repulsive_force
        
        # This is a simplified approach. In a real implementation, you'd convert
        # Cartesian forces to joint velocities using the Jacobian matrix.
        # For now, we'll return a pseudo joint velocity proportional to the force.
        joint_velocity = np.zeros(self.num_joints, dtype=np.float32)
        for i in range(min(self.num_joints, 3)):
            joint_velocity[i] = total_force[i]
        
        return joint_velocity
