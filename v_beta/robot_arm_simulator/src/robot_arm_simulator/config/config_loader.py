"""Configuration loader for the robot arm simulator."""
import os
import yaml
from typing import Dict, Any, Optional, List, Union
from pathlib import Path

class ConfigLoader:
    """Load and manage configuration files for the robot arm simulator."""
    
    def __init__(self, config_dir: Optional[Union[str, os.PathLike]] = None):
        """Initialize the configuration loader.
        
        Args:
            config_dir: Directory containing configuration files. If None, uses default location.
        """
        if config_dir is None:
            # Default to a 'config' directory in the package
            self.config_dir = Path(__file__).parent.parent.parent / 'config'
        else:
            self.config_dir = Path(config_dir)
            
        # Ensure the config directory exists
        self.config_dir.mkdir(parents=True, exist_ok=True)
        
        # Cache for loaded configurations
        self._config_cache: Dict[str, Dict[str, Any]] = {}
    
    def load_config(self, config_name: str) -> Dict[str, Any]:
        """Load a configuration file by name.
        
        Args:
            config_name: Name of the configuration file (without extension)
            
        Returns:
            Dictionary containing the configuration
            
        Raises:
            FileNotFoundError: If the configuration file does not exist
            yaml.YAMLError: If there is an error parsing the YAML file
        """
        # Check cache first
        if config_name in self._config_cache:
            return self._config_cache[config_name]
            
        # Look for YAML file
        config_path = self.config_dir / f"{config_name}.yaml"
        
        if not config_path.exists():
            # Try YML extension
            config_path = self.config_dir / f"{config_name}.yml"
            if not config_path.exists():
                raise FileNotFoundError(f"Configuration file not found: {config_name}")
        
        # Load and parse the YAML file
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            
        # Cache the loaded configuration
        self._config_cache[config_name] = config
        
        return config
    
    def get_robot_config(self, robot_type: str) -> Dict[str, Any]:
        """Get configuration for a specific robot type.
        
        Args:
            robot_type: Type of robot (e.g., 'ur5', 'kuka_iiwa')
            
        Returns:
            Dictionary containing the robot configuration
        """
        try:
            config = self.load_config('robots')
            return config.get(robot_type, {})
        except FileNotFoundError:
            return {}
    
    def get_gripper_config(self, gripper_type: str) -> Dict[str, Any]:
        """Get configuration for a specific gripper type.
        
        Args:
            gripper_type: Type of gripper (e.g., 'parallel', 'suction')
            
        Returns:
            Dictionary containing the gripper configuration
        """
        try:
            config = self.load_config('grippers')
            return config.get(gripper_type, {})
        except FileNotFoundError:
            return {}
    
    def get_simulation_config(self, config_name: str = 'simulation') -> Dict[str, Any]:
        """Get simulation configuration.
        
        Args:
            config_name: Name of the simulation configuration (default: 'simulation')
            
        Returns:
            Dictionary containing the simulation configuration
        """
        try:
            return self.load_config(config_name)
        except FileNotFoundError:
            # Return default simulation configuration
            return {
                'gui': True,
                'realtime': True,
                'time_step': 1.0/240.0,
                'gravity': [0, 0, -9.81]
            }
    
    def create_environment_from_config(self, config_name: str = 'simulation'):
        """Create a simulation environment from a configuration file.
        
        Args:
            config_name: Name of the simulation configuration (default: 'simulation')
            
        Returns:
            Configured SimulationEnvironment instance
        """
        from ..simulation.environment import SimulationEnvironment
        
        # Load simulation configuration
        sim_config = self.get_simulation_config(config_name)
        
        # Create the environment
        env = SimulationEnvironment(**sim_config)
        
        # Load robots configuration if available
        try:
            robots_config = self.load_config('robots')
            for robot_name, robot_config in robots_config.items():
                robot_type = robot_config.get('type')
                if not robot_type:
                    print(f"Warning: Missing 'type' in robot config: {robot_name}")
                    continue
                    
                # Create the robot
                env.add_robot(
                    robot_type=robot_type,
                    name=robot_name,
                    **{k: v for k, v in robot_config.items() if k != 'type'}
                )
                
                # Add gripper if specified
                if 'gripper' in robot_config:
                    gripper_config = robot_config['gripper']
                    gripper_type = gripper_config.get('type')
                    if gripper_type:
                        env.add_gripper(
                            robot_name=robot_name,
                            gripper_type=gripper_type,
                            **{k: v for k, v in gripper_config.items() if k != 'type'}
                        )
                    
        except FileNotFoundError:
            print("No robots configuration found. Starting with an empty environment.")
            
        # Load objects configuration if available
        try:
            objects_config = self.load_config('objects')
            for obj_name, obj_config in objects_config.items():
                urdf_path = obj_config.get('urdf_path')
                if not urdf_path:
                    print(f"Warning: Missing 'urdf_path' in object config: {obj_name}")
                    continue
                    
                # Add the object
                env.add_object(
                    urdf_path=urdf_path,
                    name=obj_name,
                    **{k: v for k, v in obj_config.items() if k != 'urdf_path'}
                )
                
        except FileNotFoundError:
            print("No objects configuration found.")
            
        return env

# Global instance for convenience
config_loader = ConfigLoader()
