"""
Advanced Production Line Simulation with Robotic Arms

This demo showcases an industry-grade production line with:
- Multiple conveyor belts
- Robot arms with trajectory planning
- Sensor suite for object detection
- Task scheduling and coordination
- Real-time monitoring
"""
import os
import sys
import time
import numpy as np
import pybullet as p
import pybullet_data
from enum import Enum
from typing import Dict, List, Optional

# Add parent directory to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

# Import from the robot_arm package instead of robot_arm_simulator
from robot_arm.simulation.environment import SimulationEnvironment
from robot_arm.simulation.conveyor import ConveyorBelt
from robot_arm.simulation.sensors import ProximitySensor, CameraSensor
from robot_arm.control.trajectory_planner import TrajectoryPlanner

class RobotRole(Enum):
    PICKER = 1
    INSPECTOR = 2
    ASSEMBLER = 3
    PACKER = 4

class ProductionState(Enum):
    INCOMING = 1
    PICKED = 2
    UNDER_INSPECTION = 3
    INSPECTION_FAILED = 4
    INSPECTION_PASSED = 5
    UNDER_ASSEMBLY = 6
    ASSEMBLED = 7
    PACKED = 8

class ProductionObject:
    def __init__(self, obj_id: int, obj_type: str, position: List[float]):
        self.obj_id = obj_id
        self.obj_type = obj_type
        self.position = np.array(position, dtype=np.float32)
        self.state = ProductionState.INCOMING
        self.conveyor_id = None
        self.robot_id = None
        self.quality_score = None
        self.timestamp = time.time()

    def update(self, physics_client):
        pos, _ = physics_client.getBasePositionAndOrientation(self.obj_id)
        self.position = np.array(pos, dtype=np.float32)

    def change_state(self, new_state: ProductionState):
        if new_state != self.state:
            self.state = new_state
            print(f"Object {self.obj_id} changed state to {new_state}")

class Task:
    def __init__(self, task_type: str, robot_role: RobotRole, priority: int = 1, object_id: Optional[int] = None, target_position: Optional[List[float]] = None):
        self.task_id = id(self)
        self.task_type = task_type
        self.robot_role = robot_role
        self.priority = priority
        self.object_id = object_id
        self.target_position = np.array(target_position, dtype=np.float32) if target_position else None
        self.status = "pending"

class AdvancedProductionLine:
    def __init__(self, gui: bool = True, realtime: bool = True):
        self.gui = gui
        self.realtime = realtime
        self.env = None
        self.conveyors = {}
        self.robots = {}
        self.sensors = {}
        self.objects = {}
        self.object_counter = 0
        self.running = False
        self.spawn_timer = 0
        self.spawn_interval = 5.0
        self.available_types = ['component_a', 'component_b']
        self.tasks = []
        self.active_tasks = {}
        self.stats = {'objects_created': 0, 'objects_inspected': 0, 'objects_assembled': 0, 'objects_packed': 0}

    def initialize(self):
        print("Initializing advanced production line simulation...")
        self.env = SimulationEnvironment(gui=self.gui, realtime=self.realtime)
        self._setup_scene()
        self._add_conveyors()
        self._add_robots()
        self._add_sensors()
        if self.gui:
            self._setup_ui()
        print("Advanced production line simulation initialized.")

    def _setup_scene(self):
        self.env.add_plane()
        if self.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            p.resetDebugVisualizerCamera(cameraDistance=5.0, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.5])

    def _add_conveyors(self):
        input_conveyor = ConveyorBelt(physics_client=self.env.physics_client, position=[-1.5, 0.5, 0], orientation=p.getQuaternionFromEuler([0, 0, 0]), length=5.0, width=0.6, height=0.1, speed=0.2, max_speed=0.5)
        self.conveyors['input'] = input_conveyor
        output_conveyor = ConveyorBelt(physics_client=self.env.physics_client, position=[1.5, 0.5, 0], orientation=p.getQuaternionFromEuler([0, 0, 0]), length=5.0, width=0.6, height=0.1, speed=0.2, max_speed=0.5)
        self.conveyors['output'] = output_conveyor
        for conveyor in self.conveyors.values():
            conveyor.start()

    def _add_robots(self):
        robot_configs = [
            {'name': 'picker_robot', 'type': 'ur5', 'position': [-1.5, -0.5, 0], 'orientation': p.getQuaternionFromEuler([0, 0, np.pi/2]), 'gripper': 'parallel', 'role': RobotRole.PICKER},
            {'name': 'inspector_robot', 'type': 'ur5', 'position': [-0.5, -1.0, 0], 'orientation': p.getQuaternionFromEuler([0, 0, np.pi/2]), 'gripper': 'suction', 'role': RobotRole.INSPECTOR},
            {'name': 'assembler_robot', 'type': 'ur5', 'position': [0.5, -1.0, 0], 'orientation': p.getQuaternionFromEuler([0, 0, np.pi/2]), 'gripper': 'parallel', 'role': RobotRole.ASSEMBLER},
            {'name': 'packer_robot', 'type': 'ur5', 'position': [1.5, -0.5, 0], 'orientation': p.getQuaternionFromEuler([0, 0, np.pi/2]), 'gripper': 'parallel', 'role': RobotRole.PACKER}
        ]
        for config in robot_configs:
            robot_name = self.env.add_robot(robot_type=config['type'], name=config['name'], position=config['position'], orientation=config['orientation'])
            robot = self.env.get_robot(robot_name)
            if 'gripper' in config:
                self.env.add_gripper(robot_name=config['name'], gripper_type=config['gripper'])
            traj_planner = TrajectoryPlanner(num_joints=len(robot.joint_indices), max_velocity=2.0, max_acceleration=4.0, time_step=self.env.time_step, planning_horizon=3.0)
            self.robots[config['name']] = {'robot': robot, 'role': config['role'], 'busy': False, 'current_task': None, 'last_update': time.time(), 'trajectory_planner': traj_planner}

    def _add_sensors(self):
        self.sensors['input_proximity'] = ProximitySensor(name='input_proximity', body_id=-1, link_id=-1, position=[-2.0, 0.5, 0.3], direction=[1, 0, 0], max_range=0.5, min_range=0.05, fov=30.0, update_rate=10.0)
        inspector_robot = self.robots['inspector_robot']['robot']
        self.sensors['inspection_camera'] = CameraSensor(name='inspection_camera', body_id=inspector_robot.robot_id, link_id=inspector_robot.ee_index if hasattr(inspector_robot, 'ee_index') else -1, position=[0.1, 0, 0], orientation=[0, np.pi/2, 0], width=320, height=240, fov=60.0, near=0.1, far=1.0, update_rate=2.0)

    def _setup_ui(self):
        self.status_text_id = p.addUserDebugText("Advanced Production Line Simulation\nStatus: Initializing...", textPosition=[-2, -1.5, 1.5], textColorRGB=[1, 1, 1], textSize=1.2, lifeTime=0)
        self.stats_text_id = p.addUserDebugText("Statistics:\nObjects Created: 0", textPosition=[-2, -1.5, 1.0], textColorRGB=[0.9, 0.9, 0.9], textSize=1.0, lifeTime=0)
        self.help_text_id = p.addUserDebugText("Controls:\n  SPACE: Toggle simulation\n  C: Spawn object\n  S: Toggle conveyors\n  Q: Quit", textPosition=[1.5, -1.5, 1.0], textColorRGB=[0.8, 0.8, 0.8], textSize=1.0, lifeTime=0)
        self.robot_status_ids = {}
        for robot_name, robot_data in self.robots.items():
            pos = robot_data['robot'].get_base_position()
            self.robot_status_ids[robot_name] = p.addUserDebugText(f"{robot_name}: IDLE", textPosition=[pos[0], pos[1], 1.2], textColorRGB=[0.2, 0.8, 0.2], textSize=1.0, lifeTime=0)

    def spawn_object(self, obj_type: str = None, position: List[float] = None):
        if obj_type is None:
            obj_type = np.random.choice(self.available_types)
        if position is None:
            position = [-2.5, 0.5, 0.2]
        obj_id = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "cube.urdf"), basePosition=position, baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), globalScaling=0.1)
        color = [0.2, 0.2, 0.8, 1] if 'component_a' in obj_type else [0.8, 0.2, 0.2, 1]
        p.changeVisualShape(obj_id, -1, rgbaColor=color)
        obj = ProductionObject(obj_id, obj_type, position)
        self.objects[obj_id] = obj
        self.conveyors['input'].add_object(obj_id)
        obj.conveyor_id = 'input'
        self.stats['objects_created'] += 1
        self.object_counter += 1
        print(f"Spawned {obj_type} with ID {obj_id}")
        return obj_id

    def update(self, time_step: float):
        if not self.running:
            return
        current_time = time.time()
        for conveyor in self.conveyors.values():
            conveyor.update(time_step)
        for sensor in self.sensors.values():
            sensor.update(current_time, self.env.physics_client)
        self._update_objects()
        self._handle_object_spawning(time_step)
        self._update_robots(current_time)
        self._generate_tasks()
        self._update_ui(current_time)

    def _update_objects(self):
        for obj_id in list(self.objects.keys()):
            if obj_id >= p.getNumBodies() or obj_id < 0:
                for conveyor in self.conveyors.values():
                    if obj_id in conveyor.tracked_objects:
                        conveyor.remove_object(obj_id)
                del self.objects[obj_id]
                continue
            obj = self.objects[obj_id]
            obj.update(self.env.physics_client)
            if obj.conveyor_id and obj.conveyor_id in self.conveyors:
                conveyor = self.conveyors[obj.conveyor_id]
                if not conveyor.is_object_on_conveyor(obj_id):
                    conveyor.remove_object(obj_id)
                    obj.conveyor_id = None

    def _handle_object_spawning(self, time_step: float):
        self.spawn_timer += time_step
        if self.spawn_timer >= self.spawn_interval:
            self.spawn_object()
            self.spawn_timer = 0

    def _update_robots(self, current_time: float):
        for robot_name, robot_data in self.robots.items():
            robot = robot_data['robot']
            traj_planner = robot_data['trajectory_planner']
            if robot_data['busy'] and robot_data['current_task']:
                task = robot_data['current_task']
                if self._is_task_complete(task, robot_data, current_time):
                    robot_data['busy'] = False
                    task.status = "completed"
                    self.active_tasks.pop(robot_name, None)
                    robot_data['current_task'] = None
                    print(f"{robot_name} completed task: {task.task_type}")
                    traj_planner.cancel_trajectory()
                else:
                    setpoint = traj_planner.get_setpoint(current_time)
                    if setpoint:
                        robot.set_joint_positions(setpoint['positions'])
                continue
            if not robot_data['busy']:
                task = self._assign_task_to_robot(robot_name, robot_data['role'])
                if task:
                    robot_data['current_task'] = task
                    robot_data['busy'] = True
                    robot_data['last_update'] = current_time
                    task.status = "active"
                    self.active_tasks[robot_name] = task
                    print(f"{robot_name} started task: {task.task_type}")
                    if task.task_type == "pick":
                        obj = self.objects.get(task.object_id)
                        if obj:
                            current_pos = robot.get_joint_positions()
                            target_pos = current_pos.copy()
                            traj_planner.plan_trajectory(start_positions=current_pos, target_positions=target_pos, current_time=current_time)
                            obj.change_state(ProductionState.PICKED)
                            obj.robot_id = robot_name
                            if obj.conveyor_id and obj.conveyor_id in self.conveyors:
                                self.conveyors[obj.conveyor_id].remove_object(obj.obj_id)
                                obj.conveyor_id = None

    def _is_task_complete(self, task, robot_data, current_time):
        if task is None:
            return True
        return (current_time - robot_data['last_update']) > 5.0

    def _assign_task_to_robot(self, robot_name: str, role: RobotRole) -> Optional[Task]:
        suitable_tasks = [task for task in self.tasks if task.robot_role == role and task.status == "pending"]
        if not suitable_tasks:
            return None
        suitable_tasks.sort(key=lambda t: t.priority, reverse=True)
        task = suitable_tasks[0]
        self.tasks.remove(task)
        return task

    def _generate_tasks(self):
        for obj_id, obj in self.objects.items():
            if obj.state == ProductionState.INCOMING and obj.conveyor_id == 'input':
                self.tasks.append(Task(task_type="pick", robot_role=RobotRole.PICKER, priority=2, object_id=obj_id))
            elif obj.state == ProductionState.PICKED and obj.robot_id is None:
                self.tasks.append(Task(task_type="inspect", robot_role=RobotRole.INSPECTOR, priority=2, object_id=obj_id))
            elif obj.state == ProductionState.INSPECTION_PASSED and obj.robot_id is None:
                self.tasks.append(Task(task_type="assemble", robot_role=RobotRole.ASSEMBLER, priority=2, object_id=obj_id))
            elif obj.state == ProductionState.ASSEMBLED and obj.robot_id is None:
                self.tasks.append(Task(task_type="pack", robot_role=RobotRole.PACKER, priority=2, object_id=obj_id))

    def _update_ui(self, current_time):
        if not self.gui:
            return
        status_text = (f"Advanced Production Line Simulation\nStatus: {'Running' if self.running else 'Paused'}\nObjects: {len(self.objects)} created")
        p.addUserDebugText(status_text, textPosition=[-2, -1.5, 1.5], textColorRGB=[1, 1, 1], textSize=1.0, lifeTime=0, replaceItemUniqueId=self.status_text_id)
        stats_text = (f"Statistics:\nObjects Created: {self.stats['objects_created']}\nInspected: {self.stats['objects_inspected']}\nAssembled: {self.stats['objects_assembled']}\nPacked: {self.stats['objects_packed']}")
        p.addUserDebugText(stats_text, textPosition=[-2, -1.5, 1.0], textColorRGB=[0.9, 0.9, 0.9], textSize=1.0, lifeTime=0, replaceItemUniqueId=self.stats_text_id)
        for robot_name, robot_data in self.robots.items():
            status = "BUSY" if robot_data['busy'] else "IDLE"
            color = [0.8, 0.2, 0.2] if robot_data['busy'] else [0.2, 0.8, 0.2]
            pos = robot_data['robot'].get_base_position()
            p.addUserDebugText(f"{robot_name}: {status}", textPosition=[pos[0], pos[1], 1.2], textColorRGB=color, textSize=1.0, lifeTime=0, replaceItemUniqueId=self.robot_status_ids[robot_name])

    def run(self):
        if self.env is None:
            self.initialize()
        self.running = True
        print("Starting advanced production line simulation. Press SPACE to pause/resume, Q to quit.")
        try:
            while True:
                keys = p.getKeyboardEvents()
                if ord(' ') in keys and keys[ord(' ')] & p.KEY_WAS_TRIGGERED:
                    self.running = not self.running
                    print(f"Simulation {'paused' if not self.running else 'resumed'}")
                if ord('c') in keys and keys[ord('c')] & p.KEY_WAS_TRIGGERED:
                    self.spawn_object()
                if ord('s') in keys and keys[ord('s')] & p.KEY_WAS_TRIGGERED:
                    for conveyor in self.conveyors.values():
                        if conveyor.running:
                            conveyor.stop()
                        else:
                            conveyor.start()
                if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
                    print("Quitting simulation...")
                    break
                self.env.step()
                self.update(self.env.time_step)
                if self.realtime:
                    time.sleep(self.env.time_step)
        except KeyboardInterrupt:
            print("Simulation stopped by user")
        finally:
            self.cleanup()

    def cleanup(self):
        if self.env:
            self.env.cleanup()
        print("Simulation cleaned up.")

if __name__ == "__main__":
    sim = AdvancedProductionLine(gui=True, realtime=True)
    sim.run()
