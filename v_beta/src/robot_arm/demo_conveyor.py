"""
Robot Arm Conveyor Demo

This demo spawns a conveyor belt and three robot arms, and allows you to control the conveyor with single-key presses (including numpad keys).

Controls:
- 's' or NUMPAD_5: Start/stop conveyor
- 'r' or NUMPAD_8: Reverse conveyor
- 'q': Quit
"""
import pybullet as p
import pybullet_data
import time
import sys
# No longer using ProductionLine or custom robot logic

# Key mapping (PyBullet uses ASCII codes; numpad keys are detected as extended codes)
KEY_START_STOP = ord('s')
KEY_REVERSE = ord('r')
KEY_QUIT = ord('q')
# Numpad keys (PyBullet: 65437 = NUMPAD_5, 65431 = NUMPAD_8)
KEY_NUMPAD_5 = 65437
KEY_NUMPAD_8 = 65431


def main():
    print("Starting Conveyor Demo with Keyboard Controls")
    print("==========================================")
    print("Controls:")
    print("- 's' or NUMPAD_5: Start/stop conveyor")
    print("- 'r' or NUMPAD_8: Reverse conveyor direction")
    print("- 'q': Quit")
    print()

    # PyBullet setup
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1./240.)

    # Load plane
    p.loadURDF("plane.urdf")

    # Create conveyor (box)
    conveyor_length = 6.0
    conveyor_width = 0.5
    conveyor_height = 0.1
    conveyor_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[conveyor_length/2, conveyor_width/2, conveyor_height/2]),
        baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[conveyor_length/2, conveyor_width/2, conveyor_height/2], rgbaColor=[0.4,0.4,0.4,1]),
        basePosition=[0, 0, conveyor_height/2]
    )
    p.changeVisualShape(conveyor_id, -1, textureUniqueId=p.loadTexture("checker_blue.png"), rgbaColor=[1,1,1,1])

    # Spawn three KUKA iiwa arms spaced along the conveyor
    kuka_urdf = "kuka_iiwa/model.urdf"
    # Offset robots to stand next to the conveyor (positive y direction)
    robot_y_offset = 0.4  # Place robots at y=+0.4, next to conveyor (conveyor width is 0.5)
    kuka_positions = [(-1.5, robot_y_offset, 0), (0, robot_y_offset, 0), (1.5, robot_y_offset, 0)]
    kuka_ids = []
    for i, pos in enumerate(kuka_positions):
        kuka_id = p.loadURDF(kuka_urdf, basePosition=pos, baseOrientation=p.getQuaternionFromEuler([0,0,0]), useFixedBase=True)
        kuka_ids.append(kuka_id)
        # Label
        p.addUserDebugText(f"KUKA-{i+1}", [pos[0], pos[1], 1.0], [0.1,0.1,0.8], 1.2)

    # Marbles (spheres) on conveyor
    marbles = []
    marble_radius = 0.05
    marble_mass = 0.02
    marble_spawn_interval = 2.0
    last_marble_time = time.time()
    conveyor_speed = 0.5  # m/s
    conveyor_running = True
    conveyor_direction = 1

    try:
        while True:
            keys = p.getKeyboardEvents()

            # Start/stop conveyor
            if (KEY_START_STOP in keys and keys[KEY_START_STOP] & p.KEY_WAS_TRIGGERED) or \
               (KEY_NUMPAD_5 in keys and keys[KEY_NUMPAD_5] & p.KEY_WAS_TRIGGERED):
                conveyor_running = not conveyor_running
                print(f"Conveyor running: {conveyor_running}")

            # Reverse conveyor
            if (KEY_REVERSE in keys and keys[KEY_REVERSE] & p.KEY_WAS_TRIGGERED) or \
               (KEY_NUMPAD_8 in keys and keys[KEY_NUMPAD_8] & p.KEY_WAS_TRIGGERED):
                conveyor_direction *= -1
                print(f"Conveyor direction: {'forward' if conveyor_direction == 1 else 'reverse'}")

            # Quit
            if KEY_QUIT in keys and keys[KEY_QUIT] & p.KEY_WAS_TRIGGERED:
                print("Quitting demo...")
                break

            # Spawn marbles at intervals
            now = time.time()
            if now - last_marble_time > marble_spawn_interval:
                marble_start = [-conveyor_length/2 + 0.3, 0, 0.2]
                marble_id = p.createMultiBody(
                    baseMass=marble_mass,
                    baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_SPHERE, radius=marble_radius),
                    baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=marble_radius, rgbaColor=[1,0.7,0.2,1]),
                    basePosition=marble_start
                )
                marbles.append(marble_id)
                last_marble_time = now

            # Move marbles if conveyor is running
            if conveyor_running:
                for marble_id in marbles:
                    pos, orn = p.getBasePositionAndOrientation(marble_id)
                    new_pos = [pos[0] + conveyor_speed * (1./240.) * conveyor_direction, pos[1], pos[2]]
                    p.resetBasePositionAndOrientation(marble_id, new_pos, orn)

            p.stepSimulation()
            time.sleep(1.0/240.0)
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        try:
            if hasattr(p, 'getConnectionInfo'):
                info = p.getConnectionInfo()
                if info.get('isConnected', 0):
                    p.disconnect()
        except Exception:
            pass

if __name__ == "__main__":
    main()
