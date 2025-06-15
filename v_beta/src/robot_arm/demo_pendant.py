"""
Robot Arm Pendant Demo

This demo shows a simple pendant (keyboard-based control panel) to move one KUKA iiwa arm's joints in real time.

Controls:
- Arrow keys / WASD: Move selected joint
- 1-7: Select joint (KUKA iiwa has 7 joints)
- +/-: Increase/decrease movement step
- q: Quit
"""
import pybullet as p
import pybullet_data
import time

KEY_QUIT = ord('q')
KEY_PLUS = ord('+')
KEY_MINUS = ord('-')
KEYS_JOINTS = [ord(str(i)) for i in range(1, 8)]  # 1-7

# Arrow keys (PyBullet key codes)
KEY_LEFT = 65361
KEY_RIGHT = 65363
KEY_UP = 65362
KEY_DOWN = 65364

# WASD for joint movement
KEY_W = ord('w')
KEY_S = ord('s')
KEY_A = ord('a')
KEY_D = ord('d')


def main():
    print("Starting Pendant Demo for Manual Arm Control")
    print("============================================")
    print("Controls:")
    print("- Arrow keys / WASD: Move selected joint")
    print("- 1-7: Select joint (KUKA iiwa has 7 joints)")
    print("- +/-: Increase/decrease movement step")
    print("- q: Quit")
    print()

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1./240.)
    p.loadURDF("plane.urdf")

    kuka_urdf = "kuka_iiwa/model.urdf"
    kuka_id = p.loadURDF(kuka_urdf, basePosition=[0, 0, 0], useFixedBase=True)
    num_joints = p.getNumJoints(kuka_id)

    # Initial joint positions
    joint_positions = [0.0] * num_joints
    for i in range(num_joints):
        p.resetJointState(kuka_id, i, joint_positions[i])

    selected_joint = 0
    step_size = 0.05

    def print_status():
        print(f"Selected joint: {selected_joint+1} | Step size: {step_size:.3f} | Joint positions: {[round(j,2) for j in joint_positions]}")

    print_status()

    try:
        while True:
            keys = p.getKeyboardEvents()
            # Joint selection
            for idx, key in enumerate(KEYS_JOINTS):
                if key in keys and keys[key] & p.KEY_WAS_TRIGGERED:
                    selected_joint = idx
                    print_status()
            # Step size adjustment
            if KEY_PLUS in keys and keys[KEY_PLUS] & p.KEY_WAS_TRIGGERED:
                step_size *= 1.2
                print_status()
            if KEY_MINUS in keys and keys[KEY_MINUS] & p.KEY_WAS_TRIGGERED:
                step_size /= 1.2
                print_status()
            # Joint movement
            move = 0
            if KEY_LEFT in keys and keys[KEY_LEFT] & p.KEY_IS_DOWN:
                move = -step_size
            if KEY_RIGHT in keys and keys[KEY_RIGHT] & p.KEY_IS_DOWN:
                move = step_size
            if KEY_A in keys and keys[KEY_A] & p.KEY_IS_DOWN:
                move = -step_size
            if KEY_D in keys and keys[KEY_D] & p.KEY_IS_DOWN:
                move = step_size
            if KEY_UP in keys and keys[KEY_UP] & p.KEY_IS_DOWN:
                move = step_size
            if KEY_DOWN in keys and keys[KEY_DOWN] & p.KEY_IS_DOWN:
                move = -step_size
            if KEY_W in keys and keys[KEY_W] & p.KEY_IS_DOWN:
                move = step_size
            if KEY_S in keys and keys[KEY_S] & p.KEY_IS_DOWN:
                move = -step_size
            if move != 0:
                joint_positions[selected_joint] += move
                p.setJointMotorControl2(kuka_id, selected_joint, p.POSITION_CONTROL, joint_positions[selected_joint], force=500)
                print_status()
            # Quit
            if KEY_QUIT in keys and keys[KEY_QUIT] & p.KEY_WAS_TRIGGERED:
                print("Quitting demo...")
                break
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
