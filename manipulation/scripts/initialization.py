import pybullet as p
import time
import pybullet_data

# --- Connect to PyBullet ---
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# --- Load plane and manipulator ---
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("../model/Robot.urdf", startPos, startOrientation, useFixedBase=True)

# --- Let simulation settle ---
for _ in range(200):  # ~1 sec at 240 Hz
    p.stepSimulation()
    time.sleep(1. / 240.)

print("Simulation initialized.")
time.sleep(2.0)

# --- Get number of joints and print info ---
num_joints = p.getNumJoints(robotId)
print(f"\nNumber of joints detected: {num_joints}")

for i in range(num_joints):
    info = p.getJointInfo(robotId, i)
    print(f"Joint {i}: {info[1].decode('utf-8')} | limits: [{info[8]}, {info[9]}]")

# --- Motion parameters (slower) ---
move_duration = 1.5       # seconds to go from min → max
wait_between_joints = 1.0  # seconds delay between joints
time_step = 1. / 240.

# --- Motion sequence ---
for joint_id in range(num_joints):
    info = p.getJointInfo(robotId, joint_id)
    joint_name = info[1].decode('utf-8')
    lower_limit, upper_limit = info[8], info[9]

    # Skip joints with invalid limits
    if lower_limit >= upper_limit:
        continue
    print(f"\nMoving joint '{joint_name}' from {lower_limit:.2f} → {upper_limit:.2f}")
    steps = int(move_duration / time_step)

    # --- Forward motion ---
    for step in range(steps):
        target = lower_limit + (upper_limit - lower_limit) * (step / steps)
        p.setJointMotorControl2(robotId, joint_id, p.POSITION_CONTROL, targetPosition=target)
        p.stepSimulation()
        time.sleep(time_step)

    # --- Small pause at max ---
    time.sleep(0.5)

    # --- Backward motion ---
    for step in range(steps):
        target = upper_limit - (upper_limit - lower_limit) * (step / steps)
        p.setJointMotorControl2(robotId, joint_id, p.POSITION_CONTROL, targetPosition=target)
        p.stepSimulation()
        time.sleep(time_step)

    # --- Wait before next joint starts ---
    time.sleep(wait_between_joints)

print("\nMotion sequence complete ")
p.disconnect()
