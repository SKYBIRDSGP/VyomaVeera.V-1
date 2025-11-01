import pybullet as p
import time
import pybullet_data
import numpy as np

# -----------------------------
# Forward Kinematics Function
# -----------------------------
def forward_kinematics(theta1, theta2, theta3):
    """
    Compute analytical FK for 3R manipulator with 50.959 mm base offset.
    Angles in radians.
    """
    c1, s1 = np.cos(theta1), np.sin(theta1)
    c2, s2 = np.cos(theta2), np.sin(theta2)
    c3, s3 = np.cos(theta3), np.sin(theta3)

    # From derived transformation matrix
    x = c1 * (100 * c2 - 25 * s2 + 170 * (c2 * c3 - s2 * s3))
    y = s1 * (100 * c2 - 25 * s2 + 170 * (c2 * c3 - s2 * s3))
    z = 110.959 + 100 * s2 + 25 * c2 + 170 * (s2 * c3 + c2 * s3)

    return x / 1000.0, y / 1000.0, z / 1000.0  # convert mm → m for PyBullet scale


# -----------------------------
# PyBullet Setup
# -----------------------------
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf")

startPos = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("../model/Robot.urdf", startPos, startOrientation, useFixedBase=True)

for _ in range(100):
    p.stepSimulation()
    time.sleep(1. / 240.)

num_joints = p.getNumJoints(robotId)
print(f"\nNumber of joints detected: {num_joints}")
for i in range(num_joints):
    info = p.getJointInfo(robotId, i)
    print(f"Joint {i}: {info[1].decode('utf-8')} | limits: [{info[8]}, {info[9]}]")

# Create EE visual marker
marker_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 1])
marker_id = p.createMultiBody(baseVisualShapeIndex=marker_visual)

# -----------------------------
# User Input for Joint Angles
# -----------------------------
while True:
    try:
        user_input = input("\nEnter θ1, θ2, θ3 in degrees (or 'q' to quit): ")
        if user_input.lower() == 'q':
            break

        theta1_deg, theta2_deg, theta3_deg = map(float, user_input.split())
        theta1, theta2, theta3 = np.radians([-theta1_deg+90, theta2_deg, theta3_deg])
        Theta1, Theta2, Theta3 = np.radians([-theta1_deg , theta2_deg , theta3_deg])

        # Apply joint angles in PyBullet
        for j, angle in enumerate([-Theta1, Theta2, Theta3]):
            p.setJointMotorControl2(robotId, j, p.POSITION_CONTROL, targetPosition=angle)

        # Step simulation for smooth motion
        for _ in range(240):
            p.stepSimulation()
            time.sleep(1./240.)

        # Compute Analytical FK
        x, y, z = forward_kinematics(theta1, theta2, theta3)
        print(f"\nAnalytical EE Position (FK): x={x:.3f} m, y={y:.3f} m, z={z:.3f} m")

        # Update marker position in simulation
        p.resetBasePositionAndOrientation(marker_id, [x, y, z], [0, 0, 0, 1])

    except Exception as e:
        print("Invalid input, please enter 3 numeric angles or 'q' to quit.")

p.disconnect()
