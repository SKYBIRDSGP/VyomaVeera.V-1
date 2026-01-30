import pybullet as p
import time
import pybullet_data
import numpy as np
import tty
import termios
import sys

def getKey():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

settings = termios.tcgetattr(sys.stdin)

# -----------------------------
# Forward Kinematics Function
# -----------------------------
def forward_kinematics(theta1, theta2, theta3):
    c1, s1 = np.cos(theta1), np.sin(theta1)
    c2, s2 = np.cos(theta2), np.sin(theta2)
    c3, s3 = np.cos(theta3), np.sin(theta3)

    x = c1 * (100 * c2 - 25 * s2 + 170 * (c2 * c3 - s2 * s3))
    y = s1 * (100 * c2 - 25 * s2 + 170 * (c2 * c3 - s2 * s3))
    z = 110.959 + 100 * s2 + 25 * c2 + 170 * (s2 * c3 + c2 * s3)

    return x / 1000.0, y / 1000.0, z / 1000.0


# -----------------------------
# PyBullet Setup
# -----------------------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robotId = p.loadURDF(
    "../model/Robot.urdf",
    [0, 0, 0],
    p.getQuaternionFromEuler([0, 0, 0]),
    useFixedBase=True
)

# Let sim settle
for _ in range(100):
    p.stepSimulation()
    time.sleep(1/240)

# -----------------------------
# EE Marker
# -----------------------------
marker_visual = p.createVisualShape(
    p.GEOM_SPHERE,
    radius=0.02,
    rgbaColor=[1, 0, 0, 1]
)
marker_id = p.createMultiBody(baseVisualShapeIndex=marker_visual)

# -----------------------------
# HOME CONFIGURATION (degrees)
# -----------------------------
theta1_deg = 0
theta2_deg = 0
theta3_deg = 180
theta4_deg = 90

increment = np.radians(10)

# -----------------------------
# SAME COMPENSATION LOGIC
# -----------------------------
theta1_fk, theta2_fk, theta3_fk = np.radians([
    -theta1_deg + 90,
    theta2_deg,
    theta3_deg
])

Theta1, Theta2, Theta3, Theta4 = np.radians([
    -theta1_deg,
    theta2_deg,
    theta3_deg,
    theta4_deg
])

# -----------------------------
# APPLY TO SIMULATION
# -----------------------------
p.setJointMotorControl2(robotId, 0, p.POSITION_CONTROL, targetPosition=-Theta1)
p.setJointMotorControl2(robotId, 1, p.POSITION_CONTROL, targetPosition=Theta2)
p.setJointMotorControl2(robotId, 2, p.POSITION_CONTROL, targetPosition=Theta3)
p.setJointMotorControl2(robotId, 3, p.POSITION_CONTROL, targetPosition=Theta4)

# # Step simulation
# for _ in range(480):
#     p.stepSimulation()
#     time.sleep(1/240)

# -----------------------------
# FK + EE MARKER UPDATE
# -----------------------------
x, y, z = forward_kinematics(theta1_fk, theta2_fk, theta3_fk)

gripper_open = False
gripper_angle_open = 0.0
gripper_angle_closed = np.radians(90)

p.resetBasePositionAndOrientation(
    marker_id,
    [x, y, z],
    [0, 0, 0, 1]
)

print("\n SETTING TO HOME POSITION .")
print(f"EE Position: x = {x:.3f} m, y = {y:.3f} m, z = {z:.3f} m")

msg = """
Active Input Keys are as follows: 

w   e   r     p
 s   d   f
      c   v

w : Incrementing the joint angle of base 
s : Decrementing the joint angle of base 

e : Incrementing the joint angle of shoulder
d : Decrementing the joint angle of shoulder

r : Incrementing the joint angle of elbow
d : Decrementing the joint angle of elbow

c : Gripper close
v : Gripper open

p : Display current EE position
"""

for _ in range(200):
        p.stepSimulation()
        time.sleep(1./200.)

print("\n===== VYOMAVEERA TELEOPERATION =====")
print("\nControl Inputs:")
print(msg)

while True:
    key = getKey()

    if key =='q':
        break

    elif key == 'r':
        p.setJointMotorControl2(
        robotId,
        2,  # base joint index
        p.POSITION_CONTROL,
        targetPosition=(Theta3 - increment)
        )   
        Theta3 = Theta3 - increment
        theta3_fk = theta3_fk - increment
    
    elif key == 'f':
        p.setJointMotorControl2(
        robotId,
        2,  # base joint index
        p.POSITION_CONTROL,
        targetPosition=(Theta3 + increment)
        )   
        Theta3 = Theta3 + increment
        theta3_fk = theta3_fk + increment
    
    elif key == 'e':
        p.setJointMotorControl2(
        robotId,
        1,  # base joint index
        p.POSITION_CONTROL,
        targetPosition=(Theta2 + increment)
        )   
        Theta2 = Theta2 + increment
        theta2_fk = theta2_fk + increment
    
    elif key == 'd':
        p.setJointMotorControl2(
        robotId,
        1,  # base joint index
        p.POSITION_CONTROL,
        targetPosition=(Theta2 - increment)
        )   
        Theta2 = Theta2 - increment
        theta2_fk = theta2_fk - increment

    elif key == 'w':
        p.setJointMotorControl2(
        robotId,
        0,  # base joint index
        p.POSITION_CONTROL,
        targetPosition=(Theta1 + increment)
        )   
        Theta1 = Theta1 + increment
        theta1_fk = theta1_fk - increment
    
    elif key == 's':
        p.setJointMotorControl2(
        robotId,
        0,  # base joint index
        p.POSITION_CONTROL,
        targetPosition=(Theta1 - increment)
        )   
        Theta1 = Theta1 - increment
        theta1_fk = theta1_fk + increment

    elif key == 'c':
        gripper_angle = np.radians(90)   # CLOSE
        print("Gripper CLOSED")
        p.setJointMotorControl2(
        robotId,
        3,  # gripper joint index
        p.POSITION_CONTROL,
        targetPosition=gripper_angle
    )

    elif key == 'v':
        gripper_angle = 0.0               # OPEN
        print("Gripper OPEN")
        p.setJointMotorControl2(
            robotId,
            3,  # gripper joint index
            p.POSITION_CONTROL,
            targetPosition=gripper_angle
        )

    elif key == 'p':
        x, y, z = forward_kinematics(theta1_fk, theta2_fk, theta3_fk)
        print(f"Current EE Position: x = {x:.3f} m, y = {y:.3f} m, z = {z:.3f} m")
    
    else:
        print('\nInvalid Input!')
        print(msg)


    x, y, z = forward_kinematics(theta1_fk, theta2_fk, theta3_fk)
    p.resetBasePositionAndOrientation(
        marker_id,
        [x, y, z],
        [0, 0, 0, 1]
    )

    for _ in range(150):
        p.stepSimulation()
        time.sleep(0.5/150.)

p.disconnect()