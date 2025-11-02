# ik_corrected.py
import pybullet as p
import pybullet_data
import numpy as np
import math
import time

# -----------------------------
# Forward kinematics (analytic)
# input: theta_math in radians (theta1, theta2, theta3)
# outputs: x,y,z in meters (world frame; includes base offset)
# -----------------------------
def forward_kinematics(theta1, theta2, theta3):
    c1, s1 = math.cos(theta1), math.sin(theta1)
    c2, s2 = math.cos(theta2), math.sin(theta2)
    c3, s3 = math.cos(theta3), math.sin(theta3)

    # mm formulas (from your derivation)
    x_mm = c1 * (100 * c2 - 25 * s2 + 170 * (c2 * c3 - s2 * s3))
    y_mm = s1 * (100 * c2 - 25 * s2 + 170 * (c2 * c3 - s2 * s3))
    z_mm = 110.959 + 100 * s2 + 25 * c2 + 170 * (s2 * c3 + c2 * s3)  # includes base offset

    return x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0

# -----------------------------
# Analytical IK (returns theta_math angles in radians)
# target x,y,z are in meters (world)
# -----------------------------
def analytic_ik_math(x_m, y_m, z_m):
    # convert to mm
    x = x_m * 1000.0
    y = y_m * 1000.0
    z = z_m * 1000.0

    # link constants (mm)
    a2 = 100.0
    oy = 25.0
    a = math.hypot(a2, oy)       # effective first link
    delta = math.atan2(oy, a2)
    b = 170.0
    base_offset = 50.959

    # theta1 math (in math-frame)
    theta1 = math.atan2(y, x)

    # planar reduction
    r = math.hypot(x, y)
    z1 = z - base_offset
    D = math.hypot(r, z1)

    # check reachability
    cosDelta = (D*D - a*a - b*b) / (2 * a * b)
    if cosDelta > 1.0 or cosDelta < -1.0:
        return None  # unreachable

    # two branches: elbow-down (+) and elbow-up (-). We'll return both.
    Delta_pos = math.acos(max(-1.0, min(1.0, cosDelta)))
    Delta_neg = -Delta_pos

    solutions = []
    for Delta in (Delta_pos, Delta_neg):
        gamma = math.atan2(z1, r)
        phi = gamma - math.atan2(b * math.sin(Delta), a + b * math.cos(Delta))

        # recover math-frame theta2, theta3
        theta2_math = phi - delta
        theta3_math = Delta + delta  # note earlier we used different sign conventions; this matches derivation

        solutions.append((theta1, theta2_math, theta3_math))
    return solutions  # list of 0,1 or 2 solutions (radians)

# -----------------------------
# Map math→user→sim
# -----------------------------
def math_to_user_deg(theta1_math, theta2_math, theta3_math):
    # user_deg mapping per your FK: theta_math = -user_deg + 90  => user_deg = 90 - theta_math(deg)
    user1_deg = 90.0 - math.degrees(theta1_math)
    user2_deg = math.degrees(theta2_math)
    user3_deg = math.degrees(theta3_math)
    return user1_deg, user2_deg, user3_deg

def user_deg_to_sim_rad(user1_deg, user2_deg, user3_deg):
    # your simulation feed was essentially radians(user_deg)
    sim1 = math.radians(user1_deg)
    sim2 = math.radians(user2_deg)
    sim3 = math.radians(user3_deg)
    return sim1, sim2, sim3

# -----------------------------
# PyBullet init
# -----------------------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
plane = p.loadURDF("plane.urdf")
startPos = [0,0,0]
startOri = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadURDF("../model/Robot.urdf", startPos, startOri, useFixedBase=True)

time.sleep(0.2)
num_j = p.getNumJoints(robotId)
print("Num joints:", num_j)
for i in range(num_j):
    print(i, p.getJointInfo(robotId, i)[1].decode())

# assume first 3 actuated joints correspond to indices 0,1,2 — change if URDF differs
joint_indices = [0,1,2]
ee_link = num_j - 1

# visual marker for desired EE
marker_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1,0,0,1])
marker_id = p.createMultiBody(baseVisualShapeIndex=marker_visual)

# helper to step a bit
def step(n=120):
    for _ in range(n):
        p.stepSimulation()
        time.sleep(1.0/240.0)

# -----------------------------
# Interactive IK loop
# -----------------------------
print("\nEnter target EE x y z in meters (q to quit). The script will try both elbow solutions and show checks.")
while True:
    ui = input("Target (x y z): ")
    if ui.strip().lower().startswith('q'):
        break
    try:
        x_t, y_t, z_t = map(float, ui.split())
    except:
        print("Invalid input. e.g. try: 0.10 0.00 0.30")
        continue

    sols = analytic_ik_math(x_t, y_t, z_t)
    if not sols:
        print("Unreachable target or no solutions.")
        continue

    # show both solutions and pick the one with smallest FK->sim error
    best = None
    for idx, (t1m, t2m, t3m) in enumerate(sols):
        # 1) Analytical FK check (math angles)
        x_fk, y_fk, z_fk = forward_kinematics(t1m, t2m, t3m)

        # 2) Map to user degrees and simulation radians
        user1_deg, user2_deg, user3_deg = math_to_user_deg(t1m, t2m, t3m)
        sim1, sim2, sim3 = user_deg_to_sim_rad(user1_deg, user2_deg, user3_deg)

        # 3) Send to PyBullet
        for j, angle in zip(joint_indices, [sim1, sim2, sim3]):
            p.setJointMotorControl2(robotId, j, p.POSITION_CONTROL, targetPosition=angle, force=500)
        step(120)

        # 4) read simulated EE pose
        ls = p.getLinkState(robotId, ee_link)
        sim_pos = np.array(ls[0])   # meters

        # 5) compute FK (analytic) and sims in same units
        fk_pos = np.array([x_fk, y_fk, z_fk])

        err = np.linalg.norm(sim_pos - fk_pos)
        print(f"\nSolution {idx+1}: user_deg=({user1_deg:.2f},{user2_deg:.2f},{user3_deg:.2f}), sim_rad=({sim1:.3f},{sim2:.3f},{sim3:.3f})")
        print(f"  Analytic FK pos: {fk_pos}, Sim EE pos: {sim_pos}, error (m): {err:.6f}")

        # record best
        if best is None or err < best['err']:
            best = {'err':err, 'idx':idx, 'user_deg':(user1_deg, user2_deg, user3_deg), 'sim_rad':(sim1,sim2,sim3), 'fk_pos':fk_pos, 'sim_pos':sim_pos}

    # show best solution and leave robot at that pose
    print("\nBest solution chosen:", best['idx']+1, "error:", best['err'])
    print("User angles (deg):", best['user_deg'])
    print("Sim angles (rad):", best['sim_rad'])
    # place visual marker at requested target
    p.resetBasePositionAndOrientation(marker_id, [x_t, y_t, z_t], [0,0,0,1])
    # keep robot at best pos for a while
    step(240)

print("Done.")
p.disconnect()
