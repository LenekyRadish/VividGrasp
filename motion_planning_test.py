import pybullet as p
import pybullet_data
import numpy as np 
import time
import tempfile



import constants

BALL_RADIUS = constants.tennis_ball_r
BALL_POS = [0.4, 0.35, BALL_RADIUS]
BALL_TOP    = [BALL_POS[0], BALL_POS[1], BALL_RADIUS * 2 + 0.02]

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8) #newton found gravitational constant

p.loadURDF(pybullet_data.getDataPath() + "/plane.urdf") #load the plane for pybullet
#This is Christopher's URDF I took from the github
arm_id = p.loadURDF("2AxisArm/2AxisArm_pybullet.urdf", basePosition=[0, 0, 0], useFixedBase=True)
#Tennis Ball makes exist here with color and location from previous constants
col_ball = p.createCollisionShape(p.GEOM_SPHERE, radius=BALL_RADIUS)
vis_ball = p.createVisualShape(p.GEOM_SPHERE, radius=BALL_RADIUS, rgbaColor=[0.8, 1.0, 0.1, 1.0])
p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col_ball, baseVisualShapeIndex=vis_ball, basePosition=BALL_POS)

print(f"The arm is loaded with {p.getNumJoints(arm_id)} total joints.")

# IK for all 3 joints so the arm faces and reaches the ball
EE_LINK = 3
goal_angles = p.calculateInverseKinematics(arm_id, EE_LINK, BALL_TOP)  # yaw, shoulder, elbow
print(f"IK solution — yaw: {goal_angles[0]:.2f}, shoulder: {goal_angles[1]:.2f}, elbow: {goal_angles[2]:.2f}")

start_angles = np.array([0.0, 0.0, 0.0]) #obviously wanna start at 0
steps = 480 # ideal number for best rendering in pybullet

#sinusoidal "blend" motion planning
def sinusoidal_interpolation(start, goal, steps):
    t = np.linspace(0, np.pi, steps)
    alpha = (1 - np.cos(t)) / 2  
    return np.outer(1-alpha, start) + np.outer(alpha, goal)

interp_sin = sinusoidal_interpolation(start_angles, goal_angles, steps)
print("Sinusoidal interpolation angles:\n", np.round(interp_sin, 3))

print("\nRunning sinusoidal trajectory...")

for waypoint in interp_sin: #list of angles commanding the arm to move using constants
    p.setJointMotorControl2(arm_id, 0, p.POSITION_CONTROL, targetPosition=float(waypoint[0]), maxVelocity=2.0, force=20.0) #maxVelocity prevents overshooting
    p.setJointMotorControl2(arm_id, 1, p.POSITION_CONTROL, targetPosition=float(waypoint[1]), maxVelocity=2.0, force=20.0)
    p.setJointMotorControl2(arm_id, 2, p.POSITION_CONTROL, targetPosition=float(waypoint[2]), maxVelocity=2.0, force=20.0)
    p.stepSimulation()
    time.sleep(1 / 240)

print("Motion Planning Completed.")
while True:
    p.stepSimulation()
    time.sleep(1 / 240)