import pybullet as p
import pybullet_data
import numpy as np
import cv2
import random
import os
import time
import math
from pathlib import Path
import constants

ROOT = Path(__file__).parent
ARM_FOLDER = ROOT / "2AxisArm"

CAMERA_WIDTH  = 600
CAMERA_HEIGHT = 600
CAMERA_Z      = 1.0
CAMERA_FOV    = 42

EE_LINK           = 3
STEPS             = 240
ARM_AZIMUTH_OFFSET = 1.0988

CAMERA_ASPECT      = CAMERA_WIDTH / CAMERA_HEIGHT
GROUND_HALF_HEIGHT = CAMERA_Z * math.tan(math.radians(CAMERA_FOV / 2))
GROUND_HALF_WIDTH  = GROUND_HALF_HEIGHT * CAMERA_ASPECT
SPAWN_AREA         = min(GROUND_HALF_WIDTH, GROUND_HALF_HEIGHT) - 0.07

#This is almost the same as from the notebooks but it's instead using what the "camera" in real life would also see instead of -0.5, 0.5
WORLD_MIN_X = -GROUND_HALF_WIDTH
WORLD_MAX_X =  GROUND_HALF_WIDTH
WORLD_MIN_Y = -GROUND_HALF_HEIGHT
WORLD_MAX_Y =  GROUND_HALF_HEIGHT

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

#changed pybullet settings from object(2)2.py
plane = p.loadURDF("plane.urdf")
p.changeVisualShape(plane, -1, rgbaColor=[0.25, 0.25, 0.25, 1])
p.changeDynamics(plane, -1, contactStiffness=1e6, contactDamping=1e3)

os.chdir(ARM_FOLDER)
arm_id = p.loadURDF("2AxisArm_pybullet.urdf", basePosition=[0, 0, 0], useFixedBase=True)
os.chdir(ROOT)

print(f"The arm is loaded with {p.getNumJoints(arm_id)} total joints.")
#loading the trays and defining the positions for each
tray_scale = 0.18
TRAY_POSITIONS = {
    "tennis":    [0.32,  0.18, 0.0],
    "baseball":  [0.34,  0.0,  0.0],
    "ping_pong": [0.32, -0.18, 0.0]
}

tray1 = p.loadURDF("tray/traybox.urdf", basePosition=TRAY_POSITIONS["tennis"],    useFixedBase=True, globalScaling=tray_scale)
tray2 = p.loadURDF("tray/traybox.urdf", basePosition=TRAY_POSITIONS["baseball"],  useFixedBase=True, globalScaling=tray_scale)
tray3 = p.loadURDF("tray/traybox.urdf", basePosition=TRAY_POSITIONS["ping_pong"], useFixedBase=True, globalScaling=tray_scale)

def create_sports_ball(radius, mass, lateral_friction, rolling_friction, color, position):
    collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    visual_shape    = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
    ball_id = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_shape,
                                baseVisualShapeIndex=visual_shape, basePosition=position)
    p.changeDynamics(ball_id, -1, lateralFriction=lateral_friction,
                     rollingFriction=rolling_friction, restitution=0.3)
    return ball_id

placed = []
#ensures that trays and balls spawn in away from the base to avoid improper object detection or impossible to reach locations
def position_is_clear_of_trays(x, y, radius):
    for tray_position in TRAY_POSITIONS.values():
        if math.hypot(x - tray_position[0], y - tray_position[1]) < radius + 0.12:
            return False
    return True

def random_safe_position(radius, min_from_origin=0.18, area=SPAWN_AREA):
    while True:
        x = random.uniform(-area, area)
        y = random.uniform(-area, area)
        if math.hypot(x, y) < min_from_origin:
            continue
        if not position_is_clear_of_trays(x, y, radius):
            continue
        safe = True
        for old_x, old_y, old_radius in placed:
            if math.hypot(x - old_x, y - old_y) < radius + old_radius + 0.08:
                safe = False
                break
        if safe:
            placed.append((x, y, radius))
            return [x, y, 0.4]
#create each type of ball
tennis_ball = create_sports_ball(constants.tennis_ball_r,    0.058,  0.6, 0.01,  [0.1, 1.0, 0.05, 1], random_safe_position(constants.tennis_ball_r))
baseball    = create_sports_ball(constants.baseball_r,       0.145,  0.5, 0.005, [1.0, 1.0, 1.0,  1], random_safe_position(constants.baseball_r))
ping_pong   = create_sports_ball(constants.ping_pong_ball_r, 0.0027, 0.2, 0.001, [1.0, 0.35, 0.0, 1], random_safe_position(constants.ping_pong_ball_r))

for _ in range(288):
    p.stepSimulation()
    time.sleep(1 / 300)
#captures and image like a camera in real life would in pybullet and converts it to openCV
def get_camera_image():
    view_matrix       = p.computeViewMatrix([0, 0, CAMERA_Z], [0, 0, 0], [0, 1, 0])
    projection_matrix = p.computeProjectionMatrixFOV(CAMERA_FOV, CAMERA_ASPECT, 0.05, 2.0)
    _, _, rgba, _, _  = p.getCameraImage(CAMERA_WIDTH, CAMERA_HEIGHT,
                                          viewMatrix=view_matrix,
                                          projectionMatrix=projection_matrix,
                                          renderer=p.ER_TINY_RENDERER)
    rgba  = np.array(rgba, dtype=np.uint8).reshape(CAMERA_HEIGHT, CAMERA_WIDTH, 4)
    image = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
    cv2.imwrite(str(ROOT / "camera_image.png"), image)
    return image
#hsv ranges
color_ranges = {
    "tennis":    [{"lower": np.array([35,  90,  80]),  "upper": np.array([85,  255, 255])}],
    "baseball":  [{"lower": np.array([0,   0,   180]), "upper": np.array([180, 65,  255])}],
    "ping_pong": [{"lower": np.array([3,   140, 120]), "upper": np.array([22,  255, 255])}]
}
#define how circular the object is
def contour_circularity(contour):
    area      = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return 0
    return 4 * np.pi * area / (perimeter * perimeter)
#detect for each type of ball
def detect_all_balls(image):
    hsv          = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    result_image = image.copy()
    detected_balls = {}
    saved_masks    = {}

    for ball_name, ranges in color_ranges.items():
        color_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for color_range in ranges:
            mask       = cv2.inRange(hsv, color_range["lower"], color_range["upper"])
            color_mask = cv2.bitwise_or(color_mask, mask)

        kernel     = np.ones((3, 3), np.uint8)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN,  kernel)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel)

        saved_masks[ball_name] = color_mask.copy()
        cv2.imwrite(str(ROOT / f"{ball_name}_mask.png"), color_mask)

        contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        best_contour = None
        best_score   = 0

        for contour in contours:
            area        = cv2.contourArea(contour)
            circularity = contour_circularity(contour)
            moments     = cv2.moments(contour)
            if moments["m00"] == 0:
                continue

            center_x = int(moments["m10"] / moments["m00"])
            center_y = int(moments["m01"] / moments["m00"])
            center_distance = math.hypot(center_x - CAMERA_WIDTH / 2, center_y - CAMERA_HEIGHT / 2)

            if center_distance < 70:
                continue

            valid_area = (50 < area < 2200) if ball_name == "ping_pong" else (120 < area < 5000)#had to do this because pybullet's floor was picking up as the ping pong ball

            if valid_area and circularity > 0.45:
                score = area * circularity
                if score > best_score:
                    best_score   = score
                    best_contour = contour
#returns detected ball data
        if best_contour is not None:
            moments     = cv2.moments(best_contour)
            center_x    = int(moments["m10"] / moments["m00"])
            center_y    = int(moments["m01"] / moments["m00"])
            area        = cv2.contourArea(best_contour)
            circularity = contour_circularity(best_contour)

            detected_balls[ball_name] = {"center": (center_x, center_y), "area": area, "circularity": circularity}
#pop-up details on the detected objects
            cv2.drawContours(result_image, [best_contour], -1, (0, 0, 0), 2)
            cv2.circle(result_image, (center_x, center_y), 6, (0, 0, 255), -1)
            cv2.putText(result_image, ball_name, (center_x - 35, center_y - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

    cv2.imwrite(str(ROOT / "detected_camera_view.png"), result_image)
#debug details on the masks because pybullet shows synthetic masks. We use HSV masks so it's important to visualize this
    tennis_debug    = cv2.cvtColor(cv2.resize(saved_masks["tennis"],    (250, 250)), cv2.COLOR_GRAY2BGR)
    baseball_debug  = cv2.cvtColor(cv2.resize(saved_masks["baseball"],  (250, 250)), cv2.COLOR_GRAY2BGR)
    ping_pong_debug = cv2.cvtColor(cv2.resize(saved_masks["ping_pong"], (250, 250)), cv2.COLOR_GRAY2BGR)
    cv2.putText(tennis_debug,    "Tennis mask",    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(baseball_debug,  "Baseball mask",  (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(ping_pong_debug, "Ping-pong mask", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    combined_masks = np.hstack([tennis_debug, baseball_debug, ping_pong_debug])
    cv2.imwrite(str(ROOT / "combined_masks.png"), combined_masks)
    cv2.imshow("Actual OpenCV HSV masks",  combined_masks)
    cv2.imshow("Detected camera view", cv2.resize(result_image, (500, 500)))
    cv2.waitKey(1000)

    return detected_balls

#pixels to world coordinates from mapping.py
def convertCoordinates(px, py, ball_radius,
                        min_x=WORLD_MIN_X, max_x=WORLD_MAX_X,
                        min_y=WORLD_MIN_Y, max_y=WORLD_MAX_Y,
                        imgW=CAMERA_WIDTH,  imgH=CAMERA_HEIGHT):
    px = max(0, min(px, imgW))
    py = max(0, min(py, imgH))
    norm_x  = px / imgW
    norm_y  = py / imgH  
    world_x = min_x + norm_x * (max_x - min_x)
    world_y = max_y - norm_y * (max_y - min_y)
    return [world_x, world_y, ball_radius]
#shorter path to object (don't do an unneccessary 360 turn)
def wrap_angle(angle):
    while angle >  np.pi: angle -= 2 * np.pi
    while angle < -np.pi: angle += 2 * np.pi
    return angle
#sinusoidal interpolation
def sinusoidal_interpolation(start, goal, steps):
    t     = np.linspace(0, np.pi, steps)
    alpha = (1 - np.cos(t)) / 2
    difference    = goal - start
    difference[0] = wrap_angle(difference[0])
    return start + np.outer(alpha, difference)

def get_current_joint_angles():
    return np.array([p.getJointState(arm_id, j)[0] for j in range(3)])

def calculate_goal_angles(target_position):
    target_direction = math.atan2(target_position[1], target_position[0])
    desired_yaw      = wrap_angle(target_direction - ARM_AZIMUTH_OFFSET)

    lower_limits = [desired_yaw - 0.03, -1.5708, -2.2]
    upper_limits = [desired_yaw + 0.03,  1.5708,  2.2]
    joint_ranges = [0.06, 3.1416, 4.4]
    rest_poses   = [desired_yaw, -0.8, 1.2]
#ranges for arm
    ik_solution = p.calculateInverseKinematics(arm_id, EE_LINK, target_position,
                      lowerLimits=lower_limits, upperLimits=upper_limits,
                      jointRanges=joint_ranges, restPoses=rest_poses,
                      jointDamping=[0.05, 0.05, 0.05],
                      maxNumIterations=300, residualThreshold=0.00001)
#IK calculations
    goal_angles    = np.array(ik_solution[:3])
    goal_angles[0] = desired_yaw
    goal_angles[1] = np.clip(goal_angles[1], -1.5708, 1.5708)
    goal_angles[2] = np.clip(goal_angles[2], -2.2,    2.2)
    return goal_angles

def command_joint_angles(goal_angles, label):
    start_angles = get_current_joint_angles()
    trajectory   = sinusoidal_interpolation(start_angles, goal_angles, STEPS)

    print(f"\nMoving arm to {label}")
    print("Starting angles:", np.round(start_angles, 3))
    print("Goal angles:",     np.round(goal_angles,  3))

    for waypoint in trajectory: #list of angles commanding the arm to move using constants
        p.setJointMotorControl2(arm_id, 0, p.POSITION_CONTROL, targetPosition=float(waypoint[0]), maxVelocity=2.0, force=80.0)
        p.setJointMotorControl2(arm_id, 1, p.POSITION_CONTROL, targetPosition=float(waypoint[1]), maxVelocity=2.0, force=80.0)
        p.setJointMotorControl2(arm_id, 2, p.POSITION_CONTROL, targetPosition=float(waypoint[2]), maxVelocity=2.0, force=80.0)
        p.stepSimulation()
        time.sleep(1 / 1200)

    for _ in range(48): #list of angles commanding the arm to move using constants
        p.setJointMotorControl2(arm_id, 0, p.POSITION_CONTROL, targetPosition=float(goal_angles[0]), force=80.0)
        p.setJointMotorControl2(arm_id, 1, p.POSITION_CONTROL, targetPosition=float(goal_angles[1]), force=80.0)
        p.setJointMotorControl2(arm_id, 2, p.POSITION_CONTROL, targetPosition=float(goal_angles[2]), force=80.0)
        p.stepSimulation()
        time.sleep(1 / 600)

def move_arm_to_position(target_position, label):
    goal_angles = calculate_goal_angles(target_position)
    command_joint_angles(goal_angles, label)

def attach_ball(ball_id): #we need to simulate without a grabber mechanism so it pretends the ball attatches to the arm
    constraint_id = p.createConstraint(arm_id, EE_LINK, ball_id, -1,
                                        p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])
    p.changeConstraint(constraint_id, maxForce=100)
    return constraint_id

def release_ball(constraint_id): #used to drop into the tray
    p.removeConstraint(constraint_id)
    for _ in range(72):
        p.stepSimulation()
        time.sleep(1 / 600)
#all of the steps for moving balls
def move_ball_to_tray(ball_name, ball_id, detected_position, ball_radius):
    approach_height  = 0.15 #heights specific for each step
    carry_height     = 0.22
    tray_drop_height = 0.11

    ball_approach = [detected_position[0], detected_position[1], approach_height] #paths for each ball point movement
    ball_pickup   = [detected_position[0], detected_position[1], ball_radius]
    ball_lift     = [detected_position[0], detected_position[1], carry_height]

    tray_position = TRAY_POSITIONS[ball_name]
    tray_approach = [tray_position[0], tray_position[1], carry_height] #paths for each tray point movement
    tray_drop     = [tray_position[0], tray_position[1], tray_drop_height]
    tray_lift     = [tray_position[0], tray_position[1], carry_height]

    move_arm_to_position(ball_approach, f"{ball_name} approach")
    move_arm_to_position(ball_pickup,   f"{ball_name} pickup")
    constraint_id = attach_ball(ball_id)
    move_arm_to_position(ball_lift,     f"lifting {ball_name}")
    move_arm_to_position(tray_approach, f"{ball_name} tray approach")
    move_arm_to_position(tray_drop,     f"placing {ball_name}")
    release_ball(constraint_id)
    move_arm_to_position(tray_lift,     f"leaving {ball_name} tray")

ball_information = {
    "tennis":    {"radius": constants.tennis_ball_r,    "body_id": tennis_ball},
    "baseball":  {"radius": constants.baseball_r,       "body_id": baseball},
    "ping_pong": {"radius": constants.ping_pong_ball_r, "body_id": ping_pong}
}
#save detection images
camera_image   = get_camera_image()
detected_balls = detect_all_balls(camera_image)

print("\nCamera image saved to:",    ROOT / "camera_image.png")
print("Detection result saved to:", ROOT / "detected_camera_view.png")
print("Detected balls:", detected_balls)
#ball is detected?, ball size if detected
for ball_name in ["tennis", "baseball", "ping_pong"]:
    if ball_name not in detected_balls:
        print(f"{ball_name} was not detected.")
        continue

    pixel_x, pixel_y = detected_balls[ball_name]["center"]
    ball_radius = ball_information[ball_name]["radius"]
    ball_id     = ball_information[ball_name]["body_id"]

    detected_position  = convertCoordinates(pixel_x, pixel_y, ball_radius)
    actual_position, _ = p.getBasePositionAndOrientation(ball_id)

    print(f"\n{ball_name} pixel center:",    (pixel_x, pixel_y))
    print(f"{ball_name} detected position:", np.round(detected_position, 3))
    print(f"{ball_name} actual position:",   np.round(actual_position,   3))
    print(f"{ball_name} coordinate error:",  np.round(np.array(detected_position[:2]) - np.array(actual_position[:2]), 3))

    move_ball_to_tray(ball_name, ball_id, detected_position, ball_radius)

print("\nAll detected balls were moved to their trays.")

while p.isConnected():
    p.stepSimulation()
    time.sleep(1 / 1200)