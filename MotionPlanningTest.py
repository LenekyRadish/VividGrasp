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

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

MIN_X = -0.5
MAX_X = 0.5
MIN_Y = -0.5
MAX_Y = 0.5

EE_LINK = 3
STEPS = 480


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

plane = p.loadURDF("plane.urdf")
p.changeVisualShape(plane, -1, rgbaColor=[0.55, 0.35, 0.2, 1])
p.changeDynamics(plane, -1, contactStiffness=1e6, contactDamping=1e3)


os.chdir(ARM_FOLDER)
arm_id = p.loadURDF("2AxisArm_pybullet.urdf", basePosition=[0, 0, 0], useFixedBase=True)
os.chdir(ROOT)

print(f"The arm is loaded with {p.getNumJoints(arm_id)} total joints.")


def create_sports_ball(radius, mass, lateral_friction, rolling_friction, color, position):
    collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)

    ball_id = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_shape, baseVisualShapeIndex=visual_shape, basePosition=position)

    p.changeDynamics(ball_id, -1, lateralFriction=lateral_friction, rollingFriction=rolling_friction, restitution=0.75)

    return ball_id


placed = []


def random_safe_position(radius, min_from_origin=0.25, area=0.45):
    while True:
        x = random.uniform(-area, area)
        y = random.uniform(-area, area)

        if math.hypot(x, y) < min_from_origin:
            continue

        location_is_safe = True

        for previous_x, previous_y, previous_radius in placed:
            if math.hypot(x - previous_x, y - previous_y) < radius + previous_radius + 0.05:
                location_is_safe = False
                break

        if location_is_safe:
            placed.append((x, y, radius))
            return [x, y, 0.5]


tennis_ball = create_sports_ball(constants.tennis_ball_r, 0.058, 0.6, 0.01, [0.2, 1.0, 0.1, 1], random_safe_position(constants.tennis_ball_r))
baseball = create_sports_ball(constants.baseball_r, 0.145, 0.5, 0.005, [1.0, 1.0, 1.0, 1], random_safe_position(constants.baseball_r))
ping_pong = create_sports_ball(constants.ping_pong_ball_r, 0.0027, 0.2, 0.001, [0.9, 0.45, 0.0, 1], random_safe_position(constants.ping_pong_ball_r))


for _ in range(360):
    p.stepSimulation()
    time.sleep(1 / 240)


def get_camera_image():
    view = p.computeViewMatrix([0, 0, 1.5], [0, 0, 0], [0, 1, 0])
    projection = p.computeProjectionMatrixFOV(60, CAMERA_WIDTH / CAMERA_HEIGHT, 0.1, 3.0)

    _, _, rgba, _, _ = p.getCameraImage(CAMERA_WIDTH, CAMERA_HEIGHT, view, projection)
    rgba = np.array(rgba, dtype=np.uint8).reshape(CAMERA_HEIGHT, CAMERA_WIDTH, 4)

    image = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
    cv2.imwrite(str(ROOT / "camera_image.png"), image)

    return image


color_ranges = {
    "tennis": [{"lower": np.array([25, 45, 45]), "upper": np.array([85, 255, 255])}],
    "baseball": [{"lower": np.array([0, 0, 150]), "upper": np.array([180, 50, 255])}],
    "ping_pong": [{"lower": np.array([5, 85, 150]), "upper": np.array([22, 255, 255])}]
}


def detect_all_balls(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    result_image = image.copy()
    detected_balls = {}

    for ball_name, ranges in color_ranges.items():
        color_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for color_range in ranges:
            mask = cv2.inRange(hsv, color_range["lower"], color_range["upper"])
            color_mask = cv2.bitwise_or(color_mask, mask)

        cv2.imwrite(str(ROOT / f"{ball_name}_mask.png"), color_mask)

        contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        best_contour = None
        best_area = 0

        for contour in contours:
            area = cv2.contourArea(contour)

            if area > 100 and area > best_area:
                moments = cv2.moments(contour)

                if moments["m00"] != 0:
                    best_contour = contour
                    best_area = area

        if best_contour is not None:
            moments = cv2.moments(best_contour)
            center_x = int(moments["m10"] / moments["m00"])
            center_y = int(moments["m01"] / moments["m00"])

            detected_balls[ball_name] = {"center": (center_x, center_y), "area": best_area}

            cv2.drawContours(result_image, [best_contour], -1, (0, 0, 0), 3)
            cv2.circle(result_image, (center_x, center_y), 6, (0, 0, 255), -1)
            cv2.putText(result_image, ball_name, (center_x - 30, center_y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)

    cv2.imwrite(str(ROOT / "detected_camera_view.png"), result_image)

    return detected_balls


def convertCoordinates(px, py, ball_radius, min_x, max_x, min_y, max_y, imgW=CAMERA_WIDTH, imgH=CAMERA_HEIGHT):
    px = max(0, min(px, imgW))
    py = max(0, min(py, imgH))

    norm_x = px / imgW
    norm_y = py / imgH

    world_x = min_x + norm_x * (max_x - min_x)
    world_y = max_y - norm_y * (max_y - min_y)

    return [world_x, world_y, ball_radius]


def sinusoidal_interpolation(start, goal, steps):
    t = np.linspace(0, np.pi, steps)
    alpha = (1 - np.cos(t)) / 2

    return np.outer(1 - alpha, start) + np.outer(alpha, goal)


def get_current_joint_angles():
    return np.array([p.getJointState(arm_id, 0)[0], p.getJointState(arm_id, 1)[0], p.getJointState(arm_id, 2)[0]])


def move_arm_to_target(target_position, ball_name):
    print(f"\nMoving arm to {ball_name}")
    print("Target position:", np.round(target_position, 3))

    ik_solution = p.calculateInverseKinematics(arm_id, EE_LINK, target_position)
    goal_angles = np.array(ik_solution[:3])
    start_angles = get_current_joint_angles()
    trajectory = sinusoidal_interpolation(start_angles, goal_angles, STEPS)

    print(f"IK solution — yaw: {goal_angles[0]:.2f}, shoulder: {goal_angles[1]:.2f}, elbow: {goal_angles[2]:.2f}")

    for waypoint in trajectory:
        for joint in range(3):
            p.setJointMotorControl2(arm_id, joint, p.POSITION_CONTROL, targetPosition=float(waypoint[joint]), maxVelocity=2.0, force=20.0)

        p.stepSimulation()
        time.sleep(1 / 240)

    for _ in range(120):
        p.stepSimulation()
        time.sleep(1 / 240)


camera_image = get_camera_image()
detected_balls = detect_all_balls(camera_image)

print("\nCamera image saved to:", ROOT / "camera_image.png")
print("Detection result saved to:", ROOT / "detected_camera_view.png")
print("Detected balls:", detected_balls)


ball_information = {
    "tennis": {"radius": constants.tennis_ball_r, "body_id": tennis_ball},
    "baseball": {"radius": constants.baseball_r, "body_id": baseball},
    "ping_pong": {"radius": constants.ping_pong_ball_r, "body_id": ping_pong}
}


for ball_name in ["tennis", "baseball", "ping_pong"]:
    if ball_name not in detected_balls:
        print(f"{ball_name} was not detected.")
        continue

    pixel_x, pixel_y = detected_balls[ball_name]["center"]
    ball_radius = ball_information[ball_name]["radius"]

    detected_position = convertCoordinates(pixel_x, pixel_y, ball_radius, MIN_X, MAX_X, MIN_Y, MAX_Y)
    target_position = [detected_position[0], detected_position[1], ball_radius]

    actual_position, _ = p.getBasePositionAndOrientation(ball_information[ball_name]["body_id"])

    print(f"\n{ball_name} pixel center:", (pixel_x, pixel_y))
    print(f"{ball_name} detected position:", np.round(detected_position, 3))
    print(f"{ball_name} actual position:", np.round(actual_position, 3))

    move_arm_to_target(target_position, ball_name)


print("\nAll detected-ball motions completed.")


while p.isConnected():
    p.stepSimulation()
    time.sleep(1 / 240)