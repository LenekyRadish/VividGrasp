from pathlib import Path
import math
import sys

import numpy as np
import pybullet as p
import pybullet_data
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import constants  # noqa: E402


OUTPUT = ROOT / "public" / "pybullet-showcase.gif"
WIDTH = 960
HEIGHT = 540
EE_LINK = 3
ARM_AZIMUTH_OFFSET = 1.0988
STEPS_PER_MOVE = 54
FRAME_STRIDE = 3

TRAY_POSITIONS = {
    "tennis": [0.32, 0.18, 0.0],
    "baseball": [0.34, 0.0, 0.0],
    "ping_pong": [0.32, -0.18, 0.0],
}

BALLS = {
    "tennis": {
        "radius": constants.tennis_ball_r,
        "mass": 0.058,
        "color": [0.1, 1.0, 0.05, 1.0],
        "start": [-0.23, 0.2, constants.tennis_ball_r],
    },
    "baseball": {
        "radius": constants.baseball_r,
        "mass": 0.145,
        "color": [1.0, 1.0, 1.0, 1.0],
        "start": [-0.19, -0.16, constants.baseball_r],
    },
    "ping_pong": {
        "radius": constants.ping_pong_ball_r,
        "mass": 0.0027,
        "color": [1.0, 0.35, 0.0, 1.0],
        "start": [0.08, -0.27, constants.ping_pong_ball_r],
    },
}


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def sinusoidal_interpolation(start, goal, steps):
    t = np.linspace(0, np.pi, steps)
    alpha = (1 - np.cos(t)) / 2
    difference = goal - start
    difference[0] = wrap_angle(difference[0])
    return start + np.outer(alpha, difference)


def get_current_joint_angles(arm_id):
    return np.array([p.getJointState(arm_id, joint_id)[0] for joint_id in range(3)])


def calculate_goal_angles(arm_id, target_position):
    target_direction = math.atan2(target_position[1], target_position[0])
    desired_yaw = wrap_angle(target_direction - ARM_AZIMUTH_OFFSET)

    ik_solution = p.calculateInverseKinematics(
        arm_id,
        EE_LINK,
        target_position,
        lowerLimits=[desired_yaw - 0.03, -1.5708, -2.2],
        upperLimits=[desired_yaw + 0.03, 1.5708, 2.2],
        jointRanges=[0.06, 3.1416, 4.4],
        restPoses=[desired_yaw, -0.8, 1.2],
        jointDamping=[0.05, 0.05, 0.05],
        maxNumIterations=300,
        residualThreshold=0.00001,
    )

    goal_angles = np.array(ik_solution[:3])
    goal_angles[0] = desired_yaw
    goal_angles[1] = np.clip(goal_angles[1], -1.5708, 1.5708)
    goal_angles[2] = np.clip(goal_angles[2], -2.2, 2.2)
    return goal_angles


def render_frame():
    view = p.computeViewMatrix(
        cameraEyePosition=[0.88, -0.78, 0.55],
        cameraTargetPosition=[0.1, 0.0, 0.12],
        cameraUpVector=[0, 0, 1],
    )
    projection = p.computeProjectionMatrixFOV(48, WIDTH / HEIGHT, 0.01, 3.0)
    _, _, rgba, _, _ = p.getCameraImage(
        WIDTH,
        HEIGHT,
        viewMatrix=view,
        projectionMatrix=projection,
        renderer=p.ER_TINY_RENDERER,
    )
    frame = np.array(rgba, dtype=np.uint8).reshape(HEIGHT, WIDTH, 4)
    return Image.fromarray(frame[:, :, :3]).resize((640, 360), Image.Resampling.LANCZOS)


def create_box(name, position, half_extents, color):
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
    body_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=position,
    )
    return body_id


def create_bucket(center, color):
    x, y, z = center
    base_z = z + 0.01
    wall_z = z + 0.055
    create_box("bucket-base", [x, y, base_z], [0.055, 0.045, 0.01], color)
    create_box("bucket-front", [x, y - 0.045, wall_z], [0.055, 0.006, 0.045], color)
    create_box("bucket-back", [x, y + 0.045, wall_z], [0.055, 0.006, 0.045], color)
    create_box("bucket-left", [x - 0.055, y, wall_z], [0.006, 0.045, 0.045], color)
    create_box("bucket-right", [x + 0.055, y, wall_z], [0.006, 0.045, 0.045], color)


def create_ball(ball):
    collision = p.createCollisionShape(p.GEOM_SPHERE, radius=ball["radius"])
    visual = p.createVisualShape(p.GEOM_SPHERE, radius=ball["radius"], rgbaColor=ball["color"])
    body_id = p.createMultiBody(
        baseMass=ball["mass"],
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=ball["start"],
    )
    p.changeDynamics(body_id, -1, lateralFriction=0.6, rollingFriction=0.01, restitution=0.25)
    return body_id


def settle(frames, steps=30):
    for step in range(steps):
        p.stepSimulation()
        if step % FRAME_STRIDE == 0:
            frames.append(render_frame())


def command_joint_angles(arm_id, goal_angles, frames):
    start_angles = get_current_joint_angles(arm_id)
    trajectory = sinusoidal_interpolation(start_angles, goal_angles, STEPS_PER_MOVE)

    for index, waypoint in enumerate(trajectory):
        for joint_id, target in enumerate(waypoint):
            p.setJointMotorControl2(
                arm_id,
                joint_id,
                p.POSITION_CONTROL,
                targetPosition=float(target),
                maxVelocity=2.0,
                force=80.0,
            )
        p.stepSimulation()
        if index % FRAME_STRIDE == 0:
            frames.append(render_frame())


def move_arm_to_position(arm_id, target_position, frames):
    goal_angles = calculate_goal_angles(arm_id, target_position)
    command_joint_angles(arm_id, goal_angles, frames)


def move_ball_to_bucket(arm_id, ball_id, ball_name, frames):
    ball = BALLS[ball_name]
    radius = ball["radius"]
    start = ball["start"]
    bucket = TRAY_POSITIONS[ball_name]

    approach_height = 0.15
    carry_height = 0.22
    drop_height = 0.11

    move_arm_to_position(arm_id, [start[0], start[1], approach_height], frames)
    move_arm_to_position(arm_id, [start[0], start[1], radius + 0.01], frames)

    constraint_id = p.createConstraint(
        arm_id,
        EE_LINK,
        ball_id,
        -1,
        p.JOINT_FIXED,
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
    )
    p.changeConstraint(constraint_id, maxForce=100)

    move_arm_to_position(arm_id, [start[0], start[1], carry_height], frames)
    move_arm_to_position(arm_id, [bucket[0], bucket[1], carry_height], frames)
    move_arm_to_position(arm_id, [bucket[0], bucket[1], drop_height], frames)

    p.removeConstraint(constraint_id)
    settle(frames, steps=24)
    move_arm_to_position(arm_id, [bucket[0], bucket[1], carry_height], frames)


def main():
    OUTPUT.parent.mkdir(exist_ok=True)

    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    plane_id = p.loadURDF("plane.urdf")
    p.changeVisualShape(plane_id, -1, rgbaColor=[0.28, 0.31, 0.33, 1])

    arm_id = p.loadURDF(str(ROOT / "2AxisArm" / "2AxisArm_pybullet.urdf"), basePosition=[0, 0, 0], useFixedBase=True)

    create_bucket(TRAY_POSITIONS["tennis"], [0.18, 0.45, 0.1, 1.0])
    create_bucket(TRAY_POSITIONS["baseball"], [0.9, 0.9, 0.82, 1.0])
    create_bucket(TRAY_POSITIONS["ping_pong"], [0.9, 0.35, 0.08, 1.0])

    ball_ids = {name: create_ball(ball) for name, ball in BALLS.items()}

    frames = []
    settle(frames, steps=42)
    for ball_name in ["tennis", "baseball", "ping_pong"]:
        move_ball_to_bucket(arm_id, ball_ids[ball_name], ball_name, frames)

    settle(frames, steps=42)
    p.disconnect()

    frames[0].save(
        OUTPUT,
        save_all=True,
        append_images=frames[1:],
        duration=40,
        loop=0,
        optimize=True,
    )
    print(f"Rendered {len(frames)} PyBullet frames to {OUTPUT}")


if __name__ == "__main__":
    main()
