from pathlib import Path
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
BALL_RADIUS = constants.tennis_ball_r
BALL_POS = [0.4, 0.35, BALL_RADIUS]
BALL_TOP = [BALL_POS[0], BALL_POS[1], BALL_RADIUS * 2 + 0.02]
EE_LINK = 3
STEPS = 180


def sinusoidal_interpolation(start, goal, steps):
    t = np.linspace(0, np.pi, steps)
    alpha = (1 - np.cos(t)) / 2
    return np.outer(1 - alpha, start) + np.outer(alpha, goal)


def render_frame():
    view = p.computeViewMatrix(
        cameraEyePosition=[0.95, -0.95, 0.55],
        cameraTargetPosition=[0.18, 0.12, 0.13],
        cameraUpVector=[0, 0, 1],
    )
    projection = p.computeProjectionMatrixFOV(45, WIDTH / HEIGHT, 0.01, 3.0)
    _, _, rgba, _, _ = p.getCameraImage(
        WIDTH,
        HEIGHT,
        viewMatrix=view,
        projectionMatrix=projection,
        renderer=p.ER_TINY_RENDERER,
    )
    frame = np.array(rgba, dtype=np.uint8).reshape(HEIGHT, WIDTH, 4)
    return Image.fromarray(frame[:, :, :3])


def main():
    OUTPUT.parent.mkdir(exist_ok=True)

    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    plane_id = p.loadURDF("plane.urdf")
    p.changeVisualShape(plane_id, -1, rgbaColor=[0.28, 0.31, 0.33, 1])

    arm_id = p.loadURDF(str(ROOT / "2AxisArm" / "2AxisArm_pybullet.urdf"), basePosition=[0, 0, 0], useFixedBase=True)
    col_ball = p.createCollisionShape(p.GEOM_SPHERE, radius=BALL_RADIUS)
    vis_ball = p.createVisualShape(p.GEOM_SPHERE, radius=BALL_RADIUS, rgbaColor=[0.8, 1.0, 0.1, 1.0])
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col_ball,
        baseVisualShapeIndex=vis_ball,
        basePosition=BALL_POS,
    )

    goal_angles = np.array(p.calculateInverseKinematics(arm_id, EE_LINK, BALL_TOP)[:3])
    trajectory = sinusoidal_interpolation(np.array([0.0, 0.0, 0.0]), goal_angles, STEPS)

    frames = []
    for index, waypoint in enumerate(trajectory):
        for joint_id, target in enumerate(waypoint):
            p.setJointMotorControl2(
                arm_id,
                joint_id,
                p.POSITION_CONTROL,
                targetPosition=float(target),
                maxVelocity=2.0,
                force=20.0,
            )
        p.stepSimulation()
        if index % 3 == 0:
            frames.append(render_frame())

    for _ in range(18):
        p.stepSimulation()
        frames.append(render_frame())

    p.disconnect()

    frames[0].save(
        OUTPUT,
        save_all=True,
        append_images=frames[1:],
        duration=45,
        loop=0,
        optimize=True,
    )
    print(f"Rendered {len(frames)} PyBullet frames to {OUTPUT}")


if __name__ == "__main__":
    main()
