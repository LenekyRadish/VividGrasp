import pybullet as p
import pybullet_data
import numpy as np
import random
import os
import time
import math


# load the plane
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
 
plane = p.loadURDF("plane.urdf")
p.changeVisualShape(plane, -1, rgbaColor=[0.55, 0.35, 0.2, 1])   # brown floor
p.changeDynamics(plane, -1, contactStiffness=1e6, contactDamping=1e3)

for _ in range(480):
    p.stepSimulation()
    time.sleep(1/240)    

# make the uv spheres for the texture to wrap around on
def make_uv_sphere_obj(filename, stacks=32, slices=32):
    vertices, tex_coords, faces = [], [], []

    # walk the sphere grid: rings top-to-bottom (stacks), divisions around (slices)
    for ring in range(stacks + 1):
        vertical_fraction = ring / stacks          # 0 at top pole, 1 at bottom
        theta = vertical_fraction * np.pi          # up-down angle
        for segment in range(slices + 1):
            around_fraction = segment / slices      # 0 to 1 going around
            phi = around_fraction * 2 * np.pi       # around angle
            # place a point on the sphere's surface using sphere math
            vertices.append((np.sin(theta)*np.cos(phi),
                             np.sin(theta)*np.sin(phi),
                             np.cos(theta)))
            # record where this point maps onto the flat image (the UV map)
            tex_coords.append((around_fraction, 1 - vertical_fraction))

    # connect the points into 4-corner tiles (faces)
    for ring in range(stacks):
        for segment in range(slices):
            top_left     = ring * (slices + 1) + segment + 1
            bottom_left  = top_left + slices + 1
            faces.append((top_left, top_left + 1, bottom_left + 1, bottom_left))

    with open(filename, "w") as f:
        for x, y, z in vertices:        f.write(f"v {x} {y} {z}\n")
        for u_coord, v_coord in tex_coords: f.write(f"vt {u_coord} {v_coord}\n")
        for c1, c2, c3, c4 in faces:
            f.write(f"f {c1}/{c1} {c2}/{c2} {c3}/{c3} {c4}/{c4}\n")

SPHERE_OBJ = os.path.join(os.getcwd(), "uv_sphere.obj")
make_uv_sphere_obj(SPHERE_OBJ)

# makes the balls
def create_sports_ball(radius, mass, lateral_friction, rolling_friction,
                       color=None, texture_path=None, position=[0, 0, 0]):
    collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    if texture_path:
        visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=SPHERE_OBJ,
                                           meshScale=[radius]*3, rgbaColor=[1, 1, 1, 1])
    else:
        if color is None: color = [1, 1, 1, 1]
        visual_shape = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=radius, rgbaColor=color)
    ball_id = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_shape,
                                baseVisualShapeIndex=visual_shape, basePosition=position)
    p.changeDynamics(ball_id, -1, lateralFriction=lateral_friction,
                     rollingFriction=rolling_friction, restitution=0.75)
    if texture_path:
        tex = p.loadTexture(texture_path)
        p.changeVisualShape(ball_id, -1, textureUniqueId=tex)
    return ball_id

# load in the trays
tray_scale = 0.3
tray1 = p.loadURDF("tray/traybox.urdf", basePosition=[1,  0.0, 1], globalScaling=tray_scale)
tray2 = p.loadURDF("tray/traybox.urdf", basePosition=[1,  0.5, 1], globalScaling=tray_scale)
tray3 = p.loadURDF("tray/traybox.urdf", basePosition=[1, -0.5, 1], globalScaling=tray_scale)

# allows the trays to fall
for tray in (tray1, tray2, tray3):
    p.changeDynamics(tray, -1, mass=0.5)

placed = []  

def random_safe_position(radius, min_from_origin=0.25, area=0.5):
    while True:
        x = random.uniform(-area, area)
        y = random.uniform(-area, area)

        # makes it a little bit further from the origin
        if math.hypot(x, y) < min_from_origin:
            continue  
        not_overlapping = True
        for (px, py, pr) in placed:
            dist = math.hypot(x - px, y - py)
            if dist < (radius + pr):  # prevents overlap
                not_overlapping = False
                break
        if not not_overlapping:
            continue

        placed.append((x, y, radius))

        return [x, y, 0.5]  # spawn up high so the balls drop from the sky
    
for _ in range(100):
    p.stepSimulation()
    time.sleep(1/120)    
    
# create the specific balls
tennis_ball = create_sports_ball(
    0.0327, 0.058, 0.6, 0.01,
    position=random_safe_position(0.0327),
    texture_path=r"c:\SB VexPushback\EZ-Template-Example-Project (1)\.vscode\tennis_ball.jpg")

baseball = create_sports_ball(
    0.0365, 0.145, 0.5, 0.005,
    position=random_safe_position(0.0365),
    texture_path=r"c:\SB VexPushback\EZ-Template-Example-Project (1)\.vscode\baseball.jpg")

ping_pong = create_sports_ball(
    0.02, 0.0027, 0.2, 0.001,
    position=random_safe_position(0.02),
    color=[0.9, 0.45, 0.0, 1])

while p.isConnected():
    p.stepSimulation()
    time.sleep(1/240)
