# from klampt.math import vectorops, so3, se3
# from klampt import vis

# class dummy():
#     def __init__(self):
#         ik_objective_avoid = self.orient_ee_towards_other_hand(EE_link, avoid_link)

#     def orient_ee_towards_other_hand(self, EE_link, avoid_link):
#         # Step 1: Get current and avoid poses
#         currentTransform_EE = EE_link.getTransform()
#         avoidTransform_EE = avoid_link.getTransform()

#         currentPose_EE = currentTransform_EE[1]  # [x, y, z]
#         avoidPose_EE = avoidTransform_EE[1]      # [x, y, z]

#         # Step 2: Compute Z axis (forward/camera direction)
#         z = vectorops.sub(avoidPose_EE, currentPose_EE)
#         z = vectorops.unit(z)

#         # Step 3: Try to make X axis point upward in world frame
#         x_candidate = [0, 0, 1]  # world up
#         if abs(vectorops.dot(z, x_candidate)) > 0.99:
#             x_candidate = [0, 1, 0]  # fallback if nearly parallel

#         x = vectorops.unit(vectorops.cross(x_candidate, z))  # right vector
#         y = vectorops.cross(z, x)  # complete right-handed frame

#         R = [x, y, z]  # final rotation matrix

#         # Optional: visualize
#         if self.show_vis:
#             vis.add("Current EE", EE_link)
#             vis.add("Avoid link", avoid_link)
#             vis.add("Look-at target frame", se3.from_rotation_translation(R, currentPose_EE))

#         # Step 4: Return IKObjective pointing EE toward avoid_link
#         ik_obj = self.get_ik_objective_benno(
#             EE_link,
#             currentPose_EE,  # or a position offset if you want to move it
#             R,
#             ref=None
#         )

#         return ik_obj

from klampt.math import vectorops, so3, se3
from klampt import vis
from klampt.model.coordinates import Frame
import os
import numpy as np
from klampt import *
vis.init("GLUT")

WORLD_PATH = (os.path.join(os.path.dirname(__file__), "../robot_description/klampt_world.xml"))

def make_look_at_transform(source_pos, target_pos, world_up=[0,0,1]):
    # Z axis: pointing from source to target
    z = vectorops.unit(vectorops.sub(target_pos, source_pos))
    
    # Handle degenerate case (almost parallel to up)
    if abs(vectorops.dot(z, world_up)) > 0.99:
        world_up = [0, 1, 0]

    x = vectorops.unit(vectorops.cross(world_up, z))  # right vector
    y = vectorops.cross(z, x)                         # y = z × x

    Rcols = [x, y, z]  # orientation matrix (columns: x, y, z)
    R = [ x[0], y[0], z[0],
      x[1], y[1], z[1],
      x[2], y[2], z[2] ]
    return (R, source_pos)

# Example points
EE_pos = [1, 0, 1]
target_pos = [1, 1, 1]
No_rot = np.eye(3).flatten().tolist()

# Get transform
look_at_T = make_look_at_transform(EE_pos, target_pos)
print(look_at_T)

dir_vec = vectorops.sub(target_pos, EE_pos)
dir_vec = vectorops.unit(dir_vec)

R = so3.align([0,0,1], dir_vec)

# Visualize in Klampt
# vis.add("EE look-at frame", look_at_T)
world = WorldModel()
world.loadFile(WORLD_PATH)
planner_robot = world.robot(0)
#vis.add("world", world)
frame_obj = Frame("world", worldCoordinates=look_at_T)
#vis.add("look_at", frame_obj)

vis.add("Gazing passive hand", (R, EE_pos))
vis.add("Initial passive hand", (No_rot, EE_pos))
vis.add("Initial active hand", (No_rot, target_pos))
        
vis.run()