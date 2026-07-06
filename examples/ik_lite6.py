"""Lite6 inverse kinematics demo.

Moves the end-effector along a circular trajectory using the built-in
IK solver, then replays the trajectory with PD control.
"""

import os

import numpy as np

import genesis as gs
import genesis_lite6
from genesis_lite6.config import (
    ARM_JOINT_NAMES,
    DEFAULT_FORCE_LOWER,
    DEFAULT_FORCE_UPPER,
    DEFAULT_KP,
    DEFAULT_KV,
    EEF_LINK_NAME,
    HOME_QPOS,
)

gs.init(backend=gs.gpu)

scene = gs.Scene(
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(1.5, -1.5, 1.5),
        camera_lookat=(0.0, 0.0, 0.3),
        camera_fov=40,
    ),
    sim_options=gs.options.SimOptions(dt=0.01),
    show_viewer=True,
)

scene.add_entity(gs.morphs.Plane())

lite6 = scene.add_entity(
    gs.morphs.URDF(
        file=genesis_lite6.get_urdf_path(),
        fixed=True,
    ),
)

scene.build()

# Set the robot to home position
lite6.set_qpos(HOME_QPOS)
scene.step()

ee_link = lite6.get_link(EEF_LINK_NAME)
motors_dof_idx = [lite6.get_joint(name).dofs_idx_local[0] for name in ARM_JOINT_NAMES]

# Get initial EE position as circle center
ee_pos = ee_link.get_pos().cpu().numpy().flatten()
center = ee_pos.copy()
radius = 0.05

# Phase 1: IK visualization (kinematic, no physics)
print("Phase 1: IK trajectory (kinematic)")
n_ik_steps = 500 if "PYTEST_VERSION" not in os.environ else 5
waypoints = []

for i in range(n_ik_steps):
    angle = 2 * np.pi * i / n_ik_steps
    target_pos = center + np.array([
        radius * np.cos(angle),
        radius * np.sin(angle),
        0.0,
    ])

    qpos = lite6.inverse_kinematics(
        link=ee_link,
        pos=target_pos,
    )
    waypoints.append(qpos.cpu().numpy().flatten()[:6])
    lite6.set_qpos(qpos)
    scene.visualizer.update()

# Phase 2: replay with PD control (physics)
print("Phase 2: PD control replay")
lite6.set_dofs_kp(kp=DEFAULT_KP, dofs_idx_local=motors_dof_idx)
lite6.set_dofs_kv(kv=DEFAULT_KV, dofs_idx_local=motors_dof_idx)
lite6.set_dofs_force_range(
    lower=DEFAULT_FORCE_LOWER,
    upper=DEFAULT_FORCE_UPPER,
    dofs_idx_local=motors_dof_idx,
)

lite6.set_qpos(HOME_QPOS)
scene.step()

n_pd_steps = len(waypoints) * 4 if "PYTEST_VERSION" not in os.environ else 5
for i in range(n_pd_steps):
    wp_idx = (i // 4) % len(waypoints)
    lite6.control_dofs_position(waypoints[wp_idx], motors_dof_idx)
    scene.step()

print("Done!")
