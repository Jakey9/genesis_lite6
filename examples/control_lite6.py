"""Lite6 PD position control demo with gripper open/close.

Moves through several joint configurations using PD control,
then cycles the gripper open and closed as a sanity check.
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
        file=genesis_lite6.get_urdf_path(eef="ufgripper"),
        fixed=True,
    ),
)

scene.build()

# Arm motor setup
motors_dof_idx = [lite6.get_joint(name).dofs_idx_local[0] for name in ARM_JOINT_NAMES]

lite6.set_dofs_kp(kp=DEFAULT_KP, dofs_idx_local=motors_dof_idx)
lite6.set_dofs_kv(kv=DEFAULT_KV, dofs_idx_local=motors_dof_idx)
lite6.set_dofs_force_range(
    lower=DEFAULT_FORCE_LOWER,
    upper=DEFAULT_FORCE_UPPER,
    dofs_idx_local=motors_dof_idx,
)

# Gripper motor setup
finger_joint = lite6.get_joint("finger_joint1")
finger_dof = finger_joint.dofs_idx_local[0]
finger_upper = 0.0089  # max opening per finger (meters)

lite6.set_dofs_kp(kp=np.array([500.0]), dofs_idx_local=[finger_dof])
lite6.set_dofs_kv(kv=np.array([50.0]), dofs_idx_local=[finger_dof])
lite6.set_dofs_force_range(
    lower=np.array([-5.0]),
    upper=np.array([5.0]),
    dofs_idx_local=[finger_dof],
)

# Phase 1: move arm through poses (steps 0-999)
target_poses = [
    HOME_QPOS,
    np.array([1.0, 0.5, 1.0, 0.0, 1.0, 0.0]),
    np.array([-1.0, 0.5, 1.5, -0.5, 1.0, 0.5]),
    np.zeros(6),
]

horizon = 1000 if "PYTEST_VERSION" not in os.environ else 5
for i in range(horizon):
    phase = i // 250
    if i % 250 == 0 and phase < len(target_poses):
        print(f"Step {i}: moving to pose {phase}")
        lite6.control_dofs_position(target_poses[phase], motors_dof_idx)
    scene.step()

# Phase 2: gripper open/close cycles (steps 1000-1599)
gripper_steps = 600 if "PYTEST_VERSION" not in os.environ else 5
print("Gripper sanity check: cycling open/close...")
for i in range(gripper_steps):
    cycle = i // 200
    if i % 200 == 0:
        is_open = cycle % 2 == 0
        target = np.array([finger_upper if is_open else 0.0])
        print(f"  Gripper {'OPEN' if is_open else 'CLOSE'}")
        lite6.control_dofs_position(target, [finger_dof])
    scene.step()

print("Done!")
