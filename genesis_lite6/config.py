"""Lite6 robot configuration constants for Genesis World.

Joint specs sourced from the official xarm_ros2 xarm_description.
PD gains are starting defaults -- tune for your specific task.
"""

import math

import numpy as np

# ---------------------------------------------------------------------------
# Joint names
# ---------------------------------------------------------------------------

ARM_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")

GRIPPER_JOINT_NAMES = ("finger_joint1", "finger_joint2")

ALL_JOINT_NAMES = ARM_JOINT_NAMES + GRIPPER_JOINT_NAMES

# ---------------------------------------------------------------------------
# Link names
# ---------------------------------------------------------------------------

ARM_LINK_NAMES = ("link_base", "link1", "link2", "link3", "link4", "link5", "link6")

EEF_LINK_NAME = "link6"  # link_eef is merged into link6 by Genesis (zero offset fixed joint)

TCP_LINK_NAME = "link_tcp"

GRIPPER_LINK_NAMES = ("ufgripper_link", "ufgripper_finger1", "ufgripper_finger2")

# ---------------------------------------------------------------------------
# Joint limits (from xarm_ros2 xarm_description)
# ---------------------------------------------------------------------------

ARM_JOINT_LOWER = np.array([-2 * math.pi, -2.61799, -0.061087, -2 * math.pi, -2.1642, -2 * math.pi])
ARM_JOINT_UPPER = np.array([2 * math.pi, 2.61799, 5.235988, 2 * math.pi, 2.1642, 2 * math.pi])

ARM_EFFORT_LIMITS = np.array([50.0, 50.0, 32.0, 32.0, 32.0, 20.0])

GRIPPER_JOINT_LOWER = np.array([0.0, 0.0])
GRIPPER_JOINT_UPPER = np.array([0.0089, 0.0089])
GRIPPER_EFFORT_LIMIT = 5.0

# ---------------------------------------------------------------------------
# Home position (arm at a natural upright pose)
# ---------------------------------------------------------------------------

HOME_QPOS = np.array([0.0, 0.0, math.pi / 2, 0.0, math.pi / 2, 0.0])

# ---------------------------------------------------------------------------
# Default PD gains (reasonable starting values, tune per task)
# ---------------------------------------------------------------------------

DEFAULT_KP = np.array([4500.0, 4500.0, 3500.0, 3500.0, 2000.0, 2000.0])
DEFAULT_KV = np.array([450.0, 450.0, 350.0, 350.0, 200.0, 200.0])

DEFAULT_FORCE_LOWER = -ARM_EFFORT_LIMITS
DEFAULT_FORCE_UPPER = ARM_EFFORT_LIMITS

GRIPPER_KP = np.array([100.0, 100.0])
GRIPPER_KV = np.array([10.0, 10.0])
