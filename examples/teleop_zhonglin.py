"""Zhonglin GELLO teleoperation for Lite6 with Genesis visualizer.

Reads joint positions from a Zhonglin GELLO leader arm (connected via USB
serial) and mirrors them onto the simulated Lite6 in Genesis World.
Optionally also commands the real Lite6 hardware via xArm SDK, using
Genesis as a real-time 3D visualizer alongside the physical robot.

Requirements:
    - Zhonglin GELLO leader arm connected to /dev/ttyUSB0
    - gello_software package installed (for the driver)
    - Calibrated joint offsets (run scripts/zhonglin_get_offset.py in gello_software)
    - (optional) xarm-python-sdk installed for real robot control

Usage:
    python examples/teleop_zhonglin.py
    python examples/teleop_zhonglin.py --port /dev/ttyUSB1
    python examples/teleop_zhonglin.py --no-gripper
    python examples/teleop_zhonglin.py --real-ip 192.168.1.226
"""

import argparse
import sys
import time

import numpy as np

import genesis as gs
import genesis_lite6
from genesis_lite6.config import (
    ARM_JOINT_NAMES,
    DEFAULT_FORCE_LOWER,
    DEFAULT_FORCE_UPPER,
    DEFAULT_KP,
    DEFAULT_KV,
)

# -----------------------------------------------------------------------
# Zhonglin GELLO calibration -- paste your values from
#   gello_software/scripts/zhonglin_get_offset.py
# -----------------------------------------------------------------------
ZHONGLIN_JOINT_IDS = (0, 1, 2, 3, 4, 5)
ZHONGLIN_JOINT_OFFSETS = (2.6245, 2.2370, 2.3797, 1.0733, 2.4172, 0.1155)  # TODO: re-calibrate
ZHONGLIN_JOINT_SIGNS = (1, 1, 1, 1, -1, 1)

ZHONGLIN_GRIPPER_ID = 6
ZHONGLIN_GRIPPER_OPEN_DEG = 110.0
ZHONGLIN_GRIPPER_CLOSE_DEG = 68.2

TELEOP_HZ = 30
SIM_DT = 0.005
SIM_SUBSTEPS = max(1, round((1.0 / TELEOP_HZ) / SIM_DT))


def main():
    parser = argparse.ArgumentParser(description="Zhonglin GELLO teleop in Genesis")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
    parser.add_argument("--no-gripper", action="store_true")
    parser.add_argument(
        "--real-ip", type=str, default=None,
        help="Lite6 IP address to also command the real robot (e.g. 192.168.1.226)",
    )
    args = parser.parse_args()

    with_gripper = not args.no_gripper

    # Import gello driver
    try:
        from gello.robots.zhonglin import ZhonglinRobot
    except ImportError:
        print("ERROR: gello_software is not installed.")
        print("Install it with: pip install -e /path/to/gello_software")
        sys.exit(1)

    # Set up gripper config
    gripper_config = None
    if with_gripper:
        gripper_config = (ZHONGLIN_GRIPPER_ID, ZHONGLIN_GRIPPER_OPEN_DEG, ZHONGLIN_GRIPPER_CLOSE_DEG)

    # Connect to GELLO leader arm
    print(f"Connecting to Zhonglin GELLO on {args.port}...")
    leader = ZhonglinRobot(
        joint_ids=ZHONGLIN_JOINT_IDS,
        joint_offsets=list(ZHONGLIN_JOINT_OFFSETS),
        joint_signs=list(ZHONGLIN_JOINT_SIGNS),
        real=True,
        port=args.port,
        gripper_config=gripper_config,
        start_joints=np.zeros(7) if with_gripper else np.zeros(6),
    )
    print("GELLO connected!")

    # Optionally connect to real Lite6 hardware
    real_robot = None
    if args.real_ip:
        try:
            from gello.robots.xarm_robot import XArmRobot
        except ImportError:
            print("ERROR: gello.robots.xarm_robot not available.")
            print("Make sure gello_software and xarm-python-sdk are installed.")
            sys.exit(1)

        print(f"Connecting to real Lite6 at {args.real_ip}...")
        real_robot = XArmRobot(
            ip=args.real_ip,
            real=True,
            num_arm_joints=6,
        )
        print("Real Lite6 connected!")

    # Set up Genesis scene
    gs.init(backend=gs.gpu)

    scene = gs.Scene(
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(1.5, -1.5, 1.5),
            camera_lookat=(0.0, 0.0, 0.3),
            camera_fov=40,
        ),
        sim_options=gs.options.SimOptions(dt=SIM_DT),
        show_viewer=True,
    )

    scene.add_entity(gs.morphs.Plane())

    lite6 = scene.add_entity(
        gs.morphs.URDF(
            file=genesis_lite6.get_urdf_path(eef="ufgripper" if with_gripper else None),
            fixed=True,
        ),
    )

    scene.build()

    motors_dof_idx = [lite6.get_joint(name).dofs_idx_local[0] for name in ARM_JOINT_NAMES]

    lite6.set_dofs_kp(kp=DEFAULT_KP, dofs_idx_local=motors_dof_idx)
    lite6.set_dofs_kv(kv=DEFAULT_KV, dofs_idx_local=motors_dof_idx)
    lite6.set_dofs_force_range(
        lower=DEFAULT_FORCE_LOWER,
        upper=DEFAULT_FORCE_UPPER,
        dofs_idx_local=motors_dof_idx,
    )

    # Gripper DOF lookup (cached outside loop)
    finger_dof = None
    finger_range = None
    if with_gripper:
        finger_joint = lite6.get_joint("finger_joint1")
        finger_dof = finger_joint.dofs_idx_local[0]
        finger_range = (finger_joint.dofs_limit[0][0], finger_joint.dofs_limit[0][1])
        lite6.set_dofs_kp(kp=np.array([500.0]), dofs_idx_local=[finger_dof])
        lite6.set_dofs_kv(kv=np.array([50.0]), dofs_idx_local=[finger_dof])
        lite6.set_dofs_force_range(
            lower=np.array([-5.0]), upper=np.array([5.0]), dofs_idx_local=[finger_dof],
        )

    # Indices of continuous (full-rotation) joints that need angle unwrapping
    CONTINUOUS_JOINT_INDICES = [0, 3, 5]  # joint1, joint4, joint6

    # Robot starts at qpos=0 (URDF default), matching GELLO convention.
    # Hold position to let PD control settle before teleop begins.
    lite6.control_dofs_position(np.zeros(6), motors_dof_idx)
    for _ in range(50):
        scene.step()

    mode_str = "sim + real" if real_robot else "sim only"
    print(f"Teleop running at {TELEOP_HZ} Hz, {SIM_SUBSTEPS} substeps ({mode_str}).")
    print("Press Ctrl+C to stop.")

    rate_dt = 1.0 / TELEOP_HZ

    try:
        while True:
            t_start = time.monotonic()

            gello_state = leader.get_joint_state()
            arm_joints = gello_state[:6].copy()

            # Unwrap continuous joints so PD control takes the shortest path
            current_qpos = lite6.get_dofs_position(motors_dof_idx)
            if hasattr(current_qpos, 'cpu'):
                current_qpos = current_qpos.cpu().numpy().flatten()
            for idx in CONTINUOUS_JOINT_INDICES:
                delta = arm_joints[idx] - current_qpos[idx]
                arm_joints[idx] = current_qpos[idx] + (delta + np.pi) % (2 * np.pi) - np.pi

            # --- Genesis sim ---
            lite6.control_dofs_position(arm_joints, motors_dof_idx)

            if finger_dof is not None and len(gello_state) > 6:
                gripper_val = gello_state[6]
                finger_pos = (1.0 - gripper_val) * (finger_range[1] - finger_range[0]) + finger_range[0]
                lite6.control_dofs_position(np.array([finger_pos]), [finger_dof])

            for _ in range(SIM_SUBSTEPS):
                scene.step()

            # --- Real robot ---
            if real_robot is not None:
                if with_gripper and len(gello_state) > 6:
                    real_robot.command_joint_state(
                        np.concatenate([arm_joints, [gello_state[6]]])
                    )
                else:
                    real_robot.command_joint_state(arm_joints)

            elapsed = time.monotonic() - t_start
            if elapsed < rate_dt:
                time.sleep(rate_dt - elapsed)

    except KeyboardInterrupt:
        print("\nStopping teleop.")
    finally:
        leader._driver.close()
        if real_robot is not None:
            real_robot.stop()
            print("Real robot disconnected.")
        print("Done.")


if __name__ == "__main__":
    main()
