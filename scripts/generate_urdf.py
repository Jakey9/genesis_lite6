#!/usr/bin/env python3
"""Generate standalone Lite6 URDF files from xarm_ros2 xarm_description data.

Reads kinematics and inertial YAML configs from xarm_ros2, then produces
modular URDF files with relative mesh paths suitable for Genesis:
  - lite6_base.urdf     (arm only + link_eef)
  - eef_ufgripper.urdf     (ufgripper parallel gripper fragment, attaches to link_eef)

These are merged at runtime by genesis_lite6.get_urdf_path(eef=...).

Usage:
    python scripts/generate_urdf.py [--xarm-description /path/to/xarm_description]
"""

import argparse
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from xml.dom import minidom

import yaml


def load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def indent_xml(elem: ET.Element) -> str:
    raw = ET.tostring(elem, encoding="unicode")
    parsed = minidom.parseString(raw)
    lines = parsed.toprettyxml(indent="  ").split("\n")
    # Drop the XML declaration line added by minidom
    return "\n".join(lines[1:])


def add_material_defs(robot: ET.Element):
    for name, rgba in [
        ("White", "1.0 1.0 1.0 1.0"),
        ("Silver", "0.753 0.753 0.753 1.0"),
        ("Black", "0.0 0.0 0.0 1.0"),
    ]:
        mat = ET.SubElement(robot, "material", name=name)
        ET.SubElement(mat, "color", rgba=rgba)


def add_link(
    robot: ET.Element,
    name: str,
    mass: float,
    origin_xyz: str,
    ixx: float, ixy: float, ixz: float,
    iyy: float, iyz: float, izz: float,
    visual_mesh: str,
    collision_mesh: str,
    material: str = "White",
    visual_origin_xyz: str = "0 0 0",
):
    link = ET.SubElement(robot, "link", name=name)

    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", xyz=origin_xyz, rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=str(mass))
    ET.SubElement(
        inertial, "inertia",
        ixx=str(ixx), ixy=str(ixy), ixz=str(ixz),
        iyy=str(iyy), iyz=str(iyz), izz=str(izz),
    )

    visual = ET.SubElement(link, "visual")
    geom_v = ET.SubElement(visual, "geometry")
    ET.SubElement(geom_v, "mesh", filename=visual_mesh)
    ET.SubElement(visual, "origin", xyz=visual_origin_xyz, rpy="0 0 0")
    mat_v = ET.SubElement(visual, "material", name=material)

    collision = ET.SubElement(link, "collision")
    geom_c = ET.SubElement(collision, "geometry")
    ET.SubElement(geom_c, "mesh", filename=collision_mesh)
    ET.SubElement(collision, "origin", xyz=visual_origin_xyz, rpy="0 0 0")


def add_revolute_joint(
    robot: ET.Element,
    name: str,
    parent: str,
    child: str,
    origin_xyz: str,
    origin_rpy: str,
    lower: float,
    upper: float,
    effort: float,
    velocity: float = 3.14,
):
    joint = ET.SubElement(robot, "joint", name=name, type="revolute")
    ET.SubElement(joint, "origin", xyz=origin_xyz, rpy=origin_rpy)
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    ET.SubElement(joint, "axis", xyz="0 0 1")
    ET.SubElement(
        joint, "limit",
        lower=str(lower), upper=str(upper),
        effort=str(effort), velocity=str(velocity),
    )
    ET.SubElement(joint, "dynamics", damping="1.0", friction="1.0")


def build_arm_urdf(kin: dict, inertials: dict, mesh_prefix: str) -> ET.Element:
    """Build the 6-DOF Lite6 arm URDF tree."""
    robot = ET.Element("robot", name="lite6")
    add_material_defs(robot)

    # link_base (inertial hardcoded in xacro, not from YAML)
    add_link(
        robot, "link_base",
        mass=1.65394,
        origin_xyz="-0.00829545 3.26357e-05 0.0631195",
        ixx=0, ixy=0, ixz=0, iyy=0, iyz=0, izz=0,
        visual_mesh=f"{mesh_prefix}/lite6/visual/link_base.dae",
        collision_mesh=f"{mesh_prefix}/lite6/visual/link_base.dae",
        material="White",
    )

    joint_specs = [
        # (joint_name, effort, lower, upper)
        ("joint1", 50.0, -6.28319, 6.28319),
        ("joint2", 50.0, -2.61799, 2.61799),
        ("joint3", 32.0, -0.061087, 5.235988),
        ("joint4", 32.0, -6.28319, 6.28319),
        ("joint5", 32.0, -2.1642, 2.1642),
        ("joint6", 20.0, -6.28319, 6.28319),
    ]
    link_materials = ["White", "White", "White", "White", "White", "Silver"]

    for i, (jname, effort, lower, upper) in enumerate(joint_specs, 1):
        link_name = f"link{i}"
        parent = "link_base" if i == 1 else f"link{i - 1}"

        k = kin[jname]
        origin_xyz = f"{k['x']} {k['y']} {k['z']}"
        origin_rpy = f"{k['roll']} {k['pitch']} {k['yaw']}"

        lp = inertials[link_name]
        o = lp["origin"]
        ip = lp["inertia"]

        # link2 visual mesh has a Y-offset to compensate joint2's Y origin
        visual_origin = "0 0 0"
        if i == 2:
            visual_origin = f"0 0 {-k['y']}"

        add_link(
            robot, link_name,
            mass=lp["mass"],
            origin_xyz=f"{o['x']} {o['y']} {o['z']}",
            ixx=ip["ixx"], ixy=ip["ixy"], ixz=ip["ixz"],
            iyy=ip["iyy"], iyz=ip["iyz"], izz=ip["izz"],
            visual_mesh=f"{mesh_prefix}/lite6/visual/{link_name}.dae",
            collision_mesh=f"{mesh_prefix}/lite6/visual/{link_name}.dae",
            material=link_materials[i - 1],
            visual_origin_xyz=visual_origin,
        )

        add_revolute_joint(
            robot, jname, parent, link_name,
            origin_xyz=origin_xyz, origin_rpy=origin_rpy,
            lower=lower, upper=upper, effort=effort,
        )

    # Fixed world joint
    world_link = ET.Element("link", name="world")
    robot.insert(0, world_link)
    world_joint = ET.SubElement(robot, "joint", name="world_joint", type="fixed")
    ET.SubElement(world_joint, "parent", link="world")
    ET.SubElement(world_joint, "child", link="link_base")
    ET.SubElement(world_joint, "origin", xyz="0 0 0", rpy="0 0 0")

    # link_eef (end-effector frame, fixed to link6)
    eef_link = ET.SubElement(robot, "link", name="link_eef")
    eef_joint = ET.SubElement(robot, "joint", name="joint_eef", type="fixed")
    ET.SubElement(eef_joint, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(eef_joint, "parent", link="link6")
    ET.SubElement(eef_joint, "child", link="link_eef")

    return robot


def build_eef_ufgripper(mesh_prefix: str) -> ET.Element:
    """Build a standalone EEF fragment for the UFactory parallel gripper."""
    robot = ET.Element("robot", name="eef_ufgripper")

    # gripper_fix joint (fixed, attaches to link_eef)
    gfix = ET.SubElement(robot, "joint", name="gripper_fix", type="fixed")
    ET.SubElement(gfix, "parent", link="link_eef")
    ET.SubElement(gfix, "child", link="ufgripper_link")
    ET.SubElement(gfix, "origin", xyz="0 0 0", rpy="0 0 0")

    # Gripper body
    gripper_link = ET.SubElement(robot, "link", name="ufgripper_link")
    gi = ET.SubElement(gripper_link, "inertial")
    ET.SubElement(gi, "origin", xyz="0.0 0.0 0.030", rpy="0 0 0")
    ET.SubElement(gi, "mass", value="0.25")
    ET.SubElement(
        gi, "inertia",
        ixx="0.00047106", ixy="3.9292e-07", ixz="2.6537e-06",
        iyy="0.00033072", iyz="-1.0975e-05", izz="0.00025642",
    )
    gv = ET.SubElement(gripper_link, "visual")
    gvg = ET.SubElement(gv, "geometry")
    ET.SubElement(gvg, "mesh", filename=f"{mesh_prefix}/gripper/lite/visual/shell.dae")
    ET.SubElement(gv, "origin", xyz="0 0 0", rpy="0 0 0")
    gc = ET.SubElement(gripper_link, "collision")
    gcg = ET.SubElement(gc, "geometry")
    ET.SubElement(gcg, "mesh", filename=f"{mesh_prefix}/gripper/lite/visual/shell.dae")
    ET.SubElement(gc, "origin", xyz="0 0 0", rpy="0 0 0")

    # Finger 1
    for fname, sign, finger_mesh in [
        ("ufgripper_finger1", 1, "finger1"),
        ("ufgripper_finger2", -1, "finger2"),
    ]:
        fl = ET.SubElement(robot, "link", name=fname)
        fi = ET.SubElement(fl, "inertial")
        ET.SubElement(fi, "origin", xyz=f"0.0 {sign * 0.01} 0.0086", rpy="0 0 0")
        ET.SubElement(fi, "mass", value="0.0163")
        iyz_val = f"{sign * -3.1e-07}"
        ET.SubElement(
            fi, "inertia",
            ixx="1.425e-06", ixy="0.0", ixz="0.0",
            iyy="1.63e-06", iyz=iyz_val, izz="8.63e-07",
        )
        fv = ET.SubElement(fl, "visual")
        fvg = ET.SubElement(fv, "geometry")
        ET.SubElement(fvg, "mesh", filename=f"{mesh_prefix}/gripper/lite/visual/{finger_mesh}.dae")
        ET.SubElement(fv, "origin", xyz="0 0 0", rpy="0 0 0")
        fc = ET.SubElement(fl, "collision")
        fcg = ET.SubElement(fc, "geometry")
        ET.SubElement(fcg, "mesh", filename=f"{mesh_prefix}/gripper/lite/visual/{finger_mesh}.dae")
        ET.SubElement(fc, "origin", xyz="0 0 0", rpy="0 0 0")

    # finger_joint1 (prismatic)
    fj1 = ET.SubElement(robot, "joint", name="finger_joint1", type="prismatic")
    ET.SubElement(fj1, "origin", xyz="0 0 0.0543", rpy="0 0 0")
    ET.SubElement(fj1, "parent", link="ufgripper_link")
    ET.SubElement(fj1, "child", link="ufgripper_finger1")
    ET.SubElement(fj1, "axis", xyz="0 1 0")
    ET.SubElement(fj1, "limit", effort="5", lower="0", upper="0.0089", velocity="2")

    # finger_joint2 (prismatic, mimic)
    fj2 = ET.SubElement(robot, "joint", name="finger_joint2", type="prismatic")
    ET.SubElement(fj2, "origin", xyz="0 0 0.0543", rpy="0 0 0")
    ET.SubElement(fj2, "parent", link="ufgripper_link")
    ET.SubElement(fj2, "child", link="ufgripper_finger2")
    ET.SubElement(fj2, "axis", xyz="0 -1 0")
    ET.SubElement(fj2, "limit", effort="5", lower="0", upper="0.0089", velocity="2")
    ET.SubElement(fj2, "mimic", joint="finger_joint1", multiplier="1", offset="0")

    # link_tcp + fixed joint
    ET.SubElement(robot, "link", name="link_tcp")
    jtcp = ET.SubElement(robot, "joint", name="joint_tcp", type="fixed")
    ET.SubElement(jtcp, "origin", xyz="0 0 0.0836", rpy="0 0 0")
    ET.SubElement(jtcp, "parent", link="ufgripper_link")
    ET.SubElement(jtcp, "child", link="link_tcp")

    return robot


def write_urdf(robot: ET.Element, path: str):
    xml_str = indent_xml(robot)
    # Clean up excessive blank lines
    lines = [l for l in xml_str.split("\n") if l.strip() != ""]
    with open(path, "w") as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write("\n".join(lines))
        f.write("\n")
    print(f"  Written: {path}")


def main():
    parser = argparse.ArgumentParser(description="Generate Lite6 URDF files")
    parser.add_argument(
        "--xarm-description",
        default=None,
        help="Path to xarm_description package root",
    )
    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    project_root = script_dir.parent

    if args.xarm_description:
        xarm_desc = Path(args.xarm_description)
    else:
        # Try common locations relative to this project
        candidates = [
            project_root.parent / "xarm_ros2" / "xarm_description",
            Path("/home/jake.tan/P_PAI/xarm_ros2/xarm_description"),
        ]
        xarm_desc = None
        for c in candidates:
            if c.is_dir():
                xarm_desc = c
                break
        if xarm_desc is None:
            raise FileNotFoundError(
                "Could not find xarm_description. Use --xarm-description to specify."
            )

    print(f"Using xarm_description: {xarm_desc}")

    kin_path = xarm_desc / "config" / "kinematics" / "default" / "lite6_default_kinematics.yaml"
    inertial_path = xarm_desc / "config" / "link_inertial" / "xarm6_type9_HT_BR2.yaml"

    kin_data = load_yaml(str(kin_path))["kinematics"]
    inertial_data = load_yaml(str(inertial_path))

    urdf_dir = project_root / "genesis_lite6" / "assets" / "urdf"
    urdf_dir.mkdir(parents=True, exist_ok=True)

    # Mesh paths relative from urdf/ to meshes/
    mesh_prefix = "../meshes"

    print("Generating lite6_base.urdf (arm only)...")
    arm_robot = build_arm_urdf(kin_data, inertial_data, mesh_prefix)
    write_urdf(arm_robot, str(urdf_dir / "lite6_base.urdf"))

    print("Generating eef_ufgripper.urdf (gripper fragment)...")
    eef_robot = build_eef_ufgripper(mesh_prefix)
    write_urdf(eef_robot, str(urdf_dir / "eef_ufgripper.urdf"))

    print("Done! Use genesis_lite6.get_urdf_path(eef='ufgripper') to load combined.")


if __name__ == "__main__":
    main()
