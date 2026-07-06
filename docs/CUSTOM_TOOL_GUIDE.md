# Adding a Custom Tool / Gripper to Genesis Lite6

This guide walks through adding a new end-effector tool (e.g., the OpenParallelGripper SCS3045M) to the `genesis_lite6` package for use in Genesis World simulation.

## Prerequisites

Before you start, you need:

1. **A CAD model** of your tool (STEP, Fusion 360, Onshape, etc.)
2. **Exported mesh files** (STL or DAE) for each moving part
3. **Physical measurements**: mass, inertia (estimate from CAD), joint stroke, TCP offset

## Step 1: Prepare mesh files

Export your gripper assembly into **individual mesh files**, one per link (rigid body). At minimum you need:

| Link | Example source (OpenParallelGripper) | Notes |
|------|--------------------------------------|-------|
| Gripper body | `ToArm.step` + `SCS3045MBase.step` fused | Everything that mounts rigidly to the arm flange |
| Finger left | `claw_base.step` + `claw50mm.step` (left side) | The moving jaw |
| Finger right | Same parts, mirrored | Or same mesh if symmetric |

Export as **STL** (preferred for collision) or **DAE** (preferred for visuals with color). Place them in the assets directory:

```
genesis_lite6/assets/meshes/gripper/open_parallel/
  body.stl        (or .dae)
  finger_left.stl
  finger_right.stl
```

Tips:
- Align the mesh origin to the **mounting flange center** (where it bolts to `link6`).
- Z-axis should point away from the arm (along the tool direction).
- Keep mesh complexity reasonable (< 50k triangles per part) for fast collision.

## Step 2: Measure physical properties

You need these values for the URDF:

| Property | How to get it | Example (OpenParallelGripper) |
|----------|---------------|-------------------------------|
| **Total mass** | Scale or CAD | ~340 g (0.34 kg) |
| **Body mass** | Subtract finger mass from total | ~300 g estimate |
| **Finger mass** (each) | CAD material density | ~20 g estimate |
| **Finger stroke** (per side) | Measure or from docs | 27.5 mm (0.0275 m), 55 mm total |
| **Joint axis** | Which direction fingers slide | Typically Y axis (lateral) |
| **Inertia** | CAD mass properties or estimate | Use a box approximation if needed |
| **TCP offset** | Distance from flange to fingertip center | Measure from CAD assembly |

For inertia estimation, a box approximation works for simulation:

```
Ixx = (1/12) * m * (h^2 + d^2)
Iyy = (1/12) * m * (w^2 + d^2)
Izz = (1/12) * m * (w^2 + h^2)
```

## Step 3: Create the EEF fragment URDF

Create a new file at `genesis_lite6/assets/urdf/eef_open_parallel.urdf`. This is a **fragment** -- it only contains the gripper links and joints, not the arm. Its first joint must parent `link_eef`. Use the existing `eef_ufgripper.urdf` as a reference.

```xml
<?xml version="1.0" ?>
<robot name="eef_open_parallel">
  <!-- Gripper body (fixed to link_eef) -->
  <joint name="gripper_fix" type="fixed">
    <parent link="link_eef"/>
    <child link="gripper_body"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <link name="gripper_body">
    <inertial>
      <origin xyz="0 0 0.03" rpy="0 0 0"/>
      <mass value="0.30"/>
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0004" iyz="0" izz="0.0003"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="../meshes/gripper/open_parallel/body.stl"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <mesh filename="../meshes/gripper/open_parallel/body.stl"/>
      </geometry>
    </collision>
  </link>

  <!-- Left finger (prismatic joint) -->
  <joint name="finger_joint1" type="prismatic">
    <origin xyz="0 0 0.06" rpy="0 0 0"/>
    <parent link="gripper_body"/>
    <child link="finger_left"/>
    <axis xyz="0 1 0"/>
    <limit effort="20" lower="0" upper="0.0275" velocity="0.1"/>
  </joint>

  <link name="finger_left">
    <inertial>
      <origin xyz="0 0.01 0.025" rpy="0 0 0"/>
      <mass value="0.02"/>
      <inertia ixx="1e-06" ixy="0" ixz="0" iyy="1e-06" iyz="0" izz="1e-06"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="../meshes/gripper/open_parallel/finger_left.stl"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <mesh filename="../meshes/gripper/open_parallel/finger_left.stl"/>
      </geometry>
    </collision>
  </link>

  <!-- Right finger (prismatic, mimic) -->
  <joint name="finger_joint2" type="prismatic">
    <origin xyz="0 0 0.06" rpy="0 0 0"/>
    <parent link="gripper_body"/>
    <child link="finger_right"/>
    <axis xyz="0 -1 0"/>
    <limit effort="20" lower="0" upper="0.0275" velocity="0.1"/>
    <mimic joint="finger_joint1" multiplier="1" offset="0"/>
  </joint>

  <!-- ... finger_right link similar to finger_left ... -->

  <!-- TCP (tool center point) -->
  <joint name="joint_tcp" type="fixed">
    <origin xyz="0 0 0.11" rpy="0 0 0"/>  <!-- MEASURE THIS FROM YOUR CAD -->
    <parent link="gripper_body"/>
    <child link="link_tcp"/>
  </joint>
  <link name="link_tcp"/>
</robot>
```

**EEF fragment contract** -- every EEF file must:
- Be a valid `<robot name="eef_NAME">` XML
- Have its first joint parent set to `link_eef`
- Use mesh paths relative to `../meshes/...`
- Include a `link_tcp` for the tool center point

Key values to customize:
- **`finger_joint1` origin Z** -- height where fingers attach on the body
- **`finger_joint1` limit upper** -- per-finger stroke in meters (0.0275 for 27.5mm)
- **`finger_joint1` limit effort** -- max force in Newtons (derive from servo torque)
- **`joint_tcp` origin Z** -- TCP offset from gripper mounting flange (measure from CAD)
- **All inertial values** -- from your CAD or estimates

## Step 4: Update generate_urdf.py (optional)

If you want the URDF auto-generated, add a function in `scripts/generate_urdf.py` similar to `build_eef_ufgripper()` but with your tool's geometry. Otherwise, hand-edit the URDF fragment directly.

## Step 5: No registration needed

The modular system auto-discovers EEF files by name. Just name your file `eef_open_parallel.urdf` and call:

```python
genesis_lite6.get_urdf_path(eef="open_parallel")
```

This merges `lite6_base.urdf` + `eef_open_parallel.urdf` at runtime and caches the result.

## Step 6: Add config constants

Add these to `genesis_lite6/config.py`:

```python
# OpenParallelGripper SCS3045M
OPEN_GRIPPER_JOINT_NAMES = ("finger_joint1", "finger_joint2")
OPEN_GRIPPER_LINK_NAMES = ("gripper_body", "finger_left", "finger_right")
OPEN_GRIPPER_TCP_LINK = "link_tcp"

OPEN_GRIPPER_JOINT_LOWER = np.array([0.0, 0.0])
OPEN_GRIPPER_JOINT_UPPER = np.array([0.0275, 0.0275])  # 27.5mm per finger
OPEN_GRIPPER_EFFORT_LIMIT = 20.0  # Newtons (estimate from SCS3045M torque)
OPEN_GRIPPER_TOTAL_STROKE = 0.055  # 55mm total opening

OPEN_GRIPPER_KP = np.array([500.0, 500.0])
OPEN_GRIPPER_KV = np.array([50.0, 50.0])

# TCP offset from arm flange (Z distance) -- MEASURE FROM YOUR CAD
OPEN_GRIPPER_TCP_OFFSET_Z = 0.11  # placeholder, update after measurement
```

## Step 7: Test in Genesis

```python
import genesis as gs
import genesis_lite6

gs.init(backend=gs.gpu)
scene = gs.Scene(show_viewer=True)
scene.add_entity(gs.morphs.Plane())

lite6 = scene.add_entity(
    gs.morphs.URDF(
        file=genesis_lite6.get_urdf_path(tool="open_parallel"),
        fixed=True,
    ),
)
scene.build()

# Verify links and joints
print("Links:", [l.name for l in lite6.links])
print("Joints:", [j.name for j in lite6.joints])

for _ in range(1000):
    scene.step()
```

## Checklist

- [ ] Export individual mesh files (body, finger_left, finger_right) from CAD
- [ ] Measure: mass, finger stroke, TCP offset from flange
- [ ] Estimate inertias (from CAD or box approximation)
- [ ] Create `eef_<name>.urdf` fragment with correct joint limits and mesh paths
- [ ] Place meshes in `assets/meshes/gripper/<name>/`
- [ ] Add gripper constants to `config.py`
- [ ] Test in Genesis: `get_urdf_path(eef="<name>")` loads correctly

## Reference: comparing the two grippers

| Property | UFlite (stock) | OpenParallelGripper SCS3045M |
|----------|---------------|------------------------------|
| Total stroke | ~16 mm | 55 mm |
| Per-finger | 8.1 mm (0.0081 m) | 27.5 mm (0.0275 m) |
| Mass | ~0.28 kg | ~0.34 kg |
| Actuation | Pneumatic | SCS3045M servo + rack-pinion |
| Control | Motor force | Position (Modbus-RTU or digital I/O) |
| URDF joints | 2 prismatic (mimic) | 2 prismatic (mimic) |
| Finger type | Built-in narrow/wide | 50mm CNC aluminum claws |
