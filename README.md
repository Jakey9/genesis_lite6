# Genesis Lite6

UFFactory Lite6 robot integration for the [Genesis World](https://github.com/Genesis-Embodied-AI/Genesis) simulator.

## Installation

```bash
pip install -e .
```

Requires `genesis-world` to be installed. See the [Genesis docs](https://genesis-world.readthedocs.io/) for setup instructions.

## Quick start

```python
import genesis as gs
import genesis_lite6

gs.init(backend=gs.gpu)

scene = gs.Scene(show_viewer=True)
scene.add_entity(gs.morphs.Plane())

lite6 = scene.add_entity(
    gs.morphs.URDF(
        file=genesis_lite6.get_urdf_path(eef="ufgripper"),
        fixed=True,
    ),
)

scene.build()

for _ in range(1000):
    scene.step()
```

## Running the examples

All examples assume `genesis-world` and `genesis_lite6` are installed.

### Minimal simulation

Load the Lite6 and step the physics for 1000 frames:

```bash
python examples/hello_lite6.py
```

### PD position control

Move the arm through several joint poses, then cycle the gripper open/close:

```bash
python examples/control_lite6.py
```

### Inverse kinematics

Trace a circular trajectory with the end-effector using the built-in IK solver, then replay it with PD control:

```bash
python examples/ik_lite6.py
```

### GELLO teleoperation

Drive the simulated Lite6 in real time with a Zhonglin GELLO leader arm. Optionally mirror the commands to a real Lite6 as well, using Genesis as a live 3D visualizer.

**Prerequisites**

1. Install `gello_software`:

   ```bash
   pip install -e /path/to/gello_software
   ```

2. Calibrate joint offsets by running the calibration script in `gello_software`:

   ```bash
   python scripts/zhonglin_get_offset.py
   ```

   Then paste the resulting offset values into the constants at the top of `examples/teleop_zhonglin.py` (`ZHONGLIN_JOINT_OFFSETS`, etc.).

3. Plug in the GELLO leader arm (default port: `/dev/ttyUSB0`).

**Sim only** (Genesis visualizer):

```bash
python examples/teleop_zhonglin.py
```

**Sim + real robot** (also commands the physical Lite6 via xArm SDK):

```bash
python examples/teleop_zhonglin.py --real-ip 192.168.1.226
```

**Other options:**

```bash
# Use a different serial port
python examples/teleop_zhonglin.py --port /dev/ttyUSB1

# Disable gripper control
python examples/teleop_zhonglin.py --no-gripper
```

The teleop loop runs at 30 Hz. Press `Ctrl+C` to stop.

## Package structure

```
genesis_lite6/
  __init__.py          # get_urdf_path(eef=...), get_assets_dir()
  config.py            # Joint names, PD gains, force limits, home position
  assets/
    urdf/
      lite6_base.urdf      # 6-DOF arm + link_eef (base, no EEF)
      eef_ufgripper.urdf   # UFGripper fragment (attaches to link_eef)
      .cache/              # Auto-generated combined URDFs (gitignored)
    meshes/
      lite6/visual/        # Arm link meshes (DAE)
      gripper/lite/visual/ # Gripper meshes (DAE)
scripts/
  generate_urdf.py     # Regenerate URDFs from xarm_ros2 source
examples/
  hello_lite6.py       # Minimal load + simulate
  control_lite6.py     # PD position control demo
  ik_lite6.py          # Inverse kinematics demo
  teleop_zhonglin.py   # GELLO teleoperation (sim + optional real)
```

## Modular EEF system

The URDF is split into a **base arm** (`lite6_base.urdf`) and separate **EEF fragments** (`eef_<name>.urdf`). Call `get_urdf_path(eef=None)` for arm-only, or `get_urdf_path(eef="ufgripper")` to merge the arm with the UFactory gripper at runtime. Combined URDFs are cached in `.cache/` and only rebuilt when source files change.

To add a new end-effector, create `eef_<name>.urdf` with its first joint parenting `link_eef`. See `docs/CUSTOM_TOOL_GUIDE.md` for details.

## URDF source

The URDF files are generated from the official [xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) `xarm_description` package using `scripts/generate_urdf.py`. Visual meshes (DAE) are copied from the same source. Collision geometry uses the visual meshes with Genesis's automatic convex decomposition.

To regenerate:

```bash
python scripts/generate_urdf.py --xarm-description /path/to/xarm_ros2/xarm_description
```
