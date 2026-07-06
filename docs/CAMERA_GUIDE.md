# Camera Setup Guide for Genesis Lite6

This guide covers how to add cameras to the Lite6 Genesis simulation -- both **eye-in-hand** (mounted on the robot flange) and **external** (fixed in the workspace). Includes RealSense D455 specs and hand-eye calibration workflow.

## RealSense D455 Specifications

| Parameter | Depth stream | RGB stream |
|-----------|-------------|------------|
| Resolution | 1280 x 720 | 1280 x 800 |
| Frame rate | Up to 90 fps | Up to 30 fps |
| FOV (H x V) | 87 x 58 deg | 90 x 65 deg |
| Ideal range | 0.6 m to 6 m | -- |
| Min depth (at max res) | ~0.52 m | -- |
| Depth accuracy | < 2% at 4 m | -- |
| Shutter | Global | Global |
| IMU | Yes (BMI055, 6-DOF) | -- |
| Dimensions | 124 x 29 x 26 mm | -- |
| Interface | USB 3.1 | -- |

Genesis camera parameters to match the D455:

| Camera type | `fov` | `res` |
|-------------|-------|-------|
| Depth-like | 58 | (1280, 720) |
| RGB-like | 65 | (1280, 800) |

## External Camera (Fixed in Workspace)

Use this for a tripod or bracket-mounted RealSense overlooking the workspace.

```python
import genesis as gs
import genesis_lite6

gs.init(backend=gs.gpu)

scene = gs.Scene(
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

# External camera matching D455 depth stream
external_cam = scene.add_camera(
    res=(1280, 720),
    pos=(0.5, -0.4, 0.6),       # position in world frame (adjust to your setup)
    lookat=(0.0, 0.0, 0.25),    # looking at the workspace center
    fov=58,                      # D455 depth vertical FOV
    GUI=True,                    # live preview window
)

scene.build()

for i in range(1000):
    scene.step()

    # Render every N steps (rendering is expensive)
    if i % 5 == 0:
        external_cam.render(rgb=True, depth=True)
        rgb = external_cam.rgb      # torch tensor (H, W, 3)
        depth = external_cam.depth  # torch tensor (H, W)
```

You can reposition the camera dynamically with `external_cam.set_pose(pos=(...), lookat=(...))`.

## Eye-in-Hand Camera (Mounted on Flange)

Use this for a RealSense attached to `link6` (the arm flange). The camera moves with the robot.

### Step 1: Hand-eye calibration

Run your hand-eye calibration procedure (e.g. using OpenCV `cv2.calibrateHandEye`) to obtain the 4x4 transform from the flange frame to the camera optical frame. Store it in `genesis_lite6/config.py`:

```python
import numpy as np

# T_flange_camera: 4x4 homogeneous transform from link6 to camera optical frame.
# Obtained from hand-eye calibration. Update after recalibrating.
CAMERA_T_FLANGE = np.array([
    [ 0.0,  0.0, 1.0, 0.03 ],   # camera Z = flange X (pointing forward)
    [-1.0,  0.0, 0.0, 0.00 ],   # camera X = -flange Y
    [ 0.0, -1.0, 0.0, 0.05 ],   # camera Y = -flange Z
    [ 0.0,  0.0, 0.0, 1.00 ],
])
```

The rotation part depends on how your camera is physically mounted. The translation `[0.03, 0.0, 0.05]` is the offset from the flange center to the camera optical center in the flange frame.

### Step 2: Attach in Genesis

```python
import numpy as np
import genesis as gs
import genesis_lite6
from genesis_lite6.config import HOME_QPOS

# Your calibrated transform (or import from config.py)
CAMERA_T_FLANGE = np.array([
    [ 0.0,  0.0, 1.0, 0.03 ],
    [-1.0,  0.0, 0.0, 0.00 ],
    [ 0.0, -1.0, 0.0, 0.05 ],
    [ 0.0,  0.0, 0.0, 1.00 ],
])

gs.init(backend=gs.gpu)

scene = gs.Scene(
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

# Eye-in-hand camera matching D455 depth stream
wrist_cam = scene.add_camera(
    res=(1280, 720),
    pos=(0.0, 0.0, 0.05),       # initial pos (overridden by attach)
    lookat=(0.0, 0.0, 0.15),    # initial lookat (overridden by attach)
    fov=58,                      # D455 depth vertical FOV
    GUI=True,
)

scene.build()

# Attach camera to flange link with calibrated offset
ee_link = lite6.get_link("link6")
wrist_cam.attach(ee_link, CAMERA_T_FLANGE)

# Set robot to home
lite6.set_qpos(HOME_QPOS)

for i in range(1000):
    scene.step()
    wrist_cam.move_to_attach()   # update camera pose to follow the link

    if i % 5 == 0:
        wrist_cam.render(rgb=True, depth=True)
        rgb = wrist_cam.rgb
        depth = wrist_cam.depth
```

**Key points:**
- Call `wrist_cam.move_to_attach()` every step (or every render step) so the camera follows the arm.
- The `pos` and `lookat` passed to `add_camera` are only used initially; once `attach()` is called, the `offset_T` matrix defines the pose relative to the link.

## Using Both Cameras Together

```python
# Add both before scene.build()
external_cam = scene.add_camera(
    res=(1280, 720),
    pos=(0.5, -0.4, 0.6),
    lookat=(0.0, 0.0, 0.25),
    fov=58,
    GUI=True,
)

wrist_cam = scene.add_camera(
    res=(1280, 720),
    pos=(0.0, 0.0, 0.05),
    lookat=(0.0, 0.0, 0.15),
    fov=58,
    GUI=True,
)

scene.build()

ee_link = lite6.get_link("link6")
wrist_cam.attach(ee_link, CAMERA_T_FLANGE)

for i in range(1000):
    scene.step()
    wrist_cam.move_to_attach()

    if i % 5 == 0:
        external_cam.render(rgb=True, depth=True)
        wrist_cam.render(rgb=True, depth=True)

        # Both cameras produce (H, W, 3) RGB and (H, W) depth tensors
        ext_rgb = external_cam.rgb
        ext_depth = external_cam.depth
        wrist_rgb = wrist_cam.rgb
        wrist_depth = wrist_cam.depth
```

## Why Python `attach()` Instead of URDF Camera Links

You might consider adding a `camera_link` to the URDF with a fixed joint. This works but has a practical issue: Genesis merges fixed-joint links by default (`merge_fixed_links=True`), which would make `camera_link` invisible in the kinematic tree. You'd need to either disable merging (which affects performance) or give the link a dummy inertia.

Using `cam.attach(link, offset_T)` in Python avoids this entirely and keeps the calibration data easy to update. Store the calibration matrix in `config.py` as a single source of truth.

## Calibration Workflow Summary

1. **Mount** the RealSense D455 on the Lite6 flange (or on a bracket for external).
2. **Calibrate** using OpenCV `cv2.calibrateHandEye()` with a checkerboard to get the 4x4 `T_flange_camera`.
3. **Store** the transform in `genesis_lite6/config.py` as `CAMERA_T_FLANGE`.
4. **Attach** in Genesis with `cam.attach(link, CAMERA_T_FLANGE)`.
5. **Validate** by comparing rendered images from Genesis with real RealSense images at the same joint configuration.

For the external camera, measure its world pose (position + orientation) and pass it directly as `pos` and `lookat` to `scene.add_camera()`.

## Saving Images

```python
import matplotlib.pyplot as plt
from genesis.utils.misc import tensor_to_array

# After rendering
rgb_np = tensor_to_array(wrist_cam.rgb)       # numpy (H, W, 3) uint8
depth_np = tensor_to_array(wrist_cam.depth)    # numpy (H, W) float32

plt.imsave("rgb.png", rgb_np)
plt.imsave("depth.png", depth_np, cmap="viridis")
```

## Sensor API (Alternative)

For batched RL environments, use the structured sensor API instead of `add_camera`:

```python
from genesis.options.sensors import RasterizerCameraOptions

cam_sensor = scene.add_sensor(RasterizerCameraOptions(
    res=(640, 480),
    pos=(0.5, -0.4, 0.6),
    lookat=(0.0, 0.0, 0.25),
    fov=58,
    near=0.1,
    far=6.0,
    entity_idx=lite6.idx,       # attach to robot entity
    link_idx_local=6,           # link6 (flange)
))

scene.build(n_envs=16)

# Reading sensor data
data = cam_sensor.read()
rgb_batch = data.rgb            # (n_envs, H, W, 3)
depth_batch = data.depth        # (n_envs, H, W)
```

This is preferred when you need camera observations as part of an RL observation space (see `genesis_rl` package).
