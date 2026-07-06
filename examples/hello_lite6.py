"""Minimal Lite6 example: load the robot and run the simulation."""

import genesis as gs
import genesis_lite6

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

for i in range(1000):
    scene.step()
