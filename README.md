# magni_robot

A MuJoCo model of the [Ubiquity Robotics Magni](https://www.ubiquityrobotics.com/) differential-drive base.

![Demo](demo.gif)

*Open-loop square pattern with two scripted mocap "humans" and a few obstacles. The humans bump the robot, which is why the open-loop plan drifts off course. Playback is 2x sim time.*

## Files

- `magni.xml` — robot model: bodies, joints, actuators, contact rules, sensors.
- `scene.xml` — simulation scene: floor, lighting, skybox, obstacles, mocap humans. Includes `magni.xml`.
- `demo.py` — drives the robot in a square via a `cmd_vel`-style helper and paces the humans.
- `record.py` — renders `demo.py`'s plan headlessly to `demo.gif`.
- `assets/parts/` — STL meshes (per-color splits used by the visual geoms).
- `assets/*.stl` — original whole-body STLs (kept for reference; not loaded).

## Run

```sh
# Interactive viewer with the demo plan (macOS uses mjpython for the GUI loop):
mjpython demo.py

# Or just open the scene in the standalone viewer:
python -m mujoco.viewer --mjcf scene.xml

# Re-record the GIF after changes:
python record.py
```

## Model summary

- 1 free-joint base + 2 hinge wheel joints
- 2 velocity actuators (`left_wheel`, `right_wheel`, `ctrlrange=[-10, 10]` rad/s)
- Collision: cylinder per drive wheel, low-friction sphere per caster, box for chassis
- Sensors: IMU (gyro, accel, framepos, framequat), wheel pos/vel, five HC-SR04 rangefinders

## Frames

- Forward: +x. Left: +y. Up: +z.
- Base ride height: z = 0.1 m.
