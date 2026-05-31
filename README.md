# magni_robot

A MuJoCo model of the [Ubiquity Robotics Magni](https://www.ubiquityrobotics.com/) differential-drive base.

## Files

- `magni.xml` — robot model: bodies, joints, actuators, contact rules, sensors.
- `scene.xml` — simulation scene: floor, lighting, skybox. Includes `magni.xml`.
- `assets/parts/` — STL meshes (per-color splits used by the visual geoms).
- `assets/*.stl` — original whole-body STLs (kept for reference; not loaded).

## Run

```sh
python -m mujoco.viewer --mjcf scene.xml
```

## Model summary

- 1 free-joint base + 2 hinge wheel joints
- 2 velocity actuators (`left_wheel`, `right_wheel`, `ctrlrange=[-10, 10]` rad/s)
- Collision: cylinder per drive wheel, low-friction sphere per caster, box for chassis
- Sensors: IMU (gyro, accel, framepos, framequat), wheel pos/vel, five HC-SR04 rangefinders

## Frames

- Forward: +x. Left: +y. Up: +z.
- Base ride height: z = 0.1 m.
