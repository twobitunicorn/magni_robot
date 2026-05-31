# magni_robot

A MuJoCo model of the [Ubiquity Robotics Magni](https://www.ubiquityrobotics.com/) differential-drive base.

<table>
<tr>
<td><img src="demo.gif" alt="Open-loop"/></td>
<td><img src="demo_closed_loop.gif" alt="Closed-loop"/></td>
<td><img src="demo_follow.gif" alt="Follow"/></td>
</tr>
<tr>
<td align="center"><sub><b>Open-loop</b> — timed <code>(v, w)</code> commands. The walking humans bump the robot off course and it never recovers.</sub></td>
<td align="center"><sub><b>Closed-loop</b> — P controller on heading, re-aiming at each waypoint every step. Bumps still happen, but the robot steers back to the goal.</sub></td>
<td align="center"><sub><b>Follow</b> — chase the green mocap human (<code>human_c</code>) circling the scene. Robot ramps speed down inside <code>SLOW_DIST</code> so it doesn't ram into them.</sub></td>
</tr>
</table>

All GIFs are at 2x sim speed.

## Install

```sh
pip install -r requirements.txt
```

## Run

```sh
# Interactive viewer (macOS uses mjpython for the GUI loop; Linux uses python):
mjpython demo.py                 # open-loop square
mjpython demo_closed_loop.py     # closed-loop square
mjpython demo_follow.py          # chase the circling green human

# Open the scene in the standalone viewer with no controller:
python -m mujoco.viewer --mjcf scene.xml

# Re-render both GIFs after changes to the model or controllers:
python record.py

# Run the model sanity tests:
pytest tests/
```

## Files

- `magni.xml` — robot model: bodies, joints, actuators, contact rules, sensors, keyframe.
- `scene.xml` — simulation scene: floor, lighting, obstacles, mocap humans, named cameras.
- `magni_sim.py` — shared helpers: model loading, IK, sensors, viewer/renderer loops.
- `controllers.py` — `OpenLoopController`, `ClosedLoopController`, `FollowController`.
- `demo.py`, `demo_closed_loop.py`, `demo_follow.py` — interactive viewer entrypoints.
- `record.py` — renders all three demos headlessly to GIFs.
- `tests/test_model.py` — pytest sanity checks (compile, actuators, sensors, basic dynamics).
- `assets/parts/` — STL meshes (per-color splits used by the visual geoms).
- `assets/*.stl` — original whole-body STLs (kept for reference; not loaded).

## Model summary

- 1 free-joint base + 2 hinge drive wheels + 2 swivel-and-roll caster pairs
- 2 velocity actuators (`left_wheel`, `right_wheel`, `ctrlrange=[-10, 10]` rad/s, `armature=0.005`, `frictionloss=0.05`)
- Collision: cylinder per drive wheel, low-friction sphere per caster, box for chassis
- Sensors: IMU (gyro, accel, framepos, framequat), wheel pos/vel, five HC-SR04 rangefinders
- `home` keyframe loads the robot already at rest on the floor

## Frames

- Forward: +x. Left: +y. Up: +z.
- Base ride height: z = 0.1 m.

## License

MIT — see [LICENSE](LICENSE).
