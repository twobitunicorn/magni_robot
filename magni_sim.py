"""Shared helpers for the Magni demos: model loading, IK, sensors, sim loops."""
from __future__ import annotations

import math
import time
from pathlib import Path

import mujoco

HERE = Path(__file__).parent
SCENE_PATH = str(HERE / "scene.xml")

WHEEL_RADIUS = 0.1     # m, drive wheel
TRACK_WIDTH = 0.326    # m, lateral distance between drive wheels (2 * 0.163)

CAMERA_DEFAULTS = dict(distance=7.5, azimuth=114, elevation=-36, lookat=(0, 0, 0.3))


def load() -> tuple[mujoco.MjModel, mujoco.MjData]:
    """Load scene.xml and reset to the 'home' keyframe."""
    model = mujoco.MjModel.from_xml_path(SCENE_PATH)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    return model, data


def cmd_vel_to_wheels(v: float, w: float) -> tuple[float, float]:
    """Differential drive IK: (linear m/s, angular rad/s) -> (left, right) rad/s."""
    left = (v - w * TRACK_WIDTH / 2) / WHEEL_RADIUS
    right = (v + w * TRACK_WIDTH / 2) / WHEEL_RADIUS
    return left, right


def base_yaw(qpos) -> float:
    """Base yaw (radians) extracted from the freejoint quaternion."""
    qw, qz = qpos[3], qpos[6]
    return 2 * math.atan2(qz, qw)


def mocap_index(model: mujoco.MjModel, body_name: str) -> int:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return model.body_mocapid[body_id]


def human_ids(model: mujoco.MjModel) -> dict[str, int]:
    return {name: mocap_index(model, name) for name in ("human_a", "human_b")}


def update_humans(model: mujoco.MjModel, data: mujoco.MjData, mocap_ids: dict[str, int]) -> None:
    """Move the two mocap 'humans' along scripted sinusoidal paths."""
    t = data.time
    data.mocap_pos[mocap_ids["human_a"]] = [3.0, 2.0 * math.sin(t * 2 * math.pi / 12), 0.9]
    data.mocap_pos[mocap_ids["human_b"]] = [2.5 * math.sin(t * 2 * math.pi / 10), 1.5, 0.9]


def apply_camera_defaults(cam) -> None:
    cam.distance = CAMERA_DEFAULTS["distance"]
    cam.azimuth = CAMERA_DEFAULTS["azimuth"]
    cam.elevation = CAMERA_DEFAULTS["elevation"]
    cam.lookat[:] = CAMERA_DEFAULTS["lookat"]


def open_loop_square_plan(forward_speed: float = 0.3, turn_rate: float = 0.8) -> list[tuple[float, float, float]]:
    """A 1m x 1m square as a sequence of (v, w, duration) commands."""
    plan: list[tuple[float, float, float]] = []
    for _ in range(4):
        plan.append((forward_speed, 0.0, 1.0 / forward_speed))
        plan.append((0.0, turn_rate, (math.pi / 2) / turn_rate))
    plan.append((0.0, 0.0, 1.0))
    return plan


def closed_loop_square_waypoints() -> list[tuple[float, float]]:
    """1m x 1m square as goal points (the controller chooses heading)."""
    return [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]


def _print_state(data: mujoco.MjData) -> None:
    print(
        f"{data.time:6.2f} {data.qpos[0]:7.3f} {data.qpos[1]:7.3f} "
        f"{math.degrees(base_yaw(data.qpos)):7.1f} "
        f"{data.sensor('sonar_3').data[0]:7.3f}"
    )


def run_viewer(model, data, controller, *, real_time: bool = True, print_every: float = 0.5) -> None:
    """Step `controller` in the interactive viewer until it returns None or the window closes."""
    import mujoco.viewer

    mocap_ids = human_ids(model)
    last_print_t = -math.inf
    print(f"{'t':>6} {'x':>7} {'y':>7} {'yaw':>7} {'sonar3':>7}")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        apply_camera_defaults(viewer.cam)
        while viewer.is_running():
            wall_start = time.time()
            ctrl = controller(model, data)
            if ctrl is None:
                return
            data.ctrl[:] = ctrl
            update_humans(model, data, mocap_ids)
            mujoco.mj_step(model, data)
            viewer.sync()
            if data.time >= last_print_t + print_every:
                _print_state(data)
                last_print_t = data.time
            if real_time:
                wait = model.opt.timestep - (time.time() - wall_start)
                if wait > 0:
                    time.sleep(wait)


def render_to_gif(model, data, controller, output_path: str, *,
                  width: int = 320, height: int = 240, fps: int = 10, sim_fps: float = 5) -> None:
    """Step `controller` headlessly and save the camera view as a GIF."""
    import imageio.v2 as imageio

    mocap_ids = human_ids(model)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    apply_camera_defaults(camera)

    renderer = mujoco.Renderer(model, height=height, width=width)
    frames: list = []
    frame_dt = 1.0 / sim_fps
    next_frame_t = 0.0

    while True:
        ctrl = controller(model, data)
        if ctrl is None:
            break
        data.ctrl[:] = ctrl
        update_humans(model, data, mocap_ids)
        mujoco.mj_step(model, data)
        if data.time >= next_frame_t:
            renderer.update_scene(data, camera=camera)
            frames.append(renderer.render())
            next_frame_t += frame_dt

    imageio.mimsave(output_path, frames, duration=1.0 / fps, loop=0)
    size_mb = Path(output_path).stat().st_size / 1e6
    print(f"Wrote {output_path}  ({len(frames)} frames, {size_mb:.2f} MB)")
