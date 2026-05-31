"""Render the demo plan headlessly and save as demo.gif.

Run:
    python record.py
"""
import math
from pathlib import Path

import imageio.v2 as imageio
import mujoco

HERE = Path(__file__).parent
MODEL_PATH = str(HERE / "scene.xml")
OUTPUT_PATH = str(HERE / "demo.gif")

WIDTH, HEIGHT = 320, 240
FPS = 10                # GIF playback fps
SIM_FPS = 5             # how often to capture a frame (sim time)
# 5 capture-fps / 10 playback-fps = 2x speedup

WHEEL_RADIUS = 0.1
TRACK_WIDTH = 0.326


def cmd_vel_to_wheels(v: float, w: float) -> tuple[float, float]:
    left = (v - w * TRACK_WIDTH / 2) / WHEEL_RADIUS
    right = (v + w * TRACK_WIDTH / 2) / WHEEL_RADIUS
    return left, right


def mocap_index(model, body_name: str) -> int:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return model.body_mocapid[body_id]


def update_humans(model, data, mocap_ids: dict[str, int]) -> None:
    t = data.time
    data.mocap_pos[mocap_ids["human_a"]] = [3.0, 2.0 * math.sin(t * 2 * math.pi / 12), 0.9]
    data.mocap_pos[mocap_ids["human_b"]] = [2.5 * math.sin(t * 2 * math.pi / 10), 1.5, 0.9]


def main() -> None:
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)

    mocap_ids = {name: mocap_index(model, name) for name in ("human_a", "human_b")}

    # Same orbit-camera framing as demo.py
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance = 7.5
    camera.azimuth = 114
    camera.elevation = -36
    camera.lookat[:] = [0, 0, 0.3]

    # Square pattern (4x forward 1m + turn 90°)
    forward_speed = 0.3
    turn_rate = 0.8
    plan = []
    for _ in range(4):
        plan.append((forward_speed, 0.0, 1.0 / forward_speed))
        plan.append((0.0, turn_rate, (math.pi / 2) / turn_rate))
    plan.append((0.0, 0.0, 1.0))

    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    frames = []
    frame_dt = 1.0 / SIM_FPS
    next_frame_t = 0.0

    for v, w, duration in plan:
        data.ctrl[:] = cmd_vel_to_wheels(v, w)
        t_end = data.time + duration
        while data.time < t_end:
            update_humans(model, data, mocap_ids)
            mujoco.mj_step(model, data)
            if data.time >= next_frame_t:
                renderer.update_scene(data, camera=camera)
                frames.append(renderer.render())
                next_frame_t += frame_dt

    print(f"Rendered {len(frames)} frames at {WIDTH}x{HEIGHT}, {FPS} fps")
    imageio.mimsave(OUTPUT_PATH, frames, duration=1.0 / FPS, loop=0)
    size_mb = Path(OUTPUT_PATH).stat().st_size / 1e6
    print(f"Wrote {OUTPUT_PATH}  ({size_mb:.2f} MB)")


if __name__ == "__main__":
    main()
