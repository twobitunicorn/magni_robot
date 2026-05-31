"""Drive the Magni in a 1m x 1m square in the MuJoCo viewer.

Run:
    python demo.py
"""
import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer

MODEL_PATH = str(Path(__file__).parent / "scene.xml")

WHEEL_RADIUS = 0.1     # m, drive wheel
TRACK_WIDTH  = 0.326   # m, lateral distance between drive wheels (2 * 0.163)


def cmd_vel_to_wheels(v: float, w: float) -> tuple[float, float]:
    """Differential drive inverse kinematics.

    Args:
        v: forward velocity (m/s).
        w: yaw rate (rad/s, +z = CCW from above).

    Returns:
        (left, right) wheel angular velocities in rad/s, in the order the
        actuators expect (ctrl[0] = left, ctrl[1] = right).
    """
    left  = (v - w * TRACK_WIDTH / 2) / WHEEL_RADIUS
    right = (v + w * TRACK_WIDTH / 2) / WHEEL_RADIUS
    return left, right


def base_yaw_deg(qpos) -> float:
    qw, qz = qpos[3], qpos[6]
    return math.degrees(2 * math.atan2(qz, qw))


def mocap_index(model, body_name: str) -> int:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return model.body_mocapid[body_id]


def update_humans(model, data, mocap_ids: dict[str, int]) -> None:
    """Move the mocap "humans" along scripted back-and-forth paths."""
    t = data.time
    # human_a paces north/south at x=3, period 12 s, amplitude 2 m
    data.mocap_pos[mocap_ids["human_a"]] = [3.0, 2.0 * math.sin(t * 2 * math.pi / 12), 0.9]
    # human_b paces east/west at y=1.5, period 10 s, amplitude 2.5 m
    data.mocap_pos[mocap_ids["human_b"]] = [2.5 * math.sin(t * 2 * math.pi / 10), 1.5, 0.9]


def main() -> None:
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)

    mocap_ids = {name: mocap_index(model, name) for name in ("human_a", "human_b")}

    # Rough 1m x 1m square: drive forward 1 m, turn 90 deg, repeat 4x.
    forward_speed = 0.3                  # m/s
    turn_rate     = 0.8                  # rad/s
    plan = []
    for _ in range(4):
        plan.append((forward_speed, 0.0, 1.0 / forward_speed))
        plan.append((0.0, turn_rate, (math.pi / 2) / turn_rate))
    plan.append((0.0, 0.0, 2.0))         # stop and settle

    print(f"{'t':>6} {'x':>7} {'y':>7} {'yaw':>7} {'sonar3':>7}")
    next_print = 0.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 7.5
        viewer.cam.azimuth = 114
        viewer.cam.elevation = -36  # camera ~4.7 m above floor (7.5 * sin(36°) + 0.3)
        viewer.cam.lookat[:] = [0, 0, 0.3]

        for v, w, duration in plan:
            data.ctrl[:] = cmd_vel_to_wheels(v, w)
            t_end = data.time + duration

            while data.time < t_end and viewer.is_running():
                wall_start = time.time()
                update_humans(model, data, mocap_ids)
                mujoco.mj_step(model, data)
                viewer.sync()

                if data.time >= next_print:
                    print(
                        f"{data.time:6.2f} "
                        f"{data.qpos[0]:7.3f} {data.qpos[1]:7.3f} "
                        f"{base_yaw_deg(data.qpos):7.1f} "
                        f"{data.sensor('sonar_3').data[0]:7.3f}"
                    )
                    next_print = data.time + 0.5

                wait = model.opt.timestep - (time.time() - wall_start)
                if wait > 0:
                    time.sleep(wait)

            if not viewer.is_running():
                return


if __name__ == "__main__":
    main()
