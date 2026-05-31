"""Sanity tests for the Magni model and shared helpers."""
import math
import sys
from pathlib import Path

import mujoco

# Allow `import magni_sim` when pytest is run from anywhere.
sys.path.insert(0, str(Path(__file__).parent.parent))

import magni_sim  # noqa: E402


def test_model_compiles():
    model = mujoco.MjModel.from_xml_path(magni_sim.SCENE_PATH)
    assert model.nbody > 0


def test_expected_actuators():
    model = mujoco.MjModel.from_xml_path(magni_sim.SCENE_PATH)
    names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)}
    assert names == {"left_wheel", "right_wheel"}


def test_expected_sensors_present():
    model = mujoco.MjModel.from_xml_path(magni_sim.SCENE_PATH)
    names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i) for i in range(model.nsensor)}
    required = {
        "imu_gyro", "imu_acc", "base_pos", "base_quat",
        "left_wheel_pos", "right_wheel_pos",
        "left_wheel_vel", "right_wheel_vel",
        "sonar_0", "sonar_1", "sonar_2", "sonar_3", "sonar_4",
    }
    assert required.issubset(names)


def test_keyframe_is_resting():
    model, data = magni_sim.load()
    assert abs(data.qvel).max() == 0.0
    mujoco.mj_step(model, data)
    assert abs(data.qvel).max() < 0.2  # small transient OK


def test_drives_forward():
    model, data = magni_sim.load()
    x0 = data.qpos[0]
    data.ctrl[:] = magni_sim.cmd_vel_to_wheels(0.3, 0.0)
    for _ in range(500):  # ~1 s at 500 Hz
        mujoco.mj_step(model, data)
    assert data.qpos[0] - x0 > 0.2  # ~0.3 m expected; slip-tolerant lower bound


def test_pivots_in_place():
    model, data = magni_sim.load()
    yaw0 = magni_sim.base_yaw(data.qpos)
    data.ctrl[:] = magni_sim.cmd_vel_to_wheels(0.0, 1.0)
    for _ in range(500):
        mujoco.mj_step(model, data)
    dyaw = magni_sim.base_yaw(data.qpos) - yaw0
    assert dyaw > 0.5  # ~1 rad expected


def test_cmd_vel_pure_forward():
    l, r = magni_sim.cmd_vel_to_wheels(0.3, 0.0)
    assert math.isclose(l, 3.0)
    assert math.isclose(r, 3.0)


def test_cmd_vel_pure_turn():
    l, r = magni_sim.cmd_vel_to_wheels(0.0, 1.0)
    assert l < 0 and r > 0
    assert math.isclose(l, -r)


def test_follow_controller_chases_target():
    """Robot should close on human_c within ~5 s."""
    from controllers import FollowController
    model, data = magni_sim.load()
    mocap_ids = magni_sim.human_ids(model)
    ctrl = FollowController(target_body="human_c")
    initial_dist = None
    for _ in range(2500):  # 5 s
        c = ctrl(model, data)
        assert c is not None  # no duration set -> never returns None
        data.ctrl[:] = c
        magni_sim.update_humans(model, data, mocap_ids)
        mujoco.mj_step(model, data)
        if initial_dist is None:
            tx, ty = data.mocap_pos[mocap_ids["human_c"]][:2]
            initial_dist = math.hypot(tx - data.qpos[0], ty - data.qpos[1])
    tx, ty = data.mocap_pos[mocap_ids["human_c"]][:2]
    final_dist = math.hypot(tx - data.qpos[0], ty - data.qpos[1])
    assert final_dist < initial_dist  # got closer
    assert final_dist < 1.8           # within follow / slow band
