"""Follow `human_c` (the circling mocap human) in the MuJoCo viewer.

Runs until the viewer window is closed. The controller maintains a buffer
distance so the robot doesn't ram into the target.

Run:
    mjpython demo_follow.py        # macOS
    python  demo_follow.py         # Linux
"""
import magni_sim
from controllers import FollowController


def main() -> None:
    model, data = magni_sim.load(magni_sim.SCENE_FOLLOW_PATH)
    controller = FollowController(target_body="human_c")  # no duration -> runs forever
    magni_sim.run_viewer(model, data, controller)


if __name__ == "__main__":
    main()
