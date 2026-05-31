"""Render all three demos headlessly to GIFs.

Run:
    python record.py
"""
import magni_sim
from controllers import OpenLoopController, ClosedLoopController, FollowController


def main() -> None:
    for output_path, make_controller in [
        ("demo.gif",             lambda: OpenLoopController(magni_sim.open_loop_square_plan())),
        ("demo_closed_loop.gif", lambda: ClosedLoopController(magni_sim.closed_loop_square_waypoints())),
        ("demo_follow.gif",      lambda: FollowController(target_body="human_c", duration=25.0)),
    ]:
        model, data = magni_sim.load()
        magni_sim.render_to_gif(model, data, make_controller(), output_path)


if __name__ == "__main__":
    main()
