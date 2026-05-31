"""Closed-loop 1m x 1m square in the MuJoCo viewer.

Drives to each corner by pointing at it (P controller on heading), so external
pushes (e.g. a mocap human) are corrected on the fly.

Run:
    mjpython demo_closed_loop.py        # macOS
    python  demo_closed_loop.py         # Linux
"""
import magni_sim
from controllers import ClosedLoopController


def main() -> None:
    model, data = magni_sim.load()
    controller = ClosedLoopController(magni_sim.closed_loop_square_waypoints())
    magni_sim.run_viewer(model, data, controller)


if __name__ == "__main__":
    main()
