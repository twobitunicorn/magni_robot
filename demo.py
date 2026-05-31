"""Open-loop 1m x 1m square in the MuJoCo viewer.

Run:
    mjpython demo.py        # macOS
    python  demo.py         # Linux
"""
import magni_sim
from controllers import OpenLoopController


def main() -> None:
    model, data = magni_sim.load()
    controller = OpenLoopController(magni_sim.open_loop_square_plan())
    magni_sim.run_viewer(model, data, controller)


if __name__ == "__main__":
    main()
