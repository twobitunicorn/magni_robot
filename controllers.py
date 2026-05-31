"""Controllers for the Magni demos. Each is a callable (model, data) -> ctrl_or_None."""
from __future__ import annotations

import math

import magni_sim


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class OpenLoopController:
    """Replay a fixed (v, w, duration) plan. Returns None when the plan is exhausted."""

    def __init__(self, plan: list[tuple[float, float, float]]):
        self.plan = list(plan)
        self.phase_end: float | None = None
        self.v = 0.0
        self.w = 0.0

    def __call__(self, model, data):
        if self.phase_end is None or data.time >= self.phase_end:
            if not self.plan:
                return None
            self.v, self.w, dur = self.plan.pop(0)
            self.phase_end = data.time + dur
        return magni_sim.cmd_vel_to_wheels(self.v, self.w)


class ClosedLoopController:
    """Drive to each (x, y) waypoint by pointing at it (P on heading) and driving forward.

    Always re-aims at the current target, so external disturbances (e.g. a mocap human
    pushing the robot off course) are corrected on the fly.
    """

    KP_YAW = 4.0
    FORWARD_SPEED = 0.3
    TURN_RATE_MAX = 1.5
    DIST_TOL = 0.05
    YAW_BLOCK_RAD = math.radians(30)  # don't drive forward while still mis-aimed > 30°

    def __init__(self, waypoints: list[tuple[float, float]], settle_time: float = 1.0):
        self.waypoints = list(waypoints)
        self.settle_until: float | None = None
        self.settle_time = settle_time

    def __call__(self, model, data):
        if not self.waypoints:
            if self.settle_until is None:
                self.settle_until = data.time + self.settle_time
            if data.time >= self.settle_until:
                return None
            return magni_sim.cmd_vel_to_wheels(0.0, 0.0)

        tx, ty = self.waypoints[0]
        dx = tx - data.qpos[0]
        dy = ty - data.qpos[1]
        if math.hypot(dx, dy) < self.DIST_TOL:
            self.waypoints.pop(0)
            return magni_sim.cmd_vel_to_wheels(0.0, 0.0)

        target_heading = math.atan2(dy, dx)
        yaw_err = _wrap(target_heading - magni_sim.base_yaw(data.qpos))
        w = max(-self.TURN_RATE_MAX, min(self.TURN_RATE_MAX, self.KP_YAW * yaw_err))
        v = self.FORWARD_SPEED if abs(yaw_err) < self.YAW_BLOCK_RAD else 0.0
        return magni_sim.cmd_vel_to_wheels(v, w)
