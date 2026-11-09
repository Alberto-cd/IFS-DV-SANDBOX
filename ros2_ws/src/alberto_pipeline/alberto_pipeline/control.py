"""My CONTROL node.   sb-launch control:=alberto_pipeline/control

Pure pursuit for steering + speed profile from the path's curvature.
"""
import math

import numpy as np
import rclpy
from fs_msgs.msg import ControlCommand
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import Pose2D, world_to_body

WHEELBASE = 1.627                 # m
MAX_STEER = math.radians(22.4)    # steering = 1.0 means this angle at the wheels
DT = 1.0 / 40.0                   # tick() runs at 40 Hz

# Tuning (start here, then tune in IFSSIM)
V_MAX = 6.0          # m/s on the straights
A_LAT = 3.0          # m/s², max sideways acceleration in a bend (lower = slower corners)
A_BRAKE = 2.5        # m/s², how hard we plan to slow down before a bend
LD_MIN = 2.0         # m, lookahead at standstill
LD_GAIN = 0.35       # s, extra lookahead per m/s
LD_MAX = 6.0         # m
STEER_RATE = 2.5     # max change of the steering command per second (full lock in 0.4 s)
KP, KI = 0.4, 0.15   # speed PI controller

_state = {"steer": 0.0, "i": 0.0}   # remembered between ticks


def resample(path, step=0.25):
    """Same path, points every `step` metres (the planner's points can be far apart)."""
    seg = np.hypot(*np.diff(path, axis=0).T)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    if s[-1] < step:
        return path
    t = np.arange(0.0, s[-1], step)
    return np.column_stack((np.interp(t, s, path[:, 0]), np.interp(t, s, path[:, 1])))


def point_at_distance(path, ld):
    """First path point at least `ld` metres from the car and in front of it."""
    d = np.hypot(path[:, 0], path[:, 1])
    ok = np.flatnonzero((d >= ld) & (path[:, 0] > 0.0))
    return path[ok[0]] if len(ok) else path[-1]


def curvature(path, gap=6):
    """|curvature| (1/m) at each point, from the circle through 3 points `gap` apart."""
    k = np.zeros(len(path))
    for i in range(gap, len(path) - gap):
        a, b, c = path[i - gap], path[i], path[i + gap]
        area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        abc = np.linalg.norm(b - a) * np.linalg.norm(c - b) * np.linalg.norm(c - a)
        k[i] = 2.0 * area2 / max(abc, 1e-6)
    return k


def target_speed(path):
    """Fastest speed now that still lets us slow down in time for every bend ahead."""
    k = curvature(path)
    v_bend = np.minimum(V_MAX, np.sqrt(A_LAT / np.maximum(k, 1e-3)))
    s = np.arange(len(path)) * 0.25                         # distance to each point
    return float(np.min(np.sqrt(v_bend ** 2 + 2.0 * A_BRAKE * s)))


def compute(path_body: np.ndarray, speed: float):
    """Path in the car frame + current speed -> (throttle, steering, brake). steering + = left."""
    if len(path_body) < 2:
        _state["i"] = 0.0
        return 0.0, 0.0, 1.0
    path = resample(path_body)

    # Lateral: pure pursuit, lookahead grows with speed
    ld = min(LD_MIN + LD_GAIN * speed, LD_MAX)
    tx, ty = point_at_distance(path, ld)
    alpha = math.atan2(ty, tx)
    kappa = 2.0 * math.sin(alpha) / max(math.hypot(tx, ty), 0.1)
    steer = float(np.clip(math.atan(WHEELBASE * kappa) / MAX_STEER, -1.0, 1.0))
    # Rate limit: no jumps when the path flickers between two planner updates
    step = STEER_RATE * DT
    steer = _state["steer"] + float(np.clip(steer - _state["steer"], -step, step))
    _state["steer"] = steer

    # Longitudinal: speed profile from the path's curvature + PI on speed
    v_ref = target_speed(path)
    err = v_ref - speed
    _state["i"] = float(np.clip(_state["i"] + err * DT, -2.0, 2.0))
    u = KP * err + KI * _state["i"]
    throttle = float(np.clip(u, 0.0, 1.0))
    brake = float(np.clip(-u, 0.0, 1.0)) if err < -0.3 else 0.0
    return throttle, steer, brake


class Control(Node):
    def __init__(self):
        super().__init__("control")
        self.path = self.pose = None
        self.create_subscription(Path, contract.PATH, lambda m: setattr(self, "path", m), 10)
        self.create_subscription(Odometry, contract.POSE, lambda m: setattr(self, "pose", m), 10)
        self.pub = self.create_publisher(ControlCommand, contract.CONTROL, 10)
        self.create_timer(1.0 / 40.0, self.tick)

    def tick(self):
        if self.path is None or self.pose is None or not self.path.poses:
            self.pub.publish(ControlCommand(brake=1.0))
            return
        pose = Pose2D.from_odometry(self.pose)
        pts = np.array([(p.pose.position.x, p.pose.position.y) for p in self.path.poses])
        throttle, steering, brake = compute(world_to_body(pts, pose), pose.vx)
        # /control_command uses the car's convention, + = RIGHT (like IFSSIM and
        # IFS09-DV-PIPELINE). compute() gives + = left (y is left): flip it.
        self.pub.publish(ControlCommand(throttle=float(throttle), steering=-float(steering), brake=float(brake)))


def main():
    run(Control)


if __name__ == "__main__":
    main()
