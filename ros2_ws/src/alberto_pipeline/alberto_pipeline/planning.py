"""My PLANNING node.   sb-launch planning:=alberto_pipeline/planning

Centre line between the two track edges, robust to missing colours.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sandbox_msgs.msg import Cone, ConeArray

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import Pose2D, body_to_world, colour_by_side, cones_to_numpy, world_to_body

HORIZON = 20.0       # m, only use cones closer than this
HALF_WIDTH = 1.5     # m, half the track width
MAX_STEP = 6.0       # m, max gap between two centre points
MAX_TURN = math.radians(50.0)
STEP = 0.5           # m, spacing of the published path


def edge_normals(edge):
    """Unit vector pointing LEFT of the driving direction at each cone of an ordered edge."""
    if len(edge) < 2:
        return np.tile((0.0, 1.0), (len(edge), 1))
    t = np.gradient(edge, axis=0)
    t /= np.maximum(np.hypot(t[:, 0], t[:, 1]), 1e-9)[:, None]
    return np.column_stack((-t[:, 1], t[:, 0]))


def order_edge(edge):
    """Sort the cones of one edge in driving order: start nearest the car, walk to the nearest next one."""
    if len(edge) < 2:
        return edge
    left = list(range(len(edge)))
    cur = min(left, key=lambda i: math.hypot(*edge[i]) - 2.0 * (edge[i][0] < 0))   # behind first
    out = [cur]
    left.remove(cur)
    while left:
        nxt = min(left, key=lambda i: np.hypot(*(edge[i] - edge[cur])))
        if np.hypot(*(edge[nxt] - edge[cur])) > 8.0:
            break
        out.append(nxt)
        left.remove(nxt)
        cur = nxt
    return edge[out]


def centre_points(blue, yellow):
    """Candidate points on the centre line, from both edges."""
    pts = []
    for a, b in ((blue, yellow), (yellow, blue)):          # pair from both sides -> denser line
        for c in a:
            if len(b) == 0:
                break
            d = np.hypot(*(b - c).T)
            j = int(np.argmin(d))
            if 2.0 < d[j] < 6.0:                            # plausible track width
                pts.append((c + b[j]) / 2.0)
    if len(pts) >= 3:
        return np.array(pts)
    # One edge only (or a broken pairing): step half a track width inwards,
    # perpendicular to the edge (not just sideways in y: that's wrong in a bend).
    if len(blue) >= len(yellow) and len(blue):
        return blue - HALF_WIDTH * edge_normals(blue)
    if len(yellow):
        return yellow + HALF_WIDTH * edge_normals(yellow)
    return np.empty((0, 2))


def chain(mids):
    """From the car, keep jumping to the closest centre point ahead that doesn't turn too sharply."""
    path, heading, rest = [np.zeros(2)], 0.0, list(mids)
    while rest:
        cur = path[-1]
        best, best_d = None, MAX_STEP
        for k, m in enumerate(rest):
            v = m - cur
            d = math.hypot(*v)
            turn = abs((math.atan2(v[1], v[0]) - heading + math.pi) % (2 * math.pi) - math.pi)
            if 0.5 < d < best_d and turn < MAX_TURN:
                best, best_d = k, d
        if best is None:
            break
        nxt = rest.pop(best)
        heading = math.atan2(nxt[1] - cur[1], nxt[0] - cur[0])
        path.append(nxt)
        # drop the leftovers right next to the point we just used (pairs from both sides)
        rest = [m for m in rest if math.hypot(*(m - nxt)) > 1.0]
    return np.array(path)


def smooth(path, window=5):
    """Resample every STEP metres and average neighbours. The first point stays at the car."""
    if len(path) < 3:
        return path
    seg = np.hypot(*np.diff(path, axis=0).T)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    t = np.arange(0.0, s[-1], STEP)
    p = np.column_stack((np.interp(t, s, path[:, 0]), np.interp(t, s, path[:, 1])))
    if len(p) <= window:
        return p
    k = np.ones(window) / window
    pad = np.vstack([p[:1]] * (window // 2) + [p] + [p[-1:]] * (window // 2))
    out = np.column_stack([np.convolve(pad[:, i], k, mode="valid") for i in (0, 1)])
    out[0] = (0.0, 0.0)
    return out


def plan(cones_body: np.ndarray, colors: np.ndarray) -> np.ndarray:
    """Cones in the car frame -> path points (K, 2) in the car frame, starting at (0, 0)."""
    straight = np.array([[0.0, 0.0], [3.0, 0.0]])
    if len(cones_body) == 0:
        return straight
    near = (np.hypot(cones_body[:, 0], cones_body[:, 1]) < HORIZON) & (cones_body[:, 0] > -3.0)
    xy, side = cones_body[near], colors[near].copy()
    if len(xy) < 2:
        return straight
    # Big orange gate cones sit on the edges too: left of the car = left edge.
    gate = side == Cone.ORANGE_BIG
    side[gate] = np.where(xy[gate, 1] > 0.0, Cone.BLUE, Cone.YELLOW)
    # No colour (UNKNOWN, small orange)? Guess the edge from geometry.
    unsure = (side != Cone.BLUE) & (side != Cone.YELLOW)
    if unsure.any():
        side[unsure] = colour_by_side(xy)[unsure]
    blue = order_edge(xy[side == Cone.BLUE])
    yellow = order_edge(xy[side == Cone.YELLOW])
    path = chain(centre_points(blue, yellow))
    return smooth(path) if len(path) >= 2 else straight


class Planning(Node):
    def __init__(self):
        super().__init__("planning")
        self.map = self.pose = None
        self.create_subscription(ConeArray, contract.MAP, lambda m: setattr(self, "map", m), 10)
        self.create_subscription(Odometry, contract.POSE, lambda m: setattr(self, "pose", m), 10)
        self.pub = self.create_publisher(Path, contract.PATH, 10)
        self.create_timer(0.1, self.tick)

    def tick(self):
        if self.map is None or self.pose is None:
            return
        pose = Pose2D.from_odometry(self.pose)
        xy, colors, _ = cones_to_numpy(self.map)
        path_world = body_to_world(plan(world_to_body(xy, pose), colors), pose)
        msg = Path()
        msg.header.frame_id = contract.MAP_FRAME
        msg.header.stamp = self.pose.header.stamp
        for x, y in path_world:
            ps = PoseStamped(header=msg.header)
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub.publish(msg)


def main():
    run(Planning)


if __name__ == "__main__":
    main()
