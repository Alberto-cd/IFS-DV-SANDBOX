"""My PLANNING node.   sb-launch planning:=alberto_pipeline/planning

  in : /sandbox/map    ConeArray (map)
       /sandbox/pose   Odometry (map -> base_link)
  out: /sandbox/path   nav_msgs/Path (map), first point = the car

Everything inside plan() is yours.
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sandbox_msgs.msg import Cone, ConeArray

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import Pose2D, body_to_world, cones_to_numpy, world_to_body


def plan(cones_body: np.ndarray, colors: np.ndarray) -> np.ndarray:
    """Cones in the car frame -> path points (K, 2) in the car frame, starting at (0, 0)."""
    return np.array([[0.0, 0.0], [5.0, 0.0]])   # straight ahead. Replace me!


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
