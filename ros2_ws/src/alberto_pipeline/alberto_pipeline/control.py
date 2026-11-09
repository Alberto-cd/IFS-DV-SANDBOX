"""My CONTROL node.   sb-launch control:=alberto_pipeline/control

  in : /sandbox/path      nav_msgs/Path (map)
       /sandbox/pose      Odometry (map -> base_link), twist.linear.x = speed
  out: /control_command   fs_msgs/ControlCommand: throttle [0,1], brake [0,1], steering [-1,1] (+ = RIGHT)

Everything inside compute() is yours. Wheelbase 1.627 m, max steer 22.4 deg.
"""
import numpy as np
import rclpy
from fs_msgs.msg import ControlCommand
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import Pose2D, world_to_body


def compute(path_body: np.ndarray, speed: float):
    """Path in the car frame + current speed -> (throttle, steering, brake).

    steering: + = left, like y in the car frame. tick() flips it for /control_command.
    """
    return 0.0, 0.0, 1.0   # stand still. Replace me!


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
