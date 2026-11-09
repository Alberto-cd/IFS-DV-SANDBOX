"""My SLAM node.   sb-launch slam:=alberto_pipeline/slam

  in : /sandbox/odom    nav_msgs/Odometry, odom -> base_link (smooth, drifts)
       /sandbox/cones   ConeArray (base_link) from perception
       (/testing_only/odom is ground truth: fine for debugging, but the real car doesn't have it)
  out: /sandbox/pose    nav_msgs/Odometry, frame map, child base_link
       TF map -> odom   your correction of the odometry drift
       /sandbox/map     ConeArray (map)

    pose (map -> base_link) = correction (map -> odom) * odometry (odom -> base_link)

publish_pose() and publish_map() do the ROS part for you. compose()/inverse()
are in sandbox_pipeline/slam_baseline.py if you need them.
"""
import math

import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sandbox_msgs.msg import ConeArray
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import Pose2D, cones_to_numpy, numpy_to_cones, quat_from_yaw


class Slam(Node):
    def __init__(self):
        super().__init__("slam")
        self.correction = Pose2D()                # map -> odom, start with "no correction"
        self.odom = Pose2D()
        self.tf = TransformBroadcaster(self)
        self.pub_pose = self.create_publisher(Odometry, contract.POSE, 10)
        self.pub_map = self.create_publisher(ConeArray, contract.MAP, 10)
        self.create_subscription(Odometry, contract.ODOM, self.on_odom, 10)
        self.create_subscription(ConeArray, contract.CONES, self.on_cones, 10)

    def on_odom(self, msg: Odometry):
        self.odom = Pose2D.from_odometry(msg)
        self.publish_pose(msg.header.stamp)

    def on_cones(self, msg: ConeArray):
        xy, colors, _ = cones_to_numpy(msg)
        # e.g. associate with the map, update self.correction, add new cones...
        self.publish_map(msg.header.stamp, np.empty((0, 2)), [])

    def pose(self) -> Pose2D:
        """correction * odometry = the pose in the map frame."""
        c, s = math.cos(self.correction.yaw), math.sin(self.correction.yaw)
        o = self.odom
        return Pose2D(self.correction.x + c * o.x - s * o.y, self.correction.y + s * o.x + c * o.y,
                      self.correction.yaw + o.yaw, vx=o.vx)

    def publish_pose(self, stamp):
        p = self.pose()
        header = Header(frame_id=contract.MAP_FRAME, stamp=stamp)
        out = Odometry(header=header, child_frame_id=contract.BASE_FRAME)
        out.pose.pose.position.x, out.pose.pose.position.y = p.x, p.y
        o = out.pose.pose.orientation
        o.x, o.y, o.z, o.w = quat_from_yaw(p.yaw)
        out.twist.twist.linear.x = p.vx            # control uses this as the car speed
        self.pub_pose.publish(out)
        tf = TransformStamped(header=header, child_frame_id=contract.ODOM_FRAME)
        tf.transform.translation.x, tf.transform.translation.y = self.correction.x, self.correction.y
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = quat_from_yaw(self.correction.yaw)
        self.tf.sendTransform(tf)

    def publish_map(self, stamp, xy, colors):
        self.pub_map.publish(numpy_to_cones(Header(frame_id=contract.MAP_FRAME, stamp=stamp), xy, colors))


def main():
    run(Slam)


if __name__ == "__main__":
    main()
