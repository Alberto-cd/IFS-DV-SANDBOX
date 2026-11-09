"""My PERCEPTION node.   sb-launch perception:=alberto_pipeline/perception

  in : /lidar_points   PointCloud2 (x, y, z, intensity), frame hesai_lidar. Ground at z = -1.10 on the mock,
       z = 0 on IFSSIM: find its height in the scan (see perception_baseline.py)
  out: /sandbox/cones  ConeArray in base_link, header.stamp = stamp of the scan

Everything inside find_cones() is yours. Start from scratch, or copy ideas from
sandbox_pipeline/perception_baseline.py and change them.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sandbox_msgs.msg import Cone, ConeArray
from sensor_msgs.msg import PointCloud2

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import numpy_to_cones
from sandbox_tools.pointcloud import read_fields


def find_cones(points: np.ndarray):
    """points: (N, 4) x, y, z, intensity.  Return (xy (M, 2), colors (M,))."""
    xy = np.empty((0, 2))
    colors = np.empty(0, dtype=np.uint8)   # Cone.BLUE, Cone.YELLOW, Cone.ORANGE_BIG, Cone.UNKNOWN...
    return xy, colors


class Perception(Node):
    def __init__(self):
        super().__init__("perception")
        self.pub = self.create_publisher(ConeArray, contract.CONES, 10)
        self.create_subscription(PointCloud2, contract.LIDAR, self.on_cloud, contract.SENSOR_QOS)

    def on_cloud(self, msg: PointCloud2):
        xy, colors = find_cones(read_fields(msg, ("x", "y", "z", "intensity")))
        header = msg.header
        header.frame_id = contract.BASE_FRAME
        self.pub.publish(numpy_to_cones(header, xy, colors))


def main():
    run(Perception)


if __name__ == "__main__":
    main()
