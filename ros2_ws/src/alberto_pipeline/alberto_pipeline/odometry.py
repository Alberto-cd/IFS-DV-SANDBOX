"""My ODOMETRY node.   sb-launch odometry:=alberto_pipeline/odometry

  in : /motor_rpm      std_msgs/Float32, motor RPM (speed ≈ rpm * 0.00727 m/s)
       /imu            sensor_msgs/Imu (yaw rate = angular_velocity.z, ~400 Hz in IFSSIM)
       /steering_angle std_msgs/Float32, road-wheel angle (rad), if you want it
  out: /sandbox/odom   nav_msgs/Odometry, frame odom -> base_link  + TF odom -> base_link

Everything inside step() is yours: dead reckoning, a Kalman filter...
The odom frame starts at the start pose (read once from ground truth, like
"the car knows it's on the start line"); on a real-car bag it starts at 0.
"""
import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Header
from tf2_ros import TransformBroadcaster

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import Pose2D, quat_from_yaw, stamp_to_sec


class Odom(Node):
    def __init__(self):
        super().__init__("odometry")
        self.x = self.y = self.yaw = 0.0          # your state
        self.speed = 0.0
        self.started = False
        self.last_t = None
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, contract.ODOM, 10)
        self.create_subscription(Float32, contract.MOTOR_RPM, self.on_rpm, contract.SENSOR_QOS)
        self.create_subscription(Imu, contract.IMU, self.on_imu, contract.SENSOR_QOS)
        self.gt_sub = self.create_subscription(Odometry, contract.GT_ODOM, self.on_start, contract.SENSOR_QOS)

    def on_start(self, msg: Odometry):
        p = Pose2D.from_odometry(msg)
        self.x, self.y, self.yaw, self.started = p.x, p.y, p.yaw, True
        self.destroy_subscription(self.gt_sub)        # only the start pose, never again

    def on_rpm(self, msg: Float32):
        self.speed = msg.data * contract.RPM_TO_MS

    def on_imu(self, msg: Imu):
        t = stamp_to_sec(msg.header.stamp)
        if self.last_t is not None:
            self.step(t - self.last_t, msg)
        self.last_t = t
        self.publish(msg.header.stamp)

    def step(self, dt: float, imu: Imu):
        """Move the state forward by dt seconds. Replace me with something better!"""
        self.yaw += imu.angular_velocity.z * dt
        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt

    def publish(self, stamp):
        header = Header(frame_id=contract.ODOM_FRAME, stamp=stamp)
        q = quat_from_yaw(self.yaw)
        odom = Odometry(header=header, child_frame_id=contract.BASE_FRAME)
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        o = odom.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        odom.twist.twist.linear.x = self.speed      # control uses this as the car speed
        self.pub.publish(odom)
        tf = TransformStamped(header=header, child_frame_id=contract.BASE_FRAME)
        tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = q
        self.tf.sendTransform(tf)


def main():
    run(Odom)


if __name__ == "__main__":
    main()
