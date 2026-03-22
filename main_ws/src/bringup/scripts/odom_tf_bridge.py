#!/usr/bin/env python3
"""odom_tf_bridge: /odom → odom→base_footprint TF broadcaster.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfBridge(Node):
    def __init__(self) -> None:
        super().__init__("odom_tf_bridge")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_stamp_ns = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_ = self.create_subscription(Odometry, "/odom", self.odom_cb, qos)
        self.get_logger().info("odom_tf_bridge started: /odom → TF odom→base_footprint")
        self.received_first = False

    def odom_cb(self, msg: Odometry) -> None:
        if not self.received_first:
            self.get_logger().info("Received first odom message!")
            self.received_first = True

        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns <= self.last_stamp_ns:
            return
        self.last_stamp_ns = stamp_ns

        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id or "base_footprint"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
