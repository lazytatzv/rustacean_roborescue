#!/usr/bin/env python3
"""CrawlerVelocity → Twist 変換ブリッジ.

シミュレーション時に joy_controller の CrawlerVelocity (m1_vel, m2_vel) を
Gazebo diff_drive プラグインが受け取る Twist (cmd_vel) に変換する。

  linear.x  = (m1 + m2) / 2
  angular.z = (m2 - m1) / track_width
"""
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import CrawlerVelocity
from geometry_msgs.msg import Twist


class CrawlerVelBridge(Node):
    def __init__(self) -> None:
        super().__init__("crawler_vel_bridge")
        self.declare_parameter("track_width", 0.4)
        self.track_width_ = self.get_parameter("track_width").value

        self.sub_ = self.create_subscription(
            CrawlerVelocity, "/crawler_driver", self.callback, 10
        )
        self.pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info(
            f"CrawlerVelocity → Twist bridge (track_width={self.track_width_})"
        )

    def callback(self, msg: CrawlerVelocity) -> None:
        twist = Twist()
        twist.linear.x = (msg.m1_vel + msg.m2_vel) / 2.0
        twist.angular.z = (msg.m2_vel - msg.m1_vel) / self.track_width_
        self.pub_.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CrawlerVelBridge()
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
