#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time


class JointDemo(Node):
    def __init__(self):
        super().__init__("joint_demo")
        self.publisher_ = self.create_publisher(JointState, "joint_states", 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.start_time = time.time()

        self.joint_names = [
            "flipper_fr_joint",
            "flipper_fl_joint",
            "flipper_br_joint",
            "flipper_bl_joint",
            "arm_joint1",
            "arm_joint2",
            "arm_joint3",
            "arm_joint4",
            "arm_joint5",
            "arm_joint6",
        ]

    def timer_callback(self):
        t = time.time() - self.start_time
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # サイン波で各関節を動かす
        flipper_pos = 0.5 * math.sin(t)
        arm_pos = 0.8 * math.sin(t * 0.5)

        msg.position = [
            flipper_pos,
            flipper_pos,
            -flipper_pos,
            -flipper_pos,  # フリッパー
            arm_pos,
            -0.5 + 0.5 * math.sin(t),
            0.5 + 0.5 * math.cos(t),
            arm_pos,
            arm_pos,
            arm_pos,  # アーム
        ]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
