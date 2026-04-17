#!/usr/bin/env python3
"""点群トピックを1回取得してPCDファイルに保存する"""
import argparse
import struct
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class PointCloudSaver(Node):
    def __init__(self, topic: str, output: str, timeout: float):
        super().__init__("pointcloud_saver")
        self.output = output
        self.saved = False

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(PointCloud2, topic, self.callback, qos)
        self.timer = self.create_timer(timeout, self.on_timeout)
        self.get_logger().info(f"Waiting for pointcloud on {topic}...")

    def callback(self, msg: PointCloud2):
        if self.saved:
            return
        self.saved = True

        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        self.save_pcd(points)
        self.get_logger().info(f"Saved {len(points)} points to {self.output}")
        rclpy.shutdown()

    def save_pcd(self, points):
        """ASCII PCD形式で保存"""
        with open(self.output, "w") as f:
            f.write("# .PCD v0.7 - Point Cloud Data\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {len(points)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(points)}\n")
            f.write("DATA ascii\n")
            for p in points:
                f.write(f"{p[0]} {p[1]} {p[2]}\n")

    def on_timeout(self):
        if not self.saved:
            self.get_logger().error("Timeout: no pointcloud received")
            rclpy.shutdown()
            sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Save pointcloud to PCD file")
    parser.add_argument("--topic", default="/kiss/local_map", help="Pointcloud topic")
    parser.add_argument("--output", required=True, help="Output PCD file path")
    parser.add_argument("--timeout", type=float, default=10.0, help="Timeout in seconds")
    args = parser.parse_args()

    rclpy.init()
    node = PointCloudSaver(args.topic, args.output, args.timeout)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
