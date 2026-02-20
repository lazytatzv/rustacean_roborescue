#!/usr/bin/env python3

from typing import Iterable
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class FixPointCloudTimeNode(Node):
    def __init__(self) -> None:
        super().__init__("fix_pointcloud_time")

        self.declare_parameter("input_topic", "/velodyne_points")
        self.declare_parameter("output_topic", "/velodyne_points_fixed")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.on_cloud, 10)

        self.get_logger().info(
            f"Fixing point time field: {self.input_topic} -> {self.output_topic}"
        )

    def on_cloud(self, msg: PointCloud2) -> None:
        fields = [f.name for f in msg.fields]
        if "time" not in fields:
            # Nothing to fix, just pass through
            self.pub.publish(msg)
            return

        try:
            points = list(point_cloud2.read_points(msg, field_names=None, skip_nans=False))
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f"Failed to read point cloud: {exc}")
            return

        # VLP16: 900 RPM = 15 Hz = 66.7ms per rotation
        # Calculate time offset based on azimuth angle (arctan2(y, x))
        import math
        SCAN_DURATION_MS = 66.7
        
        time_idx = fields.index("time")
        x_idx = fields.index("x")
        y_idx = fields.index("y")
        
        fixed_points = []
        for p in points:
            p_list = list(p)
            
            # Calculate azimuth from x, y coordinates
            azimuth_rad = math.atan2(p_list[y_idx], p_list[x_idx])
            azimuth_deg = azimuth_rad * 180.0 / math.pi
            if azimuth_deg < 0:
                azimuth_deg += 360.0
            
            # Time offset: azimuth/360 * scan_duration
            time_offset_ms = (azimuth_deg / 360.0) * SCAN_DURATION_MS
            p_list[time_idx] = time_offset_ms
            
            fixed_points.append(p_list)

        out_msg = point_cloud2.create_cloud(msg.header, msg.fields, fixed_points)
        self.pub.publish(out_msg)


def main() -> None:
    rclpy.init()
    node = FixPointCloudTimeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
