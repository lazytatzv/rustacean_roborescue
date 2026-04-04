#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

# VLP-16: 900 RPM = 15 Hz = 66.7 ms/rotation
_SCAN_DURATION_MS = 66.7

_FIELD_TYPE_TO_NUMPY = {
    1: np.int8,
    2: np.uint8,
    3: np.int16,
    4: np.uint16,
    5: np.int32,
    6: np.uint32,
    7: np.float32,
    8: np.float64,
}


def _fields_dtype(fields, point_step: int) -> np.dtype:
    names, formats, offsets = [], [], []
    for f in sorted(fields, key=lambda x: x.offset):
        names.append(f.name)
        formats.append(_FIELD_TYPE_TO_NUMPY[f.datatype])
        offsets.append(f.offset)
    return np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": point_step})


class FixPointCloudTimeNode(Node):
    def __init__(self) -> None:
        super().__init__("fix_pointcloud_time")

        self.declare_parameter("input_topic", "/velodyne_points")
        self.declare_parameter("output_topic", "/velodyne_points_fixed")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        # PointCloud2 は大きいので depth を絞る
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 3)
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.on_cloud, 3)

        self.get_logger().info(
            f"Fixing point time field: {self.input_topic} -> {self.output_topic}"
        )

    def on_cloud(self, msg: PointCloud2) -> None:
        field_names = [f.name for f in msg.fields]
        if "time" not in field_names:
            self.pub.publish(msg)
            return

        try:
            dtype = _fields_dtype(msg.fields, msg.point_step)
            # frombuffer は読み取り専用なので copy() で書き込み可能にする
            arr = np.frombuffer(bytes(msg.data), dtype=dtype).copy()

            # アジマス角を一括ベクトル演算
            azimuth_deg = np.degrees(np.arctan2(arr["y"].astype(np.float64), arr["x"].astype(np.float64)))
            azimuth_deg[azimuth_deg < 0] += 360.0
            arr["time"] = ((azimuth_deg / 360.0) * _SCAN_DURATION_MS).astype(arr.dtype["time"])

            out_msg = PointCloud2()
            out_msg.header = msg.header
            out_msg.height = msg.height
            out_msg.width = msg.width
            out_msg.fields = msg.fields
            out_msg.is_bigendian = msg.is_bigendian
            out_msg.point_step = msg.point_step
            out_msg.row_step = msg.row_step
            out_msg.is_dense = msg.is_dense
            out_msg.data = arr.tobytes()
            self.pub.publish(out_msg)
        except Exception as exc:
            self.get_logger().warn(f"Failed to process point cloud: {exc}")


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
