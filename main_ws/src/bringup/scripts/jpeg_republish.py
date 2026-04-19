#!/usr/bin/env python3
"""image_raw → CompressedImage (JPEG) 常時配信ノード。

image_transport::RepublishNode は lazy publishing のため subscriber が
いないと圧縮トピックが止まる。このノードは subscriber 数に関わらず
常に JPEG を配信する。

パラメータ:
  in_topic     : 購読する raw image トピック (デフォルト: image_raw)
  out_topic    : 配信する CompressedImage トピック (デフォルト: image_raw/compressed)
  jpeg_quality : JPEG 品質 1-100 (デフォルト: 80)
"""

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class JpegRepublishNode(Node):
    def __init__(self) -> None:
        super().__init__("jpeg_republish")

        self.declare_parameter("in_topic", "image_raw")
        self.declare_parameter("out_topic", "image_raw/compressed")
        self.declare_parameter("jpeg_quality", 80)

        in_topic = self.get_parameter("in_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("out_topic").get_parameter_value().string_value
        quality = self.get_parameter("jpeg_quality").get_parameter_value().integer_value

        self._bridge = CvBridge()
        self._encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), quality]

        # v4l2_camera は SensorDataQoS (BEST_EFFORT) で配信するため合わせる。
        # BEST_EFFORT subscriber は RELIABLE/BEST_EFFORT どちらの publisher とも接続できる。
        qos_sub = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Foxglove bridge 向けは RELIABLE で配信する。
        qos_pub = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._pub = self.create_publisher(CompressedImage, out_topic, qos_pub)
        self._sub = self.create_subscription(Image, in_topic, self._cb, qos_sub)
        self.get_logger().info(f"jpeg_republish: {in_topic} → {out_topic} (quality={quality})")

    def _cb(self, msg: Image) -> None:
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return

        ok, buf = cv2.imencode(".jpg", cv_img, self._encode_params)
        if not ok:
            return

        out = CompressedImage()
        out.header = msg.header
        out.format = "jpeg"
        out.data = buf.tobytes()
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JpegRepublishNode()
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
