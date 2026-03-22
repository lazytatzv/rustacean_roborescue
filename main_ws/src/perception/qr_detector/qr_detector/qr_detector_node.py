"""
qr_detector_node: ROS 2 node for real-time QR code detection.

Uses OpenCV's WeChatQRCode (contrib) detector for robust detection.
Optionally publishes JPEG-compressed images for remote monitoring.

Subscribe: /camera/image_raw  (sensor_msgs/Image)
Publish:   /qr_codes          (std_msgs/String)       — detected QR content
Publish:   /image/compressed   (sensor_msgs/CompressedImage) — optional
"""

import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


class QrDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("qr_detector")

        # ---- Parameters ----------------------------------------------------
        self.declare_parameter("model_dir", "")
        self.declare_parameter("publish_compressed", True)
        self.declare_parameter("jpeg_quality", 60)
        self.declare_parameter("detection_interval", 1)  # process every N-th frame

        model_dir = self.get_parameter("model_dir").get_parameter_value().string_value
        self._publish_compressed: bool = (
            self.get_parameter("publish_compressed").get_parameter_value().bool_value
        )
        self._jpeg_quality: int = (
            self.get_parameter("jpeg_quality").get_parameter_value().integer_value
        )
        self._detection_interval: int = (
            self.get_parameter("detection_interval").get_parameter_value().integer_value
        )

        # Resolve model directory
        if not model_dir:
            # Try installed share directory first, fall back to source
            try:
                pkg_share = get_package_share_directory("qr_detector")
                model_dir = os.path.join(pkg_share, "models")
            except Exception:
                model_dir = os.path.join(
                    os.path.dirname(__file__), "..", "..", "models"
                )

        # ---- WeChatQRCode detector -----------------------------------------
        detect_proto = os.path.join(model_dir, "detect.prototxt")
        detect_model = os.path.join(model_dir, "detect.caffemodel")
        sr_proto = os.path.join(model_dir, "sr.prototxt")
        sr_model = os.path.join(model_dir, "sr.caffemodel")

        for f in (detect_proto, detect_model, sr_proto, sr_model):
            if not os.path.isfile(f):
                self.get_logger().warn(f"Model file not found: {f}")

        self._detector = cv2.wechat_qrcode_WeChatQRCode(
            detect_proto, detect_model, sr_proto, sr_model
        )
        self.get_logger().info("WeChatQRCode detector initialized")

        # ---- cv_bridge -----------------------------------------------------
        self._bridge = CvBridge()

        # ---- QoS: sensor data = BEST_EFFORT --------------------------------
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ---- Subscriber ----------------------------------------------------
        self.create_subscription(
            Image, "/camera/image_raw", self._image_callback, sensor_qos
        )

        # ---- Publishers ----------------------------------------------------
        self._qr_pub = self.create_publisher(String, "/qr_codes", 10)

        if self._publish_compressed:
            self._compressed_pub = self.create_publisher(
                CompressedImage, "/image/compressed", sensor_qos
            )
        else:
            self._compressed_pub = None

        self._frame_count: int = 0

        self.get_logger().info(
            f"qr_detector started (interval={self._detection_interval}, "
            f"jpeg_q={self._jpeg_quality}, compressed={self._publish_compressed})"
        )

    # -----------------------------------------------------------------------
    def _image_callback(self, msg: Image) -> None:
        self._frame_count += 1

        # Frame skipping for performance
        if self._frame_count % self._detection_interval != 0:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # ---- QR detection --------------------------------------------------
        results, points = self._detector.detectAndDecode(frame)

        for i, content in enumerate(results):
            if not content:
                continue

            qr_msg = String()
            qr_msg.data = content
            self._qr_pub.publish(qr_msg)
            self.get_logger().info(f"QR detected: {content}")

            # Draw bounding box on frame for compressed output
            if (
                self._compressed_pub is not None
                and points is not None
                and i < len(points)
            ):
                pts = points[i].astype(np.int32).reshape(-1, 2)
                cv2.polylines(
                    frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2
                )
                cv2.putText(
                    frame,
                    content[:40],
                    (pts[0][0], pts[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2,
                )

        # ---- Publish compressed image (with annotations) -------------------
        if self._compressed_pub is not None:
            comp_msg = CompressedImage()
            comp_msg.header = msg.header
            comp_msg.format = "jpeg"
            _, buf = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
            )
            comp_msg.data = buf.tobytes()
            self._compressed_pub.publish(comp_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QrDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
