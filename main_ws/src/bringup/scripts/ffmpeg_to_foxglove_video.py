#!/usr/bin/env python3
"""FFMPEGPacket → foxglove_msgs/CompressedVideo 変換ノード.

foxglove_bridge + Foxglove Studio でネイティブH264表示するために
ffmpeg_image_transport_msgs/msg/FFMPEGPacket を
foxglove_msgs/msg/CompressedVideo に詰め替える。

パラメータ:
  input_topic  : 購読する FFMPEGPacket トピック (デフォルト: /camera/image_raw/ffmpeg)
  output_topic : 配信する CompressedVideo トピック (デフォルト: /camera/compressed_video)
"""

import rclpy
from ffmpeg_image_transport_msgs.msg import FFMPEGPacket
from foxglove_msgs.msg import CompressedVideo
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class FfmpegToFoxgloveVideo(Node):
    def __init__(self) -> None:
        super().__init__("ffmpeg_to_foxglove_video")

        self.declare_parameter("input_topic", "/camera/image_raw/ffmpeg")
        self.declare_parameter("output_topic", "/camera/compressed_video")

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        qos_sub = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_pub = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._pub = self.create_publisher(CompressedVideo, output_topic, qos_pub)
        self._sub = self.create_subscription(FFMPEGPacket, input_topic, self._cb, qos_sub)
        self.get_logger().info(f"ffmpeg_to_foxglove_video: {input_topic} → {output_topic}")

    def _cb(self, msg: FFMPEGPacket) -> None:
        out = CompressedVideo()
        out.timestamp = msg.header.stamp
        out.frame_id = msg.header.frame_id
        out.data = bytes(msg.data)
        out.format = "h264"
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FfmpegToFoxgloveVideo()
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
