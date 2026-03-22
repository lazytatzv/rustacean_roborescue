#!/usr/bin/env python3
"""
ROS2 node: subscribe to H.264 CompressedImage topic and republish decoded
frames as sensor_msgs/Image for reliable viewing (e.g., Foxglove).

Entry point: `h264_republisher`
"""

import sys

import av
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class H264Republisher(Node):
    def __init__(self, in_topic: str, out_topic: str):
        super().__init__("h264_republisher")
        self.get_logger().info(f"Listening: {in_topic} -> Republishing: {out_topic}")
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, out_topic, 10)
        self.sub = self.create_subscription(
            CompressedImage, in_topic, self.cb_compressed, 10
        )
        try:
            self.decoder = av.CodecContext.create("h264", "r")
        except Exception as e:
            self.get_logger().error(f"Failed to create H264 decoder: {e}")
            raise

    def cb_compressed(self, msg: CompressedImage):
        data = msg.data
        if not data:
            return
        try:
            packet = av.packet.Packet(data)
            frames = self.decoder.decode(packet)
        except Exception as e:
            self.get_logger().warn(f"Decode error: {e}")
            return

        for frame in frames:
            try:
                arr = frame.to_ndarray(format="bgr24")
                out_msg = self.bridge.cv2_to_imgmsg(arr, encoding="bgr8")
                out_msg.header = msg.header
                self.pub.publish(out_msg)
            except Exception as e:
                self.get_logger().warn(f"Frame conversion error: {e}")


def main(argv=sys.argv[1:]):
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--in-topic", default="/camera/image_raw/ffmpeg")
    parser.add_argument("--out-topic", default="/camera/image_raw")
    args = parser.parse_args(argv)

    rclpy.init()
    node = H264Republisher(args.in_topic, args.out_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
