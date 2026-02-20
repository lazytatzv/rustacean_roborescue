#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class DummyImuNode(Node):
    def __init__(self) -> None:
        super().__init__("dummy_imu")

        self.declare_parameter("topic", "/imu/data")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("use_gravity", True)
        self.declare_parameter("gravity_mps2", 9.80665)

        topic = self.get_parameter("topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        self.use_gravity = self.get_parameter("use_gravity").get_parameter_value().bool_value
        self.gravity = self.get_parameter("gravity_mps2").get_parameter_value().double_value

        self.pub = self.create_publisher(Imu, topic, 10)
        period = 1.0 / max(rate_hz, 1.0)
        self.timer = self.create_timer(period, self.publish_imu)

        self.get_logger().info(
            f"Publishing dummy IMU to {topic} @ {rate_hz:.1f} Hz (frame_id={self.frame_id})"
        )

    def publish_imu(self) -> None:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Identity orientation
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0

        # Zero angular velocity
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0

        # Linear acceleration (gravity on Z if enabled)
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = self.gravity if self.use_gravity else 0.0

        # Simple covariances
        msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]

        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = DummyImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
