#!/usr/bin/env python3
"""
odom_selector: /imu/health に基づいて /odom ソースを切り替える

- IMU 健全時: /spark_lio/odom → /odom (spark_fast_lio が TF も担当)
- IMU 死亡時: /kiss/odometry → /odom + TF odom→base_link をブロードキャスト

ヒステリシス:
  - IMU 死亡判定: health=false が DEAD_THRESHOLD 秒継続後に切り替え
  - IMU 復活判定: health=true  が ALIVE_THRESHOLD 秒継続後に切り替え
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdomSelector(Node):
    DEAD_THRESHOLD = 1.0   # IMU dead → switch to KISS-ICP after N seconds
    ALIVE_THRESHOLD = 3.0  # IMU alive → switch back to spark_fast_lio after N seconds

    def __init__(self):
        super().__init__('odom_selector')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self._base_frame = self.get_parameter('base_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value

        self._use_imu = True
        self._health_false_since = None
        self._health_true_since = None

        self._pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Bool, '/imu/health', self._health_cb, 10)
        self.create_subscription(Odometry, '/spark_lio/odom', self._spark_cb, 10)
        self.create_subscription(Odometry, '/kiss/odometry', self._kiss_cb, 10)

        self.get_logger().info(
            f'odom_selector started (mode=spark_fast_lio, base={self._base_frame})'
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _health_cb(self, msg: Bool):
        now = self._now()
        if msg.data:
            self._health_false_since = None
            if self._health_true_since is None:
                self._health_true_since = now
            if not self._use_imu and (now - self._health_true_since) >= self.ALIVE_THRESHOLD:
                self._use_imu = True
                self._health_true_since = None
                self.get_logger().info('IMU recovered → switching back to spark_fast_lio odom')
        else:
            self._health_true_since = None
            if self._health_false_since is None:
                self._health_false_since = now
            if self._use_imu and (now - self._health_false_since) >= self.DEAD_THRESHOLD:
                self._use_imu = False
                self._health_false_since = None
                self.get_logger().warn('IMU dead → switching to KISS-ICP lidar odom')

    def _spark_cb(self, msg: Odometry):
        if self._use_imu:
            self._pub.publish(msg)
            # spark_fast_lio publishes its own TF; no extra broadcast needed

    def _kiss_cb(self, msg: Odometry):
        if not self._use_imu:
            # Normalise frame IDs
            msg.header.frame_id = self._odom_frame
            msg.child_frame_id = self._base_frame
            self._pub.publish(msg)

            # Broadcast TF because spark_fast_lio is stalled
            tf = TransformStamped()
            tf.header = msg.header
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = msg.pose.pose.position.x
            tf.transform.translation.y = msg.pose.pose.position.y
            tf.transform.translation.z = msg.pose.pose.position.z
            tf.transform.rotation = msg.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = OdomSelector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
