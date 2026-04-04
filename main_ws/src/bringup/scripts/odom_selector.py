#!/usr/bin/env python3
"""
odom_selector: /imu/health と spark odom の死活に基づいて /odom ソースを切り替える

切り替えロジック:
  1. IMU health=true かつ spark odom が fresh -> spark_fast_lio を使用
  2. IMU health=false (DEAD_THRESHOLD 秒継続) -> KISS-ICP へ切替
  3. /imu/health が health_timeout_s 秒届かない -> IMU 未接続とみなし KISS-ICP へ
  4. spark_fast_lio が stale (spark_timeout_s 秒届かない) -> KISS-ICP へ切替

ヒステリシス:
  - IMU 死亡判定: health=false が DEAD_THRESHOLD 秒継続後に切り替え
  - IMU 復活判定: health=true が ALIVE_THRESHOLD 秒継続後に切り替え
"""

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool


class OdomSelector(Node):
    DEAD_THRESHOLD = 1.0
    ALIVE_THRESHOLD = 3.0

    def __init__(self):
        super().__init__("odom_selector")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("health_timeout_s", 5.0)
        self.declare_parameter("spark_timeout_s", 3.0)
        self.declare_parameter("allow_kiss_fallback", True)
        self._base_frame = self.get_parameter("base_frame").value
        self._odom_frame = self.get_parameter("odom_frame").value
        health_timeout = self.get_parameter("health_timeout_s").value
        self._spark_timeout = self.get_parameter("spark_timeout_s").value
        self._allow_kiss_fallback = bool(self.get_parameter("allow_kiss_fallback").value)

        self._use_imu = True
        self._spark_stale = False
        self._health_received = False
        self._health_false_since = None
        self._health_true_since = None
        self._last_spark_time = None

        # 複数コールバックを並行実行するため ReentrantCallbackGroup を使う
        self._cb_group = ReentrantCallbackGroup()

        self._pub = self.create_publisher(Odometry, "/odom", 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Bool, "/imu/health", self._health_cb, 10, callback_group=self._cb_group)
        self.create_subscription(Odometry, "/spark_lio/odom", self._spark_cb, 10, callback_group=self._cb_group)
        self.create_subscription(Odometry, "/kiss/odometry", self._kiss_cb, 10, callback_group=self._cb_group)

        self._timeout_timer = self.create_timer(health_timeout, self._health_timeout_cb, callback_group=self._cb_group)
        self.create_timer(1.0, self._spark_watchdog_cb, callback_group=self._cb_group)

        self.get_logger().info(
            f"odom_selector started (mode=spark_fast_lio, base={self._base_frame}, "
            f"health_timeout={health_timeout}s, spark_timeout={self._spark_timeout}s)"
        )
        if not self._allow_kiss_fallback:
            self.get_logger().warn("KISS fallback is disabled; /odom will stay on spark_fast_lio")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _health_timeout_cb(self):
        if not self._allow_kiss_fallback:
            return
        self._timeout_timer.cancel()
        if not self._health_received:
            self._use_imu = False
            self.get_logger().warn(
                "No /imu/health received within timeout -> switching to KISS-ICP odom"
            )

    def _spark_watchdog_cb(self):
        if not self._allow_kiss_fallback:
            return
        now = self._now()
        if self._last_spark_time is None:
            return
        stale = (now - self._last_spark_time) > self._spark_timeout
        if stale and not self._spark_stale:
            self._spark_stale = True
            self.get_logger().warn(
                f"spark_fast_lio odom stale (>{self._spark_timeout}s) -> switching to KISS-ICP"
            )
        elif not stale and self._spark_stale:
            self._spark_stale = False
            self.get_logger().info("spark_fast_lio odom recovered")

    def _is_kiss_mode(self) -> bool:
        if not self._allow_kiss_fallback:
            return False
        return (not self._use_imu) or self._spark_stale

    def _health_cb(self, msg: Bool):
        if not self._allow_kiss_fallback:
            return
        self._health_received = True
        now = self._now()
        if msg.data:
            self._health_false_since = None
            if self._health_true_since is None:
                self._health_true_since = now
            if not self._use_imu and (now - self._health_true_since) >= self.ALIVE_THRESHOLD:
                self._use_imu = True
                self._health_true_since = None
                self.get_logger().info("IMU recovered -> switching back to spark_fast_lio odom")
        else:
            self._health_true_since = None
            if self._health_false_since is None:
                self._health_false_since = now
            if self._use_imu and (now - self._health_false_since) >= self.DEAD_THRESHOLD:
                self._use_imu = False
                self._health_false_since = None
                self.get_logger().warn("IMU dead -> switching to KISS-ICP odom")

    def _spark_cb(self, msg: Odometry):
        self._last_spark_time = self._now()
        if not self._is_kiss_mode():
            self._pub.publish(msg)

    def _kiss_cb(self, msg: Odometry):
        if self._is_kiss_mode():
            msg.header.frame_id = self._odom_frame
            msg.child_frame_id = self._base_frame
            self._pub.publish(msg)

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
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
