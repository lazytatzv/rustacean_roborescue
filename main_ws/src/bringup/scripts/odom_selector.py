#!/usr/bin/env python3
"""
odom_selector: /imu/health と各 odom ソースの死活に基づいて /odom ソースを切り替える

切り替えロジック (優先順):
  1. spark_fast_lio  : IMU health=true かつ spark odom が fresh
  2. KISS-ICP        : IMU 死亡 / タイムアウト / spark stale (allow_kiss_fallback=true 時)
  3. T265 visual odom: kiss も stale もしくは LiDAR 無効 (use_t265_odom=true 時)

T265 座標変換について:
  T265 が publish する odom は T265 カメラ自身の座標系での位置・姿勢。
  ロボットの /odom (base_link の位置) を得るには、起動時に TF から
  camera_t265_link → base_link の静的変換を取得し、T265 odom に合成する。
  URDF に camera_t265_link が定義されていれば自動的に計算される。
  TF が取得できない場合は変換なしで republish し警告を出す (クラッシュしない)。

将来の改善候補:
  - robot_localization EKF によるハードスイッチングの廃止 (ジャンプ防止)
  - D435i IMU を EKF に追加
"""

import threading

import rclpy
import rclpy.duration
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from std_msgs.msg import Bool


# ---------------------------------------------------------------------------
# クォータニオン演算ユーティリティ (外部依存なし)
# ---------------------------------------------------------------------------


def _quat_mul(q1: tuple, q2: tuple) -> tuple:
    """q1 * q2"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def _quat_rotate(q: tuple, v: tuple) -> tuple:
    """クォータニオン q でベクトル v を回転する。"""
    qv = (0.0, v[0], v[1], v[2])
    q_conj = (q[0], -q[1], -q[2], -q[3])
    r = _quat_mul(_quat_mul(q, qv), q_conj)
    return r[1], r[2], r[3]


def _quat_inv(q: tuple) -> tuple:
    """単位クォータニオンの逆 (= 共役)。"""
    w, x, y, z = q
    return (w, -x, -y, -z)


def _compute_se3_offset(last_pose: tuple, odom_msg) -> tuple:
    """
    SE(3) オフセット = last_robot_pose ∘ t265_current^{-1} を計算する。

    T265 odom の原点がロボット odom の原点と異なる場合に、切り替え時の位置ジャンプを
    補正するためのオフセット変換。以降の T265 odom にこの変換を適用することで
    /odom トピックの連続性が保たれる。

    返値: (tx, ty, tz, qw, qx, qy, qz)
    """
    lx, ly, lz, lqw, lqx, lqy, lqz = last_pose
    px = odom_msg.pose.pose.position.x
    py = odom_msg.pose.pose.position.y
    pz = odom_msg.pose.pose.position.z
    q_t265 = (
        odom_msg.pose.pose.orientation.w,
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
    )
    q_last = (lqw, lqx, lqy, lqz)
    # q_offset = q_last * q_t265^{-1}
    q_off = _quat_mul(q_last, _quat_inv(q_t265))
    # t_offset = p_last - R_offset * p_t265
    rotated = _quat_rotate(q_off, (px, py, pz))
    t_off = (lx - rotated[0], ly - rotated[1], lz - rotated[2])
    return (*t_off, *q_off)


def _apply_se3_offset(msg, offset: tuple):
    """SE(3) オフセットを odom メッセージに適用し、新しい Odometry を返す。"""
    from nav_msgs.msg import Odometry as _Odom

    tx, ty, tz, qow, qox, qoy, qoz = offset
    q_off = (qow, qox, qoy, qoz)
    px = msg.pose.pose.position.x
    py = msg.pose.pose.position.y
    pz = msg.pose.pose.position.z
    q_in = (
        msg.pose.pose.orientation.w,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
    )
    rotated = _quat_rotate(q_off, (px, py, pz))
    q_pub = _quat_mul(q_off, q_in)
    # 速度もオフセットの回転成分で変換する
    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    vz = msg.twist.twist.linear.z
    wx = msg.twist.twist.angular.x
    wy = msg.twist.twist.angular.y
    wz = msg.twist.twist.angular.z
    v_rot = _quat_rotate(q_off, (vx, vy, vz))
    w_rot = _quat_rotate(q_off, (wx, wy, wz))

    out = _Odom()
    out.header = msg.header
    out.child_frame_id = msg.child_frame_id
    out.pose.pose.position.x = tx + rotated[0]
    out.pose.pose.position.y = ty + rotated[1]
    out.pose.pose.position.z = tz + rotated[2]
    out.pose.pose.orientation.w = q_pub[0]
    out.pose.pose.orientation.x = q_pub[1]
    out.pose.pose.orientation.y = q_pub[2]
    out.pose.pose.orientation.z = q_pub[3]
    out.twist.twist.linear.x = v_rot[0]
    out.twist.twist.linear.y = v_rot[1]
    out.twist.twist.linear.z = v_rot[2]
    out.twist.twist.angular.x = w_rot[0]
    out.twist.twist.angular.y = w_rot[1]
    out.twist.twist.angular.z = w_rot[2]
    return out


# ---------------------------------------------------------------------------
# OdomSelector
# ---------------------------------------------------------------------------


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
        self.declare_parameter("use_lidar", True)
        self.declare_parameter("use_t265_odom", False)
        self.declare_parameter("t265_odom_topic", "/camera_t265/odom/sample")
        self.declare_parameter("t265_link_frame", "camera_t265_link")
        self.declare_parameter("t265_timeout_s", 5.0)
        self.declare_parameter("kiss_timeout_s", 5.0)

        self._base_frame = self.get_parameter("base_frame").value
        self._odom_frame = self.get_parameter("odom_frame").value
        health_timeout = float(self.get_parameter("health_timeout_s").value)
        self._spark_timeout = float(self.get_parameter("spark_timeout_s").value)
        self._allow_kiss_fallback = bool(self.get_parameter("allow_kiss_fallback").value)
        # LaunchConfiguration から来る値は文字列の場合もある
        _use_lidar = self.get_parameter("use_lidar").value
        self._use_lidar = (
            _use_lidar if isinstance(_use_lidar, bool) else str(_use_lidar).lower() == "true"
        )
        _use_t265 = self.get_parameter("use_t265_odom").value
        self._use_t265_odom = (
            _use_t265 if isinstance(_use_t265, bool) else str(_use_t265).lower() == "true"
        )
        t265_topic = str(self.get_parameter("t265_odom_topic").value)
        self._t265_link_frame = str(self.get_parameter("t265_link_frame").value)
        self._t265_timeout = float(self.get_parameter("t265_timeout_s").value)
        self._kiss_timeout = float(self.get_parameter("kiss_timeout_s").value)

        # --- LiDAR odom 状態 -----------------------------------------------
        self._use_imu = True
        self._spark_stale = False
        self._health_received = False
        self._health_false_since = None
        self._health_true_since = None
        self._last_spark_time = None

        # --- KISS staleness 追跡 -------------------------------------------
        self._last_kiss_time = None
        self._kiss_stale = False

        # --- T265 staleness 追跡 -------------------------------------------
        self._last_t265_time = None
        self._t265_stale = True  # データ到着まで stale 扱い

        # --- T265 座標変換キャッシュ ----------------------------------------
        # camera_t265_link → base_link の静的 TF
        # (tx, ty, tz, qw, qx, qy, qz) または None
        self._t265_to_base: tuple | None = None
        self._t265_tf_warn_logged = False

        # --- T265 origin オフセット -----------------------------------------
        # T265 モードへの切り替え時に /odom が連続するよう SE(3) 補正を保持する。
        # _t265_offset_active: True の間はオフセットが計算済み。
        # T265 モードを抜けたらリセットし、次回再入時に再計算する。
        self._last_pub_pose: tuple | None = None  # 最後に publish した odom pose
        self._t265_offset: tuple | None = None  # SE(3) オフセット変換
        self._t265_offset_active = False

        self._cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()

        self._pub = self.create_publisher(Odometry, "/odom", 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- TF バッファ (T265 座標変換のため) ------------------------------
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # --- 既存の LiDAR odom サブスクリプション --------------------------
        self.create_subscription(
            Bool, "/imu/health", self._health_cb, 10, callback_group=self._cb_group
        )
        self.create_subscription(
            Odometry, "/spark_lio/odom", self._spark_cb, 10, callback_group=self._cb_group
        )
        self.create_subscription(
            Odometry, "/kiss/odometry", self._kiss_cb, 10, callback_group=self._cb_group
        )

        # --- T265 サブスクリプション (use_t265_odom=true の場合のみ) --------
        if self._use_t265_odom:
            self.create_subscription(
                Odometry, t265_topic, self._t265_cb, 10, callback_group=self._cb_group
            )
            self.get_logger().info(f"T265 odom enabled: subscribing to {t265_topic}")

        # --- タイマー -------------------------------------------------------
        self._timeout_timer = self.create_timer(
            health_timeout, self._health_timeout_cb, callback_group=self._cb_group
        )
        self.create_timer(1.0, self._spark_watchdog_cb, callback_group=self._cb_group)
        self.create_timer(1.0, self._kiss_watchdog_cb, callback_group=self._cb_group)
        if self._use_t265_odom:
            self.create_timer(1.0, self._t265_watchdog_cb, callback_group=self._cb_group)

        mode_desc = "spark_fast_lio"
        if self._allow_kiss_fallback:
            mode_desc += " → KISS-ICP"
        if self._use_t265_odom:
            mode_desc += " → T265"
        self.get_logger().info(
            f"odom_selector started (priority={mode_desc}, "
            f"use_lidar={self._use_lidar}, base={self._base_frame})"
        )
        if not self._allow_kiss_fallback:
            self.get_logger().warn("KISS fallback is disabled")

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _is_kiss_mode(self) -> bool:
        if not self._allow_kiss_fallback:
            return False
        with self._lock:
            return (not self._use_imu) or self._spark_stale

    def _is_t265_mode(self) -> bool:
        if not self._use_t265_odom:
            return False
        with self._lock:
            if self._t265_stale:
                return False
            if not self._use_lidar:
                return True
            in_kiss = self._allow_kiss_fallback and ((not self._use_imu) or self._spark_stale)
            return in_kiss and self._kiss_stale

    def _broadcast_odom_tf(self, msg: Odometry) -> None:
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(tf)

    def _lookup_t265_static_tf(self) -> bool:
        """
        camera_t265_link → base_link の静的 TF を lookup してキャッシュする。
        成功したら True を返す。
        """
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                self._t265_link_frame,
                self._base_frame,
                RclpyTime(),
                rclpy.duration.Duration(seconds=0.1),
            )
            t = tf_msg.transform.translation
            r = tf_msg.transform.rotation
            self._t265_to_base = (t.x, t.y, t.z, r.w, r.x, r.y, r.z)
            self.get_logger().info(
                f"T265→base_link static TF cached: " f"t=({t.x:.3f},{t.y:.3f},{t.z:.3f})"
            )
            return True
        except Exception:
            return False

    def _transform_t265_odom(self, msg: Odometry) -> Odometry | None:
        """
        T265 odom (camera_t265_link 座標系) を base_link odom に変換する。

        T265 は自分自身の位置を自分の odom_frame で報告する。
        base_link の位置を得るには:
          base_link_pos = T265_pos + rotate(T265_orientation, offset_camera_to_base)
          base_link_ori = T265_orientation × static_rotation_camera_to_base

        TF が未取得の場合はキャッシュを試みる。それでも失敗なら None を返す。
        """
        if self._t265_to_base is None:
            if not self._lookup_t265_static_tf():
                if not self._t265_tf_warn_logged:
                    self.get_logger().warn(
                        f"T265 static TF not yet available "
                        f"({self._t265_link_frame}→{self._base_frame}). "
                        "URDF に camera_t265_link が定義されているか確認してください。"
                        "変換なしで republish します (位置精度が低下します)。"
                    )
                    self._t265_tf_warn_logged = True
                # フォールバック: 変換なしで republish
                out = Odometry()
                out.header = msg.header
                out.header.frame_id = self._odom_frame
                out.child_frame_id = self._base_frame
                out.pose = msg.pose
                return out

        tx, ty, tz, qw, qx, qy, qz = self._t265_to_base

        # T265 の現在位置・姿勢 (camera_t265_odom_frame 内)
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        q_t265 = (
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        )
        q_static = (qw, qx, qy, qz)

        # base_link の位置 = T265位置 + T265向きで回転した camera→base オフセット
        rotated = _quat_rotate(q_t265, (tx, ty, tz))
        bx = px + rotated[0]
        by = py + rotated[1]
        bz = pz + rotated[2]

        # base_link の向き = T265向き × camera→base 静的回転
        q_base = _quat_mul(q_t265, q_static)

        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self._odom_frame
        out.child_frame_id = self._base_frame
        out.pose.pose.position.x = bx
        out.pose.pose.position.y = by
        out.pose.pose.position.z = bz
        out.pose.pose.orientation.w = q_base[0]
        out.pose.pose.orientation.x = q_base[1]
        out.pose.pose.orientation.y = q_base[2]
        out.pose.pose.orientation.z = q_base[3]
        return out

    # -----------------------------------------------------------------------
    # IMU health callbacks
    # -----------------------------------------------------------------------

    def _health_timeout_cb(self):
        if not self._allow_kiss_fallback:
            return
        self._timeout_timer.cancel()
        with self._lock:
            if not self._health_received:
                self._use_imu = False
        self.get_logger().warn(
            "No /imu/health received within timeout -> switching to KISS-ICP odom"
        )

    def _health_cb(self, msg: Bool):
        if not self._allow_kiss_fallback:
            return
        now = self._now()
        log = None
        with self._lock:
            self._health_received = True
            if msg.data:
                self._health_false_since = None
                if self._health_true_since is None:
                    self._health_true_since = now
                if not self._use_imu and (now - self._health_true_since) >= self.ALIVE_THRESHOLD:
                    self._use_imu = True
                    self._health_true_since = None
                    log = ("info", "IMU recovered -> switching back to spark_fast_lio odom")
            else:
                self._health_true_since = None
                if self._health_false_since is None:
                    self._health_false_since = now
                if self._use_imu and (now - self._health_false_since) >= self.DEAD_THRESHOLD:
                    self._use_imu = False
                    self._health_false_since = None
                    log = ("warn", "IMU dead -> switching to KISS-ICP odom")
        if log:
            getattr(self.get_logger(), log[0])(log[1])

    # -----------------------------------------------------------------------
    # Watchdogs
    # -----------------------------------------------------------------------

    def _spark_watchdog_cb(self):
        if not self._allow_kiss_fallback:
            return
        now = self._now()
        with self._lock:
            if self._last_spark_time is None:
                return
            stale = (now - self._last_spark_time) > self._spark_timeout
            prev_stale = self._spark_stale
            self._spark_stale = stale
        if stale and not prev_stale:
            self.get_logger().warn(
                f"spark_fast_lio odom stale (>{self._spark_timeout}s) -> switching to KISS-ICP"
            )
        elif not stale and prev_stale:
            self.get_logger().info("spark_fast_lio odom recovered")

    def _kiss_watchdog_cb(self):
        if not self._use_t265_odom:
            return
        now = self._now()
        with self._lock:
            if self._last_kiss_time is None:
                return
            stale = (now - self._last_kiss_time) > self._kiss_timeout
            prev_stale = self._kiss_stale
            self._kiss_stale = stale
        if stale and not prev_stale:
            self.get_logger().warn(
                f"KISS-ICP odom stale (>{self._kiss_timeout}s) -> T265 fallback may activate"
            )
        elif not stale and prev_stale:
            self.get_logger().info("KISS-ICP odom recovered")

    def _t265_watchdog_cb(self):
        now = self._now()
        with self._lock:
            if self._last_t265_time is None:
                return
            stale = (now - self._last_t265_time) > self._t265_timeout
            prev_stale = self._t265_stale
            self._t265_stale = stale
        if stale and not prev_stale:
            self.get_logger().warn(
                f"T265 odom stale (>{self._t265_timeout}s) -> T265 mode deactivated"
            )
        elif not stale and prev_stale:
            self.get_logger().info("T265 odom recovered")

    # -----------------------------------------------------------------------
    # odom callbacks
    # -----------------------------------------------------------------------

    def _spark_cb(self, msg: Odometry):
        with self._lock:
            self._last_spark_time = self._now()
        if not self._is_kiss_mode():
            msg.header.frame_id = self._odom_frame
            msg.child_frame_id = self._base_frame
            self._pub.publish(msg)
            with self._lock:
                self._last_pub_pose = (
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                    msg.pose.pose.orientation.w,
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                )

    def _kiss_cb(self, msg: Odometry):
        with self._lock:
            self._last_kiss_time = self._now()
        if self._is_kiss_mode() and not self._is_t265_mode():
            msg.header.frame_id = self._odom_frame
            msg.child_frame_id = self._base_frame
            self._pub.publish(msg)
            self._broadcast_odom_tf(msg)
            with self._lock:
                self._last_pub_pose = (
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                    msg.pose.pose.orientation.w,
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                )

    def _t265_cb(self, msg: Odometry):
        with self._lock:
            first_msg = self._last_t265_time is None
            self._last_t265_time = self._now()
            if first_msg:
                self._t265_stale = False

        if not self._is_t265_mode():
            # T265 モード外: 次回有効化時に origin オフセットを再計算するためリセット
            with self._lock:
                self._t265_offset_active = False
            return

        transformed = self._transform_t265_odom(msg)
        if transformed is None:
            return

        # T265 モードに入った最初のフレームでオフセットを1回だけ計算する
        with self._lock:
            if not self._t265_offset_active:
                self._t265_offset_active = True
                self._t265_offset = (
                    _compute_se3_offset(self._last_pub_pose, transformed)
                    if self._last_pub_pose is not None
                    else None
                )
                if self._t265_offset is not None:
                    self.get_logger().info("T265 origin offset computed (odom continuity ensured)")
                else:
                    self.get_logger().warn(
                        "T265 origin offset: no prior odom available, using T265 as-is"
                    )
            offset = self._t265_offset

        if offset is not None:
            transformed = _apply_se3_offset(transformed, offset)

        self._pub.publish(transformed)
        self._broadcast_odom_tf(transformed)
        with self._lock:
            self._last_pub_pose = (
                transformed.pose.pose.position.x,
                transformed.pose.pose.position.y,
                transformed.pose.pose.position.z,
                transformed.pose.pose.orientation.w,
                transformed.pose.pose.orientation.x,
                transformed.pose.pose.orientation.y,
                transformed.pose.pose.orientation.z,
            )


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
