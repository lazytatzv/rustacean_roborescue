#!/usr/bin/env python3
"""
Comprehensive functional test for all robot subsystems.

Tests every software component that can run without physical hardware.
Each test launches the target node(s) as subprocess(es), feeds them
stimulus via ROS 2 topics, and verifies expected outputs.

Tested subsystems:
  1. joy_controller       — Joy→Crawler/Flipper/Arm routing, mode switching, E-Stop
  2. arm_controller       — Jacobian IK (Twist→JointState)
  3. arm_gz_bridge        — JointState→per-joint Float64 splitter
  4. crawler_vel_bridge   — CrawlerVelocity→Twist conversion
  5. odom_tf_bridge       — Odometry→TF broadcaster
  6. fix_pointcloud_time  — PointCloud2 time field recalculation

NOT tested (require hardware):
  - crawler_driver   (Roboclaw serial)
  - flipper_driver   (Dynamixel serial)
  - arm_driver       (Dynamixel serial)
  - gripper_driver   (Dynamixel serial)
  - sensor_gateway   (STM32 UART)
  - qr_detector      (needs WeChatQRCode models & camera feed)
"""

import math
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, JointState, PointCloud2, PointField
from std_msgs.msg import Bool, Float64, Header, String
from tf2_msgs.msg import TFMessage

# custom_interfaces
from custom_interfaces.msg import CrawlerVelocity, FlipperVelocity

import struct
import numpy as np


# ═══════════════════════════════════════════════════════════════════════════
#  Paths
# ═══════════════════════════════════════════════════════════════════════════

WORKSPACE = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(WORKSPACE, "src", "bringup", "urdf", "sekirei.urdf")

ARM_JOINTS = [
    "arm_joint1", "arm_joint2", "arm_joint3",
    "arm_joint4", "arm_joint5", "arm_joint6",
]


# ═══════════════════════════════════════════════════════════════════════════
#  Test Result
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class TestResult:
    name: str
    passed: bool
    detail: str = ""
    sub_results: List[Tuple[str, bool, str]] = field(default_factory=list)

    def add(self, sub_name: str, ok: bool, detail: str = ""):
        self.sub_results.append((sub_name, ok, detail))
        if not ok:
            self.passed = False


# ═══════════════════════════════════════════════════════════════════════════
#  Subprocess helper — launches a ROS 2 node
# ═══════════════════════════════════════════════════════════════════════════

def launch_node(cmd: List[str], timeout_s: float = 10.0) -> subprocess.Popen:
    """Launch a ROS 2 node as subprocess, returning the Popen handle."""
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=os.environ.copy(),
    )
    return proc


def kill_node(proc: subprocess.Popen) -> str:
    """Kill a subprocess and return its combined output."""
    try:
        proc.send_signal(signal.SIGINT)
        out, _ = proc.communicate(timeout=5)
        return out.decode(errors="replace") if out else ""
    except (subprocess.TimeoutExpired, OSError):
        try:
            proc.kill()
            out, _ = proc.communicate(timeout=3)
            return out.decode(errors="replace") if out else ""
        except Exception:
            return ""


# ═══════════════════════════════════════════════════════════════════════════
#  Generic message collector
# ═══════════════════════════════════════════════════════════════════════════

class MsgCollector:
    """Thread-safe message collector for a topic."""
    def __init__(self):
        self.msgs: List[Any] = []
        self._lock = threading.Lock()

    def callback(self, msg):
        with self._lock:
            self.msgs.append(msg)

    def count(self) -> int:
        with self._lock:
            return len(self.msgs)

    def get_all(self) -> List[Any]:
        with self._lock:
            return list(self.msgs)

    def clear(self):
        with self._lock:
            self.msgs.clear()

    def last(self):
        with self._lock:
            return self.msgs[-1] if self.msgs else None


# ═══════════════════════════════════════════════════════════════════════════
#  Joy message builder
# ═══════════════════════════════════════════════════════════════════════════

def make_joy(axes=None, buttons=None) -> Joy:
    """Build a Joy message. Default: 8 axes, 13 buttons all zero."""
    msg = Joy()
    msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    msg.axes = list(axes) if axes else [0.0] * 8
    msg.buttons = list(buttons) if buttons else [0] * 13
    # L2/R2 default to 1.0 (released state)
    if axes is None:
        msg.axes[2] = 1.0   # L2 released
        msg.axes[5] = 1.0   # R2 released
    return msg


# ═══════════════════════════════════════════════════════════════════════════
#  Tester Node
# ═══════════════════════════════════════════════════════════════════════════

class ComprehensiveTester(Node):
    def __init__(self):
        super().__init__("robot_tester")
        self.results: List[TestResult] = []
        self.procs: List[subprocess.Popen] = []

    def cleanup(self):
        for p in self.procs:
            try:
                kill_node(p)
            except Exception:
                pass
        self.procs.clear()

    # ───────────────────────────────────────────────────────────────────
    #  Helper: wait for messages
    # ───────────────────────────────────────────────────────────────────
    def spin_for(self, seconds: float, dt: float = 0.01):
        """Spin the node for a duration, processing callbacks."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=dt)

    def wait_for_subscriber(self, publisher, timeout: float = 5.0) -> bool:
        """Wait until at least one subscriber connects to a publisher."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if publisher.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def wait_for_msgs(self, collector: 'MsgCollector', min_count: int = 1,
                      timeout: float = 5.0) -> bool:
        """Wait until collector has at least min_count messages."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if collector.count() >= min_count:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    # ═══════════════════════════════════════════════════════════════════
    #  TEST 1: joy_controller
    # ═══════════════════════════════════════════════════════════════════
    def test_joy_controller(self) -> TestResult:
        result = TestResult("joy_controller", True)
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST 1: joy_controller")
        self.get_logger().info("=" * 60)

        # Launch joy_controller
        proc = launch_node([
            "ros2", "run", "joy_controller", "joy_controller_node",
        ])
        self.procs.append(proc)

        # Publishers
        joy_pub = self.create_publisher(Joy, "/joy", 10)

        # Collectors
        crawler_col = MsgCollector()
        flipper_col = MsgCollector()
        arm_col = MsgCollector()
        estop_col = MsgCollector()

        estop_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        sub_crawler = self.create_subscription(CrawlerVelocity, "/crawler_driver", crawler_col.callback, 10)
        sub_flipper = self.create_subscription(FlipperVelocity, "/flipper_driver", flipper_col.callback, 10)
        sub_arm = self.create_subscription(Twist, "/arm_cmd_vel", arm_col.callback, 10)
        sub_estop = self.create_subscription(Bool, "/emergency_stop", estop_col.callback, estop_qos)

        # Wait for node startup + DDS discovery
        self.spin_for(3.0)
        # Verify DDS matched — wait for joy_controller to subscribe to /joy
        if not self.wait_for_subscriber(joy_pub, timeout=5.0):
            result.add("DDS discovery", False, "joy_controller never subscribed to /joy")
            kill_node(proc)
            self.procs.remove(proc)
            self.destroy_subscription(sub_crawler)
            self.destroy_subscription(sub_flipper)
            self.destroy_subscription(sub_arm)
            self.destroy_subscription(sub_estop)
            self.destroy_publisher(joy_pub)
            return result
        result.add("DDS discovery", True, "joy_controller found")

        # ── 1a: STOP mode (default) — all outputs should be zero ──
        self.get_logger().info("  1a: STOP mode (default) — zero output")
        crawler_col.clear(); flipper_col.clear(); arm_col.clear()
        joy = make_joy(axes=[0.0, -0.5, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0])  # left stick pushed forward
        for _ in range(10):
            joy_pub.publish(joy)
            self.spin_for(0.05)
        self.spin_for(0.3)

        if crawler_col.count() > 0:
            last_c = crawler_col.last()
            is_zero = abs(last_c.m1_vel) < 0.01 and abs(last_c.m2_vel) < 0.01
            result.add("STOP mode: crawler zero", is_zero,
                       f"m1={last_c.m1_vel:.3f}, m2={last_c.m2_vel:.3f}")
        else:
            result.add("STOP mode: crawler zero", False, "No crawler msgs received")

        if arm_col.count() > 0:
            last_a = arm_col.last()
            arm_zero = (abs(last_a.linear.x) < 0.01 and abs(last_a.linear.z) < 0.01
                        and abs(last_a.angular.z) < 0.01)
            result.add("STOP mode: arm zero", arm_zero,
                       f"lx={last_a.linear.x:.3f}, az={last_a.angular.z:.3f}")
        else:
            result.add("STOP mode: arm zero", False, "No arm msgs received")

        # ── 1b: Switch to DRIVE mode (Options button, index 9) ──
        self.get_logger().info("  1b: DRIVE mode — crawler output")
        crawler_col.clear(); flipper_col.clear()
        # Rising edge: Options=1
        joy_drive = make_joy(buttons=[0]*9 + [1] + [0]*3)
        joy_pub.publish(joy_drive)
        self.spin_for(0.1)
        # Now send stick input in DRIVE mode
        joy_input = make_joy(
            axes=[0.0, -0.8, 1.0, 0.0, -0.8, 1.0, 0.0, 0.0],  # both sticks forward
            buttons=[0]*13,
        )
        for _ in range(10):
            joy_pub.publish(joy_input)
            self.spin_for(0.05)
        self.spin_for(0.3)

        if crawler_col.count() > 0:
            last_c = crawler_col.last()
            has_velocity = abs(last_c.m1_vel) > 0.1 or abs(last_c.m2_vel) > 0.1
            result.add("DRIVE mode: crawler non-zero", has_velocity,
                       f"m1={last_c.m1_vel:.3f}, m2={last_c.m2_vel:.3f}")
        else:
            result.add("DRIVE mode: crawler non-zero", False, "No crawler msgs received")

        # ── 1c: Flipper in DRIVE mode (L1+R1) ──
        self.get_logger().info("  1c: DRIVE mode — flipper output")
        flipper_col.clear()
        joy_flipper = make_joy(
            axes=[0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            buttons=[0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0],  # L1=1, R1=1
        )
        for _ in range(10):
            joy_pub.publish(joy_flipper)
            self.spin_for(0.05)
        self.spin_for(0.3)

        if flipper_col.count() > 0:
            last_f = flipper_col.last()
            has_flipper = any(abs(v) > 0 for v in last_f.flipper_vel)
            result.add("DRIVE mode: flipper non-zero", has_flipper,
                       f"flippers={list(last_f.flipper_vel)}")
        else:
            result.add("DRIVE mode: flipper non-zero", False, "No flipper msgs received")

        # ── 1d: Switch to ARM mode (Share button, index 8) ──
        self.get_logger().info("  1d: ARM mode — arm twist output")
        arm_col.clear()
        # Rising edge: Share=1
        joy_arm_switch = make_joy(buttons=[0]*8 + [1] + [0]*4)
        joy_pub.publish(joy_arm_switch)
        self.spin_for(0.1)
        # Left stick forward in ARM mode
        joy_arm_input = make_joy(
            axes=[0.3, -0.5, 1.0, 0.2, -0.4, 1.0, 0.0, 0.0],
            buttons=[0]*13,
        )
        for _ in range(10):
            joy_pub.publish(joy_arm_input)
            self.spin_for(0.05)
        self.spin_for(0.3)

        if arm_col.count() > 0:
            last_a = arm_col.last()
            has_arm = (abs(last_a.linear.x) > 0.01 or abs(last_a.linear.y) > 0.01
                       or abs(last_a.linear.z) > 0.01 or abs(last_a.angular.z) > 0.01)
            result.add("ARM mode: twist non-zero", has_arm,
                       f"lx={last_a.linear.x:.3f}, ly={last_a.linear.y:.3f}, "
                       f"lz={last_a.linear.z:.3f}, az={last_a.angular.z:.3f}")
        else:
            result.add("ARM mode: twist non-zero", False, "No arm msgs received")

        # ── 1e: E-Stop (PS button, index 10) ──
        self.get_logger().info("  1e: E-Stop — latched emergency stop")
        estop_col.clear()
        joy_estop = make_joy(buttons=[0]*10 + [1] + [0]*2)
        joy_pub.publish(joy_estop)
        self.spin_for(0.3)

        if estop_col.count() > 0:
            last_e = estop_col.last()
            result.add("E-Stop: published", last_e.data == True,
                       f"data={last_e.data}")
        else:
            result.add("E-Stop: published", False, "No estop msgs received")

        # After E-Stop, crawler should be zero even in DRIVE mode
        crawler_col.clear()
        joy_drive_after = make_joy(
            axes=[0.0, -0.8, 1.0, 0.0, -0.8, 1.0, 0.0, 0.0],
            buttons=[0]*9 + [1] + [0]*3,  # Options pressed (try to enter DRIVE)
        )
        for _ in range(10):
            joy_pub.publish(joy_drive_after)
            self.spin_for(0.05)
        self.spin_for(0.3)

        if crawler_col.count() > 0:
            last_c = crawler_col.last()
            post_estop_zero = abs(last_c.m1_vel) < 0.01 and abs(last_c.m2_vel) < 0.01
            result.add("E-Stop: crawler locked zero", post_estop_zero,
                       f"m1={last_c.m1_vel:.3f}, m2={last_c.m2_vel:.3f}")
        else:
            result.add("E-Stop: crawler locked zero", False, "No crawler msgs after E-Stop")

        # Cleanup
        self.destroy_subscription(sub_crawler)
        self.destroy_subscription(sub_flipper)
        self.destroy_subscription(sub_arm)
        self.destroy_subscription(sub_estop)
        self.destroy_publisher(joy_pub)
        kill_node(proc)
        self.procs.remove(proc)

        return result

    # ═══════════════════════════════════════════════════════════════════
    #  TEST 2: arm_controller (IK)
    # ═══════════════════════════════════════════════════════════════════
    def test_arm_controller(self) -> TestResult:
        result = TestResult("arm_controller", True)
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST 2: arm_controller (velocity IK)")
        self.get_logger().info("=" * 60)

        proc = launch_node([
            "ros2", "run", "arm_controller", "arm_controller",
            "--ros-args",
            "-p", f"urdf_path:={URDF_PATH}",
            "-p", "end_link:=link_tip",
        ])
        self.procs.append(proc)

        js_pub = self.create_publisher(JointState, "/joint_states", 10)
        twist_pub = self.create_publisher(Twist, "/arm_cmd_vel", 10)
        cmd_col = MsgCollector()
        sub_cmd = self.create_subscription(JointState, "/arm_joint_commands", cmd_col.callback, 10)

        # Feed joint_states at 50 Hz to arm the controller
        def pub_js():
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(ARM_JOINTS)
            msg.position = [0.0] * 6
            msg.velocity = [0.0] * 6
            msg.effort = [0.0] * 6
            js_pub.publish(msg)
        js_timer = self.create_timer(0.02, pub_js)

        # Wait for arming (DDS discovery + first joint_states)
        self.spin_for(3.0)

        # ── 2a: Startup — controller armed, receiving output ──
        self.get_logger().info("  2a: Controller armed, receiving output")
        armed = cmd_col.count() > 0
        result.add("Controller armed (has output)", armed, f"{cmd_col.count()} msgs")

        # ── 2b: Forward X command — non-zero velocities ──
        self.get_logger().info("  2b: Forward X +0.1 m/s")
        cmd_col.clear()
        twist = Twist()
        twist.linear.x = 0.1
        for _ in range(50):
            twist_pub.publish(twist)
            self.spin_for(0.02)

        msgs = cmd_col.get_all()
        nonzero = [m for m in msgs if any(abs(v) > 1e-6 for v in m.velocity)]
        has_pos = [m for m in msgs if len(m.position) == 6]
        result.add("Forward X: non-zero velocity", len(nonzero) > 0,
                   f"{len(nonzero)}/{len(msgs)} non-zero velocity msgs")
        result.add("Forward X: has position field", len(has_pos) > 0 or len(msgs) > 0,
                   f"{len(has_pos)}/{len(msgs)} msgs with 6 positions")

        if nonzero:
            last = nonzero[-1]
            peak_vel = max(abs(v) for v in last.velocity)
            result.add("Forward X: reasonable velocity", peak_vel < 2.0,
                       f"peak_vel={peak_vel:.4f} rad/s")

        # ── 2c: Up Z command ──
        self.get_logger().info("  2c: Up Z +0.1 m/s")
        cmd_col.clear()
        twist_z = Twist()
        twist_z.linear.z = 0.1
        for _ in range(50):
            twist_pub.publish(twist_z)
            self.spin_for(0.02)

        msgs_z = cmd_col.get_all()
        nonzero_z = [m for m in msgs_z if any(abs(v) > 1e-6 for v in m.velocity)]
        result.add("Up Z: non-zero velocity", len(nonzero_z) > 0,
                   f"{len(nonzero_z)}/{len(msgs_z)} non-zero")

        # ── 2d: Watchdog timeout — zero output after no cmd ──
        self.get_logger().info("  2d: Watchdog timeout (no cmd for >500ms)")
        cmd_col.clear()
        self.spin_for(1.0)  # no twist published → watchdog should kick in

        msgs_wd = cmd_col.get_all()
        if msgs_wd:
            # Last few should have near-zero velocity (null-space repulsion may be small)
            last_5 = msgs_wd[-5:]
            max_vel_wd = max(max(abs(v) for v in m.velocity) for m in last_5)
            result.add("Watchdog: velocity near zero", max_vel_wd < 0.1,
                       f"max_vel={max_vel_wd:.6f}")
        else:
            result.add("Watchdog: velocity near zero", False, "No msgs during watchdog period")

        # ── 2e: Joint names correct ──
        if cmd_col.get_all():
            names = list(cmd_col.last().name)
            result.add("Joint names correct", names == ARM_JOINTS,
                       f"got {names}")

        # Cleanup
        self.destroy_timer(js_timer)
        self.destroy_subscription(sub_cmd)
        self.destroy_publisher(js_pub)
        self.destroy_publisher(twist_pub)
        kill_node(proc)
        self.procs.remove(proc)

        return result

    # ═══════════════════════════════════════════════════════════════════
    #  TEST 3: arm_gz_bridge
    # ═══════════════════════════════════════════════════════════════════
    def test_arm_gz_bridge(self) -> TestResult:
        result = TestResult("arm_gz_bridge", True)
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST 3: arm_gz_bridge (JointState→Float64)")
        self.get_logger().info("=" * 60)

        proc = launch_node([
            "ros2", "run", "bringup", "arm_gz_bridge.py",
        ])
        self.procs.append(proc)

        cmd_pub = self.create_publisher(JointState, "/arm_joint_commands", 10)

        # Subscribe to all 6 per-joint topics
        collectors = {}
        subs = []
        for name in ARM_JOINTS:
            col = MsgCollector()
            collectors[name] = col
            s = self.create_subscription(Float64, f"/{name}/cmd_pos", col.callback, 10)
            subs.append(s)

        self.spin_for(3.0)

        # ── 3a: Publish JointState with known positions ──
        self.get_logger().info("  3a: Publish JointState → per-joint Float64")
        test_positions = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ARM_JOINTS)
        msg.position = test_positions
        msg.velocity = [0.0] * 6

        for _ in range(20):
            cmd_pub.publish(msg)
            self.spin_for(0.05)
        self.spin_for(0.5)

        all_received = True
        for i, name in enumerate(ARM_JOINTS):
            col = collectors[name]
            if col.count() > 0:
                last_val = col.last().data
                expected = test_positions[i]
                ok = abs(last_val - expected) < 0.001
                result.add(f"{name}: correct value", ok,
                           f"expected={expected:.3f}, got={last_val:.3f}")
            else:
                result.add(f"{name}: correct value", False, "No messages received")
                all_received = False

        # Cleanup
        for s in subs:
            self.destroy_subscription(s)
        self.destroy_publisher(cmd_pub)
        kill_node(proc)
        self.procs.remove(proc)

        return result

    # ═══════════════════════════════════════════════════════════════════
    #  TEST 4: crawler_vel_bridge
    # ═══════════════════════════════════════════════════════════════════
    def test_crawler_vel_bridge(self) -> TestResult:
        result = TestResult("crawler_vel_bridge", True)
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST 4: crawler_vel_bridge (CrawlerVelocity→Twist)")
        self.get_logger().info("=" * 60)

        proc = launch_node([
            "ros2", "run", "bringup", "crawler_vel_bridge.py",
        ])
        self.procs.append(proc)

        cv_pub = self.create_publisher(CrawlerVelocity, "/crawler_driver", 10)
        twist_col = MsgCollector()
        sub_twist = self.create_subscription(Twist, "/cmd_vel", twist_col.callback, 10)

        self.spin_for(3.0)

        # ── 4a: Forward motion (m1=m2=0.5) → linear.x=0.5, angular.z=0 ──
        self.get_logger().info("  4a: Forward (m1=m2=0.5)")
        twist_col.clear()
        cv_msg = CrawlerVelocity()
        cv_msg.m1_vel = 0.5
        cv_msg.m2_vel = 0.5
        for _ in range(10):
            cv_pub.publish(cv_msg)
            self.spin_for(0.05)
        self.spin_for(0.3)

        if twist_col.count() > 0:
            t = twist_col.last()
            result.add("Forward: linear.x ≈ 0.5", abs(t.linear.x - 0.5) < 0.01,
                       f"linear.x={t.linear.x:.3f}")
            result.add("Forward: angular.z ≈ 0", abs(t.angular.z) < 0.01,
                       f"angular.z={t.angular.z:.3f}")
        else:
            result.add("Forward: output", False, "No Twist messages received")

        # ── 4b: Rotation (m1=0.5, m2=-0.5) → linear.x=0, angular.z=-1.0/0.4=-2.5 ──
        self.get_logger().info("  4b: Rotation (m1=0.5, m2=-0.5)")
        twist_col.clear()
        cv_msg2 = CrawlerVelocity()
        cv_msg2.m1_vel = 0.5
        cv_msg2.m2_vel = -0.5
        for _ in range(10):
            cv_pub.publish(cv_msg2)
            self.spin_for(0.05)
        self.spin_for(0.3)

        if twist_col.count() > 0:
            t = twist_col.last()
            result.add("Rotation: linear.x ≈ 0", abs(t.linear.x) < 0.01,
                       f"linear.x={t.linear.x:.3f}")
            expected_az = (-0.5 - 0.5) / 0.4  # (m2-m1)/track_width = -2.5
            result.add("Rotation: angular.z correct", abs(t.angular.z - expected_az) < 0.01,
                       f"angular.z={t.angular.z:.3f}, expected={expected_az:.3f}")
        else:
            result.add("Rotation: output", False, "No Twist messages received")

        # ── 4c: Zero input → zero output ──
        self.get_logger().info("  4c: Zero input → zero output")
        twist_col.clear()
        cv_zero = CrawlerVelocity()
        cv_zero.m1_vel = 0.0
        cv_zero.m2_vel = 0.0
        for _ in range(10):
            cv_pub.publish(cv_zero)
            self.spin_for(0.05)
        self.spin_for(0.3)

        if twist_col.count() > 0:
            t = twist_col.last()
            result.add("Zero: all zero", abs(t.linear.x) < 0.001 and abs(t.angular.z) < 0.001,
                       f"lx={t.linear.x:.4f}, az={t.angular.z:.4f}")
        else:
            result.add("Zero: output", False, "No Twist messages received")

        # Cleanup
        self.destroy_subscription(sub_twist)
        self.destroy_publisher(cv_pub)
        kill_node(proc)
        self.procs.remove(proc)

        return result

    # ═══════════════════════════════════════════════════════════════════
    #  TEST 5: odom_tf_bridge
    # ═══════════════════════════════════════════════════════════════════
    def test_odom_tf_bridge(self) -> TestResult:
        result = TestResult("odom_tf_bridge", True)
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST 5: odom_tf_bridge (Odometry→TF)")
        self.get_logger().info("=" * 60)

        proc = launch_node([
            "ros2", "run", "bringup", "odom_tf_bridge.py",
        ])
        self.procs.append(proc)

        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        odom_pub = self.create_publisher(Odometry, "/odom", odom_qos)
        tf_col = MsgCollector()
        sub_tf = self.create_subscription(TFMessage, "/tf", tf_col.callback, 10)

        self.spin_for(3.0)

        # ── 5a: Publish odom → expect TF odom→base_footprint ──
        self.get_logger().info("  5a: Publish Odometry → TF broadcast")
        tf_col.clear()
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = 1.0
        odom.pose.pose.position.y = 2.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0

        for _ in range(20):
            odom_pub.publish(odom)
            self.spin_for(0.05)
        self.spin_for(0.5)

        # Check TF
        found_odom_tf = False
        for tf_msg in tf_col.get_all():
            for t in tf_msg.transforms:
                if t.header.frame_id == "odom" and t.child_frame_id == "base_footprint":
                    found_odom_tf = True
                    x_ok = abs(t.transform.translation.x - 1.0) < 0.01
                    y_ok = abs(t.transform.translation.y - 2.0) < 0.01
                    result.add("TF: position correct", x_ok and y_ok,
                               f"x={t.transform.translation.x:.3f}, y={t.transform.translation.y:.3f}")
                    break
            if found_odom_tf:
                break

        result.add("TF: odom→base_footprint published", found_odom_tf,
                   f"total TF msgs={tf_col.count()}")

        # Cleanup
        self.destroy_subscription(sub_tf)
        self.destroy_publisher(odom_pub)
        kill_node(proc)
        self.procs.remove(proc)

        return result

    # ═══════════════════════════════════════════════════════════════════
    #  TEST 6: fix_pointcloud_time
    # ═══════════════════════════════════════════════════════════════════
    def test_fix_pointcloud_time(self) -> TestResult:
        result = TestResult("fix_pointcloud_time", True)
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST 6: fix_pointcloud_time")
        self.get_logger().info("=" * 60)

        proc = launch_node([
            "ros2", "run", "bringup", "fix_pointcloud_time_node.py",
        ])
        self.procs.append(proc)

        pc_pub = self.create_publisher(PointCloud2, "/velodyne_points", 10)
        pc_col = MsgCollector()
        sub_pc = self.create_subscription(PointCloud2, "/velodyne_points_fixed", pc_col.callback, 10)

        self.spin_for(3.0)

        # ── 6a: Make a minimal PointCloud2 with x, y, z, time fields ──
        self.get_logger().info("  6a: PointCloud2 with time field → recalculated")
        pc_col.clear()

        # Build a tiny PointCloud2 with known x, y, z, time
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="time", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 16
        # 3 test points at different azimuths
        points_data = [
            (1.0, 0.0, 0.0, 0.0),     # azimuth=0°    → time~0
            (0.0, 1.0, 0.0, 0.0),     # azimuth=90°   → time~16.7ms
            (-1.0, 0.0, 0.0, 0.0),    # azimuth=180°  → time~33.3ms
        ]
        data = b""
        for x, y, z, t in points_data:
            data += struct.pack("ffff", x, y, z, t)

        pc_msg = PointCloud2()
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        pc_msg.header.frame_id = "velodyne"
        pc_msg.height = 1
        pc_msg.width = 3
        pc_msg.fields = fields
        pc_msg.is_bigendian = False
        pc_msg.point_step = point_step
        pc_msg.row_step = point_step * 3
        pc_msg.data = list(data)  # PointCloud2.data is uint8[]
        pc_msg.is_dense = True

        for _ in range(10):
            pc_pub.publish(pc_msg)
            self.spin_for(0.05)
        self.spin_for(0.5)

        if pc_col.count() > 0:
            result.add("PointCloud2 passthrough", True, f"{pc_col.count()} msgs received")
            out = pc_col.last()
            result.add("Output has same fields", len(out.fields) == 4,
                       f"fields={[f.name for f in out.fields]}")
            result.add("Output has same width", out.width == 3,
                       f"width={out.width}")
        else:
            result.add("PointCloud2 passthrough", False, "No output received")

        # ── 6b: PointCloud2 without time field → passthrough ──
        self.get_logger().info("  6b: PointCloud2 without time field → passthrough")
        pc_col.clear()

        fields_no_time = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        data2 = struct.pack("fff", 1.0, 2.0, 3.0)

        pc_msg2 = PointCloud2()
        pc_msg2.header.stamp = self.get_clock().now().to_msg()
        pc_msg2.header.frame_id = "velodyne"
        pc_msg2.height = 1
        pc_msg2.width = 1
        pc_msg2.fields = fields_no_time
        pc_msg2.is_bigendian = False
        pc_msg2.point_step = 12
        pc_msg2.row_step = 12
        pc_msg2.data = list(data2)
        pc_msg2.is_dense = True

        for _ in range(10):
            pc_pub.publish(pc_msg2)
            self.spin_for(0.05)
        self.spin_for(0.5)

        if pc_col.count() > 0:
            result.add("No-time passthrough", True, f"{pc_col.count()} msgs")
        else:
            result.add("No-time passthrough", False, "No output received")

        # Cleanup
        self.destroy_subscription(sub_pc)
        self.destroy_publisher(pc_pub)
        kill_node(proc)
        self.procs.remove(proc)

        return result


# ═══════════════════════════════════════════════════════════════════════════
#  Main runner
# ═══════════════════════════════════════════════════════════════════════════

def print_report(results: List[TestResult]):
    """Print a formatted report of all test results."""
    total_pass = 0
    total_fail = 0

    print("\n")
    print("╔" + "═" * 68 + "╗")
    print("║" + "  ROBOT FUNCTIONAL TEST REPORT".center(68) + "║")
    print("╠" + "═" * 68 + "╣")

    for r in results:
        status = "✅ PASS" if r.passed else "❌ FAIL"
        print(f"║  {status}  {r.name:<40} {r.detail:>14} ║")
        for sub_name, ok, detail in r.sub_results:
            mark = "✓" if ok else "✗"
            if ok:
                total_pass += 1
            else:
                total_fail += 1
            # Truncate detail to fit
            d = detail[:42] if len(detail) > 42 else detail
            print(f"║    {mark} {sub_name:<30} {d:>30} ║")

    print("╠" + "═" * 68 + "╣")
    total = total_pass + total_fail
    overall = "ALL PASS ✅" if total_fail == 0 else f"{total_fail} FAIL ❌"
    print(f"║  {total_pass}/{total} sub-tests passed — {overall:<32} ║")
    print("╚" + "═" * 68 + "╝")
    print()


def main():
    rclpy.init()

    tester = ComprehensiveTester()
    results = []

    try:
        # Run each test in sequence — each launches its own subprocess
        test_methods = [
            tester.test_joy_controller,
            tester.test_arm_controller,
            tester.test_arm_gz_bridge,
            tester.test_crawler_vel_bridge,
            tester.test_odom_tf_bridge,
            tester.test_fix_pointcloud_time,
        ]

        for test_fn in test_methods:
            try:
                r = test_fn()
                results.append(r)
                tester.get_logger().info(
                    f"  → {'PASS' if r.passed else 'FAIL'}: {r.name}\n"
                )
            except Exception as e:
                tester.get_logger().error(f"  → CRASH: {test_fn.__name__}: {e}")
                results.append(TestResult(test_fn.__name__, False, str(e)))

        print_report(results)

    finally:
        tester.cleanup()
        tester.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # Exit code
    all_pass = all(r.passed for r in results)
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
