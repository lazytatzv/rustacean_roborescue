#!/usr/bin/env python3
"""
arm_controller IK integration test.

This script:
  1. Launches the arm_controller node via subprocess
  2. Publishes fake /joint_states (all zeros) to arm the controller
  3. Sends test Twist commands to /arm_cmd_vel
  4. Subscribes to /arm_joint_commands and checks for non-zero output
"""

import subprocess
import sys
import time
import signal
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


ARM_JOINTS = [
    "arm_joint1",
    "arm_joint2",
    "arm_joint3",
    "arm_joint4",
    "arm_joint5",
    "arm_joint6",
]

URDF_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "src",
    "bringup",
    "urdf",
    "sekirei.urdf",
)


class ArmTester(Node):
    def __init__(self):
        super().__init__("arm_tester")

        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.twist_pub = self.create_publisher(Twist, "/arm_cmd_vel", 10)
        self.results = []
        self.armed = False
        self.test_phase = 0
        self.phase_start = None

        self.sub = self.create_subscription(
            JointState, "/arm_joint_commands", self._on_result, 10
        )

        # Publish joint_states at 50 Hz to keep the controller fed
        self.js_timer = self.create_timer(0.02, self._pub_joint_states)

        # Test state machine at 1 Hz
        self.test_timer = self.create_timer(1.0, self._test_tick)

        self.tests = [
            ("ARMING (zero twist)", Twist()),  # just wait for arm
            ("Forward X +0.1", self._make_twist(lx=0.1)),
            ("Up Z +0.1", self._make_twist(lz=0.1)),
            ("Yaw +0.5 rad/s", self._make_twist(az=0.5)),
            ("Combined X+Z", self._make_twist(lx=0.05, lz=0.05)),
            ("Watchdog (no cmd)", None),  # don't publish, wait for watchdog
        ]
        self.get_logger().info(f"URDF: {URDF_PATH}")
        self.get_logger().info(f"Test plan: {len(self.tests)} phases")

    def _make_twist(self, lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0):
        t = Twist()
        t.linear.x = lx
        t.linear.y = ly
        t.linear.z = lz
        t.angular.x = ax
        t.angular.y = ay
        t.angular.z = az
        return t

    def _pub_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = list(ARM_JOINTS)
        msg.position = [0.0] * 6
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6
        self.js_pub.publish(msg)

    def _on_result(self, msg: JointState):
        vel = list(msg.velocity)
        pos = list(msg.position)
        has_nonzero_vel = any(abs(v) > 1e-9 for v in vel)
        has_nonzero_pos = any(abs(p) > 1e-9 for p in pos)
        max_vel = max((abs(v) for v in vel), default=0.0)
        max_pos = max((abs(p) for p in pos), default=0.0)
        self.results.append(
            {
                "phase": self.test_phase,
                "vel": vel,
                "pos": pos,
                "nonzero_vel": has_nonzero_vel,
                "nonzero_pos": has_nonzero_pos,
                "max_vel": max_vel,
                "max_pos": max_pos,
                "names": list(msg.name),
            }
        )

    def _test_tick(self):
        if self.test_phase >= len(self.tests):
            self._print_summary()
            raise SystemExit(0)

        name, twist = self.tests[self.test_phase]

        if self.phase_start is None:
            self.get_logger().info(f"--- Phase {self.test_phase}: {name} ---")
            self.phase_start = time.monotonic()

        # Publish twist if we have one (skip for watchdog test)
        if twist is not None:
            self.twist_pub.publish(twist)

        # Wait 1.5s per phase (this tick fires every 1s, so ~2 ticks)
        elapsed = time.monotonic() - self.phase_start
        if elapsed >= 1.5:
            # Summarise this phase
            phase_results = [r for r in self.results if r["phase"] == self.test_phase]
            n = len(phase_results)
            n_nonzero_vel = sum(1 for r in phase_results if r["nonzero_vel"])
            n_nonzero_pos = sum(1 for r in phase_results if r["nonzero_pos"])

            if phase_results:
                last = phase_results[-1]
                best_vel = max(r["max_vel"] for r in phase_results)
                best_pos = max(r["max_pos"] for r in phase_results)
                self.get_logger().info(
                    f"  Phase {self.test_phase} '{name}': "
                    f"{n} msgs, {n_nonzero_vel} with non-zero vel, "
                    f"{n_nonzero_pos} with non-zero pos"
                )
                self.get_logger().info(
                    f"  Peak |vel|: {best_vel:.6f}, Peak |pos|: {best_pos:.6f}"
                )
                self.get_logger().info(
                    f"  Last vel: {[f'{v:.6f}' for v in last['vel']]}"
                )
                self.get_logger().info(
                    f"  Last pos: {[f'{p:.6f}' for p in last['pos']]}"
                )
                self.get_logger().info(f"  Names: {last['names']}")
            else:
                self.get_logger().warn(
                    f"  Phase {self.test_phase} '{name}': NO messages received!"
                )

            self.test_phase += 1
            self.phase_start = None

    def _print_summary(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info("  ARM CONTROLLER TEST SUMMARY")
        self.get_logger().info("=" * 60)
        pass_count = 0
        fail_count = 0
        for i, (name, twist) in enumerate(self.tests):
            phase_results = [r for r in self.results if r["phase"] == i]
            n_msgs = len(phase_results)
            n_nonzero = sum(1 for r in phase_results if r["nonzero_vel"])

            if i == 0:
                # Arming phase: expect zero velocities (controller armed but no command)
                ok = n_msgs > 0
                status = "PASS" if ok else "FAIL"
            elif i == len(self.tests) - 1:
                # Watchdog phase: expect zero velocities (no command sent)
                ok = n_msgs > 0 and n_nonzero == 0
                status = "PASS" if ok else "FAIL"
            else:
                # Active phase: expect non-zero velocities
                ok = n_nonzero > 0
                status = "PASS" if ok else "FAIL"

            if ok:
                pass_count += 1
            else:
                fail_count += 1

            self.get_logger().info(
                f"  [{status}] Phase {i}: {name} ({n_msgs} msgs, {n_nonzero} non-zero)"
            )

        self.get_logger().info("-" * 60)
        self.get_logger().info(f"  Result: {pass_count} PASS, {fail_count} FAIL")
        self.get_logger().info("=" * 60)


def main():
    rclpy.init()

    # Launch arm_controller as a subprocess
    arm_cmd = [
        "ros2",
        "run",
        "arm_controller",
        "arm_controller",
        "--ros-args",
        "-p",
        f"urdf_path:={URDF_PATH}",
        "-p",
        "end_link:=link_tip",
    ]
    print(f"Starting arm_controller: {' '.join(arm_cmd)}")
    arm_proc = subprocess.Popen(
        arm_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
    )

    # Wait for arm_controller to initialize
    time.sleep(2.0)

    tester = ArmTester()
    try:
        rclpy.spin(tester)
    except SystemExit:
        pass
    finally:
        tester.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        arm_proc.send_signal(signal.SIGINT)
        try:
            stdout, _ = arm_proc.communicate(timeout=5)
            if stdout:
                print("\n=== arm_controller subprocess output ===")
                print(stdout.decode(errors="replace"))
        except subprocess.TimeoutExpired:
            arm_proc.kill()


if __name__ == "__main__":
    main()
