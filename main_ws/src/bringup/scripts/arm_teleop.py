#!/usr/bin/env python3
"""アーム Cartesian + グリッパー テレオペ.

  W / S  : 前 / 後  (X)
  A / D  : 左 / 右  (Y)
  R / F  : 上 / 下  (Z)
  O      : グリッパー 開
  C      : グリッパー 閉
  Space  : アーム停止
  Ctrl-C : 終了
"""
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

HELP = """
アーム操作
──────────────────────────
  W / S  : 前 / 後
  A / D  : 左 / 右
  R / F  : 上 / 下
  O      : グリッパー 開
  C      : グリッパー 閉
  Space  : 停止
  Ctrl-C : 終了
──────────────────────────
"""

SPEED = 0.05  # m/s
GRIPPER_OPEN = [0.9, -0.9]
GRIPPER_CLOSE = [0.0, 0.0]


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rclpy.init()
    node = Node("arm_teleop")
    twist_pub = node.create_publisher(Twist, "/arm_cmd_vel", 10)
    gripper_pub = node.create_publisher(Float64MultiArray, "/gripper_controller/commands", 10)

    settings = termios.tcgetattr(sys.stdin)
    print(HELP)

    try:
        while True:
            key = get_key(settings).lower()
            twist = Twist()
            if key == "w":
                twist.linear.x = SPEED
            elif key == "s":
                twist.linear.x = -SPEED
            elif key == "a":
                twist.linear.y = SPEED
            elif key == "d":
                twist.linear.y = -SPEED
            elif key == "r":
                twist.linear.z = SPEED
            elif key == "f":
                twist.linear.z = -SPEED
            elif key == "o":
                msg = Float64MultiArray()
                msg.data = GRIPPER_OPEN
                gripper_pub.publish(msg)
                continue
            elif key == "c":
                msg = Float64MultiArray()
                msg.data = GRIPPER_CLOSE
                gripper_pub.publish(msg)
                continue
            elif key == " ":
                pass  # 全ゼロ = 停止
            elif key == "\x03":
                break
            else:
                continue
            twist_pub.publish(twist)
    finally:
        twist_pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
