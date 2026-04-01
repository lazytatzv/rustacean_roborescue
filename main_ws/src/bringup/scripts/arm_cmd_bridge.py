#!/usr/bin/env python3
"""Bridge: /arm_joint_commands (JointState) → /forward_position_controller/commands (Float64MultiArray)

arm_backend:=ros2_control 時のみ起動される。
arm_controller (Rust IK) の出力をそのまま forward_position_controller に流す。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINT_ORDER = [
    "arm_joint1",
    "arm_joint2",
    "arm_joint3",
    "arm_joint4",
    "arm_joint5",
    "arm_joint6",
]


class ArmCmdBridge(Node):
    def __init__(self):
        super().__init__("arm_cmd_bridge")
        self.pub = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10
        )
        self.sub = self.create_subscription(JointState, "/arm_joint_commands", self._on_cmd, 10)

    def _on_cmd(self, msg: JointState):
        pos_map = dict(zip(msg.name, msg.position))
        try:
            positions = [pos_map[j] for j in JOINT_ORDER]
        except KeyError as e:
            self.get_logger().warn(f"Missing joint in /arm_joint_commands: {e}")
            return
        out = Float64MultiArray()
        out.data = positions
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ArmCmdBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
