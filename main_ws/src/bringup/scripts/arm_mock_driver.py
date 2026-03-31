#!/usr/bin/env python3
"""arm_mock_driver: IKテスト用モックドライバ.

/arm_joint_commands (JointState) と /gripper_controller/commands (Float64MultiArray) を
受け取り /joint_states にフィードバックする。
実機なしで arm_controller + グリッパーの動作確認に使う。
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

ARM_JOINTS = [
    "arm_joint1",
    "arm_joint2",
    "arm_joint3",
    "arm_joint4",
    "arm_joint5",
    "arm_joint6",
]
FLIPPER_JOINTS = [
    "flipper_fl_joint",
    "flipper_fr_joint",
    "flipper_bl_joint",
    "flipper_br_joint",
]
GRIPPER_JOINTS = ["gripper_joint7", "gripper_joint8"]
ALL_JOINTS = FLIPPER_JOINTS + ARM_JOINTS + GRIPPER_JOINTS


class ArmMockDriver(Node):
    def __init__(self):
        super().__init__("arm_mock_driver")
        self._positions = {j: 0.0 for j in ALL_JOINTS}

        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, "/arm_joint_commands", self._on_arm_cmd, 10)
        self.create_subscription(
            Float64MultiArray, "/gripper_controller/commands", self._on_gripper_cmd, 10
        )
        self.create_timer(0.02, self._publish)  # 50 Hz

    def _on_arm_cmd(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self._positions:
                self._positions[name] = pos

    def _on_gripper_cmd(self, msg: Float64MultiArray):
        if len(msg.data) >= 2:
            self._positions["gripper_joint7"] = msg.data[0]
            self._positions["gripper_joint8"] = msg.data[1]

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ALL_JOINTS
        msg.position = [self._positions[j] for j in ALL_JOINTS]
        self._pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(ArmMockDriver())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
