#!/usr/bin/env python3
"""arm_gz_bridge: JointState → per-joint Float64 splitter for Gazebo simulation.

Gazebo Harmonic's JointPositionController expects individual position commands
on per-joint topics:  arm_joint{1-6}/cmd_pos  (gz.msgs.Double)

ros_gz_bridge converts:  /arm_joint{1-6}/cmd_pos (std_msgs/Float64) → gz transport

This node subscribes to /arm_joint_commands (sensor_msgs/JointState) from
arm_controller, extracts position[i], and publishes to each per-joint topic.

  /arm_joint_commands (JointState.position)
      │
      ├→ /arm_joint1/cmd_pos (Float64)
      ├→ /arm_joint2/cmd_pos (Float64)
      ├→ /arm_joint3/cmd_pos (Float64)
      ├→ /arm_joint4/cmd_pos (Float64)
      ├→ /arm_joint5/cmd_pos (Float64)
      └→ /arm_joint6/cmd_pos (Float64)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# Joint names must match the URDF and arm_controller output
ARM_JOINTS = [
    "arm_joint1",
    "arm_joint2",
    "arm_joint3",
    "arm_joint4",
    "arm_joint5",
    "arm_joint6",
]


class ArmGzBridge(Node):
    def __init__(self) -> None:
        super().__init__("arm_gz_bridge")

        # Create a publisher for each arm joint
        self.pubs_ = {}
        for name in ARM_JOINTS:
            topic = f"/{name}/cmd_pos"
            self.pubs_[name] = self.create_publisher(Float64, topic, 10)
            self.get_logger().info(f"  Publishing: {topic}")

        # Subscribe to arm_controller output
        self.sub_ = self.create_subscription(
            JointState, "/arm_joint_commands", self.callback, 10
        )

        self.get_logger().info(
            "arm_gz_bridge started: /arm_joint_commands → per-joint cmd_pos"
        )

    def callback(self, msg: JointState) -> None:
        # Build name→position map from incoming message
        pos_map = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                pos_map[name] = msg.position[i]

        # Publish to each joint topic
        for name in ARM_JOINTS:
            if name in pos_map:
                out = Float64()
                out.data = pos_map[name]
                self.pubs_[name].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ArmGzBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
