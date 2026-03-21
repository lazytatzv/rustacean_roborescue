#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys, select, termios, tty
import threading

msg = """
Robot Joint Teleop (Virtual)
---------------------------
Flippers: 1/2(FL), 3/4(FR), 5/6(BL), 7/8(BR)
Arm: Q/A(J1), W/S(J2), E/D(J3), R/F(J4)
CTRL-C to quit
"""

class TeleopJoints(Node):
    def __init__(self):
        super().__init__('teleop_joints')
        # 明示的に絶対パスを指定
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.joint_names = [
            'flipper_fl_joint', 'flipper_fr_joint', 'flipper_bl_joint', 'flipper_br_joint',
            'arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5', 'arm_joint6'
        ]
        self.pos = [0.0] * 10
        self.timer = self.create_timer(0.05, self.publish_joint_state)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_joint_state(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = self.pos
        self.pub.publish(js)

def main():
    rclpy.init()
    node = TeleopJoints()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(msg)
    try:
        while rclpy.ok():
            key = node.getKey()
            if key == '1': node.pos[0] += 0.1
            elif key == '2': node.pos[0] -= 0.1
            elif key == '3': node.pos[1] += 0.1
            elif key == '4': node.pos[1] -= 0.1
            elif key == '5': node.pos[2] += 0.1
            elif key == '6': node.pos[2] -= 0.1
            elif key == '7': node.pos[3] += 0.1
            elif key == '8': node.pos[3] -= 0.1
            elif key.upper() == 'Q': node.pos[4] += 0.1
            elif key.upper() == 'A': node.pos[4] -= 0.1
            elif key.upper() == 'W': node.pos[5] += 0.1
            elif key.upper() == 'S': node.pos[5] -= 0.1
            elif key.upper() == 'E': node.pos[6] += 0.1
            elif key.upper() == 'D': node.pos[6] -= 0.1
            elif key.upper() == 'R': node.pos[7] += 0.1
            elif key.upper() == 'F': node.pos[7] -= 0.1
            elif key == '\x03': break
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
