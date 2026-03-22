import rclpy
from rclpy.node import Node


class RoboMapExporter(Node):
    def __init__(self):
        super().__init__("robo_map_exporter")
        self.get_logger().info("robo_map_exporter node started (PoC)")
        # Placeholder: subscribe to SLAM/map topics and implement PLY/CSV export


def main(argv=None):
    rclpy.init(args=argv)
    node = RoboMapExporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
