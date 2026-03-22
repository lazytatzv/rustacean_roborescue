import os
import sys
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

def locate_submodule():
    # Search upward for a 'src/external/omron-2jcie-bu01' folder and add to sys.path
    p = Path(__file__).resolve()
    for parent in p.parents:
        candidate = parent / 'src' / 'external' / 'omron-2jcie-bu01'
        if candidate.exists():
            sys.path.insert(0, str(candidate))
            return True
    return False

if not locate_submodule():
    # fallback: rely on installed package
    pass

try:
    from omron_2jcie_bu01 import Omron2JCIE_BU01
except Exception as e:
    Omron2JCIE_BU01 = None

class OmronBridge(Node):
    def __init__(self):
        super().__init__('omron_bridge')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('poll_period', 1.0)
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        period = float(self.get_parameter('poll_period').get_parameter_value().double_value)

        self.temp_pub = self.create_publisher(Float32, 'omron/temperature', 10)
        self.hum_pub = self.create_publisher(Float32, 'omron/humidity', 10)
        self.light_pub = self.create_publisher(Float32, 'omron/light', 10)
        self.press_pub = self.create_publisher(Float32, 'omron/pressure', 10)

        self.sensor = None
        if Omron2JCIE_BU01 is not None:
            try:
                self.sensor = Omron2JCIE_BU01.serial(port)
                self.get_logger().info(f'Connected to Omron sensor on {port}')
            except Exception as e:
                self.get_logger().warn(f'Failed to open Omron sensor on {port}: {e}')
        else:
            self.get_logger().warn('omron_2jcie_bu01 module not available')

        self.timer = self.create_timer(period, self.timer_cb)

    def timer_cb(self):
        if self.sensor is None:
            return
        try:
            data = self.sensor.latest_data_long()
            # data keys: temperature (0.01 degC), humidity (0.01 %RH), light (lx), pressure (0.001 hPa)
            temp = Float32()
            hum = Float32()
            light = Float32()
            press = Float32()
            temp.data = data.get('temperature', 0) / 100.0
            hum.data = data.get('humidity', 0) / 100.0
            light.data = float(data.get('light', 0))
            press.data = data.get('pressure', 0) / 1000.0
            self.temp_pub.publish(temp)
            self.hum_pub.publish(hum)
            self.light_pub.publish(light)
            self.press_pub.publish(press)
        except Exception as e:
            self.get_logger().error(f'Error reading sensor: {e}')

def main(argv=None):
    rclpy.init(args=argv)
    node = OmronBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
