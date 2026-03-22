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

        # POI export parameters (RoboCup-usable: heat_sig only)
        self.declare_parameter('enable_poi_export', True)
        self.declare_parameter('team_name', 'Team')
        self.declare_parameter('country', 'Country')
        self.declare_parameter('heat_threshold_c', 40.0)
        enable_poi = self.get_parameter('enable_poi_export').get_parameter_value().bool_value
        team_name = self.get_parameter('team_name').get_parameter_value().string_value
        country = self.get_parameter('country').get_parameter_value().string_value
        self.heat_threshold = float(self.get_parameter('heat_threshold_c').get_parameter_value().double_value)
        if enable_poi:
            try:
                from .poi_writer import POIWriter
                self.poi = POIWriter(out_dir='docs/outputs', team_name=team_name, country=country)
                self.get_logger().info(f'POI export enabled, heat threshold={self.heat_threshold}C')
            except Exception:
                self.poi = None
        else:
            self.poi = None

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
            temp_c = data.get('temperature', 0) / 100.0
            hum_val = data.get('humidity', 0) / 100.0
            light_val = float(data.get('light', 0))
            press_val = data.get('pressure', 0) / 1000.0
            temp.data = temp_c
            hum.data = hum_val
            light.data = light_val
            press.data = press_val
            self.temp_pub.publish(temp)
            self.hum_pub.publish(hum)
            self.light_pub.publish(light)
            self.press_pub.publish(press)

            # RoboCup-usable detection: heat_sig (threshold-based)
            if self.poi is not None and temp_c >= self.heat_threshold:
                path = self.poi.add_heat_detection(name='0', x=0.0, y=0.0, z=0.0, robot='robot1', mode='A')
                self.get_logger().info(f'Heat detection logged to {path} (temp={temp_c:.2f}C)')
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
