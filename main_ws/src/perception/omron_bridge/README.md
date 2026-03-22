# omron_bridge

Bridge node to publish OMRON 2JCIE-BU01 sensor readings to ROS2 topics.

Usage (after colcon build):

ros2 run omron_bridge omron_bridge_node --ros-args -p serial_port:=/dev/ttyUSB0 -p poll_period:=1.0

It uses the `main_ws/src/external/omron-2jcie-bu01` submodule when available.
