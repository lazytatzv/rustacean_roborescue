# omron_bridge

Bridge node to publish OMRON 2JCIE-BU01 sensor readings to ROS2 topics.

Usage (after colcon build):

ros2 run omron_bridge omron_bridge_node --ros-args -p serial_port:=/dev/ttyUSB0 -p poll_period:=1.0

It uses the `main_ws/src/external/omron-2jcie-bu01` submodule when available, or the
`main_ws/third_party/omron-2jcie-bu01` location if the project keeps the library vendored there.

RoboCup usage (implemented):
- POI CSV export for `heat_sig` detections when temperature >= `heat_threshold_c` (default 40.0°C).
- Output file is written to `docs/outputs/RoboCup<year>-<Team>-Mission-<time>-pois.csv`.

Parameters:
- `enable_poi_export` (bool, default true)
- `team_name` (string)
- `country` (string)
- `heat_threshold_c` (float)

