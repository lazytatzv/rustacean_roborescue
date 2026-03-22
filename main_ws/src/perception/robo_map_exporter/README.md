# robo_map_exporter

PoC ROS2 node to export mapping results to PLY and detection CSV (POI) for RoboCup submissions.

Usage (after colcon build):

ros2 run robo_map_exporter robo_map_exporter_node

This is a scaffold; next steps: subscribe to SLAM/map topics, implement PLY writer and CSV writer matching competition formats.
