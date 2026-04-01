# bringup

`bringup` is the integration package for Rustacean RoboRescue. It holds the launch files, configuration, URDF/Xacro, Gazebo worlds, and helper scripts that glue the rest of the workspace together.

If you are looking for the high-level system diagram, start with [docs/ARCHITECTURE.md](../../../docs/ARCHITECTURE.md). If you need the runbook, use [docs/OPERATION.md](../../../docs/OPERATION.md).

## Launch Files

| Launch file | Role |
|-------------|------|
| `system.launch.py` | Real robot bringup: drivers, camera, audio, control, optional Nav2 |
| `simulation.launch.py` | Gazebo simulation, SLAM, and optional Nav2 |
| `camera.launch.py` | Camera drivers, FFmpeg transport, and QR detection |
| `audio.launch.py` | Opus-based audio bridge |
| `control.launch.py` | Joystick control and arm command wiring |
| `nav2.launch.py` | Navigation stack only |
| `network.launch.py` | Zenoh configuration and transport setup |
| `perception.launch.py` | Perception stack entry point |
| `display.launch.py` | RViz and visualization helpers |
| `test_arm.launch.py` | Arm bringup and smoke test |
| `test_ik.launch.py` | IK verification launch |

## Configuration Files

| File | Purpose |
|------|---------|
| `config/joy_controller.yaml` | Joystick limits, deadzones, and mode parameters |
| `config/crawler_driver.yaml` | Roboclaw serial and PID settings |
| `config/flipper_driver.yaml` | Dynamixel flipper parameters |
| `config/arm_controller.yaml` | Arm IK / FK limits and gains |
| `config/arm_gripper_driver.yaml` | Arm and gripper driver parameters |
| `config/sensor_gateway.yaml` | IMU bridge parameters |
| `config/cameras.yaml` | Camera topology used by `camera.launch.py` |
| `config/nav2_params.yaml` | Nav2 planner / controller configuration |
| `config/slam_toolbox_async.yaml` | SLAM Toolbox parameters for sim and real robot |
| `config/pointcloud_to_laserscan.yaml` | PointCloud2 to LaserScan conversion settings |
| `config/fix_pointcloud_time.yaml` | Timestamp repair helper for point clouds |
| `config/spark_fast_lio_min.yaml` | Minimal LiDAR-IMU odometry settings |
| `config/velodyne_driver.yaml` | LiDAR driver parameters |
| `config/velodyne_pointcloud.yaml` | Velodyne pointcloud conversion settings |
| `config/zenoh_robot.json5` | Robot-side Zenoh client configuration |
| `config/zenoh_router.json5` | Robot-side Zenoh router configuration |

## Robot Models and Worlds

| File | Purpose |
|------|---------|
| `urdf/robot.urdf.xacro` | Simulation robot description and Gazebo sensor setup |
| `urdf/sekirei.urdf` | Arm IK / visualization model |
| `worlds/rescue_field.sdf` | Gazebo rescue field world |
| `worlds/rescue_field_headless.sdf` | Headless Gazebo world for CI and software rendering |
| `worlds/rescue_field.world` | Convenience world wrapper |

## Typical Commands

```bash
cd main_ws
source install/setup.bash

# Real robot
ros2 launch bringup system.launch.py

# Real robot with Nav2
ros2 launch bringup system.launch.py use_nav2:=true

# Simulation with SLAM
ros2 launch bringup simulation.launch.py use_slam:=true

# Simulation with Nav2
ros2 launch bringup simulation.launch.py use_slam:=true use_nav2:=true

# Camera and QR only
ros2 launch bringup camera.launch.py
```

## Notes

- The package is intentionally mostly configuration and launch code; the runtime logic lives in the smaller driver and perception packages.
- Keep this package in sync with the top-level docs when launch arguments or topic names change.
