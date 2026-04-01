# Workspace Package Guide

This page is the fast map for the ROS 2 workspace under `main_ws/src`. It answers two questions:

1. What is a project-owned package?
2. Where should I look for launch files, drivers, or support code?

If you only need a quick orientation, read this page together with [main_ws/README.md](../main_ws/README.md) and [docs/ARCHITECTURE.md](ARCHITECTURE.md).

## Top-Level Groups

| Group | What it contains |
|------|-------------------|
| `audio_bridge/` | Bidirectional audio transport over ROS 2 topics using GStreamer + Opus |
| `bringup/` | Launch files, parameters, URDF/Xacro, Gazebo worlds, and bridge helpers |
| `control/` | Operator-facing control logic such as joystick processing and arm IK |
| `custom_interfaces/` | Project-specific ROS message definitions |
| `extras/` | Small utilities that support camera/debug workflows |
| `hardware_drivers/` | Real robot hardware drivers for crawler, flipper, arm, and IMU bridge |
| `perception/` | QR detection and environmental sensor bridges |
| `external/` | Vendored third-party ROS packages and interface definitions |

## Runtime Packages

| Package | Language | Role | Main entry points |
|---------|----------|------|-------------------|
| `audio_bridge` | Python | Encodes microphone input to Opus and decodes operator audio back to speakers | `audio_sender`, `audio_receiver`, `bringup/launch/audio.launch.py` |
| `bringup` | Python + XML + YAML | Central launch/config package for real robot and simulation | `system.launch.py`, `simulation.launch.py`, `camera.launch.py`, `control.launch.py`, `nav2.launch.py`, `network.launch.py`, `audio.launch.py`, `display.launch.py` |
| `joy_controller` | C++ | Maps joystick input to crawler, flipper, and arm commands with STOP/DRIVE/ARM modes | `bringup/launch/control.launch.py` |
| `arm_controller` | Rust | Jacobian-based arm controller with DLS IK/FK and safety limits | `bringup/launch/control.launch.py`, `bringup/launch/test_ik.launch.py` |
| `custom_interfaces` | Interface defs | Project-specific messages such as crawler, flipper, gripper, and audio payloads | Imported by the drivers and launch stack |
| `h264_republisher` | C++ | Decodes `/camera/image_raw/ffmpeg` into raw `sensor_msgs/Image` for tools that expect uncompressed frames | `ros2 run h264_republisher h264_republisher` |
| `arm_driver` | Rust | Dynamixel position-control driver for the 6-DOF arm and gripper | `system.launch.py` |
| `crawler_driver` | C++ | Roboclaw differential-drive driver for the crawler tracks | `system.launch.py` |
| `flipper_driver` | C++ | Dynamixel flipper driver for the four flippers | `system.launch.py` |
| `sensor_gateway` | Rust | Serial parser for the STM32 IMU bridge | `system.launch.py` |
| `omron_bridge` | Python | Publishes OMRON 2JCIE-BU01 environmental sensor readings to ROS topics | Standalone CLI and optional POI export |
| `qr_detector` | Python | Python QR detector built around OpenCV WeChatQRCode | `camera.launch.py` |
| `qr_detector_cpp` | C++ | Composable QR detector for the camera pipeline | `camera.launch.py` |

## Bringup Assets

The `bringup` package is the integration hub. It contains the files that decide how the rest of the workspace is wired together:

| Asset type | Examples |
|------------|----------|
| Launch files | `system.launch.py`, `simulation.launch.py`, `camera.launch.py`, `audio.launch.py`, `control.launch.py`, `nav2.launch.py`, `network.launch.py`, `perception.launch.py`, `display.launch.py` |
| Configuration | `nav2_params.yaml`, `slam_toolbox_async.yaml`, `pointcloud_to_laserscan.yaml`, `velodyne_driver.yaml`, `velodyne_pointcloud.yaml`, `fix_pointcloud_time.yaml`, `joy_controller.yaml`, `crawler_driver.yaml`, `flipper_driver.yaml`, `arm_controller.yaml`, `sensor_gateway.yaml`, `cameras.yaml`, `zenoh_robot.json5`, `zenoh_router.json5` |
| Robot models | `urdf/robot.urdf.xacro`, `urdf/sekirei.urdf` |
| Gazebo worlds | `worlds/rescue_field.sdf`, `worlds/rescue_field_headless.sdf`, `worlds/rescue_field.world` |

## Vendored Dependencies

Everything under `main_ws/src/external/` is third-party code or ROS interface packages that are checked in so the workspace can be built in a reproducible way. The important project-facing ones are:

| Subtree | Role |
|---------|------|
| `external/ros2_rust/rclrs` | Rust ROS 2 client library |
| `external/rosidl_rust/rosidl_generator_rs` | Rust message generator |
| `external/rosidl_defaults` | Shared ROS interface defaults |
| `external/common_interfaces` | Standard ROS message packages such as `geometry_msgs` and `sensor_msgs` |
| `external/rcl_interfaces` | ROS core interface packages |
| `external/spark-fast-lio` | LiDAR-IMU odometry stack |
| `external/dynamixel_hardware_interface` | ros2_control hardware interface for Dynamixel devices |
| `external/kiss-icp` | KISS-ICP dependency tree |
| `external/unique_identifier_msgs` | Shared identifier message definitions |
| `external/test_interface_files` | Test interface definitions used by the Rust bindings |

## Where To Start

| Need | Read this |
|------|-----------|
| Build or run the system | [docs/OPERATION.md](OPERATION.md) |
| Understand the node graph | [docs/ARCHITECTURE.md](ARCHITECTURE.md) |
| Get a package inventory | This page |
| Learn the dev shell and toolchain | [docs/NIX.md](NIX.md) |
| Get a first-pass setup | [docs/DEV_QUICKSTART.md](DEV_QUICKSTART.md) |
