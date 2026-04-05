# Package Inventory

This document lists all active packages in the `main_ws` and their primary roles.

## Core Workspaces

| Workspace | Description |
|-----------|-------------|
| `main_ws` | Primary ROS 2 workspace running on the robot (NUC). |
| `operator_ws` | Lightweight workspace for the operator PC (Joy node, Foxglove bridge). |
| `stm32_ws` | Embedded Rust firmware for the sensor bridge. |

## main_ws Packages

### Control Logic
| Package | Language | Role |
|---------|----------|------|
| `joy_controller` | C++ | Maps /joy inputs to robot commands (DRIVE/ARM/STOP modes). |
| `arm_controller` | Rust | Jacobian-based IK for 6-DOF arm control. |

### Hardware Drivers
| Package | Language | Role |
|---------|----------|------|
| `crawler_driver` | C++ | Roboclaw motor controller interface for tracks. |
| `flipper_driver` | C++ | Dynamixel driver for the 4 sub-tracks (flippers). |
| `arm_driver` | Rust | Unified Dynamixel driver for Arm axes and Gripper. |
| `sensor_gateway` | Rust | Serial bridge for STM32/IMU data. |
| `audio_bridge` | Python | Opus-based bidirectional audio transfer via ROS topics. |

### Perception
| Package | Language | Role |
|---------|----------|------|
| `qr_detector_cpp` | C++ | Composable node for QR detection (WeChatQRCode/ZBar). |
| `omron_bridge` | Python | Environment sensing (Temp/Hum/etc.) and RoboCup POI logging. |
| `spark_fast_lio` | C++ | (Submodule) High-performance LiDAR-IMU odometry. |

### Integration & Support
| Package | Language | Role |
|---------|----------|------|
| `bringup` | Mixed | Central launch files, URDF models, and system configuration. |
| `custom_interfaces` | CMake | Shared message/service definitions. |
| `h264_republisher` | C++ | Debug tool to decode FFMPEGPackets back to raw images. |

## operator_ws Packages

| Package | Role |
|---------|------|
| `operator_ws` (ws root) | Contains `operator.launch.py` and Zenoh peer configuration. |
| `joy` (standard) | Standard ROS 2 joy node for controller input. |
| `foxglove_bridge` | WebSocket gateway for Foxglove Studio. |
