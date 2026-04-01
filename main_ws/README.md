# main_ws — ROS 2 Workspace

`main_ws` is the robot-side ROS 2 workspace for Rustacean RoboRescue. It contains the code that runs on the robot, the simulation bringup, and the vendored dependencies needed to build everything reproducibly.

For a package-by-package inventory, see [docs/PACKAGES.md](../docs/PACKAGES.md).

## Workspace Layout

| Group | Packages |
|------|----------|
| `audio_bridge/` | `audio_bridge` |
| `bringup/` | `bringup` |
| `control/` | `joy_controller`, `arm_controller` |
| `custom_interfaces/` | `custom_interfaces` |
| `extras/` | `h264_republisher` |
| `hardware_drivers/` | `arm_driver`, `crawler_driver`, `flipper_driver`, `sensor_gateway` |
| `perception/` | `qr_detector`, `qr_detector_cpp`, `omron_bridge` |
| `external/` | Vendored ROS interfaces and third-party libraries such as `rclrs`, `rosidl_generator_rs`, `spark_fast_lio`, `dynamixel_hardware_interface`, and `kiss_icp` |

## Key Launch Entry Points

| Launch file | Purpose |
|-------------|---------|
| `bringup/launch/system.launch.py` | Real robot bringup with drivers, control, cameras, audio, and optional Nav2 |
| `bringup/launch/simulation.launch.py` | Gazebo simulation with SLAM / Nav2 toggles |
| `bringup/launch/control.launch.py` | Joystick control and arm command path |
| `bringup/launch/camera.launch.py` | Camera drivers, FFmpeg transport, and QR detection |
| `bringup/launch/audio.launch.py` | Opus-based audio bridge |
| `bringup/launch/nav2.launch.py` | Navigation-only bringup |
| `bringup/launch/network.launch.py` | Zenoh configuration for robot-side networking |
| `bringup/launch/display.launch.py` | RViz and visualization helpers |
| `bringup/launch/test_arm.launch.py` / `test_ik.launch.py` | Arm and IK verification helpers |

## Build

```bash
# Enter the Nix shell
nix develop --impure

# Build the full workspace
cd main_ws
just forge

# Build with colcon directly when needed
colcon build --merge-install

# Run checks and tests
just check
just test
```

## Run

```bash
cd main_ws
source install/setup.bash

# Real robot bringup
ros2 launch bringup system.launch.py

# Real robot with Nav2
ros2 launch bringup system.launch.py use_nav2:=true

# Simulation
ros2 launch bringup simulation.launch.py

# Simulation with SLAM and Nav2
ros2 launch bringup simulation.launch.py use_slam:=true use_nav2:=true
```

## Notes On Implementation

- Low-level drivers such as `arm_driver` and `sensor_gateway` are implemented in Rust for safety and recoverability.
- Time-sensitive motion control such as `crawler_driver` and `flipper_driver` remains in C++.
- Perception utilities are split between Python and C++ depending on the amount of image processing and composable-node integration needed.
- The workspace includes vendored ROS interface packages and third-party libraries so the build can run without depending on a moving upstream checkout.

Related docs:

- [docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md)
- [docs/OPERATION.md](../docs/OPERATION.md)
- [docs/PACKAGES.md](../docs/PACKAGES.md)
