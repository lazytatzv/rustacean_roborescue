# hardware_drivers

This directory contains the real-robot hardware interface layer. The packages here are the code that talks to motors, the IMU bridge, and other low-level devices.

## Packages

| Package | Language | Role |
|---------|----------|------|
| `arm_driver` | Rust | Dynamixel position-control driver for the 6-DOF arm and gripper |
| `crawler_driver` | C++ | Roboclaw driver for the differential crawler tracks |
| `flipper_driver` | C++ | Dynamixel driver for the four flippers |
| `sensor_gateway` | Rust | Serial bridge for the STM32 IMU stream |

## Runtime Behavior

- The drivers are designed to fail safely and recover when hardware is temporarily unavailable.
- `crawler_driver` and `flipper_driver` use watchdog-style stops so stale commands do not keep the robot moving.
- `arm_driver` and `sensor_gateway` are structured around reconnect/retry logic instead of hard exits during startup.

## Typical Entry Point

The normal way to start these packages is through [bringup/launch/system.launch.py](../bringup/launch/system.launch.py).

```bash
cd main_ws
source install/setup.bash
ros2 launch bringup system.launch.py
```

If you are changing one driver, keep the corresponding bringup YAML and topic documentation in sync.
