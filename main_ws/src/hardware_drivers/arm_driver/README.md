# arm_driver

Unified Dynamixel driver for a 6-DOF robotic arm and an attached gripper sharing the same RS485 bus.

Implemented in **Rust** using `rclrs` and `dynamixel2`.

## Features

- **High-speed Synchronous Control**: Communicates with multiple servos in a dedicated hardware thread at 50Hz.
- **Arm + Gripper Integration**: Manages arm joints (position control) and gripper (current-limited position control) on a single serial port.
- **Safety**:
  - **Torque Management**: Automatically enables/disables torque during E-Stop and shutdown.
  - **Temperature Monitoring**: Periodically checks motor temperatures and triggers a torque-off if critical levels are reached.
  - **E-Stop**: Instant lockout and torque-off upon receiving `/emergency_stop`.
- **Flexible Offsets**: Supports per-joint zero-point offsets to align physical motor positions with the URDF model.

## Topics

### Subscriptions
| Topic | Type | Description |
|-------|------|-------------|
| `/arm_joint_commands` | `sensor_msgs/JointState` | Target positions for arm joints |
| `/gripper_cmd` | `custom_interfaces/GripperCommand` | Target position and current for the gripper |
| `/emergency_stop` | `std_msgs/Bool` | Hard stop trigger |

### Publications
| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Actual arm joint positions |
| `/gripper_status` | `custom_interfaces/GripperStatus` | Gripper position, current, and temperature |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port_name` | string | `/dev/ttyUSB0` | Serial port path |
| `baud_rate` | int | `1000000` | Dynamixel bus speed |
| `arm_ids` | int[] | `[1, 2, 3, 4, 5, 6]` | Dynamixel IDs for arm joints |
| `gripper_id` | int | `10` | Dynamixel ID for the gripper |
| `arm_offsets` | double[] | `[0.0, ...]` | Radian offsets for each joint |

## Build & Run

```bash
# Build
just forge-packages arm_driver

# Run
ros2 run arm_driver arm_driver --ros-args --params-file src/bringup/config/arm_gripper_driver.yaml
```
