# arm_driver

Unified Dynamixel driver for a 6-DOF robotic arm and an attached gripper.

Implemented in **Rust** using `rclrs` and `dynamixel2`. Designed for low-latency, high-reliability serial communication.

## Architecture

The driver uses a **Dual-Thread Design** to isolate ROS 2 overhead from hardware-critical timing:

1. **ROS Node Thread**: Handles asynchronous ROS 2 subscriptions (`/arm_joint_commands`, `/gripper_cmd`, `/emergency_stop`) and buffers the latest requests.
2. **Hardware Polling Thread**: Runs a deterministic loop at **50Hz**. 
   - Uses `std::sync::mpsc` to receive commands from the ROS thread.
   - Synchronously reads/writes all motor positions in a single bus cycle using SyncRead/SyncWrite (where supported).
   - Monitors motor health (Temperature, Error status) and publishes feedback.

## Safety & Fault Handling

- **Emergency Stop (E-Stop)**: 
  - Subscribes to `/emergency_stop` with `TransientLocal` durability (receives state even if node restarts).
  - On E-Stop, it immediately calls `torque_off_all()`.
  - Torque can only be re-enabled by switching the E-Stop state to `false`.
- **Thermal Protection**: 
  - Periodically polls internal temperature of all servos.
  - Logs warnings at $55^\circ C$.
  - **Auto-Lockout**: If any motor hits $70^\circ C$ (configurable in Dynamixel firmware), the driver forces torque-off and triggers a software E-Stop to protect the hardware.
- **Fail-safe Shutdown**: On SIGINT or process exit, the driver ensures all motors are stopped and torque is released to prevent the arm from being locked in a dangerous position.

## Topics

### Subscriptions
| Topic | Type | Description |
|-------|------|-------------|
| `/arm_joint_commands` | `sensor_msgs/JointState` | Target positions for the 6 arm joints. |
| `/gripper_cmd` | `custom_interfaces/GripperCommand` | Target position for the gripper servo. |
| `/emergency_stop` | `std_msgs/Bool` | Global hard-stop trigger. |

### Publications
| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Actual position feedback for all arm joints. |
| `/gripper_status` | `custom_interfaces/GripperStatus` | Gripper position, motor current, and temperature. |

## Configuration

The driver is configured via `arm_gripper_driver.yaml`.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port_name` | string | `/dev/ttyUSB0` | Device path (recommended to use udev alias `/dev/dynamixel_arm`). |
| `baud_rate` | int | `1000000` | 1Mbps is standard for XM-series servos. |
| `arm_ids` | int[] | `[1, 2, 3, 4, 5, 6]` | Individual Dynamixel IDs for the arm joints. |
| `gripper_id` | int | `10` | Dynamixel ID for the gripper servo. |
| `gripper_max_current` | int | `500` | Current limit for the gripper to prevent crushing objects. |
| `arm_offsets` | double[] | `[0.0, ...]` | Radian offsets to align physical zero with URDF zero. |

## Troubleshooting

- **"init failed"**: Check power supply (12V) and U2D2 connection. Ensure `baud_rate` matches the servo configuration (usually 1Mbps or 57600bps).
- **"ID not responding"**: Use the `dxl_scan.py` tool in `tools/maintenance` to verify all IDs are visible on the bus.
- **Permissions**: Ensure your user is in the `dialout` group: `sudo usermod -aG dialout $USER`.
