arm_driver (Dynamixel arm + gripper)

Overview

This package provides a unified driver for a 6-DOF Dynamixel-based robotic arm (XM-series style) and an attached gripper sharing the same RS485 bus. It wraps a Rust `ArmDynamixelDriver` implementation and exposes ROS 2 parameters, topics, and a hardware polling thread.

Intended use

- Run as a ROS 2 node named `arm_gripper_driver`.
- Publish joint states on `/joint_states` and gripper status on `/gripper_status`.
- Subscribe to `/arm_joint_commands` (a `sensor_msgs/JointState`) for desired arm joint positions, and `/gripper_cmd` (a `custom_interfaces/GripperCommand`) to control gripper position.

Key files

- `src/main.rs` — Node entrypoint, parameter handling, subscriptions, publishers, and hardware thread spawn.
- `src/driver.rs` — The `ArmDynamixelDriver` that handles serial comms to motors (motor read/write, init, torque_off, emergency_stop, etc.).
- `package.xml`, `Cargo.toml` — package metadata and build config.

ROS 2 parameters (declared by node)

- `port_name` (string) — Serial port path. Default: `/dev/ttyUSB0`.
- `baud_rate` (int) — Baud rate for bus. Default: `1000000`.
- `arm_joints` (string[]) — Names of the arm joints in order. Default: `["arm_joint1"]`.
- `arm_ids` (int[]) — Dynamixel IDs for arm motors in the same order as `arm_joints`. Default: `[21]`.
- `gripper_id` (int) — Dynamixel ID of the gripper. Default: `10`.
- `gripper_max_current` (int) — Gripper current limit (mA). Default: `500`.
- `profile_velocity` (int) — Motor profile velocity used on initialization. Default: `100`.

Topics

- Publishes:
  - `/joint_states` (`sensor_msgs/JointState`): current arm joint positions. Header stamp is set in code.
  - `/gripper_status` (`custom_interfaces/GripperStatus`): gripper telemetry (position/current/temperature).
- Subscribes:
  - `/arm_joint_commands` (`sensor_msgs/JointState`): desired joint positions. Only positions for names matching `arm_joints` are used.
  - `/gripper_cmd` (`custom_interfaces/GripperCommand`): desired gripper position.

Concurrency & architecture

- A hardware thread (spawned via `std::thread::spawn`) runs the polling/control loop at `HW_LOOP_HZ` (50 Hz by default).
- Inter-thread communication uses an `std::sync::mpsc::channel` for high-level commands (`HwCommand` struct).
- The ROS node threads handle subscriptions and publish; the hardware thread interacts with the physical motors through `ArmDynamixelDriver`.
- Two `Arc<AtomicBool>` flags (`shutdown_flag`, `estop_flag`) provide shutdown and emergency-stop coordination.

Safety notes

- When `estop_flag` is set, the hardware thread calls `driver.emergency_stop()` and stops interacting with motors.
- On shutdown the driver calls `torque_off_all()` to release motor torque.
- Ensure wiring and current limits are configured properly for your gripper motor to avoid damage.

Building and running

From workspace root (colcon/ament):

1. Build the workspace (assuming ament/cargo is configured in your environment):

```bash
colcon build --packages-select arm_driver
```

2. Source the workspace and run the node:

```bash
source install/setup.bash
ros2 run arm_driver arm_driver
```

3. Alternatively, launch using your bringup config that sets parameters (see `main_ws/src/bringup/config/arm_gripper_driver.yaml`).

Troubleshooting

- Serial permissions: ensure the user has permission to open `/dev/ttyUSB*`. Use udev rules or run under appropriate user.
- Parameter mismatch: ensure `arm_ids` length matches `arm_joints` length.
- If motors don't respond, check bus termination, power, and that `baud_rate` matches motor settings.

Contact

For implementation questions, see the repository or contact the maintainer in `package.xml`.
